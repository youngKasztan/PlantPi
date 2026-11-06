"""Load and validate the TOML configuration, and the probe calibration."""
from __future__ import annotations

import json
import re
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from . import MAX_DOSE_ML, MIN_COOLDOWN_MIN

# BCM GPIOs we never use: 0,1 = HAT EEPROM, 2,3 = I2C, 14,15 = UART.
# (SPI is free now - no MCP3008 - but kept free for future add-ons: 7..11.)
RESERVED_GPIOS = {0, 1, 2, 3, 7, 8, 9, 10, 11, 14, 15}
VALID_GPIOS = set(range(2, 28))
SENSOR_NAME = re.compile(r"^[a-z][a-z0-9_]{0,15}$")


class ConfigError(ValueError):
    pass


@dataclass
class PotConfig:
    name: str
    sensor: str
    relay_gpio: int
    motor_gpios: list[int]
    reverse: bool = False
    threshold_pct: float = 35.0
    dose_ml: float = 30.0
    ml_per_rev: float = 0.30
    cooldown_min: float = 60.0
    max_doses_per_day: int = 4


@dataclass
class Config:
    port: str = "auto"
    baud: int = 115200
    dtr: bool = True
    stale_s: float = 10.0
    tank_sensor: str = "tank"
    tank_min_pct: float = 15.0
    tank_stop_pct: float = 10.0
    sample_interval_s: float = 60.0
    startup_holdoff_min: float = 10.0
    time_check_interval_s: float = 600.0
    steps_per_rev: int = 2048
    step_delay_ms: float = 3.0
    relay_active_high: bool = True
    pots: list[PotConfig] = field(default_factory=list)

    @property
    def step_delay_s(self) -> float:
        return self.step_delay_ms / 1000.0

    def validate(self) -> "Config":
        if not self.pots:
            raise ConfigError("at least one [[pot]] is required")
        if not 1.5 <= self.step_delay_ms <= 20:
            raise ConfigError("step_delay_ms must be 1.5 .. 20 (28BYJ-48 skips steps when faster)")
        if self.steps_per_rev not in (2048, 4096):
            raise ConfigError("steps_per_rev must be 2048 (full step) or 4096 (half step)")
        if not 2 <= self.stale_s <= 120:
            raise ConfigError("stale_s must be 2 .. 120")
        if not 0 <= self.tank_stop_pct <= self.tank_min_pct < 100:
            raise ConfigError("need 0 <= tank stop_pct <= min_pct < 100")
        if not SENSOR_NAME.match(self.tank_sensor):
            raise ConfigError(f"bad tank sensor name {self.tank_sensor!r}")
        sensors, gpios = {self.tank_sensor}, set()
        relays = {p.relay_gpio for p in self.pots}     # pots may share one relay

        def take(g, who):
            if not isinstance(g, int) or g not in VALID_GPIOS or g in RESERVED_GPIOS:
                raise ConfigError(f"{who}: GPIO{g} is not allowed (reserved or not a GPIO)")
            if g in gpios:
                raise ConfigError(f"{who}: GPIO{g} is used twice")
            gpios.add(g)

        for g in relays:
            take(g, "relay")
        for p in self.pots:
            if not SENSOR_NAME.match(p.sensor):
                raise ConfigError(f"{p.name}: bad sensor name {p.sensor!r}")
            if p.sensor in sensors:
                raise ConfigError(f"{p.name}: sensor {p.sensor!r} used twice (or same as the tank)")
            sensors.add(p.sensor)
            if len(p.motor_gpios) != 4:
                raise ConfigError(f"{p.name}: motor_gpios needs exactly 4 GPIOs (ULN2003 IN1..IN4)")
            for g in p.motor_gpios:
                if g in relays:
                    raise ConfigError(f"{p.name}: GPIO{g} is a relay pin and a motor pin")
                take(g, p.name)
            if not 0 < p.threshold_pct < 100:
                raise ConfigError(f"{p.name}: threshold_pct must be between 0 and 100")
            if not 0.01 <= p.ml_per_rev <= 20:
                raise ConfigError(f"{p.name}: ml_per_rev must be 0.01 .. 20 (measure it: plantpi test pump)")
            # Clamp to the hard safety limits instead of trusting the file.
            p.dose_ml = max(1.0, min(float(p.dose_ml), MAX_DOSE_ML))
            p.cooldown_min = max(float(p.cooldown_min), MIN_COOLDOWN_MIN)
            p.max_doses_per_day = max(1, int(p.max_doses_per_day))
        return self


def load_config(path: str | Path) -> Config:
    with open(path, "rb") as f:
        raw = tomllib.load(f)
    node, tank, ctl, mot = (raw.get(k, {}) for k in ("node", "tank", "control", "motor"))
    try:
        pots = [PotConfig(**p) for p in raw.get("pot", [])]
    except TypeError as e:
        raise ConfigError(f"[[pot]]: {e}") from None
    cfg = Config(
        port=node.get("port", "auto"),
        baud=int(node.get("baud", 115200)),
        dtr=bool(node.get("dtr", True)),
        stale_s=float(node.get("stale_s", 10.0)),
        tank_sensor=tank.get("sensor", "tank"),
        tank_min_pct=float(tank.get("min_pct", 15.0)),
        tank_stop_pct=float(tank.get("stop_pct", 10.0)),
        sample_interval_s=float(ctl.get("sample_interval_s", 60.0)),
        startup_holdoff_min=float(ctl.get("startup_holdoff_min", 10.0)),
        time_check_interval_s=float(ctl.get("time_check_interval_s", 600.0)),
        steps_per_rev=int(mot.get("steps_per_rev", 2048)),
        step_delay_ms=float(mot.get("step_delay_ms", 3.0)),
        relay_active_high=bool(mot.get("relay_active_high", True)),
        pots=pots,
    )
    return cfg.validate()


# ---------------------------------------------------------------- calibration
@dataclass
class Calibration:
    """Two reference readings in millivolts.

    soil probe: dry = in air, wet = in a glass of water
    tank probe: dry = tank empty (probe in air), wet = water up to the MAX line
    """
    dry: int
    wet: int

    def percent(self, mv: int) -> float:
        """Map millivolts to 0..100 % (not clamped). Works whichever of dry/wet is larger."""
        return (mv - self.dry) / (self.wet - self.dry) * 100.0


def load_calibration(path: str | Path) -> dict[str, Calibration]:
    p = Path(path)
    if not p.exists():
        return {}
    data = json.loads(p.read_text())
    return {str(k): Calibration(int(v["dry"]), int(v["wet"])) for k, v in data.items()}


def save_calibration(path: str | Path, cal: dict[str, Calibration]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    data = {k: {"dry": c.dry, "wet": c.wet} for k, c in sorted(cal.items())}
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(p)
