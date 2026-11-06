"""Simulated hardware and clocks, so the whole program runs on a laptop.

The model is deliberately simple: soil dries slowly, pumped water wets it and
leaves the tank, and the tank probe reads the level of the bottom part of the
tank, like the real probe standing upright in the bucket.
"""
from __future__ import annotations

import random
import time

from .config import Config
from .node import NodeSnapshot

DRY_MV, WET_MV = 2600, 1200       # simulated soil probe in air / in water
EMPTY_MV, FULL_MV = 2700, 1450    # simulated tank probe: tank empty / water at the MAX line
POT_ML = 300.0                    # ml that take a pot from bone dry to soaked
TANK_ZONE_ML = 2000.0             # water inside the probe's sensing zone (MAX line)


class RealClock:
    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, s: float) -> None:
        time.sleep(s)


class FastClock:
    """Virtual time that runs `speed` times faster than real time."""

    def __init__(self, speed: float = 60.0):
        self.speed = speed
        self.t = 0.0

    def monotonic(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.t += s
        if self.speed > 0:
            time.sleep(s / self.speed)


class SimNode:
    def __init__(self, hw: "SimHardware"):
        self.hw = hw
        self.error = ""

    def snapshot(self):
        return self.hw.node_snapshot()

    def close(self) -> None:
        pass


class SimHardware:
    def __init__(self, cfg: Config, clock, start_pct: list[float] | None = None,
                 tank_ml: float = 1800.0, seed: int = 1, dry_pct_per_h: float = 2.0):
        self.cfg, self.clock = cfg, clock
        self.rng = random.Random(seed)
        n = len(cfg.pots)
        self.moist = [p / 100 for p in (start_pct or [30.0] * n)]
        self.tank_ml = tank_ml
        self.dry_rate = dry_pct_per_h / 100 / 3600
        self.powered = [False] * n
        self.last_t = clock.monotonic()
        self.max_simultaneous = 0          # tests: never more than 1 pump powered
        self.history: list[tuple[float, int, bool]] = []
        self.pumped_ml = [0.0] * n
        self.steps_done = [0] * n
        self.node_silent = False           # tests: the Pico stops talking
        self.unplugged: set[str] = set()   # tests: sensor names whose probe lost power
        self.fail_on_steps = False         # tests: the motor driver raises mid-run
        self._seq = 0
        self._last_snap: NodeSnapshot | None = None
        self.node = SimNode(self)

    def _advance(self) -> None:
        now = self.clock.monotonic()
        dt = max(0.0, now - self.last_t)
        self.last_t = now
        self.moist = [max(0.0, m - self.dry_rate * dt) for m in self.moist]

    # sensors -------------------------------------------------------------
    def _mv(self) -> dict[str, int]:
        mv = {}
        for p, m in zip(self.cfg.pots, self.moist):
            mv[p.sensor] = int(DRY_MV + (WET_MV - DRY_MV) * min(1.0, m) + self.rng.uniform(-8, 8))
        level = max(0.0, min(1.0, self.tank_ml / TANK_ZONE_ML))
        mv[self.cfg.tank_sensor] = int(EMPTY_MV + (FULL_MV - EMPTY_MV) * level + self.rng.uniform(-8, 8))
        for name in self.unplugged:
            mv[name] = self.rng.randint(0, 40)
        return mv

    def node_snapshot(self):
        self._advance()
        if self.node_silent:
            return self._last_snap
        self._seq += 1
        env = {"t_c": round(21.5 + self.rng.uniform(-0.3, 0.3), 2),
               "rh": round(47 + self.rng.uniform(-2, 2), 1),
               "hpa": round(1009 + self.rng.uniform(-0.5, 0.5), 1)}
        self._last_snap = NodeSnapshot("sim-pico", self._seq, self._mv(), self.clock.monotonic(),
                                       env=env)
        return self._last_snap

    def node_error(self) -> str:
        return "simulated: node silent" if self.node_silent else ""

    # actuators -----------------------------------------------------------
    def pump_on(self, pot: int) -> None:
        self._advance()
        self.powered[pot] = True
        self.history.append((self.clock.monotonic(), pot, True))
        self.max_simultaneous = max(self.max_simultaneous, sum(self.powered))

    def pump_steps(self, pot: int, n: int, backwards: bool = False) -> int:
        if not self.powered[pot]:
            raise RuntimeError("stepper not powered (relay off)")
        if self.fail_on_steps:
            raise OSError("simulated GPIO failure")
        self.clock.sleep(n * self.cfg.step_delay_s)
        self._advance()
        self.steps_done[pot] += n
        if backwards:
            return n
        p = self.cfg.pots[pot]
        ml = min(self.tank_ml, n / self.cfg.steps_per_rev * p.ml_per_rev)
        self.tank_ml -= ml
        self.moist[pot] = min(1.0, self.moist[pot] + ml / POT_ML)
        self.pumped_ml[pot] += ml
        return n

    def pump_off(self, pot: int) -> None:
        self._advance()
        if self.powered[pot]:
            self.history.append((self.clock.monotonic(), pot, False))
        self.powered[pot] = False

    def all_off(self) -> None:
        for i in range(len(self.powered)):
            self.pump_off(i)

    def pumps_running(self) -> list[int]:
        return [i for i, v in enumerate(self.powered) if v]

    def close(self) -> None:
        self.all_off()
