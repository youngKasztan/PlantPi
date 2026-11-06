"""PlantPi command line.

  plantpi run                    start the controller (systemd does this)
  plantpi status                 last samples and events from the log
  plantpi water 1 30             ask the running service to pump ~30 ml into pot 1
  plantpi calibrate 1 | tank     record dry/wet values for a probe
  plantpi test node              live readings from the Pico sensor node
  plantpi test relay 1           relay click test
  plantpi test motor 1           motor turns a little back and forth
  plantpi test pump 1            measure the pump's ml per turn
  plantpi test touch 1           demo: motor turns, stops while you hold the probe
  plantpi pins                   print the wiring table

Add --sim to any command to run it on a laptop without hardware.
"""
from __future__ import annotations

import argparse
import logging
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

from . import MAX_DOSE_ML, __version__
from .config import Calibration, load_calibration, load_config, save_calibration

PI_CONFIG = Path("/etc/plantpi/config.toml")
PI_STATE = Path("/var/lib/plantpi")
HERE = Path(__file__).resolve().parent.parent

PIN_TABLE = """\
 #    from                                -> to                                  colour   jumper
 --- Pico sensor node on its breadboard (3.3 V only, NO 5 V on this board) ---
 W01  Pico pin 36 3V3 (hole j5)           -> bottom + rail                       red      M-M
 W02  Pico pin 38 GND (hole j3)           -> bottom - rail                       black    M-M
 W03  bottom + rail                       -> top + rail (bridge)                 red      M-M
 W04  bottom - rail                       -> top - rail (bridge)                 black    M-M
 W05  bottom + rail                       -> pot probe 1 VCC                     red      M-M into probe cable
 W06  bottom - rail                       -> pot probe 1 GND                     black    M-M into probe cable
 W07  pot probe 1 AOUT                    -> Pico pin 31 GP26/ADC0 (hole j10)    yellow   M-M into probe cable
 W08  bottom + rail                       -> tank probe VCC (+)                  red      M-M into probe cable
 W09  bottom - rail                       -> tank probe GND (-)                  black    M-M into probe cable
 W10  tank probe AOUT (A)                 -> Pico pin 32 GP27/ADC1 (hole j9)     blue     M-M into probe cable
 W11  top + rail                          -> BME280 VIN                          red      M-M
 W12  top - rail                          -> BME280 GND                          black    M-M
 W13  BME280 SCL                          -> Pico pin 7 GP5 (hole a7)            white    M-M
 W14  BME280 SDA                          -> Pico pin 6 GP4 (hole a6)            green    M-M
 --- link ---
 W15  Pico micro-USB socket               -> any Pi USB port                     grey     USB data cable
 --- Pi 4 side: jumpers straight onto the header ---
 W16  Pi pin 4 (5V)                       -> relay VCC                           red      F-F
 W17  Pi pin 6 (GND)                      -> relay GND                           black    F-F
 W18  Pi pin 12 (GPIO18)                  -> relay IN                            white    F-F
 W19  Pi pin 2 (5V)                       -> relay COM (screw)                   red      F-M
 W20  relay NO (screw)                    -> ULN2003 #1 +                        orange   M-F
 W21  Pi pin 20 (GND)                     -> ULN2003 #1 -                        black    F-F
 W22  Pi pin 11 (GPIO17)                  -> ULN2003 #1 IN1                      blue     F-F
 W23  Pi pin 13 (GPIO27)                  -> ULN2003 #1 IN2                      green    F-F
 W24  Pi pin 15 (GPIO22)                  -> ULN2003 #1 IN3                      yellow   F-F
 W25  Pi pin 18 (GPIO24)                  -> ULN2003 #1 IN4                      purple   F-F
 W26  28BYJ-48 #1 plug                    -> ULN2003 #1 white socket             grey     motor cable
 --- later: Pot 2 (then uncomment Pot 2 in config.toml) ---
 W27  bottom + rail                       -> pot probe 2 VCC                     red      M-M into probe cable
 W28  bottom - rail                       -> pot probe 2 GND                     black    M-M into probe cable
 W29  pot probe 2 AOUT                    -> Pico pin 34 GP28/ADC2 (hole j7)     yellow   M-M into probe cable
 W30  relay NO (same screw as W20)        -> ULN2003 #2 +                        orange   M-F
 W31  Pi pin 34 (GND)                     -> ULN2003 #2 -                        black    F-F
 W32  Pi pin 32 (GPIO12)                  -> ULN2003 #2 IN1                      blue     F-F
 W33  Pi pin 36 (GPIO16)                  -> ULN2003 #2 IN2                      green    F-F
 W34  Pi pin 38 (GPIO20)                  -> ULN2003 #2 IN3                      yellow   F-F
 W35  Pi pin 40 (GPIO21)                  -> ULN2003 #2 IN4                      purple   F-F
 W36  28BYJ-48 #2 plug                    -> ULN2003 #2 white socket             grey     motor cable
 One relay switches the 5 V of both motor drivers; the second relay module is a spare.
"""


def _paths(args):
    if args.sim:
        cfg = Path(args.config or HERE / "config.example.toml")
        state = Path(args.state or HERE / "sim-data")
    else:
        cfg = Path(args.config or PI_CONFIG)
        state = Path(args.state or PI_STATE)
    return cfg, state / "plantpi.db", state / "calibration.json"


def _sim_calibration(cfg):
    from .sim import DRY_MV, EMPTY_MV, FULL_MV, WET_MV
    cal = {p.sensor: Calibration(DRY_MV, WET_MV) for p in cfg.pots}
    cal[cfg.tank_sensor] = Calibration(EMPTY_MV, FULL_MV)
    return cal


def _hardware(cfg, args, clock):
    if args.sim:
        from .sim import SimHardware
        return SimHardware(cfg, clock, start_pct=[25, 60][: len(cfg.pots)])
    from .hardware import RealHardware
    return RealHardware(cfg)


def _service_running() -> bool:
    try:
        r = subprocess.run(["systemctl", "is-active", "--quiet", "plantpi"], timeout=5)
        return r.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _need_service_stopped(args) -> bool:
    if not args.sim and _service_running():
        print("Stop the service first:  sudo systemctl stop plantpi")
        return True
    return False


def _pot_index(cfg, n: int) -> int | None:
    if not 1 <= n <= len(cfg.pots):
        print(f"pot {n} is not in the config (it has {len(cfg.pots)} pot(s)).")
        return None
    return n - 1


def cmd_run(args) -> int:
    from .controller import Controller
    from .notify import time_synced
    from .sim import FastClock, RealClock
    from .store import Store

    cfg_path, db, cal_path = _paths(args)
    cfg = load_config(cfg_path)
    clock = FastClock(args.speed) if args.sim else RealClock()
    cal = load_calibration(cal_path)
    if args.sim and not cal:
        cal = _sim_calibration(cfg)
        cfg.startup_holdoff_min = 0
    hw = _hardware(cfg, args, clock)
    store = Store(db)
    ctl = Controller(cfg, hw, store, clock, cal, time_synced=(lambda: True) if args.sim else time_synced)

    def _stop(signum, _frame):
        ctl.stop = True
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    try:
        ctl.run(max_cycles=args.cycles)
    finally:
        hw.close()
        store.close()
    return 0


def cmd_status(args) -> int:
    from .store import Store
    _, db, _ = _paths(args)
    if not db.exists():
        print(f"no log yet at {db}")
        return 1
    st = Store(db)
    print("Last samples:")
    for s, soil in st.last_samples(args.n):
        if not s["node_ok"]:
            print(f"  {s['ts_utc']}{'' if s['synced'] else '*'}  sensor node SILENT")
            continue
        tank = f"tank={s['tank_mv']}mV" + (f"/{s['tank_pct']:.0f}%" if s["tank_pct"] is not None else "")
        pots = "  ".join(f"pot{r['pot'] + 1}={r['mv']}mV" + (f"/{r['pct']:.0f}%" if r["pct"] is not None else "")
                         + ("" if r["valid"] else "!") for r in soil)
        air = (f"  air {s['air_c']:.1f}C" if s["air_c"] is not None else "") + \
              (f" {s['air_rh']:.0f}%RH" if s["air_rh"] is not None else "") + \
              (f" {s['air_hpa']:.0f}hPa" if s["air_hpa"] is not None else "")
        print(f"  {s['ts_utc']}{'' if s['synced'] else '*'}  {tank}  {pots}{air}")
    print("Last events:")
    for e in st.last_events(args.n):
        pot = f"pot{e['pot'] + 1}" if e["pot"] is not None else "    "
        amt = (f"{e['ml']:.0f}ml/{e['seconds']:.0f}s" if e["ml"] is not None else "")
        print(f"  {e['ts_utc']}{'' if e['synced'] else '*'}  {e['kind']:8s} {pot} {amt:>11s}  {e['reason']}")
    print("(* = clock not yet NTP-synchronised; time may be wrong.  ! = invalid reading)")
    st.close()
    return 0


def cmd_water(args) -> int:
    from .store import Store
    cfg_path, db, _ = _paths(args)
    cfg = load_config(cfg_path)
    i = _pot_index(cfg, args.pot)
    if i is None:
        return 2
    ml = max(1.0, min(args.ml, MAX_DOSE_ML))
    p = cfg.pots[i]
    minutes = ml / p.ml_per_rev * cfg.steps_per_rev * cfg.step_delay_s / 60
    st = Store(db)
    rid = st.add_request(i, ml)
    st.close()
    print(f"request #{rid}: pump ~{ml:.0f} ml into {p.name} (takes ~{minutes:.1f} min). "
          "The running service picks it up within ~1 s (see: plantpi status).")
    return 0


def _median_mv(hw, sensor: str, n: int = 6, timeout_s: float = 30.0) -> list[int]:
    """Collect n readings of one sensor from n different node messages."""
    vals, last_seq = [], None
    t_end = time.monotonic() + timeout_s
    while len(vals) < n and time.monotonic() < t_end:
        snap = hw.node_snapshot()
        if snap is not None and snap.seq != last_seq and sensor in snap.mv:
            last_seq = snap.seq
            vals.append(snap.mv[sensor])
            print(f"   {len(vals)}/{n}: {snap.mv[sensor]} mV", end="\r", flush=True)
        time.sleep(0.2)
    print()
    return vals


def cmd_calibrate(args) -> int:
    from .sim import RealClock
    cfg_path, _, cal_path = _paths(args)
    cfg = load_config(cfg_path)
    if _need_service_stopped(args):
        return 1
    if args.which == "tank":
        sensor, label = cfg.tank_sensor, "the TANK probe"
        steps = ("1/2  EMPTY tank: the probe stands in the empty, dry tank (or hold it in the air).",
                 "2/2  Fill the tank up to the MAX line drawn on the probe (below the electronics!).")
    else:
        i = _pot_index(cfg, int(args.which))
        if i is None:
            return 2
        sensor, label = cfg.pots[i].sensor, cfg.pots[i].name
        steps = ("1/2  Hold the probe in the AIR, dry, not touching anything.",
                 "2/2  Put the probe in a glass of WATER up to the line on the probe (never deeper).")
    hw = _hardware(cfg, args, RealClock())
    try:
        res = []
        print(f"Calibrating {label} (Pico sensor '{sensor}').")
        for text in steps:
            input(f"\n{text}\nPress Enter when ready... ")
            vals = _median_mv(hw, sensor)
            if not vals:
                print(f"No readings of '{sensor}' from the Pico - run:  plantpi test node")
                return 1
            v = int(statistics.median(vals))
            print(f"  median: {v} mV   (spread {min(vals)}..{max(vals)})")
            res.append(v)
    finally:
        hw.close()
    dry, wet = res
    if abs(dry - wet) < 300:
        print(f"\nCHECK: the two values ({dry} / {wet} mV) are only {abs(dry - wet)} mV apart. "
              "Right probe on the right Pico pin? Nothing saved.")
        return 1
    cal = load_calibration(cal_path)
    cal[sensor] = Calibration(dry, wet)
    save_calibration(cal_path, cal)
    print(f"\nSaved {label}: dry/empty={dry} mV, wet/full={wet} mV -> {cal_path}")
    return 0


def cmd_test(args) -> int:
    from . import commission as c
    from .sim import RealClock
    cfg_path, _, _ = _paths(args)
    cfg = load_config(cfg_path)
    if _need_service_stopped(args):
        return 1
    i = 0
    if args.what != "node":
        if args.n is None:
            print(f"give the pot number: plantpi test {args.what} 1")
            return 2
        i = _pot_index(cfg, args.n)
        if i is None:
            return 2
    hw = _hardware(cfg, args, RealClock())
    try:
        if args.what == "node":
            ok = c.test_node(hw, cfg)
        elif args.what == "relay":
            ok = c.test_relay(hw, i, cfg)
        elif args.what == "motor":
            ok = c.test_motor(hw, i, cfg)
        elif args.what == "touch":
            ok = c.test_touch(hw, i, cfg, args.seconds, args.delta, args.sensor)
        else:
            ok = c.test_pump(hw, i, cfg, args.turns)
    finally:
        hw.close()
    return 0 if ok else 1


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="plantpi", description="PlantPi - plant watering controller")
    p.add_argument("--version", action="version", version=__version__)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--sim", action="store_true", help="simulated hardware (laptop)")
    common.add_argument("--config", help="config file (default /etc/plantpi/config.toml)")
    common.add_argument("--state", help="data folder (default /var/lib/plantpi)")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", parents=[common], help="run the controller")
    r.add_argument("--speed", type=float, default=60.0, help="sim only: time speed-up (default 60x)")
    r.add_argument("--cycles", type=int, help="stop after N sample cycles")
    r.set_defaults(fn=cmd_run)

    s = sub.add_parser("status", parents=[common], help="show recent log")
    s.add_argument("-n", type=int, default=8)
    s.set_defaults(fn=cmd_status)

    w = sub.add_parser("water", parents=[common], help="request a manual watering")
    w.add_argument("pot", type=int)
    w.add_argument("ml", type=float, help="millilitres")
    w.set_defaults(fn=cmd_water)

    c = sub.add_parser("calibrate", parents=[common], help="record dry/wet values for a probe")
    c.add_argument("which", choices=["1", "2", "tank"])
    c.set_defaults(fn=cmd_calibrate)

    t = sub.add_parser("test", parents=[common], help="hardware check after a wiring step")
    t.add_argument("what", choices=["node", "relay", "motor", "pump", "touch"])
    t.add_argument("n", type=int, nargs="?", help="pot number: 1 or 2")
    t.add_argument("--turns", type=int, default=20, help="pump test: number of turns")
    t.add_argument("--seconds", type=float, default=300, help="touch demo: how long (0 = until Ctrl+C)")
    t.add_argument("--delta", type=int, default=200, help="touch demo: mV drop that counts as 'touched'")
    t.add_argument("--sensor", help="touch demo: follow another probe, e.g. tank")
    t.set_defaults(fn=cmd_test)

    pn = sub.add_parser("pins", help="print the wiring table")
    pn.set_defaults(fn=lambda a: print(PIN_TABLE) or 0)

    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(message)s", stream=sys.stdout)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
