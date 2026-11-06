"""Step-by-step hardware checks, one per wiring step.

    plantpi test node        live readings from the Pico sensor node (all probes)
    plantpi test relay 1     relay click test (motor plug may stay in)
    plantpi test motor 1     a few slow turns, no water needed
    plantpi test pump 1      pump N turns into a measuring cup -> ml_per_rev for the config
    plantpi test touch 1     demo: motor turns, stops while you hold the probe

Every check prints what you should see and what to do if you don't.
"""
from __future__ import annotations

import time

OK, BAD = "PASS", "CHECK"


def _ask(question: str) -> bool:
    try:
        return input(f"{question} [y/n] ").strip().lower().startswith("y")
    except EOFError:
        return False


def _ask_float(question: str) -> float | None:
    try:
        return float(input(f"{question} ").strip().replace(",", "."))
    except (EOFError, ValueError):
        return None


def test_node(hw, cfg, seconds: int = 12, clock=time) -> bool:
    names = [p.sensor for p in cfg.pots] + [cfg.tank_sensor]
    print(f"Pico sensor node - watching for {seconds} s. Touch/wet a probe and watch its number change.\n")
    t_end = clock.monotonic() + seconds
    seen = 0
    last_seq = None
    values: dict[str, list[int]] = {n: [] for n in names}
    envs: list[dict] = []
    while clock.monotonic() < t_end:
        snap = hw.node_snapshot()
        if snap is not None and snap.seq != last_seq:
            last_seq = snap.seq
            seen += 1
            for n in names:
                if n in snap.mv:
                    values[n].append(snap.mv[n])
            row = "  ".join(f"{k}={v:4d}mV" for k, v in sorted(snap.mv.items()))
            env = "  ".join(f"{k}={v}" for k, v in snap.env.items())
            envs.append(snap.env)
            print(f"  #{snap.seq:<5d} {row}  {env}")
        clock.sleep(0.25)
    print()
    if seen == 0:
        err = hw.node_error() if hasattr(hw, "node_error") else ""
        print(f"{BAD}: no messages from the Pico. {err}\n"
              "  * Is the Pico plugged into a Pi USB port with a DATA cable (charge-only cables don't work)?\n"
              "  * ls /dev/ttyACM*  should show it. Is MicroPython on it, with main.py AND bmx280.py?\n"
              "  * On a laptop, Thonny's Shell should print a line starting with {\"v\":1 every 2 s.")
        return False
    ok = True
    for n in names:
        v = values[n]
        if not v:
            print(f"{BAD}: {n!r} is missing in the messages - add it to SENSORS[] in the firmware.")
            ok = False
        elif max(v) <= 80:
            print(f"{BAD}: {n} reads ~0 mV - its VCC or GND wire is off, or AOUT is on the wrong Pico pin (GP26 = pin 31, GP27 = pin 32).")
            ok = False
        elif min(v) >= 3250:
            print(f"{BAD}: {n} reads ~3.3 V - AOUT is touching 3V3, or the probe is on the 5 V rail!")
            ok = False
        else:
            print(f"{OK}: {n:5s} {min(v)}..{max(v)} mV")
    if any("t_c" in e for e in envs):
        kind = "BME280 (temp + humidity + pressure)" if any("rh" in e for e in envs) \
            else "BMP280 (temp + pressure, no humidity)"
        print(f"{OK}: air sensor found: {kind}")
    else:
        print("INFO: no air sensor (BME280/BMP280) - fine, it is optional. If it IS wired: check"
              " SDA -> GP4 (pin 6), SCL -> GP5 (pin 7), VIN -> 3V3, and the soldering of its header pins.")
    if ok:
        print(f"\n{OK}: {seen} messages. In air a probe reads high (~2.4-2.9 V), in water low (~1.0-1.6 V).")
    return ok


def test_relay(hw, pot: int, cfg) -> bool:
    p = cfg.pots[pot]
    print(f"Relay {pot + 1} - switching ON for 2 seconds (motor coils stay off).")
    time.sleep(1.0)
    try:
        hw.pump_on(pot)
        print("   ON  ... (listen for a click; the relay LED and the ULN2003 board's LEDs/power light up)")
        time.sleep(2.0)
    finally:
        hw.pump_off(pot)
    print("   OFF ... (second click)")
    if _ask("Did you hear two clicks?"):
        if _ask("Was the relay LED ON *only* during those 2 seconds (not before / after)?"):
            print(f"{OK}: relay {pot + 1} works from a 3.3 V GPIO.")
            return True
        print(f"{BAD}: the relay is inverted (active-low). Set  relay_active_high = false  in [motor]"
              " of /etc/plantpi/config.toml and run this test again.")
        return False
    print(f"{BAD}: check relay IN -> GPIO{p.relay_gpio} (pin 12), VCC -> pin 4 (5 V), GND -> pin 6."
          " If the LED lights but there is no click, the 3.3 V signal is too weak for this module.")
    return False


def test_motor(hw, pot: int, cfg, turns: float = 0.5) -> bool:
    print(f"Motor {pot + 1} - {turns} turn forward, then {turns} turn back. No water needed.")
    steps = int(turns * cfg.steps_per_rev)
    try:
        hw.pump_on(pot)
        hw.pump_steps(pot, steps)
        hw.pump_steps(pot, steps, backwards=True)
    finally:
        hw.pump_off(pot)
    if _ask("Did the shaft turn one way and then back, smoothly (the 4 LEDs on the ULN2003 blink)?"):
        print(f"{OK}: motor {pot + 1} works.")
        return True
    print(f"{BAD}: if it only buzzes/shakes: IN1..IN4 are in the wrong order - check the four wires"
          f" go to GPIO{cfg.pots[pot].motor_gpios} in that order. If nothing at all: is the relay"
          " clicking, is ULN2003 '+' wired to relay NO and relay COM to Pi pin 2 (5 V), is the jumper on the"
          " ULN2003 board in place?")
    return False


def test_pump(hw, pot: int, cfg, turns: int = 20) -> bool:
    p = cfg.pots[pot]
    est = turns * cfg.steps_per_rev * cfg.step_delay_s
    print(f"Pump {pot + 1} - {turns} turns (~{est / 60:.1f} min). Put the outlet tube into an EMPTY"
          " measuring cup or a cup on a kitchen scale (1 g = 1 ml), inlet tube in the tank.")
    if not _ask("Ready?"):
        print("Stopped. Nothing was switched.")
        return False
    try:
        hw.pump_on(pot)
        for k in range(turns):
            hw.pump_steps(pot, cfg.steps_per_rev)
            print(f"   turn {k + 1}/{turns}", end="\r", flush=True)
    finally:
        hw.pump_off(pot)
    print()
    ml = _ask_float("How many ml are in the cup? (0 if nothing came out)")
    if not ml:
        print(f"{BAD}: no water. Does the rotor turn? If it turns the wrong way the pump sucks from the"
              f" pot side: set  reverse = true  for {p.name}. Is the tube pressed by the rollers?"
              " Tube too stiff or rollers too loose = no flow.")
        return False
    per_rev = ml / turns
    rate = per_rev / (cfg.steps_per_rev * cfg.step_delay_s) * 60
    print(f"{OK}: {ml:.1f} ml in {turns} turns = {per_rev:.3f} ml per turn (~{rate:.1f} ml/min).")
    print(f"Put this into /etc/plantpi/config.toml under {p.name}:\n    ml_per_rev = {per_rev:.3f}")
    print(f"With dose_ml = {p.dose_ml:.0f} one watering will take about {p.dose_ml / rate:.1f} min.")
    return True


def _median(values):
    s = sorted(values)
    return s[len(s) // 2]


def test_touch(hw, pot: int, cfg, seconds: float = 300, delta: int = 200, sensor: str | None = None,
               clock=time) -> bool:
    """Demo of the control loop: the motor turns while the probe reads 'dry' and stops while you
    hold the probe in your hand (or put it in water). Let go and it starts again. Ctrl+C ends it."""
    name = sensor or cfg.pots[pot].sensor
    print(f"Touch demo - motor {pot + 1} follows probe '{name}'. Ctrl+C to end"
          + (f" (or it ends after {seconds:.0f} s)." if seconds > 0 else "."))
    print("Keep the probe in the AIR and don't touch it for the first readings...")
    vals, last = [], None
    t_end = clock.monotonic() + 15
    while len(vals) < 3 and clock.monotonic() < t_end:
        snap = hw.node_snapshot()
        if snap is not None and snap.seq != last and name in snap.mv:
            last = snap.seq
            vals.append(snap.mv[name])
        clock.sleep(0.1)
    if len(vals) < 3:
        print(f"{BAD}: no readings of '{name}' from the Pico - run:  plantpi test node")
        return False
    base = _median(vals)
    stop_below, go_above = base - delta, base - delta // 2
    print(f"Start value {base} mV. The motor STOPS below {stop_below} mV and runs again above {go_above} mV.")
    print("Now grab the probe with your whole hand (or dip it in water), then let go.\n")

    chunk = max(1, int(0.25 / cfg.step_delay_s))      # check the probe 4 times per second
    running = touched = False
    stops = starts = steps = 0
    last_seq = None
    t_end = clock.monotonic() + seconds if seconds > 0 else float("inf")
    try:
        while clock.monotonic() < t_end:
            snap = hw.node_snapshot()
            stale = snap is None or snap.age(clock.monotonic()) > cfg.stale_s
            mv = None if stale else snap.mv.get(name)
            if mv is None:
                want = False
            else:
                touched = mv < (go_above if touched else stop_below)
                want = not touched
            if want and not running:
                hw.pump_on(pot)
                running, starts = True, starts + 1
            elif not want and running:
                hw.pump_off(pot)
                running, stops = False, stops + 1
            seq = snap.seq if snap is not None else None
            if seq != last_seq:
                last_seq = seq
                state = ("MOTOR RUNS" if running else
                         "STOPPED - no data from the Pico" if mv is None else "STOPPED - probe touched / wet")
                bar = "#" * max(0, min(30, (base - (mv or base)) * 30 // max(1, 3 * delta)))
                print(f"  {name} {mv if mv is not None else '----':>4} mV  {bar:<30s} {state}")
            if running:
                steps += hw.pump_steps(pot, chunk)
            else:
                clock.sleep(0.1)
    except KeyboardInterrupt:
        print("\nEnded with Ctrl+C.")
    finally:
        hw.pump_off(pot)
    turns = steps / cfg.steps_per_rev
    print(f"\nMotor made {turns:.1f} turns, stopped {stops} time(s) because of the probe.")
    if stops:
        print(f"{OK}: the motor reacts to the probe.")
        return True
    print(f"{BAD}: the motor never stopped. Hold the probe harder / longer (the Pico sends a reading every 2 s),"
          f" or make it react sooner:  plantpi test touch {pot + 1} --delta 100")
    return False
