"""Tests of the PlantPi v2 logic. Run:  python3 -m pytest -q"""
from __future__ import annotations

import textwrap
from pathlib import Path

import pytest

from plantpi import MAX_DOSE_ML, MAX_RUN_S
from plantpi.config import Calibration, ConfigError, load_config
from plantpi.controller import Controller, PotState, Reading, decide, evaluate
from plantpi.hardware import FULL_STEP, StepperPump
from plantpi.node import NodeState, parse_line
from plantpi.sim import DRY_MV, EMPTY_MV, FULL_MV, WET_MV, FastClock, SimHardware
from plantpi.store import Store

ROOT = Path(__file__).resolve().parent.parent
EXAMPLE = ROOT / "config.example.toml"


# ------------------------------------------------------------------ helpers
def two_pot_config(tmp_path):
    text = EXAMPLE.read_text()
    pot2 = text.split("# [[pot]]", 1)[1]
    uncommented = "[[pot]]" + "\n".join(l[2:] if l.startswith("# ") else l for l in pot2.splitlines())
    p = tmp_path / "two.toml"
    p.write_text(text.split("# --- Pot 2")[0] + uncommented + "\n")
    return load_config(p)


def make(tmp_path, pots=1, start_pct=None, tank_ml=1800.0, holdoff=0):
    cfg = load_config(EXAMPLE) if pots == 1 else two_pot_config(tmp_path)
    cfg.startup_holdoff_min = holdoff
    clock = FastClock(speed=0)
    hw = SimHardware(cfg, clock, start_pct=start_pct or [20.0] * pots, tank_ml=tank_ml)
    cal = {p.sensor: Calibration(DRY_MV, WET_MV) for p in cfg.pots}
    cal[cfg.tank_sensor] = Calibration(EMPTY_MV, FULL_MV)
    store = Store(":memory:")
    return cfg, clock, hw, store, Controller(cfg, hw, store, clock, cal)


def events(store, kind=None):
    rows = store.db.execute("SELECT * FROM events ORDER BY id").fetchall()
    return [r for r in rows if kind is None or r["kind"] == kind]


# ------------------------------------------------------------------ config
def test_example_config_loads_one_pot():
    cfg = load_config(EXAMPLE)
    assert len(cfg.pots) == 1
    assert cfg.pots[0].motor_gpios == [17, 27, 22, 24]
    assert cfg.pots[0].relay_gpio == 18


def test_pot2_block_uncomments_cleanly(tmp_path):
    cfg = two_pot_config(tmp_path)
    assert [p.sensor for p in cfg.pots] == ["pot1", "pot2"]
    assert cfg.pots[1].motor_gpios == [12, 16, 20, 21]
    assert cfg.pots[1].relay_gpio == cfg.pots[0].relay_gpio == 18


@pytest.mark.parametrize("patch, msg", [
    ("motor_gpios = [17, 27, 22, 24]", "motor_gpios = [17, 27, 22]"),
    ("relay_gpio = 18 ", "relay_gpio = 17 "),
    ("relay_gpio = 18 ", "relay_gpio = 2 "),
    ('sensor = "pot1"', 'sensor = "tank"'),
    ("step_delay_ms = 3.0", "step_delay_ms = 0.5"),
])
def test_bad_configs_are_rejected(tmp_path, patch, msg):
    p = tmp_path / "bad.toml"
    p.write_text(EXAMPLE.read_text().replace(patch, msg, 1))
    with pytest.raises(ConfigError):
        load_config(p)


def test_dose_is_clamped(tmp_path):
    p = tmp_path / "c.toml"
    p.write_text(EXAMPLE.read_text().replace("dose_ml = 30 ", "dose_ml = 5000 ", 1))
    assert load_config(p).pots[0].dose_ml == MAX_DOSE_ML


# ------------------------------------------------------------------ node protocol
FIRMWARE_LINES = [   # copied from the firmware host test output
    '{"v":1,"type":"hello","node":"pico-1","fw":"2.0.0","sensors":["pot1","pot2","tank"],"air":"BME280"}',
    '{"v":1,"type":"data","node":"pico-1","seq":1,"up":110,"mv":{"pot1":1834,"pot2":377,"tank":2210},'
    '"env":{"t_c":25.08,"hpa":1006.5,"rh":39.0}}',
    '{"v":1,"type":"data","node":"pico-1","seq":5,"up":8135,"mv":{"pot1":1834,"pot2":377,"tank":2210}}',
]


def test_parse_firmware_lines():
    hello, data, no_env = (parse_line(l) for l in FIRMWARE_LINES)
    assert hello["type"] == "hello"
    assert data["mv"] == {"pot1": 1834, "pot2": 377, "tank": 2210}
    assert data["env"] == {"t_c": 25.08, "rh": 39.0, "hpa": 1006.5}
    assert no_env["env"] == {}


@pytest.mark.parametrize("line", [
    "", "ets Jun  8 2016 00:22:57", "{not json", '{"v":2,"type":"data","seq":1,"mv":{}}',
    '{"v":1,"type":"data","seq":1,"mv":{"pot1":"high"}}', '{"v":1,"type":"data","seq":1,"mv":{"pot1":9999}}',
    '{"v":1,"type":"data","mv":{"pot1":100}}', '{"v":1,"type":"data","seq":1,"mv":[1,2]}',
])
def test_parse_rejects_garbage(line):
    assert parse_line(line) is None


def test_bad_env_values_are_dropped_not_fatal():
    m = parse_line('{"v":1,"type":"data","seq":1,"mv":{"tank":1500},"env":{"t_c":999,"rh":40}}')
    assert m["env"] == {"rh": 40.0}


def test_node_state_tracks_age_and_restarts():
    now = [100.0]
    st = NodeState(clock=lambda: now[0])
    assert st.snapshot() is None
    st.feed(b'{"v":1,"type":"data","seq":7,"mv":{"tank":1500}}\n')
    now[0] = 112.0
    snap = st.snapshot()
    assert snap.seq == 7 and snap.age(now[0]) == pytest.approx(12.0)
    st.feed('{"v":1,"type":"data","seq":1,"mv":{"tank":1500}}')     # node rebooted
    assert st.snapshot().restarts == 1
    assert st.feed("{broken") is False and st.bad_lines == 1


# ------------------------------------------------------------------ readings and decisions
def test_evaluate():
    cal = Calibration(dry=2600, wet=1200)
    assert evaluate(1900, cal).pct == pytest.approx(50.0)
    assert not evaluate(20, cal).valid                    # probe unpowered
    assert not evaluate(3300, cal).valid                  # saturated
    assert not evaluate(None, cal).valid                  # missing in message
    assert evaluate(1900, None).pct is None and evaluate(1900, None).valid
    assert not evaluate(500, cal).valid                   # way past "wet": not soil


def test_decide_rules():
    pot = load_config(EXAMPLE).pots[0]
    dry, wet = Reading(2400, 10.0, True), Reading(1500, 80.0, True)
    st = PotState()
    assert decide(pot, st, wet, True, "", 0, 0) == (False, "moist enough")
    assert decide(pot, st, dry, False, "tank low", 0, 0) == (False, "tank low")
    assert decide(pot, st, dry, True, "", 0, 100) == (False, "start-up hold-off")
    assert decide(pot, st, dry, True, "", 0, 0)[0] is True
    st.last_dose_t = 0
    assert decide(pot, st, dry, True, "", 60, 0) == (False, "cooldown")
    st.last_dose_t = None
    st.doses.extend([1, 2, 3, 4])
    assert decide(pot, st, dry, True, "", 10, 0) == (False, "daily dose limit reached")
    assert decide(pot, st, dry, True, "", 2 * 86400, 0)[0] is True   # old doses expire


# ------------------------------------------------------------------ stepper driver
class FakeOut:
    def __init__(self, pin, active_high, initial):
        self.pin, self.value, self.closed = pin, int(initial), False

    def on(self):
        self.value = 1

    def off(self):
        self.value = 0

    def close(self):
        self.closed = True


def test_stepper_sequence_and_release():
    s = StepperPump([17, 27, 22, 24], FakeOut(18, True, False), FakeOut, sleep=lambda t: None)
    assert not s.powered and all(c.value == 0 for c in s.coils)
    with pytest.raises(RuntimeError):
        s.steps(1)                                       # relay off -> refuse
    s.power_on()
    seen = []
    for _ in range(4):
        s.steps(1)
        seen.append(tuple(c.value for c in s.coils))
    assert sorted(seen) == sorted(FULL_STEP)            # all four phases, two coils each
    s.steps(4, backwards=True)
    s.power_off()
    assert not s.powered and all(c.value == 0 for c in s.coils)
    s.close()
    assert all(c.closed for c in s.coils)


def test_two_pumps_share_one_relay(tmp_path):
    from plantpi.hardware import RealHardware
    cfg = two_pot_config(tmp_path)
    hw = RealHardware(cfg, node=SimHardware(cfg, FastClock(0)).node, make_output=FakeOut)
    assert len(hw._relays) == 1
    relay = hw._relays[18]
    hw.pump_on(1)
    assert relay.value == 1 and hw.pumps_running() == [1]
    hw.pump_off(1)
    assert relay.value == 0 and hw.pumps_running() == []
    hw.close()
    assert relay.closed


# ------------------------------------------------------------------ controller with simulated hardware
def test_waters_dry_pot_and_logs(tmp_path):
    cfg, clock, hw, store, ctl = make(tmp_path)
    ctl.run(max_cycles=1)
    w = events(store, "watered")
    assert len(w) == 1
    assert w[0]["ml"] == pytest.approx(cfg.pots[0].dose_ml, rel=0.02)
    assert hw.pumped_ml[0] == pytest.approx(cfg.pots[0].dose_ml, rel=0.02)
    assert hw.pumps_running() == []
    s = store.db.execute("SELECT * FROM samples").fetchone()
    assert s["node_ok"] == 1 and s["air_c"] is not None


def test_moist_pot_is_not_watered(tmp_path):
    _, _, hw, store, ctl = make(tmp_path, start_pct=[70.0])
    ctl.run(max_cycles=1)
    assert events(store, "watered") == [] and hw.pumped_ml[0] == 0


def test_never_two_pumps_at_once(tmp_path):
    _, _, hw, store, ctl = make(tmp_path, pots=2, start_pct=[10.0, 10.0])
    ctl.run(max_cycles=1)
    assert len(events(store, "watered")) == 2
    assert hw.max_simultaneous == 1


def test_low_tank_refuses(tmp_path):
    _, _, hw, store, ctl = make(tmp_path, tank_ml=100.0)        # 5 % of the probe zone
    ctl.run(max_cycles=1)
    assert hw.pumped_ml[0] == 0
    assert any("tank low" in e["reason"] for e in events(store, "refused"))


def test_tank_running_low_stops_the_pump(tmp_path):
    cfg, _, hw, store, ctl = make(tmp_path, tank_ml=330.0)      # 16.5 %: may start, stop at 10 %
    cfg.pots[0].dose_ml = 200
    cfg.pots[0].ml_per_rev = 3.0               # fast pump, so the 20-min cap does not end it first
    ctl.run(max_cycles=1)
    w = events(store, "watered")[0]
    assert "stopped: tank low" in w["reason"]
    assert hw.tank_ml == pytest.approx(200.0, abs=20)   # 10 % of 2000 ml, +- probe noise
    assert hw.pumps_running() == []


def test_silent_node_blocks_watering(tmp_path):
    _, clock, hw, store, ctl = make(tmp_path)
    hw.node_snapshot()
    hw.node_silent = True
    clock.sleep(30)
    ctl.run(max_cycles=1)
    assert hw.pumped_ml[0] == 0
    assert any("silent" in e["reason"] for e in events(store, "fault"))


def test_node_going_silent_mid_run_stops_pump(tmp_path):
    cfg, clock, hw, store, ctl = make(tmp_path)
    real_steps = hw.pump_steps
    calls = {"n": 0}

    def steps(pot, n, backwards=False):
        calls["n"] += 1
        if calls["n"] == 3:
            hw.node_silent = True                  # Pico unplugged during watering
        return real_steps(pot, n, backwards)
    hw.pump_steps = steps
    ctl.run(max_cycles=1)
    w = events(store, "watered")[0]
    assert "silent" in w["reason"] and w["ml"] < cfg.pots[0].dose_ml
    assert hw.pumps_running() == []


def test_driver_error_switches_everything_off(tmp_path):
    _, _, hw, store, ctl = make(tmp_path)
    hw.fail_on_steps = True
    ctl.run(max_cycles=1)
    assert hw.pumps_running() == []
    assert any("driver error" in e["reason"] for e in events(store, "fault"))


def test_unpowered_probe_is_a_fault_not_watering(tmp_path):
    _, _, hw, store, ctl = make(tmp_path)
    hw.unplugged.add("pot1")
    ctl.run(max_cycles=1)
    assert hw.pumped_ml[0] == 0
    assert any("no power" in e["reason"] for e in events(store, "fault"))


def test_holdoff_prevents_auto_watering(tmp_path):
    _, _, hw, store, ctl = make(tmp_path, holdoff=10)
    ctl.run(max_cycles=1)
    assert hw.pumped_ml[0] == 0


def test_cooldown_between_doses(tmp_path):
    cfg, clock, hw, store, ctl = make(tmp_path, start_pct=[0.0])
    cfg.sample_interval_s = 60
    ctl.run(max_cycles=5)                      # 5 min: still dry after 30 ml, but cooldown is 60 min
    assert len(events(store, "watered")) == 1


def test_manual_request_capped_and_done(tmp_path):
    cfg, _, hw, store, ctl = make(tmp_path, start_pct=[90.0])
    rid = store.add_request(0, 10.0)
    store.add_request(5, 10.0)                 # pot 6 does not exist
    ctl.run(max_cycles=1)
    rows = {r["id"]: r for r in store.db.execute("SELECT * FROM requests")}
    assert rows[rid]["status"] == "done"
    assert hw.pumped_ml[0] == pytest.approx(10.0, rel=0.02)
    assert rows[rid + 1]["status"] == "refused"


def test_run_is_capped_in_time(tmp_path):
    cfg, clock, hw, store, ctl = make(tmp_path, start_pct=[90.0], tank_ml=1e9)
    cfg.pots[0].ml_per_rev = 0.01              # very slow pump: 250 ml would take hours
    ctl.water(0, 250, "test")
    assert hw.steps_done[0] <= int(MAX_RUN_S / cfg.step_delay_s)
    assert hw.pumps_running() == []
    assert "20 min" in events(store, "watered")[0]["reason"]


def test_doses_survive_a_restart(tmp_path):
    cfg, clock, hw, store, ctl = make(tmp_path, start_pct=[0.0])
    ctl.run(max_cycles=1)
    assert len(events(store, "watered")) == 1
    # service restarts (new Controller, same database, clock synced): cooldown must still hold
    ctl2 = Controller(cfg, hw, store, clock, ctl.cal)
    ctl2.run(max_cycles=1)
    assert len(events(store, "watered")) == 1
    assert any(e["reason"] == "cooldown" for e in events(store, "refused"))


def test_stop_flag_ends_watering(tmp_path):
    _, _, hw, store, ctl = make(tmp_path)
    real_steps = hw.pump_steps

    def steps(pot, n, backwards=False):
        ctl.stop = True
        return real_steps(pot, n, backwards)
    hw.pump_steps = steps
    ctl.run(max_cycles=3)
    assert "shutting down" in events(store, "watered")[0]["reason"]
    assert hw.pumps_running() == []


def test_v1_database_is_refused(tmp_path):
    import sqlite3
    db = tmp_path / "old.db"
    con = sqlite3.connect(db)
    con.execute("CREATE TABLE samples (id INTEGER PRIMARY KEY, tank_ok INTEGER)")
    con.commit()
    con.close()
    with pytest.raises(RuntimeError):
        Store(db)


def test_cli_sim_smoke(tmp_path, capsys):
    from plantpi.__main__ import main
    state = ["--state", str(tmp_path)]
    assert main(["run", "--sim", "--speed", "0", "--cycles", "2", *state]) == 0
    assert main(["water", "1", "20", "--sim", *state]) == 0
    assert main(["run", "--sim", "--speed", "0", "--cycles", "1", *state]) == 0
    assert main(["status", "--sim", *state]) == 0
    out = capsys.readouterr().out
    assert "request #1" in out and "watered" in out
    assert main(["pins"]) == 0


def test_example_config_is_documented():
    text = EXAMPLE.read_text()
    assert textwrap.dedent(text).count("[[pot]]") == 2


def test_serial_node_over_a_pty():
    """The real SerialNode reading from a pseudo-terminal that plays the Pico."""
    pytest.importorskip("serial")
    import os
    import pty
    import time as _t
    from plantpi.node import SerialNode
    master, slave = pty.openpty()
    node = SerialNode(os.ttyname(slave), 115200)
    try:
        t_open = _t.monotonic() + 5
        while node.port is None and _t.monotonic() < t_open:   # opening flushes old input
            _t.sleep(0.05)
        _t.sleep(0.2)
        for line in FIRMWARE_LINES:
            os.write(master, line.encode() + b"\n")
        deadline = _t.monotonic() + 5
        while _t.monotonic() < deadline and (node.snapshot() is None or node.snapshot().seq != 5):
            _t.sleep(0.05)
        snap = node.snapshot()
        assert snap is not None and snap.seq == 5 and snap.mv["tank"] == 2210
        assert node.state.hello["air"] == "BME280"
        assert os.read(master, 64).startswith(b"INFO")     # the Pi asks the node to introduce itself
    finally:
        node.close()
        os.close(master)
        os.close(slave)


def test_touch_demo_stops_and_restarts(tmp_path, capsys):
    from plantpi.commission import test_touch
    cfg, clock, hw, store, ctl = make(tmp_path, start_pct=[10.0])
    cfg.pots[0].ml_per_rev = 0.0001            # the demo itself must not wet the soil noticeably
    real = hw.node_snapshot
    calls = {"n": 0}

    def snap():
        calls["n"] += 1
        if 40 <= calls["n"] < 80:
            hw.moist[0] = 0.8                  # hand on the probe
        elif calls["n"] >= 80:
            hw.moist[0] = 0.1                  # let go
        return real()
    hw.node_snapshot = snap
    assert test_touch(hw, 0, cfg, seconds=40, clock=clock) is True
    out = capsys.readouterr().out
    assert "STOPPED - probe touched" in out and "MOTOR RUNS" in out
    assert hw.pumps_running() == []
    ons = [h for h in hw.history if h[2]]
    assert len(ons) >= 2                       # ran, stopped, ran again


def test_touch_demo_without_node_fails_safely(tmp_path):
    from plantpi.commission import test_touch
    cfg, clock, hw, store, ctl = make(tmp_path)
    hw.node_silent = True
    assert test_touch(hw, 0, cfg, seconds=5, clock=clock) is False
    assert hw.pumps_running() == [] and hw.steps_done[0] == 0
