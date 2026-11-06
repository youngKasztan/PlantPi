"""The control loop: read -> decide -> water -> log.

Safety rules enforced here:
  * water a pot only if its probe says dry AND the tank level is above min_pct AND the
    Pico sensor node data is fresh AND the pot's cooldown and daily limit allow it
  * never two pumps at once; every run is capped at MAX_RUN_S and MAX_DOSE_ML
  * a run stops early if the tank falls below stop_pct, the node goes silent,
    or the service is told to stop; the motor coils and relay are always switched off
  * all timers use the monotonic clock (the Pi 4 has no RTC)
  * nothing is watered automatically during the start-up hold-off
"""
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field

from . import MAX_DOSE_ML, MAX_RUN_S, MV_DEAD, MV_SATURATED
from .config import Calibration, Config, PotConfig
from .notify import sd_notify

log = logging.getLogger("plantpi")
DAY_S = 24 * 3600
CHUNK_S = 0.5            # check stop / tank / node this often while the pump runs


@dataclass
class PotState:
    last_dose_t: float | None = None
    doses: deque = field(default_factory=deque)      # monotonic times of automatic doses
    last_note: str = ""                              # avoids logging the same refusal every minute


@dataclass
class Reading:
    mv: int | None
    pct: float | None
    valid: bool
    note: str = ""


def evaluate(mv: int | None, cal: Calibration | None, what: str = "probe") -> Reading:
    """Turn a millivolt reading into a checked reading."""
    if mv is None:
        return Reading(None, None, False, f"{what}: no value from the Pico (sensor name in SENSORS of main.py?)")
    if mv <= MV_DEAD:
        return Reading(mv, None, False, f"{what}: ~0 V - probe has no power or AOUT wire is off")
    if mv >= MV_SATURATED:
        return Reading(mv, None, False, f"{what}: ~3.3 V - AOUT shorted to 3V3 or wrong pin")
    if cal is None:
        return Reading(mv, None, True, f"{what}: not calibrated")
    pct = cal.percent(mv)
    if pct < -15 or pct > 115:
        return Reading(mv, None, False, f"{what}: {pct:.0f}% is outside the calibrated range")
    return Reading(mv, max(0.0, min(100.0, pct)), True)


def decide(pot: PotConfig, st: PotState, r: Reading, tank_ok: bool, tank_note: str,
           now: float, holdoff_until: float) -> tuple[bool, str]:
    """Pure decision function: (water?, reason). No side effects except pruning old doses."""
    if not r.valid:
        return False, r.note
    if r.pct is None:
        return False, r.note or "probe not calibrated"
    if r.pct >= pot.threshold_pct:
        return False, "moist enough"
    if not tank_ok:
        return False, tank_note or "tank low"
    if now < holdoff_until:
        return False, "start-up hold-off"
    if st.last_dose_t is not None and now - st.last_dose_t < pot.cooldown_min * 60:
        return False, "cooldown"
    while st.doses and now - st.doses[0] > DAY_S:
        st.doses.popleft()
    if len(st.doses) >= pot.max_doses_per_day:
        return False, "daily dose limit reached"
    return True, f"soil {r.pct:.0f}% < {pot.threshold_pct:.0f}%"


class Controller:
    def __init__(self, cfg: Config, hw, store, clock, calibration: dict[str, Calibration],
                 time_synced=lambda: True):
        self.cfg, self.hw, self.store, self.clock = cfg, hw, store, clock
        self.cal = calibration
        self.time_synced_fn = time_synced
        self.synced = False
        self.states = [PotState() for _ in cfg.pots]
        self.stop = False
        self.holdoff_until = 0.0
        self._next_time_check = 0.0
        self._node_note = ""

    # ------------------------------------------------------------- helpers
    def _event(self, kind: str, pot=None, seconds=None, ml=None, reason: str = "") -> None:
        self.store.log_event(self.synced, kind, pot, seconds, ml, reason)
        name = self.cfg.pots[pot].name if pot is not None else "-"
        extra = (f" {seconds:.0f}s" if seconds is not None else "") + (f" {ml:.0f}ml" if ml is not None else "")
        log.info("%-8s %-6s%s %s", kind.upper(), name, extra, reason)

    def _check_time(self) -> None:
        now = self.clock.monotonic()
        if now >= self._next_time_check:
            self.synced = bool(self.time_synced_fn())
            self._next_time_check = now + self.cfg.time_check_interval_s

    def fresh_snapshot(self):
        """The node's latest data, or (None, reason) if there is none or it is too old."""
        snap = self.hw.node_snapshot()
        if snap is None:
            err = self.hw.node_error() if hasattr(self.hw, "node_error") else ""
            return None, "sensor node: no data yet" + (f" ({err})" if err else "")
        age = snap.age(self.clock.monotonic())
        if age > self.cfg.stale_s:
            return None, f"sensor node silent for {age:.0f}s"
        return snap, ""

    def tank(self, snap, limit_pct: float) -> tuple[Reading, bool, str]:
        r = evaluate(snap.mv.get(self.cfg.tank_sensor) if snap else None,
                     self.cal.get(self.cfg.tank_sensor), "tank probe")
        if not r.valid or r.pct is None:
            return r, False, r.note
        if r.pct < limit_pct:
            return r, False, f"tank low ({r.pct:.0f}% < {limit_pct:.0f}%)"
        return r, True, ""

    # ------------------------------------------------------------ actuation
    def water(self, i: int, ml: float, reason: str, manual: bool = False) -> float:
        """Pump about `ml` into pot i (hard-capped). Returns the ml actually pumped (estimate)."""
        pot = self.cfg.pots[i]
        ml = max(1.0, min(float(ml), MAX_DOSE_ML))
        spr, delay = self.cfg.steps_per_rev, self.cfg.step_delay_s
        target = round(ml / pot.ml_per_rev * spr)
        capped = target > int(MAX_RUN_S / delay)
        target = min(target, int(MAX_RUN_S / delay))
        snap, why = self.fresh_snapshot()
        if snap is None:
            self._event("refused", i, reason=("manual: " if manual else "") + why)
            return 0.0
        _, ok, why = self.tank(snap, self.cfg.tank_min_pct)
        if not ok:
            self._event("refused", i, reason=("manual: " if manual else "") + why)
            return 0.0

        chunk = max(1, int(CHUNK_S / delay))
        done, stopped = 0, ""
        t0 = self.clock.monotonic()
        self.hw.all_off()                           # never two pumps at once
        try:
            self.hw.pump_on(i)
            while done < target:
                if self.stop:
                    stopped = "stopped: service shutting down"
                    break
                snap, why = self.fresh_snapshot()
                if snap is None:
                    stopped = f"stopped: {why}"
                    break
                _, ok, why = self.tank(snap, self.cfg.tank_stop_pct)
                if not ok:
                    stopped = f"stopped: {why}"
                    break
                if self.clock.monotonic() - t0 >= MAX_RUN_S:
                    stopped = "stopped: max run time"
                    break
                done += self.hw.pump_steps(i, min(chunk, target - done))
                sd_notify("WATCHDOG=1")
        except Exception as e:                      # noqa: BLE001 - log it, pump goes off below
            stopped = f"stopped: driver error {type(e).__name__}: {e}"
        finally:
            self.hw.pump_off(i)
        ran = self.clock.monotonic() - t0
        pumped = done / spr * pot.ml_per_rev
        st = self.states[i]
        if not manual and done > 0:
            st.last_dose_t = self.clock.monotonic()
            st.doses.append(st.last_dose_t)
        st.last_note = ""
        if capped and not stopped:
            stopped = f"run limited to {MAX_RUN_S / 60:.0f} min (pump too slow for {ml:.0f} ml)"
        why = ("manual: " if manual else "") + reason + (f"; {stopped}" if stopped else "")
        self._event("fault" if "driver error" in stopped else "watered", i, ran, pumped, why)
        return pumped

    # ------------------------------------------------------------- one cycle
    def step(self) -> None:
        self._check_time()
        snap, node_why = self.fresh_snapshot()
        tank_r, tank_ok, tank_note = self.tank(snap, self.cfg.tank_min_pct)
        readings = []
        for pot in self.cfg.pots:
            mv = snap.mv.get(pot.sensor) if snap else None
            readings.append(evaluate(mv, self.cal.get(pot.sensor), pot.sensor))
        self.store.log_sample(self.synced, snap is not None, snap.seq if snap else None,
                              tank_r.mv, tank_r.pct, [
            {"pot": i, "sensor": p.sensor, "mv": r.mv, "pct": r.pct, "valid": r.valid}
            for i, (p, r) in enumerate(zip(self.cfg.pots, readings))],
            snap.env if snap else None)

        if snap is None:
            if node_why != self._node_note:
                self._event("fault", reason=node_why + " - no watering until it is back")
                self._node_note = node_why
            log.info("sample   %s", node_why)
            return
        if self._node_note:
            self._event("start", reason="sensor node back")
            self._node_note = ""
        env = snap.env
        air = (f"  air={env['t_c']:.1f}C" if "t_c" in env else "") + \
              (f"/{env['rh']:.0f}%" if "rh" in env else "") + \
              (f"/{env['hpa']:.0f}hPa" if "hpa" in env else "")
        log.info("sample   tank=%s%s  %s",
                 f"{tank_r.mv}mV" + (f"={tank_r.pct:.0f}%" if tank_r.pct is not None else ""), air,
                 "  ".join(f"{p.name}: {r.mv}mV" + (f"={r.pct:.0f}%" if r.pct is not None else "")
                           for p, r in zip(self.cfg.pots, readings)))

        now = self.clock.monotonic()
        for i, (pot, r) in enumerate(zip(self.cfg.pots, readings)):
            st = self.states[i]
            go, reason = decide(pot, st, r, tank_ok, tank_note, now, self.holdoff_until)
            if go:
                self.water(i, pot.dose_ml, reason)
                if self.stop:
                    return
            elif reason == "moist enough":
                st.last_note = ""
            elif reason != st.last_note:            # log each new refusal once
                kind = "fault" if not r.valid else "refused"
                self._event(kind, i, reason=reason)
                st.last_note = reason

    def process_requests(self) -> None:
        for req in self.store.pending_requests():
            i = req["pot"]
            if not 0 <= i < len(self.cfg.pots):
                self.store.finish_request(req["id"], "refused", "no such pot (is Pot 2 in the config?)")
                continue
            ml = self.water(i, req["ml"], f"request #{req['id']}", manual=True)
            if ml > 0:
                self.store.finish_request(req["id"], "done", f"pumped ~{ml:.0f} ml")
            else:
                self.store.finish_request(req["id"], "refused", "see events (plantpi status)")
            if self.stop:
                return

    def restore_doses(self) -> None:
        """After a restart, reload today's automatic doses so cooldown and the daily limit hold.

        Only possible when the wall clock is NTP-synchronised (the Pi has no RTC)."""
        if not self.synced or not hasattr(self.store, "recent_doses"):
            return
        now = self.clock.monotonic()
        for i, st in enumerate(self.states):
            ages = self.store.recent_doses(i, DAY_S)
            st.doses = deque(sorted(now - a for a in ages))
            st.last_dose_t = st.doses[-1] if st.doses else None

    # ------------------------------------------------------------- main loop
    def run(self, max_cycles: int | None = None) -> None:
        self.hw.all_off()
        self.holdoff_until = self.clock.monotonic() + self.cfg.startup_holdoff_min * 60
        self._check_time()
        self.restore_doses()
        self._event("start", reason=f"hold-off {self.cfg.startup_holdoff_min:.0f} min")
        sd_notify("READY=1")
        next_sample = self.clock.monotonic()
        cycles = 0
        try:
            while not self.stop:
                now = self.clock.monotonic()
                if now >= next_sample:
                    self.step()
                    cycles += 1
                    next_sample = max(next_sample + self.cfg.sample_interval_s, now + 1.0)
                self.process_requests()
                if max_cycles is not None and cycles >= max_cycles:
                    break
                sd_notify("WATCHDOG=1")
                self.clock.sleep(min(1.0, max(0.05, next_sample - self.clock.monotonic())))
        finally:
            self.hw.all_off()
            sd_notify("STOPPING=1")
            self._event("stop", reason="all pumps off")
