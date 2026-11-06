"""Real hardware on the Raspberry Pi.

    Pico node   -> USB serial (node.py), soil + tank probes in millivolts
    relay       -> gpiozero DigitalOutputDevice; cuts the 5 V to the motor driver(s) when idle.
                   Pots may share one relay (the example config does: it feeds both ULN2003s).
    stepper     -> 28BYJ-48 through a ULN2003 board, 4 GPIOs, full-step two-phase drive
"""
from __future__ import annotations

import time

from .config import Config

# Full-step, two coils at a time: most torque, which a peristaltic pump needs.
FULL_STEP = ((1, 1, 0, 0), (0, 1, 1, 0), (0, 0, 1, 1), (1, 0, 0, 1))
# Half-step (4096 steps/rev): smoother, less torque.
HALF_STEP = ((1, 0, 0, 0), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 1, 0),
             (0, 0, 1, 0), (0, 0, 1, 1), (0, 0, 0, 1), (1, 0, 0, 1))


class StepperPump:
    """One 28BYJ-48 + ULN2003, powered through a relay (which may be shared).

    `make_output(pin, active_high, initial)` builds an object with on()/off()/close()/value;
    gpiozero.DigitalOutputDevice on the Pi, a fake in the tests. `relay` is such an object.
    """

    def __init__(self, motor_pins, relay, make_output,
                 steps_per_rev=2048, step_delay_s=0.003, reverse=False,
                 sleep=time.sleep, perf=time.perf_counter):
        self.relay = relay
        self.active = False
        self.coils = [make_output(p, True, False) for p in motor_pins]
        self.seq = FULL_STEP if steps_per_rev == 2048 else HALF_STEP
        self.delay = step_delay_s
        self.direction = -1 if reverse else 1
        self.phase = 0
        self._sleep, self._perf = sleep, perf

    @property
    def powered(self) -> bool:
        return self.active and bool(self.relay.value)

    def _apply(self, pattern) -> None:
        for coil, v in zip(self.coils, pattern):
            if v:
                coil.on()
            else:
                coil.off()

    def power_on(self) -> None:
        self._apply((0, 0, 0, 0))
        self.active = True
        self.relay.on()
        self._sleep(0.05)                     # let the relay contact settle
        self._apply(self.seq[self.phase])     # hold the current position

    def steps(self, n: int, backwards: bool = False) -> int:
        """Do n steps at the configured rate. Returns the steps done."""
        if not self.powered:
            raise RuntimeError("stepper not powered (relay off)")
        d = -self.direction if backwards else self.direction
        deadline = self._perf()
        for _ in range(n):
            self.phase = (self.phase + d) % len(self.seq)
            self._apply(self.seq[self.phase])
            deadline += self.delay
            wait = deadline - self._perf()
            if wait > 0:
                self._sleep(wait)
            elif wait < -0.05:                # we fell far behind (system busy): don't sprint
                deadline = self._perf()
        return n

    def power_off(self) -> None:
        try:
            self._apply((0, 0, 0, 0))         # release the coils: no current, no heat
        finally:
            self.active = False
            self.relay.off()                  # only one pump ever runs, so a shared relay may go off

    def close(self) -> None:
        try:
            self.power_off()
        finally:
            for d in self.coils:
                d.close()


def _gpiozero_output(pin, active_high, initial):
    from gpiozero import DigitalOutputDevice
    return DigitalOutputDevice(pin, active_high=active_high, initial_value=initial)


class RealHardware:
    """Everything PlantPi touches, behind one small interface (same as SimHardware)."""

    def __init__(self, cfg: Config, node=None, make_output=_gpiozero_output):
        self.cfg = cfg
        # Relays first (driven OFF at once), one object per GPIO even if pots share it.
        self._relays = {}
        for p in cfg.pots:
            if p.relay_gpio not in self._relays:
                self._relays[p.relay_gpio] = make_output(p.relay_gpio, cfg.relay_active_high, False)
        self._pumps = [
            StepperPump(p.motor_gpios, self._relays[p.relay_gpio], make_output,
                        cfg.steps_per_rev, cfg.step_delay_s, p.reverse)
            for p in cfg.pots
        ]
        if node is None:
            from .node import SerialNode
            node = SerialNode(cfg.port, cfg.baud, cfg.dtr)
        self.node = node

    # sensors -------------------------------------------------------------
    def node_snapshot(self):
        return self.node.snapshot()

    def node_error(self) -> str:
        return getattr(self.node, "error", "")

    # actuators -----------------------------------------------------------
    def pump_on(self, pot: int) -> None:
        self._pumps[pot].power_on()

    def pump_steps(self, pot: int, n: int, backwards: bool = False) -> int:
        return self._pumps[pot].steps(n, backwards)

    def pump_off(self, pot: int) -> None:
        self._pumps[pot].power_off()

    def all_off(self) -> None:
        for p in self._pumps:
            p.power_off()

    def pumps_running(self) -> list[int]:
        return [i for i, p in enumerate(self._pumps) if p.powered]

    def close(self) -> None:
        try:
            self.all_off()
        finally:
            for p in self._pumps:
                p.close()
            for r in self._relays.values():
                r.close()
            self.node.close()
