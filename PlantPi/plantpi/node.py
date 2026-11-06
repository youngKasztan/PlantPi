"""The sensor node (Raspberry Pi Pico, MicroPython): message format and the USB-serial client on the Pi.

The node sends one JSON object per line (newline-terminated), version 1:

  {"v":1,"type":"hello","node":"pico-1","fw":"3.0.0","sensors":["pot1","pot2","tank"]}
  {"v":1,"type":"data","node":"pico-1","seq":42,"up":84012,"mv":{"pot1":1834,"pot2":2610,"tank":1502},
   "env":{"t_c":21.4,"rh":45.2,"hpa":1008.1}}

* "mv" holds one reading per analog sensor, in millivolts (median of 15 ADC reads).
* "env" is optional: air from the BME280/BMP280 on the node's I2C bus
  (a BMP280 has no humidity, so "rh" is missing then; no sensor = no "env").
* Adding a sensor = one new line in the firmware table; the Pi needs no code change,
  only a config entry that uses the new name.
* The transport is just "lines of text". Today it is USB serial; the same lines can be
  sent over Wi-Fi (MQTT/TCP) later by swapping the transport class, nothing else.
"""
from __future__ import annotations

import glob
import json
import logging
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger("plantpi.node")
PROTOCOL_VERSION = 1


@dataclass
class NodeSnapshot:
    node: str
    seq: int
    mv: dict[str, int]
    t: float                                   # monotonic time the line arrived
    uptime_ms: int = 0
    restarts: int = 0                          # how often we saw seq go backwards
    env: dict[str, float] = field(default_factory=dict)   # t_c, rh, hpa (all optional)

    def age(self, now: float) -> float:
        return max(0.0, now - self.t)


def parse_line(line: str | bytes) -> dict | None:
    """Return the message as a dict, or None for anything that is not a valid v1 message."""
    if isinstance(line, bytes):
        line = line.decode("utf-8", "replace")
    line = line.strip()
    if not line.startswith("{"):
        return None                            # MicroPython banner, tracebacks, etc.
    try:
        msg = json.loads(line)
    except ValueError:
        return None
    if not isinstance(msg, dict) or msg.get("v") != PROTOCOL_VERSION:
        return None
    kind = msg.get("type")
    if kind == "hello":
        return msg if isinstance(msg.get("node"), str) else None
    if kind != "data":
        return None
    mv = msg.get("mv")
    if not isinstance(mv, dict) or not isinstance(msg.get("seq"), int):
        return None
    clean = {}
    for k, v in mv.items():
        if not isinstance(k, str) or isinstance(v, bool) or not isinstance(v, (int, float)):
            return None
        if not 0 <= v <= 4000:
            return None
        clean[k] = int(v)
    msg["mv"] = clean
    env = msg.get("env", {})
    if not isinstance(env, dict):
        return None
    limits = {"t_c": (-40.0, 85.0), "rh": (0.0, 100.0), "hpa": (300.0, 1100.0)}
    msg["env"] = {}
    for k, (lo, hi) in limits.items():        # unknown keys are ignored, bad values dropped
        v = env.get(k)
        if isinstance(v, (int, float)) and not isinstance(v, bool) and lo <= v <= hi:
            msg["env"][k] = float(v)
    return msg


class NodeState:
    """Keeps the latest data message. Thread-safe; fed by any transport."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._snap: NodeSnapshot | None = None
        self.hello: dict | None = None
        self.bad_lines = 0
        self._restarts = 0

    def feed(self, line: str | bytes) -> bool:
        msg = parse_line(line)
        if msg is None:
            if isinstance(line, (str, bytes)) and line.strip()[:1] in ("{", b"{"):
                self.bad_lines += 1
            return False
        if msg["type"] == "hello":
            self.hello = msg
            log.info("node %s fw %s sensors %s", msg.get("node"), msg.get("fw"), msg.get("sensors"))
            return True
        with self._lock:
            if self._snap is not None and msg["seq"] < self._snap.seq:
                self._restarts += 1              # the node rebooted
            self._snap = NodeSnapshot(str(msg.get("node", "?")), msg["seq"], msg["mv"],
                                      self._clock(), int(msg.get("up", 0)), self._restarts,
                                      msg["env"])
        return True

    def snapshot(self) -> NodeSnapshot | None:
        with self._lock:
            return self._snap


def find_port() -> str | None:
    """Best guess for the sensor node's serial port (Pico with MicroPython = /dev/ttyACM0)."""
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    for hint in ("MicroPython", "Raspberry_Pi_Pico", "2e8a", "CP210", "CH340", "usb"):
        for p in by_id:
            if hint.lower() in p.lower():
                return p
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


class SerialNode:
    """Reads the sensor node over USB serial in a background thread and reconnects by itself."""

    def __init__(self, port: str = "auto", baud: int = 115200, dtr: bool = True):
        self.port_cfg, self.baud, self.dtr = port, baud, dtr
        self.state = NodeState()
        self.port: str | None = None
        self.error = ""
        self._stop = threading.Event()
        self._ser = None
        self._thread = threading.Thread(target=self._loop, name="sensor-node", daemon=True)
        self._thread.start()

    def snapshot(self) -> NodeSnapshot | None:
        return self.state.snapshot()

    def _open(self):
        import serial  # apt: python3-serial

        port = find_port() if self.port_cfg == "auto" else self.port_cfg
        if port is None:
            raise OSError("no USB serial device found - is the Pico plugged into the Pi?")
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = port, self.baud, 1.0
        # The Pico (MicroPython) only sends while DTR is set ("terminal open"), so keep it on.
        # Set dtr = false in the config for boards that reset on DTR (most ESP32 boards).
        ser.dtr = self.dtr
        ser.rts = False
        ser.open()
        self.port = port
        return ser

    def _loop(self) -> None:
        backoff = 1.0
        while not self._stop.is_set():
            try:
                self._ser = self._open()
                if self.error:
                    log.info("node connected again on %s", self.port)
                self.error, backoff = "", 1.0
                self._ser.write(b"INFO\n")
                buf = b""
                while not self._stop.is_set():
                    chunk = self._ser.readline()
                    if not chunk:
                        continue
                    buf += chunk
                    if buf.endswith(b"\n") or len(buf) > 2048:
                        self.state.feed(buf)
                        buf = b""
            except Exception as e:  # noqa: BLE001 - any serial problem: wait and retry
                msg = f"{type(e).__name__}: {e}"
                if msg != self.error:
                    log.warning("node: %s (retrying)", msg)
                self.error = msg
            finally:
                if self._ser is not None:
                    try:
                        self._ser.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self._ser = None
            self._stop.wait(backoff)
            backoff = min(backoff * 2, 10.0)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3)
