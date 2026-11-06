"""The Pico firmware (MicroPython) run on CPython with a fake `machine` module."""
from __future__ import annotations

import io
import sys
import types
from pathlib import Path

import pytest

from plantpi.node import parse_line

FW = Path(__file__).resolve().parent.parent / "firmware" / "pico_sensor_node"


class FakeADC:
    values = {26: 1834 * 65535 // 3300, 27: 1502 * 65535 // 3300, 28: 400}

    def __init__(self, gpio):
        self.gpio = gpio

    def read_u16(self):
        return self.values[self.gpio]


def put16(regs, a, v):
    regs[a], regs[a + 1] = v & 0xFF, (v >> 8) & 0xFF


class FakeI2C:
    """One BME280/BMP280 with the Bosch datasheet example calibration."""

    def __init__(self, addr=0x76, chip=0x60):
        self.addr, self.regs = addr, bytearray(256)
        r = self.regs
        r[0xD0] = chip
        for i, v in enumerate([27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000]):
            put16(r, 0x88 + 2 * i, v)
        r[0xA1] = 75
        put16(r, 0xE1, 362)
        r[0xE4], r[0xE5], r[0xE6], r[0xE7] = 0x13, 0x29, 0x03, 30
        ap, at = 415148, 519888
        r[0xF7], r[0xF8], r[0xF9] = ap >> 12, (ap >> 4) & 0xFF, (ap & 0xF) << 4
        r[0xFA], r[0xFB], r[0xFC] = at >> 12, (at >> 4) & 0xFF, (at & 0xF) << 4
        r[0xFD], r[0xFE] = 0x6A, 0x00

    def readfrom_mem(self, addr, reg, n):
        if addr != self.addr:
            raise OSError(19, "ENODEV")
        return bytes(self.regs[reg:reg + n])

    def writeto_mem(self, addr, reg, data):
        if addr != self.addr:
            raise OSError(19, "ENODEV")


@pytest.fixture()
def fw(monkeypatch):
    machine = types.ModuleType("machine")
    machine.ADC, machine.I2C, machine.Pin = FakeADC, object, object
    monkeypatch.setitem(sys.modules, "machine", machine)
    monkeypatch.syspath_prepend(str(FW))
    for m in ("main", "bmx280"):
        sys.modules.pop(m, None)
    import main  # noqa: E402  (the firmware's main.py)
    yield main
    for m in ("main", "bmx280"):
        sys.modules.pop(m, None)


def run_node(main, i2c):
    out = io.StringIO()
    node = main.Node(adc_factory=FakeADC, i2c=i2c, ticks_ms=lambda: 1234, out=out)
    node.send(node.hello())
    node.send(node.data())
    node.command("INFO")
    node.send(node.data())
    return [parse_line(line) for line in out.getvalue().splitlines()], node


def test_bme280_messages_parse_on_the_pi(fw):
    msgs, _ = run_node(fw, FakeI2C())
    hello, d1, hello2, d2 = msgs
    assert hello["air"] == "BME280" and hello2 == hello
    assert d1["mv"]["pot1"] == pytest.approx(1834, abs=1)
    assert d1["mv"]["tank"] == pytest.approx(1502, abs=1)
    assert d1["env"]["t_c"] == pytest.approx(25.08)               # Bosch datasheet example
    assert d1["env"]["hpa"] == pytest.approx(1006.5, abs=0.1)
    assert 0 <= d1["env"]["rh"] <= 100
    assert (d1["seq"], d2["seq"]) == (1, 2)


def test_bmp280_has_no_humidity(fw):
    msgs, _ = run_node(fw, FakeI2C(addr=0x77, chip=0x58))
    assert msgs[0]["air"] == "BMP280" and "rh" not in msgs[1]["env"]


def test_no_air_sensor_still_sends_probes(fw):
    msgs, node = run_node(fw, FakeI2C(addr=0x50))
    assert msgs[0]["air"] == "none" and msgs[1]["env"] == {} and len(msgs[1]["mv"]) == 3


def test_median_rejects_spikes(fw):
    class Spiky(FakeADC):
        n = 0

        def read_u16(self):
            Spiky.n += 1
            return 65535 if Spiky.n % 5 == 0 else 30000
    node = fw.Node(adc_factory=Spiky, i2c=None, ticks_ms=lambda: 0, out=io.StringIO())
    assert node.millivolts(Spiky(26)) == 30000 * 3300 // 65535
