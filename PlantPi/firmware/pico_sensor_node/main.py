# PlantPi sensor node - Raspberry Pi Pico (RP2040) running MicroPython.
#
# Copy this file and bmx280.py onto the Pico (Thonny: File > Save as > Raspberry Pi Pico).
# It starts by itself at power-up and sends one JSON line every 2 s over the USB cable:
#
#   {"v":1,"type":"data","node":"pico-1","seq":42,"up":84012,
#    "mv":{"pot1":1834,"tank":1502,"pot2":377},"env":{"t_c":21.4,"hpa":1008.1,"rh":45.2}}
#
# To add an analog sensor: add one line to SENSORS. The Pico has 3 analog pins: GP26, GP27, GP28.
# The node never decides anything about watering. It only measures; the Raspberry Pi 4 decides.

import json
import sys
import time

from machine import ADC, I2C, Pin

from bmx280 import Bmx280

NODE_ID = "pico-1"
FW_VERSION = "3.0.0"
PERIOD_MS = 2000          # one message every 2 s
N_SAMPLES = 15            # median of 15 ADC reads per sensor
VREF_MV = 3300            # ADC full scale = the Pico's 3.3 V (ADC_VREF pin)

SENSORS = [               # (name, GPIO). Physical pins: GP26 = pin 31, GP27 = pin 32, GP28 = pin 34
    ("pot1", 26),         # soil probe, pot 1 (V2.0.0)
    ("tank", 27),         # probe standing upright in the tank (V1.0)
    ("pot2", 28),         # soil probe, pot 2 - floats until wired; the Pi ignores it until configured
]

I2C_SDA, I2C_SCL = 4, 5   # BME280/BMP280: GP4 = pin 6, GP5 = pin 7


def median(values):
    s = sorted(values)
    return s[len(s) // 2]


class Node:
    def __init__(self, adc_factory=ADC, i2c=None, ticks_ms=None, out=None):
        self.adcs = [(name, adc_factory(gpio)) for name, gpio in SENSORS]
        self.air = Bmx280(i2c) if i2c is not None else None
        self.ticks_ms = ticks_ms or time.ticks_ms
        self.out = out or sys.stdout
        self.seq = 0
        if self.air:
            self.air.probe()

    def millivolts(self, adc):
        raw = median([adc.read_u16() for _ in range(N_SAMPLES)])
        return raw * VREF_MV // 65535

    def hello(self):
        return {"v": 1, "type": "hello", "node": NODE_ID, "fw": FW_VERSION,
                "sensors": [n for n, _ in SENSORS], "air": self.air.kind if self.air else "none"}

    def data(self):
        self.seq += 1
        msg = {"v": 1, "type": "data", "node": NODE_ID, "seq": self.seq, "up": self.ticks_ms(),
               "mv": {name: self.millivolts(adc) for name, adc in self.adcs}}
        if self.air:
            if self.air.addr is None and self.seq % 15 == 0:       # sensor plugged in later? (every 30 s)
                self.air.probe()
            env = self.air.read()
            if env:
                msg["env"] = env
        return msg

    def send(self, msg):
        self.out.write(json.dumps(msg) + "\n")

    def command(self, line):
        if line.strip() == "INFO":
            self.send(self.hello())


def make_i2c():
    try:
        return I2C(0, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=100000)
    except Exception:  # noqa: BLE001 - no air sensor is fine
        return None


def run():
    import select

    led = Pin("LED", Pin.OUT)
    node = Node(i2c=make_i2c())
    node.send(node.hello())
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ""
    next_t = time.ticks_ms()
    while True:
        while poll.poll(0):                       # commands from the Pi, e.g. "INFO"
            ch = sys.stdin.read(1)
            if ch in ("\n", "\r"):
                node.command(buf)
                buf = ""
            elif len(buf) < 16:
                buf += ch
        if time.ticks_diff(time.ticks_ms(), next_t) >= 0:
            next_t = time.ticks_add(next_t, PERIOD_MS)
            led.on()
            node.send(node.data())
            led.off()
        time.sleep_ms(5)


if __name__ == "__main__":
    run()
