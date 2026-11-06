"""PlantPi - automatic plant monitoring and watering on a Raspberry Pi 4.

v3: soil and tank probes are read by a Raspberry Pi Pico sensor node (MicroPython, USB serial),
the pump is a 3D-printed peristaltic pump on a 28BYJ-48 stepper motor.
"""

__version__ = "3.0.0"

# Absolute safety limits. Config values are clamped to these; nothing can exceed them.
MAX_RUN_S = 1200.0        # one pump run never lasts longer than 20 min
MAX_DOSE_ML = 250.0       # and never moves more than this (by the calibrated ml/rev)
MIN_COOLDOWN_MIN = 5.0
# Sensor-node millivolt readings at or below this mean the probe has no power (wire off).
MV_DEAD = 80
# ...and at or above this the ADC is saturated (shorted to 3V3 or wrong pin).
MV_SATURATED = 3250        # Pico ADC full scale is 3300 mV
