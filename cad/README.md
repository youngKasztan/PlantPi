# Peristaltic pump for the 28BYJ-48

`peristaltic_pump_28byj48.py` is an Autodesk Fusion script. Run it from
*Utilities → Add-Ins → Scripts* and it builds the pump from the numbers in `P`.
The defaults are for soft silicone tube, 2 mm inside × 4 mm outside diameter.

| Part | Qty | Print |
|---|---|---|
| Housing | 1 | plate on the bed, no supports |
| Rotor base | 1 | disc on the bed, pins up |
| Rotor top | 1 | flip it so the flat face is on the bed |
| Roller | 3 | standing up |

PLA or PETG, 0.2 mm layers, 4 walls, 40 % infill.

**Hardware:** 2 × M3 screws, about 8–10 mm long. They go up through the motor ears into the pillars and cut their own thread. A drop of glue holds the rotor top on the pins.

**Assembly**

1. Screw the housing onto the motor.
2. Put the 3 rollers on the pins and press the rotor top on.
3. Push the rotor onto the shaft. It is a double-D hole, so it only fits two ways.
4. Lay the tube around the rollers and push both ends into the exit slots.

**Theory:** about 0.29 ml per turn. Measure the real value with `plantpi test pump 1` and put it in the config.

**If the motor stalls or buzzes**, set `occlusion = 0.75` and print a new housing. A larger occlusion gap squeezes the tube less.

**If no water comes out**, set `occlusion = 0.95`.
