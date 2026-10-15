# v0.7.3 — baseline floor-to-ceiling windows

Baseline 0.1.13 adds two 1.5 m windows on the refrigerator right exterior wall.
From the inside corner: 0.8 m wall, 1.5 m window, 0.8 m wall, 1.5 m window.
Height follows the existing 2.7 m wall. Frame width is estimated at 35 mm.
Glazing uses ideal light transmission; glass reflection/refraction is not modeled.
The closed facade collision remains unchanged. USD, MuJoCo and viewer visuals agree.
SIMPLE daylight is optional, sampled once per episode and recorded for replay.
Sunlight parameters are simulation distributions, not measured calibration.
Fridge remains 0.1.8; no mass, joint, resistance or door collision changes.
