# Office chair 0.1.0

FQ photo reconstruction V2, dimensions and physical parameters pending measurement.

- MuJoCo: free base, upper swivel, five caster swivels, ten wheel axles, two armrest slides. 19 joints including the free joint; 14 kg estimated.
- Armrest travel -0.06 to 0 m is a simulation design; tested lowering 0.04 m.
- USD/GLB/Blender contain the visual geometry and regional materials. USD is NOT a PhysX articulation.
- Validation uses applied force/torque in an isolated unoccupied flat-floor simulation; table insertion includes upper-yaw stabilization, not robot manipulation.
- Internal chair collisions are disabled; external collision proxies preserve the previous validated dynamics.
- Six private reference photos are not distributed. Materials are procedural estimates.
- Validation reports are in validation/office_chair/.
