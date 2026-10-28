# Sampling-sheet supply box v0.1.0

Open cavity, separate wall collisions, front finger notch, hinged lid and pickup frame. The hinge is raised to avoid rear-wall collision when opening. MuJoCo actuator lid_position opens to 1.65 rad. USD uses an X-axis revolute joint with negative opening angle (about -94.5 degrees).

dispensing_demo.xml opens the lid, attaches one edge of the native flexible top sheet, lifts above the walls, transfers and releases. Nine lower sheets are fixed stock spacers. This validates top-sheet motion, not frictional grasp, multilayer cloth or automatic single-sheet separation. Material/size parameters remain uncalibrated. See interfaces.json and validation.json.
