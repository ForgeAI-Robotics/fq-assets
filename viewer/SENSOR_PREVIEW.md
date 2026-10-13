# G1 sensor preview

G1 O6 / O7 detail pages retain the standalone model as their initial view. Enter
感测预览 to load the current main snapshot's office_baseline. No simulation,
robot-control, stream capture, navigation, or asset-writing endpoints are used.

The round-table preset stands 0.65 m south of the table origin and faces +Y.
The fridge preset stands 1.60 m south of the refrigerator origin and faces +Y.
Robot root Z is the negated neutral visual lower bound plus 2 mm. These are
temporary browser transforms, not edits to the baseline or robot assets. Reset
restores authored joint values and closes both refrigerator doors. Switching
presets keeps the selected camera. Other robot joints remain adjustable.

Camera poses and intrinsics are exported from compiled MuJoCo definitions.
The browser uses their OpenGL camera convention, -Z forward / +Y up, and an
off-axis pinhole projection including principal-point offsets. O6 currently has
7 projection definitions (head RGB, native depth, color-aligned depth, and two
per wrist); O7 has only 3 head definitions. Missing wrist definitions are not
invented. Depth selections show color geometry from the depth projection, not
depth values or a RealSense depth simulation.

The camera inset renders 640 x 480 at at most 10 fps in the client browser. The
near plane is 0.1 mm, far plane 50 m. Display-frustum distance is independent of
the render far plane and does not assert hardware sensing range. The third-person
view can hide ceilings; sensor rendering always restores those surfaces, hides
frustum markers and collision overlays, and preserves normal geometry occlusion.

Head calibration/slot approximations and wrist reference intrinsics remain
explicitly unvalidated in the UI. Camera CAD may obstruct unvalidated wrist
views. The viewer does not silently hide the robot to make those views clear.

Validation on 2026-10-13: O6/O7 camera FK at three waist/wrist poses compared to
MuJoCo; head K compared to source calibration; off-axis projection maps corners
and principal point correctly. Both neutral stand presets checked against the
baseline, with fridge doors at 0, 45, 90 degrees: no robot-scene penetration over
1 mm. This is a static geometry check, not stability or runtime collision control.
Browser checks cover model/sensing switching, both presets, camera selection,
door sliders, waist-pitch linkage, reset, and inset scaling.

Deploy viewer/web files and exporter only. Exported camera data appears on the
next main snapshot. Existing SIMPLE processes and physical G1 remain untouched.
