# Lansi wheel-arm robot

Vendor attribution supplied by project owner; archive only identifies wheel_arm_robot. 28 links, 24 revolute joints, 2 continuous wheel joints, 1 fixed joint.

Original URDF, CSV and all 28 original STL files retained under wheel_arm_robot/. preview_meshes/ contains reduced meshes for MuJoCo inspection, with mesh_conversion.json recording counts. urdf/portable.urdf references preview meshes relatively. urdf/robot.xml is the compiled fixed-base, unactuated MuJoCo preview.

24 revolute joints have zero effort/velocity limits. Motor data, control interfaces, self collision geometry and manufacturer calibration must be supplied before control validation. No locomotion, IK or task execution is claimed.

Public publication explicitly confirmed by project owner on 2026-10-27. No upstream license text was supplied; this confirmation is not a blanket downstream open-source license. See provenance.json.
