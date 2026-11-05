# G1/O6 interlocking mount correction

The user approved the revised assembly preview on 2026-11-05. The previous
assembly flipped the O6-side flange and stacked the two parts end to end.
This revision restores their interlocking orientation and places the O6
mounting plane 23.5 mm beyond the assumed G1 outer ring.

| Quantity | Previous | Revised |
| --- | ---: | ---: |
| O6 root X in wrist frame | 80.42 mm | 65.00 mm |
| Outer ring X (assumed datum) | 41.50 mm | 41.50 mm |
| Exposed mounting length | 38.92 mm | 23.50 mm |
| Bracket/camera axial translation | — | -8.50 mm |

Both hands move -15.42 mm along their wrist X axes, with existing mount
orientations preserved. Wrist camera housings, optical sites, cameras and
optional camera profile positions follow the bracket by -8.50 mm. Existing
camera orientations and intrinsics remain unchanged. Old profile image-fit
residuals are historical and do not validate the revised geometry.

The CAD stack is 32.0 mm, versus the user's approximate 32.5 mm measurement.
The approved preview uses the original CAD dimensions, giving 8.5 mm insertion
at the assumed ring datum. It does not stretch the meshes to match 32.5 mm.
The internal stop is not independently resolved. Native CAD solid overlap
remains unresolved; mounting pieces retain their visual-only collision flags.
This is user-approved assembly geometry, not hardware or grasp calibration.

Robot asset version is 0.1.8. Existing dataset renders, exported USD assets,
compiled MJB scenes and evaluation snapshots do not automatically change.
Regenerate downstream scene exports to use this model. Historical records
must retain their original asset revision.

Validation command (MuJoCo and NumPy required):

```sh
python tools/validate_o6_interlock_mount.py --reference /path/to/previous/assets-root --output validation/o6_interlock_mount_20261105.json
```

The check compares both hands across three joint poses, validates the -15.42 mm
hand and -8.50 mm camera translations, checks URDF/MJCF placement agreement,
and verifies that joint/actuator, mass and contact parameters remain unchanged.
