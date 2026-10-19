# FQ Assets viewer

Read-only LAN catalog for assets on `main`. The three top-level categories are
**Objects, Scenes and Actions**. Robots, hands, cameras and materials/textures
are object subtypes. Python's standard library serves the UI; vendored Three.js
0.180.0 renders exported MuJoCo geometry without a frontend build or CDN.

## Deployment on 4080S

- URL: http://192.168.31.69:18780/
- Service: `systemctl --user status fq-assets-viewer`
- Logs: `journalctl --user -u fq-assets-viewer -n 50`
- Application checkout: `/home/fangqi/code/fq-assets-arch-release`
- State and immutable snapshots: `/home/fangqi/.local/share/fq-assets-viewer`
- Service configuration: inspect `systemctl --user cat fq-assets-viewer`; the
  machine-specific override takes precedence over the repository's example unit.

The service is LAN-only without authentication. Assets cannot be edited over
HTTP, and GitHub credentials are not sent to the browser.

## Version semantics

The header displays the latest GitHub release (**v0.7.2** at this documentation
update). The server fetches `origin/main` every 60 seconds. Each object's version
comes from its `asset.json` at that commit, independently of the release header.
For v0.7.2 the refrigerator is **0.1.8**, and office baseline is **0.1.12**.
Browser polling is every 20 seconds; release metadata refreshes every five minutes.

Each commit gets a detached worktree with hydrated LFS files and an immutable
export snapshot. Activation occurs after the complete export succeeds. A sync
failure keeps the last successful snapshot and reports an error. Old snapshots
are retained; never remove the snapshot referenced by `current.json` while active.

Asset-only changes on `main` sync automatically. Application-code changes require
deployment of the corresponding viewer code and a service restart. Documentation
commits may advance the displayed main SHA without changing the asset versions.
SIMPLE uses a separate pinned asset snapshot and never upgrades a running task
merely because the browser has refreshed.

## Model, material and property semantics

- Orbit, zoom and pan; drag doors/handles or use sliders; restore initial pose.
  Calibrated hand-pose presets and action previews are also available.
- Geometry, body transforms, hinge pivots/axes/limits and physical metadata come
  from compiled MJCF. Textures are capped at 2048 pixels for browser preview.
- Refrigerator regional roughness and metalness come from `appearance.json` and
  are exported explicitly for both the standalone asset and baseline. Other
  materials retain the existing renderer fallback. Window lighting is not baked
  into the refrigerator; browser lighting differs from the isolated window-light
  comparison and from a particular SIMPLE rendering environment.
- The upper refrigerator doors share a 10 mm central arch and conforming front
  details. Collision overlay shows the synchronized piecewise convex geometry.
- Browser motion is **kinematic only**: it does not test contacts, force or torque.
- Refrigerator whole-appliance mass is 91 kg, with 14.3 kg in movable parts and
  76.7 kg in the fixed assembly. Component allocation and inertia are estimates.
- Right-door frictionloss is0.1 N m and viscous damping is0;
  fixed and compatible uniform modes both use0.1 N m. This is not hardware calibration. Left-door/mullion settings remain
  unchanged. Real seal force is uncalibrated, mullion motion is independent, and
  the freezer is static. Appearance parameters are not measured hardware values.
- Provenance and licenses remain governed by NOTICE.md and asset manifests.

## Runtime and manual startup

4080S uses Python 3.10+, Git LFS, authenticated `~/.local/bin/gh`, and the existing
`g1-navigation-sim:20261008` exporter image. Validation dependencies are mounted
read-only from `~/.local/share/fq-assets-validation-deps`. Export containers run
without network access and with the current user's UID/GID.

```sh
python3 viewer/server.py --repo /path/to/fq-assets --state /path/to/viewer-state --host 0.0.0.0 --port 18780
```

Run this from the repository root. A separate state directory and port should be
used for development previews so they do not replace the active service.

## Validation

From `viewer/`, `python3 -m unittest test_versions` tests release/main version
separation and restoration of the cached catalog. The older version numbers in
these fixtures are test data, not the currently deployed release.

`verify_export.py --root ASSET_ROOT --snapshot SNAPSHOT` checks body transforms
at three joint configurations, mass and manifest versions. v0.7.2 passed **711
body-pose comparisons**, explicit regional-material export checks, and deployed
browser checks for both door controls and the published version. The slider's
display rounds to its configured step; browser movement is not a dynamics test.

The corresponding SIMPLE validation separately covers torque-driven opening and
closing plus independent same-engine replay in MuJoCo and CPU PhysX, fixed and
uniform modes. Four replay errors were zero. This does not establish hardware,
GPU PhysX or cross-engine replay equivalence.

Third-party Three.js 0.180.0 is covered by `web/vendor/LICENSE.txt` (MIT).
