# FQ Assets viewer

Lightweight read-only LAN catalog for the assets on `main`. Python's standard
library serves the UI; vendored Three.js 0.180.0 renders compiled MuJoCo geometry.
No frontend build, CDN, database or browser-side Python runtime is required.

## Deployed on 4080S

- URL: http://192.168.31.69:18780
- Service: `systemctl --user status fq-assets-viewer`
- Logs: `journalctl --user -u fq-assets-viewer -n 50`
- Restart: `systemctl --user restart fq-assets-viewer`
- State: `/home/fangqi/.local/share/fq-assets-viewer`
- User lingering and the service are enabled for restart persistence.

This is a LAN service without authentication. Do not expose this port publicly.
The browser only receives rendered geometry, textures, catalog metadata and
previews. It receives no GitHub credentials. Assets cannot be modified via HTTP.

## Version semantics

The header reads the latest GitHub release (currently v0.1.0). Every 60 seconds,
the server fetches `origin/main`; each object's version comes directly from its
`asset.json` in that commit, not the release tag or a duplicated catalog version.
An object at 0.1.1 on main can therefore appear under the v0.1.0 release header.
Browser catalog polling is every 20 seconds; release metadata refreshes every
five minutes. The commit is visible in the UI.

Each commit gets a detached worktree with LFS objects and an immutable exported
snapshot. Catalog activation occurs only after the whole export succeeds. A
failure leaves the last successful snapshot available and marks sync status.
Snapshots persist across restarts. Old snapshots/worktrees are retained; prune
them manually when no longer needed (never remove the active snapshot named in
`current.json`). Application code changes require a service deployment/restart;
asset-only updates on main are automatic.

## Model and property semantics

- Objects: Coke, round table, basket, refrigerator, conference table.
- Scene: office baseline, with a ceiling cutaway and collision overlay.
- Orbit, zoom, pan; drag doors/handles or use joint sliders; restore initial pose.
- Meshes, body transforms, hinge axes/pivots/limits and physical metadata come
  from MuJoCo 3.3.6's compiled MJCF. Browser geometry is generated, not authored
  separately. Textures are capped at 2048 pixels for this preview.
- Joint motion is a kinematic preview; it does not simulate contact or force.
- Friction and mass are model settings, not measured values. A fixed refrigerator
  only defines some moving-part masses, so their sum is not whole-appliance mass.
  Opening force is uncalibrated; mullion is independent; freezer is static.
- FQ provenance and upstream origins remain visible. Existing assets' licenses
  and limitations remain governed by the repository's NOTICE and manifests.

## Runtime prerequisites

4080S uses Python >=3.10, Git + LFS, existing authenticated `~/.local/bin/gh`, and
Docker image `g1-navigation-sim:20261008` (MuJoCo 3.3.6 + NumPy). Pillow 11.3.0 is
mounted read-only from `~/.local/share/fq-assets-validation-deps`. The exporter
container has no network and runs with the current user's UID/GID.

`fq-assets-viewer.service` records the current machine-specific paths. Install
it in `~/.config/systemd/user/`, then daemon-reload and enable --now the service.
To run manually:

```sh
python3 server.py --repo /path/to/fq-sim-assets --state /path/to/viewer-state --host 0.0.0.0 --port 18780
```

## Validation

`python3 -m unittest test_versions` tests a main asset at 0.1.1 against a release
at v0.1.0, including restoration of the cached catalog.

Run `verify_export.py --root ASSET_ROOT --snapshot SNAPSHOT` with MuJoCo/NumPy to
compare every exported body's pose with MuJoCo at three joint configurations,
and verify mass and manifest-version provenance. Initial deployment passed 51
body-pose comparisons across all six assets. Browser QA covered real pointer
drags on both refrigerator doors and both basket handles, asset/scene switching,
texture orientation and an error-free browser console.

Third-party viewer code is Three.js 0.180.0, under its included MIT license in
`web/vendor/LICENSE.txt`.
