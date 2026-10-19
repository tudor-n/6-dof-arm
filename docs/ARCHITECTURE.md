# Software architecture

Three processes' worth of responsibility, two of them in one Python process.

```
┌─ browser ────────────────────────────────────────────────┐
│  index.html        Move + Calibrate tabs, canvas views   │
│  cam_panel.js      Camera tab, STOP, log  (injected)     │
└───────────┬──────────────────────────────────────────────┘
            │ WebSocket /ws  (JSON)      MJPEG /video
┌───────────▼──────────────────────────────────────────────┐
│ server.py                                                │
│   kinematics      fk / fk_planar / ik / ik_auto          │
│   control_loop    50 Hz interpolate -> serial            │
│   pick_seq        hover, descend, close, lift, drop      │
│   vision (thread) ─────────► vision.py                   │
└───────────┬──────────────────────────────────────────────┘
            │ USB serial, "J a0..a5"
┌───────────▼──────────────────────────────────────────────┐
│ ESP32  src/main.c    PWM only, no model of the arm       │
└──────────────────────────────────────────────────────────┘
```

## server.py

One file, deliberately. Everything that knows about the arm's geometry lives here.

**Kinematics.** A 3-link planar chain (`L1`, `L2`, `L3` above a base height `L0`) plus a
base yaw, so the arm is solved in the (radius, height) plane and then rotated. `fk()`
returns the chain `[base, shoulder, elbow, wrist, L3 tip, tool point]` plus the tool
pitch; the **last** point is the tool centre point and is what every caller should use.
`ik()` solves for the tool point. Because the tool offset depends on base yaw and the yaw
depends on the solution, `ik()` resolves it by fixed-point iteration - skipped entirely
when no tool offset is configured, in which case it is the original closed-form solve.

`ik_auto()` is the forgiving variant: if a target is unreachable it shrinks it toward the
base (up to ~66%) and sweeps the tool pitch looking for anything achievable. `move_tip()`
is the single entry point that solves, logs what it actually got versus what was asked,
and with `strict=True` refuses to move at all rather than silently going somewhere else.
The pick sequence uses `strict` for the descent, because a descent that quietly lands
somewhere else is how you crash a gripper into a table.

**Control loop.** 50 Hz. Interpolates `S.cur` toward `S.tgt` under a speed limit, writes
`J a0..a5`, broadcasts `state` every 2 ticks and vision status every 10. It also feeds
`V.tip_cur` / `V.tip_tgt` / `V.jaw_deg` so the camera overlay can draw where the arm
thinks its tool is.

**Joint convention.** `servo = zero + sign * joint_angle`, per joint, from `config.json`.
Several joints have inverted bounds (`min > max`), which is legitimate - it means the
servo counts backwards relative to the joint - so `clamp()` and `ik()` sort the pair
before using it. `joint_angles()` returns only the four arm joints; wrist roll is read
separately by `roll_angle()` because it affects the tool offset but not the link chain.

**Picking.** `start_pick` → `pick_seq`: open the gripper, set wrist roll from the object's
jaw-closing angle, hover, descend strictly, close, lift, move to the drop pose, open. It
runs as a cancellable asyncio task; STOP cancels it and brakes the gripper. `start_pick`
refuses outright when the camera pose is unhealthy.

## vision.py

A `Vision` class imported by `server.py`, running two daemon threads. It was once a
separate process that talked to the server over WebSocket like a browser; folding it in
removed a whole category of synchronisation problems and let the operator UI move into
the browser.

**Capture thread.** Opens the camera (DSHOW on Windows, MJPG locked), undistorts with the
stored intrinsics, detects ArUco 4x4_50 markers, and solves the camera pose.

**Pose estimation** is the part that must not be casually modified:

- `SOLVEPNP_SQPNP`, not `ITERATIVE`. Four near-coplanar points are the textbook case where
  ITERATIVE descends into the wrong local minimum. SQPNP is globally optimal.
- Solves are **rejected**, not averaged in, when they put the camera below the table (the
  mirrored branch of the coplanar ambiguity) or exceed a raw reprojection threshold.
- Rotations are averaged with `avg_rotation()`, which averages rotation *matrices* and
  projects back onto SO(3). **Rodrigues vectors must never be averaged componentwise**:
  `R(a, t) == R(-a, 2pi - t)`, so every rotation has two vector forms, and near
  `|rvec| = pi` - exactly where a downward-looking camera over a +Z-up table sits - the
  axis sign flips between frames and the mean collapses toward identity. This produced a
  mirrored overlay and centimetres of error. `test_geometry.py` pins the behaviour.
- Two reprojection figures are tracked: the **raw** per-frame solve and the error of the
  **averaged** pose in use. When they disagree, the averaging is at fault; when both are
  large, the marker coordinates or the intrinsics are.

**Geometry.** `pixel_to_table()` casts a ray from the camera centre through an undistorted
pixel onto the fitted table plane and returns arm-frame X/Y. This single function underlies
clicking, object detection and the jaw-angle estimate, so an error in the pose or the plane
shows up everywhere at once.

`pixel_to_table_h()` is a second, independent route to the same answer: a homography fitted
directly from the four marker correspondences, using neither the camera matrix nor the solved
pose. `intrinsics_check()` compares the two across the marker area. Since both share the
marker model, that error cancels and the residual implicates the intrinsics - about 3.4 mm of
disagreement per 1% of focal error.

**The homography is a cross-check, not an alternative.** Four points give 8 equations for its
8 unknowns, so it fits them perfectly no matter how wrong the marker model is, and carries no
residual to warn anyone with. PnP has 6 unknowns for the same 8 equations, and those 2 spare
degrees of freedom are the only reason a wrong marker model surfaced as reprojection error at
all. Replacing PnP with a homography would have silently absorbed a 95 mm shape error. The
homography also has no camera position to cast a ray from, so it cannot handle a point above
the table.

**Detection thread.** Lazily loads YOLO-World with editable text prompts. `_axis()` finds
each object's long axis with Otsu plus `fitLine`, then converts it to a jaw-closing angle
in table coordinates, because a gripper has to close *across* a pen, not along it.

**Calibration data** lives in `vision.json`: arm-measured marker positions, the raw
`touched` recordings (with the wrist pitch and roll each was taken at), pick settings, and
the wrist-roll zero and sign.

## Frames and the chain of trust

Each layer is only as good as the one under it, which is why CALIBRATION.md has a strict order:

```
camera intrinsics     camera_matrix.npy      from a checkerboard
      ▼
link lengths + joint zeros                   config.json
      ▼
tool centre point                            config.json "tool"
      ▼
marker positions      measured BY THE ARM, so they inherit everything above
      ▼
table plane           fitted from marker Z values
      ▼
camera pose           solvePnP: markers in arm frame vs. their pixels
      ▼
every table coordinate the system produces
```

Marker positions being arm-measured is a deliberate choice: it puts the camera into the
arm's own space, so systematic FK errors partly cancel instead of adding. The cost is that
nothing is ever checked against an external ruler, so an FK error is invisible. See
ROADMAP.md.

## Browser UI

`index.html` is the original console - Move and Calibrate tabs, a side view and a top
view on canvases, drag handles that drive IK, and a calibration wizard for joint zeros,
signs and limits. It knows nothing about the camera.

`cam_panel.js` is a self-contained IIFE that the server injects before `</body>` when
serving `/`. It adds the Camera tab, a STOP button, and a log panel visible from every
tab. Keeping it separate means the camera UI can be rewritten without touching the
original console.

## Testing

`test_geometry.py` runs offline - no camera, no arm, no serial port. It builds a synthetic
camera over the real marker layout from `vision.json`, so the whole solve → average →
raycast chain is checked against known ground truth, including reproducing the coplanar
pose ambiguity through IPPE and asserting the gates reject the wrong branch. The kinematics
half round-trips IK → FK with a rolled, offset tool across the roll range.

```bash
python test_geometry.py        # or: pytest test_geometry.py
```

`test_markers.py` and `calibrate_camera.py` are interactive hardware tools, not tests.
