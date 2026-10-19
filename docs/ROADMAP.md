# Roadmap

## Where this came from

The arm picked pens and pliers reliably while its base was weighted down with dumbbells.
Knocking the table destroyed that, and the base was bolted down properly. Re-measuring
everything afterwards did not restore accuracy: the gripper landed centimetres off in a
pattern that varied across the table, and the overlay grid came out roughly mirrored
against the real table.

The cause was software, introduced when vision moved from a separate WebSocket-client
process into `vision.py` inside the server.

## Fixed

### Rotation averaging (the mirrored overlay)

Camera poses were averaged over 15 frames by taking the componentwise mean of their
Rodrigues vectors. That is not a valid operation. `R(a, t) == R(-a, 2pi - t)`, so every
rotation has two vector representations, and near `|rvec| = pi` the axis sign flips
between frames. A camera looking down at a +Z-up table frame sits exactly there, so the
mean drifted toward identity and the projected world came out mirrored. The previous
working agent did not average at all, which is why it never showed this.

Replaced with `avg_rotation()`: average the rotation matrices, project back onto SO(3) by
SVD. `test_geometry.py` pins it - the naive mean is off by more than 45 degrees on inputs
where the SO(3) mean stays within 1.

### Pose solver and gates

`SOLVEPNP_ITERATIVE` on four near-coplanar points is the textbook case for converging to
the wrong local minimum. Switched to `SOLVEPNP_SQPNP` (globally optimal) plus LM
refinement. Solves are now rejected rather than silently used when they put the camera
below the table or exceed a raw reprojection threshold, and rejections are reported.

Both reprojection figures are now tracked and displayed: the raw per-frame solve and the
error of the averaged pose actually in use. Previously only the latter was shown, so a
corrupted average was indistinguishable from a bad solve - which is what made this hard
to diagnose.

### Tool centre point

`joint_angles()` iterated `range(4)`, so **wrist roll never entered forward kinematics**,
and the tip was modelled as a point on the L3 centreline. A real grasp point sits off the
roll axis, so the true fingertip swings in a circle as the wrist rolls, invisibly. That is
why the tip overlay never tracked the real gripper and why `z_grasp_offset` had drifted to
-34 mm absorbing an unmodelled tool.

`config.json` now carries `tool` offsets that rotate with roll; `fk`/`ik` use them. All
default to zero, reproducing the old behaviour exactly.

### Measurement hygiene

Marker recordings now store the wrist pitch and roll they were taken at, and the UI warns
when they vary across markers. Touching four markers at four different wrist pitches gives
four differently-wrong Z values and a table plane fitted through nonsense - the likely
reason the fitted plane came out flat.

`calibrate_camera.py` now reports the RMS reprojection error it had been discarding, warns
when the board is too small for the working distance, and records the calibration
resolution in `camera_calib.json` so `vision.py` can hard-check the live stream against it.
Both ends lock MJPG.

### Corrected along the way

The old hardcoded table plane (`PLANE_A/B/C`) described a ~5 cm tilt across the workspace,
and restoring it looked like an obvious fix. It is not: the arm is now bolted flat to the
table, so the base plane and the table surface are parallel and a near-flat plane in arm
coordinates is *correct*. Those constants encoded the dumbbell-era base tilt. They are
obsolete, and `fit_plane` deriving the plane from the markers is the right design.

## Needs real hardware

Nothing below can be settled in software. In rough priority order.

### 1. Confirm the pose is actually fixed

Run the camera with all four markers visible and compare the two reprojection numbers. The
prediction is that "solve" was always small and "in use" was large - i.e. the averaging was
corrupting a good solve. If both are large instead, the fault is in the marker coordinates
or the intrinsics and the next two items become urgent.

### 2. Ground truth the marker layout

**Nothing in this system has ever been checked against a ruler.** Every length is
FK-derived, so an FK error is invisible by construction - the camera is fitted to the arm's
own distorted space and the whole loop stays self-consistent while being wrong.

Tape-measure the six pairwise marker centre distances and compare them against the stored
object points. `apply_touched()` already prints exactly those pairwise distances, so the
comparison is direct.

The stored layout is a pronounced trapezoid - near edge about 352 mm, far edge about 496
mm. The same shape is present in the original working configuration, so it is probably
real, but "probably" is doing a lot of work for the one number everything else rests on.

If the tape disagrees, suspect the servo pulse mapping (HARDWARE.md): the firmware maps
0-180 degrees onto 732-2441 microseconds, while MG996R are nominally 500-2500, so a
commanded degree may not be an actual degree.

### 3. Re-shoot the camera intrinsics

The existing calibration used a 100x70 mm board at a working distance of about a metre,
and its RMS was never recorded. Redo it with a 200-300 mm board and check the number.

### 4. Measure the tool offsets

Procedure in CALIBRATION.md step 3. Then re-record all four markers at a single fixed
wrist pitch and roll, and expect `z_grasp_offset` to fall toward zero.

### 5. The measurement campaign

Millimetre paper on the workspace, 30-40 commanded points spanning the quadrants and the
extremes, recording commanded versus actual. This is the right instrument, but it has to
come **after** the above, or it will carefully document the bugs instead of the arm.

Keep the results as the regression baseline.

## Later

- **Use all 16 ArUco corners, not 4 centroids.** The pose currently throws away 12 of the
  16 available points. A projected-corner centroid is also not the projection of the true
  centre under perspective, which is worth a few mm at this viewing angle. Each marker's
  size and in-plane rotation can be bootstrapped by raycasting its detected corners onto
  the table once a decent pose exists, then re-solving on the full 16-point model.
- **Per-pose gravity compensation.** A cheap arm sags more at long reach, so FK error is
  pose-dependent in a way no single trim can absorb. Worth characterising once the
  measurement campaign exists to characterise it against.
- **Retire the dead code.** `vision_agent.py` and `vision_config.json` are orphaned -
  nothing imports them - and `debug-8a3ee4.log` is committed instrumentation output.
  Left in place for now because the old agent is still the only reference for how the
  working configuration behaved.
- **Switch to `yolov8s-worldv2.pt`.** Downloaded but unused; `vision.py` still hardcodes
  the v1 weights.
- **Automated regression on real hardware.** `test_geometry.py` covers the maths offline.
  Nothing covers "the arm actually goes where it is told", and that gap is where every bug
  in this project has lived.
