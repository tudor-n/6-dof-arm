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

**Settled: the table plane was never the problem.** The post-bolting constants actually in
use were `PLANE_A/B/C = 0.0086 / 0.0045 / 6.95`, which is near-flat and sits within 0.7 mm of
the least-squares plane through the marker Z values - i.e. exactly what `_fit_plane()` now
computes. The plane has agreed with the markers all along.

The `0.1197 / 0.0573 / -14.35` set committed at HEAD is from *before* the base was bolted
down, and described a ~5 cm tilt across the workspace. Restoring it looked like an obvious
fix. It is not.

That plane does not pass through the marker coordinates it shipped alongside:

| marker | stored Z | old plane | disagreement |
|---|---|---|---|
| ID0 | 7.4 | 0.6 | 6.8 mm |
| ID1 | 7.2 | -18.5 | **25.7 mm** |
| ID2 | 10.4 | 25.5 | **-15.1 mm** |
| ID3 | 7.5 | 1.4 | 6.1 mm |

The markers are physically on the table. A faithful measurement of that surface would pass
through them within a millimetre or two; this one misses by up to 26 mm. So it was never a
measurement of the table - it was a correction surface, tuned until picks landed, absorbing
camera-pose and forward-kinematics error along the way. That also explains why it stopped
working the moment anything moved: it was compensating for errors specific to one setup.

Separately, a near-flat plane in arm coordinates is what the geometry predicts anyway. The
arm's +Z is normal to its base plate, not vertical, so bolting the base to the sloping desk
tilts the whole coordinate frame with the desk - and the markers sit on that same surface.
The desk's slope is real but does not appear in these coordinates; it matters for gravity,
not for geometry. The base is confirmed bolted parallel and flat to the desk, carrying the
same slope, so this holds - see HARDWARE.md.

Deriving the plane from the markers is therefore the right design, and the old constants
are obsolete.

## Needs real hardware

Nothing below can be settled in software. In rough priority order.

### 1. Confirm the pose is actually fixed

Run the camera with all four markers visible and compare the two reprojection numbers. The
prediction is that "solve" was always small and "in use" was large - i.e. the averaging was
corrupting a good solve. If both are large instead, the fault is in the marker coordinates
or the intrinsics and the next two items become urgent.

### 2. Fix forward kinematics  **(measured 2026-10-19 - FK is distorted)**

The marker layout has now been tape-measured, and the arm-measured model is the wrong
**shape**, not merely misplaced:

| pair | tape | arm-measured model | diff |
|---|---|---|---|
| 0-1 (near edge) | 392 | 351.9 | **-40.1** |
| 0-2 (left side) | 152 | 188.5 | +36.5 |
| 0-3 (diagonal) | 420 | 451.6 | +31.6 |
| 1-2 (diagonal) | 436 | 472.2 | +36.2 |
| 1-3 (right side) | 174 | 208.6 | +34.6 |
| 2-3 (far edge) | 401 | 495.6 | **+94.6** |

The real layout is a near-rectangle: near edge 392, far edge 401, ratio **1.02**. The
arm-measured model is a strong trapezoid: 352 and 496, ratio **1.41**. The tape closes as a
planar quadrilateral to 1.0 mm, so the measurements themselves are sound.

Pairwise distances are invariant under rotation and translation, so no placement, trim or
offset can reconcile these. Best-fit rigid placement of the true shape onto the measured
points leaves **38 mm RMS** - 50 mm at the far markers, 22 mm at the near ones - so the
distortion **grows with reach**. Allowing a uniform scale only improves it to 34 mm, which
rules out a simple scale error.

Note what this does to the camera. A rectangle viewed obliquely projects to a trapezoid, so
handing solvePnP a trapezoidal model of a rectangular reality lets the solver absorb much of
the error as **camera tilt**. That produces a modest reprojection figure while throwing every
raycast off by centimetres, with the error growing across the table. It explains the original
symptom pattern better than the rotation-averaging bug alone, and the two were compounding.

**Mitigation already in place:** `markers_from_tape()` rebuilds the model with the measured
shape, placed by rigidly fitting it onto the touched points. That makes the camera pose
trustworthy, because solvePnP is far more sensitive to shape than to placement. The placement
is still FK-derived, so expect a residual systematic offset of a few cm until the root cause
is fixed. `tape_check()` now guards this permanently and complains at startup.

**Root cause, still open.** Neither a pure radial error nor a pure yaw error fits the data:

| marker | FK reach r | FK yaw |
|---|---|---|
| 0 | 178.7 | +72.85° |
| 1 | 185.2 | -77.68° |
| 2 | 337.6 | +49.02° |
| 3 | 339.3 | -45.12° |

Solving for the reach each edge implies, holding the FK yaw angles fixed:

| edge | FK reach | reach the tape implies | FK error |
|---|---|---|---|
| near (0-1) | ~182 | ~203 | **11% too short** |
| far (2-3) | ~338 | ~274 | **19% too long** |

**The error changes sign with reach**, crossing zero somewhere near r = 250 mm. That is the
most informative thing in the dataset, because it discriminates between the candidates:

- A **link-length** error contributes `eps * L * cos(a)` to reach. Across the forward working
  range those cosines keep their sign, so a wrong link length scales reach monotonically - it
  cannot overestimate at long reach while underestimating at short reach.
- A **joint-zero** error contributes roughly `-L * sin(a) * delta`. Between a folded pose (the
  near markers) and an extended one (the far markers) `sin(a)` changes substantially, so a zero
  error naturally produces an error that varies in magnitude *and flips sign* with reach.

So the sign flip points at the **shoulder and elbow zeros** rather than the link lengths.
Re-measure those first; it is also the cheaper measurement. Revised order:

1. **Shoulder/elbow zeros.** Favoured by the sign flip above.
2. **Servo pulse scale** (HARDWARE.md): 0-180 degrees maps to 732-2441 microseconds while
   MG996R are nominally 500-2500, so a commanded degree may not be an actual degree. This
   acts as a gain on every joint angle, so it too varies with pose.
3. **Gravity sag at extension**, inherently reach-dependent, and a cheap arm has plenty. It
   would push the far markers outward when recorded, which matches the sign.
4. **Link lengths**, measured between brackets rather than between axes of rotation. Still
   worth checking, but demoted: alone it cannot produce the sign flip.
5. **Tool offset**, still unmeasured, contaminating each touch differently by wrist pitch.

These are not mutually exclusive and would partly mask one another, which is why the
Cartesian measurement below is worth more than any further inference from four points.

Settling this needs the direct measurement below.

### 2c. Two separate joint faults, not one  **(revised 2026-10-19)**

An earlier version of this file claimed a single shared servo gain explained everything. That
was too strong, and a direct observation contradicts it: at the pose the model calls all-zero
(arm straight up) **the real arm leans slightly forward**, and it does so on a flat surface as
well as on the bolted slanted desk. Checking both surfaces rules out the desk, so this is the
arm's own zero calibration. A gain error cannot produce it - at zero commanded angle a gain
error contributes nothing, so a lean at that pose is positive evidence of a ZERO error.

The two faults are separable by their signatures, and the base yaw data settles that one:

| marker | FK yaw | true yaw | difference | ratio |
|---|---|---|---|---|
| ID0 | +72.85 | +54.62 | +18.23 | 1.334 |
| ID1 | -77.68 | -61.37 | -16.31 | 1.266 |
| ID2 | +49.02 | +35.17 | +13.85 | 1.394 |
| ID3 | -45.12 | -34.71 | -10.41 | 1.300 |

The differences **flip sign with the angle** while the ratios hold steady at 1.32 (spread
3.5%). A zero offset would add the same signed amount to every yaw; it does not. So:

- **Base yaw: a gain error**, confirmed multiplicative. FK over-reports by about a third.
- **Pitch joints: at least a zero error**, confirmed by the forward lean. Probably a gain
  error too, since they share the pulse mapping, but four marker positions cannot separate
  the two in reach - both produce pose-dependent reach error, which is why the earlier
  single-cause reading looked sufficient.

This is why `config.json` carries **both** `zero` and `gain` per joint, and why calibrating
only one of them would leave the arm wrong. `fit_servo_calibration.py` fits both at once from
measured angles.

### 2b. Measure the markers in Cartesian coordinates  **(done 2026-10-19)**

Measured and installed: ID0 (134.2, +189), ID1 (109.2, -200), ID2 (285.2, +201),
ID3 (290.2, -201), from perpendicular offsets to a drawn centre line plus distances along it
and a 33.2 mm datum correction to the rotation axis. These agree with the six independent
tape distances to 7 mm worst and 2 mm on five of six pairs, so the marker model no longer
depends on the arm at all and the camera pose is now trustworthy in absolute terms.

The remaining uncertainty is the X datum: solving for the offset that best fits the yaw-gain
story prefers 55 mm over the measured 33.2 mm, but the fit barely improves (spread 0.0454
against 0.0470), so the method cannot distinguish them. The practical consequence is that the
base gain is pinned only to the range 1.30-1.42. Measuring the gain directly on the servo
settles it and does not depend on the datum at all.

Historical note - the tape fixed the shape but not where it sat relative to the base, which is
why rebuilding from tape alone still carried the FK placement error. Eight numbers remove the arm from the
loop entirely: with a square along the robot's centre line, measure each marker's distance
**along** that line from the base rotation axis (X) and its **perpendicular** offset (Y).

That makes the marker model pure ground truth, the camera pose fully independent of FK, and
turns the remaining arm pointing error into something directly measurable - commanded versus
actual, with a known-good reference to compare against.

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
