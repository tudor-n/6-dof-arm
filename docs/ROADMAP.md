# Roadmap

## Current state

**Working, and closer to good than it has been - but not finished.**

The arm picks objects off the table reasonably reliably. Vision is trustworthy; the arm model
is calibrated but imperfect, and grasp depth is still being tuned.

| | |
|---|---|
| Camera pose | Solid. Mean reprojection ~1.8 px, camera position rock-steady |
| Marker model | Hand-measured, agrees with an independent tape check to 7 mm worst |
| Arm kinematics | All four joints calibrated. **~11 mm** tip accuracy at table height (cross-validated) |
| Grasp depth | Being tuned. Two clamps that silently pinned it at zero are now removed |
| Tool model | Along-axis length fitted. Off-axis offsets unmeasured and assumed zero |

In practice: the arm reaches the right place well enough to pick pens and pliers, and the
remaining errors are millimetres rather than centimetres. Getting from "works most of the time"
to "works every time" needs the open items below.

## Where this came from

The arm picked reliably while its base was weighted down with dumbbells. Knocking the table
destroyed that, and the base was bolted down properly. Re-measuring afterwards did not restore
accuracy: the gripper landed centimetres off in a pattern that varied across the table, and the
overlay grid came out roughly mirrored.

Two unrelated faults were compounding - one in the camera path, one in the arm model - and each
partly masked the other.

## Fixed

### Rotation averaging (the mirrored overlay)

Camera poses were averaged by taking the componentwise mean of their Rodrigues vectors. That is
not a valid operation: `R(a, t) == R(-a, 2pi - t)`, so every rotation has two vector forms, and
near `|rvec| = pi` the axis sign flips between frames. A camera looking down at a +Z-up table
sits exactly there, so the mean drifted toward identity and the world came out mirrored.

Replaced with `avg_rotation()`: average the rotation matrices, project back onto SO(3) by SVD.
Tests pin it - the naive mean is off by more than 45 degrees where the SO(3) mean stays within 1.

### Pose solver and gates

`SOLVEPNP_ITERATIVE` on four near-coplanar points is the textbook case for the wrong local
minimum. Switched to `SOLVEPNP_SQPNP` plus LM refinement. Solves are now rejected rather than
silently used when they put the camera below the table or exceed a raw reprojection threshold.

Both reprojection figures are tracked and shown: the raw per-frame solve and the error of the
averaged pose in use. Previously only the latter existed, so a corrupted average was
indistinguishable from a bad solve - which is what made this hard to find.

### Pose health

Health was judged on the **worst** marker against 3 px. With four markers that is hostage to one
of them, and geometry guarantees one always will be: markers 0 and 2 sit ~670 mm from the camera
at 63 degrees incidence while 1 and 3 are ~380 mm at 36 degrees, so the same error on the table
subtends about twice the pixels at 0 and 2. The verdict flipped between healthy and unhealthy on
alternate frames, and picks were refused at random.

Now judged on the **mean**, with a separate looser worst-marker limit to catch one genuinely
broken marker, and hysteresis so the verdict cannot oscillate. Replaying nine real samples: the
old rule changed its mind six times, the new rule none.

### The marker model was the wrong shape

Hand-measured distances between marker centres showed the arm-measured model was not merely
misplaced but geometrically wrong: the real layout is a near-rectangle (near edge 392 mm, far
edge 401, ratio 1.02) while the model described a strong trapezoid (352 and 496, ratio 1.41).

This matters more than it looks. A rectangle viewed obliquely *projects* to a trapezoid, so a
trapezoidal model of a rectangular reality gets partly absorbed by solvePnP as **camera tilt** -
giving a modest reprojection error while every table coordinate is wrong by centimetres, and
worsening across the workspace.

Replaced with Cartesian hand measurements (see CALIBRATION.md), which agree with the independent
tape distances to 7 mm worst and 2 mm on five of six pairs. The marker model no longer depends on
the arm at all.

### Servo gain and the tool centre point

`joint_angles()` iterated `range(4)`, so wrist roll never entered forward kinematics, and the tip
was modelled as a point on the L3 centreline. `config.json` now carries tool offsets that rotate
with roll, and a per-joint `gain` - degrees actually rotated per degree commanded, which is not
1.0 on this rig. See CALIBRATION.md for the measured values and HARDWARE.md for why.

### Two clamps that pinned the grasp at zero

Grasp depth would not respond to `z_grasp_offset`. It was clamped by `z_floor` at 0.0, and then
independently by `z = max(0.0, z)` inside `move_tip`. Since `z = 0` is the base plane and the
table surface sits a few mm **above** it, a clamp at zero makes grasping impossible while
appearing to work, and silently defeats both settings. Both are fixed; `Z_MIN_MM` is now a
backstop at -40 rather than a working limit.

### Settled along the way

- **The table plane was never the problem.** The constants actually in use after bolting down
  were `0.0086 / 0.0045 / 6.95`, near-flat and within 0.7 mm of the least-squares plane through
  the marker Z values - exactly what `_fit_plane()` computes. The `0.1197 / 0.0573 / -14.35` set
  committed at HEAD predates the bolting.
- **The camera intrinsics are fine.** fx and fy agree to 0.02%, the principal point is 24 px off
  centre in x and spot on in y, 70.1 degree horizontal FOV at 1920 px, zero skew.
  `intrinsics_check()` measures their adequacy empirically rather than trusting an RMS that was
  never recorded.
- **The startup skew is the base zero, not a bent wrist.** At the startup servo of 90 against a
  measured zero of 97.56 the arm points 5.77 degrees right, which is 25 mm at 250 mm of reach -
  exactly what was observed. A base yaw offset displaces every joint in proportion to its
  distance from the base, so it looks like it originates at the wrist.

## Open

### 1. Finish tuning grasp depth

Picks land slightly high. `z_grasp_offset` is genuinely in control for the first time, so this is
a matter of adjusting it a couple of mm at a time from the current -7.6.

### 2. Improve the pitch-joint calibration

The current fit cross-validates at ~11 mm, from eight marker touches. Three weaknesses, each
measurable:

- **All touches were at table height**, over 228-353 mm of reach. Anything well outside that -
  the 135 mm hover, for instance - is extrapolation. Touching each marker again with the tip
  resting on a box of known height would fix this; simulation puts it at ~6.5 mm.
- **Three of four markers were touched at wrist pitches only 4-13 degrees apart.** 25-30 degrees
  of separation is what the fit needs.
- **The wrist roll changed between batches** (69 then 180). If the grasp point sits off the roll
  axis, rolling physically moves the fingertip, so the batches do not share one tool geometry.
  Keep roll constant. Costs about 10 mm.

The fitted wrist zero, 236.7 degrees, lies outside the joint's 49.5-180 travel. The parameters
are partly degenerate even though the predictions are not - which is the distinction that matters
for control, but it means the individual numbers should not be read as physical.

### 3. Measure the off-axis tool offsets

`tool.perp` and `tool.lateral` are still zero and unmeasured. The planar fit cannot see them: it
uses only radius and height, both in the arm's plane, so an off-axis grasp point is invisible to
it. If picks are systematically off to one side in a way that changes with wrist roll, this is
the cause.

### 4. Re-shoot the camera intrinsics (low priority)

The existing calibration used a 100x70 mm board at about a metre, and its RMS was never recorded.
`intrinsics_check()` says whether this actually costs anything - under 2 mm means leave it alone.

### 5. The measurement campaign

Millimetre paper, 30-40 commanded points spanning the quadrants and the extremes, commanded
versus actual. This is verification rather than diagnosis now, and it becomes the regression
baseline.

## Later

- **Widen `SERVOMIN`/`SERVOMAX` in the firmware.** The measured gains cluster at 0.71-0.76 across
  base, shoulder and elbow, consistent with the firmware's pulse window being ~85% of what the
  MG996Rs want. Fixing it at source would remove the gains entirely. Deliberately deferred: every
  joint `min`/`max` in `config.json` is expressed in the same servo units, so widening the range
  invalidates all six limits at once and risks driving into the mechanical stops.
- **Use all 16 ArUco corners rather than 4 centroids.** Checked, and currently *not* worth it: the
  perspective bias from using the mean of four corners is 0.16 px at the far markers and 1.0 px at
  the near ones - too small, and biased toward the wrong markers, to explain anything observed.
- **Per-pose gravity compensation.** A cheap arm sags more at long reach, so FK error is
  pose-dependent in a way no single trim absorbs. Worth characterising once the measurement
  campaign exists to characterise it against.
- **Retire the dead code.** `vision_agent.py` and `vision_config.json` are orphaned; nothing
  imports them. `debug-8a3ee4.log` is committed instrumentation output, though it earned its keep
  once - the pixel coordinates in it helped confirm the calibration resolution.
- **Switch to `yolov8s-worldv2.pt`.** Downloaded but unused; `vision.py` still hardcodes v1.
- **Automated regression on real hardware.** `test_geometry.py` covers the maths offline. Nothing
  covers "the arm actually goes where it is told", and that gap is where every bug in this project
  has lived.

## Lessons worth keeping

- **Low residual is not accuracy.** A seven-parameter fit to eight observations produced 1-5 mm
  residuals and physically impossible parameters, predicting ~154 mm wrong everywhere it had not
  been measured. Cross-validate, or score predictions - never the fit itself.
- **Parameter identifiability and prediction accuracy are different things.** Degenerate
  parameters are fine if the predictions agree, and predictions are what control needs.
- **Check clamps end to end.** Grasp depth was pinned at zero by two independent clamps in
  different files. Testing each in isolation is exactly what let it hide.
- **Separate the error from the thing that reports it.** The reprojection figure shown was computed
  from the averaged pose, so a corrupted average looked identical to a bad solve. Tracking both
  numbers is what made the fault visible.
- **Measure something the system cannot see.** Every length was FK-derived, so an FK error was
  invisible by construction - the camera was fitted to the arm's own distorted space and the whole
  loop stayed self-consistent while being wrong. One tape measure broke it open.
