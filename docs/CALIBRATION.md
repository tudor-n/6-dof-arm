# Calibration

**Do these in order.** Each step is measured using the ones above it, so calibrating out
of order means measuring one error with another.

```
0. Tape-measure the markers    independent of EVERYTHING - the only external ground truth
1. Camera intrinsics           independent of the arm
2. Link lengths + joint zeros  independent of the camera
3. Tool centre point           needs 2
4. Marker positions            needs 2 and 3 (the arm measures them)
5. Table plane                 needs 4
6. Wrist-roll zero             independent
7. Verify                      needs everything
```

If accuracy is wrong and you do not know why, do not start with trims. `x_trim`,
`y_trim` and `z_grasp_offset` exist to absorb a few millimetres of residual. Using them
to absorb centimetres hides the real fault, and the compensation stops working the moment
anything moves.

---

## 0. Tape-measure the markers

Measure all six distances between marker centres by hand and put them in `vision.json`
under `tape` as `"<lo>-<hi>": mm`.

This is the only number in the system that does not come from the arm. Every other length
is FK-derived, so the camera ends up fitted to the arm's own distorted space and the whole
loop stays self-consistent while being wrong. Without the tape, an FK error is invisible by
construction.

`tape_check()` runs at startup and compares the **shape** of the stored marker model against
these distances. Pairwise distances do not change when a model is rotated or translated, so
this isolates shape error from placement error - and a shape error cannot be fixed by a trim,
an offset, or by moving the model.

It matters because solvePnP is far more sensitive to shape than to placement. A rectangle
viewed obliquely projects to a trapezoid, so a trapezoidal model of a rectangular reality
gets partly absorbed as **camera tilt**: modest reprojection error, but every table
coordinate wrong by centimetres, worsening across the workspace.

If the check fails, **Camera tab -> Marker calibration -> Rebuild from tape**. That rebuilds
the model with the measured shape, placed by rigidly fitting it onto the arm-touched points.
The reported fit residual is how far forward kinematics is out; if it is more than ~10 mm,
go do step 2 properly.

Measure the ArUco black-square edge while you are there (`aruco_mm` in `vision.json`).

**Better still**, if you can: measure each marker in Cartesian coordinates with a square
along the robot's centre line - distance along the line from the base rotation axis (X) and
perpendicular offset (Y). Eight numbers, and the marker model stops depending on the arm at
all.

---

## 1. Camera intrinsics

```
python calibrate_camera.py --camera 0 --board 10x7 --square 25
```

Capture 15-20 views, varying the angle, with the board at roughly the distance the camera
actually works at. `c` captures, `u` undoes, `q` computes.

**Board size matters more than capture count.** A board 100 mm across seen from a metre
away barely constrains focal length or distortion, and a wrong focal length cannot be
absorbed by the pose - it surfaces as irreducible reprojection error and as table
coordinates that drift with position. Aim for 200-300 mm across.

**Before re-shooting, check whether you need to.** `intrinsics_check()` in the status feed
compares two independent pixel-to-table mappings: the raycast, which goes through the camera
matrix and the solved 3D pose, and a homography fitted straight from the four marker
correspondences, which uses neither. Both share the same marker model, so that error cancels
and what remains implicates the intrinsics specifically.

Sensitivity is about **3.4 mm of disagreement per 1% of focal-length error**, so the number
pins the focal length to roughly 0.3%:

| disagreement (worst) | meaning |
|---|---|
| under 2 mm | intrinsics are fine for this workspace, whatever the calibration RMS was |
| 2 - 10 mm | a 1-3% focal error; worth re-shooting eventually, not urgent |
| over 10 mm | 3%+ out; re-shoot before chasing anything else in the camera path |

The current `camera_matrix.npy` is structurally sound: fx and fy agree to 0.02%, the
principal point is 24 px off centre in x and spot on in y, implied horizontal FOV is 70.1
degrees at 1920 px, zero skew, and k1 = -0.35 is ordinary barrel distortion. It is plainly a
1920x1080 calibration made of plausible numbers. Note though that cy came out within 0.03 px
of exactly 540, which is where OpenCV initialises it - a sign the principal point and focal
length were weakly constrained and correlated, as a small board at a single distance would
leave them. Plausible but imprecise.

Read the RMS the script prints:

| RMS | Verdict |
|---|---|
| 0.5 px or less | Good enough for millimetre work |
| 0.5 to 1.0 px | Usable; a larger board or more varied angles would help |
| above 1.0 px | Do not trust table coordinates. Recalibrate |

Writes `camera_matrix.npy`, `dist_coeffs.npy` and `camera_calib.json`. The last records
the resolution the intrinsics belong to; `vision.py` checks the live stream against it and
refuses to be quiet about a mismatch. Both ends lock MJPG so the camera cannot silently
negotiate a different mode.

**Recalibrate whenever** the lens is touched, the zoom or focus changes, or you change
capture resolution.

## 2. Link lengths and joint zeros

In `config.json`, or through the Calibrate tab's wizard.

- `L0` table surface to shoulder axis height
- `L1` shoulder axis to elbow axis
- `L2` elbow axis to wrist-pitch axis
- `L3` wrist-pitch axis to the end of link 3 (**not** the fingertip - that is step 3)

Measure between **axes of rotation**, not between visible edges of brackets. Then use the
wizard to set each joint's `zero` and `sign` and creep to its real limits.

Sanity check: command the tip to a few known points and measure where it actually goes
with a ruler. Radial error that grows with reach means a link length is off; error that
grows with yaw means a base zero or sign is off, or the servo pulse scale is (see
HARDWARE.md).

## 3. Tool centre point

The grasp point is neither at the end of L3 nor on the wrist-roll axis. Until it is
measured, the arm reports a tip it does not have, and the error *changes with wrist roll*.

In `config.json`:

```json
"tool": { "along": 0.0, "perp": 0.0, "lateral": 0.0 }
```

measured in mm from the end of L3: `along` the tool axis, `perp` to it within the arm's
plane, and `lateral` out of that plane, both at roll zero.

A practical way to find them: set the wrist to pitch -90 (straight down) and roll zero,
jog the fingertip onto a fixed mark, and compare the mark's real position with the tool
point the UI reports. The difference, decomposed along the tool triad, is the offset. Then
rotate the wrist 180 degrees and repeat: if the reported point moves, the shift is twice
the off-axis component, which separates `perp` and `lateral` from `along`.

With all three at zero the behaviour is identical to the old centreline model, so it is
safe to leave them until you are ready to measure.

`z_grasp_offset` in `vision.json` should fall toward zero once the tool is modelled. If it
is still tens of millimetres, the tool offset is still absorbing the error.

## 4. Marker positions

**Re-run this whenever the base moves or is knocked.** The oversized mounting holes mean a
bump can shift the base, and every marker coordinate is relative to it.

In the Camera tab, under Marker calibration, for each of IDs 0-3:

1. Turn Live on.
2. Jog the tool point onto the marker's centre and down onto the table surface.
3. Wait for the arm to settle, then press **Record**.
4. **Use the same wrist pitch and roll for all four.** This is not a nicety: the recorded
   point is only as good as the tool model at that pose, so four markers touched at four
   different wrist pitches give four differently-wrong Z values and a table plane fitted
   through nonsense. The UI shows the pitch and roll each marker was recorded at, and
   warns when they vary.

Then press **Apply**. It reports the pairwise distances between markers against the
previous layout. Large unexplained changes mean something moved - or that a recording was
taken at the wrong point.

You can record only two or three and let the rigid fit deduce the rest, but four measured
markers is always better.

## 5. Table plane

Currently fitted automatically from the four marker Z values when you apply marker
calibration, so step 4 produces it.

Note that the arm is bolted flat to the table, so the base plane and the table surface are
parallel and a **near-flat plane in arm coordinates is the expected result** - even though
the desk itself slopes relative to the room. A fitted plane with a large tilt means either
the base is not sitting flat or the Z values are wrong.

## 6. Wrist-roll zero

In the Camera tab: **Roll zero** records the servo value at which the jaws close along the
arm's own direction, and **Flip** reverses the sign if the jaws rotate the wrong way.
`world_to_servo_roll()` uses these to turn a detected object's jaw-closing angle into a
roll servo value, choosing the candidate within the joint limits closest to the zero.

## 7. Verify

In order of increasing commitment:

1. **Pose health.** All four markers visible, badge green. Compare the two reprojection
   figures in the status line - "solve" is the raw per-frame fit, "in use" is the averaged
   pose driving the arm. Both should be under about 1.5 px.
2. **Grid.** The overlay grid should lie on the real table, `Y0` running down the robot's
   centre line, labels where you expect them. A mirrored or skewed grid is a pose fault,
   not a trim problem.
3. **Yellow marker lines** should collapse to nearly nothing. They run from each detected
   marker to where its stored coordinates say it should be.
4. **Click accuracy.** Click each marker centre in the video; the reported X/Y should match
   that marker's stored coordinates within a few mm. This closes the loop without moving
   the arm, so do it before trusting anything to move.
5. **Tip overlay.** With Live on, jog around the workspace and watch the cyan cross track
   the real gripper - at varying reach *and varying wrist roll*. The roll case is the one
   that catches a wrong tool offset.
6. **Picks.** Pens and pliers at eight or more positions spanning all four quadrants and
   the extremes of the workspace.

---

## Symptom to cause

| Symptom | Look at |
|---|---|
| Overlay grid mirrored or wildly skewed | Camera pose. Check the log for rejected solves |
| "solve" small, "in use" large | The pose average is being corrupted - a regression in `avg_rotation` |
| Both reprojection figures large | Marker coordinates or camera intrinsics disagree with reality |
| Pose never found | Not all four markers visible; check with `python test_markers.py` |
| Error grows smoothly across the table | Table plane, or the camera intrinsics |
| Error changes when the wrist rolls | Tool centre point is unmodelled or wrong |
| Error grows with reach | Link lengths |
| Error grows with yaw | Base joint zero or sign, or servo pulse scale (see HARDWARE.md) |
| Everything was fine, then suddenly was not | The base was knocked. Re-run step 4 |
| Marker model fails the tape check | FK is distorted. Rebuild from tape, then fix step 2 |
| Grid skewed but reprojection looks acceptable | Classic wrong-marker-shape signature: the solver absorbed it as camera tilt. Run step 0 |
| Reported resolution differs from calibration | Camera negotiated another mode; re-run step 1 |
