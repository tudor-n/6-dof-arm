# Testing branch `fix/vision-registration`

Read the **Gotchas** section before you start. Several of these changes make the system
*refuse to do things it used to do wrong*, which looks like a regression if you are not
expecting it.

Revert at any time with `git checkout master` - nothing here touches `master`.

---

## 0. Before touching hardware (2 minutes)

```
cd D:\RobotArmControl
git checkout fix/vision-registration
py -3.14 test_geometry.py
```

Expect `21/21 passed`. Must be run from the repo root - the tests load `vision.json` and
`camera_matrix.npy` from the working directory.

If `python` is not found, use `py -3.14`. Bare `python` hits the Windows App Execution
Alias on this machine and fails with a Microsoft Store message.

Then a dry run with no hardware at all:

```
py -3.14 server.py --sim --no-camera --http 8099
```

Open <http://localhost:8099>, confirm the Move and Calibrate tabs still work and the arm
drawing follows the drag handles. Ctrl-C when done. This isolates "did the kinematics
changes break the UI" from anything camera-related.

---

## 1. What is already calibrated  **(read this first)**

Earlier versions of this file told you to measure the base gain and then the pitch joints.
Both are done. All four joints now carry a measured zero and gain:

| joint | zero | gain |
|---|---|---|
| base | 97.56 | 0.7635 |
| shoulder | 66.74 | 0.7109 |
| elbow | 81.98 | 0.7577 |
| wrist pitch | 236.72 | 0.6307 |

Tip accuracy, by leave-one-out cross-validation on the real marker touches: **median 11.3 mm,
worst 17.3 mm**, against 29.2 / 52.0 mm uncalibrated. Valid at table height over 228-353 mm of
reach - the 135 mm hover is extrapolation.

The marker model is hand-measured in Cartesian coordinates and involves the arm nowhere.
`tool.along` is fitted at 26.4 mm; `tool.perp` and `tool.lateral` are still zero and unmeasured.

Full detail, including which hypotheses were ruled out and why, is in `docs/ROADMAP.md`.
Measured values and their provenance are in `docs/CALIBRATION.md`.

### What to do next

In order of value:

1. **Finish tuning grasp depth.** `z_grasp_offset` in `vision.json` is at -7.6, giving a grasp
   about 2.45 mm below the table plane. It is genuinely in control for the first time - two
   separate clamps used to pin it at zero - so change it a couple of mm at a time and it will do
   what you expect.
2. **Re-touch the markers properly** to improve the pitch fit from ~11 mm toward ~6 mm. Two
   touches per marker at wrist pitches **25-30 degrees apart**, with the **roll kept constant**,
   and ideally a third with the tip resting on a box of known height. Format them as
   `marker_poses` in `servo_cal.json` and run `fit_servo_calibration.py`.
3. **Measure the off-axis tool offsets** if picks drift to one side in a way that changes with
   wrist roll. The planar fit is blind to these by construction.


## 2. Camera, no arm

```
py -3.14 server.py --sim --camera 0
```

Open the **Camera** tab. Get all four markers in frame.

**Read the status line. This is the main event.** It now shows two reprojection figures:

```
worst 1.2px  mean 0.8px  (solve 0.7px)
```

- `solve` = the raw per-frame solvePnP fit
- `worst` / `mean` = the error of the averaged pose actually driving the arm

| What you see | Meaning |
|---|---|
| `solve` small, `worst` small | **Fixed.** The averaging was corrupting a good solve, exactly as diagnosed |
| `solve` small, `worst` still large | The SO(3) averaging did not fully resolve it - send me the numbers |
| Both large (>4 px) | The pose was never the only problem. Marker coordinates or intrinsics are wrong; go do step 1 and step 4 |
| No pose at all, log says "rejected" | See Gotcha 1 below - this is the big one |

Then, without moving the arm:

1. **Grid.** Should lie on the real table, `Y0` down the robot's centre line, `X` increasing
   away from you. **It must not be mirrored.** That was the headline symptom.
2. **Yellow lines** from each marker to where its stored coordinates say it should be -
   these should be nearly zero length.
3. **Camera position.** The status line now reports the recovered camera position in arm
   coordinates. Sanity-check it against where the camera physically hangs. It should read
   roughly "robot's right, high up" - negative Y, Z of several hundred mm. A wildly wrong
   number here means the pose is wrong even if reprojection looks fine.
4. **Click each marker centre.** The reported X/Y should match that marker's stored
   coordinates within a few mm. **This is the real test** - it closes the whole camera →
   table → arm-coordinate loop without anything moving.

If step 4 passes at all four markers, the registration is fixed. If it passes at some and
not others, note *which* and by how much - the pattern is diagnostic.

---

## 3. Arm, carefully

```
py -3.14 server.py --port COM5 --camera 0
```

The arm boots limp. Nothing moves until **Live** is on. **STOP** or **Esc** cancels a pick
and brakes the gripper - know where it is before you turn Live on.

1. Live on, jog around the workspace, watch the cyan tip cross against the real gripper.
   It should track reasonably well at table height. Note *how* any error behaves: a constant
   offset points at the tool model, drift with reach points at the joint gains, and sideways
   error that changes with wrist roll points at the unmeasured off-axis tool offsets.
2. **Roll the wrist and watch the cross.** It will not move, because `tool.perp` and
   `tool.lateral` are still zero. Measure how far the *real* fingertip moves - half that
   distance is your off-axis offset, and it is the number step 4 needs.
3. Click a point, **Go to hover**. The arm should arrive above where you clicked. Height is
   the weakest axis, since every calibration touch was taken on the table.
4. Then try a pick, starting near the centre of the workspace.

---

## 4. The remaining measurements

**Off-axis tool offsets.** `tool.perp` and `tool.lateral` are zero and unmeasured, and the
planar fit cannot see them - it uses only radius and height, both in the arm's plane. Procedure
in `docs/CALIBRATION.md` step 3: pitch -90, roll zero, jog the fingertip onto a mark, compare
with the reported tool point, then roll 180 degrees and repeat to separate the off-axis part.

**A better pitch fit.** The current one cross-validates at ~11 mm from eight touches taken at
table height. Two touches per marker at wrist pitches **25-30 degrees apart**, roll held
**constant**, plus a third with the tip on a box of known height, should get it near 6 mm. Put
them in `servo_cal.json` as `marker_poses` and run `fit_servo_calibration.py` - it will refuse
the fit and tell you why if the poses are too alike.

**Note** `tool.along` is already fitted at 26.4 mm, lumped out of the effective link length. A
touch cannot separate "the forearm is longer" from "the tool sticks out further", so if you
measure the tool physically, expect to adjust both.

---

## 5. Finger follow (branch `feat/finger-follow`)

With **Follow finger** on, the arm hovers over the spot your index finger points at and follows
it around the table. Only a *pointing* hand moves it: index finger straight, the other fingers
curled. An open hand, a fist or a hand holding something is ignored.

### 5a. Install and offline checks (no hardware)

MediaPipe is optional, like ultralytics. Install it, but check it hasn't swapped out your
OpenCV, because the ArUco pose needs the `opencv-contrib` build:

```
py -3.14 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
py -3.14 -m pip install mediapipe
py -3.14 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
```

Both lines must print the **same version** and `True`. If they don't, pip replaced OpenCV: run
`py -3.14 -m pip install --force-reinstall opencv-contrib-python`, then reinstall mediapipe with
`--no-deps`.

```
py -3.14 test_finger.py
py -3.14 test_geometry.py
```

Both should pass. Then:

```
py -3.14 server.py --sim --no-camera --http 8099
```

On the Camera tab, tap **Follow finger**. It should **refuse** and stay off, with a log line
saying why (no camera, so no pose). That's correct.

The first time follow switches on for real, it downloads its hand model (a few MB) into
`weights/`, so that first time needs internet. If the machine is offline, copy
`hand_landmarker.task` into `weights/` by hand (the download address is in `finger.py`).

### 5b. On the arm

Keep a hand near STOP the whole time. **Esc** works too.

1. Start the server with the arm and camera as in section 3. Check the pose chip says
   **POSE OK**. Turn **Live** on and set the speed to **20** (the lowest).
2. On the Camera tab, tap **Follow finger** (or press **F** on a keyboard; F turns follow on only from the Camera tab, but off from anywhere). The button turns
   white and the line under it says `FOLLOW: show a pointing finger`. Nothing moves yet.
   - If it stays off, open the log: it's refused when Live is off, the pose isn't healthy, a
     pick is running or MediaPipe isn't installed.
3. Point at the middle of the table, with your fingertip just above the surface (a centimetre
   or two: that's the height the aim assumes, see 5c). Within a moment the button turns **amber** (arming), then **green** (tracking), and
   the arm rises to its hover height (110 mm) and moves over a spot **just ahead** of your
   fingertip (30 mm, so it doesn't hide your finger from the camera).
   - On the video you should see your hand's outline, a ring on the fingertip and a cross where
     the arm is aiming.
4. Move your hand **slowly** left and right. The arm should follow smoothly, without buzzing
   when your hand is still. Once that works, try speed **60**.
5. Point **past the far edge** of what the arm can reach. It should go as far as it can in that
   direction and stop. The status line says it's at the reach limit.
6. Point at a spot, then **curl your finger**. The arm should stay where you were pointing, not
   jump to where the curl moved your fingertip. The button turns amber (holding). Point again
   and it carries on.
7. Pull your hand **out of view**. The arm should hold still.
   - While it holds, tap **Low**, **Hover** and **High** under Follow finger. The arm should sink
     or rise in place to 70, 110 and 160 mm, and the tapped button turns white. The status line
     shows the height.
   - **Hold** High: the arm keeps rising (about 4 cm a second) until you let go or it reaches
     200 mm. Hold Low: it sinks, down to 50 mm. Letting go stops it within half a second.
   - Height buttons do **not** switch follow off.
8. Check that follow switches **off** (button goes back to plain) each time you:
   - press **STOP**;
   - press **Esc**;
   - drag a handle on the Move tab;
   - tap a jog button;
   - turn **Live** off.

   After each one, switch follow back on for the next.

### 5c. If the aim is off

The camera can't tell how high your finger is, so it assumes it's 15 mm above the table. If you
hold it higher, the arm aims a bit **farther from the camera** than you meant. That's expected.

If the arm is **consistently** off along the line towards or away from the camera, change
**Finger height above table** in Camera → Setup → Finger follow:
- the arm lands too far from the camera: raise it;
- the arm lands too close to the camera: lower it.

Sideways error doesn't come from this, so look at the pose and calibration for that.

The other settings on that page:

| Setting | Default | What it does |
|---|---|---|
| Hover height above table | 110 mm | How high the arm hovers. Kept between 50 and 200. The Low / Hover / High buttons change it too |
| Aim ahead of the fingertip | 30 mm | How far in front of your fingertip it aims |
| Deadband | 3 mm | Hand movement smaller than this is ignored. Raise it if the arm buzzes |
| Follow tool pitch | blank | The wrist angle follow prefers. Blank = the angle it had when follow started. It tilts away from this only as far as it must to reach where you point |

Worth noting if something goes wrong: the status line text, the button colour, and whether the
hand outline on the video was there at the time.

---


## Gotchas

### 1. The pose can now be refused outright instead of being silently wrong

This is the change most likely to surprise you. Previously any `solvePnP` result was
averaged in and used. Now a solve is rejected if it puts the camera below the table or if
raw reprojection exceeds **4 px**, and the log says why.

**So if your marker coordinates or intrinsics are bad, you may now get *no* pose where
before you got a confidently wrong one.** That is the intended behaviour, but it will feel
like a step backwards.

To diagnose rather than be blocked, temporarily raise the threshold in `vision.py`:

```python
MAX_SOLVE_REPROJ_PX = 4.0    # raise to 20.0 to see what the solve is actually producing
```

With it raised, read the `solve` figure. That number tells you how badly the marker model
and the intrinsics disagree with reality. Put it back afterwards.

### 2. Picking is refused when pose health is bad

`MAX_POSE_REPROJ_PX = 3.0` in `vision.py`. Clicking and **Go to hover** only warn, because
probing a suspect pose is how you debug it - but the descending, gripper-closing pick
sequence refuses. If you deliberately want to pick through a bad pose, raise that constant;
do not silence the warning.

### 3. MJPG is now forced, and that can change your resolution

Both `vision.py` and `calibrate_camera.py` now set `CAP_PROP_FOURCC` to MJPG before
requesting a size. This fixes cameras that quietly negotiate a different mode than the one
calibrated - but if your camera does not do MJPG at 1920x1080, **you may get a different
resolution than you had before, which invalidates your current intrinsics.**

Check the log line `Camera running at WxH` on startup. If it is not what you used to get,
re-run `calibrate_camera.py` at the new resolution.

### 4. The tip overlay is better but not perfect

The cross is the **tool point**, and `tool.along` is now fitted, so it should sit near the grasp
point rather than at the base of the gripper. Expect roughly 11 mm of error at table height and
more above it, since every calibration touch was taken on the table.

`tool.perp` and `tool.lateral` are still zero. If the cross is consistently off to one side, and
that side changes when you roll the wrist, those are the cause - and the planar fit cannot see
them.


### 5. The trims are now fair game, within reason

`z_grasp_offset` went from -34 to -7.6 once the tool length was modelled, which is it returning
to its intended job of trimming a few millimetres. Adjusting it by a millimetre or two is now the
right move rather than a workaround.

`x_trim` and `y_trim` are still zero and should stay that way. If picks are systematically off in
X or Y, that is a model error with a cause worth finding, not a few mm of residual.


### 6. `camera_calib.json` exists now, and records a RECONSTRUCTED resolution

It holds the provenance of the existing intrinsics, not the intrinsics themselves - those are
still `camera_matrix.npy` and `dist_coeffs.npy`. The 1920x1080 in it was inferred rather than
recorded: the principal point only sits near centre for that mode, and detections logged in
`debug-8a3ee4.log` reach x=1631, which rules out anything smaller.

`rms` is null because the old calibration script discarded it. That is not worth worrying about -
watch the `intrinsics_check` figure instead, which measures the same thing better. Under 2 mm
means the intrinsics are fine for this workspace.


### 7. `fk()` now returns six points, not five

The chain is `[base, shoulder, elbow, wrist, L3 tip, tool point]` and **the tool point is
last**. Anything of yours that reads `pts_tgt[4]` is now reading the L3 tip rather than the
tool point. `index.html` was updated to use the last element; with zero tool offsets the two
coincide, so nothing changes until you set offsets. Once you do, the top-view drag handle
moves the *tool point*.

### 8. Marker recordings changed shape

`touched` entries are now `[x, y, z, tool_pitch, roll_servo]`. Your existing 3-element
entries still load fine, but the pitch/roll consistency warning can only see newly recorded
ones. Also note **Apply writes `vision.json` immediately** - it is not a preview.

### 9. Port already in use

If `--http 8080` fails to bind, an old `server.py` is still running. On Windows:

```
Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -like '*server.py*' }
```

---

## If something looks wrong

Worth capturing before changing anything:

1. The status line verbatim - both reprojection figures, the camera position, and the
   `off-by-mm` per-marker offsets.
2. Whether the overlay grid lies on the real table and is not mirrored.
3. Click accuracy at each marker: clicked coordinates against the stored ones.
4. Any `rejected`, `UNHEALTHY` or clamp lines from the log.
5. For a pick that misses: where the cross sat, where the gripper actually went, and whether the
   miss was in reach, height or sideways. Those three point at different joints.
