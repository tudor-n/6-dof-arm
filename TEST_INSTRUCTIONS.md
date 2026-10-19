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

## 1. Rebuild the marker model from the tape  **(do this first)**

The tape measurements are in and they changed the plan. Your arm-measured marker model is
the wrong **shape**, not just misplaced:

| pair | tape | model | diff |
|---|---|---|---|
| 0-1 (near edge) | 392 | 351.9 | **-40.1** |
| 0-2 | 152 | 188.5 | +36.5 |
| 0-3 | 420 | 451.6 | +31.6 |
| 1-2 | 436 | 472.2 | +36.2 |
| 1-3 | 174 | 208.6 | +34.6 |
| 2-3 (far edge) | 401 | 495.6 | **+94.6** |

Your real layout is a near-rectangle (392 vs 401, ratio 1.02). The model thinks it is a
strong trapezoid (352 vs 496, ratio 1.41). Your six measurements close as a planar
quadrilateral to 1.0 mm, so the tape is sound and the arm is not.

These are already saved in `vision.json`, so **the server will complain loudly at startup**
until you rebuild. That is expected, not a fault.

**Camera tab -> Marker calibration -> Rebuild from tape.** You should see:

```
tape shape closes to 1.0 mm | placed onto the arm-measured points with 38.3 mm RMS
residual (per marker: 0:24, 1:22, 2:50, 3:48 mm)
```

Afterwards the tape-vs-model readout should show every pair within about 1 mm.

**What this does and does not fix.** The model now has the correct *shape*, so the camera
pose becomes trustworthy - solvePnP is far more sensitive to shape than to placement. The
*placement* still comes from the arm, so expect a residual systematic offset of a couple of
cm until forward kinematics is fixed. Judge the result on whether the grid is square and
un-mirrored and whether clicking is self-consistent, not on absolute accuracy yet.

The 38 mm residual - 50 mm at the far markers against 22 mm at the near ones - is the real
headline: **FK error grows with reach.** `docs/ROADMAP.md` section 2 has the candidates and
why neither a pure reach error nor a pure yaw error explains it on its own.

### The eight numbers that would finish this

Think of the table as graph paper with the **robot's base at (0, 0)** and the robot looking
straight down the X axis. I need each marker's X and Y in mm. That is it - four markers, two
numbers each.

```
                    the robot looks this way
                              |
                   +X  (forward, away from you)
                              ^
                              |
          marker 2  *         |         *  marker 3
                    :         |         :
                    :     Y2  |  Y3     :
                    :<------- | ------->:
          marker 0  *         |         *  marker 1
                    :         |         :
                    :         | X0      :
                    :         |         :
     +Y  <--------------------O-------------------->  -Y
      (robot's LEFT)     base axis            (robot's RIGHT)
```

**How to measure it**

1. Send the base to its zero (servo 90) so the arm points straight ahead. Lay a tape measure
   or a taut string flat on the table, starting under the **centre of the base's rotation**
   and running straight out the way the arm is pointing. That string is your X axis.
2. For each marker, two readings:
   - **Y** - the straight-across distance from the string to the marker's centre, at right
     angles to the string. Use a book corner or a set square so it really is at right angles.
     **Positive if the marker is on the robot's LEFT, negative if on its RIGHT.**
   - **X** - read off the string at the point where that right-angle line meets it.

So for your layout I am expecting roughly: markers 0 and 2 with **positive** Y, markers 1 and
3 with **negative** Y, and markers 2 and 3 with the larger X since they sit further out.

**Two things that would spoil it**

- The origin is the base's **axis of rotation** - the centre of the plate that turns - not the
  corner or edge of the base plate. If the base plate is 80 mm across, the edge is 40 mm from
  where I need you to start.
- Measure to the **centre of each marker square**, the same point the arm was told to touch.

**Why this finishes it.** Your six tape distances fixed the markers' shape relative to each
other, but said nothing about where that shape sits relative to the robot - which is why the
rebuild still leans on the arm for placement and still carries a couple of cm of offset.
These eight numbers put the shape in the robot's own frame using only a tape measure, so the
marker model stops depending on forward kinematics entirely. The camera pose then becomes
trustworthy in absolute terms, and whatever error is left is purely the arm not going where
it is told - which is directly measurable against a reference you trust.

**If the right angles are awkward**, there is a fallback: measure the four straight-line
distances from the base rotation axis to each marker centre. With the six you already gave,
that fixes everything except which direction counts as straight ahead - so I would need one
extra angular reference, such as the perpendicular offset of any single marker, or where the
string passes between two of them. Same effort in practice, more ways to go wrong, so prefer
the X/Y method above.

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
   **Expect it to still be off** - see Gotcha 4. Note *how* it is off: a constant offset
   means the tool offset needs measuring, while drift with reach means link lengths.
2. **Roll the wrist and watch the cross.** It will not move, because the tool offsets are
   still zero. Measure how far the *real* fingertip moves - that distance is twice your
   off-axis tool offset, and it is the number you need for step 4.
3. Click a point, **Go to hover**. The arm should arrive above where you clicked.
4. Only then try a pick, starting near the centre of the workspace where error was
   smallest.

---

## 4. Measure the tool (the remaining known gap)

This branch makes the tool offset *expressible*; it does not know yours. Until you measure
it, the tip overlay stays wrong and `z_grasp_offset = -34` keeps absorbing it.

Procedure is in `docs/CALIBRATION.md` step 3. Short version: pitch -90, roll zero, jog the
fingertip onto a mark, compare with the reported tool point; then roll 180° and repeat to
separate the off-axis part. Put the numbers in `config.json` under `tool`.

Then re-record all four markers **at one fixed wrist pitch and roll**, press Apply, and
expect `z_grasp_offset` to fall toward zero.

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

### 4. The tip overlay is NOT fixed yet

Tool offsets default to zero, which reproduces the old centreline tip exactly. This was
deliberate so the branch changes nothing until you choose to. The overlay will keep not
tracking the real gripper until you do step 4. Do not read that as the branch having failed.

### 5. Leave the trims alone

`x_trim`, `y_trim` and `z_grasp_offset` are currently absorbing real errors. Resist
zeroing them "to see" before the tool is measured, and resist tuning them to chase
whatever error remains after this branch. They are for a few mm of residual. Using them for
centimetres is what made the last failure so hard to unpick.

### 6. You will see a warning about `camera_calib.json` on first run

Expected. That file does not exist yet - it is written by the new `calibrate_camera.py`.
Until then `vision.py` cannot verify that the stream resolution matches your intrinsics and
says so. Harmless, and it goes away when you recalibrate.

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

## What to report back

With these I can tell you immediately whether the diagnosis was right and what is next:

1. The `Rebuild from tape` output line, and the tape-vs-model readout afterwards.
2. The status line from step 2, verbatim - both reprojection figures and the camera position.
3. Whether the grid is still mirrored.
4. Click accuracy at each of the four markers (clicked vs. stored coordinates).
5. Any `rejected` lines from the log, verbatim.
6. The logged `Camera running at WxH`.
7. The `intrinsics_check` figure from the status feed - it says whether your existing
   `camera_matrix.npy` needs re-shooting at all (under 2 mm: leave it alone).
