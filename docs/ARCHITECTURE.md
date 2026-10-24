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
│   follow (thread) ─────────► finger.py                   │
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

## Finger follow

With **Follow finger** on, the tool hovers over the spot your index finger points at and
follows it across the table. It's for quick positioning, not precision. Most of the design is
about the arm never doing anything you didn't mean.

**finger.py knows nothing about the arm.** It has two halves, and only reports table coordinates.

- `HandTracker` is a daemon thread that runs only while follow is on. It reads `vision.frame`
  (already undistorted), runs MediaPipe's hand landmarker on a 640-px copy, and decides whether
  a hand is *pointing*: index straight, the other three curled, thumb ignored. That test uses
  MediaPipe's 3D *world* landmarks, which are metric and don't depend on the view, so the
  oblique overhead camera doesn't bend the answer. Only a pointing hand counts, so an open palm,
  a fist or a hand holding a tool never drives the arm. The fingertip pixel then goes through
  `vision.pixel_to_table()` like every other table coordinate. The tracker never raises into its
  caller: failures are logged and the observation goes `ok=False`. MediaPipe and its model file
  (`weights/hand_landmarker.task`, downloaded on first use) are optional, like ultralytics.
- `FollowController` is a pure state machine (`idle → arming → tracking → holding`) stepped by
  the control loop. It's pure so it can be tested offline, frame by frame. It does the things
  that make a hand-driven arm bearable. A **0.15 s engage debounce** means a hand passing through
  doesn't move anything. A **One Euro filter** is smooth when the hand is slow and responsive
  when it's fast. **Jump rejection** holds back a frame more than 80 mm from the filtered
  position until a second one agrees, so one bad detection can't fling the arm. A **deadband**
  stops hand tremor from buzzing the servos. The **release rewind** handles curling the finger:
  that drags the fingertip, so when the gesture ends the target goes back to where it was about
  0.15 s earlier. You meant where you were pointing, not where the curl ended. The target is
  also **led** ~30 mm ahead of the fingertip along the finger, so the arm doesn't park on top of
  the fingertip and hide it from the camera.

**reachable_target() lives in server.py**, because arm geometry lives there. When you point
somewhere, it first looks for a tool pitch that hits the point exactly, starting from the
preferred one (the pitch at the moment follow engages, or `follow_pitch` if that's set) and
moving outward a degree at a time, so the wrist turns no more than it has to. The wrist's travel
is narrow enough that, at hover height, only about ten degrees of pitch reach any given spot.
A pitch held fixed for the session would reach only a band of the table and park the tool
centimetres from where you point. Only when no pitch reaches the point does it slide the target
along the same azimuth, at fixed height, to the nearest point the arm can reach (scanned in 5 mm
rings, then bisected). The azimuth is clamped to the base's range. It deliberately isn't
`ik_auto()`: that one shrinks the target toward the base and lets the height wander, which is
fine for a one-off GO but wrong when you asked for "as far as you can in that direction". The
control loop writes only the four arm joints, at most every 50 ms, and the existing
speed-limited interpolator does the moving.

**The target height is fixed.** One camera has no depth: the fingertip pixel says which ray the
finger is on, not how high it is. So the target Z is `table_z + follow_hover` (kept
within 50-200 mm) and is never derived from the finger. You set the height yourself with the
Low / Hover / High buttons under Follow finger: a tap loads a preset (70 / 110 / 160 mm), and
holding Low or High moves it 40 mm/s from wherever it is. A hold is resent every 100 ms, like
creep, and the server stops it 0.4 s after the last one, so a lost release cannot run away. A
new height rewrites the held target, so the arm rises or sinks in place even with no finger in
view. That limitation buys a safety property for free:
the arm hovers *over* the work, and can never follow a finger down into the hand.

**The parallax caveat.** To turn the ray into a table point, the tracker intersects it with a
plane `follow_finger_z` mm above the table (default 15 mm, a finger held just over the surface).
A finger held higher than that lands the target farther from the camera, by about
`Δh × (horizontal distance to camera) / (camera height)`. With the camera 600 mm up and the hand
400 mm from it horizontally, a finger 30 mm higher than assumed moves the target about 20 mm.
That's fine for rough positioning. If the aim is consistently off toward or away from the
camera, tune `follow_finger_z` rather than anything in the pose.

Estimating the hand's height from its apparent size was tried and doesn't work with this
camera. Fitting MediaPipe's 3D hand model to the image landmarks (solvePnP: all 21, the palm
only, and palm plus index), and plain palm width, length or area, all gave errors of ±70 mm or
worse with the fingertip resting on the table, against about ±10 mm needed. It is steady (2-5 mm
jitter while still) but biased: a turned hand looks smaller, which reads as farther away, and
the fitted hand scale varied 1.04-1.28 between calibrations. Real height following needs a
depth camera.

**Interlocks.** Follow is the arm acting on its own, so it gives way to anything you do.
Switching it on is refused unless Live is on, the pose is healthy, no pick is running and
MediaPipe is available. Every message turns it off except a short allowlist that cannot fight
it (speed, gripper, wrist roll, settings, detect, and Live or unlock being switched *off*). So
STOP or Esc, HOME, REST, a drag, a jog, GO, a pick, Relax, creep and every calibration control
end it. It's an allowlist because a list of the dangerous ones goes stale silently: Relax was
missing from the first one, and the next finger move would have woken a limp servo under the
operator's hand. Live going off, losing the serial port and the last browser disconnecting also
end it. An unhealthy pose while tracking only pauses it, because a pose glitch is usually brief.
While paused the controller is reset every tick, so nothing seen through the bad pose survives
it, and the finger has to arm again afterwards. Losing the hand holds the arm where it is; it never goes looking. The
gripper and wrist roll are never touched. All of this lives in the server, and the UI only
reflects `state.follow`, so two tablets can't disagree about whether follow is on.

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

`test_finger.py` feeds synthetic hand landmarks to the pointing test (pointing, open palm, fist,
V sign, bent index) and drives `FollowController` frame by frame through engage, hold, rewind,
jump rejection, deadband and re-engage. `test_geometry.py` covers `reachable_target()` and checks
that the raised-plane raycast recovers a known point above the table.

`test_markers.py` and `calibrate_camera.py` are interactive hardware tools, not tests.
