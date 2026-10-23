# Plan: finger follow

**Goal.** With a toggle on, the arm's tool hovers over the spot your extended index finger points
at, and follows it around the table, as far as the arm can actually reach. It's for quick
positioning while working: point, let the arm catch up, curl the finger (or switch the toggle
off), and the arm stays put.

Branch: `feat/finger-follow`, from `master` at `dc42e29`.

---

## 1. Behaviour, from the operator's side

| You do | The arm does |
|---|---|
| Toggle **Follow finger** on (Camera tab) | Nothing yet. Status: `FOLLOW: show a pointing finger` |
| Hold your hand over the table and point (index straight, other fingers curled) | After ~0.15 s of a steady pointing hand it **engages**, then hovers at `follow_hover` mm above the table over the spot just ahead of your fingertip |
| Move your hand | The arm follows: smoothed, with no buzzing on small tremors, at the speed slider's rate |
| Point somewhere it cannot reach | It goes as far as it can along that direction and stops at the edge. Status: `FOLLOW: at the reach limit` |
| Curl the finger, open the hand, or pull the hand away | It **holds** where it was ~0.15 s *before* you let go, so curling the finger doesn't drag the target. Pointing again re-engages it |
| Toggle off, STOP, Esc, any manual move (drag, jog, GO, HOME, REST, pick) | Follow switches off and the arm holds |

Only a **pointing** hand drives the arm. An open palm, a fist, or a hand holding a tool don't, so
the arm doesn't chase your hand while you're just working.

## 2. Constraints this design respects

- **The camera is fixed overhead and oblique**, and sees the hand from above. MediaPipe Hands
  handles a dorsal view, and its *world* landmarks are 3D, so the "is the finger straight" test
  doesn't depend on where the camera is.
- **One camera, no depth.** The fingertip pixel says which ray the finger is on, not how high it
  is. We cast the ray onto a plane `follow_finger_z` mm above the table (default 15 mm, a finger
  held just over the surface) using the existing `pixel_to_table(u, v, z_off=...)`. Holding the
  finger higher than that shifts the target away from the camera by about
  `Δh × (horizontal distance to camera) / camera height`. That's acceptable for rough
  positioning, and it's documented. Estimating height from hand size is out of scope (§9).
- **The arm must never descend into the hand.** The target Z is always `table_z + follow_hover`
  (default 110 mm, clamped to at least 60 mm), never anything derived from the finger. The arm
  hovers *over* the work, it doesn't touch.
- **Pose code in `vision.py` is load-bearing** (ARCHITECTURE.md). The new code doesn't touch
  pose estimation, averaging or the gates. It only reads `frame`, `pixel_to_table` and
  `pose_health`.
- **Everything about arm geometry stays in `server.py`** (ARCHITECTURE.md). The hand tracker
  knows nothing about the arm. It only reports table coordinates.
- **The tablet UI is ES5 on Android 4.2 WebKit** (design_handoff README §1). The toggle uses the
  existing `act()` / `tog()` / `txt()` helpers and a class swap, nothing newer.
- The 360° gripper and wrist roll are never touched by follow.

## 3. Architecture

```
vision.py capture thread ── self.frame (undistorted, full res)
                                  │
finger.py  HandTracker thread  ◄──┘   (only runs while follow is on)
   MediaPipe HandLandmarker, VIDEO mode, on a 640-px-wide copy
   → classify_pointing(world landmarks)          pure, tested
   → fingertip px + pointing direction px → table XY via vision.pixel_to_table(z_off=follow_finger_z)
   → publishes the latest Observation(t, ok, x, y, dir_x, dir_y, px, landmarks)
                                  │
finger.py  FollowController   ◄───┘   pure state machine, tested; stepped by server.control_loop
   engage debounce, One Euro filter, deadband, jump rejection, release rewind, hold on loss
   → (target_xy | None, status string)
                                  │
server.py  control_loop (50 Hz) ──┘
   reachable_target(x, y, z, pitch, cfg, roll)   pure, tested: clamp onto the reachable set
   → writes S.tgt[:4] (base..wrist pitch only) at ≤ 20 Hz; the existing interpolator moves the arm
```

### 3.1 `finger.py` (new, no arm knowledge)

- `classify_pointing(world_lm) -> (bool, score)`: 21 landmarks in metres.
  - index straight: the angles at PIP (5-6-7) and DIP (6-7-8) are both > 150°, and
    `|wrist→tip| > 1.15 × |wrist→PIP|`.
  - middle, ring and pinky curled: each tip is closer to the wrist than its own PIP, or the PIP
    angle is < 120°.
  - Thumb ignored (people point with it in or out).
- `OneEuro(min_cutoff, beta, d_cutoff)`: the standard One Euro filter, per axis. Defaults are
  `min_cutoff=1.0 Hz` and `beta=0.02` (mm units).
- `FollowController(params)`, pure. `feed(obs)` takes observations and `step(now)` returns
  `(target_xy or None, state, status_text)`.
  - States: `idle` (no pointing seen), `arming` (pointing for < `engage_s` = 0.15 s), `tracking`,
    `holding` (lost, or the gesture ended).
  - **Engage debounce**: N consecutive pointing frames covering ≥ `engage_s`.
  - **Release**: `lost_s` = 0.12 s without a pointing observation. Target rewinds to the filtered
    value from `rewind_s` = 0.15 s before the last good frame, kept in a short ring buffer.
  - **Jump rejection**: a frame more than 80 mm from the filtered position is held back until a
    second frame agrees within 25 mm, so one bad detection can't fling the arm.
  - **Deadband**: the output only changes when the filtered target has moved > `deadband_mm`
    (3 mm) from the last output. That stops servo buzz from hand tremor.
  - **Lead**: target = filtered fingertip + `follow_lead` mm (default 30) along the finger's
    pointing direction in table coordinates (PIP→TIP, filtered too). This keeps the arm from
    parking right on top of your fingertip and blocking the camera's view of it.
- `HandTracker(vision, log)`: a daemon thread with `start()`, `stop()` and `latest`.
  - Lazily loads MediaPipe and the model the first time follow is switched on.
  - **Model**: `weights/hand_landmarker.task`, downloaded on first use from the versioned
    MediaPipe model URL. `weights/` is gitignored like the YOLO weights. If it's missing and
    can't be downloaded, follow refuses with a clear log line.
  - Runs `num_hands=2` and uses the hand that is pointing with the best score.
  - Processes about 25-30 Hz on a resized copy of `vision.frame` and maps landmarks back to
    full-res pixels.
  - Converts the tip pixel and a point along the PIP→TIP direction to table XY with
    `vision.pixel_to_table(u, v, raw=False, z_off=follow_finger_z)`.
  - Never raises into the caller. Errors are logged (throttled), and the observation goes
    `ok=False`.
  - Keeps the latest landmarks in pixels for the overlay.

### 3.2 `server.py`

- `reachable_target(x, y, z, pitch, cfg, roll_servo) -> (x, y, z, clamped: bool)`: pure.
  1. Clamp the azimuth to the base yaw range from `joint_ranges(cfg)[0]`, and keep `x ≥ 0` as
     `move_tip` does.
  2. If `ik()` at that point has neither `reach` nor `limited`, and FK round-trips within 2 mm,
     return it unchanged.
  3. Otherwise binary-search the radius along that azimuth at fixed z and pitch. That gives the
     farthest exactly-reachable radius not beyond the target, then (for points too close to the
     base) the nearest one beyond it. About 20 iterations, ~0.5 mm resolution.
  4. If nothing on that azimuth is reachable at this z and pitch, return `None`. The controller
     then holds, with status `out of reach`.
  - Deliberately **not** `ik_auto`: its pitch sweep and shrink-toward-base would make the wrist
    flail as the finger moves.
- **Revised during implementation:** a fixed pitch turned out to reach only a band of the
  table, because the wrist's travel is narrow. `reachable_target` now returns
  `(x, y, z, pitch, clamped)`. It hits the point exactly at the workable pitch nearest the
  preferred one, and only moves the point when no pitch in [-120, 30] reaches it.
- **Follow pitch** (now the *preferred* pitch): `follow_pitch` setting. `None` (the default) means the tool pitch at the
  moment follow engages, fixed for the session, so the wrist doesn't wave around. A number sets
  a fixed pitch.
- State: `S.follow` (bool), `S.follow_status` (str), `S.follow_last_write`.
- Messages: `{"type": "follow", "on": bool}`. Turning it on **refuses** (logs the reason, stays
  off) when:
  - Vision is off, the pose is unhealthy, or Live is off;
  - a pick is running;
  - MediaPipe or the model is unavailable.
- **Interlocks that turn follow off** (with a log line): `stop`, `home`, `rest`, `target`,
  `tip`, `planar`, `joints`, `pitch`, `jog`, `goto_click`, `pick_click`, `pick_detected`, Live
  going off, serial loss, and the last client disconnecting (in `client_gone`, next to the
  existing unlock reset).
  - While tracking, an unhealthy pose **pauses** (holds), and it resumes when the pose is
    healthy again.
- `control_loop`: when `S.follow`, call `F.step(now)`. With a target, it applies
  `reachable_target` at `z = table_z(x, y) + max(60, follow_hover)`, and writes `S.tgt[:4]`
  through `ik()` with the roll unchanged, at most every 50 ms and only when the target changed.
  It reports through `S.follow_status`, and logs only on state transitions.
- Broadcast: the `state` message gains
  `"follow": {"on", "state", "status", "target": [x, y, z] | null, "clamped"}`.
- `V.follow_overlay` is fed so the camera overlay can draw the hand.

### 3.3 `vision.py` (overlay only)

- In `_publish`, when `self.follow_overlay` is set: draw the hand skeleton (thin grey), a
  fingertip ring (green when pointing, grey otherwise), the aimed point on the table (cyan
  cross), and a line from fingertip to target. The banner gets a second line: `FOLLOW <state>`.
- New `DEFAULT_SETTINGS` keys, persisted to `vision.json` through `set_setting`:
  - `follow_hover`: 110.0
  - `follow_finger_z`: 15.0
  - `follow_lead`: 30.0
  - `follow_pitch`: None (handled like `pick_pitch`)
  - `follow_deadband`: 3.0
- `status()` includes nothing new. Follow state rides on the server's `state` message.

### 3.4 UI (`cam_panel.js`, ES5)

- A **Follow finger** toggle button in the Camera tab's right column, next to Detect.
  - `.on` class when active, amber while `arming`/`holding`, green while `tracking`.
  - Sends `{type: 'follow', on: !S.follow.on}`.
- A status line under it: `S.follow.status`.
- Setup sub-page: number fields for hover height, finger height and lead, through the existing
  `vset` number-field helper.
- Desktop: the **F** key toggles follow (Esc is already STOP).

## 4. Safety summary

1. Live must be on, the pose healthy, and no pick running.
2. Only a deliberate pointing gesture moves the arm, after a 0.15 s debounce.
3. Losing the hand holds the arm. It never "searches".
4. The target Z is a fixed hover ≥ 60 mm above the table, and never follows the finger downward.
5. Every target goes through `reachable_target`, and the joint limits still clamp in `ik()`.
6. Speed is the existing speed slider. Every manual control, STOP and Esc take over immediately.
7. Gripper and roll are untouched.

## 5. Dependencies

- `mediapipe` (1.1.1 is a `py3-none` wheel and installs on Python 3.14). It goes in
  `requirements.txt` as optional, like ultralytics.
- **Install check**: after installing, `cv2.__version__` and `cv2.aruco` must be unchanged. If
  pip wants to replace the OpenCV build, install mediapipe with `--no-deps` plus whatever it
  actually imports.
- The model file is downloaded at runtime into `weights/`.

## 6. Tests (offline, no hardware)

- `test_finger.py`:
  - `classify_pointing` on synthetic landmark sets: pointing → True; open palm, fist, two
    fingers (V) and a bent index → False.
  - One Euro: converges on a step and attenuates jitter.
  - `FollowController`:
    - engage debounce (a single frame doesn't engage);
    - hold on loss;
    - release rewind;
    - jump rejection;
    - deadband;
    - lead along direction;
    - re-engage after a hold.
- `test_geometry.py` additions:
  - `reachable_target`: a reachable point is unchanged; an over-reach target is clamped onto
    the boundary, on the same azimuth, and FK of its IK lands within 2 mm; the azimuth is
    clamped to base limits; a too-close target is pushed outward; a hopeless z returns `None`.
  - Finger raycast: with the synthetic camera, a 3D point at height h projects to a pixel, and
    `pixel_to_table(z_off=h)` returns its XY within 0.5 mm.
- Smoke: `server.py --sim --no-camera`, send `follow on` over `/ws`, and confirm it refuses
  with the right reason.
- Smoke: `--sim` with the camera, if one is attached, confirming the tracker thread starts and
  the stream keeps its fps.

## 7. Work breakdown (one commit per step)

1. This plan (`docs/plans/FINGER_FOLLOW.md`).
2. `finger.py`: `classify_pointing`, `OneEuro`, `FollowController` + `test_finger.py`.
3. `finger.py`: `HandTracker` (MediaPipe, model download) + `requirements.txt`.
4. `server.py`: `reachable_target` + tests in `test_geometry.py`.
5. `server.py`: follow state, the message, interlocks, control-loop integration, broadcast.
6. `vision.py`: settings keys + overlay.
7. `cam_panel.js`: toggle, status, settings, F key.
8. Docs: README feature line, ARCHITECTURE.md section, TEST_INSTRUCTIONS.md hardware test
   steps.

## 8. Hardware test (for the owner, not automatable here)

1. Run `server.py` with the arm. Turn Live on and the speed slider to ~30. On the Camera tab,
   toggle Follow finger.
2. Point over the table centre and check it engages and hovers over the spot ahead of the
   fingertip.
3. Sweep slowly left/right, then past the far edge, and check it stops at the reach limit.
4. Curl the finger and check it holds where you were pointing, not where the curl dragged it.
5. Pull the hand out of view and check it holds. Press STOP or drag a handle, and check follow
   turns off.
6. Tune `follow_finger_z` if the aim is consistently off away from or toward the camera.

## 9. Out of scope / later

- Finger height from hand scale (would let Z follow the finger).
- Pinch-to-grip gestures, or "point and pick".
- Following orientation (roll from the finger's direction).
