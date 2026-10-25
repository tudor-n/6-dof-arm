# Software architecture

Three processes' worth of responsibility, two of them in one Python process.

```
┌─ browser ────────────────────────────────────────────────┐
│  index.html        Move + Calibrate tabs, canvas views   │
│  cam_panel.js      Camera tab, STOP, log  (injected)     │
│  voice_panel.js    Voice tab              (injected)     │
└───────────┬──────────────────────────────────────────────┘
            │ WebSocket /ws  (JSON)      MJPEG /video
┌───────────▼──────────────────────────────────────────────┐
│ server.py                                                │
│   kinematics      fk / fk_planar / ik / ik_auto          │
│   control_loop    50 Hz interpolate -> serial            │
│   pick_seq        hover, descend, close, lift, hold      │
│   vision (thread) ─────────► vision.py                   │
│   follow (thread) ─────────► finger.py                   │
│   voice (threads) ─────────► hearing.py    ◄── mic       │
│   gestures        ─────────► gestures.py   nod/wave/...  │
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
jaw-closing angle, hover, descend strictly, close, lift back to hover, and **stay there
holding it**, the gripper braked. `S.held` records what it holds and where it came from;
the state broadcast carries it. It runs as a cancellable asyncio task (`S.pick_task`); STOP
cancels it and brakes the gripper. `start_pick` refuses outright when the camera pose is
unhealthy, and while something is already held. From the hold:

| Action | Does |
|---|---|
| hand-over (`handover`; the Camera tab's Bring here; "bring it to me") | carries it to `handover_x/y/z/pitch` in `vision.json` (310, 30, 240, −15: toward the operator, up, a little to their right) and waits; the Open button or "let go" releases it |
| put down (`put_down`; Put down; "put it down") | lowers it straight down to the pick's own grasp height (`max(z_floor, table + z_grasp_offset)`), opens for 1 s, brakes, rises to hover |
| put back (`put_down` + `back`; "put it back") | the same, carried first to above the spot it was picked from |
| carry (the Camera tab's Hover while holding) | to above the tapped spot |

**The carry rule.** A move made while holding drives the gripper closed on the way, as the
lift always did, and brakes on arrival (`carry()`); between moves the gear train holds the
jaws. The 360° servo therefore never drives for longer than one move, which a hold that
can last minutes needs. Every target is checked first: `aim()` finds the pitch nearest the
preferred one that puts the tool exactly on the point (`reachable_target(exact_only=True)`),
and a point no pitch reaches is refused without moving. One gripping sequence runs at a time
(`start_seq`). Opening the gripper (a grip value above 95), a put-down, Relax grip or Relax
all clears `S.held`.

**Gripper commands in these sequences are set at once** (`grip_now()`, target and current, as the
brake always was), not ramped under the speed slider like a joint. The number is a speed, not a
position: ramped, a timed pulse spent its time getting there. A put-down's 1 s open came right
after the squeeze, and at speed 20 it ended before the command was even back past 90, so the
gripper never opened. This covers the pick's open and close, the carry squeeze, put down, and the
spoken open / close. The tablet's hold-to-move grip buttons and the gesture claps keep their ramps.

**Rest pose.** Limp, the arm falls wherever gravity takes it, so the rest pose
(`config.json` `rest`, joint angles) parks the tool on the desk: 240 mm out, tool pitched
−80°, upper arm leaning forward. It was found from the calibrated kinematics: the elbow and
the heavy wrist end as low as the joint limits allow, every joint at least 6° from its
limit, nothing but the tool near the desk. (The old values, written for the default
calibration, clamped to a pose with the tool ~290 mm up, and the old handler also ignored
the servo gain, which `rest_servos()` now includes.) Getting there is the delicate part,
since a joint-space move to a pose on the desk could sweep the tool across it. `park()`:
1. rises straight up first if the tool is lower than the spot above the rest pose;
2. goes to 60 mm above the rest spot;
3. comes straight down through an exact IK waypoint every 10 mm (`rest_approach()`), at a
   fixed 20 °/s whatever the speed slider says (`S.slow`).

It runs as `S.voice_task`, so STOP, manual control or another command ends it where it is,
and `settle()` stops it the moment Live goes off. The Rest pose button and the spoken rest
pose, relax and good night all use it. **Relax all stays immediate** and goes limp
wherever the arm is; Rest pose then Relax all is the gentle way. Calibrate → Advanced →
**Set rest here** saves wherever the arm was jogged to as the new rest pose (`setrest`, a
calibration message), because a limp arm's settling can only really be judged on the bench.

**Wake all** (`relax` with mask 0) engages every servo where it was last told to be, then sends the
arm to the start pose under the speed limit, gripper braked. Engaging in place matters after a
relax at the rest pose: that is where the arm really is, so nothing jumps, and from there the
start pose lifts the tool straight off the desk before it swings back.

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
within 20-400 mm) and is never derived from the finger. You set the height yourself with the
Low / Hover / High buttons under Follow finger: a tap loads a preset (70 / 110 / 160 mm), and
holding Low or High moves it 40 mm/s from wherever it is, as far as the arm reaches. Each step
of a hold checks that the arm can still hit the held spot exactly at the new height (any pitch
in the window, `reachable_target(..., exact_only=True)`, a few ms), and stops at the last height
that works instead of letting the arm be pulled sideways; the status line then says "highest
here" or "lowest here". Far out the top comes well under 400 (about 235 mm at 380 mm out). A hold is resent every 100 ms, like
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

## Voice

The Voice tab lets you say **"Dummy, come here"** (finger follow on), **"Dummy, start pose"**,
**"Dummy, relax"** (park on the desk, then go limp), **"Dummy, stop"** and about fifty more: small
moves, the gripper, picking, the camera, questions and emotes (the catalogue is below, and on the
tab's Commands page). It listens in one of two modes, chosen on the tab: **wake word**, where only speech that starts with "Dummy" counts,
or **push to talk**, where holding the button (or V on a keyboard) is the only way in and the name
is optional. Push to talk also works in wake-word mode, as a bypass for testing.

**hearing.py knows nothing about the arm**, like finger.py. It reports what it heard and a command
id; the server decides what that means.

- Audio comes from the mic in 32 ms frames: through `parec` on Linux (PipeWire / PulseAudio does
  the resampling, and no PortAudio is needed), through `sounddevice` elsewhere. **Silero VAD**
  scores each frame for speech. It's the ONNX model faster-whisper ships, run with onnxruntime,
  because an energy threshold can't tell a voice from servo whine.
- `Segmenter`, a pure state machine, cuts the scores into utterances. Speech starts after two
  frames over the threshold, ends after `end_silence` below it (with Silero's 0.15 hysteresis),
  and keeps 0.4 s from before the start so the first syllable isn't lost. Push to talk overrides
  all of it: everything between press and release, plus a 0.3 s tail, is one utterance.
- **faster-whisper** (`small.en` by default) transcribes each utterance with "Dummy" as a hotword.
  On the GPU that takes about 60-80 ms; on the CPU about 0.7 s. CTranslate2 is built against
  CUDA 12 while torch here brings CUDA 13, so `hearing.py` loads cuBLAS 12 from the
  `nvidia-cublas-cu12` wheel by full path before the model loads, and falls back to the CPU if
  that fails. Whisper's own no-speech probability above 0.5 discards the text: on noise, the
  hotword otherwise invites a hallucinated "Dummy".
- `Matcher`, pure, turns the text into a command. The wake word must be one of the first three
  words, spelled exactly (or as a learned alias), and **never** fuzzy-matched: a fuzzy match would
  make "Mummy, come here" or "my tummy" a trigger for everyone. What follows it is tried in order
  (`Matcher.parse`):
  1. **an exact phrase**, as said or with the filler words dropped, wins outright;
  2. **an object slot**: "pick up / grab / fetch / get the X", "bring me / give me / hand me the X",
     "find / look for / where is / do you see the X", with up to three words of object. "It",
     "that" and "this" mean whatever Detect sees now;
  3. **a command with an amount** (`parse_amount`): a number as digits or words, an optional mm / cm /
     degrees, or "a bit" / "a lot", accepted only for the step commands;
  4. **the fuzzy match**: character similarity at least 0.8 and 0.1 clear of the runner-up command,
     after filler words are dropped. "Hey Dummy, please come here" works; "Dummy, can you start the
     pose estimation again?" doesn't.

  A not / never / n't rules every path out. Two phrases read too much like another command to ever
  act: "slower" (like "lower") and "closer" (like "close"). They come back as unknown with a hint
  saying what to say instead. A phrase whose filler-free form would read like another command
  counts only as said in full: "let go" opens the gripper, but its filler-free "let" is too close to
  "left" to match on its own. `test_voice.py` checks that every phrase maps to its own command and
  that no two commands' phrases come within the threshold, apart from a short list of pairs that
  sound different and are fine said as they are.

**The wake word isn't a separate model.** Every utterance the VAD finds is transcribed, and it counts
only if it starts with the name and the rest is a command. A dedicated spotter (openWakeWord,
Porcupine) would need a custom model trained or an account, and a two-syllable word like "dummy" is
where those are weakest. Whisper needs nothing trained, the word can be changed in `voice.json`, and
the tab can show exactly what was heard and why it did or didn't count. The cost is running Whisper on
all speech near the mic, which on the GPU is negligible. A bare "Dummy" opens a 5 s window in which
the command needs no name, for people who pause after it.

**Calibration** is on the tab, three steps, about two minutes:
1. *Mic check*: shows the voice-over-room margin. Under 12 dB is too quiet to be reliable.
2. *Quiet room*: 8 s of silence (servos running, ideally). It sets the speech threshold just above
   the loudest 1% of what the VAD scored there, never below 0.5.
3. *Your voice*: each command twice. When Whisper consistently writes your "Dummy" as something else,
   it learns that spelling as a wake alias, and likewise for a command phrase. `learn()` is cautious:
   an alias must look a little like the word it stands for, must not look like another command, and
   the Settings page lists everything learned with a button to forget it.

**Doing a command (server.py).** Voice is one more client of `handle()` (`VOICE`), so Live, the pose
check, pick refusal and follow's allowlist apply to it exactly as to a button. On top of that:
- **Motion is refused while Live is off**, rather than queued, since a target set now would make
  the arm jump when Live goes on.
- **Commands are refused when no page is open**, the same presence check follow uses: whoever
  talks can see the STOP button. **"Stop" is exempt**, from this and from dry run: it never moves
  anything.
- **"Relax" folds to the rest pose first** (`voice_relax`), waits until the arm arrives, and only then
  sends `relax 63`, so the arm never drops from mid-air. These cancel it: STOP, another spoken
  command, follow switching on, voice going off or into dry run, and the same manual messages that
  end follow. Even uncancelled, it checks again right before going limp: Live still on, follow off,
  and the arm at exactly the rest pose it was sent to. Anything else (a USB reconnect resets the
  target and turns Live off) leaves the arm engaged. `test_voice.py` runs these cases against the real
  control loop.
- **Nothing acts late.** Every utterance carries the `Hearing.gen` it started in. STOP, a mode change
  and a calibration starting or ending bump it, so speech still being transcribed when you press
  STOP is dropped, and so is a command that reaches the server after it.
- **Dry run** hears and shows everything and acts on nothing except "stop". The questions (where
  are you, is the pose OK, what do you see) and the voice settings still answer.

**The commands** (`hearing.COMMANDS`, `server.VOICE_DO`). The operator stands in front of the arm,
facing it, so their left is the robot's right (−Y) and "toward me" is +X.

| Group | Say | Does |
|---|---|---|
| Moves (need Live; refused while an arm joint is limp) | up, down, left, right, toward me, away | the tool 20 mm that way (a bit 10, a lot 50, a number capped at 50). It lands on that point exactly, at the pitch nearest the current one, or doesn't move and shakes its head. While following, up and down change the hover height and follow stays on |
| | turn left / right | the base 15° (at most 45) |
| | tilt up / down, point down | the tool pitch ±15° about the same tip point, or straight down |
| | roll left / right, straighten | the wrist 30° (at most 90), top of the tool to your left / right; or back to roll zero |
| | open, close (let go, release, drop it; grip, squeeze) | a timed pulse, 1.0 s / 1.5 s (a bit 0.4, a lot 2.5), then the brake. A voice task, so STOP brakes it too; it doesn't end follow |
| | rest pose, wake up | `park()`; Wake all then Start pose |
| | undo, again | back to before the last spoken step (the last 10 are kept); the last step once more |
| Pick | pick up the pen, bring me the pen, pick it up | finds it (sets the detect prompt, waits up to 8 s for 3 frames of it), then picks it, and with bring hands it over |
| | bring it to me, put it down, put it back | hand-over, put down, put back (see Picking) |
| Speed, follow, camera | slow down, faster, full speed, normal speed | the speed buttons, one step or straight to 180 / 60 |
| | hover low / high, normal height | the Follow height presets |
| | find the pen, stop looking | Detect on with that prompt, then a nod (and where) or a shake; Detect off |
| | is the pose OK?, what do you see?, where are you? | answers in the Heard list, with a nod or a shake for the yes / no ones |
| Voice | yes, no; what can I say; stop listening; be quiet, sounds on | answer a pick waiting for yes; open the Commands page; voice off; sounds off / on |

**A spoken pick asks first.** With `pick_confirm` on (the default; Voice → Settings → Confirm picks),
the pick hovers over what it found so you can see its choice, plays a rising chime, and the chip
reads SAY YES with a countdown. "Yes" within 8 s runs `start_pick` there. "No", silence, STOP or any
other command cancels it (`voice_ask()`, a future the yes or no resolves). With it off, it picks at
once, like the Pick button. Either way every refusal of `start_pick` still applies. A command that
finishes later, like find or a pick, sends its Heard entry again with the outcome.

**The no-name window.** A bare "Dummy" opens 5 s in which the command needs no name, as before.
After a successful step the server opens the same window with `Hearing.keep_awake()`, but it then
takes only steps, again, undo, the speed words and stop. So "Dummy, up … up … left … again" needs
the name once, while "relax" said in passing still needs it. While a pick waits, only yes, no and
stop are taken without the name.

**Not by voice, on purpose**: Live (on or off), calibration and setup (set zero, flip, limits,
links, Set rest here, markers, roll zero / flip, unlock, raw servos, camera settings, pose lock),
go-to-click and pick-click (the tap on the video is right there), dictated coordinates, continuous
motion ("keep going": a voice STOP lands 0.3–1 s late), and "pick that" while pointing (it would
descend where the hand is). No spoken replies either: like the film's Dummy, he answers with beeps
and gestures.

The Voice tab's own messages (`voice_set`, `voice_ptt`, `voice_train`, `voice_mics`) are in
`FOLLOW_SAFE`: changing a voice setting doesn't end follow. A push-to-talk hold is resent every
100 ms, and the server releases it 0.4 s after the last one. Short beeps on the laptop say "heard
the name", "done" and "refused"; the mic ignores the audio while one plays. The last 20 utterances
stay in memory for the tab's play buttons, and nothing is written to disk.

## Personality

DUMMY answers with his gripper as a head:

| You say | He does |
|---|---|
| "Dummy" (alone) | **nods**, then listens for the command |
| "Dummy, are you dumb?" (stupid, an idiot, broken, useless; "you're dumb") | **shakes his head**: the base swings ±10° |
| "Dummy, good boy" (good job, well done, good robot), or "dance" | a **happy dance**: base sway with the head bobbing against it, a roll wiggle, a gripper clap, a double nod |
| "Dummy, are you awake?" (ready, listening, there, can you hear me, are you happy, how are you) | **nods** |
| "Dummy, hello" (hi, bye, good morning, see you) | a **wave**: head up, the wrist rolls side to side |
| "Dummy, thank you" (thanks, take a bow) | a **bow**: one deep, slow nod that holds |
| "Dummy, bad boy" (bad robot, naughty, shame on you) | a **sad droop**: sinks slowly, looks away, comes back |
| "Dummy, …" and a short phrase that isn't a command | a **puzzled head tilt**, with a rising "hm?" instead of the "no" beep |
| "Dummy, do a trick" (show off, surprise me) | one of nod, shake, wave, bow, dance |

Each reaction has its own sound on the laptop, and the Voice tab's Personality page has Try buttons
for all seven gestures, size and speed sliders, and a switch for the clap. The bow and the droop move
at half and a third of the gesture speed (`gestures.TEMPO`), so they look solemn. The head tilt never
comes for a long sentence (more than four words after the name: that's talk, not a command), nor in
the no-name window. Extra phrases can go in `voice.json` under
`reactions`. A not / never / n't the phrase doesn't have rules a match out, so "you're not dumb" gets
nothing.

**gestures.py is choreography only, and pure.** A gesture is keyframes of joint-degree offsets from
the pose the arm is in when it starts. That's why it works from anywhere and ends where it began.
`plan()` turns the offsets into servo targets with the same sign / gain convention as
`joint_angles()`. It finds which way of the wrist tips the tool *down* from the arm's own forward
kinematics instead of assuming a sign. Then it checks every keyframe:
- inside the joint limits;
- the tool tip at least 60 mm above the table (`V.table_z`);
- never more than 40 mm below where it started.

If a keyframe fails, the whole gesture shrinks in steps to 40%, and below that it's skipped with the
reason. Near the table there's nothing at all, so at the rest pose (parked on the desk) no gesture
fits: a start pose or a wake up gets him up first.

**server.py runs it.** `voice_gesture()` checks first:
- Live is on;
- no pick is running;
- no limp arm joint. `S.limp` mirrors the firmware: it boots limp, `R` sets it, and moving a channel
  wakes it. A nod after a relax would otherwise wag the wrist on a limp arm.
- the arm isn't still moving. It plans from the target, so the rest of an unfinished move would
  otherwise run at gesture speed along a path nobody checked. Taking over from another gesture is
  the exception.
- for a spoken reaction, no relax fold is running. Reactions never interrupt one.

Then it plans, and only if the plan works does it cancel whatever is running. A reaction that has no
room therefore leaves the current gesture to finish and come home.

Roll and gripper hold nothing up, and a start pose after a relax never moves them, so a gesture wakes
those two itself, but only the ones it moves. The gripper is braked before it's woken, so a 360°
servo relaxed mid-drive can't wake up driving.

The gesture runs as `S.voice_task`, so STOP, manual control, another spoken command, follow switching
on, voice going off and losing the serial link all cancel it, exactly as they cancel a spoken relax.
It also stops by itself the moment Live is off, so nothing of it is left to happen when Live comes
back:
- **STOP holds where the arm is.**
- **Another gesture hands over the first one's start pose**, so both end there.
- **Speed:** while `S.gesture` is set, the control loop steps linearly at the gesture's own speed
  (120 °/s by default, not the speed slider, which is left alone), so keyframes land on time.
- **Follow** keeps tracking but doesn't write during a gesture, then carries on with whatever is
  pending.
- **The gripper clap** is open and close pulses of the same length, and every keyframe of a gesture
  that claps sets the gripper (back to the 90 brake unless it's clapping). `run_gesture`'s `finally`
  brakes it however the gesture ends, the same dead-man rule as `pick_seq`. A 360° servo isn't
  exactly symmetric, so the jaws can drift a little per dance; the clap can be switched off.

A bare "Dummy" nods, and a short miss tilts his head, only when no voice task is running, so neither
interrupts a relax fold or another gesture. A step refused at the reach limit, a find that saw
nothing and a "no" answer shake his head through the same path, and only when a gesture can run.

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
original console. `voice_panel.js` is injected after it the same way and adds the Voice tab.

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

`test_voice.py` feeds transcripts to the matcher (wake word, chatter, look-alike words, push to
talk, learned aliases), made-up speech probabilities to the segmenter (pre-roll, blips, hysteresis,
push to talk) and takes to the calibration's `learn()`. With `DUMMY_VOICE_CLIPS=<folder>` it also runs
real WAVs through the whole chain. A wrong action fails it outright, and misses may be up to 5%.
It also guards the whole catalogue (every phrase maps to its own command, no two commands read
alike), the amounts and object slots, and runs the spoken commands against the real control loop:
steps that move exactly that far in the operator's frame or are refused without moving, the
gripper pulse that always ends braked, a pick that never descends without a yes, and the no-name
window that takes only steps. `test_geometry.py` runs the pick, hand-over and put-down sequences
against a fake `move_tip` (the gripper squeezes while carrying, brakes whenever it stops, never
opens over a spot it can't reach), and checks the rest pose (on the desk, clear of the limits) and
`park()`'s straight-down approach.
`DUMMY_VOICE_WAV=<folder>` makes the server play WAVs instead of the mic, for testing the tab with
no one talking.

`test_gestures.py` checks `gestures.plan()` on its own: the nod dips first, keyframes stay inside the
limits and away from the table, gestures shrink near a limit and refuse near the table, the gripper is
braked between claps, and every gesture ends where it began. It also runs the server's gestures
against the real control loop, sampling the tool every tick:
- they come back to the start pose and never get near the table;
- they're refused with Live off, a limp arm, a pick running, or personality off;
- STOP holds, and a dance cut off mid-clap leaves the gripper braked;
- the real `follow_tick` pauses during a gesture and carries on after.

`test_markers.py` and `calibrate_camera.py` are interactive hardware tools, not tests.
