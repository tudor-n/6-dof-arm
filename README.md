# 6-DOF Robot Arm

A budget 6-DOF servo arm that picks objects off a table using a fixed overhead camera.
An ESP32 drives the servos, a Python server does the kinematics and vision, and the
operator works from a browser.

```
browser  ──WebSocket/MJPEG──  server.py (IK, vision, pick logic)  ──USB serial──  ESP32  ──I2C──  PCA9685  ──PWM──  6x MG996R
                                   │
                              USB camera  ──  4x ArUco markers on the table
```

The camera never sees the arm's joints. It sees four ArUco markers whose positions in the
arm's own coordinate frame were measured *by the arm*, and solves for where the camera is
from those. Everything downstream - clicking a point, detecting an object, picking it -
is a ray from the camera onto the fitted table plane, expressed in arm coordinates.

The same camera can steer the arm by hand. With **Follow finger** on, point at the table and
the tool hovers over the spot your index finger points at, following it as far as the arm can
reach. Curl the finger and it stays put. It's for quick positioning, not precision. The
**Low / Hover / High** buttons under it pick the hover height: tap for a preset, hold Low or
High to go as low or as high as the arm reaches.

## Documentation

| | |
|---|---|
| [docs/HARDWARE.md](docs/HARDWARE.md) | The physical build: frame, servos, power, wiring, the quirks that matter |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | How the software is put together and why |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | **Start here when accuracy is wrong.** Every calibration procedure, in dependency order |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Where this is going, and the known-unknowns |
| [TEST_INSTRUCTIONS.md](TEST_INSTRUCTIONS.md) | How to test the current working branch, and what will surprise you |

## Quickstart

```bash
pip install -r requirements.txt
pio run -t upload                      # firmware, once
python server.py --port COM5           # auto-detects a CP210x/CH340 port if omitted
```

Then open <http://localhost:8080>.

**Or as a desktop app (Linux):** `python3 -I -S desktop/launcher.py install --seed-venv .venv --assets-from .`
once, then start **DUMMY** from the launcher. It keeps an installed copy on master by itself
(calibration edits survive updates); see [desktop/README.md](desktop/README.md).

```bash
python server.py --sim                 # no arm; vision and IK still run
python server.py --no-camera           # no camera; arm only
python server.py --camera 1 --http 8080
python test_geometry.py                # offline maths tests, no hardware needed
python test_finger.py                  # offline finger-follow tests
```

`ultralytics` (Detect) and `mediapipe` (Follow finger) are optional: without them the rest of
the camera tab still works. Follow downloads its hand model into `weights/` the first time you
switch it on.

The arm boots limp and stays limp. Nothing moves until you turn **Live** on, which sends
`R 0` to engage the servos. **STOP** (or Esc) cancels any running pick and brakes the
gripper.

## Repository map

| Path | What it is |
|---|---|
| `server.py` | The brain. Kinematics, 50 Hz control loop, HTTP/WebSocket, pick sequencing |
| `vision.py` | Camera thread, ArUco pose, table raycast, YOLO-World detection, MJPEG overlay |
| `finger.py` | Finger follow: MediaPipe hand tracking and the follow state machine. Knows nothing about the arm |
| `cam_panel.js` | Camera tab, injected into `index.html` by the server at serve time |
| `index.html` | Move and Calibrate tabs; the canvas arm views and the calibration wizard |
| `src/main.c` | ESP32 firmware. A deliberately dumb executor - see HARDWARE.md |
| `config.json` | Link lengths, joint zero/sign/limits, tool offsets |
| `vision.json` | Arm-measured marker positions, pick settings, roll calibration |
| `camera_matrix.npy`, `dist_coeffs.npy`, `camera_calib.json` | Camera intrinsics and their provenance |
| `calibrate_camera.py` | Checkerboard intrinsics calibration |
| `test_markers.py` | Pre-flight: are all four ArUco markers visible? |
| `test_geometry.py` | Offline tests for the pose and kinematics maths |
| `test_finger.py` | Offline tests for pointing detection and the follow state machine |
| `desktop/` | DUMMY, the desktop app: a Tauri window around the UI, and the launcher that installs and updates it |
| `docs/brand/` | The DUMMY logo: marks and launcher tiles (dark and light) |

## Current state

**Working, and closer to good than it has been - but not finished.** The arm picks objects off
the table reasonably reliably.

| | |
|---|---|
| Camera pose | Solid. Mean reprojection ~1.8 px, camera position steady |
| Marker model | Hand-measured, independently tape-checked to 7 mm worst |
| Arm kinematics | All four joints calibrated. ~11 mm at table height (cross-validated) |
| Grasp depth | Still being tuned |
| Tool model | Along-axis length fitted; off-axis offsets unmeasured |
| Finger follow | New. Tested offline only; not yet tried on the arm |

The remaining errors are millimetres rather than centimetres. Getting from "works most of the
time" to "works every time" is what [docs/ROADMAP.md](docs/ROADMAP.md) tracks - it also records
what was wrong, how each fault was found, and which hypotheses were ruled out and why.

Offline tests: `python test_geometry.py` and `python test_finger.py` (no hardware needed).
