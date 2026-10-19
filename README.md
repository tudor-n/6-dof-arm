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

## Documentation

| | |
|---|---|
| [docs/HARDWARE.md](docs/HARDWARE.md) | The physical build: frame, servos, power, wiring, the quirks that matter |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | How the software is put together and why |
| [docs/CALIBRATION.md](docs/CALIBRATION.md) | **Start here when accuracy is wrong.** Every calibration procedure, in dependency order |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Where this is going, and the known-unknowns |

## Quickstart

```bash
pip install -r requirements.txt
pio run -t upload                      # firmware, once
python server.py --port COM5           # auto-detects a CP210x/CH340 port if omitted
```

Then open <http://localhost:8080>.

```bash
python server.py --sim                 # no arm; vision and IK still run
python server.py --no-camera           # no camera; arm only
python server.py --camera 1 --http 8080
python test_geometry.py                # offline maths tests, no hardware needed
```

The arm boots limp and stays limp. Nothing moves until you turn **Live** on, which sends
`R 0` to engage the servos. **STOP** (or Esc) cancels any running pick and brakes the
gripper.

## Repository map

| Path | What it is |
|---|---|
| `server.py` | The brain. Kinematics, 50 Hz control loop, HTTP/WebSocket, pick sequencing |
| `vision.py` | Camera thread, ArUco pose, table raycast, YOLO-World detection, MJPEG overlay |
| `cam_panel.js` | Camera tab, injected into `index.html` by the server at serve time |
| `index.html` | Move and Calibrate tabs; the canvas arm views and the calibration wizard |
| `src/main.c` | ESP32 firmware. A deliberately dumb executor - see HARDWARE.md |
| `config.json` | Link lengths, joint zero/sign/limits, tool offsets |
| `vision.json` | Arm-measured marker positions, pick settings, roll calibration |
| `camera_matrix.npy`, `dist_coeffs.npy`, `camera_calib.json` | Camera intrinsics and their provenance |
| `calibrate_camera.py` | Checkerboard intrinsics calibration |
| `test_markers.py` | Pre-flight: are all four ArUco markers visible? |
| `test_geometry.py` | Offline tests for the pose and kinematics maths |

## Current state

Mechanically and electrically the arm works. The software registration between camera and
arm was recently repaired - see [docs/ROADMAP.md](docs/ROADMAP.md) for what was wrong,
what is fixed, and what still needs measuring on real hardware.
