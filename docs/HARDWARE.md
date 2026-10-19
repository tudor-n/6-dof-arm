# Hardware

## Bill of materials

| Part | Notes |
|---|---|
| Aluminium 6-DOF arm frame | Generic AliExpress kit |
| 5x MG996R (180°) | Base, shoulder, elbow, wrist pitch, wrist roll |
| 1x MG996R modified to 360° | Gripper. Continuous rotation, so it has no position feedback |
| ESP32-WROOM | Receives commands over USB, drives the PCA9685 |
| PCA9685 | 16-channel PWM driver, I2C address `0x40` |
| 220 V → 5 V PSU | **Servo power only.** Must not come from the laptop |
| USB webcam | Fixed mount, high on the robot's right, pointing down |
| 4x ArUco 4x4_50 markers, IDs 0-3 | Stuck to the table in front of the base |

## Wiring

```
laptop ──USB-C── ESP32 ──I2C── PCA9685 ──PWM── 6 servos
                 SDA 21                          ▲
                 SCL 22                          │
                                      220V PSU ──┘  5 V, high current
```

The ESP32 takes logic power and serial from the laptop's USB. The servos take power from
the mains PSU. **The grounds must be common** and the servo rail must never be fed from
USB - six MG996R under load will brown out the ESP32 and the symptom looks like random
joint jumps, not like a power problem.

## Geometry

The arm is bolted to a desk. The desk slopes: the far side (against the wall, behind the
arm) sits a few cm higher than the near side.

**This slope does not appear in the arm's coordinate frame.** The arm's +Z is normal to its
base plate, not vertical - the frame is defined by the mounting, not by gravity. Bolting
the base to the sloping desk tilts the entire coordinate frame along with the desk, and the
markers sit on that same surface, so in arm coordinates the table is `z = constant`. The
slope matters for gravity (servo sag, objects rolling) and not for geometry.

The arm-measured marker heights agree: 5.0, 5.0, 5.2 and 5.4 mm, a 0.4 mm spread across a
200 x 500 mm footprint.

**This assumes the base plate ends up genuinely parallel to the desk, and the mounting does
not guarantee it.** M5 screws in M6 holes give the plate room to shift, and the PETG spacers
between the two aluminium plates can differ in height or compress unevenly under the nuts.
Either tilts the base relative to the desk, and then the table *will* show a slope in arm
coordinates. A warped desk does the same. Worth checking with a straightedge: the plate
against the desk, and the desk against itself across the marker footprint.

Base mounting: M5 screws through M6 holes, wide washers, nuts, and custom PETG spacers
between the two aluminium plates. The oversized holes mean the base *can* shift if
knocked - and everything downstream is measured relative to the base, so a knock
invalidates the marker measurements. If the arm is bumped, re-run marker calibration.

### Coordinate frame

```
        +X  forward, away from the operator
         ▲
         │
 +Y ◄────┼─────  robot's LEFT
         │
       base      +Z up, origin at the base rotation axis at table level
```

All lengths in millimetres, all angles in degrees.

### Marker layout

Described facing the robot, so the operator's left is the robot's right:

| ID | Operator's view | Robot frame |
|---|---|---|
| 1 | left, nearer the robot | right, near the base |
| 3 | left, nearer the operator | right, far from the base |
| 0 | right, nearer the robot | **left**, near the base |
| 2 | right, nearer the operator | **left**, far from the base |

The four markers form a quadrilateral, not a rectangle - the far edge is noticeably wider
than the near edge. That asymmetry is recorded in `vision.json` and has been present since
the first working configuration. It has never been checked against a tape measure; see
ROADMAP.md.

The arm can physically touch all four markers, which is what makes arm-measured marker
positions possible in the first place.

## Firmware

`src/main.c` is a deliberately dumb executor. It holds no model of the arm: no kinematics,
no link lengths, no notion of a tool. It receives joint angles and writes PWM. All the
intelligence is in `server.py`, so it can be changed without reflashing.

- ESP-IDF, `[env:esp32dev]`, 2 MB flash, `pio run -t upload`
- UART0, 115200 8N1, newline-terminated
- PCA9685 at 50 Hz, all six channels written in one I2C transaction
- Exponential approach to target with a **180 °/s firmware-side cap**, a safety net
  independent of whatever the server asks for
- Channel 5 (the gripper) snaps straight to target instead of ramping, because a
  continuous-rotation servo has no position to ramp to
- **Boots fully limp** (`relaxed = 0x3F`): PWM is cut on every channel at power-on so the
  arm cannot lurch before the server has adopted its pose

### Serial protocol

| Command | Meaning |
|---|---|
| `J a0 a1 a2 a3 a4 a5` | Target angles for the six channels |
| `L <ch> <lo> <hi>` | Hard per-channel limits, enforced in firmware |
| `R <bitmask>` | Bit set = cut PWM on that channel (limp). `R 0` engages everything |
| `GET` | Replies `S <a0..a5> E<engaged>` |

Moving a channel wakes it from the relaxed state automatically.

### Servo pulse mapping - a known soft spot

The firmware maps 0-180° onto PCA9685 counts 150-500. At 50 Hz with 4096 counts per
period that is **732-2441 µs**. MG996R are nominally 500-2500 µs for their full sweep, so
a commanded degree is not guaranteed to be an actual degree. Any gain error here becomes
an angular scale error in forward kinematics, which would show up as marker positions that
are self-consistent but do not match a tape measure. Untested so far - see ROADMAP.md.

## The gripper dead-man's switch

The 360° gripper servo has no position feedback, so a "closed" command is really "keep
driving closed", which stalls the servo and draws current indefinitely. The workaround is
in software: `do_stop()` in `server.py` sets both the current and target gripper value to
90 (neutral), which is the continuous-rotation servo's stop point. STOP and Esc in the UI
both route here. Treat this as a real safety feature, not a nicety - a stalled MG996R gets
hot enough to damage itself and anything it is gripping.

## Tool centre point

The point the gripper actually grasps with is **not** the end of link 3, and it is **not**
on the wrist-roll axis. `config.json` carries three offsets measured from the end of L3:

| Key | Direction |
|---|---|
| `along` | Further along the tool axis, beyond L3 |
| `perp` | Perpendicular to the tool axis, in the arm's plane, at roll zero |
| `lateral` | Out of the arm's plane, at roll zero |

`perp` and `lateral` rotate about the tool axis with wrist roll, which is why an
unmodelled offset makes the arm's idea of its own fingertip wander as the wrist turns.
They default to zero, which reproduces a tip sitting exactly on the L3 centreline.
Measuring them is in CALIBRATION.md.
