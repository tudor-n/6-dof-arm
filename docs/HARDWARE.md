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

**Confirmed:** the base is bolted parallel and flat to the desk, carrying the same slope as
the desk. So the geometry above holds and a near-flat table plane in arm coordinates is the
correct result.

**Measured slope (2026-10-19):** 35 mm of drop over 457 mm along the surface, back to front,
so the plate leans forward-down by **4.39 degrees**. Front edge 905 mm from the floor, back
edge 940 mm.

This matters only for gravity-referenced measurements. Any joint angle read with an
inclinometer or a plumb line must have 4.39 degrees subtracted before it means anything to
the kinematics, because joint angles are defined relative to the base plate and not to
gravity. Uncorrected it is worth 18 mm of tip error at 230 mm of reach and 27 mm at 350 mm.
Measuring instead in the table's own frame - distances along the surface and heights above it
- sidesteps the whole correction, since the table and the arm tilted together.

Worth re-checking only if the mounting is ever disturbed, because it is not self-evidently
rigid: M5 screws in M6 holes give the plate room to shift, and the PETG spacers between the
two aluminium plates can differ in height or compress unevenly under the nuts. Either tilts
the base relative to the desk, and then the table *will* show a slope in arm coordinates.

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

**Tape-measured 2026-10-19.** The four markers form a near-rectangle: near edge
(0-1) 392 mm, far edge (2-3) 401 mm, sides 152 mm and 174 mm, diagonals 420 mm and 436 mm.
Self-consistent as a planar quadrilateral to 1.0 mm.

The ArUco black-square edge is **23 mm**.

The arm-measured model in `vision.json` disagreed with this badly - it described a strong
trapezoid, far edge 496 mm against the real 401 mm. That was the first independent check on
forward kinematics this project had, and FK failed it. `tape_check()` now guards against it
permanently, and the measurements live in `vision.json` under `tape`. See ROADMAP.md.

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
