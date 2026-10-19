#!/usr/bin/env python3
"""Fit each joint's zero, sign and gain from measured angles -> a config.json block.

    python fit_servo_calibration.py --template      write a blank servo_cal.json to fill in
    python fit_servo_calibration.py                 fit servo_cal.json and print the result
    python fit_servo_calibration.py --apply         also write the values into config.json

The model server.py uses is

    joint_angle = sign * (servo - zero) * gain

so each joint has TWO unknowns worth calibrating, not one. Calibrating only the zero leaves
a gain error in place, and calibrating only the gain leaves the arm pointing somewhere other
than the model believes. Both fall straight out of a line fitted through three or more
measurements, which is all this script does:

    measured_angle = m * servo + c        ->   sign*gain = m,   zero = -c/m

WHY TWO PARAMETERS. src/main.c maps 0-180 deg onto PCA9685 counts 150-500, i.e. 732-2441 us
at 50 Hz, while an MG996R is nominally 500-2500 us for its full sweep. That is 85% of the
pulse window before any per-unit shortfall, so a commanded degree is not an actual degree -
that is the gain. Separately, the pose the model calls "all joints at zero" (arm straight up)
is not the pose the arm actually adopts - that is the zero.

MEASURING THE ANGLES. Two ways in, and the second needs no angle tool at all.

  "angles"     - direct readings from a phone inclinometer or a digital angle gauge taped to
                 the link. Gravity-referenced, so it needs --plate-tilt (see below).

  "axis_poses" - the forward offset and height of each joint axis, measured with a ruler and
                 a square IN THE TABLE'S OWN FRAME: forward along the table surface from the
                 base rotation axis, height straight up from the table surface. The joint
                 angles then fall out of atan2 between consecutive axes, and the link lengths
                 come free from the same numbers as a cross-check on config.json.
                 Because the measurements are already relative to the table - which the base
                 plate is bolted flat to - they are plate-relative to begin with and
                 --plate-tilt MUST NOT be applied. That is the real advantage: the tilt
                 cancels instead of being corrected.

For the base, you need no protractor either way: the ArUco markers have known true azimuths,
so jogging the tip onto each marker gives four (servo, angle) pairs for free - run with
--base-from-markers to list them.

THE TILTED-DESK TRAP. An inclinometer measures from gravity-vertical, but joint angles are
defined relative to the BASE PLATE, which is bolted to a sloping desk. Measure the plate's
own tilt first and pass it as --plate-tilt so it is subtracted from every pitch reading;
otherwise the desk's slope is baked into all your zeros. Sign convention: positive means the
plate leans forward, away from the operator.
"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).parent
CAL_PATH = HERE / "servo_cal.json"
CFG_PATH = HERE / "config.json"

# True azimuths of the hand-measured markers, for calibrating the base without a protractor.
MARKER_AZIMUTH = {0: +54.62, 1: -61.37, 2: +35.17, 3: -34.71}

PITCH_JOINTS = {"shoulder", "elbow", "wrist pitch"}

TEMPLATE = {
    "_how": "Each entry is [commanded servo value, measured actual joint angle in degrees]. "
            "Three points minimum, four or more is better, spread as widely as the joint "
            "safely allows. Delete any joint you are not calibrating.",
    "_plate_tilt_deg": "Measure the base plate's tilt with the same inclinometer and pass it "
                       "as --plate-tilt. Positive = leaning forward, away from you.",
    "_axis_method": "Alternative to measuring angles: fill in axis_poses instead and no angle "
                    "tool is needed. All coordinates are [forward, height] in mm - forward "
                    "along the table surface from the base rotation axis, height up from the "
                    "table surface - with the base at its zero so the arm lies in one plane. "
                    "Do NOT pass --plate-tilt with this method; these are already "
                    "plate-relative. Delete this section if you measured angles instead.",
    "shoulder_axis": [0, 83],
    "axis_poses": [
        {"servo": [90, 70, 120, 150], "elbow": [None, None], "wrist": [None, None], "tip": [None, None]},
        {"servo": [90, 90, 90, 150], "elbow": [None, None], "wrist": [None, None], "tip": [None, None]},
        {"servo": [90, 110, 60, 120], "elbow": [None, None], "wrist": [None, None], "tip": [None, None]},
        {"servo": [90, 140, 40, 100], "elbow": [None, None], "wrist": [None, None], "tip": [None, None]}
    ],
    "base": [[45, None], [70, None], [90, None], [110, None], [135, None]],
    "shoulder": [[40, None], [70, None], [90, None], [120, None], [150, None]],
    "elbow": [[30, None], [60, None], [90, None], [120, None], [160, None]],
    "wrist pitch": [[60, None], [90, None], [120, None], [150, None], [180, None]],
    "wrist roll": [[0, None], [45, None], [90, None], [135, None], [180, None]],
}


def angles_from_axes(shoulder, elbow, wrist, tip):
    """Pitch-joint angles and link lengths from four axis positions in the table frame.

    Each argument is (forward, height): forward measured along the table surface from the
    base rotation axis, height measured up from the table surface, with the base at its zero
    so the whole arm lies in one vertical plane.

    Mirrors server.py's fk(): a1, a2 and p are the directions of the upper arm, the forearm
    and the tool, and the joint angles are the turns between them.
    """
    (rs, zs), (re, ze), (rw, zw), (rt, zt) = shoulder, elbow, wrist, tip
    a1 = math.degrees(math.atan2(ze - zs, re - rs))
    a2 = math.degrees(math.atan2(zw - ze, rw - re))
    pp = math.degrees(math.atan2(zt - zw, rt - rw))
    return {
        "shoulder": 90.0 - a1,          # fk: a1 = 90 - shoulder
        "elbow": a1 - a2,               # fk: a2 = a1 - elbow
        "wrist pitch": a2 - pp,         # fk: p  = a2 - wrist
        "links": {"L1": math.dist(shoulder, elbow),
                  "L2": math.dist(elbow, wrist),
                  "L3": math.dist(wrist, tip)},
        "L0": zs,
    }


def pairs_from_axis_poses(poses, shoulder_axis):
    """Turn measured axis poses into per-joint (servo, angle) pairs, plus link stats."""
    SERVO_INDEX = {"shoulder": 1, "elbow": 2, "wrist pitch": 3}
    out = {k: [] for k in SERVO_INDEX}
    links = {"L0": [], "L1": [], "L2": [], "L3": []}
    for i, pose in enumerate(poses):
        try:
            servo = pose["servo"]
            got = angles_from_axes(shoulder_axis, pose["elbow"], pose["wrist"], pose["tip"])
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"axis_poses[{i}] is malformed: {e}") from e
        for name, idx in SERVO_INDEX.items():
            if idx < len(servo) and servo[idx] is not None:
                out[name].append([float(servo[idx]), got[name]])
        links["L0"].append(got["L0"])
        for k, v in got["links"].items():
            links[k].append(v)
    return out, links


def fit_line(xs, ys):
    """Least-squares fit of y = m*x + c. Returns (m, c, r2, residuals)."""
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-12:
        raise ValueError("all servo values identical - cannot fit a slope")
    m = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    c = my - m * mx
    res = [y - (m * x + c) for x, y in zip(xs, ys)]
    syy = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - sum(r * r for r in res) / syy if syy > 1e-12 else 1.0
    return m, c, r2, res


def fit_joint(name, pairs, plate_tilt=0.0):
    pts = [(float(s), float(a)) for s, a in pairs if a is not None]
    if len(pts) < 2:
        return None, f"{name}: needs at least 2 measured points, has {len(pts)}"
    if name in PITCH_JOINTS and plate_tilt:
        # readings are gravity-referenced; joint angles are relative to the base plate
        pts = [(s, a - plate_tilt) for s, a in pts]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    try:
        m, c, r2, res = fit_line(xs, ys)
    except ValueError as e:
        # contract is (result, message) - a stuck joint should read as a message, not a traceback
        return None, f"{name}: {e}"

    if abs(m) < 1e-9:
        return None, f"{name}: fitted slope is zero - did the joint actually move?"
    sign = 1 if m > 0 else -1
    gain = abs(m)
    zero = -c / m
    out = {"name": name, "zero": round(zero, 2), "sign": sign, "gain": round(gain, 4)}
    notes = []
    if len(pts) < 3:
        notes.append("only 2 points: no way to tell a bad measurement from a good one")
    if r2 < 0.999:
        notes.append(f"r2 {r2:.5f} is low - the joint may not be linear, or a reading is off")
    worst = max(abs(r) for r in res)
    if worst > 1.5:
        notes.append(f"worst residual {worst:.2f} deg - check the outlying measurement")
    if not 0.5 <= gain <= 1.2:
        notes.append(f"gain {gain:.3f} is far from 1.0 - plausible for this rig, but re-check "
                     "that you measured actual angle and not commanded")
    return {"fit": out, "r2": r2, "worst": worst, "n": len(pts), "notes": notes}, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--template", action="store_true", help="write a blank servo_cal.json")
    ap.add_argument("--apply", action="store_true", help="write the fitted values into config.json")
    ap.add_argument("--plate-tilt", type=float, default=0.0,
                    help="base plate tilt in degrees, subtracted from pitch-joint readings")
    ap.add_argument("--base-from-markers", action="store_true",
                    help="show the marker azimuths for calibrating the base without a protractor")
    a = ap.parse_args()

    if a.base_from_markers:
        print("Jog the tip onto each marker centre, note the BASE servo value, and enter the")
        print("pair [servo, azimuth] under \"base\" in servo_cal.json:\n")
        for mid, az in sorted(MARKER_AZIMUTH.items()):
            print(f"  ID{mid}: true azimuth {az:+7.2f} deg")
        print("\nThese come from the hand-measured marker positions in vision.json, so they are")
        print("independent of the arm - which is what makes them usable as a reference.")
        return 0

    if a.template:
        if CAL_PATH.exists():
            print(f"{CAL_PATH.name} already exists - not overwriting it.")
            return 1
        CAL_PATH.write_text(json.dumps(TEMPLATE, indent=2))
        print(f"Wrote {CAL_PATH.name}. Replace each null with the measured angle, drop the")
        print("rows you did not measure, then run this script again.")
        return 0

    if not CAL_PATH.exists():
        print(f"No {CAL_PATH.name}. Run with --template to create one.")
        return 1

    cal = json.loads(CAL_PATH.read_text())
    if a.plate_tilt:
        print(f"Subtracting a base-plate tilt of {a.plate_tilt:+.2f} deg from the pitch joints "
              f"({', '.join(sorted(PITCH_JOINTS))}).\n")
    else:
        print("No --plate-tilt given. If the pitch angles were measured with a gravity-referenced\n"
              "tool and the base plate is not level, every zero below carries the desk's slope.\n")

    cfg = json.loads(CFG_PATH.read_text())
    by_name = {j["name"]: j for j in cfg["joints"]}
    results, failed = {}, []

    # axis measurements are plate-relative already, so the tilt must not be applied to them
    axis_pairs, axis_tilt = {}, 0.0
    poses = [q for q in cal.get("axis_poses", [])
             if all(v is not None for k in ("elbow", "wrist", "tip") for v in q.get(k, [None]))]
    if poses:
        sh = cal.get("shoulder_axis")
        if not sh or any(v is None for v in sh):
            failed.append("axis_poses given but shoulder_axis is missing")
        else:
            try:
                axis_pairs, links = pairs_from_axis_poses(poses, sh)
                print(f"{len(poses)} axis pose(s) measured in the table frame - these are already "
                      "plate-relative, so no tilt correction is applied to them.\n")
                print("link lengths implied by those measurements:")
                cl = cfg["links"]
                for k in ("L0", "L1", "L2", "L3"):
                    vs = links[k]
                    if not vs:
                        continue
                    mean = sum(vs) / len(vs)
                    spread = max(vs) - min(vs)
                    flag = ""
                    if abs(mean - cl.get(k, mean)) > 3.0:
                        flag = f"  <-- config.json says {cl.get(k)}"
                    print(f"  {k} {mean:7.1f} mm   spread across poses {spread:5.1f} mm{flag}")
                print("  (a large spread means an axis position was misread, not that the link "
                      "changed length)\n")
            except ValueError as e:
                failed.append(str(e))

    for name, pairs in list(cal.items()) + [(k, v) for k, v in axis_pairs.items() if v]:
        if name.startswith("_") or name in ("axis_poses", "shoulder_axis"):
            continue
        if name in axis_pairs and axis_pairs[name] and name in cal:
            continue        # prefer the axis-derived pairs over duplicate angle entries
        tilt = 0.0 if (name in axis_pairs and pairs is axis_pairs[name]) else a.plate_tilt
        got, err = fit_joint(name, pairs, tilt)
        if err:
            failed.append(err)
            continue
        results[name] = got
        f = got["fit"]
        cur = by_name.get(name, {})
        print(f"{name}")
        print(f"  {got['n']} points, r2 {got['r2']:.5f}, worst residual {got['worst']:.2f} deg")
        print(f"  zero {f['zero']:8.2f}   (was {cur.get('zero', '?')})")
        print(f"  sign {f['sign']:8d}   (was {cur.get('sign', '?')})")
        print(f"  gain {f['gain']:8.4f}   (was {cur.get('gain', '?')})")
        if f["gain"] < 0.999:
            print(f"         -> the joint turns {f['gain']:.3f} deg per commanded deg, so its full "
                  f"0-180 command spans only {180 * f['gain']:.0f} deg")
        for n in got["notes"]:
            print(f"  WARNING: {n}")
        print()

    for e in failed:
        print(f"skipped - {e}")

    if not results:
        print("Nothing was fitted.")
        return 1

    if a.apply:
        for name, got in results.items():
            j = by_name.get(name)
            if j is None:
                print(f"{name} is not in config.json - skipped")
                continue
            j.update({k: got["fit"][k] for k in ("zero", "sign", "gain")})
        CFG_PATH.write_text(json.dumps(cfg, indent=2))
        print(f"Written into {CFG_PATH.name}.")
        print("The joint min/max limits are in SERVO units and were not touched - re-check them,")
        print("because a changed zero moves where those limits actually land.")
    else:
        print("Re-run with --apply to write these into config.json.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
