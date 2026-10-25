"""gestures.py - DUMMY's body language: nod (yes), shake (no), the happy dance, a wave, a bow, a
puzzled head tilt and a sad droop.

Choreography and planning only, and pure, so it is tested offline. server.py owns everything that
knows the arm: it hands plan() the joint calibration, a tip-height function (forward kinematics)
and the lowest the tool may go, and runs the keyframes plan() returns.

A gesture is a list of keyframes, each (seconds, {part: offset}), offsets relative to the pose the
arm is in when it starts, so a gesture works from anywhere and ends where it began. Parts:
  base   base yaw, joint degrees
  down   wrist pitch, joint degrees, positive = tool tips down (the sign is found from the
         arm's own kinematics, never assumed)
  roll   wrist roll, joint degrees
  grip   the 360 gripper's speed command, servo units around its 90 brake (+ opens)
The seconds are minimums: server.py waits for each keyframe to arrive before the next.
"""

FLOOR_MM = 60.0          # the tool stays this far above the table during a gesture
DROP_MM = 40.0           # ...and never goes more than this below where it started
SCALES = (1.0, 0.8, 0.6, 0.4)   # tried in turn until the gesture fits

GESTURES = {
    # yes: dip, up a little, dip again, back
    "nod": [(0.18, {"down": 12}), (0.2, {"down": -4}), (0.18, {"down": 10}), (0.22, {})],
    # no: the base swings side to side, a little less each time
    "shake": [(0.2, {"base": -10}), (0.3, {"base": 10}), (0.26, {"base": -8}), (0.22, {"base": 6}), (0.2, {})],
    # good boy: sway with the head bobbing against it, wiggle, clap, a double nod
    "dance": [(0.35, {"base": 15, "down": -8}), (0.35, {"base": -15, "down": 8}),
              (0.35, {"base": 15, "down": -8}), (0.35, {"base": -15, "down": 8}), (0.3, {}),
              (0.3, {"roll": 25}), (0.3, {"roll": -25}), (0.3, {"roll": 25}), (0.3, {"roll": -25}), (0.25, {}),
              (0.2, {"grip": 30}), (0.2, {"grip": -30}), (0.2, {"grip": 30}), (0.2, {"grip": -30}), (0.15, {}),
              (0.15, {"down": 10}), (0.15, {}), (0.15, {"down": 10}), (0.2, {})],
    # hello / bye: head up, and the wrist waves side to side
    "wave": [(0.25, {"down": -10}), (0.22, {"down": -10, "roll": 30}), (0.3, {"down": -10, "roll": -30}),
             (0.3, {"down": -10, "roll": 30}), (0.3, {"down": -10, "roll": -30}), (0.25, {})],
    # thank you: one deep, slow nod that holds for a moment
    "bow": [(0.5, {"down": 13}), (0.7, {"down": 13}), (0.6, {})],
    # didn't get that: the head tilts to one side and waits
    "huh": [(0.3, {"roll": 22, "down": -4}), (0.8, {"roll": 22, "down": -4}), (0.3, {})],
    # bad boy: sinks slowly, looks away, then comes back
    "sad": [(0.9, {"down": 13}), (0.6, {"down": 13, "base": -6}), (0.9, {"down": 13, "base": -6}), (1.0, {})],
}
TEMPO = {"bow": 0.5, "sad": 0.35}   # x the gesture speed: the solemn ones move slowly
JOINT = {"base": 0, "down": 3, "roll": 4}
DONE = {"nod": "nodded", "shake": "shook his head", "dance": "danced", "wave": "waved", "bow": "bowed",
        "huh": "tilted his head", "sad": "drooped"}
VERB = {"nod": "nod", "shake": "shake his head", "dance": "dance", "wave": "wave", "bow": "bow",
        "huh": "tilt his head", "sad": "droop"}


def _servo(home, offsets, joints, down_sign, scale):
    """Servo targets for one keyframe: home plus the offsets, joint degrees -> servo via sign / gain."""
    s = list(home[:5])
    for part, deg in offsets.items():
        if part == "grip": continue
        i = JOINT[part]; j = joints[i]
        d = deg * scale * (down_sign if part == "down" else 1)
        s[i] = home[i] + j["sign"] * d / (j.get("gain", 1.0) or 1.0)
    return s


def plan(name, home, joints, tip_z, floor_z, size=1.0, clap=True):
    """Keyframes for one gesture from pose `home` (6 servo values).

    joints   the calibration (config.json "joints": zero/sign/gain/min/max)
    tip_z    servo list -> tool-tip height in mm (forward kinematics)
    floor_z  the lowest the tool may go, mm (the table + FLOOR_MM)
    Returns (steps, scale, why): steps is [(seconds, [6 servo targets])] with the gripper None
    where it isn't touched, ending at `home`; or None with why it can't fit anywhere near here."""
    frames = [(t, o) for t, o in GESTURES[name] if clap or "grip" not in o]
    # A gesture that uses the gripper sets it in every keyframe: back to the 90 brake unless that
    # keyframe claps, so the 360 servo is never left driving into its stop.
    grips = any("grip" in o for _, o in frames)
    z0 = tip_z(list(home))
    if z0 < floor_z: return None, 0.0, "the table"       # already lower than a gesture may ever go
    probe = list(home); j3 = joints[3]
    probe[3] = home[3] + j3["sign"] * 5.0 / (j3.get("gain", 1.0) or 1.0)
    down_sign = 1.0 if tip_z(probe) <= z0 else -1.0
    why = ""
    for f in SCALES:
        scale = size * f; steps = []; why = ""
        for t, o in frames:
            s = _servo(home, o, joints, down_sign, scale)
            for i in range(5):
                lo, hi = sorted((float(joints[i]["min"]), float(joints[i]["max"])))
                if not lo - 1e-6 <= s[i] <= hi + 1e-6: why = f"the {joints[i].get('name', 'joint %d' % i)} limit"
            if not why:
                z = tip_z(s + [home[5]])
                if z < floor_z - 1e-6: why = "the table"
                elif z < z0 - DROP_MM - 1e-6: why = "how far the tool would dip"
            if why: break
            steps.append((t, s + [90.0 + o.get("grip", 0) if grips else None]))
        if not why:
            steps[-1] = (steps[-1][0], list(home[:5]) + [90.0 if grips else None])   # exactly home, gripper braked
            return steps, scale, ""
    return None, 0.0, why
