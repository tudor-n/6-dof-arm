#!/usr/bin/env python3
"""Arm brain: inverse kinematics + web UI + serial link to the ESP32 + camera vision.
   pip install aiohttp pyserial opencv-contrib-python numpy ultralytics
   python server.py --port COM5            real arm + camera 0
   python server.py --sim                  no arm (camera still works, handy for checking vision)
   python server.py --port COM5 --camera 1 / --no-camera
"""
import argparse, asyncio, collections, copy, json, math, time
from pathlib import Path
from aiohttp import web, WSMsgType
try:
    import serial, serial.tools.list_ports
except ImportError:
    serial = None

HERE = Path(__file__).parent
CFG_PATH = HERE / "config.json"
START_POSE = [90.0, 16.0, 10.0, 72.0, 90.0, 100.0]  # must match firmware power-on pose
DEFAULT_CFG = {
    # millimetres - MEASURE THESE (or edit in the Calibrate tab)
    "links": {"L0": 80.0,    # table -> shoulder axis height
              "L1": 105.0,   # shoulder axis -> elbow axis
              "L2": 100.0,   # elbow axis -> wrist-pitch axis
              "L3": 150.0},  # wrist-pitch axis -> gripper tip
    # servo = zero + sign * joint_angle   (only first 4 joints use zero/sign)
    # low-stress rest pose, in JOINT angles (deg): upper arm ~vertical, forearm folded down/in, gripper tucked
    "rest": {"base": 0, "shoulder": -10, "elbow": 145, "wrist": 50},
    "joints": [
        {"name": "base",        "zero": 90, "sign": 1, "min": 5,  "max": 175},
        {"name": "shoulder",    "zero": 90, "sign": 1, "min": 15, "max": 165},
        {"name": "elbow",       "zero": 90, "sign": 1, "min": 10, "max": 170},
        {"name": "wrist pitch", "zero": 90, "sign": 1, "min": 10, "max": 170},
        {"name": "wrist roll",  "zero": 90, "sign": 1, "min": 0,  "max": 180},
        {"name": "gripper",     "zero": 90, "sign": 1, "min": 20, "max": 100},
    ],
}

def load_cfg():
    cfg = copy.deepcopy(DEFAULT_CFG)
    if CFG_PATH.exists():
        try: cfg.update(json.loads(CFG_PATH.read_text()))
        except Exception as e: print("config.json unreadable, using defaults:", e)
    return cfg

def save_cfg(): CFG_PATH.write_text(json.dumps(S.cfg, indent=2))

# ---------------- kinematics ----------------

def joint_angles(servo, cfg):
    J = cfg["joints"]
    return [J[i]["sign"] * (servo[i] - J[i]["zero"]) for i in range(4)]

def fk(servo, cfg):
    L = cfg["links"]
    yaw, al, be, ga = [math.radians(a) for a in joint_angles(servo, cfg)]
    a1 = math.pi / 2 - al; a2 = a1 - be; p = a2 - ga
    sh = (0.0, L["L0"])
    el = (sh[0] + L["L1"] * math.cos(a1), sh[1] + L["L1"] * math.sin(a1))
    wr = (el[0] + L["L2"] * math.cos(a2), el[1] + L["L2"] * math.sin(a2))
    tip = (wr[0] + L["L3"] * math.cos(p), wr[1] + L["L3"] * math.sin(p))
    pts = [[r * math.cos(yaw), r * math.sin(yaw), z] for r, z in [(0, 0), sh, el, wr, tip]]
    pitch = (math.degrees(p) + 180) % 360 - 180
    return pts, pitch

def fk_planar(servo, cfg):
    L = cfg["links"]
    _, al, be, ga = [math.radians(a) for a in joint_angles(servo, cfg)]
    a1 = math.pi / 2 - al; a2 = a1 - be; p = a2 - ga
    sh = (0.0, L["L0"]); el = (L["L1"] * math.cos(a1), sh[1] + L["L1"] * math.sin(a1))
    wr = (el[0] + L["L2"] * math.cos(a2), el[1] + L["L2"] * math.sin(a2))
    tip = (wr[0] + L["L3"] * math.cos(p), wr[1] + L["L3"] * math.sin(p))
    return [[0.0, 0.0], list(sh), list(el), list(wr), list(tip)]

def ik(x, y, z, pitch, cfg):
    L = cfg["links"]; L0, L1, L2, L3 = L["L0"], L["L1"], L["L2"], L["L3"]
    r = math.hypot(x, y)
    yaw = math.atan2(y, x) if r > 1e-6 else 0.0
    p = math.radians(pitch)
    wr = r - L3 * math.cos(p); wz = z - L0 - L3 * math.sin(p)
    d = math.hypot(wr, wz); reach = False
    hi = (L1 + L2) * 0.9995; lo = abs(L1 - L2) + 1e-3
    if d > hi: wr, wz, reach = wr * hi / d, wz * hi / d, True
    elif d < lo:
        reach = True
        if d < 1e-6: wr, wz = lo, 0.0
        else: wr, wz = wr * lo / d, wz * lo / d
    c = (wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    be = math.acos(max(-1.0, min(1.0, c)))
    a1 = math.atan2(wz, wr) + math.atan2(L2 * math.sin(be), L1 + L2 * math.cos(be))
    a2 = a1 - be
    ga = a2 - p
    ang = [math.degrees(yaw), 90 - math.degrees(a1), math.degrees(be), (math.degrees(ga) + 180) % 360 - 180]
    servo, limited = [], False
    for i in range(4):
        J = cfg["joints"][i]
        s = J["zero"] + J["sign"] * ang[i]
        # Sort limits before clamping to handle inverted bounds (min > max)
        _lo, _hi = min(J["min"], J["max"]), max(J["min"], J["max"])
        c2 = max(_lo, min(_hi, s))
        limited |= abs(c2 - s) > 1e-6
        servo.append(c2)
    return servo, reach, limited

def joint_ranges(cfg):
    J = cfg["joints"]; R = []
    for i in range(4):
        a = (J[i]["min"] - J[i]["zero"]) * J[i]["sign"]; b = (J[i]["max"] - J[i]["zero"]) * J[i]["sign"]
        R.append((min(a, b), max(a, b)))
    return R

def ik_auto(x, y, z, p0, cfg):
    """Tries the exact target first. If that is impossible it SHRINKS the target toward the base (up to ~66%)
    and/or changes the tool pitch. The second return value is True when the target was changed."""
    L0 = cfg["links"]["L0"]
    for i in range(0, 24):
        k = 1.0 - 0.03 * i
        xx, yy, zz = x * k, y * k, L0 + (z - L0) * k
        step = 1 if i == 0 else 4
        for dp in range(0, 226, step):
            for sg in ((1, -1) if dp else (1,)):
                p = p0 + sg * dp
                if p > 180 or p < -180: continue
                servo, reach, lim = ik(xx, yy, zz, p, cfg)
                if not reach and not lim: return servo, i > 0, False
    servo, reach, lim = ik(x, y, z, p0, cfg)
    return servo, True, lim

# ---------------- state ----------------
class S:
    cfg = load_cfg(); ser = None
    cur = list(START_POSE); tgt = list(START_POSE)
    live = False; speed = 60.0
    tool = {}; reach = False; limited = False
    clients = set(); port = None
    engaged = False; unlock = False; creep = None
    pick_task = None

V = None   # Vision instance (or None when the camera is off)

# Automatically sort hi/lo bounds to tolerate inverted calibration
def clamp(v, lo, hi):
    _lo, _hi = min(lo, hi), max(lo, hi)
    return max(_lo, min(_hi, v))

# ---------------- log (shown in the UI, also printed) ----------------
LOGQ = collections.deque(maxlen=300)   # replayed to a freshly opened browser
NEWLOGS = []                           # drained by control_loop and broadcast

def log(msg, lvl="info"):
    e = {"type": "log", "t": time.strftime("%H:%M:%S"), "lvl": lvl, "msg": msg}
    print(f"[{lvl}] {msg}"); LOGQ.append(e); NEWLOGS.append(e)

def tool_from_tgt():
    pts, pitch = fk(S.tgt, S.cfg)
    x, y, z = pts[-1]
    return {"type": "tool", "x": x, "y": y, "z": z, "pitch": pitch, "roll": S.tgt[4], "grip": S.tgt[5]}

def tool_state():
    d = tool_from_tgt(); d.pop("type"); return d

def solve():
    t = S.tool
    servo, S.reach, S.limited = ik(t["x"], t["y"], t["z"], t["pitch"], S.cfg)
    S.tgt[:4] = servo

def move_tip(x, y, z, lock=False, pitch=None, who="tip", strict=False, verbose=True):
    """Solve a tip target and write it to S.tgt. Returns True when the arm will really be where you asked.
    strict=True: refuse (and do not move) when IK had to change the target or hit a joint limit."""
    zr = z
    x, z = max(0.0, x), max(0.0, z)          # same clamps as before
    p0 = fk(S.tgt, S.cfg)[1] if pitch is None else pitch
    servo, reach, lim = ik(x, y, z, p0, S.cfg) if lock else ik_auto(x, y, z, p0, S.cfg)
    old = S.tgt[:4]; S.tgt[:4] = servo
    got, gp = fk(S.tgt, S.cfg); gx, gy, gz = got[-1]
    err = max(abs(gx - x), abs(gy - y), abs(gz - z))
    bad = reach or lim or err > 2.0
    if strict and bad:
        S.tgt[:4] = old
    else:
        S.reach, S.limited = reach, lim
    if verbose:
        m = f"{who}: want ({x:.1f}, {y:.1f}, {z:.1f}) -> IK gives ({gx:.1f}, {gy:.1f}, {gz:.1f}), tool pitch {gp:.0f}"
        if zr < 0: m += f" | z {zr:.1f} was below 0 and was raised to 0"
        if reach: m += " | TARGET CHANGED to reach it (moved toward the base / pitch changed)"
        if lim: m += " | JOINT LIMIT hit"
        if err > 2.0 and not reach and not lim: m += f" | miss {err:.1f} mm"
        log(m, "warn" if bad else "info")
    return not bad

async def wait_arrived(timeout=8.0, tol=0.8):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if max(abs(S.tgt[i] - S.cur[i]) for i in range(5)) < tol:
            await asyncio.sleep(0.3); return True
        await asyncio.sleep(0.05)
    log(f"arm did not settle within {timeout:.0f}s (is Live on? speed too low?)", "warn")
    return False

def tip_azimuth(servo=None):
    x, y, _ = fk(S.tgt if servo is None else servo, S.cfg)[0][-1]
    return math.degrees(math.atan2(y, x))

def world_to_servo_roll(closing_deg, az_deg):
    """Jaw-closing direction in table coordinates -> roll servo value, using the calibrated zero and sign."""
    R = V.roll; J = S.cfg["joints"][4]
    lo, hi = min(J["min"], J["max"]), max(J["min"], J["max"])
    rel = ((closing_deg - az_deg + 90.0) % 180.0) - 90.0
    cands = [R["zero"] + R["sign"] * (rel + k) for k in (0, 180, -180)]
    ok = [c for c in cands if lo <= c <= hi]
    return min(ok, key=lambda c: abs(c - R["zero"])) if ok else clamp(cands[0], lo, hi)

def do_stop():
    if S.pick_task and not S.pick_task.done(): S.pick_task.cancel()
    S.creep = None; S.tgt = list(S.cur); S.tgt[5] = 90.0; S.cur[5] = 90.0
    log("STOP: holding position, gripper braked", "warn")

def start_pick(x, y, closing):
    if V is None: log("Pick needs the camera/vision module.", "err"); return
    if S.pick_task and not S.pick_task.done(): log("A pick is already running (press STOP to cancel).", "warn"); return
    S.pick_task = asyncio.create_task(pick_seq(x, y, closing))

async def pick_seq(x, y, closing):
    s = V.s
    try:
        if not S.live: log("Pick refused: Live is OFF, the arm would not move.", "err"); return
        tz = V.table_z(x, y)
        want_g = tz + s["z_grasp_offset"]; grasp = max(s["z_floor"], want_g); hover = tz + s["hover"]
        log(f"PICK start: object ({x:.1f}, {y:.1f}), table z {tz:.1f} -> hover z {hover:.0f}, grasp z {grasp:.1f}")
        if grasp != want_g:
            log(f"grasp z would be {want_g:.1f} (table {tz:.1f} {s['z_grasp_offset']:+.0f}) but the floor is {s['z_floor']:.1f}, so the tip goes to {grasp:.1f}", "warn")
        S.tgt[5] = clamp(s["grip_open"], 0, 180)
        if closing is not None:
            az = math.degrees(math.atan2(y, x)); r = world_to_servo_roll(closing, az); S.tgt[4] = r
            log(f"roll: jaws should close along {closing:.0f} deg (table), arm azimuth {az:.0f} deg -> roll servo {r:.0f}")
        await asyncio.sleep(0.8)
        pp = s["pick_pitch"]
        move_tip(x, y, hover, pitch=pp, who="hover"); await wait_arrived()
        if not move_tip(x, y, grasp, pitch=pp, who="descend", strict=True):
            log("PICK ABORTED: the grasp point cannot be reached exactly. Try a different object position, "
                "pick pitch, hover height or z offset.", "err"); return
        await wait_arrived()
        S.tgt[5] = clamp(s["grip_close"], 0, 180); log("closing gripper"); await asyncio.sleep(1.5)
        move_tip(x, y, hover, pitch=pp, who="lift"); await wait_arrived()
        move_tip(s["drop_x"], s["drop_y"], s["drop_z"], pitch=pp, who="drop"); await wait_arrived()
        S.tgt[5] = clamp(s["grip_open"], 0, 180); await asyncio.sleep(1.0)
        log("PICK finished", "ok")
    except asyncio.CancelledError:
        log("pick cancelled", "warn"); raise
    except Exception as e:
        log(f"pick crashed: {e}", "err")

def record_marker(mid):
    if V is None: return
    if not S.live: log("Turn Live ON, jog the tip onto the marker centre, then record.", "err"); return
    if max(abs(S.tgt[i] - S.cur[i]) for i in range(5)) > 1.0: log("Arm is still moving - wait, then record.", "warn"); return
    pts, pitch = fk(S.cur, S.cfg); x, y, z = pts[-1]
    V.record_marker(mid, (x, y, z))
    log(f"Marker {mid} recorded: tip at X={x:.1f} Y={y:.1f} Z={z:.1f} (tool pitch {pitch:.0f})", "ok")

def push_limits():
    if S.ser is None: return
    try:
        for i, j in enumerate(S.cfg["joints"]):
            lo, hi = (0, 180) if S.unlock else (j["min"], j["max"])

            # --- FORCE FULL RANGE FOR 360 GRIPPER ---
            if i == 5:
                lo, hi = 0.0, 180.0

            S.ser.write(f"L {i} {lo:.1f} {hi:.1f}\n".encode())
    except Exception as e: print("serial error:", e)

def lims(i):
    j = S.cfg["joints"][i]
    return (0.0, 180.0) if S.unlock else (float(j["min"]), float(j["max"]))

def serial_send(angles):
    if S.ser is None: return
    try: S.ser.write(("J " + " ".join(f"{a:.1f}" for a in angles) + "\n").encode())
    except Exception as e:
        print("serial error:", e, "- will try to reconnect")
        try: S.ser.close()
        except Exception: pass
        S.ser = None; S.live = False

async def reconnect_loop():
    while True:
        await asyncio.sleep(2)
        if S.ser is None and S.port:
            try:
                ser, pose = await asyncio.get_running_loop().run_in_executor(None, connect_serial, S.port)
            except Exception:
                continue
            S.ser = ser; push_limits()
            if pose and len(pose) == 6:
                S.cur = pose; S.tgt = list(pose); S.live = False
                S.tool = tool_from_tgt(); await broadcast(S.tool)
            log("Serial reconnected. Live is OFF - turn it on again.", "warn")

async def broadcast(msg):
    for ws in list(S.clients):
        try: await ws.send_json(msg)
        except Exception: S.clients.discard(ws)

async def control_loop():
    last = time.time(); last_sent = None; tick = 0
    while True:
        await asyncio.sleep(0.02)
        now = time.time(); dt = now - last; last = now
        if S.creep:
            i, d, dl = S.creep
            if now > dl: S.creep = None
            else:
                S.tgt[i] = clamp(S.tgt[i] + d * 4.0 * dt, 0, 180)
                S.tgt[i] = clamp(S.tgt[i], S.cur[i] - 3, S.cur[i] + 3)
        if S.live:
            max_dist = max(abs(S.tgt[i] - S.cur[i]) for i in range(6))
            if max_dist > 0.1:
              for i in range(6):
                dist = S.tgt[i] - S.cur[i]
                joint_speed_limit = (S.speed * dt) * (abs(dist) / max_dist)

                # Gripper (5) gets no decay smoothing - strictly bound by speed slider
                if i == 5:
                    step = clamp(dist, -joint_speed_limit, joint_speed_limit)
                else:
                    step = clamp(dist * 6.0 * dt, -joint_speed_limit, joint_speed_limit)

                if abs(dist) < 0.1:
                    S.cur[i] = S.tgt[i]
                else:
                    S.cur[i] += step
            if last_sent != S.cur:
                serial_send(S.cur)
                last_sent = list(S.cur)
        if V is not None:   # feed the camera overlay with where the arm thinks its tip is
            tip = fk(S.cur, S.cfg)[0][-1]; V.tip_cur = tip; V.tip_tgt = fk(S.tgt, S.cfg)[0][-1]
            R = V.roll; V.jaw_deg = (math.degrees(math.atan2(tip[1], tip[0])) + R["sign"] * (S.cur[4] - R["zero"])) % 180.0
        tick += 1
        while NEWLOGS: await broadcast(NEWLOGS.pop(0))
        if tick % 2 == 0 and S.clients:
            await broadcast({"type": "state", "cur": S.cur, "tgt": S.tgt,
                             "pts_cur": fk(S.cur, S.cfg)[0], "pts_tgt": fk(S.tgt, S.cfg)[0],
                             "pl_cur": fk_planar(S.cur, S.cfg), "pl_tgt": fk_planar(S.tgt, S.cfg),
                             "ang": joint_angles(S.tgt, S.cfg), "ranges": [[round(a), round(b)] for a, b in joint_ranges(S.cfg)], "tool": tool_state(),
                             "live": S.live, "speed": S.speed, "reach": S.reach, "limited": S.limited,
                             "hw": S.ser is not None, "engaged": S.engaged, "lost": S.ser is None and S.port is not None})
        if tick % 10 == 0 and S.clients and V is not None:
            st = V.status(); st["busy"] = bool(S.pick_task and not S.pick_task.done())
            await broadcast(st)

async def handle(ws, m):
    t = m.get("type")
    if t == "target":
        S.tool = {k: float(m[k]) for k in ("x", "y", "z", "pitch")}
        S.tool["x"] = max(0.0, S.tool["x"])
        S.tgt[4] = clamp(float(m["roll"]), S.cfg["joints"][4]["min"], S.cfg["joints"][4]["max"])
        S.tgt[5] = clamp(float(m["grip"]), S.cfg["joints"][5]["min"], S.cfg["joints"][5]["max"])
        solve()
    elif t == "tip":
        move_tip(float(m["x"]), float(m["y"]), float(m["z"]), lock=bool(m.get("lock")),
                 pitch=m.get("pitch"), who="tip", verbose=bool(m.get("log")))

    elif t == "joints":
        J = S.cfg["joints"]; ang = joint_angles(S.tgt, S.cfg); lim = False
        for i, k in enumerate(("yaw", "alpha", "beta", "gamma")):
            if k in m: ang[i] = float(m[k])
        for i in range(4):
            v = J[i]["zero"] + J[i]["sign"] * ang[i]; c = clamp(v, J[i]["min"], J[i]["max"])
            lim |= abs(c - v) > 1e-6; S.tgt[i] = c
        S.limited = lim; S.reach = False

    elif t == "pitch":
        tip = tool_state()
        servo, S.reach, S.limited = ik(tip["x"], tip["y"], tip["z"], float(m["pitch"]), S.cfg)
        S.tgt[:4] = servo
    elif t in ("roll", "grip"):
        i = 4 if t == "roll" else 5
        if i == 5:
            # 360 servo needs the full 0-180 range for speed control
            S.tgt[i] = clamp(float(m["angle"]), 0.0, 180.0)
        else:
            J = S.cfg["joints"][i]
            S.tgt[i] = clamp(float(m["angle"]), J["min"], J["max"])
    elif t == "raw":
        i = int(m["servo"]); lo, hi = lims(i); S.tgt[i] = clamp(float(m["angle"]), lo, hi)
    elif t == "nudge":
        i = int(m["servo"]); lo, hi = lims(i); d = float(m["delta"])
        if m.get("joint") and i < 4: d *= S.cfg["joints"][i]["sign"]
        S.tgt[i] = clamp(S.tgt[i] + d, lo, hi)
    elif t == "jointpos":
        i = int(m["joint"]); J = S.cfg["joints"][i]; lo, hi = lims(i)
        S.tgt[i] = clamp(J["zero"] + J["sign"] * float(m["angle"]), lo, hi)
    elif t == "unlock":
        S.unlock = bool(m["on"]); S.creep = None; push_limits()
    elif t == "creep":
        i, d = int(m["servo"]), int(m["dir"])
        if d != 0: S.creep = (i, d, time.time() + 0.4)
        elif S.creep and S.creep[0] == i:
            dd = S.creep[1]; S.creep = None; J = S.cfg["joints"][i]; back = 8.0
            if dd < 0: J["min"] = round(clamp(S.cur[i] + back, 0, 180), 1); S.tgt[i] = J["min"]
            else:      J["max"] = round(clamp(S.cur[i] - back, 0, 180), 1); S.tgt[i] = J["max"]
            if J["min"] > J["max"] - 10: J["min"], J["max"] = min(J["min"], J["max"] - 10), J["max"]
            save_cfg(); push_limits(); await broadcast({"type": "cfg", "cfg": S.cfg})
    elif t == "resync":
        S.tool = tool_from_tgt(); await ws.send_json(S.tool)
    elif t == "rest":
        R, J, servo, clipped = S.cfg["rest"], S.cfg["joints"], [], False
        for i, a in enumerate([R["base"], R["shoulder"], R["elbow"], R["wrist"]]):
            v = J[i]["zero"] + J[i]["sign"] * a; c = clamp(v, J[i]["min"], J[i]["max"])
            clipped |= abs(c - v) > 1e-6; servo.append(round(c, 1))
        S.tgt[:4] = servo; S.tgt[4] = 90.0
        S.tool = tool_from_tgt(); await broadcast(S.tool)
        await broadcast({"type": "restinfo", "servo": S.tgt[:5] + [S.tgt[5]], "clipped": clipped})
    elif t == "relax":
        if S.ser:
            try: S.ser.write(f"R {int(m['mask']) & 63}\n".encode()); S.engaged = True
            except Exception as e: print("serial error:", e)
    elif t == "live":
        S.live = bool(m["on"])
        log("Live " + ("ON" if S.live else "OFF"), "ok" if S.live else "warn")
        if S.live and S.ser:
            try: S.ser.write(b"R 0\n"); S.engaged = True; push_limits()
            except Exception as e: print("serial error:", e)
    elif t == "speed": S.speed = clamp(float(m["dps"]), 5, 180)
    elif t == "home":
        S.tgt = list(START_POSE); S.tool = tool_from_tgt(); await broadcast(S.tool)
    elif t in ("setzero", "flip", "setlim", "links"):
        if t == "setzero":
            i = int(m["joint"]); S.cfg["joints"][i]["zero"] = round(S.tgt[i], 1)
        elif t == "flip":
            i = int(m["joint"]); S.cfg["joints"][i]["sign"] *= -1
        elif t == "setlim":
            i = int(m["joint"]); S.cfg["joints"][i][m["which"]] = round(S.tgt[i], 1)
        else:
            for k, v in m["links"].items():
                if k in S.cfg["links"] and float(v) > 0: S.cfg["links"][k] = float(v)
        save_cfg(); push_limits()
        await broadcast({"type": "cfg", "cfg": S.cfg})

    # ---------- camera / vision / pick ----------
    elif t == "stop": do_stop()
    elif t == "jog":
        tip = tool_state()
        move_tip(tip["x"] + float(m.get("dx", 0)), tip["y"] + float(m.get("dy", 0)), tip["z"] + float(m.get("dz", 0)),
                 lock=True, who="jog")
    elif V is None and t in ("cam_click", "goto_click", "pick_click", "pick_detected", "vset", "detect", "pose_lock",
                             "marker_record", "marker_apply", "marker_reset", "roll_zero", "roll_flip", "jaws"):
        log("Vision is not running (camera files missing, --no-camera, or OpenCV not installed).", "err")
    elif t == "cam_click":
        r = V.click_at(float(m["u"]), float(m["v"]))
        if r: log(f"click -> table X={r['x']:.1f} Y={r['y']:.1f} (table z {r['z']:.1f})")
        else: log("click ignored: camera pose is not found yet (all 4 markers must be visible).", "warn")
    elif t == "goto_click":
        if V.click is None: log("Click a point on the video first.", "warn")
        else:
            x, y = V.click[0], V.click[1]; tz = V.table_z(x, y)
            z = tz + V.s["hover"] if m.get("mode") == "hover" else max(V.s["z_floor"], tz + V.s["z_grasp_offset"])
            move_tip(x, y, z, pitch=V.s["pick_pitch"], who="go to click (" + str(m.get("mode")) + ")")
    elif t == "pick_click":
        if V.click is None: log("Click a point on the video first.", "warn")
        else: start_pick(V.click[0], V.click[1], None)
    elif t == "pick_detected":
        b = V.best_target()
        if not b: log("Nothing detected. Turn on Detect and make sure the object is visible.", "warn")
        else:
            log(f"picking {b['name']} ({b['conf']:.2f}) at ({b['x']:.1f}, {b['y']:.1f})")
            start_pick(b["x"], b["y"], b["closing"])
    elif t == "vset": V.set_setting(m["key"], m.get("value"))
    elif t == "detect":
        V.detect_on = bool(m["on"])
        if m.get("prompt", "").strip():
            V.prompts = [p.strip() for p in m["prompt"].split(",") if p.strip()]; log("detect prompts: " + ", ".join(V.prompts))
    elif t == "pose_lock":
        V.pose_locked = bool(m["on"] and V.rvec is not None)
        if not V.pose_locked: V.hist.clear()
        log("Camera pose " + ("LOCKED (frozen)" if V.pose_locked else "live (re-solved every frame)"))
    elif t == "marker_record": record_marker(int(m["id"]))
    elif t == "marker_apply":
        ok, info = V.apply_touched(); log(("Markers updated. " if ok else "") + info, "ok" if ok else "err")
    elif t == "marker_reset": V.reset_markers(); log("Markers reset to the old hand-measured defaults.", "warn")
    elif t == "roll_zero":
        V.roll["zero"] = round(S.cur[4], 1); V.save(); log(f"Roll zero set: servo {V.roll['zero']} = jaws close along the arm")
    elif t == "roll_flip":
        V.roll["sign"] *= -1; V.save(); log(f"Roll direction: servo up turns the jaws {'CCW' if V.roll['sign'] > 0 else 'CW'} seen from above")
    elif t == "jaws":
        r = world_to_servo_roll(float(m["angle"]), tip_azimuth()); S.tgt[4] = r
        log(f"jaws -> close along {float(m['angle']):.0f} deg (table), arm azimuth {tip_azimuth():.0f} -> roll servo {r:.0f}")

async def ws_handler(req):
    ws = web.WebSocketResponse(); await ws.prepare(req); S.clients.add(ws)
    await ws.send_json({"type": "cfg", "cfg": S.cfg}); await ws.send_json(tool_from_tgt())
    for e in list(LOGQ): await ws.send_json(e)
    async for msg in ws:
        if msg.type == WSMsgType.TEXT:
            try: await handle(ws, json.loads(msg.data))
            except Exception as e: log(f"bad message {msg.data[:80]}: {e}", "err")
    S.clients.discard(ws)
    if not S.clients: S.unlock = False; S.creep = None; push_limits()
    return ws

async def index(req):
    html = (HERE / "index.html").read_text(encoding="utf-8")
    html = html.replace("</body>", '<script src="/cam_panel.js"></script></body>')
    return web.Response(text=html, content_type="text/html")

async def video(req):
    resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame", "Cache-Control": "no-cache"})
    await resp.prepare(req); last = -1
    try:
        while V is not None:
            if V.jpg is not None and V.seq != last:
                last = V.seq; jpg = V.jpg
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg) + jpg + b"\r\n")
            await asyncio.sleep(0.03)
    except Exception:
        pass
    return resp

def connect_serial(port):
    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(2.5); ser.reset_input_buffer()
    for _ in range(5):
        ser.write(b"GET\n"); t0 = time.time()
        while time.time() - t0 < 0.6:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("S "):
                tok = line[2:].split()
                S.engaged = len(tok) > 6 and tok[6] == "E1"
                return ser, [float(v) for v in tok[:6]]
    return ser, None

def main():
    global V
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--sim", action="store_true", help="no arm hardware (camera still runs)")
    ap.add_argument("--http", type=int, default=8080)
    ap.add_argument("--camera", type=int, default=0, help="camera index")
    ap.add_argument("--no-camera", action="store_true")
    a = ap.parse_args()
    if not a.sim:
        if serial is None: raise SystemExit("pip install pyserial")
        port = a.port
        if not port:
            ports = list(serial.tools.list_ports.comports())
            print("Serial ports:", [p.device for p in ports])
            hits = [p.device for p in ports if any(k in (p.description or "").upper() for k in ("CP210", "CH340", "USB", "UART"))]
            if not hits: raise SystemExit("No ESP32 found. Use --port COMx or --sim")
            port = hits[0]
        print("Connecting to", port); S.port = port
        try: S.ser, pose = connect_serial(port)
        except serial.SerialException as e:
            raise SystemExit(f"\nCannot open {port}: {e}\n-> Another program is probably using it (idf monitor, another server.py, serial monitor).\n   Close it, or unplug/replug the ESP32's USB, then try again.")
        if pose and len(pose) == 6: S.cur = pose; S.tgt = list(pose); print("Arm reports pose:", pose); push_limits()
        else: print("WARNING: ESP32 didn't answer GET - is the new firmware flashed? Assuming start pose.")
    if not a.no_camera:
        try:
            from vision import Vision
            V = Vision(HERE, a.camera, log)
        except Exception as e:
            print("Vision disabled:", e)
    S.tool = tool_from_tgt()
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/cam_panel.js", lambda r: web.FileResponse(HERE / "cam_panel.js"))
    app.router.add_get("/video", video)
    app.router.add_get("/ws", ws_handler)
    async def start(app):
        app["loop"] = asyncio.create_task(control_loop())
        if S.port: app["rc"] = asyncio.create_task(reconnect_loop())
        if V is not None: V.start()
    app.on_startup.append(start)
    print(f"Open http://localhost:{a.http}")
    web.run_app(app, host="0.0.0.0", port=a.http, print=None)

if __name__ == "__main__":
    main()