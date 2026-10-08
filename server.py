#!/usr/bin/env python3
"""Arm brain: inverse kinematics + web UI + serial link to the ESP32.
   pip install aiohttp pyserial
   python server.py --port COM5        (or --sim to run with no hardware)
"""
import argparse, asyncio, copy, json, math, time
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
        # FIX: Sort limits before clamping to handle inverted bounds (min > max)
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

# FIX: Automatically sort hi/lo bounds to tolerate inverted calibration
def clamp(v, lo, hi): 
    _lo, _hi = min(lo, hi), max(lo, hi)
    return max(_lo, min(_hi, v))

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

def push_limits():
    if S.ser is None: return
    try:
        for i, j in enumerate(S.cfg["joints"]):
            lo, hi = (0, 180) if S.unlock else (j["min"], j["max"])
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
            print("Serial reconnected. Live is OFF - turn it on again.")

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
        tick += 1
        if tick % 2 == 0 and S.clients:
            await broadcast({"type": "state", "cur": S.cur, "tgt": S.tgt,
                             "pts_cur": fk(S.cur, S.cfg)[0], "pts_tgt": fk(S.tgt, S.cfg)[0],
                             "pl_cur": fk_planar(S.cur, S.cfg), "pl_tgt": fk_planar(S.tgt, S.cfg),
                             "ang": joint_angles(S.tgt, S.cfg), "ranges": [[round(a), round(b)] for a, b in joint_ranges(S.cfg)], "tool": tool_state(),
                             "live": S.live, "speed": S.speed, "reach": S.reach, "limited": S.limited,
                             "hw": S.ser is not None, "engaged": S.engaged, "lost": S.ser is None and S.port is not None})

async def handle(ws, m):
    t = m.get("type")
    if t == "target":
        S.tool = {k: float(m[k]) for k in ("x", "y", "z", "pitch")}
        S.tool["x"] = max(0.0, S.tool["x"])   
        S.tgt[4] = clamp(float(m["roll"]), S.cfg["joints"][4]["min"], S.cfg["joints"][4]["max"])
        S.tgt[5] = clamp(float(m["grip"]), S.cfg["joints"][5]["min"], S.cfg["joints"][5]["max"])
        solve()
    elif t == "tip":            
        x, y, z = max(0.0, float(m["x"])), float(m["y"]), max(5.0, float(m["z"]))
        _, current_pitch = fk(S.tgt, S.cfg)
        
        if m.get("lock"):
            servo, S.reach, S.limited = ik(x, y, z, current_pitch, S.cfg)
        else:
            servo, S.reach, S.limited = ik_auto(x, y, z, current_pitch, S.cfg)
        S.tgt[:4] = servo

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
        i = 4 if t == "roll" else 5; J = S.cfg["joints"][i]
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

async def ws_handler(req):
    ws = web.WebSocketResponse(); await ws.prepare(req); S.clients.add(ws)
    await ws.send_json({"type": "cfg", "cfg": S.cfg}); await ws.send_json(tool_from_tgt())
    async for msg in ws:
        if msg.type == WSMsgType.TEXT:
            try: await handle(ws, json.loads(msg.data))
            except Exception as e: print("bad message:", e)
    S.clients.discard(ws)
    if not S.clients: S.unlock = False; S.creep = None; push_limits()
    return ws

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
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--sim", action="store_true", help="no hardware, UI only")
    ap.add_argument("--http", type=int, default=8080)
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
    S.tool = tool_from_tgt()
    app = web.Application()
    app.router.add_get("/", lambda r: web.FileResponse(HERE / "index.html"))
    app.router.add_get("/ws", ws_handler)
    async def start(app):
        app["loop"] = asyncio.create_task(control_loop())
        if S.port: app["rc"] = asyncio.create_task(reconnect_loop())
    app.on_startup.append(start)
    print(f"Open http://localhost:{a.http}")
    web.run_app(app, host="0.0.0.0", port=a.http, print=None)

if __name__ == "__main__":
    main()