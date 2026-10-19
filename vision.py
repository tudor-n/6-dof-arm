"""vision.py - camera, ArUco pose, table raycast, YOLO-World detection and an annotated MJPEG stream.
Runs in background threads inside server.py. Everything it knows is published through status().
Needs: pip install opencv-contrib-python numpy ultralytics   (ultralytics only if you use Detect)
"""
import json, math, os, threading, time
from collections import deque
from pathlib import Path
import cv2, numpy as np

# Hand-measured layout from the OLD base position. Re-measure with the arm (Camera tab -> Marker calibration).
DEFAULT_MARKERS = {0: [36.1, 185.8, 7.4], 1: [52.1, -181.8, 7.2], 2: [206.2, 263.9, 10.4], 3: [244.7, -236.3, 7.5]}
DEFAULT_SETTINGS = {
    "x_trim": 0.0, "y_trim": 0.0,          # leave at 0 once markers are measured with the arm
    "object_z": 5.0,                       # height above the table plane that the raycast aims at
    "hover": 130.0,                        # hover height above the table plane
    "z_grasp_offset": -34.0,               # grasp z = table z + this (never below z_floor)
    "z_floor": 0.0,                        # the arm will not command a tip lower than this
    "pick_pitch": None,                    # None = keep current tool pitch (as before); -90 = straight down
    "grip_open": 130.0, "grip_close": 50.0,
    "drop_x": 100.0, "drop_y": 200.0, "drop_z": 180.0,
    "show_grid": True, "show_tip": True, "show_markers": True,
}
DEFAULT_ROLL = {"zero": 90.0, "sign": 1}   # servo value where the jaws close along the arm; +1 = servo up turns jaws CCW from above
DISP_W = 960


def fit_rigid_2d(old, new):
    old, new = np.asarray(old, float), np.asarray(new, float)
    co, cn = old.mean(0), new.mean(0)
    U, _, Vt = np.linalg.svd((old - co).T @ (new - cn))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, cn - R @ co


class Vision:
    def __init__(self, here, cam_index, log):
        self.here, self.cam_index, self.log = Path(here), cam_index, log
        self.file = self.here / "vision.json"
        self.markers = {k: list(v) for k, v in DEFAULT_MARKERS.items()}
        self.measured = False
        self.touched = {}
        self.s = dict(DEFAULT_SETTINGS)
        self.roll = dict(DEFAULT_ROLL)
        self.fit_info = ""
        self._load()
        self._fit_plane()
        self.K = self.D = None
        self.Z = np.zeros((4, 1))
        self.problem = ""
        self.warn = []
        for d in (self.here, Path.cwd()):
            try:
                self.K = np.load(d / "camera_matrix.npy"); self.D = np.load(d / "dist_coeffs.npy"); break
            except Exception:
                pass
        if self.K is None:
            self.problem = "camera_matrix.npy / dist_coeffs.npy not found - run camera calibration first"
        import cv2.aruco as aruco
        self.aruco = aruco
        p = aruco.DetectorParameters(); p.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_4X4_50), p)
        # pose
        self.hist = deque(maxlen=15); self.rvec = self.tvec = None
        self.pose_locked = False
        self.err, self.proj, self.seen = {}, {}, []
        # live data
        self.running = False
        self.frame = None; self.res = (0, 0); self.fps = 0.0
        self.jpg = None; self.seq = 0
        self.tip_cur = self.tip_tgt = None; self.jaw_deg = None
        self.click = None          # (x, y, u, v) in full-res pixels
        self.detect_on = False; self.prompts = ["screwdriver", "tool handle", "metal tool", "pen", "pencil"]
        self.dets = []; self.track = deque(maxlen=5)

    # ---------- persistence ----------
    def _load(self):
        try:
            d = json.loads(self.file.read_text())
            self.markers = {int(k): v for k, v in d.get("markers", self.markers).items()}
            self.touched = {int(k): v for k, v in d.get("touched", {}).items()}
            self.measured = bool(d.get("measured", False))
            self.s.update({k: v for k, v in d.get("settings", {}).items() if k in DEFAULT_SETTINGS})
            self.roll.update(d.get("roll", {}))
        except FileNotFoundError:
            pass
        except Exception as e:
            print("vision.json unreadable, using defaults:", e)

    def save(self):
        self.file.write_text(json.dumps({"markers": self.markers, "touched": self.touched, "measured": self.measured,
                                         "settings": self.s, "roll": self.roll}, indent=2))

    def set_setting(self, key, val):
        if key not in DEFAULT_SETTINGS: return
        d = DEFAULT_SETTINGS[key]
        if isinstance(d, bool): val = bool(val)
        elif key == "pick_pitch": val = None if val in (None, "") else float(val)
        else: val = float(val)
        self.s[key] = val; self.save()

    # ---------- table plane + marker calibration ----------
    def _fit_plane(self):
        pts = list(self.markers.values())
        A = np.array([[p[0], p[1], 1.0] for p in pts]); z = np.array([p[2] for p in pts])
        self.plane = tuple(float(c) for c in np.linalg.lstsq(A, z, rcond=None)[0])

    def table_z(self, x, y):
        a, b, c = self.plane
        return a * x + b * y + c

    def record_marker(self, mid, xyz):
        self.touched[int(mid)] = [round(float(v), 1) for v in xyz]; self.save()

    def apply_touched(self):
        t = self.touched
        if len(t) < 2: return False, "Touch at least 2 markers first (4 is best)."
        new = {k: list(v) for k, v in t.items()}
        info = []
        ids = sorted(t)
        for a in range(len(ids)):
            for b in range(a + 1, len(ids)):
                o = np.linalg.norm(np.subtract(DEFAULT_MARKERS[ids[a]][:2], DEFAULT_MARKERS[ids[b]][:2]))
                n = np.linalg.norm(np.subtract(t[ids[a]][:2], t[ids[b]][:2]))
                info.append(f"{ids[a]}<->{ids[b]}: old {o:.1f} mm, measured {n:.1f} mm (diff {n - o:+.1f})")
        if len(t) < len(DEFAULT_MARKERS):
            R, tr = fit_rigid_2d([DEFAULT_MARKERS[i][:2] for i in ids], [t[i][:2] for i in ids])
            dz = float(np.mean([t[i][2] - DEFAULT_MARKERS[i][2] for i in ids]))
            for k, p in DEFAULT_MARKERS.items():
                if k not in t:
                    xy = R @ np.array(p[:2]) + tr
                    new[k] = [round(float(xy[0]), 1), round(float(xy[1]), 1), round(p[2] + dz, 1)]
            info.append("unmeasured markers were deduced from the old layout (rigid fit)")
        self.markers = {int(k): v for k, v in new.items()}
        self.measured = True
        self._fit_plane(); self.hist.clear(); self.rvec = self.tvec = None
        self.fit_info = " | ".join(info); self.save()
        return True, self.fit_info

    def reset_markers(self):
        self.markers = {k: list(v) for k, v in DEFAULT_MARKERS.items()}
        self.touched = {}; self.measured = False; self.fit_info = ""
        self._fit_plane(); self.hist.clear(); self.rvec = self.tvec = None; self.save()

    # ---------- geometry ----------
    def pixel_to_table(self, u, v, raw=False, z_off=None):
        if self.rvec is None or self.K is None: return None
        n = cv2.undistortPoints(np.array([[[float(u), float(v)]]], np.float32), self.K, self.Z)[0, 0]
        R, _ = cv2.Rodrigues(self.rvec)
        cam = (-R.T @ self.tvec).ravel(); ray = R.T @ np.array([n[0], n[1], 1.0])
        a, b, c = self.plane
        c += self.s["object_z"] if z_off is None else z_off
        den = ray[2] - a * ray[0] - b * ray[1]
        if abs(den) < 1e-9: return None
        t = (a * cam[0] + b * cam[1] + c - cam[2]) / den
        if t <= 0: return None
        x, y = cam[0] + t * ray[0], cam[1] + t * ray[1]
        return (float(x), float(y)) if raw else (float(x + self.s["x_trim"]), float(y + self.s["y_trim"]))

    def project(self, pts):
        pr, _ = cv2.projectPoints(np.array(pts, np.float32).reshape(-1, 1, 3), self.rvec, self.tvec, self.K, self.Z)
        return pr.reshape(-1, 2)

    # ---------- lifecycle ----------
    def start(self):
        if self.K is None:
            self.log(f"Vision disabled: {self.problem}", "err"); return
        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._detect_loop, daemon=True).start()

    def _open(self):
        cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(self.cam_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        return cap

    def _capture_loop(self):
        cap = self._open(); size = None; fails = 0; t_fps = time.time(); n = 0
        while self.running:
            ok, raw = cap.read()
            if not ok:
                fails += 1; self.problem = "camera read failed"
                if fails > 30:
                    cap.release(); time.sleep(1); cap = self._open(); fails = 0
                time.sleep(0.05); continue
            self.problem = ""; fails = 0
            h, w = raw.shape[:2]
            if size != (w, h):
                size = (w, h); self.res = size
                self.m1, self.m2 = cv2.initUndistortRectifyMap(self.K, self.D, None, self.K, size, cv2.CV_16SC2)
                self.warn = []
                cx, cy = self.K[0, 2], self.K[1, 2]
                if abs(cx - w / 2) > 0.15 * w or abs(cy - h / 2) > 0.15 * h:
                    self.warn.append(f"Calibration centre ({cx:.0f},{cy:.0f}) is far from the middle of a {w}x{h} frame: "
                                     "the camera was probably calibrated at a different resolution.")
                self.log(f"Camera running at {w}x{h}", "info")
                for m in self.warn: self.log(m, "warn")
            frame = cv2.remap(raw, self.m1, self.m2, cv2.INTER_LINEAR)
            self.frame = frame
            try:
                corners, ids = self._update_pose(frame)
                self._publish(frame, corners, ids)
            except Exception as e:
                self.log(f"vision error: {e}", "err")
            n += 1
            if time.time() - t_fps > 1.0:
                self.fps = n / (time.time() - t_fps); n = 0; t_fps = time.time()

    def _update_pose(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        centers = {}
        if ids is not None:
            centers = {int(i): c[0].mean(axis=0) for i, c in zip(ids.flatten(), corners)}
        self.seen = sorted(centers)
        need = sorted(self.markers)
        if all(i in centers for i in need) and not self.pose_locked:
            ok, rv, tv = cv2.solvePnP(np.array([self.markers[i] for i in need], np.float32),
                                      np.array([centers[i] for i in need], np.float32),
                                      self.K, self.Z, flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                first = self.rvec is None
                self.hist.append((rv.copy(), tv.copy()))
                self.rvec = np.mean([h[0] for h in self.hist], axis=0); self.tvec = np.mean([h[1] for h in self.hist], axis=0)
                if first: self.log("Camera pose found.", "info")
        self.err, self.proj = {}, {}
        if self.rvec is not None and centers:
            have = [i for i in centers if i in self.markers]
            if have:
                pr = self.project([self.markers[i] for i in have])
                for k, i in enumerate(have):
                    self.proj[i] = pr[k]; self.err[i] = float(np.linalg.norm(pr[k] - centers[i]))
        return corners, ids

    def _axis(self, frame, box):
        H, W = frame.shape[:2]
        x1, y1, x2, y2 = [int(v) for v in box]
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(W, x2), min(H, y2)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return None
        g = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        if np.count_nonzero(th) / th.size > 0.40: _, th = cv2.threshold(g, 185, 255, cv2.THRESH_BINARY_INV)
        th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        cs, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, bs = None, -1.0
        for c in cs:
            a = cv2.contourArea(c)
            if a < 30: continue
            (_, (rw, rh), _) = cv2.minAreaRect(c)
            asp = max(rw, rh) / max(1.0, min(rw, rh))
            sc = a * (asp if asp >= 2.0 else 0.15)
            if sc > bs: bs, best = sc, c
        if best is None: return None
        vx, vy, x0, y0 = [float(v[0]) for v in cv2.fitLine(best, cv2.DIST_L2, 0, 0.01, 0.01)]
        x0 += x1; y0 += y1
        p1, p2 = (x0 + vx * 50, y0 + vy * 50), (x0 - vx * 50, y0 - vy * 50)
        a, b = self.pixel_to_table(*p1, raw=True), self.pixel_to_table(*p2, raw=True)
        if not a or not b: return None
        axis = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 180.0
        return (axis + 90.0) % 180.0, (p1, p2)      # jaws close ACROSS the long axis

    # ---------- detection thread ----------
    def _detect_loop(self):
        model, applied = None, None
        while self.running:
            if not self.detect_on or self.rvec is None or self.frame is None:
                time.sleep(0.1); continue
            try:
                if model is None:
                    self.log("Loading YOLO-World (first time takes a while)...")
                    from ultralytics import YOLOWorld
                    model = YOLOWorld("yolov8s-world.pt"); self.log("YOLO-World ready.")
                if applied != self.prompts:
                    model.set_classes(self.prompts); applied = list(self.prompts)
                frame = self.frame
                dets = []
                for r in model.predict(frame, conf=0.18, verbose=False):
                    for b in r.boxes:
                        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].cpu().numpy()]
                        xy = self.pixel_to_table((x1 + x2) / 2, (y1 + y2) / 2)
                        if not xy: continue
                        ax = self._axis(frame, (x1, y1, x2, y2))
                        dets.append({"box": (x1, y1, x2, y2), "conf": float(b.conf[0]), "x": xy[0], "y": xy[1],
                                     "closing": ax[0] if ax else None, "line": ax[1] if ax else None,
                                     "name": str(r.names.get(int(b.cls[0]), "?"))})
                dets.sort(key=lambda d: -d["conf"])
                if dets:
                    best = dets[0]
                    if self.track and math.hypot(best["x"] - self.track[-1][0], best["y"] - self.track[-1][1]) > 20: self.track.clear()
                    self.track.append((best["x"], best["y"]))
                else:
                    self.track.clear()
                self.dets = dets
            except Exception as e:
                self.log(f"detection error: {e}", "err"); self.detect_on = False
            time.sleep(0.03)

    def best_target(self):
        """Most confident detection, position = median of the last few frames (less jitter)."""
        if not self.dets or not self.track: return None
        d = self.dets[0]
        return {"x": float(np.median([p[0] for p in self.track])), "y": float(np.median([p[1] for p in self.track])),
                "closing": d["closing"], "name": d["name"], "conf": d["conf"]}

    def click_at(self, nu, nv):
        W, H = self.res
        if not W: return None
        u, v = nu * W, nv * H
        xy = self.pixel_to_table(u, v)
        if not xy: return None
        self.click = (xy[0], xy[1], u, v)
        return {"x": xy[0], "y": xy[1], "z": self.table_z(*xy)}

    # ---------- drawing ----------
    def _publish(self, frame, corners, ids):
        H, W = frame.shape[:2]
        dw = min(DISP_W, W); sc = dw / W
        d = cv2.resize(frame, (dw, int(H * sc)), interpolation=cv2.INTER_AREA)
        P = lambda u, v: (int(u * sc), int(v * sc))
        pose = self.rvec is not None
        if pose and self.s["show_grid"]: self._grid(d, P)
        if self.s["show_markers"]:
            if ids is not None:
                for i, c in zip(ids.flatten(), corners):
                    pts = (c[0] * sc).astype(np.int32)
                    cv2.polylines(d, [pts], True, (0, 255, 0), 1)
                    e = self.err.get(int(i))
                    cv2.putText(d, f"{int(i)}" + (f" {e:.1f}px" if e is not None else ""), tuple(pts[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    if int(i) in self.proj:   # yellow = where the stored marker coordinates say it should be
                        cen = c[0].mean(axis=0); pp = self.proj[int(i)]
                        cv2.line(d, P(*cen), P(*pp), (0, 255, 255), 1); cv2.circle(d, P(*pp), 5, (0, 255, 255), 1)
        if pose and self.s["show_tip"] and self.tip_cur is not None:
            x, y, z = self.tip_cur
            tp, gp = self.project([[x, y, z], [x, y, self.table_z(x, y)]])
            cv2.line(d, P(*tp), P(*gp), (255, 255, 0), 1)
            cv2.drawMarker(d, P(*tp), (255, 255, 0), cv2.MARKER_CROSS, 22, 2)
            cv2.circle(d, P(*gp), 6, (255, 255, 0), 1)
            cv2.putText(d, f"TIP {x:.0f},{y:.0f},{z:.0f}", (P(*tp)[0] + 10, P(*tp)[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            if self.jaw_deg is not None:   # jaw closing axis, 25 mm either side, on the table under the tip
                r = math.radians(self.jaw_deg); dx, dy = 25 * math.cos(r), 25 * math.sin(r)
                a, b = self.project([[x - dx, y - dy, self.table_z(x, y)], [x + dx, y + dy, self.table_z(x, y)]])
                cv2.line(d, P(*a), P(*b), (0, 140, 255), 2)
            if self.tip_tgt is not None and np.hypot(self.tip_tgt[0] - x, self.tip_tgt[1] - y) > 1:
                g = self.project([self.tip_tgt])[0]; cv2.circle(d, P(*g), 4, (160, 160, 160), 1)
        if self.click is not None:
            _, _, u, v = self.click
            cv2.drawMarker(d, P(u, v), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 24, 2)
            cv2.putText(d, f"{self.click[0]:.0f},{self.click[1]:.0f}", (P(u, v)[0] + 12, P(u, v)[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        for k, dt in enumerate(self.dets):
            x1, y1, x2, y2 = dt["box"]; col = (255, 100, 0) if k == 0 else (140, 90, 40)
            cv2.rectangle(d, P(x1, y1), P(x2, y2), col, 2)
            cv2.putText(d, f"{dt['name']} {dt['conf']:.2f} X{dt['x']:.0f} Y{dt['y']:.0f}" +
                        (f" jaws {dt['closing']:.0f}" if dt["closing"] is not None else ""), (P(x1, y1)[0], P(x1, y1)[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            if dt["line"]: cv2.line(d, P(*dt["line"][0]), P(*dt["line"][1]), (0, 0, 255), 2)
        errs = list(self.err.values())
        if not pose: msg, col = "SEARCHING FOR ALL 4 MARKERS", (0, 0, 255)
        else:
            msg = ("POSE LOCKED" if self.pose_locked else "POSE LIVE") + (f"  reproj {np.mean(errs):.1f}px" if errs else "")
            col = (0, 255, 0) if (errs and max(errs) < 3) else (0, 165, 255)
        cv2.putText(d, msg, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
        ok, buf = cv2.imencode(".jpg", d, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok: self.jpg = buf.tobytes(); self.seq += 1

    def _grid(self, d, P):
        for fixed_x in range(0, 401, 50):
            pts = [[fixed_x, y, self.table_z(fixed_x, y)] for y in range(-300, 301, 25)]
            pp = self.project(pts)
            cv2.polylines(d, [np.array([P(*p) for p in pp], np.int32)], False, (90, 90, 90), 1)
            if fixed_x: cv2.putText(d, f"X{fixed_x}", P(*pp[len(pp) // 2 - 6]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)
        for fixed_y in range(-300, 301, 50):
            pts = [[x, fixed_y, self.table_z(x, fixed_y)] for x in range(0, 401, 25)]
            pp = self.project(pts)
            cv2.polylines(d, [np.array([P(*p) for p in pp], np.int32)], False, (200, 200, 60) if fixed_y == 0 else (90, 90, 90), 1)
            cv2.putText(d, f"Y{fixed_y}", P(*pp[-1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)

    # ---------- status for the UI ----------
    def status(self):
        errs = self.err
        return {"type": "vision", "enabled": self.K is not None, "problem": self.problem, "warn": self.warn,
                "pose": self.rvec is not None, "locked": self.pose_locked, "seen": self.seen, "res": list(self.res),
                "fps": round(self.fps, 1), "err": {str(k): round(v, 2) for k, v in errs.items()},
                "markers": {str(k): v for k, v in self.markers.items()}, "touched": {str(k): v for k, v in self.touched.items()},
                "measured": self.measured, "fit_info": self.fit_info, "plane": [round(c, 5) for c in self.plane],
                "settings": self.s, "roll": self.roll, "detect": self.detect_on, "prompts": self.prompts,
                "dets": [{"name": d["name"], "conf": round(d["conf"], 2), "x": round(d["x"], 1), "y": round(d["y"], 1),
                          "closing": None if d["closing"] is None else round(d["closing"], 1)} for d in self.dets[:5]],
                "click": None if self.click is None else {"x": round(self.click[0], 1), "y": round(self.click[1], 1),
                                                          "z": round(self.table_z(self.click[0], self.click[1]), 1)},
                "jaw": self.jaw_deg}