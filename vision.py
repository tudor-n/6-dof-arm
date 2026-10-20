"""vision.py - camera, ArUco pose, table raycast, YOLO-World detection and an annotated MJPEG stream.
Runs in background threads inside server.py. Everything it knows is published through status().
Needs: pip install opencv-contrib-python numpy ultralytics   (ultralytics only if you use Detect)
"""
import itertools, json, math, os, threading, time
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
# Hand-measured distances between marker centres, "<lo>-<hi>" -> mm. This is the only
# ground truth in the system that does not come from the arm itself, so it is what catches
# a forward-kinematics error - everything else is FK-derived and therefore self-consistent
# even when wrong. Measure all six pairs for four markers.
DEFAULT_TAPE = {}
DEFAULT_ARUCO_MM = 0.0      # black-square edge length, for the future 16-corner pose
TAPE_WARN_MM = 10.0         # shape disagreement above this is reported loudly
# Words the tablet's in-app keyboard suggests for the detect prompt (vision.json "vocab" overrides).
DEFAULT_VOCAB = ["pen", "pencil", "pliers", "phone", "paper clip", "plate", "cup", "cube", "cutter", "coin", "cap", "cable",
                 "block", "bottle", "bolt", "battery", "ball", "box", "tape", "tape measure", "scissors", "screw", "sponge",
                 "spoon", "marker", "mug", "nut", "key", "knife", "eraser", "glue stick", "wrench", "remote", "lego brick",
                 "usb stick", "washer", "die", "dice", "fork", "hex key", "lid", "toy car", "screwdriver"]
DEFAULT_ROLL = {"zero": 90.0, "sign": 1}   # servo value where the jaws close along the arm; +1 = servo up turns jaws CCW from above
DISP_W = 960

MIN_CAM_HEIGHT_MM = 100.0    # a solve putting the camera lower than this above the table is
                             # the mirrored branch of the coplanar-PnP ambiguity - reject it
MAX_SOLVE_REPROJ_PX = 4.0    # per-frame solves worse than this never enter the average
MAX_POSE_REPROJ_PX = 3.0     # MEAN reprojection above this means the pose is not trustworthy
MAX_POSE_WORST_PX = 8.0      # one marker this far out is broken, however good the mean is
HEALTH_RECOVER = 0.8         # hysteresis: recover below this fraction of the limit


def shape_from_distances(tape, ids):
    """Reconstruct marker XY from pairwise distances alone, up to placement and reflection.

    Puts ids[0] at the origin and ids[1] on +x, then trilaterates the other two. Each has a
    two-fold choice of side, so all four combinations are tried and the one whose remaining
    pair distance best matches the tape is kept. Returns (points, closure_error_mm); the
    closure error is how self-consistent the six measurements are, which is a direct check
    on the measuring rather than on the arm.
    """
    a, b, c, d = ids

    def g(i, j):
        v = tape.get(f"{min(i, j)}-{max(i, j)}")
        if v is None:
            raise ValueError(f"no tape measurement for pair {min(i, j)}-{max(i, j)}")
        return float(v)

    if g(a, b) <= 0:
        raise ValueError("tape distances must be positive")
    best = None
    for signs in itertools.product((1.0, -1.0), repeat=2):
        P = {a: np.zeros(2), b: np.array([g(a, b), 0.0])}
        ok = True
        for k, sg in zip((c, d), signs):
            d0, d1 = g(a, k), g(b, k)
            x = (d0 * d0 - d1 * d1 + P[b][0] ** 2) / (2 * P[b][0])
            h2 = d0 * d0 - x * x
            if h2 < -1.0:                       # the triangle does not close at all
                ok = False
                break
            P[k] = np.array([x, sg * math.sqrt(max(0.0, h2))])
        if not ok:
            continue
        err = abs(float(np.linalg.norm(P[c] - P[d])) - g(c, d))
        if best is None or err < best[0]:
            best = (err, P)
    if best is None:
        raise ValueError("these tape distances do not describe a planar quadrilateral")
    return best[1], best[0]


def fit_rigid_2d(old, new):
    old, new = np.asarray(old, float), np.asarray(new, float)
    co, cn = old.mean(0), new.mean(0)
    U, _, Vt = np.linalg.svd((old - co).T @ (new - cn))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, cn - R @ co


def avg_rotation(rvecs):
    """Mean of rotations supplied as Rodrigues vectors.

    Rodrigues vectors must NOT be averaged componentwise. R(a, t) == R(-a, 2pi - t), so
    every rotation has two vector forms, and near |rvec| = pi the axis sign flips between
    frames - which is exactly where a camera looking down at a +Z-up table frame sits.
    Averaging the vectors there collapses toward identity and the projected world comes
    out mirrored. Average the rotation matrices instead, then project back onto SO(3)
    using the same determinant guard as fit_rigid_2d.
    """
    M = np.mean([cv2.Rodrigues(np.asarray(r, float).reshape(3, 1))[0] for r in rvecs], axis=0)
    U, _, Vt = np.linalg.svd(M)
    if np.linalg.det(U @ Vt) < 0:
        U[:, -1] *= -1
    return cv2.Rodrigues(U @ Vt)[0]


def camera_centre(rvec, tvec):
    """Camera position expressed in the arm's base frame."""
    R, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))
    return (-R.T @ np.asarray(tvec, float).reshape(3, 1)).ravel()


class Vision:
    def __init__(self, here, cam_index, log):
        self.here, self.cam_index, self.log = Path(here), cam_index, log
        self.file = self.here / "vision.json"
        self.markers = {k: list(v) for k, v in DEFAULT_MARKERS.items()}
        self.measured = False
        self.touched = {}
        self.s = dict(DEFAULT_SETTINGS)
        self.roll = dict(DEFAULT_ROLL)
        self.tape = dict(DEFAULT_TAPE)
        self.vocab = list(DEFAULT_VOCAB)
        self.aruco_mm = DEFAULT_ARUCO_MM
        self.fit_info = ""
        self._load()
        self._fit_plane()
        self.K = self.D = None
        self.Z = np.zeros((4, 1))
        self.problem = ""
        self.warn = []
        self.calib_res = None; self.calib_rms = None
        for d in (self.here, Path.cwd()):
            try:
                self.K = np.load(d / "camera_matrix.npy"); self.D = np.load(d / "dist_coeffs.npy")
                try:                      # written by calibrate_camera.py; older runs lack it
                    c = json.loads((d / "camera_calib.json").read_text())
                    self.calib_res = tuple(c["resolution"]); self.calib_rms = c.get("rms")
                except Exception:
                    pass
                break
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
        self.H = None              # pixel -> table homography, independent of K and the pose
        self.pose_locked = False
        self.err, self.proj, self.seen, self.centers = {}, {}, [], {}
        # pose diagnostics: raw_err is the per-frame solve, avg_err the pose actually in use
        self.raw_err = self.avg_err = None
        self.rejects = 0; self.diag = ""; self._healthy = None
        # live data
        self.running = False
        self.frame = None; self.res = (0, 0); self.fps = 0.0
        self.jpg = None; self.seq = 0
        self.annot = None; self.small = None; self.small_key = None   # overlaid frame + its downscaled JPEG
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
            self.tape = {str(k): float(v) for k, v in (d.get("tape") or {}).items()}
            self.aruco_mm = float(d.get("aruco_mm", DEFAULT_ARUCO_MM))
            self.vocab = [str(w) for w in d.get("vocab", self.vocab)]
        except FileNotFoundError:
            pass
        except Exception as e:
            print("vision.json unreadable, using defaults:", e)

    def save(self):
        self.file.write_text(json.dumps({"markers": self.markers, "touched": self.touched, "measured": self.measured,
                                         "settings": self.s, "roll": self.roll, "tape": self.tape,
                                         "aruco_mm": self.aruco_mm, "vocab": self.vocab}, indent=2))

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

    def record_marker(self, mid, xyz, pitch=None, roll=None):
        """Store an arm-measured marker centre as [x, y, z, tool_pitch, roll_servo].

        The tool pose is kept because the recorded point is only as good as the tool
        model at that pose: touch four markers at four different wrist pitches and the
        four Z values carry four different errors. Older 3-element entries still load.
        """
        rec = [round(float(v), 1) for v in xyz]
        rec += [None if pitch is None else round(float(pitch), 1),
                None if roll is None else round(float(roll), 1)]
        self.touched[int(mid)] = rec
        self.save()

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
        poses = [(v[3], v[4]) for v in t.values() if len(v) > 4 and v[3] is not None]
        if len(poses) > 1:
            ps = [q[0] for q in poses]
            spread = max(ps) - min(ps)
            if spread > 5.0:
                info.append(f"WARNING: these markers were touched at tool pitches spanning "
                            f"{spread:.0f} deg ({min(ps):.0f}..{max(ps):.0f}). Unless the tool offset "
                            f"in config.json is correct, each Z carries a different error and the "
                            f"fitted table plane will be wrong. Re-touch them all at one pitch.")
            rs = [q[1] for q in poses if q[1] is not None]
            if rs and max(rs) - min(rs) > 5.0:
                info.append(f"WARNING: wrist roll varied by {max(rs) - min(rs):.0f} deg across these "
                            f"recordings. A grasp point off the roll axis moves with roll, so the "
                            f"measurements are not comparable.")
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

    # ---------- tape measurements: the only non-FK ground truth ----------
    def tape_check(self):
        """Compare the stored marker model's SHAPE against the hand-measured distances.

        Distances are invariant under rotation and translation, so this isolates shape error
        from placement error. A shape disagreement cannot be fixed by trims or by moving the
        model - it means the measurement that produced the model is distorted.
        """
        if not self.tape:
            return None
        rows, worst = [], 0.0
        for key in sorted(self.tape):
            try:
                i, j = (int(v) for v in key.split("-"))
            except ValueError:
                continue
            if i not in self.markers or j not in self.markers:
                continue
            t = float(self.tape[key])
            m = float(np.linalg.norm(np.subtract(self.markers[i][:2], self.markers[j][:2])))
            rows.append({"pair": key, "tape": round(t, 1), "model": round(m, 1), "diff": round(m - t, 1)})
            worst = max(worst, abs(m - t))
        if not rows:
            return None
        return {"rows": rows, "worst": round(worst, 1), "ok": worst <= TAPE_WARN_MM}

    def markers_from_tape(self, apply=False):
        """Build a shape-correct marker model from the tape, placed onto the touched points.

        The tape fixes the shape exactly; only where that shape sits in the arm's frame comes
        from the arm. So a distorted forward kinematics can still shift and rotate the result
        but can no longer deform it - which matters because solvePnP is far more sensitive to
        a wrong shape than to a wrong placement. A wrong shape gets partly absorbed as camera
        tilt and then throws off every raycast.

        Both reflections of the reconstructed shape are fitted and the better one kept, since
        a rigid fit cannot undo a mirror.
        """
        ids = sorted(self.markers)
        if len(ids) != 4:
            return None, "need exactly 4 markers"
        anchor = {i: self.touched.get(i) or self.markers.get(i) for i in ids}
        if any(anchor[i] is None for i in ids):
            return None, "no arm-measured positions to place the tape shape onto"
        try:
            shape, closure = shape_from_distances(self.tape, ids)
        except ValueError as e:
            return None, str(e)

        B = np.array([anchor[i][:2] for i in ids], float)
        best = None
        for mirror in (1.0, -1.0):
            A = np.array([[shape[i][0], mirror * shape[i][1]] for i in ids], float)
            R, t = fit_rigid_2d(A, B)
            fit = (R @ A.T).T + t
            res = np.linalg.norm(fit - B, axis=1)
            rms = float(np.sqrt((res ** 2).mean()))
            if best is None or rms < best[0]:
                best = (rms, fit, res)
        rms, fit, res = best

        out = {}
        for k, i in enumerate(ids):
            z = anchor[i][2] if len(anchor[i]) > 2 else self.table_z(fit[k][0], fit[k][1])
            out[i] = [round(float(fit[k][0]), 1), round(float(fit[k][1]), 1), round(float(z), 1)]
        info = (f"tape shape closes to {closure:.1f} mm | placed onto the arm-measured points with "
                f"{rms:.1f} mm RMS residual (per marker: "
                + ", ".join(f"{i}:{res[k]:.0f}" for k, i in enumerate(ids)) + " mm)")
        if rms > TAPE_WARN_MM:
            info += (f" | the arm-measured shape disagrees with the tape by {rms:.0f} mm RMS, so "
                     "forward kinematics is distorted - re-measure link lengths and joint zeros")
        if apply:
            self.markers = out
            self.measured = True
            self._fit_plane(); self.hist.clear(); self.rvec = self.tvec = None
            self.fit_info = info
            self.save()
        return out, info

    def reset_markers(self):
        self.markers = {k: list(v) for k, v in DEFAULT_MARKERS.items()}
        self.touched = {}; self.measured = False; self.fit_info = ""
        self._fit_plane(); self.hist.clear(); self.rvec = self.tvec = None; self.save()

    # ---------- camera pose ----------
    def pose_is_sane(self, rvec, tvec):
        """Reject the mirrored solution: the camera must be above the table, not under it.

        Four near-coplanar points admit two poses that both reproject almost perfectly,
        one of them reflected through the marker plane. The reflected one puts the camera
        below the table, which is physically impossible here, so this check removes it
        deterministically instead of hoping the solver picks the right branch.
        """
        cam = camera_centre(rvec, tvec)
        return bool(cam[2] > self.table_z(cam[0], cam[1]) + MIN_CAM_HEIGHT_MM)

    def solve_pose(self, obj, px):
        """Camera pose from marker object points and their pixel centres.

        Returns (rvec, tvec, mean_reproj_px), or None when the solve is unusable.
        Uses SQPNP, which is globally optimal, rather than ITERATIVE, which descends into
        a local minimum on near-coplanar point sets. (IPPE returns both planar branches
        explicitly but demands exact coplanarity; these markers have a ~0.4 mm Z spread.)
        """
        obj = np.asarray(obj, np.float32).reshape(-1, 3)
        px = np.asarray(px, np.float32).reshape(-1, 2)
        ok, rv, tv = cv2.solvePnP(obj, px, self.K, self.Z, flags=cv2.SOLVEPNP_SQPNP)
        if not ok:
            return None
        rv, tv = cv2.solvePnPRefineLM(obj, px, self.K, self.Z, rv, tv)
        if not self.pose_is_sane(rv, tv):
            return None
        pr = cv2.projectPoints(obj.reshape(-1, 1, 3), rv, tv, self.K, self.Z)[0].reshape(-1, 2)
        return rv, tv, float(np.mean(np.linalg.norm(pr - px, axis=1)))

    def pose_health(self):
        """(ok, message) - drives the UI badge and gates picking.

        Judged on the MEAN reprojection, not the worst marker. With only four markers, one
        sitting 2-3 mm off - which is inside hand-measurement precision, and worse for the
        markers furthest from the camera because a fixed error on the table subtends more
        pixels there - would otherwise condemn a pose that is demonstrably stable, and the
        verdict would flap sample to sample while picks were randomly refused.

        A separate and much looser worst-marker limit still catches one genuinely broken
        marker, and hysteresis stops the verdict oscillating at the boundary.
        """
        if self.K is None:
            return False, self.problem or "no camera calibration"
        if self.rvec is None:
            return False, "no camera pose yet (all 4 markers must be visible at once)"
        if not self.err:
            return False, "no markers visible - pose is stale"
        errs = list(self.err.values())
        mean, worst = float(np.mean(errs)), float(max(errs))
        # The outlier test comes first deliberately: a single wildly wrong marker also drags
        # the mean over its limit, and "marker 3 is wrong" is far more actionable than
        # "the mean is high".
        if worst > MAX_POSE_WORST_PX:
            bad = max(self.err, key=lambda k: self.err[k])
            return False, (f"marker {bad} reprojects {worst:.1f}px, far outside the rest - its "
                           f"stored position is probably wrong or it is mis-detected")
        # once unhealthy, require a clearly better reading before going healthy again
        lim = MAX_POSE_REPROJ_PX if self._healthy else MAX_POSE_REPROJ_PX * HEALTH_RECOVER
        if mean > lim:
            return False, (f"camera pose unreliable: mean reprojection {mean:.1f}px "
                           f"(limit {lim:.1f}px)")
        return True, f"pose ok (mean {mean:.1f}px, worst {worst:.1f}px)"

    def marker_offsets(self):
        """Each marker's residual as an offset on the table, in mm.

        Reprojection error is in pixels, which tangles how wrong a marker's recorded position
        is with how far it sits from the camera - the same millimetre of error subtends far
        more pixels up close. Raycasting each detected centre back onto the table and
        comparing against the stored coordinates gives the discrepancy in the units the
        markers were actually measured in, which is the form you can act on.
        """
        if self.rvec is None or not self.centers:
            return {}
        out = {}
        for i, uv in self.centers.items():
            if i not in self.markers:
                continue
            p = self.markers[i]
            hit = self.pixel_to_table(uv[0], uv[1], raw=True,
                                      z_off=p[2] - self.table_z(p[0], p[1]))
            if not hit:
                continue
            dx, dy = hit[0] - p[0], hit[1] - p[1]
            out[i] = {"dx": round(dx, 2), "dy": round(dy, 2),
                      "dist": round(math.hypot(dx, dy), 2)}
        return out

    # ---------- a second, independent pixel -> table mapping ----------
    def _update_homography(self, centers):
        """Fit pixels -> table plane straight from the four marker correspondences.

        This uses neither the camera matrix nor the solved pose - only where the markers are
        on the table and where they appear in the (already undistorted) frame. For points on
        the table plane it is exact.

        It is NOT a replacement for the PnP pose. Four points give 8 equations for a
        homography's 8 unknowns, so it fits them perfectly no matter how wrong the marker
        model is and carries no residual to warn you with. PnP has 6 unknowns for the same 8
        equations, and those 2 spare degrees of freedom are the only reason a bad marker
        model shows up as reprojection error at all. The homography also cannot handle a
        point above the table, having no camera position to cast a ray from.

        Its value is as a cross-check on the intrinsics, which it does not share.
        """
        need = sorted(self.markers)
        if not all(i in centers for i in need) or len(need) != 4:
            self.H = None
            return
        src = np.array([centers[i] for i in need], np.float32)
        dst = np.array([self.markers[i][:2] for i in need], np.float32)
        try:
            self.H = cv2.getPerspectiveTransform(src, dst)
        except cv2.error:
            self.H = None

    def pixel_to_table_h(self, u, v):
        """Table XY for a pixel, via the homography. No intrinsics, no pose, plane only."""
        if self.H is None:
            return None
        q = cv2.perspectiveTransform(np.array([[[float(u), float(v)]]], np.float64), self.H)[0, 0]
        return (float(q[0]), float(q[1]))

    def intrinsics_check(self, n=5):
        """Compare the two independent pixel -> table mappings across the marker area.

        The raycast goes through the camera matrix and the solved 3D pose; the homography uses
        neither. Both share the same marker model, so that error cancels and what is left
        implicates the INTRINSICS. Small disagreement means the camera matrix is good enough
        for this workspace, whatever its calibration RMS was.
        """
        if self.H is None or self.rvec is None or not self.proj:
            return None
        px = np.array(list(self.proj.values()), float)
        lo, hi = px.min(axis=0), px.max(axis=0)
        ds = []
        for i in range(n):
            for j in range(n):
                u = lo[0] + (hi[0] - lo[0]) * (i / (n - 1) if n > 1 else 0.5)
                v = lo[1] + (hi[1] - lo[1]) * (j / (n - 1) if n > 1 else 0.5)
                a = self.pixel_to_table(u, v, raw=True, z_off=0.0)
                b = self.pixel_to_table_h(u, v)
                if a and b:
                    ds.append(math.hypot(a[0] - b[0], a[1] - b[1]))
        if not ds:
            return None
        return {"mean": round(float(np.mean(ds)), 2), "worst": round(float(np.max(ds)), 2),
                "n": len(ds)}

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
        chk = self.tape_check()
        if chk and not chk["ok"]:
            self.log(f"MARKER MODEL FAILS THE TAPE CHECK: worst pair is off by {chk['worst']:.0f} mm. "
                     "Pairwise distances do not depend on where the model sits, so this is a SHAPE "
                     "error - no trim or offset can fix it, and solvePnP will absorb part of it as "
                     "camera tilt and throw off every table coordinate. Rebuild the model from the "
                     "tape (Camera tab -> Marker calibration -> From tape).", "err")
            for r in chk["rows"]:
                self.log(f"  {r['pair']}: tape {r['tape']:.0f} mm, model {r['model']:.1f} mm "
                         f"({r['diff']:+.1f})", "warn")
        elif chk:
            self.log(f"marker model agrees with the tape (worst pair {chk['worst']:.1f} mm)", "ok")
        if self.K is None:
            self.log(f"Vision disabled: {self.problem}", "err"); return
        self.running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._detect_loop, daemon=True).start()

    def _open(self):
        cap = cv2.VideoCapture(self.cam_index, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(self.cam_index)
        # Lock MJPG first: without it the camera may negotiate a different (often cropped)
        # mode than the one the intrinsics were calibrated at, which silently invalidates K.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
        w, h = self.calib_res or (1920, 1080)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
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
                if self.calib_res and tuple(self.calib_res) != (w, h):
                    self.warn.append(f"Camera is streaming {w}x{h} but the intrinsics were calibrated at "
                                     f"{self.calib_res[0]}x{self.calib_res[1]}. camera_matrix.npy does not "
                                     "apply to this frame size, so every table coordinate will be wrong. "
                                     "Re-run calibrate_camera.py at this resolution.")
                elif not self.calib_res:
                    self.warn.append("camera_calib.json is missing, so the stream resolution cannot be checked "
                                     "against the calibration. Re-run calibrate_camera.py to record it.")
                    cx, cy = self.K[0, 2], self.K[1, 2]
                    if abs(cx - w / 2) > 0.15 * w or abs(cy - h / 2) > 0.15 * h:
                        self.warn.append(f"Calibration centre ({cx:.0f},{cy:.0f}) is far from the middle of a "
                                         f"{w}x{h} frame: it was probably calibrated at another resolution.")
                if self.calib_rms is not None and self.calib_rms > 1.0:
                    self.warn.append(f"Camera calibration RMS is {self.calib_rms:.2f}px. Above ~1px the intrinsics "
                                     "are too poor for millimetre work - recalibrate with a larger board.")
                elif self.calib_res and self.calib_rms is None:
                    # Deliberately not a warning. The fit quality was never recorded, but
                    # intrinsics_check() measures it empirically and is the better test anyway.
                    self.log("Camera calibration RMS was never recorded for these intrinsics. "
                             "Watch the intrinsics-check figure instead - under 2 mm means they "
                             "are good enough for this workspace.", "info")
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
        self.centers = centers
        need = sorted(self.markers)
        if all(i in centers for i in need) and not self.pose_locked:
            got = self.solve_pose([self.markers[i] for i in need], [centers[i] for i in need])
            if got is None:
                self._reject("the solve put the camera below the table (mirrored solution), or the "
                             "marker coordinates and the camera intrinsics disagree")
            else:
                rv, tv, self.raw_err = got
                if self.raw_err > MAX_SOLVE_REPROJ_PX:
                    self._reject(f"{self.raw_err:.1f}px raw reprojection, limit {MAX_SOLVE_REPROJ_PX:.0f}px - "
                                 "the stored marker coordinates or camera_matrix.npy are wrong")
                else:
                    first = self.rvec is None
                    self.rejects = 0
                    self.hist.append((rv.copy(), tv.copy()))
                    self.rvec = avg_rotation([h[0] for h in self.hist])
                    self.tvec = np.mean([h[1] for h in self.hist], axis=0)
                    if first: self.log("Camera pose found.", "info")
        self._update_homography(centers)
        self.err, self.proj = {}, {}
        if self.rvec is not None and centers:
            have = [i for i in centers if i in self.markers]
            if have:
                pr = self.project([self.markers[i] for i in have])
                for k, i in enumerate(have):
                    self.proj[i] = pr[k]; self.err[i] = float(np.linalg.norm(pr[k] - centers[i]))
                self.avg_err = float(np.mean(list(self.err.values())))
                cam = camera_centre(self.rvec, self.tvec)
                off = self.marker_offsets()
                self.diag = (f"raw {self.raw_err:.2f}px | in-use {self.avg_err:.2f}px | per-marker "
                             + " ".join(f"{i}:{self.err[i]:.1f}" for i in sorted(self.err))
                             + f" | camera at X{cam[0]:.0f} Y{cam[1]:.0f} Z{cam[2]:.0f}"
                             + (" | off-by-mm " + " ".join(f"{i}:{off[i]['dist']:.1f}"
                                                           for i in sorted(off)) if off else ""))
                self._health_transition()
        return corners, ids

    def _reject(self, why):
        """Throttled complaint about an unusable solve - never silently use a bad pose."""
        self.rejects += 1
        if self.rejects % 60 == 1:
            self.log(f"pose solve rejected ({self.rejects}x): {why}", "warn")

    def _health_transition(self):
        """Log only when pose health changes, so the UI log stays readable."""
        ok, msg = self.pose_health()
        if ok != self._healthy:
            self._healthy = ok
            self.log(f"camera pose {'HEALTHY' if ok else 'UNHEALTHY'}: {msg} | {self.diag}",
                     "ok" if ok else "err")

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
        if not pose:
            msg, col = "SEARCHING FOR ALL 4 MARKERS", (0, 0, 255)
        else:
            ok, _ = self.pose_health()
            msg = ("POSE LOCKED" if self.pose_locked else "POSE LIVE")
            if errs:
                msg += f"  worst {max(errs):.1f}px  mean {np.mean(errs):.1f}px"
            if self.raw_err is not None:
                msg += f"  (solve {self.raw_err:.1f}px)"
            col = (0, 255, 0) if ok else (0, 165, 255)
        cv2.putText(d, msg, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
        if pose and self.rejects:
            cv2.putText(d, f"{self.rejects} solves rejected - pose is stale", (12, 46),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        ok, buf = cv2.imencode(".jpg", d, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok: self.annot = d; self.jpg = buf.tobytes(); self.seq += 1

    def small_jpg(self, w, h, q=60):
        """The overlaid frame re-encoded at w x h: the old tablet then neither rescales nor decodes
        more pixels than it shows. Cached per frame, so several clients cost one encode."""
        d, key = self.annot, (self.seq, w, h, q)
        if d is None: return None
        if self.small_key != key:
            ok, buf = cv2.imencode(".jpg", cv2.resize(d, (w, h), interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, q])
            if not ok: return None
            self.small, self.small_key = buf.tobytes(), key
        return self.small

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
        ok, health = self.pose_health()
        return {"type": "vision", "enabled": self.K is not None, "problem": self.problem, "warn": self.warn,
                "pose": self.rvec is not None, "locked": self.pose_locked, "seen": self.seen, "res": list(self.res),
                "healthy": ok, "health": health, "diag": self.diag, "rejects": self.rejects,
                "raw_err": None if self.raw_err is None else round(self.raw_err, 2),
                "calib": {"res": list(self.calib_res) if self.calib_res else None, "rms": self.calib_rms},
                "cam_pos": None if self.rvec is None else [round(float(v), 1) for v in camera_centre(self.rvec, self.tvec)],
                "fps": round(self.fps, 1), "err": {str(k): round(v, 2) for k, v in errs.items()},
                "markers": {str(k): v for k, v in self.markers.items()}, "touched": {str(k): v for k, v in self.touched.items()},
                "measured": self.measured, "fit_info": self.fit_info, "plane": [round(c, 5) for c in self.plane],
                "tape": self.tape, "tape_check": self.tape_check(), "aruco_mm": self.aruco_mm,
                "intrinsics_check": self.intrinsics_check(), "offsets": self.marker_offsets(),
                "settings": self.s, "roll": self.roll, "detect": self.detect_on, "prompts": self.prompts,
                "dets": [{"name": d["name"], "conf": round(d["conf"], 2), "x": round(d["x"], 1), "y": round(d["y"], 1),
                          "closing": None if d["closing"] is None else round(d["closing"], 1)} for d in self.dets[:5]],
                "click": None if self.click is None else {"x": round(self.click[0], 1), "y": round(self.click[1], 1),
                                                          "z": round(self.table_z(self.click[0], self.click[1]), 1)},
                "jaw": self.jaw_deg}