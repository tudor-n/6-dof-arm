import asyncio
import json
import cv2
import math
import time
from pathlib import Path
import cv2.aruco as aruco
import numpy as np
import websockets
from ultralytics import YOLOWorld

# --- SYSTEM CONFIGURATION ---
WS_URI = "ws://localhost:8080/ws"
CAMERA_INDEX = 0

# Load 3D Lens Calibration
try:
    CAM_MATRIX = np.load("camera_matrix.npy")
    DIST_COEFFS = np.load("dist_coeffs.npy")
    BLANK_DIST = np.zeros((4, 1))  # Used for math after the frame is undistorted
    print("[INFO] 3D Calibration files loaded successfully.")
except FileNotFoundError:
    raise SystemExit("[ERROR] camera_matrix.npy or dist_coeffs.npy not found! Run calibration first.")

# --- KINEMATIC OFFSETS ---
X_TRIM = 0.0
Y_TRIM = 0.0
YAW_TRIM_DEGREES = 0.0
Z_GRASP_OFFSET = -32.0
OBJECT_Z_HEIGHT = 5.0

ARUCO_SIZE_MM = 40.0

# Physical position of ID-5 centre in Robot Base Frame (mm)
# (+X = Forward, +Y = Left, +Z = Up)
ID5_ROBOT_POS = np.array([-40.0, -32.0, 45.0], dtype=np.float32)

L_ID4 = 100.0
H_ID4 = 35.0

# Physical Table Plane (overwritten by auto-calibration)
PLANE_A = 0.1197
PLANE_B = 0.0573
PLANE_C = -14.35

def get_table_z(x, y, hover=False):
    z_surface = PLANE_A * x + PLANE_B * y + PLANE_C
    return z_surface + 130.0 if hover else z_surface

# TRUE 3D MARKER COORDINATES – updated by auto-calibration
MARKERS_3D = {
    0: [-17.8,  203.2, get_table_z(-17.8,  203.2)],
    1: [ 37.1, -190.8, get_table_z( 37.1, -190.8)],
    2: [217.4,  157.9, get_table_z(217.4,  157.9)],
    3: [212.1, -191.0, get_table_z(212.1, -191.0)],
}

def fit_plane(points):
    pts = np.asarray(points, dtype=np.float64)
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(len(pts))]
    a, b, c = np.linalg.lstsq(A, pts[:, 2], rcond=None)[0]
    return float(a), float(b), float(c)

class PrecisionVisionAgent:
    def __init__(self):
        self.ws = None
        self.is_connected = False
        self.cap = None

        print("[INFO] Loading YOLO-World model...")
        self.model = YOLOWorld('yolov8s-world.pt')
        self.target_prompts = ["screwdriver", "tool handle", "metal tool", "pen", "pencil"]
        self.model.set_classes(self.target_prompts)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.rvec = None
        self.tvec = None
        self.calibration_locked = False

    async def connect(self):
        try:
            self.ws = await websockets.connect(WS_URI, ping_interval=None)
            self.is_connected = True
            print("[INFO] Successfully connected to Arm IK Server.")
        except (ConnectionRefusedError, OSError):
            print("[WARN] Robot server not reachable. Running in VISION-ONLY mode.")

    async def send_cmd(self, cmd):
        if not self.is_connected or self.ws is None:
            return
        await self.ws.send(json.dumps(cmd))
        await asyncio.sleep(0.02)

    async def wait_and_pump(self, duration, msg=""):
        end_time = asyncio.get_event_loop().time() + duration
        while asyncio.get_event_loop().time() < end_time:
            ret, raw_frame = self.cap.read()
            if ret:
                frame = cv2.undistort(raw_frame, CAM_MATRIX, DIST_COEFFS)
                self.update_3d_pose(frame)
                cv2.putText(frame, msg, (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
                cv2.imshow("Precision Vision Agent", frame)
                cv2.waitKey(1)
            await asyncio.sleep(0.03)

    async def arm_move(self, x, y, z):
        print(f"Moving to X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f}")
        await self.send_cmd({"type": "tip", "x": x, "y": y, "z": z, "lock": False})
        await self.wait_and_pump(2.0, f"MOVING ARM TO Z={z:.0f}...")

    def auto_calibrate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            print("[CALIB] No markers detected.")
            return False

        ids_flat = ids.flatten().tolist()
        print(f"[CALIB] Detected IDs: {sorted(ids_flat)}")

        if 5 not in ids_flat:
            print("[CALIB] ID-5 not visible. Point the camera at the base sticker.")
            return False

        # ----- 1. Pose of the ID-5 marker in the camera -----
        idx5 = ids_flat.index(5)
        half = ARUCO_SIZE_MM / 2.0
        obj_pts = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        ok, rvec_m, tvec_m = cv2.solvePnP(
            obj_pts, corners[idx5][0],
            CAM_MATRIX, BLANK_DIST,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            ok, rvec_m, tvec_m = cv2.solvePnP(
                obj_pts, corners[idx5][0],
                CAM_MATRIX, BLANK_DIST,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
        if not ok:
            print("[CALIB] solvePnP failed on ID-5")
            return False

        R_m, _ = cv2.Rodrigues(rvec_m)          # point_cam = R_m * point_marker + t_m
        t_m = tvec_m.reshape(3, 1)

        # ----- 2. Convert marker pose → robot-base pose -----
        # Marker axes assumed aligned with robot base.
        # point_marker = point_robot - ID5_ROBOT_POS
        # → t_des = t_m - R_m @ ID5_ROBOT_POS
        t_des = t_m - R_m @ ID5_ROBOT_POS.reshape(3, 1)

        self.rvec, _ = cv2.Rodrigues(R_m)
        self.tvec = t_des.astype(np.float32)
        self.calibration_locked = True
        print("[CALIB] ✓ Camera pose locked from ID-5")
        print(f"         tvec = {self.tvec.ravel()}")

        # ----- 3. Optionally re-measure visible table markers -----
        cam_origin = (-R_m.T @ t_des).flatten()

        updated = {}
        for mid in (0, 1, 2, 3):
            if mid not in ids_flat:
                continue
            i = ids_flat.index(mid)
            c = corners[i][0]
            u = float(np.mean(c[:, 0]))
            v = float(np.mean(c[:, 1]))

            pt = np.array([[[u, v]]], dtype=np.float32)
            n = cv2.undistortPoints(pt, CAM_MATRIX, BLANK_DIST)[0, 0]
            ray_cam = np.array([n[0], n[1], 1.0])
            ray_robot = (R_m.T @ ray_cam.reshape(3, 1)).flatten()

            if abs(ray_robot[2]) < 1e-6:
                continue
            t = (ID5_ROBOT_POS[2] - cam_origin[2]) / ray_robot[2]
            x = cam_origin[0] + t * ray_robot[0]
            y = cam_origin[1] + t * ray_robot[1]
            z = float(ID5_ROBOT_POS[2])
            updated[mid] = [x, y, z]

        if updated:
            for mid, pos in updated.items():
                MARKERS_3D[mid] = pos
            print(f"[CALIB] Updated markers: { {k: [round(v,1) for v in p] for k,p in updated.items()} }")

            if len(updated) >= 3:
                global PLANE_A, PLANE_B, PLANE_C
                pts = np.array(list(updated.values()))
                PLANE_A, PLANE_B, PLANE_C = fit_plane(pts)
                print(f"[CALIB] New plane: A={PLANE_A:.5f}  B={PLANE_B:.5f}  C={PLANE_C:.2f}")

                # Refine with the table markers if we have enough
                obj_pts, img_pts = [], []
                for i, mid in enumerate(ids_flat):
                    if mid in MARKERS_3D:
                        c = corners[i][0]
                        img_pts.append([float(np.mean(c[:, 0])), float(np.mean(c[:, 1]))])
                        obj_pts.append(MARKERS_3D[mid])

                if len(obj_pts) >= 4:
                    ok, rvec, tvec = cv2.solvePnP(
                        np.array(obj_pts, dtype=np.float32),
                        np.array(img_pts, dtype=np.float32),
                        CAM_MATRIX, BLANK_DIST,
                        flags=cv2.SOLVEPNP_IPPE
                    )
                    if ok:
                        self.rvec = rvec
                        self.tvec = tvec
                        print("[CALIB] Camera pose refined with table markers")
            else:
                print(f"[CALIB] Only {len(updated)} table marker(s) – plane kept")
        else:
            print("[CALIB] No table markers – previous plane kept")

        print("[CALIB] ===== Auto-calibration DONE =====")
        return True

    def update_3d_pose(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return

        aruco.drawDetectedMarkers(frame, corners, ids)
        ids_flat = ids.flatten().tolist()

        # Collect whatever markers from {0,1,2,3} are currently visible
        obj_pts, img_pts = [], []
        for i, mid in enumerate(ids_flat):
            if mid in MARKERS_3D:
                c = corners[i][0]
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))
                img_pts.append([cx, cy])
                obj_pts.append(MARKERS_3D[mid])

        if len(obj_pts) < 4:
            return

        obj_pts = np.array(obj_pts, dtype=np.float32)
        img_pts = np.array(img_pts, dtype=np.float32)

        # Prefer IPPE (best for planar markers)
        ok = False
        for flag in (cv2.SOLVEPNP_IPPE, cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_ITERATIVE):
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, img_pts,
                    CAM_MATRIX, BLANK_DIST,
                    flags=flag
                )
                if ok:
                    break
            except cv2.error:
                continue

        if ok:
            self.rvec = rvec
            self.tvec = tvec
            if not self.calibration_locked:
                print("[INFO] 3D Camera Pose successfully locked.")
                self.calibration_locked = True

    def pixel_to_3d_raycast(self, u, v):
        if self.rvec is None or self.tvec is None:
            return None

        pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
        norm_pt = cv2.undistortPoints(pt, CAM_MATRIX, BLANK_DIST)
        ray_cam = np.array([[norm_pt[0][0][0]], [norm_pt[0][0][1]], [1.0]])

        R, _ = cv2.Rodrigues(self.rvec)

        cam_pos_base = -R.T @ self.tvec
        cx = float(cam_pos_base[0][0])
        cy = float(cam_pos_base[1][0])
        cz = float(cam_pos_base[2][0])

        ray_base = R.T @ ray_cam
        rx = float(ray_base[0][0])
        ry = float(ray_base[1][0])
        rz = float(ray_base[2][0])

        effective_C = PLANE_C + OBJECT_Z_HEIGHT
        denominator = rz - PLANE_A * rx - PLANE_B * ry
        if abs(denominator) < 1e-8:
            return None

        t = (PLANE_A * cx + PLANE_B * cy + effective_C - cz) / denominator
        target_x = cx + t * rx
        target_y = cy + t * ry

        if YAW_TRIM_DEGREES != 0.0:
            theta = math.radians(YAW_TRIM_DEGREES)
            rot_x = target_x * math.cos(theta) - target_y * math.sin(theta)
            rot_y = target_x * math.sin(theta) + target_y * math.cos(theta)
            target_x, target_y = rot_x, rot_y

        return target_x + X_TRIM, target_y + Y_TRIM

    def _object_min_rect(self, frame, box):
        x1, y1, x2, y2 = [int(v) for v in box]
        h, w, _ = frame.shape
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return None

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        if (np.count_nonzero(otsu) / otsu.size) > 0.40:
            _, thresh = cv2.threshold(gray_roi, 185, 255, cv2.THRESH_BINARY_INV)
        else:
            thresh = otsu

        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best, best_score = None, -1.0
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < 30:
                continue
            rect = cv2.minAreaRect(c)
            rw, rh = float(rect[1][0]), float(rect[1][1])
            aspect = max(rw, rh) / (min(rw, rh) if min(rw, rh) > 1 else 1.0)
            score = area * (aspect if aspect >= 2.0 else 0.15)
            if score > best_score:
                best_score = score
                best = (rect, rw, rh)

        if best is None:
            return None
        return {"width": best[1], "height": best[2], "raw_angle": float(best[0][2])}

    def get_object_orientation(self, frame, box):
        info = self._object_min_rect(frame, box)
        if info is None:
            return 90.0

        angle = info["raw_angle"]
        rw, rh = info["width"], info["height"]

        if rw < rh:
            angle += 90.0

        optimal_angle = (angle + 90.0 - YAW_TRIM_DEGREES) % 180.0
        return optimal_angle

    def find_object(self, frame):
        if not self.calibration_locked:
            return None, None
        results = self.model.predict(frame, conf=0.18, verbose=False)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_center = (x1 + x2) / 2
                v_center = (y1 + y2) / 2
                coords = self.pixel_to_3d_raycast(u_center, v_center)
                if coords:
                    return coords, (x1, y1, x2, y2)
        return None, None

    async def run(self):
        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        print(f"\n[AGENT READY] Active Prompts: {self.target_prompts}")
        print("Controls:")
        print("  'C' = Auto-calibrate with ID-5 (base sticker)")
        print("  'c' = Change detection prompt")
        print("  'p' = Pick")
        print("  'q' / ESC = Quit\n")

        while True:
            ret, raw_frame = self.cap.read()
            if not ret:
                break

            frame = cv2.undistort(raw_frame, CAM_MATRIX, DIST_COEFFS)

            self.update_3d_pose(frame)
            coords, box = self.find_object(frame)

            if coords and box:
                ox, oy = coords
                x1, y1, x2, y2 = [int(v) for v in box]

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)
                cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), 5, (255, 100, 0), -1)

                roll_angle = self.get_object_orientation(frame, box)
                label = f"3D Target: X={ox:.1f}, Y={oy:.1f}, Roll={roll_angle:.0f}°"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)

            status = "3D POSE LOCKED" if self.calibration_locked else "SEARCHING FOR MARKERS"
            color = (0, 255, 0) if self.calibration_locked else (0, 0, 255)
            cv2.putText(frame, status, (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            if self.calibration_locked:
                cv2.putText(frame, "ID-5 base reference active", (30, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow("Precision Vision Agent", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                break
            elif key == ord('C'):
                print("\n[CALIB] Starting automatic calibration with ID-5 ...")
                if self.auto_calibrate(frame):
                    print("[CALIB] Success – you can now move the arm base freely.")
                else:
                    print("[CALIB] Failed – make sure ID-5 is clearly visible.")
            elif key == ord('c'):
                new_prompt = input("\nEnter target prompt: ").strip()
                if new_prompt:
                    self.target_prompts = [new_prompt]
                    self.model.set_classes(self.target_prompts)
            elif key == ord('p') and coords:
                obj_x, obj_y = coords
                target_roll = self.get_object_orientation(frame, box)
                z_hover = get_table_z(obj_x, obj_y, hover=True)
                z_grasp = get_table_z(obj_x, obj_y, hover=False) + Z_GRASP_OFFSET

                await self.send_cmd({"type": "grip", "angle": 130})
                await self.send_cmd({"type": "roll", "angle": target_roll})
                await self.wait_and_pump(1.0, f"ROTATING WRIST TO {target_roll:.0f}°")

                await self.arm_move(obj_x, obj_y, z_hover)
                await self.arm_move(obj_x, obj_y, z_grasp)

                await self.send_cmd({"type": "grip", "angle": 50})
                await self.wait_and_pump(1.5, "GRASPING...")

                await self.arm_move(obj_x, obj_y, z_hover)
                await self.arm_move(100.0, 200.0, 180.0)
                await self.send_cmd({"type": "grip", "angle": 130})
                await self.wait_and_pump(1.0, "DROPPING")

            await asyncio.sleep(0.001)

        self.cap.release()
        cv2.destroyAllWindows()
        if self.ws:
            await self.ws.close()

async def main():
    agent = PrecisionVisionAgent()
    await agent.connect()
    await agent.run()

if __name__ == "__main__":
    asyncio.run(main())