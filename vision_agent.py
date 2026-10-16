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

# #region agent log
_DBG_LOG = Path(__file__).resolve().parent / "debug-8a3ee4.log"

def _agent_dbg(hypothesis_id, location, message, data, run_id="pre-fix"):
    try:
        payload = {
            "sessionId": "8a3ee4",
            "timestamp": int(time.time() * 1000),
            "hypothesisId": hypothesis_id,
            "location": location,
            "message": message,
            "data": data,
            "runId": run_id,
        }
        with open(_DBG_LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload) + "\n")
    except Exception:
        pass
# #endregion

# --- SYSTEM CONFIGURATION ---
WS_URI = "ws://localhost:8080/ws"
CAMERA_INDEX = 0
CONFIG_FILE = Path(__file__).resolve().parent / "vision_config.json"

# --- CALIBRATED WITH TRUE CONFIG.JSON ---
MARKER_COORDS = {
    0: [-17.8,  203.2],  # Near Left (relative to camera perspective)
    1: [ 37.1, -190.8],  # Near Right
    2: [217.4,  157.9],  # Far Left
    3: [212.1, -191.0],  # Far Right
}

# --- SERVOING CONFIGURATION ---
GRIPPER_MARKER_ID = 4        

SERVO_TOLERANCE_PX = 4.0     
SERVO_MAX_ITERATIONS = 5     

X_TRIM = 0.0
Y_TRIM = 24.0
Z_GRASP_OFFSET = -20 

Z_DROPOFF = 180.0
DROPOFF_XY = (100.0, 200.0)

PLANE_A = 0.1197
PLANE_B = 0.0573
PLANE_C = -14.35

def get_table_z(target_x, target_y, hover=False):
    z_surface = PLANE_A * target_x + PLANE_B * target_y + PLANE_C
    if hover:
        return z_surface + 130.0
    return z_surface

class VisionAgent:
    def __init__(self):
        self.ws = None
        self.is_connected = False
        self.cap = None
        
        # Load Interactive UI Calibration State
        self.load_config()
        
        # UI Dragging State
        self.drag_state = None  # Can be None, 'tcp', or 'aruco'
        self.current_tcp_pt = (0, 0)
        self.current_aruco_pt = (0, 0)
        self.raw_aruco_pt = (0, 0)
        self.base_marker_angle = 0.0
        self.active_offset = (self.offset_tip + self.offset_belly) // 2

        self.last_marker_pose = None
        
        print("[INFO] Loading YOLO-World model...")
        self.model = YOLOWorld('yolov8s-world.pt')
        self.target_prompts = [
            "screwdriver", 
            "tool handle", 
            "hand tool", 
            "metal tool", 
            "pen"
        ]
        self.model.set_classes(self.target_prompts)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.homography_matrix = None
        self.calibration_locked = False

    def load_config(self):
        """Loads offset configuration from file or initializes defaults."""
        if CONFIG_FILE.exists():
            with open(CONFIG_FILE, "r") as f:
                config = json.load(f)
                self.offset_tip = config.get("offset_tip", 273)
                self.offset_belly = config.get("offset_belly", 108)
                self.offset_lateral = config.get("offset_lateral", 0.0)
                self.tcp_angle = config.get("tcp_angle", 0.0)
                self.marker_bias_x = config.get("marker_bias_x", 0)
                self.marker_bias_y = config.get("marker_bias_y", 0)
            print(f"[INFO] Loaded configuration from {CONFIG_FILE.name}")
        else:
            self.offset_tip = 273
            self.offset_belly = 108
            self.offset_lateral = 0.0
            self.tcp_angle = 0.0
            self.marker_bias_x = 0
            self.marker_bias_y = 0
            self.save_config()

    def save_config(self):
        """Saves current offset configuration to file."""
        with open(CONFIG_FILE, "w") as f:
            json.dump({
                "offset_tip": self.offset_tip,
                "offset_belly": self.offset_belly,
                "offset_lateral": self.offset_lateral,
                "tcp_angle": self.tcp_angle,
                "marker_bias_x": self.marker_bias_x,
                "marker_bias_y": self.marker_bias_y
            }, f, indent=4)

    def mouse_callback(self, event, x, y, flags, param):
        """Handles mouse interactions to drag UI points and update configurations."""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Prioritize clicking the TCP point first, then ArUco center
            if math.hypot(x - self.current_tcp_pt[0], y - self.current_tcp_pt[1]) < 25:
                self.drag_state = 'tcp'
            elif math.hypot(x - self.current_aruco_pt[0], y - self.current_aruco_pt[1]) < 25:
                self.drag_state = 'aruco'

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drag_state == 'tcp':
                # Recalculate offsets based on new distance and angle from the adjusted ArUco center
                dx = x - self.current_aruco_pt[0]
                dy = y - self.current_aruco_pt[1]
                new_dist = math.hypot(dx, dy)
                new_angle = math.atan2(dy, dx)
                
                # We distribute the drag difference symmetrically so the UI doesn't jump
                dist_delta = int(new_dist - self.active_offset)
                self.offset_tip += dist_delta
                self.offset_belly += dist_delta
                
                # Lateral offset is zeroed out since length/angle solves the TCP natively during a drag
                self.offset_lateral = 0.0
                self.tcp_angle = new_angle - self.base_marker_angle

            elif self.drag_state == 'aruco':
                # Update visual bias logic to manually trim marker jitter
                self.marker_bias_x = x - self.raw_aruco_pt[0]
                self.marker_bias_y = y - self.raw_aruco_pt[1]

        elif event == cv2.EVENT_LBUTTONUP:
            if self.drag_state is not None:
                self.save_config()
                print("[INFO] Configuration saved to disk.")
            self.drag_state = None

    async def connect(self):
        try:
            self.ws = await websockets.connect(WS_URI, ping_interval=None)
            self.is_connected = True
            print("[INFO] Successfully connected to Arm IK Server.")
        except (ConnectionRefusedError, OSError):
            print("[WARN] Robot server not reachable. Running in VISION-ONLY mode.")
            self.is_connected = False

    async def send_cmd(self, cmd):
        if not self.is_connected or self.ws is None:
            print(f"[SIMULATED CMD] {cmd}")
            return
        await self.ws.send(json.dumps(cmd))
        await asyncio.sleep(0.02)

    async def wait_and_pump(self, duration, msg=""):
        end_time = asyncio.get_event_loop().time() + duration
        while asyncio.get_event_loop().time() < end_time:
            ret, frame = self.cap.read()
            if ret:
                self.update_homography(frame)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = self.detector.detectMarkers(gray)
                
                self.active_offset = (self.offset_tip + self.offset_belly) // 2
                self.get_dynamic_gripper_pos(corners, ids, self.active_offset, frame)
                
                cv2.putText(frame, msg, (30, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
                cv2.imshow("Vision Agent View", frame)
                cv2.waitKey(1)
            await asyncio.sleep(0.03)

    async def arm_move(self, x, y, z):
        print(f"Moving to X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f}")
        await self.send_cmd({"type": "tip", "x": x, "y": y, "z": z, "lock": False})
        await self.wait_and_pump(2.0, f"MOVING ARM TO Z={z:.0f}...")

    def get_object_orientation(self, frame, box):
        info = self._object_min_rect(frame, box)
        if info is None:
            return 90.0
        angle = info["raw_angle"]
        if info["width"] < info["height"]:
            angle += 90
        roll_angle = 90.0 + angle
        return max(0.0, min(180.0, roll_angle))
    
    async def gripper_action(self, action):
        if action == "open":
            print("Opening gripper...")
            await self.send_cmd({"type": "grip", "angle": 130})
            await self.wait_and_pump(1.5, "OPENING GRIPPER...")
        elif action == "close":
            print("Closing gripper...")
            await self.send_cmd({"type": "grip", "angle": 50})
            await self.wait_and_pump(4.5, "CLOSING GRIPPER...")
        
        print("Braking gripper.")
        await self.send_cmd({"type": "grip", "angle": 90})

    def update_homography(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            ids_flat = ids.flatten().tolist()
            
            if all(k in ids_flat for k in [0, 1, 2, 3]):
                img_pts = []
                phys_pts = []
                for i, marker_id in enumerate(ids_flat):
                    if marker_id in MARKER_COORDS:
                        c = corners[i][0]
                        cx = float(np.mean(c[:, 0]))
                        cy = float(np.mean(c[:, 1]))
                        img_pts.append([cx, cy])
                        phys_pts.append(MARKER_COORDS[marker_id])
                
                if len(img_pts) == 4:
                    H, _ = cv2.findHomography(
                        np.array(img_pts, dtype=np.float32),
                        np.array(phys_pts, dtype=np.float32)
                    )
                    if H is not None:
                        self.homography_matrix = H
                        if not self.calibration_locked:
                            print("[INFO] Homography successfully locked.")
                            self.calibration_locked = True

    def get_dynamic_gripper_pos(self, corners, ids, current_offset, frame=None):
        if ids is None: return None
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == GRIPPER_MARKER_ID:
                c = corners[i][0]
                top_left, top_right = c[0], c[1]
                bottom_right = c[2]

                raw_center_x = int((top_left[0] + bottom_right[0]) / 2)
                raw_center_y = int((top_left[1] + bottom_right[1]) / 2)

                # Keep track of un-biased ArUco center for calibration mapping
                self.raw_aruco_pt = (raw_center_x, raw_center_y)

                # Apply visual UI biases to the marker
                center_x = raw_center_x + self.marker_bias_x
                center_y = raw_center_y + self.marker_bias_y
                self.current_aruco_pt = (center_x, center_y)

                # Calculate standard and modified angles
                base_angle = math.atan2(top_right[1] - top_left[1], top_right[0] - top_left[0])
                self.base_marker_angle = base_angle
                angle = base_angle + self.tcp_angle

                gripper_x = int(center_x + (current_offset * math.cos(angle)) - (self.offset_lateral * math.sin(angle)))
                gripper_y = int(center_y + (current_offset * math.sin(angle)) + (self.offset_lateral * math.cos(angle)))
                self.current_tcp_pt = (gripper_x, gripper_y)

                if frame is not None:
                    # Point 1: The modified ArUco Location (Vibrant Magenta)
                    cv2.circle(frame, (center_x, center_y), 6, (255, 0, 255), -1)
                    cv2.putText(frame, "ArUco", (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    
                    # Point 2: The Calculated Gripper TCP (Vibrant Cyan)
                    cv2.circle(frame, (gripper_x, gripper_y), 6, (255, 255, 0), -1)
                    cv2.putText(frame, "Calc Tip", (gripper_x + 10, gripper_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Connection visualizer
                    cv2.line(frame, (center_x, center_y), (gripper_x, gripper_y), (180, 180, 180), 2)

                marker_w = float(np.linalg.norm(top_right - top_left))
                self.last_marker_pose = {
                    "cx": center_x, "cy": center_y, "angle": angle,
                    "marker_w": marker_w, "t": time.time(),
                }
                return (gripper_x, gripper_y, angle, marker_w, center_x, center_y)
        return None

    def tcp_from_marker_pose(self, pose, current_offset):
        cx, cy, angle = pose["cx"], pose["cy"], pose["angle"]
        gx = int(cx + (current_offset * math.cos(angle)) - (self.offset_lateral * math.sin(angle)))
        gy = int(cy + (current_offset * math.sin(angle)) + (self.offset_lateral * math.cos(angle)))
        return (gx, gy, angle, pose["marker_w"], cx, cy)

    def _object_min_rect(self, frame, box):
        x1, y1, x2, y2 = box
        h, w, _ = frame.shape
        x1, y1, x2, y2 = max(0, int(x1)), max(0, int(y1)), min(w, int(x2)), min(h, int(y2))
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return None
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        fg_ratio = float(np.count_nonzero(otsu)) / float(otsu.size)
        if fg_ratio > 0.40:
            _, thresh = cv2.threshold(gray_roi, 185, 255, cv2.THRESH_BINARY_INV)
        else:
            thresh = otsu
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        best, best_score = None, -1.0
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < 30:
                continue
            rect = cv2.minAreaRect(c)
            rw, rh = float(rect[1][0]), float(rect[1][1])
            thick = min(rw, rh) if rw > 1 and rh > 1 else 1.0
            length = max(rw, rh)
            aspect = length / thick
            score = area * (aspect if aspect >= 2.0 else 0.15)
            if score > best_score:
                best_score = score
                best = (rect, rw, rh, area, aspect)
        if best is None:
            return None
        rect, rw, rh, area, aspect = best
        return {
            "n_contours": len(contours),
            "area": area,
            "width": rw,
            "height": rh,
            "thickness": min(rw, rh) if rw > 0 and rh > 0 else 0.0,
            "length": max(rw, rh),
            "aspect": aspect,
            "fg_ratio": fg_ratio,
            "raw_angle": float(rect[2]),
            "rect": rect,
        }

    def pixel_to_mm(self, u, v):
        if self.homography_matrix is None:
            return None
        pt = np.array([[[u, v]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography_matrix)
        tx = float(transformed[0][0][0]) + X_TRIM
        ty = float(transformed[0][0][1]) + Y_TRIM
        return tx, ty

    def calculate_dynamic_offset(self, box, frame=None):
        x1, y1, x2, y2 = box
        aabb_max = max(x2 - x1, y2 - y1)
        obj_size = aabb_max
        if frame is not None:
            info = self._object_min_rect(frame, box)
            if info is not None and info.get("aspect", 0) >= 2.5 and info["thickness"] > 1.0:
                thickness = info["thickness"]
                obj_size = thickness
            else:
                obj_size = 30.0
        
        small_obj_px = 30.0
        large_obj_px = 150.0
        
        clamped_size = max(small_obj_px, min(large_obj_px, obj_size))
        ratio = (clamped_size - small_obj_px) / (large_obj_px - small_obj_px)
        offset = self.offset_tip + ratio * (self.offset_belly - self.offset_tip)
        return int(offset)

    def find_object(self, frame):
        if self.homography_matrix is None:
            return None, None, None  
            
        results = self.model.predict(frame, conf=0.18, verbose=False)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_center = float((x1 + x2) / 2)
                v_center = float((y1 + y2) / 2)
                
                coords = self.pixel_to_mm(u_center, v_center)
                if coords is not None:
                    return coords, (int(x1), int(y1), int(x2), int(y2)), (u_center, v_center)
        return None, None, None

    async def _acquire_gripper_px(self, target_offset, tries=15):
        last_frame = None
        for attempt in range(tries):
            ret, frame = self.cap.read()
            if not ret:
                await asyncio.sleep(0.04)
                continue
            last_frame = frame
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            
            # Sync active offset for visualization
            self.active_offset = target_offset
            grip_px = self.get_dynamic_gripper_pos(corners, ids, target_offset, frame)
            
            if grip_px is not None:
                return grip_px, frame
            await asyncio.sleep(0.04)
        pose = self.last_marker_pose
        if pose is not None and (time.time() - pose["t"]) < 3.0:
            grip_px = self.tcp_from_marker_pose(pose, target_offset)
            return grip_px, last_frame
        return None, last_frame

    async def visual_servo_align(self, target_px_u, target_px_v, cur_x, cur_y, hover_z, target_offset, phase="servo"):
        tx, ty = target_px_u, target_px_v
        adj_x, adj_y = cur_x, cur_y
        
        SAFE_SERVO_GAIN = 0.30 
        MAX_STEP_MM = 15.0       
    
        for step in range(SERVO_MAX_ITERATIONS):
            grip_px, frame = await self._acquire_gripper_px(target_offset)
            if frame is None and grip_px is None:
                break
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)

            if grip_px is None:
                print(f"[SERVO] Warning: Gripper ArUco (ID {GRIPPER_MARKER_ID}) not visible. Falling back to open-loop.")
                break
                
            gx, gy, grip_angle, marker_w, cx, cy = grip_px
            du = tx - gx
            dv = ty - gy
            err_dist = np.hypot(du, dv)
            
            print(f"[SERVO step {step+1}] Gripper at ({gx}, {gy}) -> Target at ({tx}, {ty}) | Error: {err_dist:.1f}px")
            
            cv2.putText(frame, f"ALIGNING... {err_dist:.1f}px", (30, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
            cv2.imshow("Vision Agent View", frame)
            cv2.waitKey(1)
            
            if err_dist <= SERVO_TOLERANCE_PX:
                print("[SERVO] Aligned within tolerance!")
                break
                
            grip_mm = self.pixel_to_mm(gx, gy)
            target_mm = self.pixel_to_mm(tx, ty)
            
            if grip_mm is None or target_mm is None: break
                
            dx_mm = (target_mm[0] - grip_mm[0]) * SAFE_SERVO_GAIN
            dy_mm = (target_mm[1] - grip_mm[1]) * SAFE_SERVO_GAIN
            
            dx_mm = max(-MAX_STEP_MM, min(MAX_STEP_MM, dx_mm))
            dy_mm = max(-MAX_STEP_MM, min(MAX_STEP_MM, dy_mm))

            adj_x += dx_mm
            adj_y += dy_mm
            
            await self.send_cmd({"type": "tip", "x": adj_x, "y": adj_y, "z": hover_z, "lock": False})
            await self.wait_and_pump(0.6, f"SERVO NUDGE {step+1}/{SERVO_MAX_ITERATIONS}")
            
        return adj_x, adj_y

    async def run(self):
        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not self.cap.isOpened():
            print(f"Camera index {CAMERA_INDEX} failed, attempting index 0...")
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if not self.cap.isOpened():
                raise SystemExit("Error: Could not open any camera device.")

        # Create Window and hook Mouse Callback
        cv2.namedWindow("Vision Agent View", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("Vision Agent View", self.mouse_callback)

        print(f"\n[AGENT READY] Active Prompts: {self.target_prompts}")
        print("Controls:")
        print("  <Mouse Drag> = Drag ArUco (Magenta) or TCP (Cyan) on-screen to calibrate")
        print("  'p' = Pick Target")
        print("  'c' = Change Target Prompt")
        print("  '[' / ']' = Adjust Tip Offset")
        print("  '-' / '=' = Adjust Belly Offset")
        print("  ';' / ''' = Adjust Lateral Offset")
        print("  ',' / '.' = Adjust TCP Rotation Angle")
        print("  'q' = Quit\n")

        while True:
            ret, frame = self.cap.read()
            if not ret: break

            self.update_homography(frame)
            coords, box, pixel_centroid = self.find_object(frame)

            if coords and box:
                self.active_offset = self.calculate_dynamic_offset(box, frame)
                ox, oy = coords
                x1, y1, x2, y2 = box
                z_target = get_table_z(ox, oy, hover=False)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)
                label = f"Target: X={ox:.1f}, Y={oy:.1f}, Z={z_target:.1f}mm"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)
            else:
                self.active_offset = (self.offset_tip + self.offset_belly) // 2
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            
            # The function handles calculating, drawing, and recording state for the mouse hooks
            self.get_dynamic_gripper_pos(corners, ids, self.active_offset, frame)

            status = "LOCKED" if self.calibration_locked else "WAITING FOR MARKERS"
            conn_txt = "ARM CONNECTED" if self.is_connected else "VISION-ONLY (SIMULATED)"
            cv2.putText(frame, f"Calibration: {status} | {conn_txt}", 
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                        (0, 255, 0) if self.calibration_locked else (0, 0, 255), 2)
            
            cv2.putText(frame, f"Tip: {self.offset_tip} [ ]", (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(frame, f"Belly: {self.offset_belly} - =", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(frame, f"Lat: {self.offset_lateral} ; '", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(frame, f"Ang: {math.degrees(self.tcp_angle):.0f}deg , .", (30, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
            cv2.putText(frame, f"Marker Bias: ({self.marker_bias_x:.1f}, {self.marker_bias_y:.1f})", (30, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

            cv2.imshow("Vision Agent View", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                break
            elif key == ord('['): self.offset_tip -= 1; self.save_config()
            elif key == ord(']'): self.offset_tip += 1; self.save_config()
            elif key == ord('-'): self.offset_belly -= 1; self.save_config()
            elif key == ord('='): self.offset_belly += 1; self.save_config()
            elif key == ord(';'): self.offset_lateral -= 1; self.save_config()
            elif key == ord('\''): self.offset_lateral += 1; self.save_config()
            elif key == ord(','): self.tcp_angle -= 0.05; self.save_config()
            elif key == ord('.'): self.tcp_angle += 0.05; self.save_config()
            elif key == ord('c'):
                new_prompt = input("\nEnter target prompt (e.g., 'screwdriver', 'pen'): ").strip()
                if new_prompt:
                    self.target_prompts = [new_prompt]
                    self.model.set_classes(self.target_prompts)
                    print(f"[AGENT] Updated detection to: '{new_prompt}'")
            elif key == ord('p'):
                if not coords:
                    print("[AGENT] Cannot trigger pick: no object detected.")
                    continue
                
                obj_x, obj_y = coords
                px_u, px_v = pixel_centroid
                
                target_roll = self.get_object_orientation(frame, box)
                print(f"[ACTION] Detected object angle: {target_roll:.1f}° -> Setting wrist roll.")

                z_hover = get_table_z(obj_x, obj_y, hover=True)             
                z_grasp = get_table_z(obj_x, obj_y, hover=False) + Z_GRASP_OFFSET
                z_servo = z_grasp + 15.0  

                print(f"\n[ACTION] Executing pick at X={obj_x:.1f}, Y={obj_y:.1f}")
                
                await self.gripper_action("open")
                
                await self.send_cmd({"type": "roll", "angle": target_roll})
                await self.wait_and_pump(1.0, f"ROTATING WRIST TO {target_roll:.0f}°...")

                self.active_offset = self.calculate_dynamic_offset(box, frame)
                
                await self.arm_move(obj_x, obj_y, z_hover)
                await self.arm_move(obj_x, obj_y, z_servo)
                refined_x, refined_y = obj_x, obj_y
                
                await self.arm_move(refined_x, refined_y, z_grasp)
                await self.gripper_action("close")
                await self.arm_move(refined_x, refined_y, z_hover)
                
                await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], Z_DROPOFF)
                await self.gripper_action("open")
                await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], Z_DROPOFF + 30)
                print("[ACTION] Pick sequence completed.")

            await asyncio.sleep(0.001)

        self.cap.release()
        cv2.destroyAllWindows()
        if self.ws:
            await self.ws.close()

async def main():
    agent = VisionAgent()
    await agent.connect()
    await agent.run()

if __name__ == "__main__":
    asyncio.run(main())