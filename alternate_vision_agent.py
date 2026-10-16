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

def _agent_dbg(hypothesis_id, location, message, data, run_id="open-loop-ik"):
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

# --- CALIBRATED WITH TRUE CONFIG.JSON ---
MARKER_COORDS = {
    0: [-17.8,  203.2],  # Near Left (relative to camera perspective)
    1: [ 37.1, -190.8],  # Near Right
    2: [217.4,  157.9],  # Far Left
    3: [212.1, -191.0],  # Far Right
}

# --- KINEMATIC OFFSETS ---
# Used to correct global mechanical slop and base rotation in the arm
X_TRIM = 0.0          # Increase to push reach outward, Decrease to pull inward
Y_TRIM = 0.0          # Increase to shift reach left, Decrease to shift right
YAW_TRIM_DEGREES = 5.0 # Positive twists grid counter-clockwise, Negative clockwise
Z_GRASP_OFFSET = -20.0 

Z_DROPOFF = 180.0
DROPOFF_XY = (100.0, 200.0)

PLANE_A = 0.1197
PLANE_B = 0.0573
PLANE_C = -14.35

def get_table_z(target_x, target_y, hover=False):
    """Exact physical table surface calculation derived from config.json FK."""
    z_surface = PLANE_A * target_x + PLANE_B * target_y + PLANE_C
    if hover:
        return z_surface + 130.0
    return z_surface

class AlternateVisionAgent:
    def __init__(self):
        self.ws = None
        self.is_connected = False
        self.cap = None
        
        print("[INFO] Loading YOLO-World model...")
        self.model = YOLOWorld('yolov8s-world.pt')
        self.target_prompts = [
            "screwdriver", 
            "tool handle", 
            "hand tool", 
            "metal tool", 
            "pen",
            "pencil"
        ]
        self.model.set_classes(self.target_prompts)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.homography_matrix = None
        self.calibration_locked = False

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
        """Keeps the OpenCV feed live while waiting for the arm to move."""
        end_time = asyncio.get_event_loop().time() + duration
        while asyncio.get_event_loop().time() < end_time:
            ret, frame = self.cap.read()
            if ret:
                self.update_homography(frame)
                cv2.putText(frame, msg, (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
                cv2.imshow("Alternate Vision Agent", frame)
                cv2.waitKey(1)
            await asyncio.sleep(0.03)

    async def arm_move(self, x, y, z):
        print(f"Moving to X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f}")
        await self.send_cmd({"type": "tip", "x": x, "y": y, "z": z, "lock": False})
        await self.wait_and_pump(2.0, f"MOVING ARM TO Z={z:.0f}...")

    def get_object_orientation(self, frame, box):
        """Calculates the perpendicular grasping angle for an object, compensating for yaw."""
        info = self._object_min_rect(frame, box)
        if info is None:
            return 90.0
            
        angle = info["raw_angle"]
        width = info["width"]
        height = info["height"]
        
        # Normalize the angle to follow the major axis (length) of the object
        if width < height:
            angle += 90.0
            
        # The gripper needs to clamp *across* the minor axis (perpendicular to length)
        gripper_target_angle = angle + 90.0
        
        # Compensate for the physical rotation of the robot's base
        gripper_target_angle -= YAW_TRIM_DEGREES
        
        # Normalize to servo bounds [0, 180)
        return gripper_target_angle % 180.0
    
    async def gripper_action(self, action):
        if action == "open":
            print("Opening gripper...")
            await self.send_cmd({"type": "grip", "angle": 130})
            await self.wait_and_pump(1.5, "OPENING GRIPPER...")
        elif action == "close":
            print("Closing gripper...")
            await self.send_cmd({"type": "grip", "angle": 50})
            await self.wait_and_pump(2.0, "CLOSING GRIPPER...")
        
        print("Braking gripper.")
        await self.send_cmd({"type": "grip", "angle": 90})

    def update_homography(self, frame):
        """Continuously maps the flat table workspace based on the 4 ArUco anchors."""
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

    def _object_min_rect(self, frame, box):
        x1, y1, x2, y2 = box
        h, w, _ = frame.shape
        x1, y1, x2, y2 = max(0, int(x1)), max(0, int(y1)), min(w, int(x2)), min(h, int(y2))
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0: return None
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, otsu = cv2.threshold(gray_roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        fg_ratio = float(np.count_nonzero(otsu)) / float(otsu.size)
        if fg_ratio > 0.40:
            _, thresh = cv2.threshold(gray_roi, 185, 255, cv2.THRESH_BINARY_INV)
        else:
            thresh = otsu
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return None
        
        best, best_score = None, -1.0
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < 30: continue
            rect = cv2.minAreaRect(c)
            rw, rh = float(rect[1][0]), float(rect[1][1])
            thick = min(rw, rh) if rw > 1 and rh > 1 else 1.0
            length = max(rw, rh)
            aspect = length / thick
            score = area * (aspect if aspect >= 2.0 else 0.15)
            if score > best_score:
                best_score = score
                best = (rect, rw, rh, area, aspect)
        if best is None: return None
        
        rect, rw, rh, area, aspect = best
        
        # rect[2] handles OpenCV 4.x angle mappings differently based on aspect ratio
        return {
            "width": rw,
            "height": rh,
            "raw_angle": float(rect[2]),
        }

    def pixel_to_mm(self, u, v):
        """Maps 2D pixel to 3D workspace, applying rotational Yaw Trims and Linear Trims."""
        if self.homography_matrix is None:
            return None
            
        pt = np.array([[[u, v]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography_matrix)
        raw_x = float(transformed[0][0][0])
        raw_y = float(transformed[0][0][1])
        
        # Rotate grid to match twisted physical base
        theta = math.radians(YAW_TRIM_DEGREES)
        rot_x = raw_x * math.cos(theta) - raw_y * math.sin(theta)
        rot_y = raw_x * math.sin(theta) + raw_y * math.cos(theta)
        
        final_x = rot_x + X_TRIM
        final_y = rot_y + Y_TRIM
        
        return final_x, final_y

    def find_object(self, frame):
        if self.homography_matrix is None:
            return None, None
            
        results = self.model.predict(frame, conf=0.18, verbose=False)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_center = float((x1 + x2) / 2)
                v_center = float((y1 + y2) / 2)
                
                coords = self.pixel_to_mm(u_center, v_center)
                if coords is not None:
                    return coords, (int(x1), int(y1), int(x2), int(y2))
        return None, None

    async def run(self):
        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if not self.cap.isOpened():
                raise SystemExit("Error: Could not open any camera device.")

        print(f"\n[AGENT READY] Active Prompts: {self.target_prompts}")
        print("Controls:")
        print("  'p' = Pick Target")
        print("  'c' = Change Target Prompt")
        print("  'q' = Quit\n")

        while True:
            ret, frame = self.cap.read()
            if not ret: break

            self.update_homography(frame)
            coords, box = self.find_object(frame)

            if coords and box:
                ox, oy = coords
                x1, y1, x2, y2 = box
                z_target = get_table_z(ox, oy, hover=False)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)
                cv2.circle(frame, ((x1+x2)//2, (y1+y2)//2), 5, (255, 100, 0), -1)
                
                # Visualized wrist roll target line
                roll_angle = self.get_object_orientation(frame, box)
                r_rad = math.radians(roll_angle)
                cx, cy = (x1+x2)//2, (y1+y2)//2
                line_len = 30
                end_x = int(cx + line_len * math.cos(r_rad))
                end_y = int(cy + line_len * math.sin(r_rad))
                cv2.line(frame, (cx, cy), (end_x, end_y), (0, 0, 255), 2)
                
                label = f"IK Target: X={ox:.1f}, Y={oy:.1f}, Z={z_target:.1f}mm"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)

            status = "WORKSPACE MAPPED" if self.calibration_locked else "WAITING FOR CORNER MARKERS"
            conn_txt = "IK SERVER CONNECTED" if self.is_connected else "VISION-ONLY (SIMULATED)"
            cv2.putText(frame, f"{status} | {conn_txt}", 
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                        (0, 255, 0) if self.calibration_locked else (0, 0, 255), 2)

            cv2.imshow("Alternate Vision Agent", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                break
            elif key == ord('c'):
                new_prompt = input("\nEnter target prompt: ").strip()
                if new_prompt:
                    self.target_prompts = [new_prompt]
                    self.model.set_classes(self.target_prompts)
                    print(f"[AGENT] Updated detection to: '{new_prompt}'")
            elif key == ord('p'):
                if not coords:
                    print("[AGENT] Cannot trigger pick: no object detected.")
                    continue
                
                obj_x, obj_y = coords
                target_roll = self.get_object_orientation(frame, box)
                
                z_hover = get_table_z(obj_x, obj_y, hover=True)             
                z_grasp = get_table_z(obj_x, obj_y, hover=False) + Z_GRASP_OFFSET

                print(f"\n[ACTION] Executing Open-Loop IK Pick at X={obj_x:.1f}, Y={obj_y:.1f}")
                
                # 1. Open Gripper & Rotate Wrist
                await self.gripper_action("open")
                await self.send_cmd({"type": "roll", "angle": target_roll})
                await self.wait_and_pump(1.0, f"ROTATING WRIST TO {target_roll:.0f}°...")

                # 2. Hover directly over target
                await self.arm_move(obj_x, obj_y, z_hover)
                
                # 3. Descend to target
                await self.arm_move(obj_x, obj_y, z_grasp)
                
                # 4. Grasp
                await self.gripper_action("close")
                
                # 5. Retract
                await self.arm_move(obj_x, obj_y, z_hover)
                
                # 6. Dropoff
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
    agent = AlternateVisionAgent()
    await agent.connect()
    await agent.run()

if __name__ == "__main__":
    asyncio.run(main())