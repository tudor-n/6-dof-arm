import asyncio
import json
import cv2
import cv2.aruco as aruco
import numpy as np
import websockets
from ultralytics import YOLOWorld

# --- SYSTEM CONFIGURATION ---
WS_URI = "ws://localhost:8080/ws"
CAMERA_INDEX = 1

# --- CALIBRATED WITH TRUE CONFIG.JSON ---
MARKER_COORDS = {
    0: [-17.8,  203.2],  # Near Right
    1: [ 37.1, -190.8],  # Near Left
    2: [217.4,  157.9],  # Far Right
    3: [212.1, -191.0],  # Far Left
}

# --- SERVOING TUNING ---
# Color mask for the gripper marker (example: bright neon green HSV)
# Adjust these HSV ranges or switch to an ArUco ID on the wrist if preferred
GRIPPER_HSV_LOWER = np.array([35, 100, 100])
GRIPPER_HSV_UPPER = np.array([85, 255, 255])

def detect_gripper_pixel(frame):
    """Detects the gripper marker in pixel space via HSV color masking."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GRIPPER_HSV_LOWER, GRIPPER_HSV_UPPER)
    
    # Clean up noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    
    # Largest blob corresponds to the gripper tracker
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 50:
        return None
        
    m = cv2.moments(largest)
    if m["m00"] == 0:
        return None
    gx = int(m["m10"] / m["m00"])
    gy = int(m["m01"] / m["m00"])
    return (gx, gy)

SERVO_TOLERANCE_PX = 8.0     # Target pixel accuracy before descending (pixels)
SERVO_MAX_ITERATIONS = 5     # Maximum correction steps to avoid infinite loops
SERVO_GAIN = 0.85            # Damping factor to prevent overshooting

X_TRIM = 0.0
Y_TRIM = 14.0

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


class VisionAgent:
    def __init__(self):
        self.ws = None
        self.is_connected = False
        
        print("[INFO] Loading YOLO-World model...")
        self.model = YOLOWorld('yolov8s-world.pt')
        self.target_object = "screwdriver"
        
        # Multi-prompt synonyms to help detect thin hand tools from top-down
        self.target_prompts = [
            "screwdriver", 
            "tool handle", 
            "hand tool", 
            "metal tool", 
            "pen"
        ]
        self.model.set_classes(self.target_prompts)
        
        # Modern ArUco detector (OpenCV 4.7+)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.homography_matrix = None
        self.calibration_locked = False

    async def connect(self):
        """Attempts connection to the robot server; falls back to vision-only if server is off."""
        try:
            self.ws = await websockets.connect(WS_URI)
            self.is_connected = True
            print("[INFO] Successfully connected to Arm IK Server.")
        except (ConnectionRefusedError, OSError):
            print("[WARN] Robot server not reachable. Running in VISION-ONLY mode (Arm disconnected).")
            self.is_connected = False

    async def send_cmd(self, cmd):
        if not self.is_connected or self.ws is None:
            print(f"[SIMULATED CMD] {cmd}")
            return
        await self.ws.send(json.dumps(cmd))
        await asyncio.sleep(0.05)

    async def arm_move(self, x, y, z):
        print(f"Moving to X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f}")
        await self.send_cmd({"type": "tip", "x": x, "y": y, "z": z, "lock": False})
        await asyncio.sleep(2.0)

    async def gripper_action(self, action):
        """Controls the 360-degree dead-man switch gripper."""
        if action == "open":
            print("Opening gripper...")
            await self.send_cmd({"type": "grip", "angle": 130})
            await asyncio.sleep(1.5)
        elif action == "close":
            print("Closing gripper...")
            await self.send_cmd({"type": "grip", "angle": 50})
            await asyncio.sleep(1.5)
        
        print("Braking gripper.")
        await self.send_cmd({"type": "grip", "angle": 90})

    def update_homography(self, frame):
        """Detects ArUco tags and latches the homography matrix without dropping when blocked."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            ids_flat = ids.flatten().tolist()
            
            # Only calculate when all 4 markers are visible
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
        # If any marker is occluded by the arm or hand, keep using self.homography_matrix


    def pixel_to_mm(self, u, v):
     if self.homography_matrix is None:
        return None
     pt = np.array([[[u, v]]], dtype=np.float32)
     transformed = cv2.perspectiveTransform(pt, self.homography_matrix)
     tx = float(transformed[0][0][0]) + X_TRIM
     ty = float(transformed[0][0][1]) + Y_TRIM
     return tx, ty

    def find_object(self, frame):
        """Safe detection with lower confidence threshold and center-point extraction."""
        if self.homography_matrix is None:
            return None, None
            
        results = self.model.predict(frame, conf=0.18, verbose=False)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_center = float((x1 + x2) / 2)
                v_center = float((y1 + y2) / 2)  # Centroid for tabletop objects
                
                coords = self.pixel_to_mm(u_center, v_center)
                if coords is not None:
                    return coords, (int(x1), int(y1), int(x2), int(y2))
        return None, None

    async def visual_servo_align(self, target_px, cur_x, cur_y, hover_z):
     """Iteratively drives the arm at hover height until gripper center matches target pixel."""
     tx, ty = target_px
     adj_x, adj_y = cur_x, cur_y
    
     for step in range(SERVO_MAX_ITERATIONS):
        # 1. Capture fresh camera frame
        ret, frame = self.cap.read()
        if not ret:
            break
            
        # 2. Find where the gripper actually is in camera view
        grip_px = detect_gripper_pixel(frame)
        if grip_px is None:
            print(f"[SERVO] Warning: Gripper marker not visible. Proceeding with coarse coordinates.")
            break
            
        gx, gy = grip_px
        du = tx - gx
        dv = ty - gy
        err_dist = np.hypot(du, dv)
        
        print(f"[SERVO step {step+1}] Gripper at ({gx}, {gy}) -> Target at ({tx}, {ty}) | Error: {err_dist:.1f}px")
        
        # Check if alignment is tight enough
        if err_dist <= SERVO_TOLERANCE_PX:
            print("[SERVO] Aligned within tolerance!")
            break
            
        # Convert the pixel error vector into table millimeter adjustments using homography
        # Project current grip and target to mm
        grip_mm = self.pixel_to_mm(gx, gy)
        target_mm = self.pixel_to_mm(tx, ty)
        
        if grip_mm is None or target_mm is None:
            break
            
        dx_mm = (target_mm[0] - grip_mm[0]) * SERVO_GAIN
        dy_mm = (target_mm[1] - grip_mm[1]) * SERVO_GAIN
        
        adj_x += dx_mm
        adj_y += dy_mm
        
        # Nudge arm while maintaining hover height
        await self.arm_move(adj_x, adj_y, hover_z)
        await asyncio.sleep(0.4)  # Wait for mechanical settlement
        
     return adj_x, adj_y

    async def run(self):
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not cap.isOpened():
            print(f"Camera index {CAMERA_INDEX} failed, attempting index 0...")
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if not cap.isOpened():
                raise SystemExit("Error: Could not open any camera device.")

        print(f"\n[AGENT READY] Active Prompts: {self.target_prompts}")
        print("Controls: 'p' = Pick Target | 'c' = Change Target Prompt | 'q' = Quit\n")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            self.update_homography(frame)
            coords, box = self.find_object(frame)

            # Draw target overlay if found
            if coords and box:
                ox, oy = coords
                x1, y1, x2, y2 = box
                z_target = get_table_z(ox, oy, hover=False)
                
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)
                label = f"Target: X={ox:.1f}, Y={oy:.1f}, Z={z_target:.1f}mm"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)

            # Workspace HUD overlay
            status = "LOCKED" if self.calibration_locked else "WAITING FOR MARKERS"
            conn_txt = "ARM CONNECTED" if self.is_connected else "VISION-ONLY (SIMULATED)"
            cv2.putText(frame, f"Calibration: {status} | {conn_txt}", 
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                        (0, 255, 0) if self.calibration_locked else (0, 0, 255), 2)

            cv2.imshow("Vision Agent View", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                break
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
                z_hover = get_table_z(obj_x, obj_y, hover=True)
                z_grasp = get_table_z(obj_x, obj_y, hover=False)

                print(f"\n[ACTION] Executing pick at X={obj_x:.1f}, Y={obj_y:.1f}")
                print(f"[ACTION] Incline-compensated heights: Grasp Z={z_grasp:.1f}mm, Hover Z={z_hover:.1f}mm")
                
                # Full Pick-and-Place sequence
                await self.gripper_action("open")
                await self.arm_move(obj_x, obj_y, z_hover)
                await self.arm_move(obj_x, obj_y, z_grasp)
                await self.gripper_action("close")
                await self.arm_move(obj_x, obj_y, z_hover)
                await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], Z_DROPOFF)
                await self.gripper_action("open")
                await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], z_hover)
                print("[ACTION] Pick sequence completed.")

            await asyncio.sleep(0.001)

        cap.release()
        cv2.destroyAllWindows()
        if self.ws:
            await self.ws.close()

async def main():
    agent = VisionAgent()
    await agent.connect()
    await agent.run()

if __name__ == "__main__":
    asyncio.run(main())