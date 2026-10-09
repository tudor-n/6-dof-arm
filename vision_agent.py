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

# Calibrated physical locations of the 4 markers (X, Y in mm)
MARKER_COORDS = {
    0: [86.0,  -202.5],  # Near Right
    1: [98.1,   202.5],  # Near Left
    2: [261.7, -201.0],  # Far Right
    3: [268.0,  201.0]   # Far Left
}

# Hover and Grasp Heights (Z in mm)
Z_HOVER = 160.0
Z_GRASP = 25.0
Z_DROPOFF = 180.0
DROPOFF_XY = (100.0, 200.0)

class VisionAgent:
    def __init__(self):
        self.ws = None
        self.is_connected = False
        
        print("[INFO] Loading YOLO-World model...")
        self.model = YOLOWorld('yolov8s-world.pt')
        self.target_object = "screwdriver"
        self.model.set_classes([self.target_object])
        
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
            print("[WARN] Robot server not running. Running in VISION-ONLY mode (Arm disconnected).")
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
        """Detects ArUco tags and updates/latches the homography matrix."""
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
                
                H, _ = cv2.findHomography(
                    np.array(img_pts, dtype=np.float32),
                    np.array(phys_pts, dtype=np.float32)
                )
                if H is not None:
                    self.homography_matrix = H
                    if not self.calibration_locked:
                        print("[INFO] Homography locked.")
                        self.calibration_locked = True
                return True
        return False

    def pixel_to_mm(self, u, v):
        if self.homography_matrix is None:
            return None
        pt = np.array([[[u, v]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography_matrix)
        return float(transformed[0][0][0]), float(transformed[0][0][1])

    def find_object(self, frame):
        results = self.model.predict(frame, conf=0.30, verbose=False)
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_center = float((x1 + x2) / 2)
                v_bottom = float(y2)  # Base of object on table surface
                
                coords = self.pixel_to_mm(u_center, v_bottom)
                if coords:
                    return coords, (int(x1), int(y1), int(x2), int(y2))
        return None, None

    async def run(self):
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        if not cap.isOpened():
            print(f"Camera index {CAMERA_INDEX} failed, falling back to 0...")
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if not cap.isOpened():
                raise SystemExit("Error: Could not open camera.")

        print(f"\n[AGENT READY] Target: '{self.target_object}'")
        print("Controls: 'p' = Pick Target | 'c' = Change Target Prompt | 'q' = Quit\n")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            self.update_homography(frame)
            coords, box = self.find_object(frame)

            # Draw target overlay if detected
            if coords and box:
                ox, oy = coords
                x1, y1, x2, y2 = box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)
                label = f"{self.target_object}: X={ox:.1f}mm, Y={oy:.1f}mm"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 0), 2)

            # System HUD overlay
            status = "CALIBRATED" if self.homography_matrix is not None else "SEARCHING MARKERS"
            conn_txt = "ARM CONNECTED" if self.is_connected else "VISION-ONLY (ARM OFF)"
            cv2.putText(frame, f"Vision: {status} | {conn_txt} | Target: '{self.target_object}'", 
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                        (0, 255, 0) if self.homography_matrix is not None else (0, 0, 255), 2)

            cv2.imshow("Vision Agent View", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord('q')):
                break
            elif key == ord('c'):
                new_prompt = input("\nEnter new object prompt (e.g. 'bottle', 'pen'): ").strip()
                if new_prompt:
                    self.target_object = new_prompt
                    self.model.set_classes([self.target_object])
                    print(f"[AGENT] Tracking: '{self.target_object}'")
            elif key == ord('p'):
                if not coords:
                    print("[AGENT] Cannot pick: object not visible.")
                    continue
                
                obj_x, obj_y = coords
                print(f"\n[ACTION] Starting pick sequence on {self.target_object} at X={obj_x:.1f}, Y={obj_y:.1f}")
                await self.gripper_action("open")
                await self.arm_move(obj_x, obj_y, Z_HOVER)
                await self.arm_move(obj_x, obj_y, Z_GRASP)
                await self.gripper_action("close")
                await self.arm_move(obj_x, obj_y, Z_HOVER)
                await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], Z_DROPOFF)
                await self.gripper_action("open")
                await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], Z_HOVER)
                print("[ACTION] Pick sequence finished.")

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