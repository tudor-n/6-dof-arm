import asyncio
import json
import cv2
import cv2.aruco as aruco
import numpy as np
import websockets
from ultralytics import YOLOWorld

# --- SYSTEM CONFIGURATION ---
WS_URI = "ws://localhost:8080/ws"
CAMERA_INDEX = 0

# Define physical locations of your 4 ArUco markers relative to the arm base (X, Y in mm)
# Ensure these match where you tape them on the table.
MARKER_COORDS = {
    0: [150.0, 150.0],   # Marker ID 0: Front Right
    1: [150.0, -150.0],  # Marker ID 1: Front Left
    2: [300.0, 150.0],   # Marker ID 2: Far Right
    3: [300.0, -150.0]   # Marker ID 3: Far Left
}

# Hover and Grasp Heights (Z in mm) based on config.json L0=83.0[cite: 1]
Z_HOVER = 160.0
Z_GRASP = 25.0
Z_DROPOFF = 180.0
DROPOFF_XY = (100.0, 200.0)

class VisionAgent:
    def __init__(self):
        self.ws = None
        # Initialize YOLO-World for text-prompt object detection
        self.model = YOLOWorld('yolov8s-world.pt')
        self.target_object = "screwdriver"
        self.model.set_classes([self.target_object])
        
        # ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.homography_matrix = None

    async def connect(self):
        self.ws = await websockets.connect(WS_URI)
        print("Connected to Arm IK Server.")

    async def send_cmd(self, cmd):
        await self.ws.send(json.dumps(cmd))
        await asyncio.sleep(0.05)

    async def arm_move(self, x, y, z):
        print(f"Moving to X:{x:.1f}, Y:{y:.1f}, Z:{z:.1f}")
        # Send tip coordinates; let server.py handle IK
        await self.send_cmd({"type": "tip", "x": x, "y": y, "z": z, "lock": False})
        await asyncio.sleep(2.0) # Wait for physical movement

    async def gripper_action(self, action):
        """Controls the 360 degree dead-man switch gripper."""
        if action == "open":
            print("Opening gripper...")
            await self.send_cmd({"type": "grip", "angle": 130}) # > 95 opens
            await asyncio.sleep(1.5)
        elif action == "close":
            print("Closing gripper...")
            await self.send_cmd({"type": "grip", "angle": 50})  # < 85 closes[cite: 2]
            await asyncio.sleep(1.5)
        
        # Brake the gripper at 90 degrees[cite: 2]
        print("Braking gripper.")
        await self.send_cmd({"type": "grip", "angle": 90})

    def update_homography(self, frame):
        """Finds ArUco markers and updates the pixel-to-mm transformation matrix."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None and len(ids) >= 4:
            img_pts = []
            phys_pts = []
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in MARKER_COORDS:
                    # Get center of the marker in pixels
                    c = corners[i][0]
                    center_x = (c[0][0] + c[2][0]) / 2
                    center_y = (c[0][1] + c[2][1]) / 2
                    img_pts.append([center_x, center_y])
                    phys_pts.append(MARKER_COORDS[marker_id])
            
            if len(img_pts) >= 4:
                self.homography_matrix, _ = cv2.findHomography(np.array(img_pts), np.array(phys_pts))
                return True
        return False

    def pixel_to_mm(self, u, v):
        """Converts image pixels to physical arm coordinates using the homography matrix."""
        if self.homography_matrix is None:
            return None
        pt = np.array([[[u, v]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography_matrix)
        return transformed[0][0][0], transformed[0][0][1]

    def find_object(self, frame):
        """Runs YOLO-World to find the target object and returns physical XY coordinates."""
        results = self.model.predict(frame, conf=0.25, verbose=False)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u_center = (x1 + x2) / 2
                v_center = (y1 + y2) / 2
                # Target the bottom of the bounding box for tabletop grasping
                v_bottom = y2 
                
                coords = self.pixel_to_mm(u_center, v_bottom)
                if coords:
                    return coords
        return None

    async def execute_pick_loop(self):
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        while True:
            ret, frame = cap.read()
            if not ret: continue

            # 1. Calibrate camera position continuously
            self.update_homography(frame)

            # Display logic for debugging
            cv2.imshow("Vision Agent View", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('p'): # Press 'p' to prompt the pick action
                self.target_object = input("Enter prompt (e.g. 'screwdriver'): ")
                self.model.set_classes([self.target_object])
                
                success = False
                retries = 0
                
                while not success and retries < 3:
                    print(f"\n--- Attempt {retries + 1} ---")
                    ret, frame = cap.read()
                    self.update_homography(frame)
                    
                    target_coords = self.find_object(frame)
                    if not target_coords:
                        print("Object not found in workspace. Retrying...")
                        retries += 1
                        await asyncio.sleep(1)
                        continue

                    obj_x, obj_y = target_coords
                    print(f"Target locked at X:{obj_x:.1f}, Y:{obj_y:.1f}")

                    # 2. Open Gripper
                    await self.gripper_action("open")
                    
                    # 3. Hover over target
                    await self.arm_move(obj_x, obj_y, Z_HOVER)
                    
                    # 4. Descend to grab
                    await self.arm_move(obj_x, obj_y, Z_GRASP)
                    
                    # 5. Close Gripper
                    await self.gripper_action("close")
                    
                    # 6. Ascend
                    await self.arm_move(obj_x, obj_y, Z_HOVER)
                    await asyncio.sleep(1.0)
                    
                    # 7. Verification Loop
                    # Flush old frames, capture fresh frame
                    for _ in range(5): cap.read() 
                    ret, verify_frame = cap.read()
                    verify_coords = self.find_object(verify_frame)
                    
                    if verify_coords:
                        # If object is still on the table near the original spot, grasp failed
                        vx, vy = verify_coords
                        dist = np.hypot(vx - obj_x, vy - obj_y)
                        if dist < 50.0:
                            print("Grasp failed (object still on table). Restarting loop.")
                            retries += 1
                            continue
                    
                    # 8. Success: Move to Dropoff
                    print("Grasp successful! Moving to dropoff.")
                    await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], Z_DROPOFF)
                    await self.gripper_action("open")
                    
                    # Return arm to safe hover
                    await self.arm_move(DROPOFF_XY[0], DROPOFF_XY[1], Z_HOVER)
                    success = True

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    agent = VisionAgent()
    asyncio.run(agent.connect())
    asyncio.run(agent.execute_pick_loop())