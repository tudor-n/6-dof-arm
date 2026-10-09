import cv2
import cv2.aruco as aruco

# Use DirectShow backend on Windows to target the external USB camera properly
# 1 is typically the external USB camera; change to 2 if you have multiple peripherals
CAMERA_INDEX = 1

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

# Configure stream based on CMSXJ22A specs (MJPEG 1080p)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

if not cap.isOpened():
    print(f"Failed to open camera at index {CAMERA_INDEX}. Trying index 0...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit("Error: Could not open any video capture device.")

# Dictionary and detector configuration (OpenCV 4.7+ API)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()
aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

# Instantiate the ArucoDetector object
detector = aruco.ArucoDetector(aruco_dict, aruco_params)

print("Checking markers... Press 'ESC' or 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame not received.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Modern detection call
    corners, ids, rejected = detector.detectMarkers(gray)

    found_ids = []
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        found_ids = sorted(ids.flatten().tolist())

    # Check if all four calibration markers (0, 1, 2, 3) are visible
    all_ready = all(i in found_ids for i in [0, 1, 2, 3])
    status = f"Detected IDs: {found_ids} {'[READY]' if all_ready else '[WAITING FOR 0,1,2,3]'}"
    color = (0, 255, 0) if all_ready else (0, 0, 255)

    cv2.putText(frame, status, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    cv2.imshow("ArUco Verification - CMSXJ22A", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()