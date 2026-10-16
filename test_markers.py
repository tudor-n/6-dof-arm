import cv2
import cv2.aruco as aruco

CAMERA_INDEX = 0

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

# 1. Force MJPEG and full 1080p uncropped frame
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# 2. Reset digital zoom to the widest angle
cap.set(cv2.CAP_PROP_ZOOM, 100)  # On some Windows drivers this is 0 or 100

if not cap.isOpened():
    print(f"Failed to open camera at index {CAMERA_INDEX}. Trying index 0...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit("Error: Could not open any video capture device.")

# Print actual negotiated camera parameters to your console
actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Camera active at: {int(actual_w)}x{int(actual_h)}")

# Dictionary and detector configuration (OpenCV 4.7+ API)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()
aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

detector = aruco.ArucoDetector(aruco_dict, aruco_params)

print("Checking markers... Press 'ESC' or 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame not received.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    found_ids = []
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        found_ids = sorted(ids.flatten().tolist())

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