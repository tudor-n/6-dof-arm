import cv2
import cv2.aruco as aruco

# Replace 0 with your USB camera index if you have multiple cameras
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()

print("Checking markers... Press 'ESC' or 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Camera frame not received.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    found_ids = []
    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)
        found_ids = sorted(ids.flatten().tolist())

    # On-screen display
    status = f"Detected IDs: {found_ids}"
    color = (0, 255, 0) if all(i in found_ids for i in [0, 1, 2, 3]) else (0, 0, 255)
    cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    cv2.imshow("ArUco Verification", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()