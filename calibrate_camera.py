import cv2
import numpy as np
import os

# --- CALIBRATION SETTINGS ---
# Based on your 11x8 squares (which means 10x7 internal vertices)
CHECKERBOARD = (10, 7)
SQUARE_SIZE_MM = 10.0  # Updated to your exact physical measurement

# Termination criteria for sub-pixel accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare 3D object points based on real-world millimeter measurements
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE_MM

# Arrays to store object points and image points from all the images
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print("\n--- CAMERA CALIBRATION ---")
print("1. Hold the checkerboard in front of the camera.")
print("2. Ensure the whole board is visible and well-lit.")
print("3. Press 'c' to capture a frame (Aim for 15-20 frames).")
print("4. Tilt and rotate the board slightly between captures.")
print("5. Press 'q' to finish and calculate the matrix.\n")

captured_frames = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    display_frame = frame.copy()
    
    # Find the chess board corners
    ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    # If found, draw them
    if ret_corners:
        cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_corners)
        status = "GRID DETECTED - Press 'c' to capture"
        color = (0, 255, 0)
    else:
        status = "Grid not detected - Adjust angle/lighting"
        color = (0, 0, 255)
        
    cv2.putText(display_frame, status, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(display_frame, f"Captured: {captured_frames}/15+", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    cv2.imshow('Calibration', display_frame)
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('c') and ret_corners:
        # Refine corner locations to sub-pixel accuracy before saving
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        captured_frames += 1
        print(f"Captured frame {captured_frames}")
        
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if captured_frames > 0:
    print("\nCalculating camera matrix... This may take a few seconds.")
    # Calculate the camera matrix and distortion coefficients
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    # Save the results
    np.save("camera_matrix.npy", mtx)
    np.save("dist_coeffs.npy", dist)
    
    print("\n--- CALIBRATION SUCCESSFUL ---")
    print("Saved 'camera_matrix.npy' and 'dist_coeffs.npy' to the current directory.")
    print("\nCamera Matrix:\n", mtx)
    print("\nDistortion Coefficients:\n", dist)
else:
    print("\nCalibration aborted. No frames were captured.")