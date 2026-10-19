#!/usr/bin/env python3
"""Camera intrinsics calibration -> camera_matrix.npy, dist_coeffs.npy, camera_calib.json

    python calibrate_camera.py                        default board, camera 0
    python calibrate_camera.py --camera 1 --square 25 --board 9x6

Hold the board so it fills a good part of the frame, at roughly the distance the camera
will actually work at, and vary the angle between captures. Press 'c' to capture, 'u' to
undo the last one, 'q' to compute and save.

Target size matters more than the number of captures. A board only 100 x 70 mm seen from
a metre away constrains focal length and distortion very poorly, and a wrong focal length
cannot be absorbed by the pose - it shows up as reprojection error in vision.py and as
position-dependent error in the table coordinates. Aim for a board 200-300 mm across.
"""
import argparse
import json

import cv2
import numpy as np

CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
GOOD_RMS_PX = 0.5        # at or below this the intrinsics are fine for millimetre work
USABLE_RMS_PX = 1.0      # above this, vision.py will warn on startup


def parse_args():
    a = argparse.ArgumentParser()
    a.add_argument("--camera", type=int, default=0, help="camera index (match server.py --camera)")
    a.add_argument("--board", default="10x7", help="INNER corner count, e.g. 10x7 for an 11x8 square board")
    a.add_argument("--square", type=float, default=10.0, help="square edge length in mm")
    a.add_argument("--width", type=int, default=1920)
    a.add_argument("--height", type=int, default=1080)
    a.add_argument("--min", type=int, default=15, help="minimum captures before computing")
    return a.parse_args()


def open_camera(a):
    cap = cv2.VideoCapture(a.camera, cv2.CAP_DSHOW)
    # MJPG first: without it the camera may hand back a different (often cropped) mode than
    # the one requested, and the intrinsics would then belong to a frame size nothing uses.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    return cap


def main():
    a = parse_args()
    cols, rows = (int(v) for v in a.board.lower().split("x"))
    board = (cols, rows)

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= a.square
    span = max(cols, rows) * a.square
    if span < 150:
        print(f"[WARN] This board spans only {span:.0f} mm. That is small for a camera working at "
              f"~1 m; focal length and distortion will be poorly constrained. Prefer 200-300 mm.")

    cap = open_camera(a)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {a.camera}")

    objpoints, imgpoints = [], []
    size = None
    print(f"\n--- CALIBRATION --- board {cols}x{rows} inner corners, {a.square:.1f} mm squares")
    print("c = capture, u = undo last, q = compute and save\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] camera read failed")
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if size is None:
            size = gray.shape[::-1]
            print(f"[INFO] streaming at {size[0]}x{size[1]}")
        elif gray.shape[::-1] != size:
            raise SystemExit(f"resolution changed mid-run ({size} -> {gray.shape[::-1]}); restart")

        found, corners = cv2.findChessboardCorners(
            gray, board, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        view = frame.copy()
        if found:
            cv2.drawChessboardCorners(view, board, corners, found)
        cv2.putText(view, "GRID OK - press c" if found else "no grid - adjust angle/lighting",
                    (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.putText(view, f"captured {len(objpoints)}/{a.min}+   {size[0]}x{size[1]}",
                    (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.imshow("Calibration", view)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("c") and found:
            objpoints.append(objp)
            imgpoints.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA))
            print(f"captured {len(objpoints)}")
        elif k == ord("u") and objpoints:
            objpoints.pop()
            imgpoints.pop()
            print(f"undone, {len(objpoints)} left")
        elif k == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 4:
        raise SystemExit("not enough captures - nothing saved")
    if len(objpoints) < a.min:
        print(f"[WARN] only {len(objpoints)} captures; {a.min}+ gives a better fit")

    print("\ncomputing...")
    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)

    # Per-view error: one bad capture can drag the whole fit, and the mean alone hides it.
    per_view = []
    for i in range(len(objpoints)):
        pr, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        per_view.append(float(np.mean(np.linalg.norm(pr.reshape(-1, 2) - imgpoints[i].reshape(-1, 2), axis=1))))

    np.save("camera_matrix.npy", mtx)
    np.save("dist_coeffs.npy", dist)
    with open("camera_calib.json", "w") as f:
        json.dump({"resolution": [int(size[0]), int(size[1])], "rms": float(rms),
                   "views": len(objpoints), "board": [cols, rows], "square_mm": a.square,
                   "per_view_px": [round(v, 3) for v in per_view]}, f, indent=2)

    print(f"\nRMS reprojection error: {rms:.3f} px   ({len(objpoints)} views at {size[0]}x{size[1]})")
    print(f"worst view: {max(per_view):.3f} px")
    if rms <= GOOD_RMS_PX:
        print("Good - these intrinsics are fine for millimetre work.")
    elif rms <= USABLE_RMS_PX:
        print("Usable but not great. A larger board or more varied angles would help.")
    else:
        print("TOO HIGH. Do not trust table coordinates computed from these. Recalibrate with a\n"
              "larger board, more angles, and no motion blur; drop any view above ~1.5 px with 'u'.")
    print("\nSaved camera_matrix.npy, dist_coeffs.npy and camera_calib.json.")
    print("camera_calib.json records the resolution, which vision.py checks the live stream against.")
    print("\nCamera matrix:\n", mtx, "\n\nDistortion:\n", dist.ravel())


if __name__ == "__main__":
    main()
