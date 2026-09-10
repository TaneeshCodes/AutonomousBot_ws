"""
ArUco Marker Detection + Pose Estimation + Distance Measurement
-----------------------------------------------------------------
Detects ArUco markers in a live camera feed, estimates each marker's
3D pose (rotation + translation) relative to the camera, and prints/
displays the straight-line distance between the camera and each marker.

REQUIREMENTS
    pip install opencv-contrib-python numpy

USAGE
    python aruco_pose_distance.py
    python aruco_pose_distance.py --marker_length 0.10 --dict DICT_4X4_50
    python aruco_pose_distance.py --calib camera_calibration.npz --camera 0

NOTES ON ACCURACY
    Distance/pose accuracy depends heavily on having a proper camera
    calibration (camera matrix + distortion coefficients). If you don't
    pass --calib, this script falls back to an APPROXIMATE camera matrix
    guessed from the frame resolution and a typical 60-degree horizontal
    FOV webcam. That will get you a roughly-correct distance, but for
    real robotics/measurement work, calibrate your camera first
    (cv2.calibrateCamera with a checkerboard) and save the result as:
        np.savez("camera_calibration.npz", camera_matrix=K, dist_coeffs=D)
"""

import argparse
import sys

import cv2
import numpy as np

# ---------------------------------------------------------------------
# Supported ArUco dictionaries (add more if you need them)
# ---------------------------------------------------------------------
ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def parse_args():
    parser = argparse.ArgumentParser(description="ArUco detection + pose + distance")
    parser.add_argument("--camera", type=int, default=0,
                         help="Camera index (default: 0)")
    parser.add_argument("--dict", type=str, default="DICT_5X5_50",
                         choices=list(ARUCO_DICTS.keys()),
                         help="ArUco dictionary to use (default: DICT_5X5_50)")
    parser.add_argument("--marker_length", type=float, default=0.05,
                         help="Real-world side length of the marker in METERS "
                              "(default: 0.05 = 5cm). Set this to your actual "
                              "printed marker size for correct distances.")
    parser.add_argument("--calib", type=str, default=None,
                         help="Path to a .npz file with 'camera_matrix' and "
                              "'dist_coeffs' arrays. If omitted, an approximate "
                              "camera matrix is estimated from frame size.")
    return parser.parse_args()


def load_calibration(path):
    """Load camera_matrix and dist_coeffs from a .npz file."""
    data = np.load(path)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    return camera_matrix, dist_coeffs


def approximate_camera_matrix(frame_width, frame_height, fov_deg=60.0):
    """
    Rough camera matrix guess when no calibration file is provided.
    Assumes a pinhole model and the given horizontal FOV.
    """
    fx = fy = frame_width / (2 * np.tan(np.radians(fov_deg / 2)))
    cx, cy = frame_width / 2.0, frame_height / 2.0
    camera_matrix = np.array([[fx, 0, cx],
                               [0, fy, cy],
                               [0,  0,  1]], dtype=np.float64)
    dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    return camera_matrix, dist_coeffs


def build_detector_params(params):
    """
    Tune detector parameters to be more forgiving of angle, glare, motion
    blur, and partial occlusion -- the usual causes of intermittent
    detection dropouts.
    """
    # Wider adaptive-threshold window range catches markers under uneven
    # lighting/glare instead of only one fixed threshold size.
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 43
    params.adaptiveThreshWinSizeStep = 4
    # Be a bit more lenient on how "square" a detected quad must be,
    # helps with markers seen at an angle.
    params.polygonalApproxAccuracyRate = 0.05
    # Sub-pixel corner refinement improves pose accuracy and makes
    # detection more stable frame-to-frame.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30
    return params


def get_detector(dict_name):
    """Build an ArUco detector, compatible with old and new OpenCV APIs."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
    if hasattr(cv2.aruco, "ArucoDetector"):
        # OpenCV >= 4.7 API
        params = build_detector_params(cv2.aruco.DetectorParameters())
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector, None
    else:
        # OpenCV < 4.7 legacy API
        params = build_detector_params(cv2.aruco.DetectorParameters_create())
        return None, (aruco_dict, params)


def detect_markers(frame, detector, legacy_params):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if detector is not None:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        aruco_dict, params = legacy_params
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    return corners, ids


def estimate_pose(corners, marker_length, camera_matrix, dist_coeffs):
    """
    Estimate rvec/tvec for one marker using solvePnP.
    (Replacement for the deprecated cv2.aruco.estimatePoseSingleMarkers.)
    """
    half = marker_length / 2.0
    obj_points = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float64)

    img_points = corners.reshape(4, 2).astype(np.float64)

    ok, rvec, tvec = cv2.solvePnP(
        obj_points, img_points, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    return rvec, tvec if ok else (None, None)


def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera index {args.camera}")
        sys.exit(1)

    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Could not read from camera")
        sys.exit(1)
    frame_h, frame_w = frame.shape[:2]

    if args.calib:
        camera_matrix, dist_coeffs = load_calibration(args.calib)
        print(f"[INFO] Loaded camera calibration from {args.calib}")
    else:
        camera_matrix, dist_coeffs = approximate_camera_matrix(frame_w, frame_h)
        print("[WARN] No --calib file given. Using an APPROXIMATE camera matrix. "
              "Distances will be roughly correct, not precise.")

    detector, legacy_params = get_detector(args.dict)

    print(f"[INFO] Frame size: {frame_w}x{frame_h}")
    print(f"[INFO] Dictionary: {args.dict} | Marker length: {args.marker_length} m")
    print("[INFO] If detection drops out often: check the printed marker actually "
          "belongs to this dictionary, avoid glare/motion blur, and keep the full "
          "marker (with its white border) inside the frame and unoccluded.")
    print("[INFO] Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        corners, ids = detect_markers(frame, detector, legacy_params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = estimate_pose(
                    corners[i], args.marker_length, camera_matrix, dist_coeffs
                )
                if rvec is None:
                    continue

                # Straight-line distance from camera to marker center (meters)
                distance = float(np.linalg.norm(tvec))

                # Draw the 3D axis on the marker
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec,
                                   args.marker_length * 0.75)

                # Overlay text near the marker
                corner_pts = corners[i].reshape(4, 2)
                text_org = tuple(corner_pts[0].astype(int))
                cv2.putText(frame, f"ID {marker_id}: {distance:.3f} m",
                            (text_org[0], text_org[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                print(f"Marker {marker_id} | distance = {distance:.3f} m "
                      f"| tvec = {tvec.flatten()} | rvec = {rvec.flatten()}")
        else:
            # Visual confirmation that the loop is running fine but nothing
            # is currently detected -- so a dropout is obvious, not silent.
            cv2.putText(frame, "No marker detected", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("ArUco Pose + Distance", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()