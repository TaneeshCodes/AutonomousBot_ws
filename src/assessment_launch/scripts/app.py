"""
Flask web app: streams the Pi camera feed with ArUco detection overlay
(bounding box, axis, distance) to your browser over HTTP.

Run this ON the Raspberry Pi, then either:
  A) Browse to http://<pi-ip>:5000 from any device on the same network, or
  B) SSH in with port forwarding and open it as localhost on your laptop:
         ssh -L 5000:localhost:5000 pi@<pi-ip>
     then open http://localhost:5000 in your own browser.

REQUIREMENTS (on the Pi)
    pip install flask opencv-contrib-python numpy

USAGE
    python3 app.py
    python3 app.py --marker_length 0.10 --dict DICT_5X5_50 --camera 0
    python3 app.py --calib camera_calibration.npz --port 5000
"""

import argparse
import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template_string

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

app = Flask(__name__)

# ---------------------------------------------------------------------
# Shared state between the capture thread and the Flask routes
# ---------------------------------------------------------------------
state_lock = threading.Lock()
latest_jpeg = None
latest_detections = []  # list of {"id", "distance_m", "tvec", "rvec"}


def parse_args():
    parser = argparse.ArgumentParser(description="ArUco detection Flask stream")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--dict", type=str, default="DICT_5X5_50",
                         choices=list(ARUCO_DICTS.keys()))
    parser.add_argument("--marker_length", type=float, default=0.05,
                         help="Real marker side length in meters (default: 0.05)")
    parser.add_argument("--calib", type=str, default=None,
                         help="Path to .npz with 'camera_matrix' and 'dist_coeffs'")
    parser.add_argument("--host", type=str, default="0.0.0.0",
                         help="Bind address (default: 0.0.0.0, reachable from LAN)")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    return parser.parse_args()


def load_calibration(path):
    data = np.load(path)
    return data["camera_matrix"], data["dist_coeffs"]


def approximate_camera_matrix(frame_width, frame_height, fov_deg=60.0):
    fx = fy = frame_width / (2 * np.tan(np.radians(fov_deg / 2)))
    cx, cy = frame_width / 2.0, frame_height / 2.0
    camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    return camera_matrix, dist_coeffs


def build_detector_params(params):
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 43
    params.adaptiveThreshWinSizeStep = 4
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30
    return params


def get_detector(dict_name):
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = build_detector_params(cv2.aruco.DetectorParameters())
        return cv2.aruco.ArucoDetector(aruco_dict, params), None
    else:
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
    half = marker_length / 2.0
    obj_points = np.array([
        [-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0],
    ], dtype=np.float64)
    img_points = corners.reshape(4, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        obj_points, img_points, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    return (rvec, tvec) if ok else (None, None)


def capture_loop(args):
    global latest_jpeg, latest_detections

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera index {args.camera}")
        return

    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Could not read from camera")
        return
    frame_h, frame_w = frame.shape[:2]

    if args.calib:
        camera_matrix, dist_coeffs = load_calibration(args.calib)
        print(f"[INFO] Loaded calibration from {args.calib}")
    else:
        camera_matrix, dist_coeffs = approximate_camera_matrix(frame_w, frame_h)
        print("[WARN] No --calib given, using an approximate camera matrix.")

    detector, legacy_params = get_detector(args.dict)
    print(f"[INFO] Dictionary: {args.dict} | Marker length: {args.marker_length} m")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        corners, ids = detect_markers(frame, detector, legacy_params)
        detections = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec = estimate_pose(corners[i], args.marker_length,
                                            camera_matrix, dist_coeffs)
                if rvec is None:
                    continue
                distance = float(np.linalg.norm(tvec))
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec,
                                   args.marker_length * 0.75)
                corner_pts = corners[i].reshape(4, 2)
                text_org = tuple(corner_pts[0].astype(int))
                cv2.putText(frame, f"ID {marker_id}: {distance:.3f} m",
                            (text_org[0], text_org[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                detections.append({
                    "id": int(marker_id),
                    "distance_m": round(distance, 3),
                    "tvec": [round(float(v), 4) for v in tvec.flatten()],
                    "rvec": [round(float(v), 4) for v in rvec.flatten()],
                })
        else:
            cv2.putText(frame, "No marker detected", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue

        with state_lock:
            latest_jpeg = buf.tobytes()
            latest_detections = detections


def mjpeg_generator():
    while True:
        with state_lock:
            frame = latest_jpeg
        if frame is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.03)  # ~30 fps cap on streaming


INDEX_HTML = """
<!doctype html>
<html>
<head>
  <title>ArUco Pose + Distance</title>
  <style>
    body { font-family: sans-serif; background: #111; color: #eee; text-align: center; }
    img { max-width: 90vw; border: 2px solid #444; margin-top: 1rem; }
    table { margin: 1.5rem auto; border-collapse: collapse; }
    td, th { border: 1px solid #444; padding: 6px 14px; }
    th { background: #222; }
  </style>
</head>
<body>
  <h2>ArUco Pose + Distance (live)</h2>
  <img src="/video_feed">
  <table id="detTable">
    <thead><tr><th>ID</th><th>Distance (m)</th><th>tvec</th><th>rvec</th></tr></thead>
    <tbody><tr><td colspan="4">No marker detected</td></tr></tbody>
  </table>

  <script>
    async function poll() {
      try {
        const res = await fetch('/status');
        const data = await res.json();
        const tbody = document.querySelector('#detTable tbody');
        if (data.length === 0) {
          tbody.innerHTML = '<tr><td colspan="4">No marker detected</td></tr>';
        } else {
          tbody.innerHTML = data.map(d =>
            `<tr><td>${d.id}</td><td>${d.distance_m}</td><td>${d.tvec}</td><td>${d.rvec}</td></tr>`
          ).join('');
        }
      } catch (e) { /* ignore transient errors */ }
      setTimeout(poll, 500);
    }
    poll();
  </script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(INDEX_HTML)


@app.route("/video_feed")
def video_feed():
    return Response(mjpeg_generator(),
                     mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/status")
def status():
    with state_lock:
        return jsonify(latest_detections)


if __name__ == "__main__":
    args = parse_args()
    t = threading.Thread(target=capture_loop, args=(args,), daemon=True)
    t.start()
    print(f"[INFO] Starting Flask server on http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, threaded=True)
