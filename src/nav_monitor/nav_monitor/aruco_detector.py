#!/usr/bin/env python3
# Copyright 2026 taneesh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ArUco Marker Vision Detection & Tracking Node for Supermarket Autonomous Navigation."""

import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


# Contextual descriptions for all 12 ArUco navigation markers
MARKER_CONTEXT = {
    0: {
        "name": "Entrance Waypoint",
        "action": "South Entrance Verified -> Proceeding toward South-East Aisle Turn",
        "corridor": "South Entry Hallway",
    },
    1: {
        "name": "Aisle 4 Entrance",
        "action": "Approaching Lane 4-3 Entrance -> Aligning Westbound Heading",
        "corridor": "Aisle 4 East Transition",
    },
    2: {
        "name": "Lane 4-3 Corridor Guide",
        "action": "Lane 4-3 Centered -> Navigating Westbound along Shelving Row 4",
        "corridor": "Lane 4-3 (Westbound)",
    },
    3: {
        "name": "West Aisle 4 Exit",
        "action": "Reached West Lane Exit -> Executing Right Turn into West Corridor",
        "corridor": "West Perimeter Corridor (South)",
    },
    4: {
        "name": "Aisle 3 West Entrance",
        "action": "Approaching Lane 3-2 Entrance -> Aligning Eastbound Heading",
        "corridor": "Aisle 3 West Transition",
    },
    5: {
        "name": "Lane 3-2 Corridor Guide",
        "action": "Lane 3-2 Centered -> Navigating Eastbound along Shelving Row 3",
        "corridor": "Lane 3-2 (Eastbound)",
    },
    6: {
        "name": "East Aisle 3 Exit",
        "action": "Reached East Lane Exit -> Executing Left Turn into East Corridor",
        "corridor": "East Perimeter Corridor (Mid)",
    },
    7: {
        "name": "Aisle 2 East Entrance",
        "action": "Approaching Lane 2-1 Entrance -> Aligning Westbound Heading",
        "corridor": "Aisle 2 East Transition",
    },
    8: {
        "name": "Lane 2-1 Corridor Guide",
        "action": "Lane 2-1 Centered -> Navigating Westbound along Shelving Row 2",
        "corridor": "Lane 2-1 (Westbound)",
    },
    9: {
        "name": "West Aisle 2 Exit",
        "action": "Reached West Lane Exit -> Executing Right Turn toward North Perimeter",
        "corridor": "West Perimeter Corridor (North)",
    },
    10: {
        "name": "North Corridor Entrance",
        "action": "Approaching North-West Corner -> Turning East into Final Corridor",
        "corridor": "North Perimeter Corridor",
    },
    11: {
        "name": "Final Destination Goal",
        "action": "Goal Marker In Sight -> Decelerating and Aligning with Top-Right Goal Circle",
        "corridor": "Goal Docking Station (1.8, 3.25)",
    },
}


class ArucoDetectorNode(Node):
    """ROS 2 node for ArUco marker vision detection and simulated tracking."""

    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.declare_parameter('marker_length', 0.22)  # 22cm in meters
        self.declare_parameter('dictionary', 'DICT_5X5_50')
        self.declare_parameter('show_gui', True)

        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        dict_name = self.get_parameter('dictionary').get_parameter_value().string_value
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value

        # Check display environment
        if not os.environ.get('DISPLAY') and self.show_gui:
            self.show_gui = False

        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_log_time = {}

        # Build ArUco detector
        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_5X5_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            params.adaptiveThreshWinSizeMin = 3
            params.adaptiveThreshWinSizeMax = 43
            params.adaptiveThreshWinSizeStep = 4
            params.polygonalApproxAccuracyRate = 0.05
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
            self.legacy_params = None
        else:
            params = cv2.aruco.DetectorParameters_create()
            params.adaptiveThreshWinSizeMin = 3
            params.adaptiveThreshWinSizeMax = 43
            params.adaptiveThreshWinSizeStep = 4
            params.polygonalApproxAccuracyRate = 0.05
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = None
            self.legacy_params = (self.aruco_dict, params)

        # Publishers & Subscribers
        self.detection_pub = self.create_publisher(String, '/aruco/detections', 10)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.get_logger().info('=' * 65)
        self.get_logger().info('  ArUco Vision Navigation Detector Node Active')
        self.get_logger().info(f'  Tracking Dictionary: {dict_name} | Marker Size: {self.marker_length}m')
        self.get_logger().info('  Subscribing to: /camera/image_raw')
        self.get_logger().info('=' * 65)

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera calibration matrix from CameraInfo topic."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Received camera calibration parameters from /camera/camera_info')

    def approximate_camera_matrix(self, width, height, fov_deg=100.0):
        """Construct fallback camera matrix if CameraInfo is unavailable."""
        fx = fy = width / (2.0 * np.tan(np.radians(fov_deg / 2.0)))
        cx, cy = width / 2.0, height / 2.0
        mat = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        dist = np.zeros((5, 1), dtype=np.float64)
        return mat, dist

    def decode_image_msg(self, msg: Image):
        """Pure NumPy conversion of sensor_msgs/Image to BGR OpenCV image."""
        try:
            encoding = msg.encoding.lower() if msg.encoding else 'rgb8'
            if encoding in ('rgb8', 'rgb'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif encoding in ('bgr8', 'bgr'):
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            elif encoding in ('mono8', '8uc1'):
                gray = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            elif encoding in ('rgba8', 'bgra8'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
                if encoding.startswith('rgb'):
                    return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            else:
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1)).copy()
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return None

    def image_callback(self, msg: Image):
        """Process incoming camera frames, detect ArUco markers, and print tracking logs."""
        frame = self.decode_image_msg(msg)
        if frame is None:
            return

        h, w = frame.shape[:2]
        if self.camera_matrix is None:
            self.camera_matrix, self.dist_coeffs = self.approximate_camera_matrix(w, h)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            aruco_dict, params = self.legacy_params
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)

        now = time.time()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            detections_payload = []

            for i, marker_id in enumerate(ids.flatten()):
                # Estimate 3D pose
                half = self.marker_length / 2.0
                obj_points = np.array([
                    [-half,  half, 0],
                    [ half,  half, 0],
                    [ half, -half, 0],
                    [-half, -half, 0]
                ], dtype=np.float64)
                img_pts = corners[i].reshape(4, 2).astype(np.float64)

                ok, rvec, tvec = cv2.solvePnP(
                    obj_points, img_pts, self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if ok:
                    dist = float(np.linalg.norm(tvec))
                    dx, dy, dz = tvec.flatten()

                    # Draw 3D axis
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.75)

                    # Info overlay
                    c_pts = corners[i].reshape(4, 2)
                    pt = tuple(c_pts[0].astype(int))
                    cv2.putText(frame, f"ID: {marker_id} ({dist:.2f}m)",
                                (pt[0], max(15, pt[1] - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    context = MARKER_CONTEXT.get(int(marker_id), {
                        "name": f"Marker {marker_id}",
                        "action": "Following marker guidance trajectory",
                        "corridor": "Supermarket Corridor"
                    })

                    detections_payload.append({
                        "id": int(marker_id),
                        "name": context["name"],
                        "distance": round(dist, 3),
                        "offset_x": round(float(dx), 3),
                        "action": context["action"]
                    })

                    # Rate-limited console logging (max once every 0.8s per marker)
                    if (marker_id not in self.last_log_time) or (now - self.last_log_time[marker_id] > 0.8):
                        self.last_log_time[marker_id] = now
                        print(
                            f"\033[1;32m[ARUCO VISION]\033[0m "
                            f"Detected Marker \033[1;33m#{marker_id}\033[0m: \033[1;36m{context['name']}\033[0m | "
                            f"Distance: \033[1;35m{dist:.2f}m\033[0m (Offset X: {dx:+.2f}m)\n"
                            f"  └── \033[1;37mAction Cue:\033[0m {context['action']}"
                        )

            # Publish JSON status string
            if detections_payload:
                msg_out = String()
                msg_out.data = json.dumps(detections_payload)
                self.detection_pub.publish(msg_out)

        # Show GUI if requested and display available
        if self.show_gui:
            cv2.imshow("Supermarket Bot - ArUco Camera View", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
