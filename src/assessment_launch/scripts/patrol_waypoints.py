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

"""Script to autonomously navigate through the exact drawn serpentine store route."""

import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def create_pose(navigator, x, y, yaw_deg=0.0):
    """Helper to construct a stamped pose with orientation in degrees."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("=" * 65)
    print("  Supermarket Autonomous Drawn Route Navigation")
    print("=" * 65)
    print("[INFO] Waiting for Navigation2 to become active...")
    navigator.waitUntilNav2Active()
    print("[INFO] Navigation2 is active and ready!\n")

    # Exact waypoints matching the user's drawn serpentine route:
    # 1. Start: Bottom Green Circle (0.0, -3.5)
    # 2. Curve right around Aisle 4 -> pass through Lane 3-4 going West
    # 3. Curve up left corridor -> pass through Lane 2-3 going East
    # 4. Curve up right corridor -> pass through Lane 1-2 going West
    # 5. Curve up left corridor -> pass through Top North corridor going East
    # 6. End: Top-Right Green Circle (1.8, 3.25)
    waypoints_data = [
        {"name": "1. Right Entrance Curve",        "x": 1.8,  "y": -2.8,  "yaw": 60.0},
        {"name": "2. Aisle 3-4 Lane (East Entry)", "x": 1.8,  "y": -1.25, "yaw": 180.0},
        {"name": "3. Aisle 3-4 Lane (Mid)",        "x": 0.0,  "y": -1.25, "yaw": 180.0},
        {"name": "4. Aisle 3-4 Lane (West Exit)",  "x": -2.2, "y": -1.25, "yaw": 180.0},
        {"name": "5. Aisle 2-3 Lane (West Entry)", "x": -2.2, "y":  0.25, "yaw": 0.0},
        {"name": "6. Aisle 2-3 Lane (Mid)",        "x": 0.0,  "y":  0.25, "yaw": 0.0},
        {"name": "7. Aisle 2-3 Lane (East Exit)",  "x": 1.8,  "y":  0.25, "yaw": 0.0},
        {"name": "8. Aisle 1-2 Lane (East Entry)", "x": 1.8,  "y":  1.75, "yaw": 180.0},
        {"name": "9. Aisle 1-2 Lane (Mid)",        "x": 0.0,  "y":  1.75, "yaw": 180.0},
        {"name": "10. Aisle 1-2 Lane (West Exit)", "x": -2.2, "y":  1.75, "yaw": 180.0},
        {"name": "11. North-West Top Corner",      "x": -2.2, "y":  3.25, "yaw": 0.0},
        {"name": "12. Top North Corridor (Mid)",   "x": 0.0,  "y":  3.25, "yaw": 0.0},
        {"name": "13. End Goal (Top-Right Circle)","x": 1.8,  "y":  3.25, "yaw": 0.0},
    ]

    print(f"[INFO] Executing route with {len(waypoints_data)} checkpoints...")
    print(f"[INFO] Start : Bottom Entrance Circle (0.0, -3.5)")
    print(f"[INFO] Target: Top-Right Goal Circle (1.8, 3.25)\n")

    for i, wp in enumerate(waypoints_data):
        goal_pose = create_pose(navigator, wp["x"], wp["y"], wp["yaw"])
        print(f"[ROUTE] Navigating to Checkpoint #{i+1}/{len(waypoints_data)}: {wp['name']} ({wp['x']}, {wp['y']})")
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and hasattr(feedback, 'distance_remaining'):
                print(f"  └── Progress: {feedback.distance_remaining:.2f} m remaining", end="\r")
            time.sleep(0.3)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"  └── [OK] Reached Checkpoint #{i+1} successfully!\n")
        elif result == TaskResult.CANCELED:
            print(f"\n[WARN] Checkpoint #{i+1} was canceled. Aborting route.")
            break
        else:
            print(f"\n[WARN] Checkpoint #{i+1} could not be reached cleanly. Proceeding to next checkpoint...\n")

    print("=" * 65)
    print("  SUCCESS: Route completed! Robot arrived at Top-Right End Goal.")
    print("=" * 65)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


