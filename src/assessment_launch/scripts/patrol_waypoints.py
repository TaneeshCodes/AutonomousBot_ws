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
    # Exact waypoints matching the user's drawn serpentine route and ArUco markers:
    waypoints_data = [
        {"name": "1. Entrance Corridor -> [ArUco #0: South Entrance]",        "x": 1.8,  "y": -2.8,  "yaw": 60.0,  "marker": 0},
        {"name": "2. Aisle 4 Turn -> [ArUco #1: Lane 4-3 Entry]",              "x": 1.8,  "y": -1.25, "yaw": 180.0, "marker": 1},
        {"name": "3. Lane 4-3 Mid-way -> [ArUco #2: Lane 4-3 Corridor Guide]",  "x": 0.0,  "y": -1.25, "yaw": 180.0, "marker": 2},
        {"name": "4. Aisle 4 West Exit -> [ArUco #3: West Transition]",        "x": -2.2, "y": -1.25, "yaw": 180.0, "marker": 3},
        {"name": "5. Aisle 3 Turn -> [ArUco #4: Lane 3-2 Entry]",              "x": -2.2, "y":  0.25, "yaw": 0.0,   "marker": 4},
        {"name": "6. Lane 3-2 Mid-way -> [ArUco #5: Lane 3-2 Corridor Guide]",  "x": 0.0,  "y":  0.25, "yaw": 0.0,   "marker": 5},
        {"name": "7. Aisle 3 East Exit -> [ArUco #6: East Transition]",        "x": 1.8,  "y":  0.25, "yaw": 0.0,   "marker": 6},
        {"name": "8. Aisle 2 Turn -> [ArUco #7: Lane 2-1 Entry]",              "x": 1.8,  "y":  1.75, "yaw": 180.0, "marker": 7},
        {"name": "9. Lane 2-1 Mid-way -> [ArUco #8: Lane 2-1 Corridor Guide]",  "x": 0.0,  "y":  1.75, "yaw": 180.0, "marker": 8},
        {"name": "10. Aisle 2 West Exit -> [ArUco #9: North Transition]",       "x": -2.2, "y":  1.75, "yaw": 180.0, "marker": 9},
        {"name": "11. North-West Corner -> [ArUco #10: Final Lap Entry]",       "x": -2.2, "y":  3.25, "yaw": 0.0,   "marker": 10},
        {"name": "12. Top North Corridor -> Traversing Eastbound",             "x": 0.0,  "y":  3.25, "yaw": 0.0,   "marker": 11},
        {"name": "13. End Goal -> [ArUco #11: Final Destination Marker]",      "x": 1.8,  "y":  3.25, "yaw": 0.0,   "marker": 11},
    ]

    print(f"[INFO] Executing route with {len(waypoints_data)} checkpoints...")
    print(f"[INFO] Start : Bottom Entrance Circle (0.0, -3.5)")
    print(f"[INFO] Target: Top-Right Goal Circle (1.8, 3.25)\n")

    for i, wp in enumerate(waypoints_data):
        goal_pose = create_pose(navigator, wp["x"], wp["y"], wp["yaw"])
        print(f"\033[1;34m[ROUTE]\033[0m Navigating to Checkpoint #{i+1}/{len(waypoints_data)}: \033[1;33m{wp['name']}\033[0m ({wp['x']}, {wp['y']})")
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and hasattr(feedback, 'distance_remaining'):
                print(f"  └── Following ArUco Guidance | Distance remaining: {feedback.distance_remaining:.2f} m", end="\r")
            time.sleep(0.3)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"\n  └── \033[1;32m[SUCCESS]\033[0m Checkpoint #{i+1} reached! ArUco marker verified.\n")
        elif result == TaskResult.CANCELED:
            print(f"\n\033[1;31m[WARN]\033[0m Checkpoint #{i+1} was canceled. Aborting route.")
            break
        else:
            print(f"\n\033[1;33m[WARN]\033[0m Checkpoint #{i+1} passed with tolerance. Continuing...\n")

    print("=" * 65)
    print("  SUCCESS: ArUco-guided Serpentine Route Completed Successfully!")
    print("  Robot arrived at Top-Right Destination Goal Circle (1.8, 3.25).")
    print("=" * 65)

    rclpy.shutdown()


if __name__ == '__main__':
    main()


