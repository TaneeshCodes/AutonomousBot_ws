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

"""Script to autonomously navigate through store waypoints and return to base."""

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

    print("=" * 60)
    print("  Supermarket Autonomous Multi-Waypoint Patrol Script")
    print("=" * 60)
    print("[INFO] Waiting for Navigation2 active state...")
    navigator.waitUntilNav2Active()
    print("[INFO] Navigation2 is fully active!")

    # Define Waypoints for Store Patrol + Return to Base
    # Layout:
    # Base / Start circle: (0.0, -3.5)
    # Aisle 4: y=-2.0, Aisle 3: y=-0.5, Aisle 2: y=1.0, Aisle 1: y=2.5
    # North aisle corridor: y=3.2
    # Checkout counters: (-1.6, -3.2)
    waypoints_data = [
        {"name": "Entrance Right Walkway",  "x": 1.6,  "y": -2.0, "yaw": 90.0},
        {"name": "Aisle 4 Walkway",        "x": -1.8, "y": -2.0, "yaw": 180.0},
        {"name": "Aisle 3 West Entrance",  "x": -1.8, "y": -0.5, "yaw": 90.0},
        {"name": "Aisle 3 East Exit",      "x": 1.6,  "y": -0.5, "yaw": 0.0},
        {"name": "Aisle 2 East Entrance",  "x": 1.6,  "y": 1.0,  "yaw": 90.0},
        {"name": "Aisle 2 West Exit",      "x": -1.8, "y": 1.0,  "yaw": 180.0},
        {"name": "Aisle 1 West Entrance",  "x": -1.8, "y": 2.5,  "yaw": 90.0},
        {"name": "Aisle 1 East Exit",      "x": 1.6,  "y": 2.5,  "yaw": 0.0},
        {"name": "North Top Corridor",     "x": 0.0,  "y": 3.2,  "yaw": 180.0},
        {"name": "West Main Walkway",      "x": -2.2, "y": 0.0,  "yaw": -90.0},
        {"name": "Checkout Counter 1",     "x": -1.6, "y": -3.0, "yaw": -90.0},
        {"name": "Base Station (Spawn Circle)", "x": 0.0, "y": -3.5, "yaw": 90.0},
    ]

    goal_poses = []
    for wp in waypoints_data:
        goal_poses.append(create_pose(navigator, wp["x"], wp["y"], wp["yaw"]))

    print(f"\n[INFO] Loaded {len(goal_poses)} patrol waypoints including Return to Base.")
    print("[INFO] Beginning autonomous store patrol...\n")

    # Send entire sequence of poses to Nav2
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            curr_idx = feedback.current_waypoint
            target_name = waypoints_data[curr_idx]["name"] if curr_idx < len(waypoints_data) else "Final"
            target_pos = (waypoints_data[curr_idx]["x"], waypoints_data[curr_idx]["y"]) if curr_idx < len(waypoints_data) else (0, 0)
            print(
                f"[PATROL] Target #{curr_idx + 1}/{len(waypoints_data)}: {target_name} at {target_pos} | Navigating...",
                end="\r"
            )
        time.sleep(0.5)

    print("\n")
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("=" * 60)
        print("  SUCCESS: Robot completed full patrol and returned to Base!")
        print("=" * 60)
    elif result == TaskResult.CANCELED:
        print("[WARN] Autonomous patrol task was canceled.")
    elif result == TaskResult.FAILED:
        print("[ERROR] Autonomous patrol task failed to reach destination.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
