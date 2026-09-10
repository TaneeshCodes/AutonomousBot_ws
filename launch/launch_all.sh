#!/bin/bash
# Convenience script to source and launch the Gazebo simulation + Nav2 stack.

# Determine workspace root dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace install setup
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# Set TurtleBot3 model and simulation resource paths
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger_cam}"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/assessment_launch/textures:/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="$WS_DIR/src/assessment_launch/textures:/opt/ros/jazzy/share/turtlebot3_gazebo/models:${IGN_GAZEBO_RESOURCE_PATH}"
export SDF_PATH="$WS_DIR/src/assessment_launch/textures:/opt/ros/jazzy/share/turtlebot3_gazebo/models:${SDF_PATH}"

# Clean up any leftover background Gazebo processes to avoid session collisions
killall -9 gz sim gzserver gzclient ruby 2>/dev/null || true



echo "===================================================================="
echo "Supermarket Autonomous Bot System Ready!"
echo "--------------------------------------------------------------------"
echo "Terminal 1: Gazebo Sim + Nav2 + RViz (Running here)"
echo ""
echo "To run the ArUco Vision Detector in Terminal 2:"
echo "  source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "  ros2 run nav_monitor aruco_detector"
echo ""
echo "To run the Autonomous Route Navigator in Terminal 3:"
echo "  source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "  python3 src/assessment_launch/scripts/patrol_waypoints.py"
echo "===================================================================="
echo ""
echo "Launching Gazebo Simulation, Navigation2, and RViz..."
ros2 launch assessment_launch assessment.launch.py "$@"

