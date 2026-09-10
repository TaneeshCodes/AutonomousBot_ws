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
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${IGN_GAZEBO_RESOURCE_PATH}"
export SDF_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${SDF_PATH}"

# Clean up any leftover background Gazebo processes to avoid session collisions
killall -9 gz sim gzserver gzclient ruby 2>/dev/null || true



echo "===================================================================="
echo "To run the Navigation Monitor node, execute this in a new terminal:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source $WS_DIR/install/setup.bash"
echo "  ros2 run nav_monitor monitor_node"
echo "===================================================================="
echo ""
echo "Launching Gazebo Simulation, Navigation2, and RViz..."
ros2 launch assessment_launch assessment.launch.py "$@"

