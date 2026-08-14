#!/bin/bash
# Convenience script to source and launch the Gazebo simulation + Nav2 stack.

# Source ROS 2 Jazzy and the workspace install setup
source /opt/ros/jazzy/setup.bash
if [ -f /home/taneesh/Nav2_Assessment_ws/install/setup.bash ]; then
    source /home/taneesh/Nav2_Assessment_ws/install/setup.bash
fi

echo "===================================================================="
echo "To run the Navigation Monitor node, execute this in a new terminal:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/Nav2_Assessment_ws/install/setup.bash"
echo "  ros2 run nav_monitor monitor_node"
echo "===================================================================="
echo ""
echo "Launching Gazebo Simulation, Navigation2, and RViz..."
ros2 launch assessment_launch assessment.launch.py
