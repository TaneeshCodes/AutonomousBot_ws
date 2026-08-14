# ROS 2 Navigation & Robotics Assessment

This workspace contains implementations for the three tasks specified in the assessment document `ROS - Task.docx`.

## Repository Structure

```
Nav2_Assessment_ws/
├── src/
│   ├── assessment_launch/          # Task 1: Spawner and Navigation unified launcher
│   │   ├── launch/
│   │   │   └── assessment.launch.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── nav_monitor/                # Task 2: Custom ROS 2 navigation monitor node
│       ├── nav_monitor/
│       │   ├── __init__.py
│       │   └── monitor.py
│       ├── package.xml
│       └── setup.py
├── cone_toolpath/                  # Task 3: 6-Axis Robot Toolpath on Cone Surface
│   ├── cone_model.obj              # Generated cone 3D reference mesh
│   ├── cone_toolpath.csv           # Final exported CSV containing joint trajectories
│   ├── toolpath_visualization.png   # 3D trajectory plot showing the robot path on the cone
│   └── generate_toolpath.py        # Numerical Inverse Kinematics (IK) trajectory generator
├── launch/
│   └── launch_all.sh               # Convenience execution shell script
├── ROS - Task.docx                 # Original assessment document
├── plan.txt                        # Execution roadmap
└── README.md                       # Documentation
```

## System Environment

- **ROS 2 Distribution**: Jazzy Jalisco (`/opt/ros/jazzy`)
- **Robot Model**: TurtleBot3 Burger (`TURTLEBOT3_MODEL=burger`)
- **Simulation**: Gazebo Sim (Ignition/Harmonic)

---

## Build & Test Instructions

Ensure the ROS 2 environment is sourced first.

1. **Build the Workspace**:
   ```bash
   cd ~/Nav2_Assessment_ws
   colcon build --symlink-install
   ```

2. **Run Automated Code Quality and Unit Tests**:
   ```bash
   colcon test
   colcon test-result --all
   ```

---

## Running the Tasks

### Task 1: Autonomous Navigation with Dynamic Obstacle Replanning

1. **Launch the Simulation, RViz, and Navigation Stack**:
   ```bash
   ./launch/launch_all.sh
   ```
   This loads the Gazebo world, spawns the Burger robot at `(-2.0, -0.5)`, brings up Nav2, and starts RViz2.

2. **Autonomous Replanning Demonstration**:
   - In RViz, initialize the localization pose using **2D Pose Estimate**.
   - Send a navigation goal using **2D Goal Pose**.
   - While the robot is navigating, select the obstacle spawner tool in Gazebo and drop a box/cylinder in front of the planned path.
   - The robot will autonomously detect the obstacle, update its local/global costmaps, and plan/execute a safe path to reach the goal.

### Task 2: Custom ROS 2 Navigation Monitoring Node

1. **Start the Monitor Node**:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/Nav2_Assessment_ws/install/setup.bash
   ros2 run nav_monitor monitor_node
   ```

2. **Output Format**:
   The monitor node prints a live console dashboard matching the requirements:
   ```
   ========================================
   Current Goal: (1.20, -0.15)
   Remaining Distance: 2.35 m
   Status: NAVIGATING
   ========================================
   ```
   - **Supported Status Values**: `IDLE`, `NAVIGATING`, `REPLANNING`, `SUCCEEDED`, `FAILED`.
   - **Replanning detection**: Subscribes to `/compute_path_to_pose/_action/status` to determine when the planner server is active during navigation.

### Task 3: 6-Axis Robot Toolpath Generation for Cone Surface

1. **Generate the Toolpath**:
   Run the trajectory generator:
   ```bash
   python3 cone_toolpath/generate_toolpath.py
   ```
   This script:
   - Generates a continuous spiral path starting from the apex of the cone down to its base.
   - Computes target position $(X, Y, Z)$ and tool orientation (pointing normal to the cone surface).
   - Solves numerical Inverse Kinematics (IK) using optimization constraints to calculate joint parameters.
   - Exports the 3D model [cone_model.obj](file:///home/taneesh/Nav2_Assessment_ws/cone_toolpath/cone_model.obj), the trajectory file [cone_toolpath.csv](file:///home/taneesh/Nav2_Assessment_ws/cone_toolpath/cone_toolpath.csv), and a 3D visualization [toolpath_visualization.png](file:///home/taneesh/Nav2_Assessment_ws/cone_toolpath/toolpath_visualization.png).
