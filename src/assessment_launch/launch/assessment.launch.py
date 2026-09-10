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

"""Unified launch file for Supermarket Arena Gazebo simulation and Navigation2 stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for simulation, online SLAM mapping, and navigation."""
    # Package directories
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav_dir = get_package_share_directory('turtlebot3_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    assessment_launch_dir = get_package_share_directory('assessment_launch')
    models_dir = os.path.join(gazebo_dir, 'models')
    textures_dir = os.path.join(assessment_launch_dir, 'textures')

    # Environment variables for Gazebo model and texture loading
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger_cam')
    set_gz_resource = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', f'{models_dir}:{textures_dir}')
    set_ign_resource = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', f'{models_dir}:{textures_dir}')
    set_sdf_path = AppendEnvironmentVariable('SDF_PATH', f'{models_dir}:{textures_dir}')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam', default='False')
    world = LaunchConfiguration(
        'world',
        default=os.path.join(assessment_launch_dir, 'worlds', 'supermarket_arena.sdf')
    )
    # Circle / initial spawning coordinates
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-3.5')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    yaw_pose = LaunchConfiguration('yaw_pose', default='1.5708')

    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(assessment_launch_dir, 'maps', 'map.yaml')
    )
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(assessment_launch_dir, 'param', 'burger.yaml')
    )

    # 1. Gazebo Sim Server (loads supermarket_arena.sdf)
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    # 2. Gazebo Sim GUI Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # 3. Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. Direct Gazebo Spawner with explicit yaw orientation (-Y)
    urdf_path = os.path.join(models_dir, 'turtlebot3_burger_cam', 'model.sdf')
    bridge_params = os.path.join(gazebo_dir, 'params', 'turtlebot3_burger_cam_bridge.yaml')

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'burger',
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen',
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Bridge camera image raw
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # 5. Navigation2 Bringup with autonomous navigation enabled
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # 6. RViz2 visualization
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav_dir, 'rviz', 'tb3_navigation2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        set_tb3_model,
        set_gz_resource,
        set_ign_resource,
        set_sdf_path,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Whether to run SLAM (True) or autonomous navigation on pre-built map (False)'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world,
            description='Full path to world SDF file to load'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Robot spawn X position (circle marker)'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-3.5',
            description='Robot spawn Y position (circle marker)'
        ),
        DeclareLaunchArgument(
            'yaw_pose',
            default_value='1.5708',
            description='Robot spawn yaw orientation angle (rad)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=map_yaml_file,
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to local custom parameter file'
        ),
        gzserver_cmd,
        gzclient_cmd,
        start_gazebo_ros_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd,
        robot_state_publisher_cmd,
        navigation_stack,
        rviz_cmd
    ])


