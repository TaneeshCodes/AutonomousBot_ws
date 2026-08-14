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

"""Unified launch file for Gazebo simulation and Navigation2 stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for simulation and navigation."""
    # Package directories
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav_dir = get_package_share_directory('turtlebot3_navigation2')
    assessment_launch_dir = get_package_share_directory('assessment_launch')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(nav_dir, 'map', 'map.yaml')
    )
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(assessment_launch_dir, 'param', 'burger.yaml')
    )

    # 1. Gazebo simulation launch
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': '-2.0',
            'y_pose': '-0.5'
        }.items()
    )

    # 2. Navigation2 launch with custom parameter file
    navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
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
        gazebo_simulation,
        navigation_stack
    ])
