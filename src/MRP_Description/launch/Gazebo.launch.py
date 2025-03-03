# Copyright (c) 2024 Matthew Allan Tidd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # sim time as a launch configuration:
    use_sim_time = LaunchConfiguration('use_sim_time')

    # prepare the URDF file:
    pkg_path = os.path.join(get_package_share_directory('MRP_Description'))
    print(f'\n{pkg_path}\n')
    xacro_file = os.path.join(pkg_path, 'urdf', 'MRP.urdf.xacro')
    print(f'\n{xacro_file}\n')
    robot_description_config = xacro.process_file(xacro_file)

    # create a robot_state_publisher node:
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # include the gazebo launch file:
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
    ))

    # run the spawner node from the gazebo_ros package:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'MRP'],
                        output='screen')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name = 'use_sim_time',
            default_value = 'false',
            description = 'Use simulation time'
        ),
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])