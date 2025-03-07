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
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # launch configuration:
    use_sim_time = LaunchConfiguration('use_sim_time')

    # launch arguments:
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    # paths:
    pkg_path = os.path.join(get_package_share_directory('MRP_Description'))
    rviz_config_path = os.path.join(pkg_path, 'config', 'RVIZ_config.rviz')

    # nodes:
    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('MRP_Description'),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'false'}.items()
    )

    joint_state_publisher = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        parameters = [{'use_sim_time': use_sim_time}]
    )

    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_path],
        parameters = [{'use_sim_time' : use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])