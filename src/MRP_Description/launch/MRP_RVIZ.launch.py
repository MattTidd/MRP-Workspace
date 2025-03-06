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


def generate_launch_description():
    # launch arguments:
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = os.path.join(get_package_share_directory('MRP_Description'))
    rviz_config_path = os.path.join(pkg_path, 'config', 'RVIZ_config.rviz')

    # nodes:
    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('MRP_Description'),'launch','rsp.launch.py'
                )])
                , launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'false'}.items()
    )

    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'RVIZ2',
        output = 'screen',
        arguments = ['-d', rviz_config_path],
        parameters = [{'use_sim_time' : use_sim_time}]
    )

    joint_state_publisher_gui = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        name = 'joint_state_publisher_gui',
        parameters = [{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name = 'use_sim_time',
            default_value = 'true',
            description = 'Use simulation time'
        ),

        robot_state_publisher,
        rviz,
        joint_state_publisher_gui
    
    ])