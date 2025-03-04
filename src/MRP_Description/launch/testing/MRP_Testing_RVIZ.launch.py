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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # prepare the URDF file:
    pkg_path = os.path.join(get_package_share_directory('MRP_Description'))
    print(f'\n{pkg_path}\n')
    xacro_file = os.path.join(pkg_path, 'urdf', 'testing', 'MRP_Testing.urdf.xacro')
    print(f'\n{xacro_file}\n')
    robot_description_config = xacro.process_file(xacro_file)

    rviz_config_path = os.path.join(pkg_path, 'config', 'RVIZ_config.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name = 'rviz',
            default_value = 'true',
            description = 'Run RVIZ'
        ),

        DeclareLaunchArgument(
            name = 'use_sim_time',
            default_value = 'false',
            description = 'Use simulation time'
        ),

        DeclareLaunchArgument(
            name='publish_joints', 
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        Node(
            package = 'robot_state_publisher',
            executable = 'robot_state_publisher',
            name = 'robot_state_publisher',
            output = 'screen',
            parameters = [{
                'use_sim_time' : LaunchConfiguration('use_sim_time'),
                'robot_description' : robot_description_config.toxml()
            }]
        ),

        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz2',
            output = 'screen',
            arguments = ['-d', rviz_config_path],
            condition = IfCondition(LaunchConfiguration("rviz")),
            parameters = [{'use_sim_time' : LaunchConfiguration('use_sim_time')}]
        ),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
    ])