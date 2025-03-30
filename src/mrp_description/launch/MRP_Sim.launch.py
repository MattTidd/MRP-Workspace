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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # launch parameters:
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # launch arguments:
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true')
    
    use_ros2_control_arg = DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true')

    gazebo_params_path = os.path.join(get_package_share_directory('mrp_description'), 'config', 'gazebo_params.yaml')

    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('mrp_description'),'launch','rsp.launch.py'
                )])
                , launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items()
    )

    # gazebo:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments = {'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path}.items()
             )
 
    # spawner node:
    spawn_entity = Node(package = 'gazebo_ros', executable = 'spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'MRP'],
                        output='screen')
    
    # delayed spawner node:

    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])
    
    # spawn the diff_drive controller:
    diff_drive_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        condition = IfCondition(LaunchConfiguration("use_ros2_control")),
        arguments = ['diff_drive_controller']
    )

    # spawn the joint broadcaster:
    joint_broadcaster_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        condition = IfCondition(LaunchConfiguration("use_ros2_control")),
        arguments = ['joint_state_broadcaster']
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_ros2_control_arg,
        robot_state_publisher,
        gazebo, 
        delayed_spawn,
        diff_drive_spawner,
        joint_broadcaster_spawner
    ])

