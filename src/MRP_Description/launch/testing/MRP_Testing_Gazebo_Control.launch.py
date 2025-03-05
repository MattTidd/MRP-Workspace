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
import time
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ######################################## SIM PARAMETERS ######################################

    # sim time as a launch configuration:
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')
    pkg_path = os.path.join(get_package_share_directory('MRP_Description'))

    ####################################### URDF PARAMETERS ######################################

    # prepare the URDF file path:
    xacro_file = os.path.join(pkg_path, 'urdf', 'testing', 'MRP_Testing.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    ################################ CONTROLLER MANAGER PARAMETERS ################################
    # controllers_config = os.path.join(pkg_path, 'config', 'my_controllers.yaml')

    ############################################ NODES ############################################

    # robot_state_publisher:
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time' : use_sim_time }]
    )

    # gazebo launch:
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
            'launch', 'gazebo.launch.py')
        ])
    )

    # entity spawner:
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'MRP'],
        output='screen'
    )

    # # controller manager:
    # controller_manager = Node(
    #     package = 'controller_manager',
    #     executable = 'ros2_control_node',
    #     parameters = [controllers_config],
    #     remappings=[('~/robot_description', '/robot_description')],
    #     output = 'screen'
    # )

    # load joint broad:
    load_joint_broad = ExecuteProcess(
        cmd = ['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_broad'],
        output = 'screen'
    )

    # load diff controller:
    load_diff_drive = ExecuteProcess(
        cmd = ['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_controller'],
        output = 'screen'
    )

    ########################################## LAUNCH ###########################################

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),

        # core nodes:
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        load_diff_drive,
        load_joint_broad

    ])