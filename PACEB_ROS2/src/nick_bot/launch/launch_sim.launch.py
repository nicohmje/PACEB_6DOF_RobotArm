import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nick_bot_1'), 'launch'), '/gazebo.launch.py'])
            )

    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_control = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'start', 'arm_group_controller'],
        output='screen'
    )
    
    load_hand_control = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'start', 'hand_group_controller'],
        output='screen'
    )
        
        

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('nick_bot_1'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
            
    # Launch!
    return LaunchDescription([
        gz,
        load_jsb,
        load_arm_control,
        load_hand_control
    ])
