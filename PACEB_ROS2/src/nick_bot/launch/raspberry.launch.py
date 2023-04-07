import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node, SetRemap

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction
from launch.event_handlers import (OnProcessStart, OnProcessExit)

from launch_ros.substitutions import FindPackageShare



def load_yaml(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():

    robot_desc = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params = os.path.join(get_package_share_directory("nick_bot"), "config", "my_controllers.yaml")


    controller_manager = Node(
	    package="controller_manager",
	    executable="ros2_control_node",
	    parameters=[
            {'robot_description': robot_desc},
            controller_params
        ]
	)

    delay_c_m = TimerAction(period=3.0, actions=[controller_manager])


    load_jsb2 = Node(
	    package="controller_manager",
	    executable="spawner",
	    arguments=["joint_state_broadcaster"],
	)

    load_arm2 = Node(
		    package="controller_manager",
		    executable="spawner",
		    arguments=["arm_group_controller"],
		)
    

    # Launch!
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[load_jsb2]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=load_jsb2,
                on_start=[load_arm2]
            )
        ),
        delay_c_m,
    ])
