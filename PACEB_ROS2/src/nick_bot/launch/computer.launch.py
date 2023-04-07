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
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def load_yaml(package_name, *paths):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, *paths)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None




def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("nick_bot", 
        package_name="nick_bot_moveit_config").robot_description(file_path="config/nick_bot.urdf.xacro").robot_description_semantic(file_path="config/nick_bot.srdf").joint_limits(file_path="config/joint_limits.yaml").robot_description_kinematics(file_path="config/kinematics.yaml").to_moveit_configs()
   

 

    config_dict = moveit_config.to_dict()


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            config_dict,
             {"use_sim_time": False},
            ],
    )
    rviz_base = os.path.join(get_package_share_directory("nick_bot"), "config")
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_kinematics,
            moveit_config.robot_description_semantic,
            {"use_sim_time": False},
        ]
    )

    # Process the URDF file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("nick_bot"),
                    "description",
                    "robot.urdf.xacro",
                ]
            ),
        ]
    )  
    
    
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
	    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nick_bot'), 'launch'), '/rsp.launch.py']),
            launch_arguments={'use_sim_time': False}.items()
            )
    

    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("nick_bot"),
            "config",
            "step_and_serv_controllers.yaml",
        ]
    )


    rviz_base = os.path.join(get_package_share_directory("nick_bot"), "config")
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_kinematics,
            moveit_config.robot_description_semantic,
            # ompl_planning_pipeline_config
        ]
    )

    # Create a robot_state_publisher node

    params = {'robot_description': robot_description_content, 'use_sim_time' : False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            params,
            {"use_sim_time": False},
            ]
    )


    robot_desc = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params = os.path.join(get_package_share_directory("nick_bot"), "config", "step_and_serv_controllers.yaml")


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

    load_hand2 = Node(
		    package="controller_manager",
		    executable="spawner",
		    arguments=["hand_group_controller"],
		)
    load_arm2 = Node(
		    package="controller_manager",
		    executable="spawner",
		    arguments=["arm_group_controller"],
		)
    

    # Launch!
    return LaunchDescription([
        node_robot_state_publisher,
        static_tf,
        move_group_node,
        rviz_node,
    ])
