import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node, SetRemap

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, GroupAction
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
    
    # Check if we're told to use sim time
    use_sim_time = True 
    
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
    robot_description = {"robot_description": robot_description_content}

    move_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nick_bot_moveit_config'), 'launch'), '/move_group.launch.py']),
            launch_arguments={'use_sim_time': 'true'}.items())
            
    srdf_xacro_path = os.path.join(
        get_package_share_directory("nick_bot_moveit_config"), "config", "nick_bot.srdf"
    )

            
    use_sim_time = {'use_sim_time': True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    
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
            launch_arguments={'use_sim_time': 'true'}.items()
            )
    

    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("nick_bot"),
            "config",
            "nickbot_controllers.yaml",
        ]
    )
    

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
            )
    
    
    
    spawn_entity = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-topic', 'robot_description', '-entity', 'nicks_bot'],
		output='screen'
	    )



    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

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


    load_arm_control = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_group_controller'],
        output='screen'
    )
    
    
    
    load_hand_control = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'hand_group_controller'],
        output='screen'
    )
    
    

    # Launch!
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_jsb2]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb2,
                on_exit=[load_arm2]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm2,
                on_exit=[load_hand2]
            )
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        #control_node,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        static_tf,
	    move_node,
    ])
