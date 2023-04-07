import os


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import (OnProcessStart, OnProcessExit)

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    
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
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("nick_bot_1"),
            "config",
            "nickbot_controllers.yaml",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
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
    ])
