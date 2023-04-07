import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction
from launch.event_handlers import (OnProcessStart, OnProcessExit)
import xacro
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder




def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
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

    # Get parameters for the Servo node


    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node_main",
    #     parameters=[
    #         servo_params,
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         moveit_config.robot_description_kinematics,
    #         {'use_intra_process_comms' : True},
    #     ],
    #     output="screen",
    # )


    # RViz
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


    pkg_path = os.path.join(get_package_share_directory('nick_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file])

    params = {'robot_description': robot_description_config, 'use_sim_time' : False}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            params,
            {"use_sim_time": False}]
    )
    
    servo_yaml = load_yaml("nick_bot", "config/nickbot_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #      package="moveit_servo",
            #      plugin="moveit_servo::ServoServer",
            #      name="servo_server",
            #      parameters=[
            #          servo_params,
            #          moveit_config.robot_description,
            #          moveit_config.robot_description_semantic,
            #      ],
            # ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
                #extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                #extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    delay_c_m = TimerAction(period=10.0, actions=[controller_manager])



    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            #{'use_intra_process_comms' : True},
        ],
        output="screen",
    )

    delay_container = TimerAction(period=1.0, actions=[servo_node])

    return LaunchDescription(
        [
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
            # RegisterEventHandler(
            #     event_handler=OnProcessStart(
            #         target_action=load_arm2,
            #         on_start=[load_hand2]
            #     )
            # ),
            #node_robot_state_publisher,
            rviz_node,
            move_group_node,
            delay_c_m,
            #static_tf
            node_robot_state_publisher,
            servo_node,
            container, 
        ]
    )
