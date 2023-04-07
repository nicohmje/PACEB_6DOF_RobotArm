import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
            )
    


    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nick_bot'), 'launch'), '/rsp.launch.py']),
            launch_arguments={'use_sim_time': 'true'}.items()
            )
            
            
    
    spawn_entity = Node(
		package='gazebo_ros',
		executable='spawn_entity.py',
		arguments=['-topic', '/robot_description', '-entity', 'nicks_bot'],
		output='screen'
	    )



    # Launch!
    return LaunchDescription([
        gazebo,
        rsp,
        spawn_entity
    ])
