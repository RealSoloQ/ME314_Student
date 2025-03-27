#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Include the xarm7 MoveIt+Gazebo launch
    xarm_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xarm_moveit_config'),
                'launch',
                'xarm7_moveit_gazebo.launch.py'
            )
        ),
        launch_arguments={
            'add_gripper': 'true',
            'add_realsense_d435i': 'true'
        }.items()
    )

    block_spawn = TimerAction(
        period=5.0,  # give Gazebo time to start
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-file', os.path.join(
                    get_package_share_directory('me314'),
                    'me314', 'gazebo_models', 'red_block.sdf'
                ),
                '-entity', 'block',
                '-x', '0', '-y', '-1', '-z', '1.021'
            ],
            parameters=[{'use_sim_time': True}],
        )]
    )
    
    # 2) Launch xarm_pose_commander with a delay
    xarm_pose_commander_node = Node(
        package='me314',  
        executable='xarm_commander_node.py',    
        output='screen',
        parameters=[{'use_sim': True}] 
    )
    
    # Add a delay of 5 seconds before starting the commander
    delayed_commander = TimerAction(period=13.0, actions=[xarm_pose_commander_node])
    
    return LaunchDescription([
        xarm_moveit_gazebo_launch,
        block_spawn,
        delayed_commander
    ])