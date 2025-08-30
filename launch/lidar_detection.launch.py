#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tortoisebot_gazebo_dir = get_package_share_directory('tortoisebot_gazebo')
    
    gazebo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(tortoisebot_gazebo_dir, 'launch', 'gazebo_world.launch.py')
        ])
    )
    
    closest_object_detector_node = Node(
        package='tortoisebot_gazebo',
        executable='closest_object_detector.py',  
        name='closest_object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        gazebo_world_launch,
        TimerAction(
            period=5.0,
            actions=[closest_object_detector_node]
        ),
    ])