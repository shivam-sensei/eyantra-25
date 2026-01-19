#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='hb_control',
            executable='camera_testing.py',
            name='camera_testing',
            output='screen'
        ),

        Node(
            package='hb_control',
            executable='holonomic_perception.py',
            name='holonomic_perception',
            output='screen'
        ),
    ])
