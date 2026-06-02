#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry',
            executable='ackermann_odometry',
            name='ackermann_odometry',
            output='screen',
            parameters=[{
                'cmd_topic': '/autonomous/ackermann_cmd'
            }]
        ),

        Node(
            package='smart_parking',
            executable='parking_node',
            name='parking_node',
            output='screen'
        )
    ])