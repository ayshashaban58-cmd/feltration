#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    pkg_path = os.path.join(os.getcwd(), 'src', 'pre_interview', 'pre_interview')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_path, 'vehicle_model.py')],
            output='screen',
            name='vehicle_model'
        ),
        Node(
            package='pre_interview',
            executable='controller', 
            name='pid_controller',
            output='screen'
        )
    ])