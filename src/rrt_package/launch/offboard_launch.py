#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Replace 'my_package' with the actual name of your package
    dstar_planner_node = Node(
        package='my_package',
        executable='dstar_planner',
        name='dstar_planner',
        output='screen'
    )
    
    offboard_controller_node = Node(
        package='my_package',
        executable='offboard_controller',
        name='offboard_controller',
        output='screen'
    )
    
    return LaunchDescription([
        dstar_planner_node,
        offboard_controller_node
    ])
