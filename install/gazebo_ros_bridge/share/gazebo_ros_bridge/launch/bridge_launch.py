#!/usr/bin/env python3

# from launch import LaunchDescription
# from launch.actions import ExecuteProcess

# def generate_launch_description():
#     """
#     Launch file to bridge Gazebo lidar topics to ROS2
#     """
#     bridge_cmd = ExecuteProcess(
#         cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
#              '/world/iris_maze/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan' +
#              '@sensor_msgs/msg/LaserScan' +
#              '[gz.msgs.LaserScan' +
#              '@ros2_gz_bridge/convert'],
#         output='screen'
#     )

#     return LaunchDescription([
#         bridge_cmd
#     ])

import launch
import launch_ros.actions

def generate_launch_description():
    """
    Launch file to bridge Gazebo lidar topics to ROS2.
    """
    bridge_node = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/iris_maze/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
        ],
        output="screen"
    )

    return launch.LaunchDescription([
        bridge_node
    ])