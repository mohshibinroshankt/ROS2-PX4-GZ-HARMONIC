from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('px4_ros_com')
    
    # Declare the config file path as a launch argument
    config_file_path = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    
    return LaunchDescription([
        # Add logging to verify the path
        LogInfo(msg=['Config file path: ', config_file_path]),
        
        # Python nodes
        Node(
            package='px4_ros_com',
            executable='takeoff_node.py',
            name='takeoff_node',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            package='px4_ros_com',
            executable='path_planning_node.py',
            name='path_planning_node',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            package='px4_ros_com',
            executable='obstacle_avoidance.py',
            name='obstacle_avoidance_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # Gazebo bridge node
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            parameters=[{'config_file': config_file_path}],
        )
    ])