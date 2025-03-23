from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('auto_drive_pkg'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='auto_drive_pkg',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            parameters=[config],
            output='screen'
        )
    ])