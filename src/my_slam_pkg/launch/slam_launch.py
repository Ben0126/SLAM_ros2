from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # RPLidar節點
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('rplidar_ros'), 
                'launch', 'rplidar_c1_launch.py')
            ])
        ),
        
        # 發布map到odom的轉換
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # 發布odom到base_link的轉換
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # 發布base_link到laser的轉換
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),

        # SLAM Toolbox節點
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'scan_topic': '/scan',
                'max_laser_range': 12.0,
                'resolution': 0.05,
                'map_update_interval': 1.0,  # 更頻繁更新地圖
                'transform_timeout': 0.2,
                'update_timing': true,
                'enable_interactive_mode': false,
                
                # 調整SLAM參數
                'minimum_time_interval': 0.2,
                'transform_publish_period': 0.02,
                'map_update_interval': 1.0,
                'resolution': 0.05,
                'max_laser_range': 12.0,
                'minimum_travel_distance': 0.1,  # 最小移動距離
                'minimum_travel_heading': 0.1,   # 最小旋轉角度
                'scan_buffer_size': 10,
                'scan_buffer_maximum_scan_distance': 10.0,
                'link_match_minimum_response_fine': 0.1,
                'link_scan_maximum_distance': 1.5,
                'loop_search_maximum_distance': 3.0,
                'do_loop_closing': true,
                'loop_match_minimum_chain_size': 3,
                'loop_match_maximum_variance_coarse': 3.0,
                'loop_match_minimum_response_coarse': 0.35,
                'loop_match_minimum_response_fine': 0.45
            }]
        )
    ]) 