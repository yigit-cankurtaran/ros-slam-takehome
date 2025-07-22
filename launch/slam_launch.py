from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_link',  # standard base fram
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'resolution': 0.05,
                'max_laser_range': 10.0,  # lowering this due to queue errors
                'minimum_time_interval': 0.2,
                'transform_timeout': 2.0,
                'tf_buffer_duration': 60.0,

                'map_start_at_dock': True,
                'mode': 'mapping',
                'debug_logging': False,
                'throttle_scans': 2,
                'minimum_travel_distance': 0.05,  # Very small movement
                'minimum_travel_heading': 0.05,  # ~1 degree
                'scan_buffer_size': 50,
                'scan_queue_size': 50,
            }]
        )
    ])
