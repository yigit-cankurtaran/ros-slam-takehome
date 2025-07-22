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
                'max_laser_range': 30.0,  # might be too high, could change it
                'minimum_time_interval': 0.1,
                'transform_timeout': 1.0,
                'tf_buffer_duration': 30.0,

                'map_start_at_dock': True,
                'mode': 'mapping',
                'debug_logging': True,
                'throttle_scans': 1,
                'minimum_travel_distance': 0.01,  # Very small movement
                'minimum_travel_heading': 0.017,  # ~1 degree
            }]
        )
    ])
