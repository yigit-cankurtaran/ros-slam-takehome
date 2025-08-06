# My ROS2 SLAM Project

A complete SLAM (Simultaneous Localization and Mapping) system built with ROS2 and SLAM Toolbox that creates detailed maps from 3D LiDAR data.

![SLAM Map Overview](takehome/Screenshot%202025-07-22%20at%2021.11.25.png)

- Implementing a complete SLAM system using ROS2 and SLAM Toolbox
- I picked SLAM Toolbox because it's the most mature and actively maintained 2D SLAM package for ROS2, and ran perfectly on my machine

## Key Components

Data Source: ROS 2 bag file containing:

- /unilidar/cloud - 3D point cloud data
- /unilidar/imu - IMU sensor data


- Custom PointCloud-to-LaserScan Converter: Converts 3D point cloud to 2D laser scan data for SLAM processing
- Custom Static Odometry Generator: Provides consistent odometry estimates for SLAM when wheel encoders are unavailable
- SLAM Toolbox: Core SLAM algorithm for map building and localization
- Static TF Publisher: Provides coordinate frame transforms

I had to write the Custom components myself due to established packages (e.g. pointcloud-to-laserscan and hector SLAM) not running on my machine.

### Custom Implementation Example
![Custom Odometry Code](takehome/Screenshot%202025-07-22%20at%2021.26.51.png)

System Requirements

ROS Version: ROS 2 Humble
Required Packages:

- ros-humble-slam-toolbox
- ros-humble-tf2-ros
- ros-humble-rviz2

## Step-by-Step Execution Guide

I used 6 terminals to run my project.

### Terminal 1: Bag File Playback
``ros2 bag play ~/Downloads/rosbag2_2025_07_14-10_43_02 --clock --start-paused --rate 5.0``

- --clock: Publishes simulation time for synchronization
- --start-paused: Allows setup of other components before data playback
- --rate 5.0: starts movement at 5x the original speed

### Terminal 2: PointCloud to LaserScan Conversion
``python3 pointcloud_to_laserscan.py --ros-args \
  --remap /cloud_in:=/unilidar/cloud``

- Converts 3D point cloud data to 2D laser scan
- Output: /scan topic

### Terminal 3: Static Transform Publisher
``ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link unilidar_lidar --ros-args -p use_sim_time:=true``

- Publishes transform between robot base frame and laser frame
- Essential for coordinate frame consistency

### Terminal 4: Odometry Generator

``python3 simple_static_odom.py --ros-args -p use_sim_time:=true``

- Generates consistent odometry estimates
- Output: /odom topic and odom->base_link transform

### Terminal 5: SLAM Toolbox
inside the launch/ folder run
``ros2 launch slam_launch.py``

- Runs the core SLAM algorithm
- Output: /map topic

### Terminal 6: Visualization

``rviz2 --ros-args -p use_sim_time:=true``

## RViz2 Setup

Set Fixed Frame: map

Add Displays:

- LaserScan: Topic /scan
- Map: Topic /map
- Odometry: Topic /odom (shows robot trajectory)
- TF: Shows coordinate frame relationships

## Execution Sequence

- Start all terminals 2-6 (keep bag paused)
- Verify all topics are ready:
  - ros2 topic list  (Should show /scan, /odom)

- Press spacebar in Terminal 1 to start bag playback
- Monitor RViz2 - map should begin building immediately

## Troubleshooting
### Common Issues and Solutions

Timestamp Errors:

- Ensure all nodes use use_sim_time:=true
- Verify bag file uses --clock flag

Missing Transforms:

- Check static transform publisher is running
- Verify frame names match (base_link, unilidar_lidar)

No Map Generation:

- Ensure /scan topic is publishing
- Verify /odom topic has reasonable values
- Check SLAM Toolbox parameters

RViz Not Showing Data:

- Set correct Fixed Frame (map)
- Ensure RViz uses simulation time
- Check topic names in display settings

## Results

### Real-time 3D Point Cloud Visualization
The system processes 3D point cloud data from the LiDAR sensor, converting it to 2D laser scans for SLAM processing:

![Point Cloud Data](takehome/possiblefirstscreenshot.png)

### Generated Laser Scan Data  
The custom PointCloud-to-LaserScan converter creates clean 2D laser scan data:

![Laser Scan Data](takehome/onlylaserscan.png)

### Final SLAM Map Results
The system successfully generates:

- High-quality occupancy grid maps showing room layouts and corridors
- Real-time robot trajectory visualization  
- Consistent coordinate frame relationships
- Loop closure detection for map accuracy

![Complete SLAM Map](takehome/Screenshot%202025-07-22%20at%2021.11.25.png)

### System Console Output
SLAM Toolbox provides real-time feedback on map generation progress:

![Console Output](takehome/Screenshot%202025-07-22%20at%2020.43.24.png)
