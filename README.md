# to run ros_env
1. run miniforge
2. the eval line
3. run mamba activate ros_env

# SLAM package
- comes from ros-humble-slam-toolbox

# to see stuff in rviz
1. play the bag with "ros2 bag play ~/Downloads/rosbag2_2025_07_14-10_43_02 --clock --start-paused"
2. run rviz2, fixed frame is unilidar_lidar

fixed frame was found with the command
"ros2 topic echo /unilidar/cloud | head"
then ctrl+c and look at the frame_id line

# pointcloud to laserscan
- normal package had versioning errors whatever i did, had to get my own solution

- command is "python3 pointcloud_to_laserscan.py --ros-args \
  --remap /cloud_in:=/unilidar/cloud"

laserscan by topic in rviz2, getting a pretty nice display

# /odom doesn't exist
- lost too much time trying to get /odom working, will need to get our own odometry from IMU + pointcloud

"❯ ros2 topic echo /odom
WARNING: topic [/odom] does not appear to be published yet
Could not determine the type for the passed topic"
