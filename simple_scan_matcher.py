#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class SimpleScanMatcher(Node):
    def __init__(self):
        super().__init__('simple_scan_matcher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # State variables
        self.prev_scan = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        
        # Parameters
        self.max_range = 4.0  # Maximum usable range
        self.min_range = 0.1  # Minimum usable range
        
        self.get_logger().info('Simple Scan Matcher started')
    
    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
    
    def preprocess_scan(self, scan):
        """Clean up scan data"""
        ranges = np.array(scan.ranges)
        
        # Replace inf and nan with max_range
        ranges[np.isinf(ranges)] = self.max_range
        ranges[np.isnan(ranges)] = self.max_range
        
        # Clip ranges to reasonable bounds
        ranges = np.clip(ranges, self.min_range, self.max_range)
        
        return ranges
    
    def simple_icp_1d(self, scan1, scan2):
        """Very simple 1D scan correlation for translation estimation"""
        # Simple correlation-based matching
        # This is a simplified version - for better results, use proper ICP
        
        if len(scan1) != len(scan2):
            return 0.0, 0.0, 0.0
        
        # Calculate mean difference (very rough estimation)
        diff = np.array(scan2) - np.array(scan1)
        
        # Simple heuristic for movement estimation
        # This is quite crude but should give basic odometry
        front_idx = len(scan1) // 2  # Forward direction
        side_idx = len(scan1) // 4   # Side direction
        
        # Estimate forward/backward movement from front readings
        dx = -np.mean(diff[front_idx-10:front_idx+10]) * 0.01
        
        # Estimate side movement from side readings  
        dy = np.mean(diff[side_idx-10:side_idx+10]) * 0.01
        
        # Estimate rotation from scan difference pattern
        left_diff = np.mean(diff[:len(scan1)//3])
        right_diff = np.mean(diff[2*len(scan1)//3:])
        dtheta = (right_diff - left_diff) * 0.001
        
        # Limit changes to reasonable values
        dx = np.clip(dx, -0.1, 0.1)
        dy = np.clip(dy, -0.1, 0.1)
        dtheta = np.clip(dtheta, -0.1, 0.1)
        
        return dx, dy, dtheta
    
    def scan_callback(self, msg):
        current_time = self.get_clock().now()
        
        # Preprocess current scan
        current_ranges = self.preprocess_scan(msg)
        
        if self.prev_scan is not None and self.last_time is not None:
            # Calculate time difference
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt > 0.001:  # Avoid division by zero
                # Estimate motion using simple scan matching
                dx, dy, dtheta = self.simple_icp_1d(self.prev_scan, current_ranges)
                
                # Update pose
                cos_theta = math.cos(self.theta)
                sin_theta = math.sin(self.theta)
                
                # Transform local movement to global coordinates
                global_dx = dx * cos_theta - dy * sin_theta
                global_dy = dx * sin_theta + dy * cos_theta
                
                self.x += global_dx
                self.y += global_dy
                self.theta += dtheta
                
                # Calculate velocities
                vx = global_dx / dt
                vy = global_dy / dt
                vth = dtheta / dt
                
                # Publish odometry
                self.publish_odometry(current_time, vx, vy, vth)
                
                # Publish transform
                self.publish_transform(current_time)
        
        # Store current scan and time
        self.prev_scan = current_ranges
        self.last_time = current_time
    
    def publish_odometry(self, stamp, vx, vy, vth):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.euler_to_quaternion(self.theta)
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        # Covariance (rough estimates) - must be 36 float values
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        
        # Diagonal elements for pose (x, y, z, roll, pitch, yaw)
        pose_cov[0] = 0.1   # x
        pose_cov[7] = 0.1   # y
        pose_cov[14] = 0.1  # z
        pose_cov[21] = 0.1  # roll
        pose_cov[28] = 0.1  # pitch
        pose_cov[35] = 0.1  # yaw
        
        # Diagonal elements for twist (vx, vy, vz, wx, wy, wz)
        twist_cov[0] = 0.1   # vx
        twist_cov[7] = 0.1   # vy
        twist_cov[14] = 0.1  # vz
        twist_cov[21] = 0.1  # wx
        twist_cov[28] = 0.1  # wy
        twist_cov[35] = 0.1  # wz
        
        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov
        
        self.odom_pub.publish(odom)
    
    def publish_transform(self, stamp):
        """Publish TF transform"""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.euler_to_quaternion(self.theta)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleScanMatcher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
