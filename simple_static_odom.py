#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
import math

class StaticOdometry(Node):
    def __init__(self):
        super().__init__('static_odometry')
        
        # Use simulation time
        self.declare_parameter('use_sim_time', True)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to scan just to get timing
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Simple state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        
        # Motion parameters - adjust these based on your data
        self.linear_vel = 0.05   # 5cm/s forward (reasonable for indoor)
        self.angular_vel = 0.02  # 0.02 rad/s rotation (~1 degree/sec)
        
        self.get_logger().info('Static Odometry started - robot will move forward slowly')
    
    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)
    
    def scan_callback(self, msg):
        # Use scan timestamp for consistency
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt > 0.001:  # Avoid division by zero
                # Simple constant velocity model
                dx = self.linear_vel * dt
                dtheta = self.angular_vel * dt
                
                # Update pose
                self.x += dx * math.cos(self.theta)
                self.y += dx * math.sin(self.theta)
                self.theta += dtheta
                
                # Publish odometry and transform
                self.publish_odometry(current_time)
                self.publish_transform(current_time)
        
        self.last_time = current_time
    
    def publish_odometry(self, stamp):
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
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel
        
        # Low covariance for stable odometry
        pose_cov = [0.01] + [0.0]*35
        pose_cov[7] = 0.01    # y
        pose_cov[35] = 0.02   # yaw
        
        twist_cov = [0.01] + [0.0]*35
        twist_cov[35] = 0.02  # angular
        
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
    node = StaticOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
