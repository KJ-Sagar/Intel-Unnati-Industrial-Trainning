#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Node initialized and subscribers/publishers created')
    
    def image_callback(self, msg):
        self.get_logger().info('Image received')
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image (example: convert to grayscale)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Placeholder for additional image processing
            
            # Log processed image shape
            self.get_logger().info(f'Processed image shape: {gray_image.shape}')
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def laser_callback(self, msg):
        self.get_logger().info('Laser scan received')
        try:
            # Process the laser scan data
            ranges = np.array(msg.ranges)
            self.get_logger().info(f'Laser scan ranges: {ranges}')
            
            # Example: Find the minimum range
            min_range = np.min(ranges)
            self.get_logger().info(f'Minimum range: {min_range}')
            
            # Placeholder for additional laser processing
            
        except Exception as e:
            self.get_logger().error(f"Error processing laser scan: {e}")

    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Published cmd_vel: linear={linear}, angular={angular}')

def main(args=None):
    rclpy.init(args=args)
    
    node = MyRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
