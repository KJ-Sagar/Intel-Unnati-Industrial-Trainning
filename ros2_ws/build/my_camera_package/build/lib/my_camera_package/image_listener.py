#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageListener(Node):

    def __init__(self):
        super().__init__('image_listener')
        self.bridge = CvBridge()
        
        self.image1 = None
        self.image2 = None
        self.image3 = None
        self.image4 = None
        
        self.subscription1 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera1/image_raw',
            self.listener_callback1,
            10)
        
        self.subscription2 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera2/image_raw',
            self.listener_callback2,
            10)
        
        self.subscription3 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera3/image_raw',
            self.listener_callback3,
            10)
        
        self.subscription4 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera4/image_raw',
            self.listener_callback4,
            10)

    def listener_callback1(self, msg):
        self.get_logger().info('Receiving image from camera 1')
        self.image1 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_combined_image()

    def listener_callback2(self, msg):
        self.get_logger().info('Receiving image from camera 2')
        self.image2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_combined_image()

    def listener_callback3(self, msg):
        self.get_logger().info('Receiving image from camera 3')
        self.image3 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_combined_image()

    def listener_callback4(self, msg):
        self.get_logger().info('Receiving image from camera 4')
        self.image4 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_combined_image()

    def show_combined_image(self):
        if self.image1 is not None and self.image2 is not None and self.image3 is not None and self.image4 is not None:
            # Stitch the images together into a 2x2 grid
            top_row = np.hstack((self.image1, self.image2))
            bottom_row = np.hstack((self.image3, self.image4))
            combined_image = np.vstack((top_row, bottom_row))

            # Show the combined image
            cv2.imshow("Combined Camera Image", combined_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
