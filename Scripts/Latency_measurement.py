import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class LatencyListener(Node):
    def __init__(self):
        super().__init__('latency_listener')
        self.subscription = self.create_subscription(
            Image,
            '/stitched_image',  # Adjust the topic name based on your actual setup
            self.listener_callback,
            10)
        self.start_time = None

    def listener_callback(self, msg):
        if self.start_time is not None:
            end_time = time.time()
            latency = (end_time - self.start_time) * 1000  # Latency in milliseconds
            self.get_logger().info(f'Latency: {latency} ms')
        self.start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = LatencyListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
