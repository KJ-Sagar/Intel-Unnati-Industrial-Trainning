import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class OccupancyGridSubscriber(Node):
    def __init__(self):
        super().__init__('occupancy_grid_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'occupancy_grid',
            self.occupancy_grid_callback,
            10
        )

    def occupancy_grid_callback(self, msg):
        self.get_logger().info('Received occupancy grid with width: %d, height: %d' % (msg.info.width, msg.info.height))

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
