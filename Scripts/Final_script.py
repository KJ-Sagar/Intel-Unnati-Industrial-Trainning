import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
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

        self.combined_image_pub = self.create_publisher(Image, '/stitched_image', 10)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Load YOLO model
        self.net = cv2.dnn.readNet('yolov3.weights', 'yolov3.cfg')
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

    def listener_callback1(self, msg):
        self.get_logger().info('Receiving image from camera 1')
        self.image1 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_and_publish_combined_image()

    def listener_callback2(self, msg):
        self.get_logger().info('Receiving image from camera 2')
        self.image2 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_and_publish_combined_image()

    def listener_callback3(self, msg):
        self.get_logger().info('Receiving image from camera 3')
        self.image3 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_and_publish_combined_image()

    def listener_callback4(self, msg):
        self.get_logger().info('Receiving image from camera 4')
        self.image4 = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_and_publish_combined_image()

    def show_and_publish_combined_image(self):
        if self.image1 is not None and self.image2 is not None and self.image3 is not None and self.image4 is not None:
            # Stitch the images together as per the required order
            top_row = np.hstack((self.image4, self.image3))
            bottom_row = np.hstack((self.image2, self.image1))
            combined_image = np.vstack((top_row, bottom_row))

            # Add nametags to the combined image
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_color = (255, 255, 255)
            thickness = 2
            line_type = cv2.LINE_AA

            #cv2.putText(combined_image, 'Camera 2', (10, combined_image.shape[0] - 10), font, font_scale, font_color, thickness, line_type)
            #cv2.putText(combined_image, 'Camera 1', (combined_image.shape[1] // 2 + 10, combined_image.shape[0] - 10), font, font_scale, font_color, thickness, line_type)
            #cv2.putText(combined_image, 'Camera 4', (10, combined_image.shape[0] // 2 - 10), font, font_scale, font_color, thickness, line_type)
            #cv2.putText(combined_image, 'Camera 3', (combined_image.shape[1] // 2 + 10, combined_image.shape[0] // 2 - 10), font, font_scale, font_color, thickness, line_type)

            # Print the resolution of the combined image
            self.get_logger().info(f'Combined image resolution: {combined_image.shape}')

            # Show the combined image
            cv2.imshow("Combined Camera Image", combined_image)
            cv2.waitKey(1)

            # Publish the combined image
            combined_image_msg = self.bridge.cv2_to_imgmsg(combined_image, "bgr8")
            self.combined_image_pub.publish(combined_image_msg)

            # Process and publish occupancy grid
            self.process_and_publish_occupancy_grid(combined_image)

    def process_and_publish_occupancy_grid(self, image):
        try:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

            occupancy_grid = self.create_occupancy_grid(binary_image)
            detections = self.detect_objects(image)
            occupancy_grid = self.update_occupancy_grid(occupancy_grid, detections)
            self.occupancy_grid_pub.publish(occupancy_grid)
            self.get_logger().info('Published an occupancy grid.')
        except Exception as e:
            self.get_logger().error(f"Error in process_and_publish_occupancy_grid: {e}")

    def detect_objects(self, image):
        # Use a pre-trained model to detect objects
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        detections = self.net.forward(self.output_layers)
        return self.parse_detections(detections, image.shape[1], image.shape[0])

    def parse_detections(self, detections, img_width, img_height):
        boxes = []
        confidences = []
        class_ids = []
        for out in detections:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * img_width)
                    center_y = int(detection[1] * img_height)
                    w = int(detection[2] * img_width)
                    h = int(detection[3] * img_height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        return [(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]) for i in indices]  # Remove .flatten()

    def update_occupancy_grid(self, occupancy_grid, detections):
        grid_width = occupancy_grid.info.width
        grid_height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution

        for detection in detections:
            x, y, w, h = detection
            grid_x = int(x / resolution)
            grid_y = int(y / resolution)
            grid_w = int(w / resolution)
            grid_h = int(h / resolution)

            for i in range(grid_y, grid_y + grid_h):
                for j in range(grid_x, grid_x + grid_w):
                    if 0 <= i < grid_height and 0 <= j < grid_width:
                        occupancy_grid.data[i * grid_width + j] = 100  # Mark detected object area as occupied

        return occupancy_grid

    def create_occupancy_grid(self, binary_image):
        occupancy_grid = OccupancyGrid()

        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"

        occupancy_grid.info.resolution = 0.02  # Set this to your desired resolution
        occupancy_grid.info.width = binary_image.shape[1]
        occupancy_grid.info.height = binary_image.shape[0]
        occupancy_grid.info.origin = Pose()
        occupancy_grid.info.origin.position = Point(x=0.0, y=0.0, z=0.0)
        occupancy_grid.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        data = []
        for i in range(binary_image.shape[0]):
            for j in range(binary_image.shape[1]):
                if binary_image[i, j] == 255:
                    data.append(0)  # Free space
                else:
                    data.append(100)  # Occupied space

        occupancy_grid.data = data

        return occupancy_grid

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
