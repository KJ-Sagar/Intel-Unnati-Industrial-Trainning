import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Int32MultiArray, 'occupancy_grid', 10)
        self.bridge = CvBridge()
        self.net = cv2.dnn.readNet('yolov3.weights', 'yolov3.cfg')
        self.layer_names = self.net.getLayerNames()
        unconnected_out_layers = self.net.getUnconnectedOutLayers()

        if isinstance(unconnected_out_layers, list):
            self.output_layers = [self.layer_names[i[0] - 1] for i in unconnected_out_layers]
        else:
            self.output_layers = [self.layer_names[unconnected_out_layers - 1]]
        
        self.occupancy_grid = np.zeros((500, 500), dtype=np.int32)  # Initialize your occupancy grid

    def detect_objects(self, image):
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        detections = self.net.forward(self.output_layers)
        return detections

    def update_occupancy_grid(self, detections):
        for detection in detections:
            for obj in detection:
                if obj[5] > 0.5:  # Confidence threshold
                    x, y, w, h = obj[0:4] * 500  # Adjust according to grid size
                    self.occupancy_grid[int(y):int(y + h), int(x):int(x + w)] = 1

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detections = self.detect_objects(cv_image)
        self.update_occupancy_grid(detections)
        grid_msg = Int32MultiArray(data=self.occupancy_grid.flatten().tolist())
        self.publisher_.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
