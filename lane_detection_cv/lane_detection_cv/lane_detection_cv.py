import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cv2
import numpy as np

# Import your module
from lane_detection_cv.lane_detector import LaneDetector


class LaneDetectionNode(Node):

    def __init__(self):
        super().__init__('lane_detection_node')

        self.bridge = CvBridge()

        # Create LaneDetector object from your module
        self.detector = LaneDetector()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/lane_image',
            10
        )

        self.center_pub = self.create_publisher(
            Point,
            '/lane_center',
            10
        )

        self.get_logger().info("Lane Detection Node Started")

    def image_callback(self, msg):

        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Use your module
        output = self.detector.detect_lanes(frame)

        # Publish processed image
        img_msg = self.bridge.cv2_to_imgmsg(output, 'bgr8')
        self.image_pub.publish(img_msg)

        # Compute simple lane center (image center for now)
        height, width = frame.shape[:2]
        center_x = width // 2

        point = Point()
        point.x = float(center_x)
        point.y = float(height)
        point.z = 0.0

        self.center_pub.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
