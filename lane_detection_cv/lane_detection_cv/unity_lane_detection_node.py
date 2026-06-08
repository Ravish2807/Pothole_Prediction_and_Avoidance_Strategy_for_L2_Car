import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# Import lane detector from cv module
from lane_detection_cv.lane_detector import LaneDetector


class UnityLaneDetectionNode(Node):

    def __init__(self):
        super().__init__('unity_lane_detection_node')

        self.bridge = CvBridge()
        self.detector = LaneDetector()

        # Force the OpenCV tab to open immediately
        cv2.namedWindow("Unity Sim Lane View", cv2.WINDOW_AUTOSIZE)
        self.first_frame_received = False

        # Subscribe to the raw camera topic from Unity via DDS
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw_LD',
            self.image_callback,
            10
        )

        # Publishers for annotated lane image and simple center point
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

        self.get_logger().info("Unity lane detection node started via Cyclone DDS. Waiting for raw camera feed...")

    def image_callback(self, msg):
        if not self.first_frame_received:
            self.get_logger().info("Success: First raw frame received from Unity via DDS!")
            self.first_frame_received = True

        # Convert raw ROS image -> OpenCV image (bgr8)
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # run lane detection
        output = self.detector.detect_lanes(frame)

        # publish processed image
        img_msg = self.bridge.cv2_to_imgmsg(output, 'bgr8')
        self.image_pub.publish(img_msg)

        # simple center calculation (middle of image bottom)
        height, width = frame.shape[:2]
        center_x = width // 2
        point = Point()
        point.x = float(center_x)
        point.y = float(height)
        point.z = 0.0
        self.center_pub.publish(point)

        # display window
        cv2.imshow("Unity Sim Lane View", output)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = UnityLaneDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
