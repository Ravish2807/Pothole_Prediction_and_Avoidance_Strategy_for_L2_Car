import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import torch


class UnityPotholeDetectionNode(Node):

    def __init__(self):
        super().__init__('unity_pothole_detection_node')

        self.bridge = CvBridge()

        # Load trained YOLO model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = YOLO("/home/ravish/lane_ws/pothole.pt")
        self.model.to(self.device)

        # Force the OpenCV tab to open immediately
        cv2.namedWindow("Unity Sim Pothole View", cv2.WINDOW_AUTOSIZE)
        self.first_frame_received = False

        # Subscribe to camera from Unity
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw_PD',
            self.image_callback,
            10
        )

        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/pothole_image',
            10
        )

        self.center_pub = self.create_publisher(
            Point,
            '/pothole_center',
            10
        )

        self.get_logger().info("Unity Pothole Detection Node Started. Waiting for camera feed from Unity...")

    def image_callback(self, msg):
        if not self.first_frame_received:
            self.get_logger().info("Success: First frame received from Unity!")
            self.first_frame_received = True

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(frame, verbose=False)

        for r in results:
            boxes = r.boxes

            for box in boxes:
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                # Only detect pothole class (usually class 0)
                if conf > 0.5:

                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2),
                                  (0, 0, 255), 2)

                    label = f"Pothole {conf:.2f}"
                    cv2.putText(frame, label,
                                (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 0, 255), 2)

                    # Compute center
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    point = Point()
                    point.x = float(center_x)
                    point.y = float(center_y)
                    point.z = 0.0

                    self.center_pub.publish(point)

        # Publish annotated image
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.image_pub.publish(img_msg)

        # Display the frame
        cv2.imshow("Unity Sim Pothole View", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = UnityPotholeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
