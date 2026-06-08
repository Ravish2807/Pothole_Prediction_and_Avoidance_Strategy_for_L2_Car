import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import torch

class PotholeNode(Node):

    def __init__(self):
        super().__init__('pothole_detection_node')
        self.bridge = CvBridge()

        # Load trained YOLO model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = YOLO("/home/ravish/lane_ws/pothole.pt")
        self.model.to(self.device)

        # 1. TOPIC CHECK: Change this to match your DroidCam ROS2 topic exactly
        # Common ones are '/image_raw' or '/camera/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(Image, '/pothole_image', 10)
        self.center_pub = self.create_publisher(Point, '/pothole_center', 10)

        # Force the OpenCV window to initialize
        cv2.namedWindow("Phone Pothole View", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("DroidCam Pothole Node Started. Watching for Phone feed...")

    def image_callback(self, msg):
        # Convert ROS image from Phone to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run Pothole Inference
        results = self.model(frame, verbose=False)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                conf = float(box.conf[0])
                if conf > 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # Draw Red Box for Pothole
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(frame, f"Pothole {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # Publish Center Point for control logic
                    point = Point()
                    point.x = float((x1 + x2) / 2)
                    point.y = float((y1 + y2) / 2)
                    self.center_pub.publish(point)

        # --- THE FIX: OPEN THE TAB ---
        # This draws the frame and tells Ubuntu to show the window
        cv2.imshow("Phone Pothole View", frame)
        cv2.waitKey(1) 

        # Optional: Publish the annotated view back to ROS
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PotholeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
