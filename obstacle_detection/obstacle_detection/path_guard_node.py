import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

class PathGuardNode(Node):
    def __init__(self):
        super().__init__('path_guard_node')
        self.bridge = CvBridge()
        
        # Ensure path is correct for your new workspace
        self.model = YOLO("/home/ravish/lane_ws/yolov8n.pt")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        # --- PERSPECTIVE DISTANCE THRESHOLDS ---
        self.RED_ZONE = 0.85    # Object at bottom 15% of screen
        self.ORANGE_ZONE = 0.65 # Object in lower-middle screen
        self.CENTER_WIDTH = 0.15 # 15% of width is the "collision path"

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.brake_pub = self.create_publisher(Bool, '/cmd_brake', 10)
        self.status_pub = self.create_publisher(String, '/path_status', 10)

        self.get_logger().info(f"PathGuard Active on {self.device}. Monitoring Perspective Zones.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        img_cx = w // 2

        # 1. Draw the "Collision Path" Guide (Cyan vertical lines)
        path_left = int(img_cx - (w * self.CENTER_WIDTH))
        path_right = int(img_cx + (w * self.CENTER_WIDTH))
        cv2.line(frame, (path_left, 0), (path_left, h), (255, 255, 0), 1)
        cv2.line(frame, (path_right, 0), (path_right, h), (255, 255, 0), 1)

        results = self.model(frame, verbose=False)
        brake_active = False
        current_status = "CLEAR"

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                label = self.model.names[int(box.cls[0])]
                obj_cx = (x1 + x2) // 2

                # 2. Check if object is directly in the car's path
                in_path = path_left < obj_cx < path_right
                
                color = (0, 255, 0) # Default: Green
                
                if in_path:
                    y_pos = y2 / h # Bottom of box relative to screen height
                    if y_pos > self.RED_ZONE:
                        color = (0, 0, 255) # RED: Collision Imminent
                        brake_active = True
                        current_status = "STOP"
                    elif y_pos > self.ORANGE_ZONE:
                        color = (0, 165, 255) # ORANGE: Warning
                        current_status = "CAUTION"

                # 3. Draw UI
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label} [{current_status}]", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 4. Action
        self.brake_pub.publish(Bool(data=brake_active))
        self.status_pub.publish(String(data=current_status))

        cv2.imshow("PathGuard Perspective View", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PathGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()