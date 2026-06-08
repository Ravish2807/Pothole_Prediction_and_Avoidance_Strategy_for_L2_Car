import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch

class SafetyFusionNode(Node):
    def __init__(self):
        super().__init__('safety_fusion_node')
        self.bridge = CvBridge()
        
        # Load YOLO
        self.model = YOLO("/home/ravish/lane_ws/yolov8n.pt")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        # Distance Thresholds
        self.RED_THRESHOLD = 0.6    # Stop Immediately
        self.ORANGE_THRESHOLD = 1.5 # Caution Warning

        self.last_depth_frame = None
        self.latest_teleop = Twist() 

        # --- YOUR EXACT CAMERA TOPICS ---
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # The virtual "waiting room" for your keyboard commands
        self.create_subscription(Twist, '/teleop_cmd_vel', self.teleop_callback, 10)

        # The actual motor topic going to the Jetson Nano
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Safety Fusion Node Active. Waiting for Camera Data...")

    def depth_callback(self, msg):
        self.last_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def teleop_callback(self, msg):
        # Always save the latest keyboard command
        self.latest_teleop = msg

    def image_callback(self, msg):
        if self.last_depth_frame is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        
        results = self.model(frame, verbose=False)
        emergency_stop = False

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                label = self.model.names[int(box.cls[0])]

                # --- EXTRACT DEPTH ---
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                
                if 0 <= cx < w and 0 <= cy < h:
                    depth_patch = self.last_depth_frame[max(0, cy-5):min(h, cy+5), max(0, cx-5):min(w, cx+5)]
                    valid_depths = depth_patch[depth_patch > 0]
                    
                    if valid_depths.size > 0:
                        dist_m = np.median(valid_depths) / 1000.0  
                    else:
                        dist_m = 99.0  

                    # --- COLOR CLASSIFICATION ---
                    if dist_m < self.RED_THRESHOLD:
                        box_color = (0, 0, 255)  # RED
                        emergency_stop = True    
                    elif dist_m < self.ORANGE_THRESHOLD:
                        box_color = (0, 165, 255)  # ORANGE
                    else:
                        box_color = (0, 255, 0)  # GREEN

                    # --- DRAW BOX & DISTANCE ---
                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                    text = f"{label}: {dist_m:.2f}m"
                    cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

        # --- MOTOR DECISION ---
        final_cmd = Twist()
        if emergency_stop:
            # OBSTACLE DETECTED: Force stop, ignore teleop
            final_cmd.linear.x = 0.0
            final_cmd.angular.z = 0.0
            cv2.putText(frame, "EMERGENCY BRAKE ACTIVE", (50, 50), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 255), 3)
        else:
            # PATH CLEAR: Pass your manual keyboard commands to the motors
            final_cmd = self.latest_teleop
            cv2.putText(frame, "MANUAL CONTROL OK", (50, 50), cv2.FONT_HERSHEY_DUPLEX, 1.0, (0, 255, 0), 3)

        self.cmd_pub.publish(final_cmd)

        cv2.imshow("Obstacle Depth & Control View", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyFusionNode()
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