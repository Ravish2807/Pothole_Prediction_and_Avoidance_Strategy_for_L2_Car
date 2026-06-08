import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2

class SafetyMuxNode(Node):
    def __init__(self):
        super().__init__('safety_mux_node')
        self.bridge = CvBridge()

        # 1. Subscribers
        self.create_subscription(Twist, '/teleop_cmd_vel', self.teleop_callback, 10)
        self.create_subscription(Bool, '/cmd_brake', self.brake_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        # 2. Publisher to Jetson Nano Motors
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State Variables
        self.brake_active = False
        self.latest_teleop = Twist()
        self.get_logger().info("Safety Mux Visualizer Node Started.")

    def brake_callback(self, msg):
        self.brake_active = msg.data

    def teleop_callback(self, msg):
        self.latest_teleop = msg
        # Only forward if path is clear
        if not self.brake_active:
            self.cmd_pub.publish(msg)
        else:
            # Force stop if brake is active
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)

    def image_callback(self, msg):
        # Convert to OpenCV to draw the status
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # --- DRAW THE CONTROL HUD ---
        # Background bar for status
        overlay_color = (0, 200, 0) if not self.brake_active else (0, 0, 255)
        status_text = "CONTROL: MANUAL (FREE)" if not self.brake_active else "CONTROL: AUTO-BRAKE (LOCKED)"
        
        cv2.rectangle(frame, (0, 0), (w, 60), overlay_color, -1)
        cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 2)

        # Display current speed commands being sent to Jetson
        speed_text = f"Linear X: {self.latest_teleop.linear.x:.2f} | Angular Z: {self.latest_teleop.angular.z:.2f}"
        cv2.putText(frame, speed_text, (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        if self.brake_active:
            # Draw a big warning icon/text in the middle
            cv2.putText(frame, "STOP", (w//2 - 100, h//2), cv2.FONT_HERSHEY_BOLD, 3.0, (0, 0, 255), 10)

        cv2.imshow("Robot Master Control View", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()