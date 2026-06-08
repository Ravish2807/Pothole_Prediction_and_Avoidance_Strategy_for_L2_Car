import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import message_filters

class AutonomousDriver(Node):
    def __init__(self):
        super().__init__('autonomous_driver')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")
        
        # Publisher to the Jetson Nano
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Syncing Camera Topics
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.navigate)

        self.get_logger().info("Autonomous Depth-Safety Node Active")

    def navigate(self, rgb_msg, depth_msg):
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        w = depth_frame.shape[1]
        
        # Run YOLO
        results = self.model(self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8'), verbose=False)
        
        move_cmd = Twist()
        move_cmd.linear.x = 0.25 # Default cruising speed
        status = "GREEN: MOVING"

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist_m = depth_frame[cy, cx] / 1000.0
                
                if dist_m == 0: continue

                # RED ZONE: STOP
                if (w*0.4 < cx < w*0.6) and dist_m < 0.7:
                    move_cmd.linear.x = 0.0
                    status = "RED: STOPPING"
                    break
                
                # ORANGE ZONE: CAUTION (Slow and Turn)
                elif (w*0.2 < cx < w*0.8) and dist_m < 1.8:
                    move_cmd.linear.x = 0.1
                    move_cmd.angular.z = 0.3 # Turn away
                    status = "ORANGE: CAUTION"

        self.cmd_pub.publish(move_cmd)
        print(f"Robot Status: {status}")

def main():
    rclpy.init()
    rclpy.spin(AutonomousDriver())