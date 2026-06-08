import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import torch
import torch.nn as nn
import cv2
import numpy as np

# Note: Ensure your ParsingNet class from the previous code is included here!

class LaneGuideNode(Node):
    def __init__(self):
        super().__init__('lane_guide_node')
        self.bridge = CvBridge()
        
        # Setup Model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # self.model = ParsingNet().to(self.device) ... Load UFLD weights here

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.point_pub = self.create_publisher(Point, '/lane_center_point', 10)

        self.get_logger().info(f"LaneGuide started on {self.device}.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w, _ = frame.shape
        
        # --- PATH FINDING LOGIC ---
        # Assume lane_center_x is found via UFLD inference
        # This is the target horizontal coordinate for steering
        target_x = w // 2 # Placeholder: Replace with actual UFLD math

        # 1. Draw the "Target Steering Line" (Yellow)
        cv2.line(frame, (target_x, 0), (target_x, h), (0, 255, 255), 2)
        
        # 2. Draw Distance Threshold Markers (Perspective lines)
        cv2.line(frame, (0, int(h*0.85)), (w, int(h*0.85)), (0, 0, 255), 1) # Stop line
        cv2.line(frame, (0, int(h*0.65)), (w, int(h*0.65)), (0, 165, 255), 1) # Warning line

        # 3. Publish Data
        p = Point()
        p.x = float(target_x)
        self.point_pub.publish(p)

        cv2.imshow("LaneGuide Steering View", frame)
        cv2.waitKey(1)