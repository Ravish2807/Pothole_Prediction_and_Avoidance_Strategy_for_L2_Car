import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import std_msgs.msg

class PotholeMapper(Node):
    def __init__(self):
        super().__init__('pothole_mapper_node')
        self.bridge = CvBridge()
        
        # Subscriptions
        self.create_subscription(Point, '/pothole_center', self.center_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # Publisher for the 3D Point Cloud
        self.pc_pub = self.create_publisher(PointCloud2, '/pothole_3d_map', 10)
        
        self.latest_depth = None
        # D435i Intrinsics (Typical values, can be fetched from /camera/camera/color/camera_info)
        self.fx, self.fy = 600.0, 600.0 
        self.cx, self.cy = 320.0, 240.0

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.get_logger().info("Received depth image")
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def center_callback(self, msg):
        self.get_logger().info(f"Received pothole center: x={msg.x}, y={msg.y}")
        if self.latest_depth is None:
            self.get_logger().warn("No depth image available yet")
            return

        # Define a small region around the center to map the "hollow"
        u_center, v_center = int(msg.x), int(msg.y)
        roi_size = 20  # Pixels around the center to sample
        
        points = []
        for v in range(v_center - roi_size, v_center + roi_size):
            for u in range(u_center - roi_size, u_center + roi_size):
                if 0 <= v < self.latest_depth.shape[0] and 0 <= u < self.latest_depth.shape[1]:
                    depth_mm = self.latest_depth[v, u]
                    if depth_mm > 0:
                        # Convert to Meters
                        z = depth_mm / 1000.0
                        x = (u - self.cx) * z / self.fx
                        y = (v - self.cy) * z / self.fy
                        points.append([x, y, z])

        # Create PointCloud2 Message
        if points:
            header = std_msgs.msg.Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_link' # Or 'odom' if you have SLAM running
            
            pc_msg = pc2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(pc_msg)
            self.get_logger().info(f"Published point cloud with {len(points)} points")
        else:
            self.get_logger().warn("No valid points found in ROI")

def main():
    rclpy.init()
    node = PotholeMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()