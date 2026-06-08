import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('point_cloud_visualizer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pothole_3d_map',
            self.pc_callback,
            10
        )
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Pothole 3D Map')
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        self.first_time = True

    def pc_callback(self, msg):
        # Convert PointCloud2 to numpy array
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if len(points) == 0:
            self.get_logger().warn("Received empty point cloud")
            return
        
        # Update the point cloud
        self.pcd.points = o3d.utility.Vector3dVector(points)
        
        if self.first_time:
            self.vis.reset_view_point(True)
            self.first_time = False
        
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        
        self.get_logger().info(f"Visualized point cloud with {len(points)} points")

def main():
    rclpy.init()
    node = PointCloudVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vis.destroy_window()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()