import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose,BoundingBox2D, Pose2D
from cv_bridge import CvBridge

from ultralytics import YOLO
import torch


class YoloObstacleNode(Node):

    def __init__(self):
        super().__init__('yolo_obstacle_node')

        self.bridge = CvBridge()

        # Load YOLOv8 model (auto-downloads if not present)
        self.model = YOLO("/home/ravish/ros2_ws/yolov8n.pt")

        # Optional: use GPU if available
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish obstacle detections
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/obstacles',
            10
        )

        # Define which classes count as obstacles
        # This automatically grabs all 80 names from the YOLO model
        self.obstacle_classes = set(self.model.names.values())
        self.get_logger().info("YOLO Obstacle Node Started")

    def image_callback(self, msg):

        # Convert ROS image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference (no gradient tracking for speed)
        with torch.no_grad():
            results = self.model(frame)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for result in results:
            boxes = result.boxes

            if boxes is None:
                continue

            for box in boxes:

                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                # Filter only obstacle classes
                if class_name not in self.obstacle_classes:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # --- START OF ADDED VISUALIZATION ---
                # Draw the bounding box (Green)
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # Draw the label and confidence score
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(frame, label, (int(x1), int(y1) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # --- END OF ADDED VISUALIZATION ---

                det = Detection2D()

                # Bounding box center + size
                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = confidence

                det.results.append(hypothesis)
                detection_array.detections.append(det)
        
        cv2.imshow("YOLO Obstacle Detection View", frame)
        cv2.waitKey(1) # Necessary to refresh the window

        self.publisher.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = YoloObstacleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
