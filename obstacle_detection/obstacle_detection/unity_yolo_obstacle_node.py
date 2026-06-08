import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from ultralytics import YOLO
import torch


class UnityYoloObstacleNode(Node):

    def __init__(self):
        super().__init__('unity_yolo_obstacle_node')

        self.bridge = CvBridge()

        # Load YOLO model 
        self.model = YOLO("/home/ravish/ros2_ws/yolov8n.pt")

        # Automatically uses your RTX GPU if available
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        # Force the OpenCV tab to open immediately
        cv2.namedWindow("Unity Sim YOLO View", cv2.WINDOW_AUTOSIZE)
        self.first_frame_received = False

        # Subscribe to the raw camera topic from Unity via DDS
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw_OD',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Detection2DArray,
            '/obstacles',
            10
        )

        self.obstacle_classes = set(self.model.names.values())
        self.get_logger().info("Unity YOLO Node Started via Cyclone DDS. Waiting for raw camera feed...")

    def image_callback(self, msg):
        if not self.first_frame_received:
            self.get_logger().info("Success: First raw frame received from Unity via DDS!")
            self.first_frame_received = True

        # Convert raw ROS image -> OpenCV image (Handles the rgb8 to bgr8 conversion natively)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


        with torch.no_grad():
            results = self.model(frame)

        detection_array = Detection2DArray()
        detection_array.header = msg.header
        annotated_frame = frame 

        for result in results:
            annotated_frame = result.plot() 
            
            boxes = result.boxes
            if boxes is None:
                continue

            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])

                if class_name not in self.obstacle_classes:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                det = Detection2D()
                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = class_name
                hypothesis.hypothesis.score = confidence

                det.results.append(hypothesis)
                detection_array.detections.append(det)
        
        cv2.imshow("Unity Sim YOLO View", annotated_frame)
        cv2.waitKey(1) 

        self.publisher.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = UnityYoloObstacleNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows() 
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()