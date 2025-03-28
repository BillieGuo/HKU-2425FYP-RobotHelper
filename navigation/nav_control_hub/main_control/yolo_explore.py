import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String
from ultralytics import YOLOWorld
import numpy as np
import cv2
from cv_bridge import CvBridge

VISUALIZE = True

# RGBD Message
# std_msgs/Header header
# sensor_msgs/CameraInfo rgb_camera_info
# sensor_msgs/CameraInfo depth_camera_info
# sensor_msgs/Image rgb
# sensor_msgs/Image depth

class Yolo(Node):
    def __init__(self, camera_namespace="grasp_module", camera_name="D435i"):
        super().__init__('yolo_publisher')
        self.publisher_str = self.create_publisher(
            String, 
            'yolo_detections_str', 
            10)
        rgbd_topic = f"/{camera_namespace}/{camera_name}/rgbd"
        self.rs_rgbd_sub = self.create_subscription(
            Image,
            rgbd_topic,
            self.rgbd_callback,
            10)
        self.llm_sub = self.create_subscription(
            String,
            "llm2yolo",
            self.llm_callback,
            10)
        
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.prompt = None
        self.rgb_camera_info = None
        self.depth_camera_info = None
        
        self.model = YOLOWorld("yolov8x-worldv2.pt")
        self.model.info()
        self.model.to("cuda")
        
        
    def run(self):
        while rclpy.ok():    
            rclpy.spin_once(self)
            try:
                if not self.rgb_image or not self.depth_image:
                    continue
                self.process_lock = True
                
                # self.model.set_classes([self.prompt])
                self.model.set_classes(["person"])
                results = self.model.predict(self.rgb_image)

                detect_obj_dict = {}
                for box in results[0].boxes:
                    center_x, center_y, w, h = box.xywh.tolist()[0]
                    # Get the depth value at the center of the bounding box
                    depth_value = self.depth_image[int(center_y), int(center_x)]
                    detect_obj_dict[int(box.cls)] = {
                        "name": self.model.names[int(box.cls)],
                        "depth": depth_value
                    }
                self.detected_names = [obj["name"] for obj in detect_obj_dict.values()]
                detected_objects = []
                for box in results[0].boxes:
                    center_x, center_y, w, h = box.xywh.tolist()[0]
                    # Get the depth value at the center of the bounding box
                    depth_value = self.depth_image[int(center_y), int(center_x)]
                    detected_objects.append({
                        "class_id": int(box.cls),
                        "name": self.model.names[int(box.cls)],
                        "depth": depth_value,
                        "bbox": [center_x, center_y, w, h]
                    })
                self.ordered_detected_objects = sorted(detected_objects, key=lambda x: x["bbox"][0])  # Sort by the x-coordinate of the bounding box center (left to right)
                annotated_frame = results[0].plot()
                self.annotated_frame = annotated_frame 
                self.publish_detections()
                
                if VISUALIZE:
                    for obj in self.ordered_detected_objects:
                        cv2.putText(annotated_frame, f"{obj['depth']}mm", (int(obj["bbox"][0]), int(obj["bbox"][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    # Display the annotated frame
                    cv2.imshow("YOLO Inference", annotated_frame)
                    cv2.waitKey(1)
                
                self.process_lock = False
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                self.get_logger().info("Exiting...")
                self.destroy_node()


    def rgbd_callback(self, msg: RGBD):
        self.get_logger().info("RGBD images received\n")
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")
        if self.rgb_image is not None and self.depth_image.shape[:2] != self.rgb_image.shape[:2]:
            self.depth_image = cv2.resize(self.depth_image, (self.rgb_image.shape[1], self.rgb_image.shape[0]), interpolation=cv2.INTER_NEAREST)
        self.rgb_camera_info = msg.rgb_camera_info
        self.depth_camera_info = msg.depth_camera_info


    def llm_callback(self, msg):
        if msg.data:
            self.prompt = msg.data
        return
            
    def publish_detections(self):
        send_str = String()
        send_str.data = str(self.detected_names)
        self.publisher_str.publish(send_str)
        return


def main(args=None):
    rclpy.init(args=args)
    yolo_publisher = Yolo(camera_namespace="grasp_module", camera_name="D435i")
    yolo_publisher.run()
    # rclpy.spin(yolo_publisher)
    yolo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()