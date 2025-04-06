import rclpy
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ultralytics import YOLO
import numpy as np
import cv2
# from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Parameters
VISUALIZE = True
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_THRESHOLD = 100
DISTANCE_THRESHOLD = 600 # unit: mm (align with the depth image)

# RGBD Message
# std_msgs/Header header
# sensor_msgs/CameraInfo rgb_camera_info
# sensor_msgs/CameraInfo depth_camera_info
# sensor_msgs/Image rgb
# sensor_msgs/Image depth

class Yolo(Node):
    def __init__(self, camera_namespace="grasp_module", camera_name="D435i"):
        super().__init__('yolo_publisher')
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher_str = self.create_publisher(
            String, 
            'yolo_detections_str', 
            10)
        self.yolo2navigator_pub = self.create_publisher(
            String,
            'yolo2navigator',
            qos_profile)
        self.yolo2ser_pub = self.create_publisher(
            Twist,
            'yolo2ser',
            10)
        
        rgbd_topic = f"/{camera_namespace}/{camera_name}/rgbd"
        self.rs_rgbd_sub = self.create_subscription(
            RGBD,
            rgbd_topic,
            self.rgbd_callback,
            10)
        self.navigator2yolo_sub = self.create_subscription(
            String,
            "navigator2yolo",
            self.navigator_request_callback,
            qos_profile)
        
        # self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.prompt = None
        self.rgb_camera_info = None
        self.depth_camera_info = None
        
        model_path = "./src/HKU-2425FYP-RobotHelper/navigation/nav_control_hub/models/" 
        self.model = YOLO(model_path+"yolov8x-worldv2.pt")
        # self.model = YOLO(model_path+"yoloe-11l-seg.pt")
        # self.model = YOLO(model="yoloe-s.pt")
        self.model.info()
        self.model.to("cuda")
        self.distance_threshold = DISTANCE_THRESHOLD
        
        self.get_logger().info("Yolo explore node initialized.")
        
    def rgbd_callback(self, msg: RGBD):
        # self.get_logger().info("RGBD images received\n")
        bgr_img = np.frombuffer(msg.rgb.data, dtype=np.uint8).reshape(msg.rgb.height, msg.rgb.width, -1)
        self.rgb_image = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB) 
        self.depth_image = np.frombuffer(msg.depth.data, dtype=np.uint16).reshape(msg.depth.height, msg.depth.width, -1)
        self.rgb_camera_info = msg.rgb_camera_info
        self.depth_camera_info = msg.depth_camera_info

    def navigator_request_callback(self, msg):
        if msg.data:
            self.prompt = msg.data
        self.get_logger().info(f"Received prompt: {self.prompt}")
        return
        
    def pub_chassis_control(self, cmd):
        vel = Twist()
        linear = 0.0
        angular = 0.0

        if cmd == "Stop":
            linear = 0.0
            angular = 0.0
        elif cmd == "Start" or cmd == "CCW":
            linear = 0.0
            angular = 0.18 # CCW
        elif cmd == "CW":
            linear = 0.0
            angular = -0.18
        elif cmd == "Forward":
            linear = 0.14
            angular = 0.0

        vel.linear.x = linear
        vel.angular.z = angular
        self.yolo2ser_pub.publish(vel)
        return
        
    def pub_to_navigator(self, msg):
        # Publish the message to the navigator
        self.get_logger().info(f"Publishing to navigator: {msg}")
        tx = String()
        tx.data = str(msg)
        self.yolo2navigator_pub.publish(tx)
        return

    def run(self):
        while rclpy.ok():    
            rclpy.spin_once(self)
            try:
                if self.prompt is None:
                    continue
            # 1. Received Prompt, start exploring
                self.get_logger().info(f"Prompt {self.prompt} received, navigation done, start exploring the object.")
            # 2. Rotate Explore Movement (Send a command)
                is_detected = False

                distance = 1000
                while self.exploration_not_complete(distance):
                    # Use yolo to detect the target object
                    is_detected, detect_result = self.detect_object(self.prompt)

                    if not is_detected:
                        self.send_explore_command()
                        self.get_logger().info("Exploring, rotating.")
                        continue

                    if not self.object_fix_to_camera_center(detect_result):
                        self.get_logger().info("Object not in center, rotating.")
                        continue

                    # get distance from detect_result with simple filter
                    if detect_result["distance"] > 100:
                        distance = distance * 0.9 + detect_result["distance"] * 0.1
                    self.get_logger().info(f"Distance: {distance}")
                    # Move the car to the object
                    self.send_foward_command()
                    
                # Complete exploring, stop moving
                self.get_logger().info("Exploration complete, stopping.")
                result = "Success"
                self.pub_to_navigator(result)
                self.send_stop_command()
                self.prompt = None
                
            except KeyboardInterrupt:
                cv2.destroyAllWindows()
                self.get_logger().info("Exiting...")
                self.destroy_node()

    def send_explore_command(self):
        # Send a command to let the car rotate slowly (Use another program to control the car)
        self.get_logger().info("Sending explore command: rotating slowly.")
        self.pub_chassis_control("Start")

    def send_stop_command(self):
        # Send a command to stop the car from moving
        self.get_logger().info("Sending stop command: stopping movement.")
        self.pub_chassis_control("Stop")
        
        
    def send_foward_command(self):
        # Move forward
        self.get_logger().info("Start to move closer")
        self.pub_chassis_control("Forward")
        pass

    def detect_object(self, target_class=None):
        # Use YOLO to detect objects in the RGB image
        rclpy.spin_once(self)
        if self.rgb_image is None or self.depth_image is None:
            return False, None

        # set class
        self.model.set_classes([f'{target_class}'])
        results = self.model.predict(self.rgb_image)
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

        if VISUALIZE:
            annotated_frame = results[0].plot()
            if annotated_frame is not None:
                cv2.imshow("Annotated Frame", annotated_frame)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("Annotated frame is None, skipping visualization.")

        if detected_objects:
            # Filter objects by target class if specified
            if target_class:
                detected_targets = [obj for obj in detected_objects if obj["name"] == target_class]
                self.get_logger().info(f"Detected objects: {detected_targets}")
            if detected_targets:
                # Sort detected objects by depth (closest first)
                ordered_detected_targets = sorted(detected_objects, key=lambda x: x["depth"])
                nearest_object = ordered_detected_targets[0]
                annotated_frame = results[0].plot()

                detect_result = {
                    "detected_targets": ordered_detected_targets,
                    "nearest_object": nearest_object,
                    "distance": nearest_object["depth"],
                    "bbox": nearest_object["bbox"],
                    "annotated_frame": annotated_frame
                }
                return True, detect_result
            
        return False, None

    def object_fix_to_camera_center(self, detection):
        bbox = detection["bbox"]
        center_x = bbox[0]
        center_y = bbox[1]

        # Check if the object is in the center of the image
        if abs(center_x - CAMERA_WIDTH / 2) < CAMERA_THRESHOLD:
            # Object is in the center, return true, stop moving
            return True
        # Object is not in the center, return false, and rotate accordingly
        if center_x < CAMERA_WIDTH / 2:
            self.pub_chassis_control("CCW")
        elif center_x > CAMERA_WIDTH / 2:
            self.pub_chassis_control("CW")

        return False

    def exploration_not_complete(self, distance):
        # Add logic to determine if exploration is complete
        if distance < self.distance_threshold:
            return False
        # If the robot cannot move further, return true
        return True


def main(args=None):
    rclpy.init(args=args)
    yolo_publisher = Yolo(camera_namespace="grasp_module", camera_name="D435i")
    yolo_publisher.run()
    # rclpy.spin(yolo_publisher)
    yolo_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()