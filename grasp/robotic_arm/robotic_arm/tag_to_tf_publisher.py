import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from robotic_arm.visual_module import VisualModule
import cv2
from tf_transformations import quaternion_from_matrix
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import yaml, os
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup


class Robot(InterbotixManipulatorXS):
    def __init__(self, robot_model, group_name, gripper_name):
        super().__init__(robot_model=robot_model, group_name=group_name, gripper_name=gripper_name)
        self.HIGH_ANGLE = [0, -1.1, 0.4, 0, 1.6, 0]
        self.LOOK_DOWN = [-2.58, -0.35, 0.53, -0.29, 1.32, 0.64]
        self.is_torque_on = True
        self.torque_on()

    def go_to_initial_pose(self):
        self.arm.set_joint_positions(self.HIGH_ANGLE, moving_time=5.0, blocking=False)

    def look_down(self):
        self.arm.set_joint_positions(self.LOOK_DOWN, moving_time=5.0, blocking=False)

    def torque_off(self):
        self.core.robot_torque_enable(cmd_type='group', name='arm', enable=False)
        self.is_torque_on = False

    def torque_on(self):
        self.core.robot_torque_enable(cmd_type='group', name='arm', enable=True)
        self.is_torque_on = True

    def torque_toggle(self):
        if self.is_torque_on:
            self.torque_off()
        else:
            self.torque_on()

class TagToTFPublisher(Node):
    def __init__(self):
        super().__init__("tag_to_tf_publisher")
        self.visual_module = VisualModule(tag_size=0.03)
        self.visual_module.initialize_camera()
        self.visual_module.initialize_detector()
        self.broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.get_logger().info("Press Ctrl+C to stop the node.")
        self.robot = Robot("vx300s", "arm", "gripper")  # Use the new Robot class
        robot_startup()
        self.c2g_mat = self.load_transform()
        self.color_image = None
        self.tags = []

    def load_transform(self):
        config_dir = os.path.expanduser("~/fyp_ws/src/HKU-2425FYP-RobotHelper/grasp/robotic_arm/config")
        transform_file = os.path.join(config_dir, "transform_camera2gripper.yaml")
        with open(transform_file, "r") as file:
            camera2gripper = yaml.safe_load(file)
        camera2gripper = np.array(camera2gripper["camera2gripper"])
        return camera2gripper

    def publish_static_transform(self, t2c_mat, tag_id, g2b_mat, c2g_mat):
        # Debugging: Log input matrices
        self.get_logger().info(f"t2c_mat: {t2c_mat}")
        self.get_logger().info(f"g2b_mat: {g2b_mat}")
        self.get_logger().info(f"c2g_mat: {c2g_mat}")

        # Publish a static transform from the target tag to the robot arm base
        t2b_mat = g2b_mat @ c2g_mat @ t2c_mat
        self.get_logger().info(f"t2b_mat: {t2b_mat}")

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "/vx300s/base_link"
        transform.child_frame_id = f"tag_{tag_id}"

        transform.transform.translation.x = t2b_mat[0, 3]
        transform.transform.translation.y = t2b_mat[1, 3]
        transform.transform.translation.z = t2b_mat[2, 3]

        rotation_matrix = t2b_mat[:3, :3]
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.broadcaster.sendTransform(transform)

    def rotation_matrix_to_quaternion(self, rotation_matrix):
        # Convert a 3x3 rotation matrix to a quaternion
        m = np.eye(4)
        m[:3, :3] = rotation_matrix
        return quaternion_from_matrix(m)

    def compute_average_pose(self, samples):
        translations = []
        rotations = []

        for t2c_mat in samples:
            translations.append(t2c_mat[:3, 3])
            rotation_matrix = t2c_mat[:3, :3]
            quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)
            rotations.append(quaternion)

        avg_translation = np.mean(translations, axis=0)
        avg_rotation = np.mean(rotations, axis=0)
        avg_rotation /= np.linalg.norm(avg_rotation)  # Normalize quaternion

        avg_t2c_mat = np.eye(4)
        avg_t2c_mat[:3, 3] = avg_translation
        avg_t2c_mat[:3, :3] = quaternion_from_matrix(avg_rotation)

        return avg_t2c_mat

    def timer_callback(self):
        self.color_image = self.visual_module.get_color_image()
        if self.color_image is None:
            self.get_logger().warn("No color image received.")
            return

        self.tags = self.visual_module.detect_tags(self.color_image)
        if self.tags:
            tag = self.tags[0]
            if tag.pose_t is not None:
                self.visual_module.draw_tag_info(self.color_image, tag)

        cv2.imshow("AprilTag Detection", self.color_image)
        key = cv2.waitKey(1) & 0xFF  # Capture key press

        # Key press controls
        if key == ord("q"):  # Quit
            cv2.destroyAllWindows()
            self.visual_module.stop_camera()
            self.robot.arm.go_to_sleep_pose(moving_time=5.0, blocking=False)
            robot_shutdown()
        elif key == ord("t"):  # Toggle torque
            self.robot.torque_toggle()
        elif key == ord("h"):  # Go to initial pose
            self.robot.go_to_initial_pose()
        elif key == ord("l"):  # Look down
            self.robot.look_down()
        elif key == ord("\r") or key == ord("\n"):  # Enter key
            self.capture_and_publish_pose()

    def capture_and_publish_pose(self):
        if not self.tags:
            self.get_logger().warn("No AprilTag detected.")
            return

        tag = self.tags[0]
        if tag.pose_t is not None:
            t2c_mat = np.eye(4)
            t2c_mat[:3, :3] = tag.pose_R
            t2c_mat[:3, 3] = tag.pose_t.flatten()

            g2b_mat = self.robot.arm.get_ee_pose()
            self.publish_static_transform(t2c_mat, tag.tag_id, g2b_mat, self.c2g_mat)
            self.get_logger().info("Published transform for detected tag.")

def main(args=None):
    rclpy.init(args=args)
    node = TagToTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
