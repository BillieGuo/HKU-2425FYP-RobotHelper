import rclpy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Pose, Transform
from std_msgs.msg import String
from custom_msgs.msg import GraspResponse
import os
import yaml
import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum
from tf_transformations import quaternion_from_matrix, quaternion_matrix, rotation_matrix, inverse_matrix, translation_matrix
from ament_index_python.packages import get_package_share_directory
import time
import tkinter as tk  # Add this import

package_share_directory = get_package_share_directory("robotic_arm")

class ArmState(Enum):
    IDLE = 1
    CAPTURING = 2
    GRASPING = 3

class ArmManipulator(InterbotixManipulatorXS):
    def __init__(self):
        super().__init__(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            gripper_pressure_lower_limit=60,
            gripper_pressure_upper_limit=200
        )
        self.node = self.core.get_node()
        # tf
        self.c2g_tf_broadcaster = StaticTransformBroadcaster(self.node)
        self.camera_tf_broadcaster = StaticTransformBroadcaster(self.node)
        self.grasp_pose_tf_broadcaster = StaticTransformBroadcaster(self.node)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.broadcast_c2g_tf()

        # ros
        self.create_publisher()
        self.create_subscriber()

        # arm
        self.grasp_pressure = 1.0
        self.grasp()
        self.EXPLORE_VIEW_JOINTS = [0.0, -1.8, 1.36, 0.0, 0.7, 0.0]
        self.CAPTURE_VIEW_JOINTS = [0.0, -0.785, 0.785, 0, 0.785, 0]
        self.is_torque_on = True
        self.torque_on()
        self.go_to_explore_pose(blocking=True)
        self.state = ArmState.IDLE
        self.node.get_logger().info("ArmManipulator initialized")

        # GUI for key listening
        self.key_pressed = None
        self.create_gui()

    def create_gui(self):
        # Create a tkinter window for key listening
        self.root = tk.Tk()
        self.root.title("Arm Manipulator Key Listener")
        self.root.bind("<KeyPress>", self.on_key_press)

        # Add a label for instructions
        self.label = tk.Label(self.root, text="Press keys or buttons to control the arm.", font=("Arial", 14))
        self.label.pack(pady=10)

        # Add a frame for key descriptions
        self.key_frame = tk.Frame(self.root)
        self.key_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Add descriptions and buttons
        key_descriptions = [
            ("Go to sleep pose", "S"),
            ("Go to explore pose", "E"),
            ("Go to capture pose", "C"),
            ("Force release the gripper", "R"),
            ("Force grasp with the gripper", "G"),
            ("Torque toggle", "T"),
            ("Quit and shutdown the node", "Q"),
        ]
        # Add a status label to show the last key pressed
        self.status_label = tk.Label(self.root, text="Last key pressed: None", font=("Arial", 12), fg="blue")

        for description, key in key_descriptions:
            frame = tk.Frame(self.key_frame)
            frame.pack(fill=tk.X, padx=5, pady=5)
            # Add description label
            label = tk.Label(frame, text=description, font=("Arial", 12), anchor="w")
            label.pack(side=tk.LEFT, fill=tk.X, expand=True)
            # Add button for the action
            button = tk.Button(
                frame, 
                text=key, 
                font=("Arial", 12), 
                width=10,
                height=2,
                command=lambda k=key: self.handle_action(k.lower())
            )
            button.pack(side=tk.RIGHT)

        self.status_label.pack(pady=10)
        # Add a label to warn not to close the window
        self.warn_label = tk.Label(self.root, text="Do not close this window or node will die.", font=("Arial", 10), fg="red")
        self.warn_label.pack(pady=10)

    def on_key_press(self, event):
        self.handle_action(event.keysym)

    def handle_action(self, key):
        self.key_pressed = key
        self.status_label.config(text=f"Last key pressed: {self.key_pressed}")  # Update the status label
        if key == 'q':
            self.node.get_logger().info("Quitting...")
            self.go_to_sleep_pose()
            self.root.destroy()  # Close the tkinter window
            rclpy.shutdown()
        elif key == 's':
            self.node.get_logger().info("Go to sleep pose")
            self.go_to_sleep_pose()
        elif key == 'c':
            self.node.get_logger().info("Go to capture pose")
            self.go_to_capture_pose()
        elif key == 'e':
            self.node.get_logger().info("Go to explore pose")
            self.go_to_explore_pose()
        elif key == 'r':
            self.node.get_logger().info("Force release")
            self.release()
        elif key == 'g':
            self.node.get_logger().info("Force grasp")
            self.grasp()
        elif key == 't':
            self.node.get_logger().info("Torque toggle")
            self.torque_toggle()
        # Reset the key_pressed after handling
        self.key_pressed = None

    def broadcast_c2g_tf(self):
        tf_file_path =  os.path.join(package_share_directory, "config", "transform_camera2gripper.yaml")
        with open(tf_file_path, "r") as file:
            c2g_file = yaml.safe_load(file)
        self.c2g_matrix = np.array(c2g_file["camera2gripper"])
        c2g_tf = TransformStamped()
        c2g_tf.header.stamp = self.node.get_clock().now().to_msg()
        c2g_tf.header.frame_id = "vx300s/ee_gripper_link"
        c2g_tf.child_frame_id = "camera_frame"
        c2g_tf.transform.translation.x = self.c2g_matrix[0][3]
        c2g_tf.transform.translation.y = self.c2g_matrix[1][3]
        c2g_tf.transform.translation.z = self.c2g_matrix[2][3]
        c2g_tf.transform.rotation.x, c2g_tf.transform.rotation.y, c2g_tf.transform.rotation.z, c2g_tf.transform.rotation.w = quaternion_from_matrix(self.c2g_matrix)
        self.c2g_tf_broadcaster.sendTransform(c2g_tf)

    def broadcast_camera_tf(self):
        # the D435i_color_optical_frame is the same as camera_frame
        tf: TransformStamped = self.tf_buffer.lookup_transform(
            "D435i_color_optical_frame",
            "D435i_link",
            rclpy.time.Time()
        )
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = "camera_frame"
        self.camera_tf_broadcaster.sendTransform(tf)

    def create_publisher(self):
        # Publish for the grasp request node
        self.prompt_pub = self.node.create_publisher(String, "/text_prompt", 10)

    def create_subscriber(self):
        # translator's prompt
        self.prompt_sub = self.node.create_subscription(String, "/grasp_prompt", self.prompt_callback, 10)
        # grasp poses results
        self.grasp_sub = self.node.create_subscription(GraspResponse, "/robotic_arm/grasp_response", self.grasp_callback, 10)

    def prompt_callback(self, msg):
        # if self.state != ArmState.IDLE:
        #     self.node.get_logger().warn("Arm is not in IDLE state")
        #     return
        self.state = ArmState.CAPTURING
        # Receive the prompt from the LLM translator
        self.node.get_logger().info(f"Received prompt: {msg.data}")
        self.text_prompt = msg.data
        self.node.get_logger().info("Release the gripper")
        self.release()
        self.node.get_logger().info("Going to capture pose")
        self.go_to_capture_pose()
        time.sleep(5)
        self.prompt_pub.publish(msg)
        self.node.get_logger().info("Publish the prompt, waiting for the grasp pose")

    def grasp_callback(self, msg):
        # Receive the grasp pose from the server
        self.state = ArmState.GRASPING
        # self.node.get_logger().info(f"Received grasp pose: {msg}")
        self.num_grasp_poses = msg.num_grasp_poses
        self.grasp_poses = msg.grasp_poses
        self.scores = msg.scores
        # Use the best grasp pose
        target_pose = self.grasp_poses[0]
        self.node.get_logger().info(f"Using the best grasp pose: {target_pose}")
        # Calculate and publish the static tf from target to base
        t2c_matrix = self.pose_to_matrix(target_pose)
        c2g_matrix = self.c2g_matrix
        g2b_tf: TransformStamped = self.tf_buffer.lookup_transform(
            "vx300s/base_link",
            "vx300s/ee_gripper_link",
            rclpy.time.Time()
        )
        g2b_tf = g2b_tf.transform
        g2b_matrix = self.pose_to_matrix(g2b_tf)
        # g2b_matrix = self.arm.get_ee_pose()
        t2b_matrix = g2b_matrix @ c2g_matrix  @ t2c_matrix
        t2b_matrix = self.calibrate(t2b_matrix)
        grasp_pose_tf_msg = TransformStamped()
        grasp_pose_tf_msg.header.stamp = self.node.get_clock().now().to_msg()
        grasp_pose_tf_msg.header.frame_id = "vx300s/base_link"
        grasp_pose_tf_msg.child_frame_id = "grasp_pose"
        grasp_pose_tf_msg.transform.translation.x = t2b_matrix[0][3]
        grasp_pose_tf_msg.transform.translation.y = t2b_matrix[1][3]
        grasp_pose_tf_msg.transform.translation.z = t2b_matrix[2][3]
        x,y,z,w = quaternion_from_matrix(t2b_matrix) 
        grasp_pose_tf_msg.transform.rotation.x = x
        grasp_pose_tf_msg.transform.rotation.y = y
        grasp_pose_tf_msg.transform.rotation.z = z
        grasp_pose_tf_msg.transform.rotation.w = w
        self.grasp_pose_tf_broadcaster.sendTransform(grasp_pose_tf_msg)
        self.node.get_logger().info("Broadcast the grasp pose")
        # Move the arm to the grasp pose
        # Consider adding a waypoint
        # waypoint_matrix = t2b_matrix @ translation_matrix([-0.05, 0, 0])
        # self.arm.set_ee_pose_matrix(waypoint_matrix, moving_time=3.0)
        waypoint_matrix = t2b_matrix @ translation_matrix([-0.03, 0, 0])
        self.arm.set_ee_pose_matrix(waypoint_matrix, moving_time=5.0, blocking=True) #TODO: Add guess
        waypoint_matrix_2 = t2b_matrix @ translation_matrix([0.015, 0, 0])
        self.arm.set_ee_pose_matrix(waypoint_matrix_2, moving_time=2.0, blocking=True, custom_guess=self.arm.get_joint_positions())
        # self.arm.set_ee_pose_matrix(t2b_matrix, moving_time=2.0, blocking=True, custom_guess=self.arm.get_joint_positions())
        self.grasp(5.0)
        # self.go_to_capture_pose()
        self.go_to_explore_pose()
        self.state=ArmState.IDLE

    def calibrate(self, matrix):
        # Calibrate the orientation of the grasp pose
        # If the angle of the z-axes between 2 frames is negative, rotate the target around the x-axis by 180 degrees
        z_axis = matrix[:3, 2]
        if z_axis[2] < 0:
            rotation_180_x = rotation_matrix(np.pi, [1, 0, 0])
            matrix = matrix @ rotation_180_x
        return matrix

    def pose_to_matrix(self, pose: Pose | Transform):
        if isinstance(pose, Transform):
            translation = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
            rotation = np.array([pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w])
        else:
            translation = np.array([pose.position.x, pose.position.y, pose.position.z])
            rotation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        matrix = np.eye(4)
        matrix = quaternion_matrix(rotation)
        matrix[:3, 3] = translation
        return matrix

    def go_to_capture_pose(self, blocking=False):
        self.arm.set_joint_positions(self.CAPTURE_VIEW_JOINTS, moving_time=5.0, blocking=blocking)

    def go_to_explore_pose(self, blocking=False):
        self.arm.set_joint_positions(self.EXPLORE_VIEW_JOINTS, moving_time=5.0, blocking=blocking)

    def go_to_sleep_pose(self):
        self.arm.go_to_sleep_pose(blocking=False)

    def grasp(self, delay = 0.0):
        self.gripper.set_pressure(self.grasp_pressure)
        self.gripper.grasp(delay=delay)

    def release(self):
        self.gripper.set_pressure(0.0)
        self.gripper.release(delay=0.0)

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

    def run(self):
        try:
            self.broadcast_camera_tf()
        except Exception as e:
            self.node.get_logger().error(f"Error in broadcast_camera_tf:\n{e}")

        try:
            robot_startup()
            self.root.mainloop()  # Start the tkinter event loop
        except KeyboardInterrupt:
            robot_shutdown()

    def handle_state(self):
        if self.state == ArmState.IDLE:
            self.handle_idle()
        elif self.state == ArmState.CAPTURING:
            self.handle_capture()
        elif self.state == ArmState.GRASPING:
            self.handle_grasp()

    def handle_idle(self):
        # Idle logic
        pass

    def handle_capture(self):
        # Capturing logic
        # Wait for a specific command to transition to the next state
        pass

    def handle_grasp(self):
        # Grasping logic
        # Move the arm and the gripper to grasp an object
        pass

def main(args=None):
    rclpy.init(args=args)
    robotic_arm = ArmManipulator()
    robotic_arm.run()

if __name__ == "__main__":
    main()