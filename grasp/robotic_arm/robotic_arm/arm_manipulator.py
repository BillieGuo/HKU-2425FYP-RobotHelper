import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Pose, Transform
from std_msgs.msg import String, Bool
from custom_msgs.msg import GraspResponse
import os, sys
import yaml
import numpy as np
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from enum import Enum
from tf_transformations import quaternion_from_matrix, quaternion_matrix, rotation_matrix, inverse_matrix, translation_matrix, euler_from_matrix
from ament_index_python.packages import get_package_share_directory
import tkinter as tk
import threading

package_share_directory = get_package_share_directory("robotic_arm")

class ArmState(Enum):
    IDLE = 1
    CAPTURING = 2
    GRASPING = 3

class ArmManipulator(Node):
    def __init__(self):
        super().__init__('arm_manipulator_node')
        
        # Initialize the robotic arm
        self.robot = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
            gripper_pressure=1.0,
            gripper_pressure_lower_limit=60,
            gripper_pressure_upper_limit=200
        )
        robot_startup()

        # Initialize publishers, subscribers, tf broadcaster and listener
        self.initialize_communication()

        # Arm-specific configurations
        self.ZERO_SHOULDER, self.ZERO_ELBOW, self.ZERO_WRIST_ANGLE = -1.8, 1.36, 0.7
        self.EXPLORE_VIEW_JOINTS = [0.0, self.ZERO_SHOULDER, self.ZERO_ELBOW, 0.0, self.ZERO_WRIST_ANGLE, 0.0]
        self.SHOULDER_LIMIT, self.WRIST_LIMIT = -0.785, 1.125
        self.DELTA_ANGLE = 0.001
        self.CAPTURE_VIEW_JOINTS = [0.0, -0.785, 0.785, 0, 0.785, 0]
        self.is_torque_on = True
        self.torque_on()
        self.grasp()
        self.go_to_explore_pose()
        self.get_clock().sleep_for(Duration(seconds=6.0))
        self.state = ArmState.IDLE
        self.try_number = 0
        self.get_logger().info("ArmManipulator initialized")

        # GUI-related attributes
        self.key_pressed = None
        self.gui_update_timer = None  # Timer for periodic GUI updates

        # Schedule the Tkinter thread to start
        self.start_tkinter_thread()

        # Broadcast the static camera transform after initialization
        self.camera_tf_timer = self.create_timer(0.1, self.try_broadcast_camera_tf)
        self.capture_timeout_timer = None  # Timer for capturing timeout

    def initialize_communication(self):
        # Create publishers
        self.grasp_request_pub = self.create_publisher(String, "/text_prompt", 10)
        self.grasp_again_pub = self.create_publisher(String, "/grasp_prompt", 10)
        self.feedback_pub = self.create_publisher(Bool, "/robotic_arm/feedback", 10)

        # Create subscribers
        self.prompt_sub = self.create_subscription(String, "/grasp_prompt", self.prompt_callback, 10)
        self.grasp_sub = self.create_subscription(GraspResponse, "/robotic_arm/grasp_response", self.grasp_callback, 10)
        self.view_angle_sub = self.create_subscription(String, "/robotic_arm/view_angle", self.view_angle_callback, 10)

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.c2g_tf_broadcaster = StaticTransformBroadcaster(self)
        self.camera_tf_broadcaster = StaticTransformBroadcaster(self)
        self.grasp_pose_tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_c2g_tf()

    def start_tkinter_thread(self):
        # Start the Tkinter GUI in a separate thread
        threading.Thread(target=self.run_tkinter, daemon=True).start()

    def run_tkinter(self):
        # Create the GUI in this thread
        self.root = tk.Tk()
        self.root.attributes("-topmost", True)  # Keep the window on top
        self.root.title("Arm Manipulator Key Listener")
        self.root.bind("<KeyPress>", self.on_key_press)

        # Position the window at the bottom-left corner of the screen
        screen_height = self.root.winfo_screenheight()
        x_position = 10  # 10px margin from the left
        y_position = screen_height - self.root.winfo_reqheight() - 50  # 50px margin from the bottom
        self.root.geometry(f"+{x_position}+{y_position}")

        # Add a label for instructions
        self.label = tk.Label(self.root, text="Press keys or buttons to control the arm.", font=("Arial", 8))
        self.label.pack(pady=4)

        # Add a frame for key descriptions
        self.key_frame = tk.Frame(self.root)
        self.key_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)

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
        self.status_label = tk.Label(self.root, text="Last key pressed: None", font=("Arial", 8), fg="blue")

        for description, key in key_descriptions:
            frame = tk.Frame(self.key_frame)
            frame.pack(fill=tk.X, padx=5, pady=2)
            # Add description label
            label = tk.Label(frame, text=description, font=("Arial", 8), anchor="w")
            label.pack(side=tk.LEFT, fill=tk.X, expand=True)
            # Add button for the action
            button = tk.Button(
                frame, 
                text=key, 
                font=("Arial", 10), 
                width=7,
                height=2,
                command=lambda k=key: (self.handle_action(k.lower()), os._exit(0) if k.lower() == 'q' else None)
            )
            button.pack(side=tk.RIGHT)

        self.status_label.pack(pady=2)
        # Label showing arm state
        self.arm_state_label = tk.Label(self.root, text=f"Arm State: {self.state.name}", font=("Arial", 8), fg="green")
        self.arm_state_label.pack(pady=4)

        # Start periodic GUI updates
        self.gui_update_timer = self.create_timer(0.3, self.update_gui)
        self.root.mainloop()  # Start the tkinter event loop

    def update_gui(self):
        # Update the arm state label periodically
        self.arm_state_label.config(text=f"Arm State: {self.state.name}")

    def on_key_press(self, event:tk.Event):
        self.handle_action(event.keysym)

    def handle_action(self, key):
        self.key_pressed = key
        self.status_label.config(text=f"Last key pressed: {self.key_pressed}")  # Update the status label
        self.arm_state_label.config(text=f"Arm State: {self.state.name}")  # Update the arm state label
        if key == 'q':
            self.get_logger().info("Quitting...")
            self.go_to_sleep_pose()
            robot_shutdown()
            os._exit(0)  # Forcefully exit the program
        elif key == 's':
            self.get_logger().info("Go to sleep pose")
            self.go_to_sleep_pose()
        elif key == 'c':
            self.get_logger().info("Go to capture pose")
            self.go_to_capture_pose()
        elif key == 'e':
            self.get_logger().info("Go to explore pose")
            self.go_to_explore_pose()
        elif key == 'r':
            self.get_logger().info("Force release")
            self.release()
        elif key == 'g':
            self.get_logger().info("Force grasp")
            self.grasp()
        elif key == 't':
            self.get_logger().info("Torque toggle")
            self.torque_toggle()
        # Reset the key_pressed after handling
        self.key_pressed = None

    def broadcast_c2g_tf(self):
        tf_file_path = os.path.join(package_share_directory, "config", "transform_camera2gripper.yaml")
        with open(tf_file_path, "r") as file:
            c2g_file = yaml.safe_load(file)
        self.c2g_matrix = np.array(c2g_file["camera2gripper"])
        c2g_tf = TransformStamped()
        c2g_tf.header.stamp = self.get_clock().now().to_msg()
        c2g_tf.header.frame_id = "vx300s/ee_gripper_link"
        c2g_tf.child_frame_id = "camera_frame"
        c2g_tf.transform.translation.x = self.c2g_matrix[0][3]
        c2g_tf.transform.translation.y = self.c2g_matrix[1][3]
        c2g_tf.transform.translation.z = self.c2g_matrix[2][3]
        c2g_tf.transform.rotation.x, c2g_tf.transform.rotation.y, c2g_tf.transform.rotation.z, c2g_tf.transform.rotation.w = quaternion_from_matrix(self.c2g_matrix)
        self.c2g_tf_broadcaster.sendTransform(c2g_tf)

    def try_broadcast_camera_tf(self):
        try:
            self.broadcast_camera_tf()
        except Exception as e:
            self.get_logger().error(f"Error in broadcast_camera_tf:\n{e}")
        finally:
            self.camera_tf_timer.cancel()
            self.camera_tf_timer = None

    def broadcast_camera_tf(self):
        # the D435i_color_optical_frame is the same as camera_frame
        tf: TransformStamped = self.tf_buffer.lookup_transform(
            "D435i_color_optical_frame",
            "D435i_link",
            rclpy.time.Time()
        )
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "camera_frame"
        self.camera_tf_broadcaster.sendTransform(tf)

    def prompt_callback(self, msg: String):
        if self.state != ArmState.IDLE:
            self.get_logger().warn(f'Arm is not in IDLE state, ignore the prompt "{msg.data}" ')
            return
        # Set moving time
        self.robot.arm.set_trajectory_time(moving_time=5)
        # Receive the prompt from the LLM translator or regenerate the grasp pose
        self.text_prompt = msg.data
        self.state = ArmState.CAPTURING
        self.get_logger().info(f'Received prompt: "{self.text_prompt}". Go to capture pose and release the gripper.')
        self.release()
        self.go_to_capture_pose()
        self.get_clock().sleep_for(Duration(seconds=6.0))
        self.grasp_request_pub.publish(msg)
        self.get_logger().info(f'Requesting grasp pose for "{self.text_prompt}", waiting for the response...')
        
        # Set a timer for timeout handling
        if self.capture_timeout_timer:
            self.capture_timeout_timer.cancel()
        self.capture_timeout_timer = self.create_timer(20.0, self.handle_capture_timeout)

    def handle_capture_timeout(self):
        if self.state == ArmState.CAPTURING:
            self.get_logger().warn("Timeout waiting for grasp pose response. Returning to IDLE state.")
            self.state = ArmState.IDLE
        if self.capture_timeout_timer:
            self.capture_timeout_timer.cancel()
            self.capture_timeout_timer = None

    def grasp_callback(self, msg:GraspResponse):
        if self.state != ArmState.CAPTURING:
            self.get_logger().warn(f'Arm is not in CAPTURING state, ignore the grasp response')
            return
        # Receive the grasp pose from the server
        self.state = ArmState.GRASPING
        if self.capture_timeout_timer:
            self.capture_timeout_timer.cancel()
            self.capture_timeout_timer = None
        self.num_grasp_poses = msg.num_grasp_poses
        self.grasp_poses = msg.grasp_poses
        self.scores = msg.scores
        # If the message is empty, fail the task.
        if self.num_grasp_poses == 0:
            self.get_logger().info("No feasible grasp pose found. Grasping failed.")
            self.go_to_explore_pose()
            self.try_number = 0
            self.feedback_pub.publish(Bool(data=False))
            self.state = ArmState.IDLE
            return
        # Use the best grasp pose
        target_pose = self.grasp_poses[0]
        best_score = self.scores[0]
        self.get_logger().info(f"Grasp poses received. Using the best grasp pose with score {best_score}")
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
        # g2b_matrix = self.robot.arm.get_ee_pose()
        t2b_matrix = g2b_matrix @ c2g_matrix  @ t2c_matrix
        t2b_matrix = self.calibrate(t2b_matrix)
        grasp_pose_tf_msg = TransformStamped()
        grasp_pose_tf_msg.header.stamp = self.get_clock().now().to_msg()
        grasp_pose_tf_msg.header.frame_id = "vx300s/base_link"
        grasp_pose_tf_msg.child_frame_id = "last_grasp_pose"
        grasp_pose_tf_msg.transform.translation.x = t2b_matrix[0][3]
        grasp_pose_tf_msg.transform.translation.y = t2b_matrix[1][3]
        grasp_pose_tf_msg.transform.translation.z = t2b_matrix[2][3]
        x,y,z,w = quaternion_from_matrix(t2b_matrix) 
        grasp_pose_tf_msg.transform.rotation.x = x
        grasp_pose_tf_msg.transform.rotation.y = y
        grasp_pose_tf_msg.transform.rotation.z = z
        grasp_pose_tf_msg.transform.rotation.w = w
        self.grasp_pose_tf_broadcaster.sendTransform(grasp_pose_tf_msg)
        self.get_logger().info("Broadcast the grasp pose")
        # Convert target pose to xyz rpy format and log it
        xyz = t2b_matrix[:3, 3]
        rpy = euler_from_matrix(t2b_matrix)
        self.get_logger().info(f"Target pose in xyz: {xyz}, rpy: {rpy}")
        # Move the arm to the grasp pose
        # Use two waypoints to "insert" the gripper towards the object
        waypoint_matrix = t2b_matrix @ translation_matrix([-0.03, 0, 0])
        self.robot.arm.set_ee_pose_matrix(waypoint_matrix, moving_time=5.0, blocking=True)
        waypoint_matrix_2 = t2b_matrix @ translation_matrix([0.015, 0, 0])
        self.robot.arm.set_ee_pose_matrix(waypoint_matrix_2, moving_time=2.0, blocking=True, custom_guess=self.robot.arm.get_joint_positions())
        self.grasp()
        # Wait for the finger closing and move back to capture pose
        self.get_clock().sleep_for(Duration(seconds=4.0))
        self.go_to_capture_pose()
        self.get_clock().sleep_for(Duration(seconds=5.0))
        self.try_number += 1
        self.get_logger().info(f"Try number: {self.try_number}")
        if self.is_success():
            self.go_to_explore_pose()
            self.try_number = 0
            self.feedback_pub.publish(Bool(data=True))
        elif self.try_number < 3:
            # If grasping failed, publish a prompt and grasp again
            self.grasp_again_pub.publish(String(data=self.text_prompt))
            self.get_logger().info(f"Publish the prompt to grasp again: {self.text_prompt}")
        else:
            self.go_to_explore_pose()
            self.try_number = 0
            self.feedback_pub.publish(Bool(data=False))
            self.get_logger().info("Grasping failed.")
        self.state=ArmState.IDLE
    
    def is_success(self):
        # Check if the gripper is closed completely. If so, no object is grasped.
        self.finger_close_position = 0.021
        finger_position = self.robot.gripper.get_finger_position()
        self.get_logger().info(f"Finger position: {finger_position}")
        if finger_position < self.finger_close_position:
            self.get_logger().info("No object grasped or the object is too thin")
            return False
        self.get_logger().info("Object grasped successfully")
        return True

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

    def go_to_capture_pose(self):
        self.robot.arm.set_joint_positions(self.CAPTURE_VIEW_JOINTS, moving_time=5.0, blocking=False)

    def go_to_explore_pose(self):
        self.robot.arm.set_joint_positions(self.EXPLORE_VIEW_JOINTS, moving_time=5.0, blocking=False)

    def go_to_sleep_pose(self):
        self.robot.arm.go_to_sleep_pose(moving_time=5.0, blocking=False)

    def grasp(self, delay=0.0):
        self.robot.gripper.grasp(delay=delay)

    def release(self, delay=0.0):
        self.robot.gripper.release(delay=delay)

    def torque_off(self):
        self.robot.core.robot_torque_enable(cmd_type='group', name='all', enable=False)
        self.is_torque_on = False

    def torque_on(self):
        self.robot.core.robot_torque_enable(cmd_type='group', name='all', enable=True)
        self.is_torque_on = True
    
    def torque_toggle(self):
        if self.is_torque_on:
            self.torque_off()
        else:
            self.torque_on()

    def update_ee_pose(self):
        ee_matrix = self.robot.arm.get_ee_pose()
        xyz = ee_matrix[:3, 3]
        rpy = euler_from_matrix(ee_matrix)
        self.ee_xyz_rpy = [xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]]

    def view_angle_callback(self, msg: String):
        if self.state != ArmState.IDLE:
            self.get_logger().warn(f'Arm is not in IDLE state, ignore the view angle "{msg.data}" ')
            return
        command = msg.data
        joint_commands = self.robot.arm.get_joint_commands()
        _, shoulder, elbow, _, wrist_angle, _ = joint_commands
        moving_time = 0.2
        self.get_logger().info(f'Received view angle command: "{command}"')
        if command == "up":
            if shoulder > self.SHOULDER_LIMIT:
                self.get_logger().warn(f'Arm is at the shoulder limit, cannot move up')
                return
            if wrist_angle > self.ZERO_WRIST_ANGLE:
                joint_commands[4] = wrist_angle - self.DELTA_ANGLE
            else:
                joint_commands[1] = shoulder + self.DELTA_ANGLE
                joint_commands[2] = elbow - self.DELTA_ANGLE
        elif command == "down":
            if wrist_angle > self.WRIST_LIMIT:
                self.get_logger().warn(f'Arm is at the wrist limit, cannot move down')
                return
            if shoulder > self.ZERO_SHOULDER:
                joint_commands[1] = shoulder - self.DELTA_ANGLE
                joint_commands[2] = elbow + self.DELTA_ANGLE
            else:
                joint_commands[4] = wrist_angle + self.DELTA_ANGLE
                
        self.robot.arm.set_joint_positions(joint_commands, moving_time=moving_time, blocking=False)

def main(args=None):
    rclpy.init(args=args)
    node = ArmManipulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()