import pyrealsense2 as rs
from pupil_apriltags import Detector
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import cv2
import numpy as np
import os
import yaml

def initialize_pipeline():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    return pipeline, profile

def get_camera_intrinsics(profile):
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    return intr.fx, intr.fy, intr.ppx, intr.ppy

def initialize_detector(tag_size):
    return Detector(families="tag36h11", nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0), tag_size

def detect_tags(detector, gray_image, camera_params, tag_size):
    return detector.detect(gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=tag_size)

def draw_tag_info(color_image, tag):
    for idx in range(len(tag.corners)):
        cv2.line(color_image, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0), 2)
    cv2.circle(color_image, tuple(tag.center.astype(int)), 5, (0, 0, 255), -1)
    center_x, center_y = tag.center.astype(int)
    cv2.putText(color_image, f"ID: {tag.tag_id}", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

class robot(InterbotixManipulatorXS):
    def __init__(self, robot_model, group_name, gripper_name):
        super().__init__(robot_model=robot_model, group_name=group_name, gripper_name=gripper_name)
        self.gripper_pressure = 0.5
        self.READY_POSITIONS = [0, -1.1, 1.1, 0, 0.9, 0]
        self.HIGH_ANGLE = [-0.5, -1.1, 0.4, 0, 1.6, 0]
        robot_startup()
        self.is_torque_on = True
        self.torque_on()
    
    def go_to_initial_pose(self):
        self.arm.set_joint_positions(self.HIGH_ANGLE)

    def go_to_rest_pose(self):
        self.arm.go_to_sleep_pose()

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
    
    def set_ee_pose(self, pose, position_only=False):
        if position_only:
            position = pose[:3, 3]
            x, y, z = position[0], position[1], position[2]
            self.arm.set_ee_pose_components(x, y, z)
        else: # In 4*4 matrix
            self.arm.set_ee_pose(pose)
    
    def release(self):
        self.gripper.release()
    
    def grasp(self):
        self.gripper.grasp()

    def increase_pressure(self):
        self.gripper_pressure += 0.1
        if self.gripper_pressure > 1.0:
            self.gripper_pressure = 1.0
        self.gripper.set_pressure(self.gripper_pressure)
        print(f"Gripper pressure: {self.gripper_pressure}")

    def decrease_pressure(self):
        self.gripper_pressure -= 0.1
        if self.gripper_pressure < 0.0:
            self.gripper_pressure = 0.0
        self.gripper.set_pressure(self.gripper_pressure)
        print(f"Gripper pressure: {self.gripper_pressure}")


    def get_pose(self):
        # In 4*4 matrix
        return self.arm.get_ee_pose()

def main():
    # Camera
    pipeline, profile = initialize_pipeline()
    fx, fy, cx, cy = get_camera_intrinsics(profile)
    # AprilTag
    detector, tag_size = initialize_detector(0.07)  # Set the tag size in meters
    # Robot
    bot = robot(robot_model='vx300s', group_name='arm', gripper_name='gripper')
    bot.go_to_initial_pose()
    # Load transformation        
    config_dir = os.path.expanduser("~/fyp_ws/src/HKU-2425FYP-RobotHelper/grasp/robotic_arm/config")
    transform_file = os.path.join(config_dir, "transform_camera2gripper.yaml")
    with open(transform_file, "r") as file:
        camera2gripper = yaml.safe_load(file)
    camera2gripper = np.array(camera2gripper["camera2gripper"])
    print(f"camera2gripper Loaded: {camera2gripper}")
    print("At the camera window, press Enter to reach the AprilTag, press 'q' to quit.")

    while True:
        # Get color image
        frame = pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        if color_image is None:
            continue

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detect_tags(detector, gray_image, [fx, fy, cx, cy], tag_size)

        if tags:
            tag = tags[0]
            if tag.pose_t is not None:
                # Get target2camera transformation
                R_target2camera = tag.pose_R
                t_target2camera = tag.pose_t
                target2camera = np.eye(4)
                target2camera[:3, :3] = R_target2camera
                target2camera[:3, 3] = t_target2camera.flatten()
                # Get gripper2base transformation
                gripper_pose = bot.get_pose()
                R_gripper2base = gripper_pose[:3, :3]
                t_gripper2base = gripper_pose[:3, 3]
                gripper2base = np.eye(4)
                gripper2base[:3, :3] = R_gripper2base
                gripper2base[:3, 3] = t_gripper2base.flatten()
                draw_tag_info(color_image, tag)

        cv2.imshow("AprilTag Detection", color_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        if key == ord("t"):
            bot.torque_toggle()
        elif key == ord("h"):
            bot.go_to_initial_pose()
        elif key == ord("j"):
            bot.grasp()
        elif key == ord("k"):
            bot.release()
        elif key == ord("i"):
            bot.increase_pressure()
        elif key == ord("m"):
            bot.decrease_pressure()
        elif key == ord("\r") or key == ord("\n"):  # Enter key
            if tags:
                target2base = gripper2base @ camera2gripper @ target2camera
                print(f"target2base: {target2base}")
                bot.set_ee_pose(target2base, position_only=True)

    cv2.destroyAllWindows()
    # camera
    pipeline.stop()
    # robot
    bot.go_to_rest_pose()
    robot_shutdown()

if __name__ == "__main__":
    main()
