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
        self.READY_POSITIONS = [0, -1.1, 1.1, 0, 0.9, 0]
        robot_startup()
        self.is_torque_on = True
        self.torque_on()
    
    def go_to_initial_pose(self):
        self.arm.set_joint_positions(self.READY_POSITIONS)

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
    # Data Samples, lists of rotation and translation matrices
    # translation unit: meter
    R_target2camera_list = []
    t_target2camera_list = []
    R_gripper2base_list = []
    t_gripper2base_list = []

    print("At the camera window, press Enter to record data, press 'q' to quit and calculate the transformation.")

    while True:
        # Get color image
        frame = pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        if color_image is None:
            continue

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detect_tags(detector, gray_image, [fx, fy, cx, cy], tag_size)

        # target2camera, transforms a point in the target frame to the camera frame, which is the pose of the tag in camera frame.
        # gripper2base, transforms a point in the gripper frame to the base frame, which is the pose of the gripper in base frame.
        if tags:
            tag = tags[0]
            if tag.pose_t is not None:
                # Get target2camera transformation
                R_target2camera = tag.pose_R
                t_target2camera = tag.pose_t
                # Get gripper2base transformation
                gripper_pose = bot.get_pose()
                R_gripper2base = gripper_pose[:3, :3]
                t_gripper2base = gripper_pose[:3, 3]
                # print(f"R_target2camera: {R_target2camera},\n t_target2camera: {t_target2camera}")
                # print(f"R_gripper2base: {R_gripper2base},\n t_gripper2base: {t_gripper2base}")
                draw_tag_info(color_image, tag)

        cv2.imshow("AprilTag Detection", color_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        if key == ord("t"):
            bot.torque_toggle()
        elif key == ord("\r") or key == ord("\n"):  # Enter key
            if tags:
                R_target2camera_list.append(R_target2camera)
                t_target2camera_list.append(t_target2camera)
                R_gripper2base_list.append(R_gripper2base)
                t_gripper2base_list.append(t_gripper2base)
                print(f"R_target2camera: {R_target2camera},\n t_target2camera: {t_target2camera}")
                print(f"R_gripper2base: {R_gripper2base},\n t_gripper2base: {t_gripper2base}")
                print(f"{len(R_target2camera_list)} pairs of poses recorded.")

    if len(R_target2camera_list) >= 3:
        R_camera2gripper, t_camera2gripper = cv2.calibrateHandEye(np.array(R_gripper2base_list), np.array(t_gripper2base_list), np.array(R_target2camera_list), np.array(t_target2camera_list), method=cv2.CALIB_HAND_EYE_TSAI)
        camera2gripper = np.eye(4)
        camera2gripper[:3, :3] = R_camera2gripper
        camera2gripper[:3, 3] = t_camera2gripper.flatten()
        print(f"R_camera2gripper: {R_camera2gripper},\n t_camera2gripper: {t_camera2gripper}")
        transformation_data = {"camera2gripper": camera2gripper.tolist()}
        
        save_dir = os.path.expanduser("~/fyp_ws/src/HKU-2425FYP-RobotHelper/grasp/robotic_arm/config")
        save_file = os.path.join(save_dir, "transform_camera2gripper.yaml")
        os.makedirs(save_dir, exist_ok=True)
        with open(save_file, "w") as file:
            yaml.dump(transformation_data, file)
    else:
        print("Not enough data points to calculate the transformation.")
    
    
    cv2.destroyAllWindows()
    # camera
    pipeline.stop()
    # robot
    bot.go_to_rest_pose()
    robot_shutdown()

if __name__ == "__main__":
    main()
