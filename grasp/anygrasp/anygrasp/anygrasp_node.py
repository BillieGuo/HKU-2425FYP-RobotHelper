import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import open3d as o3d
from realsense2_camera_msgs.msg import RGBD
from custom_msgs.msg import GraspResponse
from geometry_msgs.msg import Pose
import yaml
from datetime import datetime
from scipy.spatial.transform import Rotation as R  # Add this import

package_share_directory = get_package_share_directory("anygrasp")
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup


class Config:
    def __init__(self, checkpoint_path, max_gripper_width, gripper_height, top_down_grasp, debug):
        self.checkpoint_path = checkpoint_path
        self.max_gripper_width = max_gripper_width
        self.gripper_height = gripper_height
        self.top_down_grasp = top_down_grasp
        self.debug = debug


class AnyGraspNode(Node):
    def __init__(self):
        super().__init__("anygrasp_node")
        config_path = os.path.join(package_share_directory, "config", "anygrasp_config.yaml")
        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)
        self.records = os.path.join(self.config["paths"]["package_path"], self.config["paths"]["record_path"])

        self.bridge = CvBridge()

        cfgs = Config(
            checkpoint_path=os.path.join(package_share_directory, "log", "checkpoint_detection.tar"),
            max_gripper_width=self.config["parameters"]["max_gripper_width"],
            gripper_height=self.config["parameters"]["gripper_height"],
            top_down_grasp=False,
            debug=True,
        )
        self.anygrasp = AnyGrasp(cfgs)
        self.anygrasp.load_net()
        self.get_logger().info(f"AnyGrasp model loaded.\n")
        self.max_grasp_number = int(self.config["parameters"]["max_grasp_number"])

        self.rgbd_sub = self.create_subscription(RGBD, "/cropped_rgbd", self.rgbd_callback, 10)
        self.grasp_pub = self.create_publisher(GraspResponse, "/grasp_response", 10)
        self.get_logger().info("The anygrasp_node initialized successfully, listening to /cropped_rgbd\n")

    def rgbd_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")

        rgb_camera_info = msg.rgb_camera_info
        # depth_camera_info = msg.depth_camera_info

        self.get_logger().info("Cropped rgbd received, generating grasp poses...")
        grasp_response, cloud = self.generate_grasp_poses(rgb_image, depth_image, rgb_camera_info)
        num = grasp_response.num_grasp_poses
        if num != 0:
            self.get_logger().info(f"Generated {grasp_response.num_grasp_poses} grasp poses. Publishing results...")
            self.grasp_pub.publish(grasp_response)
            self.get_logger().info("Saving the record...")
            self.save_record(grasp_response, rgb_image, depth_image, cloud)
            self.get_logger().info("Keep listening to /cropped_rgbd\n")
        else:
            self.get_logger().info("No grasp poses detected. Do not publish any grasp response. Keep listening to /cropped_rgbd\n")

    def generate_grasp_poses(self, rgb_image, depth_image, camera_info):
        points, colors, lims = self.prepare_data(rgb_image, depth_image, camera_info)
        # Print shape and type of points and colors
        # print("Points shape:", points.shape, "Points type:", points.dtype)
        # print("Colors shape:", colors.shape, "Colors type:", colors.dtype)

        # generate grasp poses and cloud (open3d.geometry.PointCloud)
        gg, cloud = self.anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)

        if gg is None:
            self.get_logger().info("No Grasp detected after collision detection!")
            return GraspResponse(num_grasp_poses=0, grasp_poses=[], scores=[]), cloud
        else:
            print(f"Got {len(gg)} grasps")

        gg = gg.nms().sort_by_score()
        if len(gg) < self.max_grasp_number:
            gg_pick = gg
        else:
            gg_pick = gg[0 : self.max_grasp_number]
        grasp_poses = []
        scores = []

        # make response
        for grasp in gg_pick:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = grasp.translation
            # Convert rotation matrix to quaternion
            r = R.from_matrix(grasp.rotation_matrix)
            q = r.as_quat()
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            grasp_poses.append(pose)
            scores.append(float(grasp.score))
        grasp_response = GraspResponse(num_grasp_poses=len(grasp_poses), grasp_poses=grasp_poses, scores=scores)
        
        return grasp_response, cloud

    def prepare_data(self, rgb_image, depth_image, camera_info):
                # Convert images to numpy arrays
        rgb_image = np.array(rgb_image, dtype=np.float32) / 255.0
        depth_image = np.array(depth_image, dtype=np.float32)

        # Camera intrinsics
        fx, fy = camera_info.k[0], camera_info.k[4]
        cx, cy = camera_info.k[2], camera_info.k[5]
        # print(fx, fy, cx, cy)

        # get point cloud
        xmap, ymap = np.arange(depth_image.shape[1]), np.arange(depth_image.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depth_image / 1000.0
        points_x = (xmap - cx) / fx * points_z
        points_y = (ymap - cy) / fy * points_z

        # set workspace to filter output grasps
        xmin, xmax = -1.0, 1.0
        ymin, ymax = -1.0, 1.0
        zmin, zmax = 0.0, 3.0
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]

        # set your workspace to crop point cloud
        mask = (points_z > 0) & (points_z < 1)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)
        colors = rgb_image[mask].astype(np.float32)

        return points, colors, lims

    def save_record(self, grasp_response, rgb_image, depth_image, cloud):
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        record_dir = os.path.join(self.records, f"record_{timestamp}")
        os.makedirs(record_dir, exist_ok=True)

        # Save the images
        cv2.imwrite(os.path.join(record_dir, "rgb_image.png"), rgb_image)
        cv2.imwrite(os.path.join(record_dir, "depth_image.png"), depth_image)

        # Save point cloud to file
        o3d.io.write_point_cloud(os.path.join(record_dir, "output_point_cloud.pcd"), cloud)
        self.get_logger().info(f"Point cloud saved to {os.path.join(record_dir, 'output_point_cloud.pcd')}")

        # Save the grasp response as yaml
        grasp_response_dict = {
            "num_grasp_poses": int(grasp_response.num_grasp_poses),
            "grasp_poses": [
                {
                    "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                    "orientation": [float(pose.orientation.w), float(pose.orientation.x), float(pose.orientation.y), float(pose.orientation.z)],
                }
                for pose in grasp_response.grasp_poses
            ],
            "scores": [float(score) for score in grasp_response.scores],
        }
        with open(os.path.join(record_dir, "grasp_response.yaml"), "w") as f:
            yaml.dump(grasp_response_dict, f)
        self.get_logger().info(f"Grasp response saved to {os.path.join(record_dir, 'grasp_response.yaml')}")


def main(args=None):
    rclpy.init(args=args)
    node = AnyGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted. Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
