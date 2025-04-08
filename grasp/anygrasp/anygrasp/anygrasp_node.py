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
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PoseArray
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

package_share_directory = get_package_share_directory("anygrasp")
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup, Grasp


class Config:
    def __init__(self, checkpoint_path, max_gripper_width, gripper_height, top_down_grasp, debug, save_record):
        self.checkpoint_path = checkpoint_path
        self.max_gripper_width = max_gripper_width
        self.gripper_height = gripper_height
        self.top_down_grasp = top_down_grasp
        self.debug = debug
        self.save_record = save_record


class AnyGraspNode(Node):
    def __init__(self):
        super().__init__("anygrasp_node")
        config_path = os.path.join(package_share_directory, "config", "anygrasp_config.yaml")
        with open(config_path, "r") as file:
            self.config = yaml.safe_load(file)
        self.records = os.path.join(self.config["paths"]["package_path"], self.config["paths"]["record_path"])

        self.bridge = CvBridge()

        self.cfgs = Config(
            checkpoint_path=os.path.join(package_share_directory, "log", "checkpoint_detection.tar"),
            max_gripper_width=self.config["parameters"]["max_gripper_width"],
            gripper_height=self.config["parameters"]["gripper_height"],
            top_down_grasp=self.config["parameters"]["top_down_grasp"],
            debug=self.config["parameters"]["debug"],
            save_record=self.config["parameters"]["save_record"],
        )
        self.anygrasp = AnyGrasp(self.cfgs)
        self.anygrasp.load_net()
        self.get_logger().info(f"AnyGrasp model loaded.\n")
        self.max_grasp_number = int(self.config["parameters"]["max_grasp_number"])

        self.rgbd_sub = self.create_subscription(RGBD, "/cropped_rgbd", self.rgbd_callback, 10)
        self.grasp_pub = self.create_publisher(GraspResponse, "/grasp_response", 10)
        # Debug timer and publishers
        self.object_pcd = PointCloud2()
        self.pcd_pub = self.create_publisher(PointCloud2, "/grasp_point_cloud", 10)
        self.grasp_pose_array = PoseArray()
        self.pose_pub = self.create_publisher(PoseArray, "/grasp_pose_array", 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Keeps the visualization alive 
        self.get_logger().info("The anygrasp_node initialized successfully, listening to /cropped_rgbd\n")

    def rgbd_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "rgb8")
        depth_image = self.bridge.imgmsg_to_cv2(msg.depth, "16UC1")

        rgb_camera_info = msg.rgb_camera_info
        # depth_camera_info = msg.depth_camera_info

        self.get_logger().info("Cropped rgbd received, generating grasp poses...")
        grasp_response, gg, cloud = self.generate_grasp_poses(rgb_image, depth_image, rgb_camera_info)
        num = grasp_response.num_grasp_poses
        if num != 0:
            self.get_logger().info(f"Generated {grasp_response.num_grasp_poses} grasp poses. Publishing results...")
            self.grasp_pub.publish(grasp_response)
            if self.cfgs.save_record:
                self.get_logger().info("Saving the record...")
                self.save_record(grasp_response, rgb_image, depth_image, gg, cloud)
            if self.cfgs.debug:
                self.publish_pcd(cloud, grasp_response)
            self.get_logger().info("Keep listening to /cropped_rgbd\n")
        else:
            self.get_logger().info("No grasp poses detected. Do not publish any grasp response. Keep listening to /cropped_rgbd\n")

    def generate_grasp_poses(self, rgb_image, depth_image, camera_info):
        points, colors, lims = self.prepare_data(rgb_image, depth_image, camera_info)
        # Print shape and type of points and colors
        # print("Points shape:", points.shape, "Points type:", points.dtype)
        # print("Colors shape:", colors.shape, "Colors type:", colors.dtype)

        # generate grasp poses and cloud (open3d.geometry.PointCloud)
        gg, cloud = self.anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=True, collision_detection=True)

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
        
        return grasp_response, gg_pick, cloud

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

    def save_record(self, grasp_response, rgb_image, depth_image, gg: GraspGroup, cloud: o3d.geometry.PointCloud):
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        record_dir = os.path.join(self.records, f"record_{timestamp}")
        os.makedirs(record_dir, exist_ok=True)

        # Save the images
        cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR, rgb_image)
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

        # Save the grasp visualization in pointcloud
        # Geometry are upside down, need to transform
        trans_mat = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        cloud.transform(trans_mat)
        grippers = gg.to_open3d_geometry_list()
        for gripper in grippers:
            gripper.transform(trans_mat)

        # Create an off-screen renderer
        width, height = 640, 480
        renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)
        # Add geometries to the renderer
        MaterialRecord = o3d.visualization.rendering.MaterialRecord()
        MaterialRecord.point_size = 6
        renderer.scene.add_geometry("cloud", cloud, MaterialRecord)
        # Visualize the pointcloud without grasp pose
        pointcloud_image = renderer.render_to_image()
        o3d.io.write_image(os.path.join(record_dir,"object_pointcloud.png"), pointcloud_image)
        # Visualize the pointcloud with grasp pose
        for i, gripper in enumerate(grippers):
            if i == 0:
                gripper.paint_uniform_color([1, 0, 0])
                renderer.scene.add_geometry("best_gripper", gripper, o3d.visualization.rendering.MaterialRecord())
            # else:
            #     gripper.paint_uniform_color([0, 1, 0])
            #     renderer.scene.add_geometry(f"gripper_{i}", gripper, o3d.visualization.rendering.MaterialRecord())
        grasp_image = renderer.render_to_image()
        o3d.io.write_image(os.path.join(record_dir,"grasp_pointcloud.png"), grasp_image)
        self.get_logger().info("Pointcloud visualization image saved")

    def publish_pcd(self, cloud, grasp_response: GraspResponse):
        # Publish the point cloud in rviz, frame_id is "static_grasp_camera_frame"
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "static_grasp_camera_frame"
        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)
        cloud_data = np.hstack([points, colors])
        self.object_pcd = pc2.create_cloud_xyz32(header, cloud_data[:, :3].tolist())
        self.pcd_pub.publish(self.object_pcd)
        self.get_logger().info("Published point cloud to /grasp_point_cloud")

        # Publish the grasp poses as PoseArray in rviz, frame_id is "static_grasp_camera_frame"
        self.grasp_pose_array.header = header
        self.grasp_pose_array.poses = [pose for pose in grasp_response.grasp_poses]
        self.pose_pub.publish(self.grasp_pose_array)
        self.get_logger().info(f"Published {len(self.grasp_pose_array.poses)} poses to /grasp_pose_array")

    def timer_callback(self):
        self.pcd_pub.publish(self.object_pcd)
        self.pose_pub.publish(self.grasp_pose_array)

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
