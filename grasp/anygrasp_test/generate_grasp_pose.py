import os
import argparse
import torch
import numpy as np
import open3d as o3d
from PIL import Image
import yaml

from gsnet import AnyGrasp
from graspnetAPI import GraspGroup

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint_path", required=True, help="Model checkpoint path")
parser.add_argument("--max_gripper_width", type=float, default=0.1, help="Maximum gripper width (<=0.1m)")
parser.add_argument("--gripper_height", type=float, default=0.03, help="Gripper height")
parser.add_argument("--top_down_grasp", action="store_true", help="Output top-down grasps.")
parser.add_argument("--debug", action="store_true", help="Enable debug mode")
cfgs = parser.parse_args()
cfgs.max_gripper_width = max(0, min(0.1, cfgs.max_gripper_width))


def demo(data_dir):
    anygrasp = AnyGrasp(cfgs)
    anygrasp.load_net()

    # get data
    colors = np.array(Image.open(os.path.join(data_dir, "cropped_color_mouse.png")), dtype=np.float32) / 255.0
    depths = np.array(Image.open(os.path.join(data_dir, "cropped_depth_mouse.png")))
    # colors = np.array(Image.open(os.path.join(data_dir, "saved_color.png")), dtype=np.float32) / 255.0
    # depths = np.array(Image.open(os.path.join(data_dir, "saved_depth.png")))
    # get camera intrinsics
    # fx, fy = 927.17, 927.37
    # cx, cy = 651.32, 349.62
    # D435i_color_optical_frame intrinsics
    fx, fy = 909.7047119140625, 909.7649536132812
    cx, cy = 647.6729736328125, 358.6357727050781
    scale = 1000.0
    # set workspace to filter output grasps
    xmin, xmax = -0.19, 0.12
    ymin, ymax = 0.02, 0.15
    zmin, zmax = 0.0, 1.0
    lims = [xmin, xmax, ymin, ymax, zmin, zmax]

    # get point cloud
    print(depths.shape)
    xmap, ymap = np.arange(depths.shape[1]), np.arange(depths.shape[0])
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depths / scale
    points_x = (xmap - cx) / fx * points_z
    points_y = (ymap - cy) / fy * points_z
    

    # set your workspace to crop point cloud
    mask = (points_z > 0) & (points_z < 1)
    points = np.stack([points_x, points_y, points_z], axis=-1)
    points = points[mask].astype(np.float32)
    colors = colors[mask].astype(np.float32)

    # Print shape and type of points and colors
    print("Points shape:", points.shape, "Points type:", points.dtype)
    print("Colors shape:", colors.shape, "Colors type:", colors.dtype)

    print(points.min(axis=0), points.max(axis=0))

    gg, cloud = anygrasp.get_grasp(points, colors, lims=lims, apply_object_mask=True, dense_grasp=False, collision_detection=True)

    if len(gg) == 0:
        print("No Grasp detected after collision detection!")

    gg = gg.nms().sort_by_score()
    gg_pick = gg[0:20]
    print(gg_pick.scores)
    print("grasp score:", gg_pick[0].score)
    best_grasp = gg_pick[0]
    print(best_grasp.translation, best_grasp.rotation_matrix)

    # Save point cloud to file
    o3d.io.write_point_cloud("output_point_cloud.pcd", cloud)
    print("Point cloud saved to output_point_cloud.pcd")

    # Save best grasp pose to file
    grasp_pose = {"translation": best_grasp.translation.tolist(), "rotation_matrix": best_grasp.rotation_matrix.tolist()}
    with open("best_grasp_pose.yaml", "w") as f:
        yaml.dump(grasp_pose, f)
    print("Best grasp pose saved to best_grasp_pose.yaml")

    # visualization
    if cfgs.debug:
        trans_mat = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        cloud.transform(trans_mat)
        grippers = gg.to_open3d_geometry_list()
        for gripper in grippers:
            gripper.transform(trans_mat)
        # With Display
        # o3d.visualization.draw_geometries([*grippers, cloud])
        # o3d.visualization.draw_geometries([grippers[0], cloud])

        # Without Display
        # Create an off-screen renderer
        width, height = 1280, 720
        renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)

        # Add geometries to the renderer
        renderer.scene.add_geometry("cloud", cloud, o3d.visualization.rendering.MaterialRecord())
        # for i, gripper in enumerate(grippers):
        #    renderer.scene.add_geometry(f"gripper_{i}", gripper, o3d.visualization.rendering.MaterialRecord())
        renderer.scene.add_geometry("best_gripper", grippers[0], o3d.visualization.rendering.MaterialRecord())

        # Capture the screen image and save it
        image = renderer.render_to_image()
        o3d.io.write_image("visualization_result.png", image)


if __name__ == "__main__":
    demo(".")
