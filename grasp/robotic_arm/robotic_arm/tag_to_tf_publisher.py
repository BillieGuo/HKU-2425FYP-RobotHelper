import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from robotic_arm.visual_module import VisualModule
import cv2

def publish_static_transform(tag_pose, tag_id):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform = TransformStamped()

    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "camera_frame"
    transform.child_frame_id = f"tag_{tag_id}"

    transform.transform.translation.x = tag_pose[0, 3]
    transform.transform.translation.y = tag_pose[1, 3]
    transform.transform.translation.z = tag_pose[2, 3]

    rotation_matrix = tag_pose[:3, :3]
    quaternion = tf2_ros.transformations.quaternion_from_matrix(
        np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]])
    )
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    broadcaster.sendTransform(transform)

def main():
    rospy.init_node("tag_to_tf_publisher")
    visual_module = VisualModule(tag_size=0.07)
    visual_module.initialize_camera()
    visual_module.initialize_detector()

    rospy.loginfo("Press Ctrl+C to stop the node.")
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            color_image = visual_module.get_color_image()
            if color_image is None:
                continue

            tags = visual_module.detect_tags(color_image)
            if tags:
                tag = tags[0]
                if tag.pose_t is not None:
                    tag_pose = np.eye(4)
                    tag_pose[:3, :3] = tag.pose_R
                    tag_pose[:3, 3] = tag.pose_t.flatten()
                    visual_module.draw_tag_info(color_image, tag)
                    publish_static_transform(tag_pose, tag.tag_id)

            cv2.imshow("AprilTag Detection", color_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except rospy.ROSInterruptException:
        pass
