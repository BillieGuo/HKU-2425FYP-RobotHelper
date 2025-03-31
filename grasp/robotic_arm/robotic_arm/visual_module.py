import pyrealsense2 as rs
from pupil_apriltags import Detector
import cv2
import numpy as np

class VisualModule:
    def __init__(self, tag_size=0.07):
        self.pipeline = None
        self.profile = None
        self.fx = self.fy = self.cx = self.cy = None
        self.detector = None
        self.tag_size = tag_size

    def initialize_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self._get_camera_intrinsics()

    def _get_camera_intrinsics(self):
        intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.fx, self.fy, self.cx, self.cy = intr.fx, intr.fy, intr.ppx, intr.ppy

    def initialize_detector(self):
        self.detector = Detector(
            families="tag36h11",
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

    def detect_tags(self, color_image):
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        camera_params = [self.fx, self.fy, self.cx, self.cy]
        return self.detector.detect(
            gray_image, estimate_tag_pose=True, camera_params=camera_params, tag_size=self.tag_size
        )

    def draw_tag_info(self, color_image, tag):
        for idx in range(len(tag.corners)):
            cv2.line(
                color_image,
                tuple(tag.corners[idx - 1, :].astype(int)),
                tuple(tag.corners[idx, :].astype(int)),
                (0, 255, 0),
                2,
            )
        cv2.circle(color_image, tuple(tag.center.astype(int)), 5, (0, 0, 255), -1)
        center_x, center_y = tag.center.astype(int)
        cv2.putText(
            color_image,
            f"ID: {tag.tag_id}",
            (center_x, center_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    def get_color_image(self):
        frame = self.pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()
        return np.asanyarray(color_frame.get_data()) if color_frame else None

    def stop_camera(self):
        if self.pipeline:
            self.pipeline.stop()
