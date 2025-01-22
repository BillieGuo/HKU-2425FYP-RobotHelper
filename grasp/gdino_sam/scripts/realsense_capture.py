import pyrealsense2 as rs
import numpy as np
import cv2
from datetime import datetime
import os

script_dir = os.path.dirname(os.path.abspath(__file__))

pipe = rs.pipeline()
cfg = rs.config()

resolutions_map = {
    "480p": (640, 480),
    "720p": (1280, 720),
}
resolution = "480p"
width, height = resolutions_map[resolution]
cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)

pipe.start(cfg)

# Ensure the directory exists
output_dir = os.path.join(script_dir, f"../assets/realsense_capture_{resolution}")
os.makedirs(output_dir, exist_ok=True)

while True:
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    cv2.imshow("rgb", color_image)
    cv2.imshow("depth", depth_image)

    key = cv2.waitKey(1)
    if key == ord("q"):
        break
    elif key == ord("s"):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        rgb_filename = os.path.join(output_dir, f"rgb_image_{timestamp}.png")
        depth_filename = os.path.join(output_dir, f"depth_image_{timestamp}.png")
        cv2.imwrite(rgb_filename, color_image)
        cv2.imwrite(depth_filename, depth_image)
        print(f"Saved {rgb_filename} and {depth_filename}")


pipe.stop()
cv2.destroyAllWindows()
