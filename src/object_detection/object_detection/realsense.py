import pyrealsense2 as rs
import numpy as np


class RSCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        # Dimensions for any camera of the Realsense series
        self.width = 640
        self.height = 480
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width,
                             self.height, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, self.width,
                             self.height, rs.format.z16, 30)

        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        image_sensor = profile.get_device().query_sensors()[1]

        # Need this to compute depth
        self.depth_scale = depth_sensor.get_depth_scale()

        # Enable automatic change of expose for high contrast scenes
        image_sensor.set_option(rs.option.enable_auto_exposure, True)

    # Get the raw frames as tensors
    def get_raw_frames(self):
        frames = self.pipeline.wait_for_frames()

        depth = np.asanyarray(frames.get_depth_frame().get_data())
        color = np.asanyarray(frames.get_color_frame().get_data())

        return (color, depth)

    # Get the frames as rs2.Frame objects
    def get_rs_frames(self):
        frames = self.pipeline.wait_for_frames()

        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

    # Get frames aligned to color as tensors
    def get_raw_color_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()

        align_to = rs.stream.color
        align = rs.align(align_to)
        aligned_frames = align.process(frames)

        depth = np.asanyarray(aligned_frames.get_depth_frame().get_data())
        color = np.asanyarray(aligned_frames.get_color_frame().get_data())

        return (color, depth)

    # Get frames aligned to color as rs2 frame objects
    def get_rs_color_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()

        align_to = rs.stream.color
        align = rs.align(align_to)
        aligned_frames = align.process(frames)

        return (aligned_frames.get_color_frame(), aligned_frames.get_depth_frame())

    def colorize_frame(self, depth_frame):
        colorizer = rs.colorizer(color_scheme=0)
        return np.asanyarray(
            colorizer.colorize(depth_frame).get_data())

    def deproject(self, intrinsics, x, y, depth):
        return rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)

    def project(self, intrinsics, point):
        return rs.rs2_project_point_to_pixel(intrinsics, point)

    def release(self):
        self.pipeline.stop()
