import math
import numpy as np

def truncate(number, digits) -> float:
        stepper = 10.0 ** digits
        return math.trunc(stepper * number) / stepper

# import pyrealsense2 as rs
# class RSCameraMockup():
#     def __init__(self):
#         self.width = 640
#         self.height = 480
#         self.intrinsics = rs.intrinsics()
#         self.intrinsics.coeffs = [-0.053769, 0.0673601, -0.000281265, 0.000637035, -0.0215778]
#         self.intrinsics.ppx = 323.432 
#         self.intrinsics.ppy = 242.815
#         self.intrinsics.fx = 384.879
#         self.intrinsics.fy = 384.372
#         self.intrinsics.width = 640
#         self.intrinsics.height = 480
#         self.intrinsics.model = rs.distortion.inverse_brown_conrady
#         self.depth_scale = 0.0010000000474974513

#     def deproject(self, intrinsics, x, y, depth):
#         return rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)

#     def project(self, intrinsics, point):
#         return rs.rs2_project_point_to_pixel(intrinsics, point)

#     def release(self):
#         print('Release called')
    
#     def colorize_frame(self, depth_frame):
#         depth_frame = rs.pyrealsense2.frame(depth_frame)
#         colorizer = rs.colorizer(color_scheme=0)
#         return np.asanyarray(
#             colorizer.colorize(depth_frame).get_data())

class CameraIntrinsics():
      def __init__(self, fx, fy, cx, cy, width, height) -> None:
            self.fx = fx 
            self.fy = fy
            self.cx = cx
            self.cy = cy
            self.width = width
            self.height = height

class OAKDCameraMockup():
    def __init__(self) -> None:
        self.intrinsics = CameraIntrinsics(393.57, 393.57, 324.69, 195.89, 640, 400)
        self.width = self.intrinsics.width
        self.height = self.intrinsics.height
        
    def release(self):
          pass
