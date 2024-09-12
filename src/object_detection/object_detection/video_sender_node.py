#!/usr/bin/env python3
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
import tf2_ros
from cv_bridge import CvBridge
from threading import Thread
import multiprocessing
import imagezmq
from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber
import socket
import json
import argparse

class VideoSenderNode(Node):
    def __init__(self, addr):
        super().__init__('video_sender_node')
        self.bridge = CvBridge()
        self.jpeg_quality = 95
        self.png_quality = 2
        self.hostname = socket.gethostname()

        self.color_sub = Subscriber(
            self,
            Image,
            '/camera/color/image_raw')
        self.depth_sub = Subscriber(
            self,
            Image,
            '/camera/depth/image_rect_raw')

        self.cam_sub = Subscriber(
            self,
            PoseStamped,
            '/slam/odometry',
        )


        self.object_pub = self.create_publisher(PoseStamped, '/object_detection/target/world_nwu', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sender_color = imagezmq.ImageSender(
            connect_to=addr)
        self.sender_depth = imagezmq.ImageSender(
            connect_to=addr)

        self.get_logger().info(f'Connected to {addr}')

        manager = multiprocessing.Manager()
        self.ret_dict = manager.dict()

        ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.cam_sub], 10, 0.1)
        ts.registerCallback(self.callback)

    def encode_color(self, color, ret_dict):
        '''
        Compress an OpenCV image frame with JPEG compression. Returns a bytestring in the given
        return dictionary.
        '''

        _, jpg_frame = cv2.imencode(
            '.jpg', color, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        ret_dict['color'] = jpg_frame

    def encode_depth(self, depth, ret_dict):
        '''
        Compress an OpenCV image frame with PNG compression. Returns a bytestring in the given
        return dictionary.
        '''

        _, frame_depth = cv2.imencode(
            '.png', depth, [int(cv2.IMWRITE_PNG_COMPRESSION), self.png_quality])
        ret_dict['depth'] = frame_depth

    def callback(self, color_msg, depth_msg, cam_pose_msg):
        '''
        Send the compressed frames using imagezmg and Python threads.
        '''

        # encode pose msg into json
        pose = {
            'position': {
                'x': cam_pose_msg.pose.position.x,
                'y': cam_pose_msg.pose.position.y,
                'z': cam_pose_msg.pose.position.z
            },
            'orientation': {
                'x': cam_pose_msg.pose.orientation.x,
                'y': cam_pose_msg.pose.orientation.y,
                'z': cam_pose_msg.pose.orientation.z,
                'w': cam_pose_msg.pose.orientation.w
            }
        }

        timestamp_color = {
            'sec': color_msg.header.stamp.sec,
            'nanosec': color_msg.header.stamp.nanosec
        }
        timestamp_depth = {
            'sec': depth_msg.header.stamp.sec,
            'nanosec': depth_msg.header.stamp.nanosec
        }

        color = self.bridge.imgmsg_to_cv2(color_msg, "passthrough")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")

        t1 = Thread(target=self.encode_color, args=(color, self.ret_dict))
        t2 = Thread(target=self.encode_depth, args=(depth, self.ret_dict))
        t1.start()
        t2.start()

        t1.join()
        t2.join()

        # Get results from threads
        jpg_color = self.ret_dict['color']
        png_depth = self.ret_dict['depth']

        header_color = {
            'timestamp': timestamp_color,
            'hostname': self.hostname,
            'cam_pose': pose,
        }

        header_depth = {
            'timestamp': timestamp_depth,
            'hostname': self.hostname + '_depth',
            'cam_pose': pose,
        }

        header_str_color = json.dumps(header_color)
        header_str_depth = json.dumps(header_depth)

        self.sender_color.send_jpg(header_str_color, jpg_color)
        target_pose_reply = self.sender_depth.send_jpg(header_str_depth, png_depth)

        # decode JSON from binary
        target_pose = json.loads(target_pose_reply.decode())

        valid = target_pose['valid']
        if valid:
            # publish target pose
            tvec = np.array([target_pose['position']['x'], target_pose['position']['y'], target_pose['position']['z']])

            # convert to world frame
            cam_rot_quat = [cam_pose_msg.pose.orientation.x, cam_pose_msg.pose.orientation.y, cam_pose_msg.pose.orientation.z, cam_pose_msg.pose.orientation.w]
            # transform point to world frame
            cam_rot_quat = [cam_pose_msg.pose.orientation.x, cam_pose_msg.pose.orientation.y, cam_pose_msg.pose.orientation.z, cam_pose_msg.pose.orientation.w]
            cam_rot = R.from_quat(cam_rot_quat)
            tvec = cam_rot.apply(tvec)
            tvec[0] += cam_pose_msg.pose.position.x
            tvec[1] += cam_pose_msg.pose.position.y
            tvec[2] += cam_pose_msg.pose.position.z

            # tvec should now be in NWU world frame

            print(f'object in world {tvec}')

            # self.recorded_positions.append(tvec)

            # tvec = self.estimate_ransac_position(self.recorded_positions)
            # print(f'object in world ransac {tvec}')

            # send output via ROS
            msg = PoseStamped()
            msg.pose.position.x = tvec[0]
            msg.pose.position.y = tvec[1]
            msg.pose.position.z = tvec[2]
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg.header.stamp = cam_pose_msg.header.stamp
            self.object_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Video Sender Node')
    parser.add_argument('--receiver_ip', type=str, default='10.10.10.188',
                        help='The address to connect to for sending video streams')
    cli_args = parser.parse_args()


    node = VideoSenderNode(f'tcp://{cli_args.receiver_ip}:5555')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
