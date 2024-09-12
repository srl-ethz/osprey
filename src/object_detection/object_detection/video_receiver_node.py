import sys

import numpy as np
import cv2
import time
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from object_detection.streamer_receiver import VideoReceiver


class VideoReceiverNode(Node):
    def __init__(self):
        super().__init__('video_receiver_node')
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/object_detection/color', 10)
        self.depth_pub = self.create_publisher(Image, '/object_detection/depth', 10)
        self.receiver = VideoReceiver(convert_grayscale=True)

        self.run_receiver()

    def run_receiver(self):
            while rclpy.ok():

                color, depth, color_header, depth_header = self.receiver.recv_frames()
                # Convert the CV Images to ROS Image messages
                timestamp_color = color_header['timestamp']
                timestamp_depth = color_header['timestamp']

                timestamp_color_ros = rclpy.time.Time(seconds=timestamp_color['sec'], nanoseconds=timestamp_color['nanosec'])
                timestamp_depth_ros = rclpy.time.Time(seconds=timestamp_depth['sec'], nanoseconds=timestamp_depth['nanosec'])
                color_msg = self.bridge.cv2_to_imgmsg(color, encoding='rgb8')
                depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='mono16')

                timestamp_color_ros = timestamp_color_ros.to_msg()
                timestamp_depth_ros = timestamp_depth_ros.to_msg()

                color_msg.header.stamp = timestamp_color_ros
                depth_msg.header.stamp = timestamp_depth_ros
                # Publish the images
                self.color_pub.publish(color_msg)
                self.depth_pub.publish(depth_msg)

def main():
    rclpy.init()
    node = VideoReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()
