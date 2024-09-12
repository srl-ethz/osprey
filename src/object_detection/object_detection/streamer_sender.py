#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
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
        
        self.sender_color = imagezmq.ImageSender(
            connect_to=addr)
        self.sender_depth = imagezmq.ImageSender(
            connect_to=addr)
        
        self.get_logger().info(f'Connected to {addr}')
        
        manager = multiprocessing.Manager()
        self.ret_dict = manager.dict()

        ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], 10, 0.1)
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

    def callback(self, color_msg, depth_msg):
        '''
        Send the compressed frames using imagezmg and Python threads. 
        '''

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
            'hostname': self.hostname
        }

        header_depth = {
            'timestamp': timestamp_depth,
            'hostname': self.hostname + '_depth'
        }

        header_str_color = json.dumps(header_color)
        header_str_depth = json.dumps(header_depth)

        self.sender_color.send_jpg(header_str_color, jpg_color)
        self.sender_depth.send_jpg(header_str_depth, png_depth)

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

