import sys

import numpy as np
import cv2
import imagezmq
import time
import json


class VideoReceiver:
    def __init__(self, convert_grayscale=False) -> None:
        self.image_hub = imagezmq.ImageHub()
        self.delays = []
        self.convert_grayscale = convert_grayscale

        print(f'Initialized VideoReceiver, convert grayscale to RGB: {self.convert_grayscale}')

    def recv_frames(self):
        start = time.time()
        color_header, color_jpg_buffer = self.image_hub.recv_jpg()
        color = cv2.imdecode(np.frombuffer(color_jpg_buffer, dtype='uint8'), -1)
        self.image_hub.send_reply(b'OK')

        depth_header, depth_buffer = self.image_hub.recv_jpg()


        depth = cv2.imdecode(np.frombuffer(depth_buffer, dtype='uint8'), -1)
        color_header = json.loads(color_header)
        depth_header = json.loads(depth_header)

        if self.convert_grayscale:
            color = cv2.merge([color, color, color])
        cv2.imshow(color_header['hostname'], color)
        cv2.imshow(depth_header['hostname'], depth)
        cv2.waitKey(1)

        # pose_data = color_header['pose']

        # without sending this, the whole pipeline will be blocked
        # but to send the target pose back to the ROS system, we send it in the reply method
        # self.image_hub.send_reply(b'OK')

        return color, depth, color_header, depth_header

    def reply(self, target_pose):
        if target_pose is None:
            target_pose_json = {
                'valid': False,
            }
        else:
            # print(f'target pose: {target_pose}')
            target_pose_json = {
                'valid': True,
                'position': {
                    'x': target_pose[0],
                    'y': target_pose[1],
                    'z': target_pose[2],
                }
            }

        target_pose_str = json.dumps(target_pose_json)

        self.image_hub.send_reply(target_pose_str.encode())
