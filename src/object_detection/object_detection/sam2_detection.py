#!/usr/bin/env python3
import os
import sys
import math
import numpy as np
import torch
import cv2
from PIL import Image
import tqdm
import matplotlib.pyplot as plt
import time
from threading import Lock
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
# Local dependencies
from pointcloud import GraspCandidate
import utils as utils
from logger import Logger
from pointcloud import GraspCandidate
from streamer_receiver import VideoReceiver
from sam2_detector import Sam2Detector



# Different runtime options

# Show window with detection visualisation
SHOW_WINDOW_VIS = True

# Target object the system will publish coordinates for
# TARGET_OBJECT = 'sports ball'
RANSAC_WINDOW = 10


class DetectionNode():
    '''
    This node works the following way:

        1. Receives the color and depth images from the camera together with the camera pose
        2. Displays the images in a window, if the user presses 'a', the display will stop at the current frame and the user can click on the object to track.
           In the meanwhile, we still record frames we receive during the annotation process.
        3. Once the user clicks on the object, we start tracking the object using the SAM2 model. First, we do inference on the images we collected so far.
           Then, we track the object in real-time based on the incoming frames.
    '''

    def __init__(self):
        # super().__init__('detection_node')
        self.starting_time = time.time()
        # store in directory with date and time added
        self.video_dir = f'./video_frames_{time.strftime("%Y-%m-%d_%H-%M-%S")}'
        os.makedirs(self.video_dir)
        os.makedirs(f'{self.video_dir}/total_frames/')
        self.data = []
        self.frame_idx = 0
        self.total_idx = 0
        self.record_frames = False
        self.tracking_has_begun = False
        self.lock = Lock()

        cv2.namedWindow('Color', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Color', self.on_click, self.frame_idx)

        sam2_checkpoint = '/home/osprey/repos/segment-anything-2/checkpoints/sam2_hiera_large.pt'
        model_cfg = 'sam2_hiera_l.yaml'
        self.sam2_detector = Sam2Detector(model_cfg=model_cfg, sam2_checkpoint=sam2_checkpoint, frame_dir=self.video_dir)

        # The receiver for the image frames sent over the local network
        self.receiver = VideoReceiver(convert_grayscale=True)
        self.cam = utils.OAKDCameraMockup()

        # Setup for a custom logger
        self.logger = Logger()
        self.records = np.empty((0, self.logger.cols))
        self.time_logger = Logger()

        self.recorded_positions = []

        print('Initialization done!')


    def estimate_ransac_position(self, positions, threshold=0.1, iterations=100):

        if len(positions) == 0:
            return None

        best_position = None
        best_inliers = 0

        positions = np.array(positions)

        if len(positions) < iterations:
            iterations = len(positions)

        for i in range(iterations):
            # randomly sample one point
            idx = np.random.randint(0, len(positions))
            point = positions[idx]

            # find all points within threshold
            inlier_idx = np.linalg.norm(positions - point, axis=1) < threshold
            inliers = positions[inlier_idx]

            if len(inliers) > best_inliers:
                best_inliers = len(inliers)
                best_position = np.mean(inliers, axis=0)

        return best_position
    
    def show_mask(mask, obj_id=None, random_color=False):
        if random_color:
            color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
        else:
            cmap = plt.get_cmap("tab10")
            cmap_idx = 0 if obj_id is None else obj_id
            color = np.array([*cmap(cmap_idx)[:3], 0.6])
        h, w = mask.shape[-2:]
        mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
        return mask_image
        
    def overlay_segmentation(self, image, mask):
        """
        Overlay a binary segmentation mask on an RGB image with green color.

        Parameters:
        - image: numpy array of shape (H, W, 3) representing the RGB image.
        - mask: numpy array of shape (H, W) representing the binary segmentation mask.

        Returns:
        - overlay_image: numpy array of shape (H, W, 3) representing the image with the mask overlaid.
        """

        # Ensure the mask is binary (0 or 255)
        mask = (mask > 0).astype(np.uint8) * 255

        # Create an empty green channel
        green_channel = np.zeros_like(mask)

        # Create the overlay in green (0, 255, 0)
        green_overlay = np.stack([green_channel, mask, green_channel], axis=-1)

        # Convert the mask to boolean for easier masking
        mask_bool = mask.astype(bool)

        # Copy the original image to create the overlay image
        overlay_image = image.copy()

        # Apply the green overlay where the mask is true
        overlay_image[mask_bool] = overlay_image[mask_bool] * 0.5 + green_overlay[mask_bool] * 0.5

        return overlay_image

    def viz_mask_logits(self, out_mask_logits):
        mask_image = out_mask_logits.squeeze((0,1)).cpu().numpy() > 0
        mask_image = mask_image.astype(np.uint8) * 255
        return mask_image
    
    def on_click(self, event, x, y, flags, params):
        self.record_frames = True
        if event == cv2.EVENT_LBUTTONDBLCLK:
            with self.lock:
                print(f'Click at: x: {x}, y: {y}')
                print(f'Writing frame to {self.video_dir}')
                cv2.imwrite(f'{self.video_dir}/0.jpg', self.current_data[0])
                out_obj_ids, out_mask_logits = self.sam2_detector.add_annotation([[x, y]], [1])
                for out_obj_ids, out_mask_logits in self.sam2_detector.run_init_batch():
                    mask_image = self.viz_mask_logits(out_mask_logits)
                    cv2.imwrite(f'{self.video_dir}/mask.jpg', mask_image)
                self.tracking_has_begun = True
                self.frame_idx = 1

    def run(self):
        '''
        Receive the RGB and depth frames from the camera and the camera pose. Store the color frames in the video directory.
        '''
        print('Received frames!')
        color, depth, color_header, depth_header = self.receiver.recv_frames()
        with self.lock:
            if self.tracking_has_begun:
                print(f'Beginning tracking')
                # run single frame tracking with SAM2
                cv2.imwrite(f'{self.video_dir}/{self.frame_idx}.jpg', color)
                self.frame_idx += 1
                obj_ids, out_mask_logits = self.sam2_detector.run_single_frame(deepcopy(color))
                if len(obj_ids) > 0:
                    mask_img = self.viz_mask_logits(out_mask_logits)
                    seg_img = self.overlay_segmentation(color, mask_img)
                    cv2.imwrite(f'{self.video_dir}/seg_{self.frame_idx}.jpg', seg_img)
                    cv2.imwrite(f'{self.video_dir}/mask_{self.frame_idx}.png', mask_img)
                    np.savez_compressed(f'{self.video_dir}/depth_{self.frame_idx}.npz', depth)
                    cv2.imshow('segmented img', seg_img)
                    cv2.imshow("masked img", mask_img)
                    cv2.waitKey(1)
                    
                    tvec = self.find_grasp(deepcopy(color), depth, mask_img)
                    self.receiver.reply(tvec)
                    print(f'Found object at {tvec}')


                    # self.receiver.reply(None)
                else:
                    self.receiver.reply(None)
            else:
                print(f'Not tracking yet...')
                self.receiver.reply(None)
                self.current_data = (color, depth)

        if not self.tracking_has_begun:
            cv2.imwrite(f'{self.video_dir}/total_frames/total_{self.total_idx}.jpg', color)
        else:
            cv2.imwrite(f'{self.video_dir}/total_frames/total_{self.total_idx}.jpg', seg_img)

        self.total_idx += 1

        cv2.imshow('Color', color)
        cv2.waitKey(10)



    def find_grasp(self, frame, depth_frame, mask):
        '''
        Given the RGB frame, the mask, the depth frame and the cam pose, determine a grasping point and publish it.
        '''

        cam_intrinsics = self.cam.intrinsics
        grasp = GraspCandidate()

        # flip the image
        # flipped_frame = cv2.flip(frame, 0)
        obj_mask = np.zeros_like(frame)
        obj_mask = np.where(mask[:, :, None] > 0, 255, obj_mask)


        # flip back the mask
        # obj_mask = cv2.flip(obj_mask, 0)


        # Create point cloud of detected object
        # First, apply mask to RGB frame
        masked_frame = cv2.bitwise_and(frame, obj_mask)
        # cv2.imwrite('masked_frame.png', masked_frame)
        cv2.imshow('masked color frame', masked_frame)
        cv2.waitKey(1)
        try:
            # Create point cloud
            grasp.set_point_cloud_from_aligned_masked_frames(masked_frame, depth_frame, cam_intrinsics)
            centroid = grasp.find_centroid()
            # grasp.find_grasping_point_from_top()
            # axis_ext, _, _ = grasp.find_largest_axis()
            # # Get longest axis
            # axis = axis_ext[0]
            # # Rotate copy of point cloud around longest axis
            # pcd = grasp.rotate_pcd_around_axis(grasp.pointcloud, centroid, math.pi, axis)
            # grasp.pointcloud += pcd
            # grasp.save_pcd(f'pcd/pointcloud_{TARGET_OBJECT}.pcd')
            # grasp.visualise_pcd()
            # # Find grasping points
            # grasp_points = grasp.find_grasping_points()
            # grasp_pcd_msg = self.o3d_pcd_to_multiarray(grasp.pointcloud)
            # self.grasp_pcd_pub.publish(grasp_pcd_msg)

        except Exception as e:
            # Sometimes, qhull fails and throws an error and we don't want the program to crash
            print('Something went wrong with the point cloud processing')
            print(e)
            return None


        # convert from open3d coordinate system to camera coordinate system
        tvec = [-centroid[0], -centroid[1], -centroid[2]]

        # self.recorded_positions.append(tvec)

        # tvec = self.estimate_ransac_position(self.recorded_positions)
        
        # Time it took to analyse one frame
        # elapsed_time = time.time() - starting_time

        # Show output frame
        # if SHOW_WINDOW_VIS:
        #     cv2.imshow('output', vis_frame)
        #     cv2.waitKey(1)

        # print(f'ELAPSED TIME (ms): {elapsed_time * 1000}')
        
        return tvec



def main(args=None):
    # rclpy.init(args=args)

    # The grasp object stores the point cloud of the object and implements grasp planning

    # rclpy.init()


    # VideoWriter outputs to store the analyzed frames at different points in time

    node = DetectionNode()
    while True:
        node.run()
    # rclpy.spin(node)
    # node.close()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
