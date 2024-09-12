#!/usr/bin/env python3
from detectron2.utils.logger import setup_logger
import rclpy.logging
import rclpy.subscription


# Initalize logger from detectron2
setup_logger()

import numpy as np
import math
import cv2
import time
import sys
from scipy.spatial.transform import Rotation as R

# detectron2 dependencies
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.video_visualizer import VideoVisualizer
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog

# Local dependencies
from object_detection.pointcloud import GraspCandidate
import object_detection.utils as utils
from object_detection.logger import Logger
from object_detection.pointcloud import GraspCandidate
from message_filters import ApproximateTimeSynchronizer, Subscriber

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import tf2_ros

# Different runtime options

# Show window with detection visualisation
SHOW_WINDOW_VIS = True

# Target object the system will publish coordinates for
# TARGET_OBJECT = 'sports ball'
TARGET_OBJECT = ['bird', 'teddy bear']
RANSAC_WINDOW = 10

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.starting_time = time.time()
        self.bridge = CvBridge()

        self.color_sub = Subscriber(
            self,
            Image,
            '/object_detection/color'
        )
        
        self.depth_sub = Subscriber(
            self,
            Image,
            '/object_detection/depth'
        )
        
        self.cam_sub = Subscriber(
            self,
            PoseStamped,
            '/slam/odometry'
        )

        self.object_pub = self.create_publisher(PoseStamped, '/object_detection/target/world_nwu', 10)
        self.segmentation_vis_pub = self.create_publisher(Image, '/object_detection/segmentation_vis', 10)
        self.grasp_pcd_pub = self.create_publisher(Float32MultiArray, '/object_detection/grasp_pcd', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # The receiver for the image frames sent over the local network
        # self.receiver = VideoReceiver()
        self.cam = utils.OAKDCameraMockup()



        # detectron2 setup - change as needed
        cfg = get_cfg()
        cfg.merge_from_file(model_zoo.get_config_file('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml'))
        cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
        cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url('COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml')
        self.predictor = DefaultPredictor(cfg)

        

        metadata = MetadataCatalog.get(cfg.DATASETS.TRAIN[0])
        self.v = VideoVisualizer(metadata)

        # Setup for a custom logger
        self.logger = Logger()
        self.records = np.empty((0, self.logger.cols))
        self.time_logger = Logger()

        # This line is particularly important to get the class labels
        self.class_catalog = metadata.thing_classes

        ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub, self.cam_sub], queue_size=10, slop=1)
        ts.registerCallback(self.callback)

        self.recorded_positions = []

        print('Initialization done!')

    def o3d_pcd_to_multiarray(self, pcd):
        colors = np.asarray(pcd.colors)
        points = np.asarray(pcd.points)

        msg_data = np.concatenate((points, colors), axis=1)

        rows_dim = MultiArrayDimension()
        rows_dim.label = 'rows'
        rows_dim.size = msg_data.shape[0]
        rows_dim.stride = msg_data.shape[0] * msg_data.shape[1]

        cols_dim = MultiArrayDimension()
        cols_dim.label = 'cols'
        cols_dim.size = msg_data.shape[1]
        cols_dim.stride = msg_data.shape[1]

        msg = Float32MultiArray()
        msg.layout.dim = [rows_dim, cols_dim]
        msg.layout.dim[0].size = msg_data.shape[0]
        msg.layout.dim[0].stride = 1
        msg.layout.dim[0].label = 'points'
        msg.data = list(msg_data.flatten().astype(np.float32))



        return msg


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
        mask = mask[:,:,0]
        mask = (mask > 0).astype(np.uint8) * 255
        print(f"mask shape {mask.shape}")

        # Create an empty green channel
        green_channel = np.zeros_like(mask)

        # Create the overlay in green (0, 255, 0)
        green_overlay = np.stack([green_channel, mask, green_channel], axis=-1)

        # Convert the mask to boolean for easier masking
        mask_bool = mask.astype(bool)

        # Copy the original image to create the overlay image
        overlay_image = image.copy()
        print(f"frame shape {overlay_image.shape}")
        print(f"frame shape {green_overlay.shape}")


        # Apply the green overlay where the mask is true
        overlay_image[mask_bool] = overlay_image[mask_bool] * 0.5 + green_overlay[mask_bool] * 0.5

        return overlay_image


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

    def callback(self, frame, depth_frame, cam_pose):
  
        cam_intrinsics = self.cam.intrinsics
        grasp = GraspCandidate()

        frame = self.bridge.imgmsg_to_cv2(frame, 'rgb8')
        # frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)  

        depth_frame = self.bridge.imgmsg_to_cv2(depth_frame, desired_encoding='mono16')

        # Get detectron2 output

        # flip the image
        flipped_frame = cv2.flip(frame, 0)

        outputs = self.predictor(flipped_frame)
        detected_class_idxs = outputs['instances'].pred_classes
        pred_boxes = outputs['instances'].pred_boxes
        
        # Show visualization if enabled
        if SHOW_WINDOW_VIS:
            out = self.v.draw_instance_predictions(flipped_frame, outputs['instances'].to('cpu'))
            vis_frame = np.asarray(out.get_image())

        # This holds the masks generated by detectron2
        mask_array = outputs['instances'].pred_masks.to('cpu').numpy()
        mask_array = np.moveaxis(mask_array, 0, -1)
        num_instances = mask_array.shape[-1]
        mask_array_instance = []
        object_has_been_found = False
        # Loop over instances that have been detected
        for i in range(num_instances):
            class_idx = detected_class_idxs[i]
            class_name = self.class_catalog[class_idx]
            
            # Create the mask frame that can be applied on the RGB image
            mask_array_instance.append(mask_array[:, :, i:(i+1)])
            obj_mask = np.zeros_like(frame)
            obj_mask = np.where(mask_array_instance[i] == True, 255, obj_mask)

            # flip back the mask
            obj_mask = cv2.flip(obj_mask, 0)

            # cv2.imwrite(f'pictures/mask_{class_name}.png',obj_mask)

            # Target has been detected - apply frame transformation and/or send coordinates

            if class_name in TARGET_OBJECT:
                masked_vis_img = self.overlay_segmentation(frame, obj_mask)
                self.segmentation_vis_pub.publish(self.bridge.cv2_to_imgmsg(masked_vis_img, 'rgb8'))
                cv2.imshow('segmented frame', masked_vis_img)
                cv2.waitKey(1)

                object_has_been_found = True
                

                # Create point cloud of detected object
                # First, apply mask to RGB frame
                masked_frame = cv2.bitwise_and(frame, obj_mask)
                # cv2.imwrite('masked_frame.png', masked_frame)
                
                try:
                    # Create point cloud 
                    grasp.set_point_cloud_from_aligned_masked_frames(masked_frame, depth_frame, cam_intrinsics)
                    centroid = grasp.find_centroid()
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
                    #grasp_pcd_msg = self.o3d_pcd_to_multiarray(grasp.pointcloud)
                    #self.grasp_pcd_pub.publish(grasp_pcd_msg)
                    
                except Exception as e:
                    # Sometimes, qhull fails and throws an error and we don't want the program to crash
                    print('Something went wrong with the point cloud processing')
                    print(e)
                    return
            
                
                
                # Align to camera frame from Open3D frame axis assignment
                tvec = [-centroid[0], -centroid[1], -centroid[2]]

                print(f'object in camera frame {tvec}')
                
                # transform point to world frame
                cam_rot_quat = [cam_pose.pose.orientation.x, cam_pose.pose.orientation.y, cam_pose.pose.orientation.z, cam_pose.pose.orientation.w]
                cam_rot = R.from_quat(cam_rot_quat)
                tvec = cam_rot.apply(tvec)
                tvec[0] += cam_pose.pose.position.x
                tvec[1] += cam_pose.pose.position.y
                tvec[2] += cam_pose.pose.position.z

                # tvec should now be in NWU world frame

                print(f'object in world {tvec}')

                self.recorded_positions.append(tvec)

                tvec = self.estimate_ransac_position(self.recorded_positions)
                print(f'object in world ransac {tvec}')

                # send output via ROS
                msg = PoseStamped()
                msg.pose.position.x = tvec[0]
                msg.pose.position.y = tvec[1]
                msg.pose.position.z = tvec[2]
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0
                msg.header.stamp = cam_pose.header.stamp
                self.object_pub.publish(msg)

                
        # Time it took to analyse one frame 
        # elapsed_time = time.time() - starting_time

        # Show output frame
        if SHOW_WINDOW_VIS:
            cv2.imshow('output', vis_frame)
            cv2.waitKey(1)

        if not object_has_been_found:
            cv2.imshow('segmented frame', frame)
            cv2.waitKey(1)
            self.segmentation_vis_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'rgb8'))
        
        # print(f'ELAPSED TIME (ms): {elapsed_time * 1000}')
    
    

def main(args=None):
    rclpy.init(args=args)

    # The grasp object stores the point cloud of the object and implements grasp planning

    # rclpy.init()


    # VideoWriter outputs to store the analyzed frames at different points in time

    node = DetectionNode()
    rclpy.spin(node)
    node.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()