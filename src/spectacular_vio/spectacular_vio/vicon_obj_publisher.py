import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
from message_filters import ApproximateTimeSynchronizer, Subscriber

import rclpy
from rclpy.node import Node

class ViconObjectPublisher(Node):
    def __init__(self):
        super().__init__('vicon_object_publisher')
        self.get_logger().info('Vicon object publisher node started')

        self.drone_topic = '/vicon/srl_osprey/srl_osprey'
        self.object_topic = '/vicon/srl_object/srl_object'
        self.px4_topic = '/px4/pose/ned'
        # self.px4_topic = '/vicon/srl_osprey/ned'

        # publisher for the object pose in the SLAM world frame
        self.pose_pub = self.create_publisher(PoseStamped, '/object/vicon/pose_ned', 10)

        # these two should be coming from the Vicon system in the same frame
        self.drone_sub = Subscriber(self, PoseStamped, self.drone_topic)
        self.obj_sub = Subscriber(self, PoseStamped, self.object_topic)

        # this should come directly from the PX4, obtained through the SLAM system
        self.px4_drone_sub = Subscriber(self, PoseStamped, self.px4_topic)

        # synchronize the timestamps of the incoming messages
        self.sync = ApproximateTimeSynchronizer([self.drone_sub, self.obj_sub, self.px4_drone_sub], 10, 0.1)
        self.sync.registerCallback(self.callback)

    def drone_to_object(self, drone_pose, obj_pose):
        # copmute difference in the translation of the poses and publish as PoseStamped

        # compute world frame translation difference
        world_x = obj_pose.pose.position.x - drone_pose.pose.position.x
        world_y = obj_pose.pose.position.y - drone_pose.pose.position.y
        world_z = obj_pose.pose.position.z - drone_pose.pose.position.z

        drone_to_obj = np.array([world_x, world_y, world_z])

        # transform into drone frame rotation
        # get quaternion from drone pose
        drone_quat = np.array([drone_pose.pose.orientation.x, drone_pose.pose.orientation.y, drone_pose.pose.orientation.z, drone_pose.pose.orientation.w])
        drone_rot = R.from_quat(drone_quat)

        # rotate the vector into drone frame
        drone_to_obj = drone_rot.inv().apply(drone_to_obj)

        # change into NED frame from ENU from vicon
        # TODO: uncomment this, only for debugging
        drone_to_obj[0], drone_to_obj[1] = drone_to_obj[1], drone_to_obj[0]
        drone_to_obj[2] = -drone_to_obj[2]

        return drone_to_obj

    def build_pose_msg(self, translation, time, rotation = [0.0, 0.0, 0.0, 1.0]):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = translation[0]
        pose.pose.position.y = translation[1]
        pose.pose.position.z = translation[2]
        pose.pose.orientation.x = rotation[0]
        pose.pose.orientation.y = rotation[1]
        pose.pose.orientation.z = rotation[2]
        pose.pose.orientation.w = rotation[3]
        return pose

    def callback(self, drone_pose, obj_pose, px4_pose):
        # copmute difference in the translation of the poses and publish as PoseStamped

        # log original object position
        self.get_logger().info('Original ENU bject position: ' + str(obj_pose.pose.position))

        # translation vector from 
        drone_to_obj = self.drone_to_object(drone_pose, obj_pose)
        # self.get_logger().info('Drone to object in drone frame: ' + str(drone_to_obj))

        # transform from drone frame to world frame from PX4 pose
        drone_trans = np.array([px4_pose.pose.position.x, px4_pose.pose.position.y, px4_pose.pose.position.z])

        # get quaternion from drone pose
        drone_quat = np.array([px4_pose.pose.orientation.x, px4_pose.pose.orientation.y, px4_pose.pose.orientation.z, px4_pose.pose.orientation.w])
        drone_rot = R.from_quat(drone_quat)

        # rotate the vector into world frame
        world_to_obj = drone_rot.apply(drone_to_obj)
        # translate with drone translation
        world_to_obj += drone_trans

        self.get_logger().info('NED World to object' + str(world_to_obj))

        # convert into ENU frame
        world_to_obj_enu = np.array([world_to_obj[1], world_to_obj[0], -world_to_obj[2]])

        self.get_logger().info('ENU World to object' + str(world_to_obj_enu))

        pose = self.build_pose_msg(world_to_obj, obj_pose.header.stamp)

        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    vicon_obj_publisher = ViconObjectPublisher()
    rclpy.spin(vicon_obj_publisher)
    vicon_obj_publisher.destroy_node()
    rclpy.shutdown()
