from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    objectdetection_node = Node(
        package='object_detection',
        executable='object_detection_node',
        parameters=[
            { 'recordingFolder': LaunchConfiguration("recordingFolder") },
        ],
    )

    return [objectdetection_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("recordingFolder", default_value='')
        ] + [
            OpaqueFunction(function=launch_setup)
        ]
    )
