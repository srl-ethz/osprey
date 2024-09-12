from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    spectacularai_node = Node(
        package='spectacular_vio',
        executable='spectacular_node',
        parameters=[
            { 'recordingFolder': LaunchConfiguration("recordingFolder") },
        ],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
        package='rviz2', executable='rviz2', output='screen',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare("spectacular_vio"),
                'spectacular.rviz'
            ])
        ]
    )

    # tf2 format:
    # x, y, z, qx, qy, qz, qw, parent, child
    # with transformation child->parent (i.e. T_PC)

    # Parent: ENU, Child: NED. 
    # Transform NED->ENU: intrinsic XYZ 180° pitch, -90° yaw
    tf2_ENU_NED = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0.7071068", "0.7071068", "0", "0", "world", "world_ned"])

    # Parent: ENU, Child: NWU.
    # Transform NWU->ENU: intrinsic XYZ 90° yaw
    tf2_ENU_NWU = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0.7071068", "0.7071068", "world", "world_nwu"])
    
    # Parent: Body NED, Child: Camera.
    # Transform Camera->Body NED:
    # intrinsic ZYX -90° yaw, -50° pitch 
    # Tranlsation: x=0.08, y=0, z=-0.02
    tf2_BNED_C = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0.08", "0", "-0.02", "-0.2988362", "0.2988362", "-0.6408564", "0.6408564", "body_ned", "cam_spec"])

    return [
        spectacularai_node,
        rviz_node,
        tf2_ENU_NED,
        tf2_ENU_NWU,
        tf2_BNED_C
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value='True'),
            DeclareLaunchArgument("recordingFolder", default_value='')
        ] + [
            OpaqueFunction(function=launch_setup)
        ]
    )
