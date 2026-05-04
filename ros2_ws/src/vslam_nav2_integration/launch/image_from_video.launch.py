"""Publish `/camera/image_raw` from an MP4 using `image_publisher` (sensor_msgs/Image)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_path = LaunchConfiguration("video_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_path",
                description="Absolute path to a video file readable by OpenCV",
            ),
            Node(
                package="image_publisher",
                executable="image_publisher_node",
                name="kitti_video_publisher",
                output="screen",
                arguments=[video_path],
                remappings=[("/image_raw", "/camera/image_raw")],
            ),
        ]
    )
