"""Compose `image_from_video` + `stella_slam_only` for offline MP4 SLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("vslam_nav2_integration")
    vocab = LaunchConfiguration("vocab")
    camera_config = LaunchConfiguration("camera_config")
    map_db_out = LaunchConfiguration("map_db_out")
    video_path = LaunchConfiguration("video_path")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("vocab"),
            DeclareLaunchArgument("camera_config"),
            DeclareLaunchArgument(
                "map_db_out", default_value="/tmp/stella_map.msg"
            ),
            DeclareLaunchArgument("video_path"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "image_from_video.launch.py"])
                ),
                launch_arguments={"video_path": video_path}.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [pkg, "launch", "stella_slam_only.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "vocab": vocab,
                            "camera_config": camera_config,
                            "map_db_out": map_db_out,
                            "publish_tf": publish_tf,
                        }.items(),
                    )
                ],
            ),
        ]
    )
