"""
Run StellaVSLAM `run_slam` with explicit paths (Humble / Jazzy compatible pattern).

Prerequisites (built from source per Stella docs):
  - stella_vslam_ros
  - ORB vocabulary .fbow file
  - Camera YAML (see repo `config/stella_pinhole_template.yaml`)

Example:
  ros2 launch vslam_nav2_integration stella_slam_only.launch.py \\
    vocab:=/path/to/orb_vocab.fbow \\
    camera_config:=/path/to/camera.yaml \\
    map_db_out:=/tmp/out.msg
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vocab = LaunchConfiguration("vocab")
    camera_config = LaunchConfiguration("camera_config")
    map_db_out = LaunchConfiguration("map_db_out")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vocab",
                description="Path to ORB vocabulary .fbow",
            ),
            DeclareLaunchArgument(
                "camera_config",
                description="Path to StellaVSLAM camera YAML",
            ),
            DeclareLaunchArgument(
                "map_db_out",
                default_value="/tmp/stella_map.msg",
                description="Where to save the map database",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Let stella_vslam publish map->odom/camera TF (verify frame names!)",
            ),
            Node(
                package="stella_vslam_ros",
                executable="run_slam",
                name="stella_vslam",
                output="screen",
                arguments=[
                    "-v",
                    vocab,
                    "-c",
                    camera_config,
                    "--map-db-out",
                    map_db_out,
                ],
                parameters=[{"publish_tf": publish_tf}],
            ),
        ]
    )
