# Tutorial: StellaVSLAM with Nav2 (no preloaded occupancy map)

This tutorial is written for a **class setting**: it explains how to combine **StellaVSLAM** (visual mapping + pose) with **Nav2** (planning + control) on a TurtleBot-class robot in a maze, while being explicit about **what each subsystem estimates** under a Recursive State Estimation (RSE) view.

## What you are integrating

| Subsystem | Typical outputs | Role in RSE |
|-----------|-----------------|-------------|
| Wheel odometry + IMU | `odom` → `base_link` | Short-horizon proprioceptive prediction |
| **StellaVSLAM** | Sparse 3D map, camera pose, optional `map`→`odom` TF | Visual loop closure, drift correction |
| **2D obstacle map** (laser, depth, or SLAM toolbox) | `/map` occupancy, obstacle layers | Exteroceptive correction for **collision avoidance** |

Nav2’s costmaps are driven by **2D occupancy or 3D voxel layers backed by range/depth data**. A monocular VSLAM map alone is usually **not** a dense occupancy grid. For a credible maze demo **without a pre-saved map file**, run an **online** mapper (for example `slam_toolbox` with a 2D lidar, or depth-based obstacle layers) **in parallel** with StellaVSLAM, and treat Stella as the **global visual backbone** for consistency and relocalization.

## Prerequisites

- Ubuntu 22.04 + ROS 2 **Humble** (Stella upstream tests against Humble or newer).
- `stella_vslam` and `stella_vslam_ros` built from source per [Stella ROS2 package](https://stella-cv.readthedocs.io/en/latest/ros2_package.html).
- ORB vocabulary `orb_vocab.fbow`.
- Robot description + simulation (TurtleBot3/4 maze world) or hardware equivalent.
- Nav2: `sudo apt install ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-turtlebot3*` (exact metapackages depend on your course VM).

## Build this overlay package

```bash
cd /path/to/final_project/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select vslam_nav2_integration
source install/setup.bash
```

`stella_vslam_ros` must already be in the same overlay or an underlay sourced **before** `install/setup.bash`.

## Frame graph (verify in RViz)

Target a consistent tree:

```text
map  -->  odom  -->  base_link  -->  camera_link / camera_optical_frame
```

- Align `map_frame`, `odom_frame`, and camera frame parameters in Stella’s ROS node with your URDF.
- If the camera is not at the robot origin, publish a **static transform** from `base_link` to the camera optical frame (or include it in the URDF).

Use `ros2 run tf2_tools view_frames` after launch to confirm there are no broken parents.

## Launch ordering (recommended)

### Terminal A — camera / bag / simulation

- Gazebo (or real robot) publishing `/camera/image_raw` (and ideally **laser** or **depth** for Nav2 costmaps).
- If you replay a video file:

  ```bash
  ros2 launch vslam_nav2_integration image_from_video.launch.py \
    video_path:=/absolute/path/to/room.mp4
  ```

### Terminal B — StellaVSLAM

```bash
ros2 launch vslam_nav2_integration stella_slam_only.launch.py \
  vocab:=/absolute/path/to/orb_vocab.fbow \
  camera_config:=/absolute/path/to/your_camera.yaml \
  map_db_out:=/tmp/stella_map.msg \
  publish_tf:=true
```

Start with `publish_tf:=false` while debugging image intrinsics; once poses look stable in the Stella viewer, enable TF and check RViz.

### Terminal C — online 2D mapping for Nav2 (example pattern)

- **Option 1 (common in labs):** `slam_toolbox` async online mode subscribing to `/scan`, publishing `/map` + TF compatible with Nav2.
- **Option 2:** Depth camera → `pointcloud_to_laserscan` → obstacle layer (document your exact topic names).

### Terminal D — Nav2

Use your course’s TurtleBot + maze launch, for example patterns from `nav2_bringup` and TurtleBot simulation packages. Ensure:

- `global_costmap.global_frame` is `map`.
- `amcl` may be **disabled** or unused when another localization stack owns `map`→`odom`; many teams still run AMCL if `/map` comes from slam_toolbox—**avoid double global corrections** by choosing one owner of large drift corrections.

Document the choice you made in your report.

## Topics checklist

| Topic / TF | Purpose |
|------------|---------|
| `/camera/image_raw` | Stella image subscription |
| `/tf`, `/tf_static` | Frame tree |
| `/map` (optional) | Occupancy for Nav2 if using slam_toolbox / map_server |
| `/scan` or depth topics | Obstacle data for costmaps |

## Failure modes (for students)

- **Wrong camera model** in YAML (fisheye vs pinhole) → immediate divergence.
- **Textureless walls** → few ORB features; add posters or improve lighting.
- **Fast motion / motion blur** → tracking loss; reduce speed.
- **No laser/depth** → Nav2 has nothing to inflate into obstacles even if pose is perfect.

## Demo video (deliverable)

Record a single take showing: RViz TF tree, Stella map/pose (or socket viewer), Nav2 goal sent in the maze, and a short narration of which estimator owns which state. Place the link in the root `README.md`.

## References

- [StellaVSLAM ROS2 package](https://stella-cv.readthedocs.io/en/latest/ros2_package.html)
- [Nav2 documentation](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
