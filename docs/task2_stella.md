# Task 2: StellaVSLAM — reference dataset, your space, and calibration

This document supports the course deliverables: Docker validation on the AIST equirectangular example, recording your own indoor video, camera calibration, and publishing a YouTube walkthrough.

## 1. Validate with Docker (AIST living lab)

Follow the official [StellaVSLAM examples](https://stella-cv.readthedocs.io/en/latest/example.html) for the **equirectangular** pipeline and the `aist_living_lab_1` video.

Typical flow (check upstream docs for exact image tags):

1. Install Docker and ensure your user can run `docker` without extra prompts.
2. Pull or build the StellaVSLAM Docker image from the project instructions.
3. Mount a local folder containing the dataset and vocabulary (`orb_vocab.fbow`).
4. Run the viewer / SLAM command exactly as in the **Equirectangular** section of the examples page.
5. Capture **screenshots or a screen recording** of the reconstruction and trajectory for your report.

## 2. Record your own indoor space

- Walk **slowly**; favor **loop closures** (revisit the same walls from new angles).
- Turn on lights; avoid motion blur and extreme rolling shutter.
- Prefer **fixed focal length** (many laptop webcams zoom digitally—disable zoom).
- Export **raw resolution** your calibration used (do not crop or re-scale after calibration without updating intrinsics).

## 3. Camera calibration (replace YAML with your intrinsics)

Use the course **Camera Calibration** material (Zhang’s method, OpenCV `calibrateCamera` or `fisheye.calibrateCamera` for wide lenses).

After calibration you need at minimum:

- Focal lengths `fx`, `fy` and principal point `cx`, `cy`
- Radial (and tangential) distortion coefficients in the **same convention** Stella’s YAML expects

## 4. YAML configuration

- **Equirectangular** template: only if your lens is omnidirectional; start from `example/aist/equirectangular.yaml` in the StellaVSLAM repository.
- **Standard USB webcam:** use a **pinhole** example under `example/` (e.g. EuRoC / monocular pinhole templates) and paste your `Camera` matrix and distortion vector. Wrong camera models dominate focal length error.

A fill-in template for a pinhole + OpenCV radial model lives in this repo at [`config/stella_pinhole_template.yaml`](../config/stella_pinhole_template.yaml). Copy it beside your vocabulary and video, adjust `mask_rect` / resolution if needed, and pass paths to the Stella binary or ROS node.

## 5. UVC camera with ROS 2 (optional path)

From [stella_vslam ROS2 package](https://stella-cv.readthedocs.io/en/latest/ros2_package.html):

- `ros2 run image_tools cam2image`
- `ros2 run image_transport republish raw in:=image raw out:=/camera/image_raw`
- `ros2 run stella_vslam_ros run_slam -v ... -c ...`

For a **video file** instead of live UVC:

- `ros2 run image_publisher image_publisher_node /path/to/video.mp4 --ros-args --remap /image_raw:=/camera/image_raw`

## 6. YouTube deliverable

Upload a short video that:

1. Explains your capture setup and calibration choices.
2. Shows StellaVSLAM running on the **reference** example (brief clip).
3. Shows your **own** reconstruction (map / trajectory / viewer).

Add the public URL to the root [`README.md`](../README.md) under **Deliverables**.
