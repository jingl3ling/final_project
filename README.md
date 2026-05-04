# Final Project — Visual SLAM (Project 2)

Part 1 result in outputs

Monorepo for **Task 1** (Python VO / VSLAM on KITTI + synthetic demo), **Task 2** (StellaVSLAM workflow + calibration template), and **Task 3** (ROS 2 overlay launches + Nav2 tutorial).

## Repository layout

| Path | Description |
|------|-------------|
| [`task1_kitti/`](task1_kitti/) | ORB, essential matrix, triangulation, pose chaining, ATE metrics |
| [`notebooks/project2_task1.ipynb`](notebooks/project2_task1.ipynb) | Executed notebook (Task 1 + optional KITTI) |
| [`docs/task2_stella.md`](docs/task2_stella.md) | StellaVSLAM Docker, recording, calibration, UVC / video |
| [`config/stella_pinhole_template.yaml`](config/stella_pinhole_template.yaml) | Pinhole intrinsics template for Stella |
| [`docs/nav2_stella_tutorial.md`](docs/nav2_stella_tutorial.md) | Task 3: Nav2 integration (class tutorial) |
| [`ros2_ws/src/vslam_nav2_integration/`](ros2_ws/src/vslam_nav2_integration/) | Example launch files |

## Task 1 — Python environment

```bash
cd final_project
python3 -m venv .venv
.venv/bin/pip install -r requirements-task1.txt
export PYTHONPATH="$(pwd)"
./scripts/check_task1.sh
```

### KITTI odometry CLI (after downloading the dataset)

`--dataset` must be the folder that **directly** contains `sequences/` and `poses/` (often `.../dataset` after unzipping).

**Color** (default: `image_2` + `P2:`):

```bash
export PYTHONPATH="$(pwd)"
.venv/bin/python task1_kitti/run_kitti.py --dataset /path/to/kitti_odometry/dataset --seq 00 --max-frames 200
```

**Grayscale** odometry (`image_0` + `P0:` in `calib.txt`):

```bash
export PYTHONPATH="$(pwd)"
.venv/bin/python task1_kitti/run_kitti.py \
  --dataset /path/to/kitti_odometry/dataset \
  --seq 00 \
  --max-frames 200 \
  --image-dir image_0 \
  --calib-key "P0:"
```

### Notebook (re-)execution

Jupyter may try to write to `~/.ipython`. From the repo root:

```bash
mkdir -p .ipython .jupyter
IPYTHONDIR="$(pwd)/.ipython" JUPYTER_CONFIG_DIR="$(pwd)/.jupyter" \
  .venv/bin/jupyter nbconvert --to notebook --execute notebooks/project2_task1.ipynb --inplace
```

Optional KITTI section inside the notebook:

```bash
export KITTI_ROOT=/path/to/kitti_odometry_dataset_root
```

Full notebook extras: `pip install -r requirements-notebook.txt`.

## Task 2 — StellaVSLAM + your space

Follow [`docs/task2_stella.md`](docs/task2_stella.md). Fill [`config/stella_pinhole_template.yaml`](config/stella_pinhole_template.yaml) with your calibration.

**Deliverables to add yourself:** screenshots / screen recordings, your YouTube URL (place it below).

### YouTube (Task 2)

- Replace with your public link: `https://www.youtube.com/watch?v=REPLACE_ME`

## Task 3 — ROS 2 + Nav2

1. Build `stella_vslam` / `stella_vslam_ros` per upstream docs on Ubuntu 22.04 + ROS 2 Humble.
2. Build this workspace package: see [`docs/nav2_stella_tutorial.md`](docs/nav2_stella_tutorial.md).
3. Record your maze navigation demo and add the link below.

### Demo video (Task 3)

- Replace with your public link: `https://www.youtube.com/watch?v=REPLACE_ME_TASK3`

## Course documentation index

- [https://aegean.ai/llms.txt](https://aegean.ai/llms.txt)

## License

Course / educational use. Third-party components (KITTI, StellaVSLAM, Nav2) follow their respective licenses.
