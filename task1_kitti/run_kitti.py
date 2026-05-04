#!/usr/bin/env python3
"""Run monocular VO on a KITTI odometry sequence (requires dataset on disk)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import cv2

from task1_kitti.kitti_loader import KittiSequence, camera_positions_from_poses, default_kitti_layout
from task1_kitti.metrics import absolute_trajectory_error
from task1_kitti.plotting import plot_trajectory_plane
from task1_kitti.vslam import MonocularVO, VOConfig


def main() -> None:
    p = argparse.ArgumentParser(description="KITTI monocular VO demo")
    p.add_argument("--dataset", type=Path, required=True, help="KITTI odometry root (contains sequences/, poses/)")
    p.add_argument("--seq", type=str, default="00", help="Sequence id, e.g. 00")
    p.add_argument("--max-frames", type=int, default=200, help="Limit frames for a quick run")
    p.add_argument("--out", type=Path, default=Path("outputs/kitti_traj.png"))
    p.add_argument(
        "--image-dir",
        type=str,
        default="image_2",
        help="Image subfolder under sequences/XX (image_2=color left, image_0=gray left)",
    )
    p.add_argument(
        "--calib-key",
        type=str,
        default="P2:",
        help="calib.txt line prefix for intrinsics (P2: with color image_2, P0: with gray image_0)",
    )
    args = p.parse_args()

    seq_root, poses_path = default_kitti_layout(args.dataset, args.seq, train_poses=True)
    kitti = KittiSequence(
        seq_root,
        poses_path,
        image_dir_name=args.image_dir,
        calib_key=args.calib_key,
    )
    if not kitti.gt_poses:
        raise SystemExit("No ground-truth poses found; check --dataset and --seq")

    vo = MonocularVO(kitti.K, VOConfig())
    n = min(len(kitti), args.max_frames, kitti.num_gt_poses())
    grays = []
    for i in range(n):
        img = cv2.imread(str(kitti.image_path(i)), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise SystemExit(f"Failed to read {kitti.image_path(i)}")
        grays.append(img)

    gt_pos = camera_positions_from_poses(kitti.gt_poses[:n])
    est_pos, _, _ = vo.run_sequence(grays, gt_positions=gt_pos)
    rmse, _ = absolute_trajectory_error(est_pos, gt_pos)
    print(f"ATE RMSE (Sim(3) aligned): {rmse:.4f} m over {n} frames")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    plot_trajectory_plane(
        est_pos,
        gt_pos,
        axes=(0, 2),
        axis_labels=("x [m]", "z [m]"),
        title=f"KITTI seq {args.seq} — VO vs ground truth",
        out_path=args.out,
    )
    print(f"Saved plot to {args.out}")


if __name__ == "__main__":
    main()
