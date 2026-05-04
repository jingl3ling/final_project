"""Load KITTI odometry sequences: images, calibration, and ground-truth poses."""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional

import numpy as np


def _parse_calib_line(line: str) -> np.ndarray:
    parts = line.split()[1:]
    return np.array([float(x) for x in parts], dtype=np.float64).reshape(3, 4)


def load_projection_matrix(calib_path: Path, key: str = "P2:") -> np.ndarray:
    """Read a 3x4 projection matrix from calib.txt (default P2 = left color camera)."""
    with open(calib_path, "r", encoding="utf-8") as f:
        for raw in f:
            if raw.startswith(key):
                return _parse_calib_line(raw)
    raise KeyError(f"{key} not found in {calib_path}")


def projection_to_intrinsics(P: np.ndarray) -> np.ndarray:
    """Extract 3x3 intrinsics K from KITTI rectified projection P = K [R|t]."""
    # For odometry sequences, the left 3x3 of P2 is the rectified camera K.
    return np.array(P[:, :3], dtype=np.float64, copy=True)


def load_poses_file(poses_path: Path) -> List[np.ndarray]:
    """Load KITTI odometry ground-truth poses (4x4, camera to world)."""
    poses: List[np.ndarray] = []
    with open(poses_path, "r", encoding="utf-8") as f:
        for line in f:
            vals = np.array([float(x) for x in line.split()], dtype=np.float64)
            if vals.size == 0:
                continue
            if vals.size != 12:
                raise ValueError(f"Expected 12 values per pose line, got {vals.size}")
            T = np.eye(4, dtype=np.float64)
            T[:3, :] = vals.reshape(3, 4)
            poses.append(T)
    return poses


def camera_positions_from_poses(poses: List[np.ndarray]) -> np.ndarray:
    """
    Camera center in world for each pose p_w = T * p_c with p_c = 0 -> translation column.

    KITTI odometry devkit stores rows of the camera pose as 3x4; the homogeneous
    transform maps homogeneous camera coordinates to homogeneous world coordinates.
    """
    out = np.zeros((len(poses), 3), dtype=np.float64)
    for i, T in enumerate(poses):
        # Camera origin in world: T @ [0,0,0,1]^T
        out[i] = T[:3, 3]
    return out


class KittiSequence:
    """One KITTI odometry sequence: left color images, K, and optional GT poses."""

    def __init__(
        self,
        sequence_root: Path,
        poses_file: Optional[Path] = None,
        image_dir_name: str = "image_2",
        calib_key: str = "P2:",
    ) -> None:
        self.sequence_root = Path(sequence_root)
        self.image_dir = self.sequence_root / image_dir_name
        calib_path = self.sequence_root / "calib.txt"
        if not calib_path.is_file():
            raise FileNotFoundError(calib_path)
        P = load_projection_matrix(calib_path, calib_key)
        self.K = projection_to_intrinsics(P)
        self._image_paths = sorted(
            p for p in self.image_dir.glob("*.png") if p.is_file()
        )
        if not self._image_paths:
            raise FileNotFoundError(f"No PNG images under {self.image_dir}")

        self.gt_poses: Optional[List[np.ndarray]] = None
        if poses_file is not None and Path(poses_file).is_file():
            self.gt_poses = load_poses_file(Path(poses_file))

    def __len__(self) -> int:
        return len(self._image_paths)

    def image_path(self, index: int) -> Path:
        return self._image_paths[index]

    def num_gt_poses(self) -> int:
        return len(self.gt_poses) if self.gt_poses else 0


def default_kitti_layout(
    dataset_root: Path, sequence_id: str, train_poses: bool = True
) -> tuple[Path, Optional[Path]]:
    """
    Typical layout:
      dataset_root/sequences/XX/{image_2,calib.txt}
      dataset_root/poses/XX.txt  (training sequences only)
    """
    seq = dataset_root / "sequences" / sequence_id
    poses: Optional[Path] = None
    if train_poses:
        p = dataset_root / "poses" / f"{sequence_id}.txt"
        if p.is_file():
            poses = p
    return seq, poses
