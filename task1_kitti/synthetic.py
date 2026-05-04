"""Synthetic forward-motion grayscale sequences for offline tests without KITTI."""

from __future__ import annotations

from typing import List

import cv2
import numpy as np


def make_synthetic_gray_sequence(
    n_frames: int = 40,
    width: int = 640,
    height: int = 180,
    n_points: int = 400,
    seed: int = 0,
) -> tuple[List[np.ndarray], np.ndarray, np.ndarray]:
    """
    Camera translates along +X; random 3D points in front; render as white blobs.

    Returns (grayscale uint8 images, K 3x3, gt_positions (n_frames, 3) camera centers in world).
    """
    rng = np.random.default_rng(seed)
    K = np.array(
        [[400.0, 0.0, width / 2.0], [0.0, 400.0, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

    # Static scene in world frame; camera moves on X axis, looks toward +Z.
    Xw = np.stack(
        [
            rng.uniform(-2, 2, size=n_points),
            rng.uniform(-1, 1, size=n_points),
            rng.uniform(4.0, 8.0, size=n_points),
        ],
        axis=1,
    )

    images: List[np.ndarray] = []
    gt_positions = np.zeros((n_frames, 3), dtype=np.float64)

    for i in range(n_frames):
        img = np.zeros((height, width), dtype=np.uint8)
        tx = 0.06 * i
        ty = 0.03 * np.sin(i * 0.35)
        gt_positions[i] = np.array([tx, ty, 0.0], dtype=np.float64)
        # Camera translates with mild lateral motion for parallax (avoids pure forward degeneracy).
        Xc = Xw.copy()
        Xc[:, 0] -= tx
        Xc[:, 1] -= ty
        uv = (K @ Xc.T).T
        uv = uv[:, :2] / (Xc[:, 2:3] + 1e-9)
        for (u, v) in uv:
            u0, v0 = int(round(u)), int(round(v))
            if 2 <= u0 < width - 2 and 2 <= v0 < height - 2:
                cv2.circle(img, (u0, v0), 3, 255, -1)
        images.append(img)

    return images, K, gt_positions
