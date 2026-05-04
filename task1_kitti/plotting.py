"""Trajectory plots for Task 1."""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np


def plot_trajectory_plane(
    est: np.ndarray,
    ref: Optional[np.ndarray],
    axes: Tuple[int, int] = (0, 2),
    axis_labels: Tuple[str, str] = ("x [m]", "z [m]"),
    title: str = "Camera trajectory",
    out_path: Optional[Path] = None,
    labels: Tuple[str, str] = ("Estimated", "Ground truth"),
) -> None:
    """Plot two components of each (N,3) trajectory (default KITTI driving plane x–z)."""
    a0, a1 = axes
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(est[:, a0], est[:, a1], "-b", linewidth=1.5, label=labels[0])
    if ref is not None:
        n = min(len(est), len(ref))
        ax.plot(ref[:n, a0], ref[:n, a1], "--r", linewidth=1.2, label=labels[1])
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel(axis_labels[0])
    ax.set_ylabel(axis_labels[1])
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    if out_path is not None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
