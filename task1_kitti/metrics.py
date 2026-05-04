"""Trajectory alignment (Umeyama Sim(3)) and absolute trajectory error."""

from __future__ import annotations

import numpy as np


def umeyama_alignment(
    src: np.ndarray,
    dst: np.ndarray,
    with_scale: bool = True,
) -> tuple[np.ndarray, np.ndarray, float]:
    """
    Least-squares Sim(3) / SE(3) alignment: dst ≈ s * R @ src + t.

    src, dst: (N, 3) point sets (e.g. estimated vs ground-truth camera positions).

    Returns (R, t, scale) with R (3,3), t (3,), scale scalar (1.0 if with_scale=False).
    """
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError("src and dst must be (N, 3) with same shape")
    n = src.shape[0]
    if n < 3:
        raise ValueError("Need at least 3 poses for Umeyama alignment")

    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    xs = src - mu_s
    xd = dst - mu_d

    cov = (xd.T @ xs) / n
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt

    if with_scale:
        var_s = (xs**2).sum() / n
        scale = np.trace(R @ cov) / (var_s + 1e-12)
    else:
        scale = 1.0

    t = mu_d - scale * (R @ mu_s)
    return R, t, float(scale)


def absolute_trajectory_error(est: np.ndarray, ref: np.ndarray) -> tuple[float, np.ndarray]:
    """
    RMSE of position errors after optimal Sim(3) alignment (monocular-friendly).

    est, ref: (N, 3) trajectories (same length).

    Returns (rmse, per_frame_errors) where per_frame_errors is (N,) Euclidean norms.
    """
    if est.shape != ref.shape:
        raise ValueError("est and ref must have identical shape")
    R, t, s = umeyama_alignment(est, ref, with_scale=True)
    aligned = (s * (R @ est.T).T + t)
    err = np.linalg.norm(aligned - ref, axis=1)
    rmse = float(np.sqrt((err**2).mean()))
    return rmse, err
