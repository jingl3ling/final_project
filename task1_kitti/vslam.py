"""Monocular visual odometry: ORB, essential matrix, triangulation, pose chaining."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class VOConfig:
    max_features: int = 2000
    orb_scale_factor: float = 1.2
    orb_n_levels: int = 8
    match_ratio_test: float = 0.75
    ransac_prob: float = 0.999
    # Max epipolar distance in pixels for findEssentialMat RANSAC
    ransac_threshold: float = 2.5


class MonocularVO:
    """
    Two-view geometry between consecutive frames, scale from ground truth when available.

    Pose convention: T_wc is 4x4 mapping homogeneous camera coordinates to world:
    p_world_hom = T_wc @ p_cam_hom (same as KITTI odometry pose files).
    """

    def __init__(self, K: np.ndarray, config: Optional[VOConfig] = None) -> None:
        self.K = np.array(K, dtype=np.float64, copy=True)
        self.config = config or VOConfig()
        self.orb = cv2.ORB_create(
            nfeatures=self.config.max_features,
            scaleFactor=self.config.orb_scale_factor,
            nlevels=self.config.orb_n_levels,
        )
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def detect_and_compute(self, gray: np.ndarray) -> Tuple[List[cv2.KeyPoint], np.ndarray]:
        return self.orb.detectAndCompute(gray, None)

    def match_frames(
        self,
        desc_prev: np.ndarray,
        desc_curr: np.ndarray,
    ) -> List[cv2.DMatch]:
        if desc_prev is None or desc_curr is None or len(desc_prev) < 8 or len(desc_curr) < 8:
            return []
        matches = self.bf.knnMatch(desc_curr, desc_prev, k=2)
        good: List[cv2.DMatch] = []
        ratio = self.config.match_ratio_test
        for pair in matches:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < ratio * n.distance:
                good.append(m)
        return good

    @staticmethod
    def _pixel_to_normalized(pts: np.ndarray, K: np.ndarray) -> np.ndarray:
        Kinv = np.linalg.inv(K)
        hom = np.hstack([pts, np.ones((pts.shape[0], 1))])
        n = (Kinv @ hom.T).T
        return n[:, :3]

    def estimate_pose_and_triangulate(
        self,
        kp_prev: List[cv2.KeyPoint],
        kp_curr: List[cv2.KeyPoint],
        matches: List[cv2.DMatch],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns (R, t_unit, pts_prev_norm, pts_curr_norm, points_3d_in_prev_cam).

        t_unit is unit-norm translation direction from recoverPose (monocular scale ambiguous).
        points_3d_in_prev_cam are triangulated in the previous camera frame (Nx3).
        """
        if len(matches) < 8:
            raise ValueError("Not enough matches for pose estimation")

        pts_prev = np.float32([kp_prev[m.trainIdx].pt for m in matches])
        pts_curr = np.float32([kp_curr[m.queryIdx].pt for m in matches])

        E, mask_e = cv2.findEssentialMat(
            pts_prev,
            pts_curr,
            self.K,
            method=cv2.RANSAC,
            prob=self.config.ransac_prob,
            threshold=self.config.ransac_threshold,
        )
        if E is None or E.shape != (3, 3):
            raise RuntimeError("findEssentialMat failed")

        # Do not pass mask_e into recoverPose: it can disagree with cheirality and zero out inliers.
        _, R, t, mask_rp = cv2.recoverPose(E, pts_prev, pts_curr, self.K)
        inlier = mask_rp.ravel() > 0

        pts_prev = pts_prev[inlier]
        pts_curr = pts_curr[inlier]

        # World = previous camera frame: P0 = K[I|0], P1 = K[R|t] with recoverPose
        # convention p_curr ~ R @ p_prev + t (normalized coords).
        P0 = self.K @ np.hstack([np.eye(3), np.zeros((3, 1))])
        P1 = self.K @ np.hstack([R, t.reshape(3, 1)])

        # triangulatePoints expects undistorted pixel coordinates as 2xN
        x1 = np.ascontiguousarray(pts_prev.T[:2, :], dtype=np.float64)
        x2 = np.ascontiguousarray(pts_curr.T[:2, :], dtype=np.float64)
        P0 = np.ascontiguousarray(P0, dtype=np.float64)
        P1 = np.ascontiguousarray(P1, dtype=np.float64)
        X_h = cv2.triangulatePoints(P0, P1, x1, x2)
        X_h /= X_h[3:4] + 1e-12
        X = X_h[:3].T

        n_prev = self._pixel_to_normalized(pts_prev, self.K)
        n_curr = self._pixel_to_normalized(pts_curr, self.K)
        return R, t, n_prev, n_curr, X

    @staticmethod
    def relative_transform_world(R: np.ndarray, t: np.ndarray) -> np.ndarray:
        """
        Build M such that p_curr_h = M @ p_prev_h (OpenCV recoverPose convention:
        normalized points satisfy p_curr ~ R @ p_prev + t).
        """
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t.ravel()
        return T

    @staticmethod
    def compose_world_poses(T_w_prev: np.ndarray, M_prev_to_curr: np.ndarray) -> np.ndarray:
        """Update world-from-camera pose: T_w_curr = T_w_prev @ inv(M_prev_to_curr)."""
        return T_w_prev @ np.linalg.inv(M_prev_to_curr)

    def run_sequence(
        self,
        gray_images: List[np.ndarray],
        gt_positions: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, List[np.ndarray], List[np.ndarray]]:
        """
        Run VO over a list of grayscale images.

        If gt_positions (N,3) is provided, scale each step translation to match GT baseline.

        Returns (positions_world Nx3, list of 4x4 T_wc, optional list of 3D points per step).
        """
        if len(gray_images) < 2:
            raise ValueError("Need at least two images")

        T_wc = np.eye(4, dtype=np.float64)
        positions: List[np.ndarray] = [T_wc[:3, 3].copy()]
        poses_out: List[np.ndarray] = [T_wc.copy()]
        landmarks_per_step: List[np.ndarray] = []

        kp_prev, desc_prev = self.detect_and_compute(gray_images[0])

        for i in range(1, len(gray_images)):
            kp_curr, desc_curr = self.detect_and_compute(gray_images[i])
            matches = self.match_frames(desc_prev, desc_curr)
            if len(matches) < 8:
                # Skip frame: repeat previous pose
                positions.append(positions[-1].copy())
                poses_out.append(poses_out[-1].copy())
                landmarks_per_step.append(np.zeros((0, 3)))
                kp_prev, desc_prev = kp_curr, desc_curr
                continue

            R, t_unit, _, _, X = self.estimate_pose_and_triangulate(kp_prev, kp_curr, matches)
            t_scaled = t_unit.copy()

            if gt_positions is not None and i < len(gt_positions):
                d_gt = np.linalg.norm(gt_positions[i] - gt_positions[i - 1])
                d_est = np.linalg.norm(t_unit)
                scale = d_gt / (d_est + 1e-9)
                t_scaled = t_unit * scale

            T_before = T_wc.copy()
            M = self.relative_transform_world(R, t_scaled)
            T_wc = self.compose_world_poses(T_wc, M)
            positions.append(T_wc[:3, 3].copy())
            poses_out.append(T_wc.copy())

            # Triangulated points are expressed in the *previous* camera frame.
            R_wp = T_before[:3, :3]
            t_wp = T_before[:3, 3]
            Xw = (R_wp @ X.T + t_wp.reshape(3, 1)).T
            landmarks_per_step.append(Xw)

            kp_prev, desc_prev = kp_curr, desc_curr

        return np.stack(positions, axis=0), poses_out, landmarks_per_step
