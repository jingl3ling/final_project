#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if [[ ! -d .venv ]]; then
  python3 -m venv .venv
fi
.venv/bin/pip install -q -r requirements-task1.txt
export PYTHONPATH="$ROOT"
.venv/bin/python -c "from task1_kitti.synthetic import make_synthetic_gray_sequence; from task1_kitti.vslam import MonocularVO, VOConfig; from task1_kitti.metrics import absolute_trajectory_error; imgs,K,gt=make_synthetic_gray_sequence(30); vo=MonocularVO(K, VOConfig()); p,_,_=vo.run_sequence(imgs, gt_positions=gt); r,_=absolute_trajectory_error(p, gt[:len(p)]); print('OK ATE RMSE', r)"
