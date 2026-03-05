#!/usr/bin/env bash
set -euo pipefail

OUT_BASE="${1:-$HOME/rl_bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_BASE}/run_${STAMP}"

mkdir -p "${OUT_BASE}"

TOPICS=(
  /perception/target_xyz
  /perception/target_lost
  /fmu/out/vehicle_odometry
  /uav/cmd_vel_pid
  /uav/cmd_vel_rl
  /uav/cmd_vel_body
  /uav/cmd_active_source
  /rl/obs
  /rl/reward
  /rl/done
  /rl/info
  /rl/step_count
  /tf
  /tf_static
)

echo "[record_rl_bag] output: ${OUT_DIR}"
echo "[record_rl_bag] topics: ${#TOPICS[@]}"
exec ros2 bag record -o "${OUT_DIR}" "${TOPICS[@]}"
