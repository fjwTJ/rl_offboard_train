#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <bag_path> [metrics_json_out]"
  exit 1
fi

BAG_PATH="$1"
JSON_OUT="${2:-}"

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "Bag path not found: ${BAG_PATH}"
  exit 1
fi

METRICS_CMD=(ros2 run rl_agent rl_metrics_node)
if [[ -n "${JSON_OUT}" ]]; then
  METRICS_CMD+=(--ros-args -p save_json_path:="${JSON_OUT}")
fi

echo "[replay_and_eval] start metrics node"
"${METRICS_CMD[@]}" &
METRICS_PID=$!

cleanup() {
  if ps -p "${METRICS_PID}" >/dev/null 2>&1; then
    kill "${METRICS_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

sleep 1
echo "[replay_and_eval] replay bag: ${BAG_PATH}"
ros2 bag play "${BAG_PATH}" --clock
sleep 1
cleanup
wait "${METRICS_PID}" 2>/dev/null || true
echo "[replay_and_eval] done"
