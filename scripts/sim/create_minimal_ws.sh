#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
TARGET_WS="${1:-${HOME}/ws_guosai_sim}"
SRC_DIR="${TARGET_WS}/src"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi

mkdir -p "${SRC_DIR}"

if [[ ! -e "${SRC_DIR}/CMakeLists.txt" ]]; then
  catkin_init_workspace "${SRC_DIR}"
fi

packages=(
  "src/utils/catkin_simple"
  "src/utils/cmake_utils"
  "src/utils/pose_utils"
  "src/utils/uav_utils"
  "src/utils/quadrotor_msgs"
  "src/planner/traj_utils"
  "src/planner/path_searching"
  "src/planner/plan_env"
  "src/planner/bspline_opt"
  "src/planner/plan_manage"
  "src/uav_simulator/map_generator"
  "src/uav_simulator/fake_drone"
  "src/uav_simulator/odom_visualization"
  "src/uav_simulator/local_sensing"
)

for rel in "${packages[@]}"; do
  src_path="${REPO_ROOT}/${rel}"
  link_name="${SRC_DIR}/$(basename "${rel}")"
  ln -sfn "${src_path}" "${link_name}"
done

echo "Minimal simulation workspace prepared at: ${TARGET_WS}"
echo "Next step: bash ${REPO_ROOT}/scripts/sim/build_minimal_ws.sh ${TARGET_WS}"
