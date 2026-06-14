#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_WS="${1:-${HOME}/ws_guosai_sim}"

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi

bash "${SCRIPT_DIR}/create_minimal_ws.sh" "${TARGET_WS}"

cd "${TARGET_WS}"
catkin_make -DCMAKE_BUILD_TYPE=Release

echo
echo "Build finished."
echo "Source this workspace before running:"
echo "  source ${TARGET_WS}/devel/setup.bash"
