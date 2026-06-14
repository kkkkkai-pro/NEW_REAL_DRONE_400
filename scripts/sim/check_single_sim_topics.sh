#!/usr/bin/env bash

set -euo pipefail

TARGET_WS="${1:-${HOME}/ws_guosai_sim}"

source /opt/ros/noetic/setup.bash
source "${TARGET_WS}/devel/setup.bash"

echo "[topics]"
rostopic list | egrep 'map_generator/global_cloud|drone_0_visual_slam/odom|drone_0_pcl_render_node/cloud|drone_0_planning/bspline' || true

echo
echo "[odom sample]"
rostopic echo -n 1 /drone_0_visual_slam/odom
