#!/usr/bin/env bash

set -euo pipefail

TARGET_WS="${1:-${HOME}/ws_guosai_sim}"
X="${2:-5.0}"
Y="${3:-0.0}"
Z="${4:-1.0}"
TOPIC="${5:-/move_base_simple/goal}"

source /opt/ros/noetic/setup.bash
source "${TARGET_WS}/devel/setup.bash"

rostopic pub -1 "${TOPIC}" geometry_msgs/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {w: 1.0}}}"
