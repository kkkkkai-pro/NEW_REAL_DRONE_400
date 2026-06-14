#!/usr/bin/env bash

set -euo pipefail

TARGET_WS="${1:-${HOME}/ws_guosai_sim}"

source /opt/ros/noetic/setup.bash
source "${TARGET_WS}/devel/setup.bash"

cd "${TARGET_WS}"
roslaunch ego_planner single_run_in_sim.launch
