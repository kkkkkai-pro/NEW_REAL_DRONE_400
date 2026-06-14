#!/usr/bin/env bash

set -euo pipefail

SEARCH_ROOT="${1:-src}"

echo "[world and model files]"
find "${SEARCH_ROOT}" -type f \( -name "*.world" -o -name "*.sdf" -o -name "*.urdf" -o -name "*.xacro" \)

echo
echo "[gazebo-related launch references]"
if command -v rg >/dev/null 2>&1; then
  rg -n "gazebo|gzserver|gzclient|gazebo_ros|spawn_model|spawn_entity" "${SEARCH_ROOT}"
else
  grep -RInE "gazebo|gzserver|gzclient|gazebo_ros|spawn_model|spawn_entity" "${SEARCH_ROOT}"
fi
