#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${workspace_dir}/env.sh"

output_prefix="${1:-${HOME}/.ros/stereo_nav/maps/indoor}"
mkdir -p "$(dirname "${output_prefix}")"

ros2 run nav2_map_server map_saver_cli \
  -f "${output_prefix}" \
  --ros-args -p map_subscribe_transient_local:=true

echo "Saved occupancy map to ${output_prefix}.yaml and ${output_prefix}.pgm"
echo "RTAB-Map's primary database remains at ${HOME}/.ros/stereo_nav/rtabmap.db"
