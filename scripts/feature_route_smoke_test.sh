#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
source "${project_dir}/env.sh"

smoke_dir="$(mktemp -d -t stereo_nav_route_XXXXXX)"
database_path="${smoke_dir}/rtabmap.db"
launch_log="${smoke_dir}/mapping.log"
audit_log="${smoke_dir}/runtime_audit.log"
trajectory_log="${smoke_dir}/trajectory.log"

setsid ros2 launch stereo_nav_bringup mapping.launch.py \
  gui:=false headless:=true rviz:=false new_map:=true \
  database_path:="${database_path}" >"${launch_log}" 2>&1 &
launch_pid=$!

cleanup() {
  trap - EXIT
  set +e
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null
    for _ in $(seq 1 24); do
      kill -0 -- "-${launch_pid}" 2>/dev/null || break
      sleep 0.25
    done
  fi
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -KILL -- "-${launch_pid}" 2>/dev/null
  fi
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

ready=false
for _ in $(seq 1 60); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -qx '/stereo/points2'; then
    ready=true
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Mapping launch exited early. Log: ${launch_log}" >&2
    exit 1
  fi
  sleep 1
done
if [[ "${ready}" != true ]]; then
  echo "Timed out waiting for the stereo pipeline. Log: ${launch_log}" >&2
  exit 1
fi

sleep 4
"${script_dir}/stereo_pair_diagnostic.py"

# The route reproduces the reported failure: leave the spawn board, turn north,
# drive to y~=2.2 m, rotate in place, and return.  It exercises about 11 m of
# low-texture floor plus a 180-degree pure-rotation failure case.
ros2 run stereo_nav_tests runtime_audit \
  --duration 75 \
  --minimum-valid-odom-fraction 0.95 \
  --maximum-invalid-odom-seconds 1.0 >"${audit_log}" 2>&1 &
audit_pid=$!
ros2 run stereo_nav_tests trajectory_evaluator \
  --duration 75 --minimum-distance 10.0 \
  --rmse-limit 0.30 --final-limit 0.25 >"${trajectory_log}" 2>&1 &
trajectory_pid=$!

timeout 5.4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: 0.30}}' >/dev/null 2>&1 || [[ $? -eq 124 ]]
timeout 26 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.22}}' >/dev/null 2>&1 || [[ $? -eq 124 ]]
timeout 10.5 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{angular: {z: 0.30}}' >/dev/null 2>&1 || [[ $? -eq 124 ]]
timeout 26 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.22}}' >/dev/null 2>&1 || [[ $? -eq 124 ]]
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null

audit_status=0
trajectory_status=0
wait "${audit_pid}" || audit_status=$?
wait "${trajectory_pid}" || trajectory_status=$?
cat "${audit_log}"
cat "${trajectory_log}"
"${script_dir}/stereo_pair_diagnostic.py"

if (( audit_status != 0 || trajectory_status != 0 )); then
  echo "Feature-route smoke test failed. Artifacts: ${smoke_dir}" >&2
  exit 1
fi
if grep -Eqi 'process has died|segmentation fault|terminate called|tf_repeated_data' "${launch_log}"; then
  echo "A crash or TF conflict was found. Log: ${launch_log}" >&2
  exit 1
fi

echo "Feature-route smoke test passed. Artifacts: ${smoke_dir}"
