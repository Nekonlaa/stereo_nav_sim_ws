#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
source "${project_dir}/env.sh"

database_path="${1:-${HOME}/.ros/stereo_nav/rtabmap.db}"
audit_duration="${2:-10}"
if [[ ! -s "${database_path}" ]]; then
  echo "Navigation smoke test requires a non-empty database: ${database_path}" >&2
  exit 1
fi

smoke_dir="$(mktemp -d -t stereo_nav_navigation_XXXXXX)"
launch_log="${smoke_dir}/navigation.log"

setsid ros2 launch stereo_nav_bringup navigation.launch.py \
  gui:=false headless:=true rviz:=false moving_obstacle:=false \
  database_path:="${database_path}" >"${launch_log}" 2>&1 &
launch_pid=$!

cleanup() {
  trap - EXIT
  set +e
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null
    for _ in $(seq 1 20); do
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
for _ in $(seq 1 90); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "Navigation launch exited early. Log: ${launch_log}" >&2
    exit 1
  fi
  if grep -Eqi 'process has died|segmentation fault|terminate called' "${launch_log}"; then
    echo "A navigation process exited while waiting for activation. Log: ${launch_log}" >&2
    exit 1
  fi
  if timeout 3 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q 'active' && \
     timeout 3 ros2 lifecycle get /controller_server 2>/dev/null | grep -q 'active' && \
     timeout 3 ros2 lifecycle get /planner_server 2>/dev/null | grep -q 'active' && \
     timeout 3 ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose'; then
    ready=true
    break
  fi
  sleep 1
done

if [[ "${ready}" != true ]]; then
  echo "Timed out waiting for active Nav2 servers. Log: ${launch_log}" >&2
  exit 1
fi

ros2 run stereo_nav_tests runtime_audit --duration "${audit_duration}"

cmd_vel_nav_info="$(timeout 10 ros2 topic info /cmd_vel_nav 2>/dev/null || true)"
if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<<"${cmd_vel_nav_info}"; then
  echo "Nav2 did not expose a /cmd_vel_nav publisher: ${cmd_vel_nav_info:-no response}. Log: ${launch_log}" >&2
  exit 1
fi
cmd_vel_info="$(timeout 10 ros2 topic info /cmd_vel 2>/dev/null || true)"
if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<<"${cmd_vel_info}"; then
  echo "Nav2 did not expose a final /cmd_vel publisher: ${cmd_vel_info:-no response}. Log: ${launch_log}" >&2
  exit 1
fi
if grep -Eqi 'process has died|segmentation fault|terminate called|tf_repeated_data' "${launch_log}"; then
  echo "A crash or TF conflict was found. Log: ${launch_log}" >&2
  exit 1
fi

echo "Navigation smoke test passed. Log: ${launch_log}"
