#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
source "${project_dir}/env.sh"

smoke_dir="$(mktemp -d -t stereo_nav_smoke_XXXXXX)"
database_path="${smoke_dir}/rtabmap.db"
launch_log="${smoke_dir}/mapping.log"
audit_duration="${1:-15}"

setsid ros2 launch stereo_nav_bringup mapping.launch.py \
  gui:=false headless:=true rviz:=false new_map:=true \
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
  echo "Timed out waiting for /stereo/points2. Log: ${launch_log}" >&2
  exit 1
fi

"${script_dir}/stereo_pair_diagnostic.py"
ros2 run stereo_nav_tests runtime_audit --duration "${audit_duration}"
if ! kill -0 "${launch_pid}" 2>/dev/null; then
  echo "Mapping launch exited during the audit. Log: ${launch_log}" >&2
  exit 1
fi

if grep -Eqi 'process has died|segmentation fault|terminate called|tf_repeated_data' "${launch_log}"; then
  echo "A crash or TF conflict was found. Log: ${launch_log}" >&2
  exit 1
fi

sync_warnings="$(grep -Eic 'did not receive data since|failed to meet update rate' "${launch_log}" || true)"
if (( sync_warnings > 4 )); then
  echo "Persistent synchronization/update warnings (${sync_warnings}) found. Log: ${launch_log}" >&2
  exit 1
fi
echo "Headless smoke test passed. Log and temporary database: ${smoke_dir}"
