#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot determine the Ubuntu release." >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "This deployment is pinned to Ubuntu 22.04; detected ${ID:-unknown} ${VERSION_ID:-unknown}." >&2
  exit 1
fi

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "humble" ]]; then
  echo "A non-Humble ROS overlay is active (${ROS_DISTRO}). Start a clean shell and retry." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-opencv \
  python3-rosdep \
  python3-pytest \
  python3-yaml \
  mesa-utils \
  ros-dev-tools \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros-gz \
  ros-humble-rtabmap-ros \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools \
  ros-humble-xacro

set +u
source /opt/ros/humble/setup.bash
set -u

if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA driver/GPU preflight failed; deployment stopped without an OS release upgrade." >&2
  exit 1
fi

if ! command -v ign >/dev/null 2>&1 || ! ign gazebo --versions 2>/dev/null | grep -Eq '(^|[^0-9])6\.[0-9]+'; then
  echo "Gazebo Fortress (Gazebo Sim 6) preflight failed." >&2
  exit 1
fi

if [[ "${STEREO_NAV_HEADLESS:-0}" != "1" ]]; then
  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "No GUI display detected. Set STEREO_NAV_HEADLESS=1 for a server deployment." >&2
    exit 1
  fi
  if ! glxinfo -B >/dev/null 2>&1; then
    echo "OpenGL GUI preflight failed. Fix display/GPU forwarding or use a headless deployment." >&2
    exit 1
  fi
fi

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "${workspace_dir}/src" --ignore-src -r -y --rosdistro humble

cd "${workspace_dir}"
colcon build --symlink-install --event-handlers console_direct+
set +u
source "${workspace_dir}/install/setup.bash"
set -u
colcon test --event-handlers console_direct+
colcon test-result --verbose

echo
echo "Deployment build completed. In each new shell run:"
echo "  source ${workspace_dir}/env.sh"
