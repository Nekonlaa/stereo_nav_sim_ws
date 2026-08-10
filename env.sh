#!/usr/bin/env bash

# This file is sourced by both interactive shells and strict project scripts.
# Do not change the caller's shell options. ROS 2 Humble's generated setup
# scripts access a few optional variables and therefore cannot be sourced while
# Bash nounset mode is active.

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  return 1 2>/dev/null || exit 1
fi

stereo_nav_restore_nounset=0
case $- in
  *u*)
    stereo_nav_restore_nounset=1
    set +u
    ;;
esac

source /opt/ros/humble/setup.bash

if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi

if [[ "${stereo_nav_restore_nounset}" == "1" ]]; then
  set -u
fi
unset stereo_nav_restore_nounset

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1

echo "stereo_nav_sim environment: ROS_DISTRO=${ROS_DISTRO}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
