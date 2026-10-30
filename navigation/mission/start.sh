#!/usr/bin/env bash
set -euo pipefail
N=/home/rcsdart/rm27/navigation
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
set -u
exec /usr/bin/python3 "$N/mission/runner.py" "$@"
