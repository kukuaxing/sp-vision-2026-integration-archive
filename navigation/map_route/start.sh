#!/usr/bin/env bash
set -euo pipefail
N=/home/rcsdart/rm27/navigation
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export PATH=/usr/local/bin:/usr/bin:/bin
set +u
source /opt/ros/humble/setup.bash
set -u
for argument in "$@"; do
  if [[ "$argument" == --execute ]]; then
    # sudo's setuid loader removes LD_LIBRARY_PATH before sudo can preserve it.
    # Pass the sourced ROS environment to env after crossing that boundary.
    exec sudo -n /usr/bin/env \
      ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
      AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}" \
      PYTHONPATH="${PYTHONPATH:-}" \
      LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
      /usr/bin/python3 "$N/map_route/live.py" "$@"
  fi
done
exec /usr/bin/python3 "$N/map_route/live.py" "$@"
