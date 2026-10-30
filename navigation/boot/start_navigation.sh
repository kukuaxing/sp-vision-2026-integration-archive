#!/usr/bin/env bash
set -euo pipefail
N=/home/rcsdart/rm27/navigation
mkdir -p "$N/logs/boot"
exec 9>"$N/logs/boot/supervisor.lock"
if ! flock -n 9; then echo '导航启动监管已运行。' >&2; exit 2; fi
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OMP_NUM_THREADS=4
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
export PATH=/usr/local/bin:/usr/bin:/bin
export LD_LIBRARY_PATH="$N/install/gtsam/lib"
set +u
source /opt/ros/humble/setup.bash
source "$N/install/livox_driver/local_setup.bash"
source "$N/install/localization/local_setup.bash"
source "$N/install/z_localization/local_setup.bash"
set -u
export LD_LIBRARY_PATH="$N/install/livox_sdk/lib:$N/install/gtsam/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="$N/logs/boot/ros"
exec /usr/bin/python3 "$N/boot/supervisor.py"
