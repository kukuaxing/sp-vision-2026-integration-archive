#!/usr/bin/env bash
set -euo pipefail
NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec 9>"$NAV_ROOT/logs/localization.lock"
if ! flock -n 9; then printf '%s\n' 'Localization is already running.'; exit 0; fi
printf '%s\n' "$$" > "$NAV_ROOT/logs/localization.pid"
# Isolate navigation from the camera SDK's older bundled libusb.
# Only this process and its children are affected; shell/camera settings stay.
export LD_LIBRARY_PATH="$(/usr/bin/python3 - <<'NAV_ENV_PY'
import os
paths = os.environ.get('LD_LIBRARY_PATH', '').split(':')
print(':'.join(p for p in paths if p.startswith('/') and '/MVS/' not in p))
NAV_ENV_PY
)"
set +u
source /opt/ros/humble/setup.bash
source "$NAV_ROOT/install/livox_driver/local_setup.bash"
source "$NAV_ROOT/install/localization/local_setup.bash"
set -u
export LD_LIBRARY_PATH="$NAV_ROOT/install/livox_sdk/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="$NAV_ROOT/logs/localization_ros"
export OMP_NUM_THREADS=3
cd "$NAV_ROOT"
exec ros2 launch "$NAV_ROOT/localization/localization.launch.py" config_file:="$NAV_ROOT/localization/fastlio.vehicle.yaml"
