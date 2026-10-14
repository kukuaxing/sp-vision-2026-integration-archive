#!/usr/bin/env bash
# Source in the current shell; no services or applications are started.
RM27_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Use the Ubuntu Python/ROS/OpenCV stack even from an active Conda shell.
export PATH="/usr/bin:/bin:${PATH:-}"
export PYTHONNOUSERSITE=1
case "$-" in *u*) RM27_RESTORE_NOUNSET=1; set +u;; *) RM27_RESTORE_NOUNSET=0;; esac
if [[ -r /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
# Keep camera libraries consistent with the installed MVS runtime and transport modules.
export LD_LIBRARY_PATH="/opt/MVS/lib/64:$RM27_ROOT/io/hikrobot/lib/amd64:$RM27_ROOT/io/mindvision/lib/amd64:${LD_LIBRARY_PATH:-}"
if [[ "$RM27_RESTORE_NOUNSET" == 1 ]]; then set -u; fi
unset RM27_RESTORE_NOUNSET
