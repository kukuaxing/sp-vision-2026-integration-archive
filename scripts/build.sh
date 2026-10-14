#!/usr/bin/env bash
set -euo pipefail
PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
# Prefer the Ubuntu GUI-enabled OpenCV package over a headless /usr/local build.
# SP_OPENCV_DIR remains an explicit override for other deployed hardware.
opencv_args=()
if [[ -n "${SP_OPENCV_DIR:-}" ]]; then
  opencv_args=("-DOpenCV_DIR=$SP_OPENCV_DIR")
else
  system_opencv="/usr/lib/$(gcc -dumpmachine)/cmake/opencv4"
  if [[ -f "$system_opencv/OpenCVConfig.cmake" ]]; then
    opencv_args=("-DOpenCV_DIR=$system_opencv")
  fi
fi
cmake -S "$PROJECT" -B "$PROJECT/build" "${opencv_args[@]}"
if (($# == 0)); then set -- standard; fi
cmake --build "$PROJECT/build" --parallel "${JOBS:-2}" --target "$@"
