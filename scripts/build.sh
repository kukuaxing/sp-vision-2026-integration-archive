#!/usr/bin/env bash
set -euo pipefail
PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
cmake -S "$PROJECT" -B "$PROJECT/build"
if (($# == 0)); then set -- standard; fi
cmake --build "$PROJECT/build" --parallel "${JOBS:-2}" --target "$@"
