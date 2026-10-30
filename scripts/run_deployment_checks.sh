#!/usr/bin/env bash
# Deployment build and bounded file/PTY validation. Never opens real devices.
set -euo pipefail
PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT"
mkdir -p logs/deployment
trap 'result=$?; printf "%d\n" "$result" > logs/deployment/exitcode; exit "$result"' EXIT
source scripts/runtime_env.sh
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export OPENCV_FOR_THREADS_NUM=1 OPENCV_FFMPEG_THREADS=1
printf 'Building standard and calibration tools\n'
JOBS=1 bash scripts/build.sh standard capture calibrate_camera calibrate_handeye calibrate_robotworld_handeye verify_handeye xuc_board_probe >logs/deployment/build.log 2>&1
python3 scripts/check_runtime_assets.py >logs/deployment/assets.log
printf 'Building and testing offline geometry, transport and detector tools\n'
cmake -S . -B build/native-offline -DSP_BUILD_OFFLINE_ONLY=ON -DSP_OFFLINE_WITH_INFERENCE=ON -DCMAKE_BUILD_TYPE=Release -DOpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0 >logs/deployment/offline-configure.log 2>&1
python3 scripts/check_offline.py --with-inference --openvino-dir /usr/lib/cmake/openvino2024.6.0 >logs/deployment/offline.log 2>&1
printf 'Testing private Linux PTY transport\n'
python3 scripts/check_rmv_pty.py >logs/deployment/pty.log 2>&1
printf 'Running actual YOLO26 on recorded video and blank images\n'
python3 scripts/replay_detector.py --input data/offline-input/test.mp4 --output data/replay/deployment_video --max-frames 8 --stride 10 --repeat-check >logs/deployment/replay.log 2>&1
python3 scripts/replay_detector.py --blank --output data/replay/deployment_blank --repeat-check >logs/deployment/blank.log 2>&1
python3 scripts/check_deployment.py >logs/deployment/check.json
python3 scripts/check_source_tree.py >logs/deployment/source-check.log
git diff --check
git diff >logs/deployment/source.patch
git status --short >logs/deployment/git-status.txt
printf 'All software deployment checks passed. Hardware commissioning is separate.\n'
