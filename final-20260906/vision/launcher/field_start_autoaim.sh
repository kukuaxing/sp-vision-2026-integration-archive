#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="/home/rm/sp_vision_25-xuc-board-20260901"
BIN="$PROJECT/build/standard"
CONFIG="$PROJECT/configs/standard_hikrobot_phase2d5_rpm6500_gyro_oneface_live_trim141_delay250_UNVALIDATED.yaml"
SERIAL="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
ROS_SETUP="/opt/ros/humble/setup.bash"
MVS_SETUP="/opt/MVS/setup.sh"
LOG_DIR="$PROJECT/logs/xuc-bringup/$(date +%F)"
PID_FILE="$PROJECT/logs/xuc-bringup/field_autoaim.pid"
LAST_LOG_FILE="$PROJECT/logs/xuc-bringup/field_autoaim.last_log"
LOG_KEEP_COUNT=20
CHILD_PID=""

fail() {
  printf '\n[启动失败] %s\n' "$1" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="自瞄启动失败" --text="$1" >/dev/null 2>&1 || true
  fi
  exit 1
}

load_runtime() {
  [[ -r "$ROS_SETUP" ]] || fail "找不到 ROS 2 运行环境：$ROS_SETUP"

  # Desktop launchers do not inherit the interactive SSH shell environment.
  # Both vendor setup scripts expect unset variables to expand as empty.
  set +u
  source "$ROS_SETUP"
  if [[ -r "$MVS_SETUP" ]]; then
    source "$MVS_SETUP" >/dev/null
  fi
  export MVCAM_SDK_PATH=/opt/MVS
  export MVCAM_COMMON_RUNENV=/opt/MVS/lib
  export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
  export ALLUSERSPROFILE=/opt/MVS/MVFG
  export LD_LIBRARY_PATH="/opt/MVS/lib/64:/opt/MVS/lib/32:${LD_LIBRARY_PATH:-}"
  set -u
}

prune_old_logs() {
  find "$PROJECT/logs/xuc-bringup" -type f -name 'one_click_autoaim_*.log' \
    -printf '%T@ %p\0' 2>/dev/null \
    | sort -z -nr \
    | tail -z -n "+$((LOG_KEEP_COUNT + 1))" \
    | cut -z -d ' ' -f 2- \
    | xargs -0r rm -f --
}

preflight() {
  local missing_libs

  [[ -x "$BIN" ]] || fail "找不到可执行文件：$BIN"
  [[ -r "$CONFIG" ]] || fail "找不到实弹配置：$CONFIG"
  [[ -e "$SERIAL" ]] || fail "下位机串口未连接：$SERIAL"
  lsusb | grep -q '2bdf:0001' || fail "未检测到 HikRobot 相机（USB 2bdf:0001）"

  missing_libs="$(ldd "$BIN" 2>&1 | awk '$2 == "=>" && $3 == "not" && $4 == "found" { print $1 }')"
  [[ -z "$missing_libs" ]] || fail "程序运行库缺失：$(printf '%s' "$missing_libs" | paste -sd ', ' -)"

  if [[ -r "$PID_FILE" ]]; then
    local old_pid
    old_pid="$(tr -cd '0-9' < "$PID_FILE")"
    if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
      fail "自瞄已经在运行（PID $old_pid），请勿重复启动。"
    fi
    rm -f -- "$PID_FILE"
  fi
}

cleanup() {
  if [[ -n "$CHILD_PID" ]] && [[ -r "$PID_FILE" ]] && [[ "$(cat "$PID_FILE")" == "$CHILD_PID" ]]; then
    rm -f -- "$PID_FILE"
  fi
}

stop_child() {
  if [[ -n "$CHILD_PID" ]] && kill -0 "$CHILD_PID" 2>/dev/null; then
    kill -INT "$CHILD_PID" 2>/dev/null || true
    wait "$CHILD_PID" 2>/dev/null || true
  fi
}

load_runtime
preflight

if [[ "${1:-}" == "--check" ]]; then
  printf '[检查通过] 相机、串口、程序和配置均已就绪。\n'
  printf '配置：%s\n' "$CONFIG"
  exit 0
fi

mkdir -p -- "$LOG_DIR"
LOG_FILE="$LOG_DIR/one_click_autoaim_$(date +%H%M%S).log"
printf '%s\n' "$LOG_FILE" > "$LAST_LOG_FILE"

printf '自瞄一键启动\n'
printf '配置：%s\n' "$CONFIG"
printf '日志：%s\n' "$LOG_FILE"
printf '请保持机器人在安全档；程序不会替你切换遥控器 AUTO。\n\n'

cd "$PROJECT"
stdbuf -oL -eL "$BIN" "$CONFIG" >> "$LOG_FILE" 2>&1 &
CHILD_PID=$!
printf '%s\n' "$CHILD_PID" > "$PID_FILE"
prune_old_logs
trap 'stop_child; cleanup; exit 130' INT TERM HUP
trap cleanup EXIT

connected=false
for _ in $(seq 1 100); do
  if ! kill -0 "$CHILD_PID" 2>/dev/null; then
    tail -n 80 "$LOG_FILE" || true
    fail "自瞄程序在建立通信前退出，请查看日志。"
  fi
  if grep -q '\[SYNC\]\[XUC\]' "$LOG_FILE"; then
    connected=true
    break
  fi
  sleep 0.1
done

if [[ "$connected" != true ]]; then
  stop_child
  tail -n 80 "$LOG_FILE" || true
  fail "10秒内未收到下位机心跳，程序已自动停止。"
fi

printf '[启动成功] 已与下位机建立通信，PID=%s。\n' "$CHILD_PID"
printf '确认现场安全后，才可由遥控器拨入 AUTO。\n'
printf '关闭本窗口或双击“停止自瞄”可结束程序。\n\n'
tail -n 40 -F --pid="$CHILD_PID" "$LOG_FILE" &
TAIL_PID=$!

set +e
wait "$CHILD_PID"
STATUS=$?
wait "$TAIL_PID" 2>/dev/null
set -e

cleanup
printf '\n自瞄程序已退出，状态码：%s\n' "$STATUS"
printf '日志保存在：%s\n' "$LOG_FILE"
sleep 3
exit "$STATUS"
