#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$PROJECT/build/standard"
BASE_CONFIG="$PROJECT/configs/standard.yaml"
RUNTIME_CONFIG="$PROJECT/logs/xuc-bringup/field_autoaim.runtime.yaml"
CONFIG="$RUNTIME_CONFIG"
SERIAL="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
ROS_SETUP="/opt/ros/humble/setup.bash"
MVS_SETUP="/opt/MVS/setup.sh"
LOG_DIR="$PROJECT/logs/xuc-bringup/$(date +%F)"
PID_FILE="$PROJECT/logs/xuc-bringup/field_autoaim.pid"
LAST_LOG_FILE="$PROJECT/logs/xuc-bringup/field_autoaim.last_log"
COLOR_STATE_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/sp-vision"
COLOR_STATE_FILE="$COLOR_STATE_DIR/field_autoaim_enemy_color"
LOG_KEEP_COUNT=20
CHILD_PID=""
LOCK_COLOR=""
SELECT_COLOR=false
CHECK_ONLY=false

fail() {
  printf '\n[启动失败] %s\n' "$1" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="自瞄启动失败" --text="$1" >/dev/null 2>&1 || true
  fi
  exit 1
}

valid_color() {
  [[ "$1" == "red" || "$1" == "blue" ]]
}

color_label() {
  if [[ "$1" == "red" ]]; then
    printf '红色 (red)'
  else
    printf '蓝色 (blue)'
  fi
}

read_saved_color() {
  local saved=""
  if [[ -r "$COLOR_STATE_FILE" ]]; then
    saved="$(tr -d '[:space:]' < "$COLOR_STATE_FILE")"
  fi
  if valid_color "$saved"; then
    printf '%s' "$saved"
    return
  fi

  # First run keeps the enemy_color already selected in the validated base config.
  saved="$(sed -n '/^[[:space:]]*enemy_color[[:space:]]*:/ {
    s/^[^:]*://
    s/[[:space:]"]//g
    p
    q
  }' "$BASE_CONFIG")"
  if ! valid_color "$saved"; then
    saved="blue"
  fi
  printf '%s' "$saved"
}

persist_color() {
  local color="$1"
  local temp
  valid_color "$color" || fail "无效的锁定颜色：$color（只能是 red 或 blue）"
  mkdir -p -- "$COLOR_STATE_DIR"
  temp="$COLOR_STATE_FILE.tmp.$$"
  (umask 077; printf '%s\n' "$color" > "$temp")
  mv -f -- "$temp" "$COLOR_STATE_FILE"
}

select_color() {
  local current="$1"
  local chosen=""
  if command -v zenity >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    if [[ "$current" == "red" ]]; then
      chosen="$(zenity --list --radiolist --title="选择自瞄锁定颜色" \
        --text="当前锁定红色；本次选择会保存，后续启动和开机自启动继续使用。" \
        --column="选择" --column="值" --column="锁定颜色" --hide-column=2 --print-column=2 \
        TRUE red 红色 FALSE blue 蓝色)" || return 1
    else
      chosen="$(zenity --list --radiolist --title="选择自瞄锁定颜色" \
        --text="当前锁定蓝色；本次选择会保存，后续启动和开机自启动继续使用。" \
        --column="选择" --column="值" --column="锁定颜色" --hide-column=2 --print-column=2 \
        TRUE blue 蓝色 FALSE red 红色)" || return 1
    fi
  elif [[ -t 0 ]]; then
    printf '选择自瞄锁定颜色（当前 %s）：[1] 红色  [2] 蓝色  [回车] 保持当前\n' \
      "$(color_label "$current")" >&2
    read -r chosen
    case "$chosen" in
      1|red|RED) chosen="red" ;;
      2|blue|BLUE) chosen="blue" ;;
      "") chosen="$current" ;;
      *) return 2 ;;
    esac
  else
    return 2
  fi
  valid_color "$chosen" || return 2
  printf '%s' "$chosen"
}

prepare_runtime_config() {
  local temp="$RUNTIME_CONFIG.tmp.$$"
  mkdir -p -- "$(dirname "$RUNTIME_CONFIG")"
  if ! awk -v color="$LOCK_COLOR" '
      BEGIN { updated = 0 }
      /^[[:space:]]*enemy_color[[:space:]]*:/ && !updated {
        print "enemy_color: \"" color "\""
        updated = 1
        next
      }
      { print }
      END { if (!updated) exit 42 }
    ' "$BASE_CONFIG" > "$temp"; then
    rm -f -- "$temp"
    fail "无法从基础配置生成颜色运行配置：$BASE_CONFIG"
  fi
  chmod 600 "$temp"
  mv -f -- "$temp" "$RUNTIME_CONFIG"
}

while (($#)); do
  case "$1" in
    --check)
      CHECK_ONLY=true
      ;;
    --select-color)
      SELECT_COLOR=true
      ;;
    --color)
      shift
      (($#)) || fail "--color 后必须指定 red 或 blue"
      LOCK_COLOR="$1"
      valid_color "$LOCK_COLOR" || fail "无效的锁定颜色：$LOCK_COLOR（只能是 red 或 blue）"
      ;;
    --help|-h)
      printf '用法：%s [--check] [--select-color] [--color red|blue]\n' "$0"
      printf '无参数及开机自启动：沿用上次保存的颜色，不弹出选择窗口。\n'
      exit 0
      ;;
    *)
      fail "未知参数：$1"
      ;;
  esac
  shift
done

[[ -r "$BASE_CONFIG" ]] || fail "找不到基础实弹配置：$BASE_CONFIG"
if [[ -z "$LOCK_COLOR" ]]; then
  LOCK_COLOR="$(read_saved_color)"
fi
if [[ "$SELECT_COLOR" == true ]]; then
  if selected="$(select_color "$LOCK_COLOR")"; then
    LOCK_COLOR="$selected"
  else
    status=$?
    if [[ "$status" -eq 1 ]]; then
      printf '[已取消] 未启动自瞄，锁定颜色保持为 %s。\n' "$(color_label "$LOCK_COLOR")"
      exit 0
    fi
    fail "当前环境无法选择颜色；可使用 --color red 或 --color blue。"
  fi
fi
persist_color "$LOCK_COLOR"
prepare_runtime_config

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
  [[ -r "$CONFIG" ]] || fail "找不到颜色运行配置：$CONFIG"
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

if [[ "$CHECK_ONLY" == true ]]; then
  printf '[检查通过] 相机、串口、程序和配置均已就绪。\n'
  printf '锁定颜色：%s\n' "$(color_label "$LOCK_COLOR")"
  printf '颜色状态：%s\n' "$COLOR_STATE_FILE"
  printf '配置：%s\n' "$CONFIG"
  exit 0
fi

mkdir -p -- "$LOG_DIR"
LOG_FILE="$LOG_DIR/one_click_autoaim_${LOCK_COLOR}_$(date +%H%M%S).log"
printf '%s\n' "$LOG_FILE" > "$LAST_LOG_FILE"

printf '自瞄一键启动\n'
printf '锁定颜色：%s（已保存）\n' "$(color_label "$LOCK_COLOR")"
printf '基础配置：%s\n' "$BASE_CONFIG"
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
