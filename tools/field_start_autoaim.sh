#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$PROJECT/build/standard"
BASE_CONFIG="$PROJECT/configs/standard.yaml"
RUNTIME_CONFIG="$PROJECT/logs/xuc-bringup/field_autoaim.runtime.yaml"
CONFIG="$RUNTIME_CONFIG"
LOG_DIR="$PROJECT/logs/xuc-bringup/$(date +%F)"
PID_FILE="$PROJECT/logs/xuc-bringup/field_autoaim.pid"
LAST_LOG_FILE="$PROJECT/logs/xuc-bringup/field_autoaim.last_log"
COLOR_STATE_DIR="$PROJECT/logs/field-state"
COLOR_STATE_FILE="$COLOR_STATE_DIR/field_autoaim_enemy_color"
LOG_KEEP_COUNT=20
CHILD_PID=""
LOCK_COLOR=""
SELECT_COLOR=false
CHECK_ONLY=false
FOLLOW_ARGS=()

fail() {
  printf '\n[启动失败] %s\n' "$1" >&2
  if [[ -n "${DISPLAY:-}" ]] && command -v zenity >/dev/null 2>&1; then
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
        --width=480 --height=250 --ok-label="启动" --cancel-label="取消" \
        --text="自检通过。选择敌方装甲板颜色；已默认上次的红色。确认后保存，下次启动沿用。" \
        --column="选择" --column="值" --column="锁定颜色" --hide-column=2 --print-column=2 \
        TRUE red 红色 FALSE blue 蓝色)" || return 1
    else
      chosen="$(zenity --list --radiolist --title="选择自瞄锁定颜色" \
        --width=480 --height=250 --ok-label="启动" --cancel-label="取消" \
        --text="自检通过。选择敌方装甲板颜色；已默认上次的蓝色。确认后保存，下次启动沿用。" \
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
    --rmv-follow)
      FOLLOW_ARGS+=(--rmv-follow)
      ;;
    --rmv-rotate|--rmv-fire)
      FOLLOW_ARGS+=("$1")
      ;;
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
      printf '用法：%s [--check] [--select-color] [--color red|blue] [--rmv-follow] [--rmv-rotate] [--rmv-fire]\n' "$0"
      printf 'RMV 默认只识别和记录；--rmv-follow 开启大 Y/P 跟踪，小 Y 保持、取消相对角度限幅，不发射。\n'
      printf '%s\n' '--rmv-rotate 开启四装甲旋转试算；--rmv-fire 显式接入单发，双上预转就绪后由上位机触发，无需操作摇杆。'
      printf '无参数及开机自启动：沿用上次保存的颜色，不弹出选择窗口。\n'
      exit 0
      ;;
    *)
      fail "未知参数：$1"
      ;;
  esac
  shift
done

prune_old_logs() {
  find "$PROJECT/logs/xuc-bringup" -type f -name 'one_click_autoaim_*.log' \
    -printf '%T@ %p\0' 2>/dev/null \
    | sort -z -nr \
    | tail -z -n "+$((LOG_KEEP_COUNT + 1))" \
    | cut -z -d ' ' -f 2- \
    | xargs -0r rm -f --
}

preflight() {
  local report="$PROJECT/logs/xuc-bringup/desktop_preflight.json"
  printf '[自检] 正在检查配置、模型、运行库、相机、串口权限和协议兼容性……\n'
  if ! python3 "$PROJECT/scripts/check_deployment.py" --require-runtime > "$report" 2>&1; then
    cat "$report"
    local details
    details="$(python3 - "$report" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    print("\n".join(d['errors']))
except (ValueError, KeyError):
    print('配置或检查程序异常，请查看终端和自检记录。')
PY
    )"
    fail "自检未通过：$details
记录：$report"
  fi
  local serial
  serial="$(python3 - "$report" <<'PY'
import json, sys
print(json.load(open(sys.argv[1]))['hardware']['serial_path'])
PY
  )"
  if fuser "$serial" >/dev/null 2>&1; then
    fail "串口已被其他程序占用：$serial。请先关闭占用程序。"
  fi
  # Also detect direct starts that did not create our PID file.
  local proc exe
  for proc in /proc/[0-9]*/exe; do
    exe="$(readlink -f "$proc" 2>/dev/null || true)"
    [[ "$exe" != "$BIN" ]] || fail "本工程自瞄已在运行，请先关闭。"
  done
  if [[ -r "$PID_FILE" ]]; then
    local old_pid old_exe
    old_pid="$(cat "$PID_FILE")"
    if [[ "$old_pid" =~ ^[0-9]+$ ]]; then
      old_exe="$(readlink -f "/proc/$old_pid/exe" 2>/dev/null || true)"
      [[ "$old_exe" != "$BIN" ]] || fail "自瞄已经在运行（PID $old_pid）。"
    fi
    rm -f -- "$PID_FILE"
  fi
  printf '[自检通过] 启动后继续验证实际相机及下位机通信。\n'
}

cleanup() {
  if [[ -n "$CHILD_PID" ]] && ! kill -0 "$CHILD_PID" 2>/dev/null && [[ -r "$PID_FILE" ]] && [[ "$(cat "$PID_FILE")" == "$CHILD_PID" ]]; then
    rm -f -- "$PID_FILE"
  fi
}

stop_child() {
  if [[ -n "$CHILD_PID" ]] && kill -0 "$CHILD_PID" 2>/dev/null; then
    kill -INT "$CHILD_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$CHILD_PID" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$CHILD_PID" 2>/dev/null; then
      kill -TERM "$CHILD_PID" 2>/dev/null || true
    fi
    # Do not hang the launcher on a driver stuck in uninterruptible sleep.
    for _ in $(seq 1 10); do
      kill -0 "$CHILD_PID" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$CHILD_PID" 2>/dev/null; then
      printf '[关闭失败] PID %s 仍在运行，已保留进程记录。\n' "$CHILD_PID" >&2
      return 1
    fi
  fi
}

[[ -r "$BASE_CONFIG" ]] || fail "找不到基础运行配置：$BASE_CONFIG"
source "$PROJECT/scripts/runtime_env.sh"
mkdir -p -- "$PROJECT/logs/xuc-bringup"
exec 9>"$PROJECT/logs/xuc-bringup/field_autoaim.lock"
flock -n 9 || fail "自瞄正在启动或运行，请勿重复点击。"
preflight

if [[ "$CHECK_ONLY" == true ]]; then
  printf '上次装甲板颜色：%s\n' "$(color_label "$(read_saved_color)")"
  exit 0
fi

if [[ -z "$LOCK_COLOR" ]]; then LOCK_COLOR="$(read_saved_color)"; fi
if [[ "$SELECT_COLOR" == true ]]; then
  if selected="$(select_color "$LOCK_COLOR")"; then
    LOCK_COLOR="$selected"
  else
    status=$?
    if [[ "$status" -eq 1 ]]; then
      printf '[已取消] 未启动自瞄，上次颜色保持不变。\n'
      exit 0
    fi
    fail "当前环境无法选择颜色；可使用 --color red 或 --color blue。"
  fi
fi
prepare_runtime_config
persist_color "$LOCK_COLOR"

mkdir -p -- "$LOG_DIR"
LOG_FILE="$LOG_DIR/one_click_autoaim_${LOCK_COLOR}_$(date +%H%M%S)_$$.log"
printf '%s\n' "$LOG_FILE" > "$LAST_LOG_FILE"

printf '视觉调试启动（RMV 默认只观察，--rmv-follow 才跟踪）\n'
printf '锁定颜色：%s（已保存）\n' "$(color_label "$LOCK_COLOR")"
printf '基础配置：%s\n' "$BASE_CONFIG"
printf '配置：%s\n' "$CONFIG"
printf '日志：%s\n' "$LOG_FILE"
python3 - "$CONFIG" "${FOLLOW_ARGS[@]}" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
if c.get('board_protocol') == 'RMV':
    fire = '--rmv-fire' in sys.argv
    follow = '--rmv-follow' in sys.argv or fire
    print('控制：' + ('开启' if follow else '只读') + '；发射：' + ('待遥控许可' if fire else '关闭'))
else:
    print('控制：' + ('开启' if c['xuc_allow_control'] else '关闭') +
          '；发射：' + ('开启' if c['xuc_allow_shoot'] and c['auto_fire'] else '关闭'))
PY
printf '请保持机器人在安全档；程序不会替你切换遥控器 AUTO。\n\n'

cd "$PROJECT"
trap 'stop_child || true; cleanup; exit 130' INT TERM HUP
trap 'stop_child || true; cleanup' EXIT
stdbuf -oL -eL "$BIN" "$CONFIG" "${FOLLOW_ARGS[@]}" >> "$LOG_FILE" 2>&1 &
CHILD_PID=$!
printf '%s\n' "$CHILD_PID" > "$PID_FILE"
prune_old_logs

connected=false
for _ in $(seq 1 100); do
  if ! kill -0 "$CHILD_PID" 2>/dev/null; then
    tail -n 80 "$LOG_FILE" || true
    fail "自瞄程序在建立通信前退出，请查看日志。"
  fi
  if grep -Eq '\[SYNC\]\[(XUC|RMV)\]' "$LOG_FILE"; then
    connected=true
    break
  fi
  sleep 0.1
done

if [[ "$connected" != true ]]; then
  stop_child || fail "未收到下位机心跳，且进程没有正常退出；请保持安全档并检查终端。"
  tail -n 80 "$LOG_FILE" || true
  fail "10秒内未收到下位机心跳，程序已自动停止。"
fi

printf '[启动成功] 已与下位机建立通信，PID=%s。\n' "$CHILD_PID"
printf '确认现场安全后，才可由遥控器拨入 AUTO。\n'
printf '关闭本窗口或双击“关闭自瞄”可结束程序。\n\n'
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
