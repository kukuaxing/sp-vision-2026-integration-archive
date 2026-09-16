#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$PROJECT/build/standard"
PID_FILE="$PROJECT/logs/xuc-bringup/field_autoaim.pid"

if [[ ! -r "$PID_FILE" ]]; then
  printf '自瞄当前没有通过一键入口运行。\n'
  sleep 2
  exit 0
fi

PID="$(tr -cd '0-9' < "$PID_FILE")"
if [[ -z "$PID" ]] || ! kill -0 "$PID" 2>/dev/null; then
  rm -f -- "$PID_FILE"
  printf '未发现仍在运行的自瞄进程，已清理旧状态。\n'
  sleep 2
  exit 0
fi

EXE="$(readlink -f "/proc/$PID/exe" 2>/dev/null || true)"
if [[ "$EXE" != "$BIN" ]]; then
  printf 'PID文件指向的不是指定自瞄程序，拒绝结束：PID=%s EXE=%s\n' "$PID" "$EXE" >&2
  sleep 4
  exit 1
fi

printf '正在安全停止自瞄（PID %s）……\n' "$PID"
kill -INT "$PID"
for _ in $(seq 1 50); do
  if ! kill -0 "$PID" 2>/dev/null; then
    rm -f -- "$PID_FILE"
    printf '自瞄已停止。\n'
    sleep 2
    exit 0
  fi
  sleep 0.2
done

printf 'SIGINT后10秒仍未退出，发送SIGTERM。\n'
kill -TERM "$PID" 2>/dev/null || true
sleep 2
if kill -0 "$PID" 2>/dev/null; then
  printf '进程仍未退出，请保持安全档并联系维护人员。\n' >&2
  exit 1
fi

rm -f -- "$PID_FILE"
printf '自瞄已停止。\n'
sleep 2
