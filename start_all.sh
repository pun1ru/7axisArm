#!/usr/bin/env bash
# 一键启动 ArmSim（MuJoCo + --host 伪串口）和 Host 上位机。
#
# 用法:
#   ./start_all.sh                 编译并打开 MuJoCo 窗口，然后启动 Host
#   ./start_all.sh --no-viewer     不打开 MuJoCo 窗口
#   ./start_all.sh --no-build      跳过编译，直接运行已有 build/armsim
#   ./start_all.sh -- --log DEBUG   -- 后面的参数原样传给 Host
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
ARMSIM_DIR="$ROOT/Simulation/ArmSim"
BUILD_DIR="$ARMSIM_DIR/build"
ARMSIM_BIN="$BUILD_DIR/armsim"
HOST_DIR="$ROOT/Host"
HOST_LINK="/tmp/armsim-host"
MUJOCO_ROOT="${MUJOCO_ROOT:-/opt/mujoco}"

armsim_args=(--host)
host_args=()
do_build=1

while (($#)); do
    case "$1" in
        --no-viewer)
            armsim_args+=(--no-viewer)
            ;;
        --no-build)
            do_build=0
            ;;
        -h|--help)
            sed -n '2,10p' "$0"
            exit 0
            ;;
        --)
            shift
            host_args=("$@")
            break
            ;;
        *)
            echo "[start_all] 未知参数: $1（Host 参数请放在 -- 后面）" >&2
            exit 2
            ;;
    esac
    shift
done

if [[ ! -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ]]; then
    echo "[start_all] 找不到 MuJoCo SDK: $MUJOCO_ROOT" >&2
    echo "[start_all] 请设置 MUJOCO_ROOT，例如: MUJOCO_ROOT=~/mujoco ./start_all.sh" >&2
    exit 1
fi

if [[ -e "$HOST_LINK" ]]; then
    echo "[start_all] $HOST_LINK 已存在，可能已有 ArmSim --host 在运行。" >&2
    echo "[start_all] 请先关闭它，或确认没有进程后删除该链接。" >&2
    exit 1
fi

if ((do_build)); then
    cmake -S "$ARMSIM_DIR" -B "$BUILD_DIR" \
        -DMUJOCO_ROOT="$MUJOCO_ROOT" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$BUILD_DIR" --parallel
elif [[ ! -x "$ARMSIM_BIN" ]]; then
    echo "[start_all] 找不到 $ARMSIM_BIN，请去掉 --no-build 先编译。" >&2
    exit 1
fi

armsim_pid=""
host_pid=""

stop_pid() {
    local pid="$1"
    [[ -n "$pid" ]] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    kill -TERM "$pid" 2>/dev/null || return 0
    for _ in {1..20}; do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.1
    done
    kill -KILL "$pid" 2>/dev/null || true
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    stop_pid "$host_pid"
    stop_pid "$armsim_pid"
    rm -f "$HOST_LINK"
    exit "$status"
}
trap cleanup EXIT INT TERM

echo "[start_all] 启动 ArmSim: ${armsim_args[*]}"
"$ARMSIM_BIN" "${armsim_args[@]}" &
armsim_pid=$!

echo "[start_all] 等待伪串口 $HOST_LINK ..."
for _ in {1..600}; do
    if ! kill -0 "$armsim_pid" 2>/dev/null; then
        wait "$armsim_pid" || true
        echo "[start_all] ArmSim 在创建伪串口前退出了。" >&2
        exit 1
    fi
    [[ -e "$HOST_LINK" ]] && break
    sleep 0.1
done

if [[ ! -e "$HOST_LINK" ]]; then
    echo "[start_all] 等待 $HOST_LINK 超时。" >&2
    exit 1
fi

echo "[start_all] ArmSim 已就绪: $HOST_LINK -> $(readlink -f "$HOST_LINK")"
echo "[start_all] 启动 Host: --port $HOST_LINK ${host_args[*]}"
"$HOST_DIR/start.sh" --port "$HOST_LINK" "${host_args[@]}" &
host_pid=$!

# 任一进程退出（关闭窗口 / Ctrl-C / 崩溃）就清理另一个。
set +e
wait -n "$armsim_pid" "$host_pid"
status=$?
set -e
exit "$status"
