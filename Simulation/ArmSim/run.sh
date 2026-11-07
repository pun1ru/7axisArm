#!/usr/bin/env bash
# ArmSim 一键运行：配置 -> 编译 -> 启动仿真（普通地面 + BigArm，Ctrl-C 退出）
# 用法:  ./run.sh                 默认场地 + BigArm，按实时节拍一直跑
#        ./run.sh --fast          不限速
#        ./run.sh --host          开伪串口等 Host/armhost
#        ./run.sh --no-arena      只加载 BigArm
#        ./run.sh --arm-position 5 -5 0.01
#        ./run.sh --exchange-position 4.05 -5 0
#        ./run.sh --no-exchange   不加载兑换站
#        ./run.sh --full-viewer   查看器加载完整碰撞模型（启动较慢）
#        ./run.sh ../mujoco/BigArm/robot.xml 10     指定模型和秒数
# MuJoCo SDK 位置：环境变量 MUJOCO_ROOT，没设就用 /opt/mujoco
set -euo pipefail
cd "$(dirname "$0")"

# 与 start_all.sh 相同的 GNOME/Wayland 兼容处理，保证单独运行查看器也能启动。
if [[ "$(uname -s)" == "Linux" && "${ARMSIM_FORCE_X11:-1}" == "1" &&
      -n "${WAYLAND_DISPLAY:-}" && -n "${DISPLAY:-}" ]]; then
    export GDK_BACKEND=x11
    export XDG_SESSION_TYPE=x11
    export GSETTINGS_BACKEND=memory
    echo "[run] 检测到 Wayland，MuJoCo 窗口使用 XWayland 兼容模式"
fi

MUJOCO_ROOT="${MUJOCO_ROOT:-/opt/mujoco}"
if [ ! -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ]; then
    echo "[run] 找不到 MuJoCo SDK: $MUJOCO_ROOT" >&2
    echo "[run] 请设置 MUJOCO_ROOT 指向它，例如: MUJOCO_ROOT=~/mujoco ./run.sh" >&2
    exit 1
fi

cmake -S . -B build -DMUJOCO_ROOT="$MUJOCO_ROOT" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build -j"$(nproc)"

exec ./build/armsim "$@"
