#!/usr/bin/env bash
# ArmSim 一键运行：配置 -> 编译 -> 启动仿真（BigArm，不限时长，Ctrl-C 退出）
# 用法:  ./run.sh                 默认 BigArm 模型，按实时节拍一直跑
#        ./run.sh --fast          不限速
#        ./run.sh --host          开伪串口等 Host/armhost
#        ./run.sh ../mujoco/BigArm/robot.xml 10     指定模型和秒数
# MuJoCo SDK 位置：环境变量 MUJOCO_ROOT，没设就用 /opt/mujoco
set -euo pipefail
cd "$(dirname "$0")"

MUJOCO_ROOT="${MUJOCO_ROOT:-/opt/mujoco}"
if [ ! -f "$MUJOCO_ROOT/include/mujoco/mujoco.h" ]; then
    echo "[run] 找不到 MuJoCo SDK: $MUJOCO_ROOT" >&2
    echo "[run] 请设置 MUJOCO_ROOT 指向它，例如: MUJOCO_ROOT=~/mujoco ./run.sh" >&2
    exit 1
fi

cmake -S . -B build -DMUJOCO_ROOT="$MUJOCO_ROOT" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build -j"$(nproc)"

exec ./build/armsim "$@"
