#!/usr/bin/env bash
# Linux 一键启动：bash start.sh [--mock | --port /dev/ttyUSB0 | --replay xxx.csv]
set -e
cd "$(dirname "$0")"
PY="${PYTHON:-python3}"

if ! "$PY" -c "import PySide6, pyqtgraph, serial, numpy" 2>/dev/null; then
    echo "缺少依赖，正在安装 requirements.txt ..."
    "$PY" -m pip install -r requirements.txt
fi

# 串口权限：当前用户不在 dialout 组时提醒一次
if ! id -nG | grep -qw dialout; then
    echo "提示：当前用户不在 dialout 组，打开真实串口可能被拒绝。执行 sudo usermod -aG dialout \$USER 后重新登录。"
fi

exec "$PY" -m armhost.app "$@"
