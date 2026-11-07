#!/usr/bin/env bash
# Linux 一键启动：bash start.sh [--mock | --port /dev/ttyUSB0 | --replay xxx.csv]
set -e
cd "$(dirname "$0")"
PY="${PYTHON:-python3}"

# 某些 GNOME/Wayland 环境的 GTK settings schema 不兼容，直接运行 Host 时也走 XWayland。
if [[ "$(uname -s)" == "Linux" && "${ARMSIM_FORCE_X11:-1}" == "1" &&
      -n "${WAYLAND_DISPLAY:-}" && -n "${DISPLAY:-}" ]]; then
    export GDK_BACKEND=x11
    export XDG_SESSION_TYPE=x11
    export GSETTINGS_BACKEND=memory
    echo "[Host] 检测到 Wayland，Qt 窗口使用 XWayland 兼容模式"
fi

if ! "$PY" -c "import PySide6, pyqtgraph, serial, numpy" 2>/dev/null; then
    echo "缺少依赖，正在安装 requirements.txt ..."
    "$PY" -m pip install -r requirements.txt
fi

# 串口权限：当前用户不在 dialout 组时提醒一次
if ! id -nG | grep -qw dialout; then
    echo "提示：当前用户不在 dialout 组，打开真实串口可能被拒绝。执行 sudo usermod -aG dialout \$USER 后重新登录。"
fi

exec "$PY" -m armhost.app "$@"
