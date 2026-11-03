#!/usr/bin/env bash
# 一键检查 robot.xml 是否与 URDF 等效.
#
#   tools/check.sh [robot.xml] [urdf]
#
# 依赖: python3, C 编译器, libmujoco 开发包 (默认 /opt/mujoco, 可用 MUJOCO_DIR 覆盖)
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
mjcf=${1:-"$here/../robot.xml"}
urdf=${2:-"$here/../../【装配体】自定义控制器（DM4310）/robot.urdf"}
MUJOCO_DIR=${MUJOCO_DIR:-/opt/mujoco}

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

"${CC:-cc}" -O2 -w -o "$tmp/mjdump" "$here/mjdump.c" \
  -I"$MUJOCO_DIR/include" -L"$MUJOCO_DIR/lib" -lmujoco \
  -Wl,-rpath,"$MUJOCO_DIR/lib"

"$tmp/mjdump" "$mjcf" > "$tmp/dump.txt"
python3 "$here/urdf_check.py" --mjcf "$mjcf" --urdf "$urdf" --dump "$tmp/dump.txt"
