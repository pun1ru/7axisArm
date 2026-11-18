#!/usr/bin/env bash
# 编译离线实验程序到 ./out/（不依赖 MuJoCo）。用法：bash build.sh
set -euo pipefail
cd "$(dirname "$0")"
ALG="$(cd ../../Algorithm && pwd)"
mkdir -p out
CXX="g++ -std=c++17 -O2 -Wall -Wextra -I$ALG -I."
LIBS="$ALG/ik.cpp $ALG/ik_cost.cpp $ALG/kinematics.cpp"
# 仓库的零空间实现已是无阻尼加权伪逆投影，不再保留叉积版本做对比
for t in exp_a exp_b dump diag off1 cmp proj sing; do
    $CXX $t.cpp $ALG/ik_numeric.cpp $LIBS -o out/$t
done
echo "built: $(ls out | grep -v '\.cpp' | tr '\n' ' ')"
