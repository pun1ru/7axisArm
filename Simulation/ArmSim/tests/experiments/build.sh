#!/usr/bin/env bash
# 编译离线实验程序到 ./out/（不依赖 MuJoCo）。用法：bash build.sh
set -euo pipefail
cd "$(dirname "$0")"
ALG="$(cd ../../Algorithm && pwd)"
mkdir -p out
python3 make_undamped_variant.py out/ik_numeric_undamped.cpp
CXX="g++ -std=c++17 -O2 -Wall -Wextra -I$ALG -I."
LIBS="$ALG/ik.cpp $ALG/ik_cost.cpp $ALG/kinematics.cpp"
for t in exp_a exp_b dump diag off1; do
    $CXX $t.cpp $ALG/ik_numeric.cpp $LIBS -o out/$t
done
# 零空间实现对比：每个程序编两份，叉积（仓库）和无阻尼变体
for t in cmp proj sing; do
    $CXX $t.cpp $ALG/ik_numeric.cpp $LIBS -o out/${t}_cross
    $CXX $t.cpp out/ik_numeric_undamped.cpp $LIBS -o out/${t}_undamped 2>/dev/null
done
echo "built: $(ls out | grep -v '\.cpp' | tr '\n' ' ')"
