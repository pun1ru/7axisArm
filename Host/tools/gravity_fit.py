"""七轴臂重力参数拟合（命令行）：tau = Y(q) · pi。算法在 armhost/model/gravity_fit.py，
上位机"重力拟合"标签页用的是同一份代码。

用法：
    python tools/gravity_fit.py                              # 默认 logs/gravity/*.csv
    python tools/gravity_fit.py a.csv b.csv --bias
    python tools/gravity_fit.py "logs/gravity/*.csv" --max-vel 20 --out fit.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HOST_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HOST_ROOT))

from armhost.kinematics import DHTable, default_dh  # noqa: E402
from armhost.model import gravity_fit as core  # noqa: E402
from armhost.model.gravity_fit import (G, Dataset, FitOptions, build_system, expand_paths,  # noqa: E402,F401
                                       filter_dataset, format_report, gravity_regressor, load_dataset,
                                       per_joint_stats, potential_energy, solve)

DEFAULT_GLOB = str(HOST_ROOT / "logs" / "gravity" / "*.csv")


def load_dh(spec: str) -> DHTable:
    if spec == "firmware":
        return default_dh()
    if spec == "builtin":
        return DHTable()
    return DHTable.load_json(spec)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="七轴臂重力参数拟合 tau = Y(q)·pi",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("csv", nargs="*", help="CSV 文件或通配符，可多个；缺省 logs/gravity/*.csv")
    ap.add_argument("--dh", default="firmware", help="firmware（默认）| builtin | DH 的 JSON 文件")
    ap.add_argument("--torque-col", choices=["torque", "cmd"], default="torque",
                    help="拟合实测力矩 torque_nm（默认）还是指令力矩 torque_cmd_nm")
    ap.add_argument("--sign", type=float, default=1.0, choices=[1.0, -1.0],
                    help="力矩符号约定，-1 表示 CSV 力矩与 tau_g 反号（只会让 pi 整体反号）")
    ap.add_argument("--arm-state", type=int, nargs="+", help="只用这些 arm_state（模式）的行")
    ap.add_argument("--max-vel", type=float, default=None,
                    help="丢弃任一关节 |velocity| 超过该值(dps)的行；缺省不过滤，按全部静止处理")
    ap.add_argument("--bias", action="store_true", help="同时拟合每个关节的力矩零偏")
    ap.add_argument("--rcond", type=float, default=1e-3, help="截断 SVD 的相对阈值（默认 1e-3）")
    ap.add_argument("--g", type=float, default=G)
    ap.add_argument("--no-cv", dest="cv", action="store_false", help="多个文件时不做留一交叉验证")
    ap.add_argument("--out", help="把结果写成 JSON")
    a = ap.parse_args(argv)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")

    try:
        opts = FitOptions(a.torque_col, a.sign, a.arm_state, a.max_vel, a.bias, a.rcond, a.g, a.cv)
        table = load_dh(a.dh)
        paths = expand_paths(a.csv, DEFAULT_GLOB)
        if not paths:
            raise ValueError("没有找到 CSV 文件")
        result = core.run(paths, table, opts)
    except (OSError, ValueError, KeyError) as e:
        print(f"错误: {e}", file=sys.stderr)
        return 2

    print(format_report(result))
    if a.out:
        Path(a.out).write_text(json.dumps(result.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\n已写入 {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
