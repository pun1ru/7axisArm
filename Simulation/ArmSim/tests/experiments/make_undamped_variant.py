"""由 Algorithm/ik_numeric.cpp 生成「无阻尼伪逆投影」变体，只用于对比实验，不进入固件。
用法：python3 make_undamped_variant.py <输出路径>"""
import sys
from pathlib import Path

src = (Path(__file__).resolve().parents[2] / "Algorithm" / "ik_numeric.cpp").read_text()
a = src.index('    /* 零空间项：7 个关节全自由时')
b = src.index('    return true;', a)
variant = '''    /* 变体：无阻尼加权伪逆的投影 N = I − W⁻¹Jᵀ(JW⁻¹Jᵀ)⁻¹J，对 z 求 N z */
    if (all_free)
    {
        Mat6 bm{};
        Vec6 jz{};
        Vec6 u{};

        for (int r = 0; r < 6; ++r)
        {
            for (int c = 0; c < 6; ++c)
            {
                for (int i = 0; i < kJointCount; ++i)
                    bm[r][c] += w_inv[static_cast<size_t>(i)] * jac[r][i] * jac[c][i];
            }
            for (int i = 0; i < kJointCount; ++i) jz[r] += jac[r][i] * z[static_cast<size_t>(i)];
        }
        if (SolveSpd6(bm, jz, u))
        {
            for (int i = 0; i < kJointCount; ++i)
            {
                double proj = 0.0;
                for (int r = 0; r < 6; ++r) proj += jac[r][i] * u[r];
                dq_null[static_cast<size_t>(i)] = z[static_cast<size_t>(i)] - w_inv[static_cast<size_t>(i)] * proj;
            }
        }
    }
'''
Path(sys.argv[1]).write_text(src[:a] + variant + src[b:])
