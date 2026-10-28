# 等效性检查工具

用来确认 `../robot.xml` 仍然和源 URDF 等效。改模型前后各跑一次。

```bash
tools/check.sh                      # robot.xml 与默认 URDF 比较
tools/check.sh my.xml other.urdf     # 指定文件
```

依赖：`python3`、C 编译器、MuJoCo SDK（默认 `/opt/mujoco`，可用 `MUJOCO_DIR` 覆盖）。

## 判定标准

输出末行为 `PASS` 表示世界坐标系下与 URDF 等效。检查项：

| 检查 | 阈值 |
|---|---|
| 每个 body 的质量 | 1e-12 kg |
| 惯性系质心、世界惯量张量 | 1e-6 m / 1e-9 kg·m² |
| 96 个 mesh 实例的世界位姿 | 1e-6 m |
| 关节轴方向、轴线位置、限位 | 1e-3 deg / 1e-6 m / 精确 |

`urdf_check.py` 只用标准库做 URDF/MJCF 正运动学；`mjdump.c` 再用 `mj_loadXML`
编译一次，把 `body_inertia`/`body_iquat` 还原成世界张量独立复算，顺带验证
`fullinertia` 的分量顺序（`ixx iyy izz ixy ixz iyz`）没写错。

## 两处"允许不一致"

1. **body 原点**（第 4 节）。坐标系沿自身关节轴滑动属于刻度选择，不改变物理。
   报告里 `dCOM_body` 非零的就是这些 body；只要世界系各项误差在 1e-9 量级就没问题。
   当前 `link2/4/5/6` 的原点相对 `O_i = J(i-1)∩J(i)` 仍有 4.0 / 2.5 / 100.0 / 1.0 mm 偏置，
   见第 4 节表格。
2. **URDF 没有的建模项**：`timestep`/`integrator`、geom 的 visual/collision 分组与
   `contype/conaffinity`、`<contact>` 的 exclude、`tcp` 与 `dh1..dh7` 站点。
   这些不影响质量、惯量、运动学。
