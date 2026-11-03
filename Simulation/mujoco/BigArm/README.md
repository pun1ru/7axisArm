# BigArm MuJoCo 模型

由修正后的 URDF 自动导出的 MuJoCo MJCF 模型。

```powershell
cd E:\rm\7axisArm\Simulation\mujoco\BigArm
python -m mujoco.viewer robot.xml
```

## 文件

| 文件 | 说明 |
|---|---|
| `robot.xml` | MJCF 主模型：8 个转动关节 + 2 个移动关节 |
| `meshes/` | 从 URDF 复制的 visual / collision STL |
| `make_mjcf.py` | 从 URDF 重新生成本模型 |
| `dh_params.md` | DH 参数与坐标系约定 |

## 坐标系约定

- `base` 仅作为 URDF 的结构根节点保留，不显示其坐标系。MJCF 将 base 的 visual/collision 合并进 `worldbody`，不生成 `base` body。
- 运动链为 `base -> link1 -> ... -> link8`，转动关节为 `joint1..joint8`。
- 每条 link 的 visual、collision、inertial 数据均已换算到该 link 的 DH 坐标系。
- 所有转动关节和移动关节的局部 `axis` 均为 `0 0 1`；即各关节自身坐标系 z 轴就是转轴/移动轴。
- 每个 DH 坐标系处的 body 上放置了 `dh1..dh8` site，便于在 MuJoCo 中查看。

## 夹爪

- `gripper_finger1_joint`：`link8 -> link9`，沿 CAD 导轨 `+Y`。
- `gripper_finger2_joint`：`link8 -> link10`，沿 CAD 导轨 `-Y`。
- 两个移动关节范围均为 `0 ~ 0.015 m`，正方向为夹紧；`q=0` 对应装配位姿。
- 因 `link8` 坐标系已按 DH 定义旋转，CAD 的 ±Y 方向对应关节局部 `+z`。

## 重新生成

```powershell
cd E:\rm\7axisArm\Simulation\mujoco\BigArm
python make_mjcf.py
```
