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
| `cpp/` | C++ 控制程序骨架：加载模型、读状态、跑自己的算法 |
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

## 执行器

`robot.xml` 和场景版 `robot_scene.xml` 都使用力矩执行器 `<motor>`（等价于
`gaintype=fixed`、`gainprm=1 0 0`、`biastype=none`、`biasprm=0 0 0` 的通用驱动器）。
`ctrl` 直接是力/力矩，没有内置位置环，也没有重力补偿。

### 大臂关节

`act_joint1` … `act_joint8`，每个转动关节一个：

- `ctrl` = **关节力矩 [N·m]**，`ctrlrange` = ±100（URDF 的 effort）。
- 没有 `kp` / `dampratio`。`ctrl=0` 时关节只受重力，臂会直接掉下来；
  要保位姿得在控制器里自己做重力补偿 + PD（Host 的 `SimBackend` 正是这样做的）。

### 夹爪（两指严格对称）

- `<equality name="gripper_symmetry">` 约束两指位移相等：`q1 − q2 = 0`。
  因此**两个手指到夹爪中心线的距离始终相同**，不会一快一慢。
- `<tendon name="gripper_tendon">` 以 `coef=0.5` 连接两指，长度为 `(q1+q2)/2`；
  由单个执行器 `act_gripper` 驱动，力的分配也是左右各一半。
- `act_gripper.ctrl` = **腱张力 [N]**，范围 ±100；经 `coef=0.5` 后每指 ≤ 50 N，
  与 URDF effort 一致。正值收紧（`q` 增大，夹紧），负值张开。
- 关节限位加了 `solreflimit="0.005 1"`（默认是 `0.02`）。力矩控制会一直顶到
  机械限位，软限位会被顶穿好几毫米；加硬后 40 N 时两指只超出限位约 0.6 mm，
  两指差值约 0.06 mm。

```python
import mujoco
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)
data.ctrl[model.actuator("act_joint1").id] = 10.0     # N·m
data.ctrl[model.actuator("act_gripper").id] = 40.0    # N，每指 20 N（夹紧）
```

注意：`link9` 与 `link10` 的 collision 凸包在行程约 `6 ~ 10 mm` 处会互相
穿插（`dist ≈ -0.9 mm`），夹爪会在该区间被卡一下。若不需要手指互碰，
可在 `<contact>` 中加 `<exclude body1="link9" body2="link10"/>`。

### joint1 被地面卡住（已修复）

`q=0` 时 `link1` 的 collision 网格会扎进 `z=0` 的地面约 4 mm。mesh-plane
接触在这么深的穿插下会产生极大的法向力，其摩擦力矩远大于 `act_joint1`
的 100 N·m 上限，于是 joint1 只能慢慢爬到约 `0.054 rad` 就停住；其余关节
不受影响。基座已焊死在世界系上，`link1` 不需要和地面碰撞，因此两个 XML
的 `<contact>` 里都加了：

```xml
<exclude body1="link1" body2="world" />
```

修复前后（`act_joint1.ctrl` = 100 N·m，2 s）：

| | joint1 角度 | 接触数 |
|---|---:|---:|
| 修复前 | 0.054 rad（顶住不动） | 15 |
| 修复后 | 1.556 rad（转到其他连杆碰地） | 14 |

### 在 simulate 里手动动关节

力矩执行器没有位置环：`ctrl=0` 时鼠标拖动是自由的，但一松手臂就会在重力下
掉下来。Control 面板里 `act_joint1 ... act_joint8` 的滑块给的是**力矩**，给常数
力矩会让关节持续加速或顶到限位。想“摆到某个角度并保持”，要么自己写 PD +
重力补偿，要么把执行器换回 `<position>`。

## 传感器

8 个转动关节各带两个传感器（共 16 个），从 `data.sensordata` 读取：

| 传感器 | 含义 | 单位 |
|---|---|---|
| `jointN_pos` (`jointpos`) | 关节角 | rad |
| `jointN_vel` (`jointvel`) | 关节角速度 | rad/s |

MuJoCo **没有** `jointacc` 传感器类型。角加速度每一步都已经算在
`mjData.qacc` 里，直接取即可，不需要额外定义：

```python
import mujoco
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

for name in ("joint1", "joint2", "joint3", "joint4",
             "joint5", "joint6", "joint7", "joint8"):
    jid = model.joint(name).id
    angle = data.sensor(f"{name}_pos").data[0]          # rad
    omega = data.sensor(f"{name}_vel").data[0]          # rad/s
    alpha = data.qacc[model.jnt_dofadr[jid]]            # rad/s^2
```

注意：`mj_step` 在积分前更新传感器，所以步进后直接读 `sensordata` 得到的是
步初状态（与 `qpos`/`qvel` 差一个 `timestep`）。需要严格对齐时先调用一次
`mujoco.mj_forward(model, data)` 再读。

夹爪两个移动关节没有加传感器；需要的话可以同样加 `jointpos` / `jointvel`。

## C++ 接入

`cpp/` 里是一个最小控制程序：加载 `robot.xml`，每个 `timestep` 读一次关节状态，
调用 `Controller::update()` 得到 8 个关节力矩，写回 `data.ctrl` 并 `mj_step`。
接自己的算法只需要改 `update()`，构建和接口约定见
[`cpp/README.md`](cpp/README.md)。

```bash
cd cpp
cmake -S . -B build -DMUJOCO_ROOT=/opt/mujoco
cmake --build build
./build/bigarm_ctrl ../robot.xml 5
```

注意：力矩执行器是显式积分，PD 增益必须按模型惯量取（示例 `kp=I·ωn²`，
`ωn=10 rad/s`）；增益给大（如 `kp=60`）在 2 ms 步长下会数值发散。

## 重新生成

```powershell
cd E:\rm\7axisArm\Simulation\mujoco\BigArm
python make_mjcf.py
```
