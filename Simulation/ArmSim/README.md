# ArmSim — 七轴臂 MuJoCo 仿真框架

C++17 面向对象实现。控制算法、状态机、上位机协议和实机 `Real/DIYcontroller` 保持一致，
只把最底下的“CAN 收发 + 电机”换成 MuJoCo 的关节读写。默认把
`../mujoco/BigArm/robot.xml`（由 `../【装配体】单臂/robot.urdf` 生成）和兑换站
合并到 [`../mujoco/scene/plain_floor.xml`](../mujoco/scene/plain_floor.xml)
普通地面中。默认不加载 2027 mjarena；其他场地仍可通过 `--arena <xml>` 显式指定。

## 一键运行

运行 `./run.sh`：配置 -> 编译 -> 启动，仿真不限时长，Ctrl-C 退出。

```bash
./run.sh                 # 普通地面 + BigArm + 兑换站，按实时节拍一直跑
./run.sh --fast          # 不限速
./run.sh --host          # 开伪串口 /tmp/armsim-host 等 Host/armhost
./run.sh --no-viewer     # 不开 MuJoCo 窗口
./run.sh --no-arena      # 只加载机械臂，保持原来的调试方式
./run.sh --arena path/to/field.xml
./run.sh --arm-position 5 -5 0.01
./run.sh --exchange-position 4.05 -5 0
./run.sh --no-exchange   # 只保留场地和机械臂
./run.sh --full-viewer   # 查看器也加载完整碰撞模型（很慢，仅用于核对）
./run.sh ../mujoco/BigArm/robot.xml 10     # 第一个位置参数是模型，第二个是秒数
```

需要 CMake、C++ 编译器和 MuJoCo 3.x SDK。SDK 位置用环境变量 `MUJOCO_ROOT` 指定，
没设就找 `/opt/mujoco`，例如 `MUJOCO_ROOT=~/mujoco ./run.sh`。
## MuJoCo 窗口

默认会弹出 MuJoCo 官方 simulate 的窗口（`Driver/viewer.*`，用 SDK 自带的 `libsimulate.a`，passive 模式），
视角操作、左右两侧 UI 面板和 `python -m mujoco.viewer` 一致。物理由 ArmSim 自己推进，
窗口只显示拷贝出来的状态，所以：

- 可用：视角和鼠标操作、Rendering / Group / Visualization 等显示选项、Watch、Joint / Control 面板的显示、
  鼠标拖拽物体施加外力。
- 不起作用：左侧的 Pause / Reset / Keyframe / 速度滑条，它们属于 simulate 自己的物理循环。
  想暂停就 Ctrl-C，想调速度用 `--fast`。
- 小心：Control 面板的滑条会写 MJCF 里 `act_*` 执行器的 `ctrl`，等于在关节上额外加力矩。
  ArmSim 自己的控制不走这些执行器（直接写 `qfrc_applied`），所以不动滑条时它们恒为 0。

窗口占主线程，仿真循环跑在副线程；ESC 或关窗口退出。没有显示环境会提示并退回无窗口运行。
启动时的 `libdecor-gtk-WARNING` 是 Wayland 窗口装饰插件的提示，不影响使用。
若 GNOME/Wayland 环境在创建窗口时因 `org.gnome.settings-daemon.plugins.xsettings`
直接退出，脚本会在 Wayland + XWayland 可用时自动切换到兼容模式。需要坚持使用
原生 Wayland 时设置 `ARMSIM_FORCE_X11=0`：

```bash
ARMSIM_FORCE_X11=0 ./run.sh
```

窗口需要 `libsimulate.a`（MuJoCo SDK 自带）和 GLFW 两样。GLFW 先找系统的（`apt install libglfw3-dev`），
再找 `GLFW_ROOT`，最后找 `~/software/mujoco/build` 里 MuJoCo 自带的那份；缺哪样就无窗口构建。

`CMakeLists.txt` 里保留了 Windows（MSVC、复制 `mujoco.dll`）的处理，但没有在 Windows 上验证过。

## 地面与附件合并

`MujocoWorld` 用 MuJoCo C API 的 `mj_parseXML` 分别读取场地和机械臂，再通过
`mjs_addFrame` / `mjs_attach` 把机械臂附加到场地 `world`，最后用 `mj_compile`
生成运行时模型。相对网格资源仍各自按原 MJCF 文件所在目录解析，不需要生成中间 XML。
机械臂 MJCF 自带的无限大 `floor` plane 会在合并前移除，避免与普通地面重复。
普通地面不包含高精度场地网格，启动和物理步进都比 2027 mjarena 轻得多。默认模型
没有单独的轻量显示副本，物理模型和 Viewer 使用同一套几何。机械臂默认附加在世界系
`(5, -5, 0.01) m`，兑换站 O 点位于 `(4.05, -5, 0) m`。
机械臂名称统一带 `arm/` 前缀，例如 `arm/joint1`、`arm/link8`、`arm/dh8`，避免与场地对象重名。
ArmSim 的关节配置会自动加前缀，控制器和 Host 协议仍使用原来的 `joint1..joint8` 逻辑名称。

可用参数：

| 参数 | 作用 |
| --- | --- |
| `--arena <xml>` | 合并指定场地；场地拥有 timestep、重力和求解器等全局选项 |
| `--no-arena` | 不合并场地，恢复纯机械臂模型 |
| `--arm-position X Y Z` | 机械臂附件在世界系的位置，单位 m |
| `--exchange-position X Y Z` | 兑换站 O 点的世界位置，默认 `4.05 -5 0 m` |
| `--no-exchange` | 不加载兑换站，Host 的兑换站页显示不可用 |
| `--full-viewer` | 自定义场地时让 Viewer 显式加载物理模型；普通地面下无额外作用 |

## 科技核心 / 兑换站

`../mujoco/scene/exchange_station.xml` 和机械臂一样以附件方式合并到普通地面；
物理和 Viewer 使用同一套机构，因此关节位置一致。模型坐标按规则 5.4.3 建立：

- O 为底座正面与台面交线中点，X 指向底座内部，Z 向上；CAD 底面比 O 低 78 mm。
  默认 `O.z=0`（普通地面）。换成带高地的场地时用 `--exchange-position` 指定。
- `--exchange-position` 直接设置 O 点；支撑台作为兑换站附件一起移动，因此手动改高度时
  需要让支撑台底面与当前地面一致。
- 初始科技核心位姿为 `(x,y,z,θ,φ,α)=(-200,200,800 mm,0°,90°,0°)`。
- 一级使用固定朝向；二至四级按表 5-10 随机 `x=[-100,0] mm、y=[100,300] mm、`
  `z=[500,700] mm、θ=[-90,90]°、φ=[0,90]°、α=[-45,45]°`。
- 装配顺序为插入 100 mm、沿 Z 上移 100 mm、P 轴抬升 90°、Q 轴到目标角、确认。
  阶段切换同时检查轨迹时间和机构实际位置，避免慢速物理下“轨迹先完成、机构未到位”。
- 能量单元使用 400 g、95 mm 外径、150 mm 高的视觉模型，并随 P/Q 轴动作。

`ExchangeStation` 只驱动 MJCF 的 position actuator，不直接改 `qpos`。Host 使用
ArmSim 扩展消息 `0x30/0x31` 收发命令和 10 Hz 状态；协议格式见
[`Host/docs/02_protocol.md`](../../Host/docs/02_protocol.md)。

## 关节分工

| 部分 | MJCF 关节 | 作用 |
| --- | --- | --- |
| 腰部 | `joint1` | 只起直立作用，PD + 重力前馈定在 `waist_hold_deg`（0°）。不进上位机协议 |
| 主臂 7 轴 | `joint2..joint8` | 状态机、PID、重力前馈。上位机协议里的 J0..J6 就是这 7 个 |
| 夹爪 | `gripper_finger1_joint` / `gripper_finger2_joint` | 两指同步，位置环输出同样的力。`SetGripperTarget(m)`，0 = 张开，0.015 m = 夹紧 |

腰部和夹爪都放在 `Arm` 对象里，使能后一直保持；主臂的 HOST 闭环掩码和上位机掉线保护不影响它们。
夹爪目前只能从 `Arm::SetGripperTarget` 驱动，上位机协议里没有对应消息。

## 目录与对象

```
Algorithm/  Pid、GravityModel、DH 正运动学、S-R-S 解析逆解 ik（纯数学，无硬件依赖）
Driver/     MujocoWorld、JointActuator、HostLink
Module/     Arm、Host
main.cpp    构造对象、步进循环、脚本输入、终端监视
```

```
main ──► MujocoWorld ◄── JointActuator ×10 ◄── Arm ◄── Host ──► HostLink (pty)
          (mj_step)       腰部1 + 主臂7 + 夹爪2     (内部持有)     (仅 --host)
```

| 类 | 职责 |
| --- | --- |
| `MujocoWorld` | 唯一包含 `mujoco.h` 的地方。按关节名绑定，读位置/速度，写 `qfrc_applied`，查连杆质量和质心 |
| `JointActuator` | 一个关节的执行器：`position()` / `velocity()` / `SetEffort()`，自带出力上限（对应 URDF effort） |
| `Arm` | 状态机、PID、全链重力前馈、腰部和夹爪保持；内部按 Hardware → Observer → Target → Control 分层，见 `Module/arm.h` 顶部注释 |
| `Host` | 上位机协议桥，帧格式与实机一致；ArmSim 额外支持 `HOST_IK`、`SET_IK_TARGET` 和 `ARM_POSTURE` |
| `GravityModel` | 腰部 + 主臂共 8 个自由度的重力前馈，`tau = dU/dq`，每个连杆只要质量和质心 |

每个控制周期（2 ms），`main.cpp` 里的循环：
`脚本输入 -> arm.Step() -> host->Step() -> world.Step() -> 监视输出`。
控制周期必须等于 MJCF 的 `timestep`，不一致时启动会提示。

## 默认演示时间线

```
0.0 s  ENABLE   全部关节闭环，主臂目标跟随当前角，靠重力前馈托住（不留保护状态，否则 6 kg 的臂会先塌一截）
0.5 s  HOST     主臂全部关节闭环在当前角，joint2 目标 +20°
2.0 s           夹爪闭合到 8 mm
3.5 s           joint2 回 0°
5.0 s           夹爪张开，之后保持姿态一直跑
```

空夹时两指在约 8.6 mm 处相碰（`link9`/`link10` 的碰撞体重叠），所以目标 8 mm 实际停在 8.6 mm，这是模型几何，不是控制误差。

## 和上位机对接

```bash
./build/armsim --host              # 开链路，等上位机；Ctrl-C 退出
```

```python
from armhost.comm import ArmLink, SerialConfig, SerialTransport
link = ArmLink(SerialTransport(SerialConfig(port="/tmp/armsim-host", baudrate=921600)))
link.start()
link.set_host_target(0, 20.0, closed_loop=True).result(1.0)   # joint2 转到多圈 20 deg

# 进入 HOST_IK：目标位姿 + 当前分支 + 臂角 ψ（或自动 ψ + 7 个代价权重）
posture = link.mirror.snapshot().posture
link.set_ik_target(posture.branch, posture.x, posture.y, posture.z,
                   posture.roll, posture.pitch, posture.yaw, posture.psi_deg).result(1.0)
link.stop_ik().result(1.0)                                    # 回到 HOST
```

`--host` 下主臂直接进入 HOST，不跑脚本、不刷终端，所有关节一开始都不闭环，只有重力前馈托着，
等上位机发 `SET_HOST_TARGET`。`SET_IK_TARGET` 会先校验请求分支与当前分支一致，再调用 S-R-S
解析逆解、检查关节限位并把 7 个关节全部切到闭环；`ARM_POSTURE` 周期上报当前分支、ψ 和末端位姿。
仿真强制按实时节拍跑。协议只覆盖主臂 7 个关节：`JOINT_STATE` 100 Hz 上报，
`PARAM_READ` 可读限位、PID 增益、DH、方向、零点，未实现的消息回 `UNSUPPORTED`。
上位机掉线超过 300 ms，主臂退出闭环，腰部和夹爪继续保持。

## 参数来源与调参

- **DH、安装位姿**：`../mujoco/BigArm/dh_params.md` 和 URDF 的 `joint1` origin，在 `main.cpp` 的 `MakeArmConfig()` 里。
- **重力前馈的质量和质心**：运行时从 MJCF 读（`MujocoWorld::BodyInertial`），夹爪两指并入 link8。
  随机 200 组位姿下，与 MuJoCo `qfrc_bias` 的最大差为 7e-6 N·m，正运动学与 `dh8` site 的最大位置差为 2e-7 m。
- **PID 增益**：按各关节 q=0 时的质量矩阵对角线算（wn≈15 rad/s，阻尼比≈0.9），误差仍是度，输出仍是 N·m。
  姿态变化后有效惯量会变，腰部和 joint2/3 的增益是折中值。
- **仿真专用电机参数**（`main.cpp` 顶部）：`kJointArmature` 1e-3、`kJointDamping` 0.05、`kJointFrictionLoss` 0.02，
  置 0 就是纯 MJCF。出力上限不再统一限幅，各执行器按 URDF effort（转动 100 N·m，夹爪每指 50 N），
  再加上每个关节自己的 `total_output_max`。

已知差异：没有电机电流环、CAN 延迟、编码器噪声和温度模型；反馈是理想值（只做了 16 bit 量化）。
`GravityModel` 只保留质量/质心形式；原来 DIY 臂的 12 维 `beta` 参数不再使用（在 git 历史里）。

## 接自己的算法

1. **换仿真输入**：改 `main.cpp` 的 `kInputEvents`，或在循环里直接调 `arm.SetState()` / `arm.SetHostTarget()` / `arm.SetIkTarget()` / `arm.SetGripperTarget()`。
2. **换上层控制**：在 `Algorithm/` 加算法类，在 `Arm::ControlJoints()` 里替换或叠加 PID 输出。

新增源文件记得加到 `CMakeLists.txt` 的 `add_executable(armsim ...)`。

## 相关

- 模型：`../mujoco/BigArm/robot.xml`（由 `make_mjcf.py` 从 URDF 生成），旧的 `../mujoco/DIY/robot.xml` 仍在，但参数表已不对应
- 实机工程：`Real/DIYcontroller`
