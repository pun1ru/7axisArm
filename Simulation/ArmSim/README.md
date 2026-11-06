# ArmSim — 七轴臂 MuJoCo 仿真框架

C++17 面向对象实现。控制算法、状态机、上位机协议和实机 `Real/DIYcontroller` 保持一致，
只把最底下的“CAN 收发 + 电机”换成 MuJoCo 的关节读写。

```
Algorithm/   算法            Pid、GravityModel、DH 正运动学（纯数学，无硬件依赖）
Driver/      MuJoCo 接口      MujocoWorld、JointActuator、HostLink
Module/      两个对象         Arm（状态机 + 关节闭环）、Host（上位机协议桥）
main.cpp     构造对象 + 步进仿真 + 脚本输入 + 终端监视
```

## 对象关系

```
main ──构造──► MujocoWorld ◄── JointActuator ×7 ◄── Arm ◄── Host ──► HostLink (pty)
                (mj_step)       读角度 / 写力矩      (Arm 内部持有)    (仅 --host)
```

| 类 | 位置 | 职责 |
| --- | --- | --- |
| `MujocoWorld` | `Driver/mujoco_world.*` | 唯一包含 `mujoco.h` 的地方。加载 MJCF，按名字找 `joint1..jointN` 和 `tcp` site，读关节角/速度，写 `qfrc_applied`（内部限幅，NaN 当 0），`Step()` |
| `JointActuator` | `Driver/joint_actuator.h` | 一个关节的执行器：`angle_rad()` / `velocity_rads()` / `SetTorque()`。电机与关节同轴、理想响应，所以没有电机状态机、控制模式、CAN 帧 |
| `HostLink` | `Driver/host_link.*` | 上位机链路。实机是 UART10，仿真里是一对 pty，`/tmp/armsim-host` 是软链接。仅 Linux / macOS |
| `Arm` | `Module/arm.*` | 状态机、每关节 PID、重力前馈、总力矩限幅。方向和零点在这里折算 |
| `Host` | `Module/host.*` | 上位机协议桥，帧格式、消息号、参数号与实机一致，`Host/armhost` 无需修改 |
| `Pid` / `GravityModel` / `ForwardKinematics` | `Algorithm/` | 算法 |

## 每个控制周期（2 ms）

`main.cpp` 里的循环：

```
脚本输入 -> arm.Step() -> host->Step() -> world.Step() -> 监视输出
```

`Arm::Step()` 内部：观测（执行器读数 -> 关节角 -> 正运动学）-> 重力前馈 -> 更新目标 -> 关节 PID + 前馈 -> 写执行器力矩。
没有任务表和调度器：所有任务都是 2 ms，监视每 50 步（100 ms）打印一次。
控制周期必须等于 MJCF 的 `timestep`，不一致时启动会提示。

## 构建与运行

依赖 MuJoCo 3.x SDK（默认 `/opt/mujoco`）。

```bash
cmake -S . -B build -DMUJOCO_ROOT=/opt/mujoco
cmake --build build -j
./build/armsim                 # 8 s，按实时节拍
./build/armsim --fast          # 不限速，尽快跑完
./build/armsim ../mujoco/DIY/robot.xml 4.0   # 参数依次是模型、秒数
```

默认模型是 `../mujoco/DIY/robot.xml`（CMake 注入绝对路径）。模型需要 `joint1..jointN` 和 `tcp` site。

默认时间线（`main.cpp` 的 `kInputEvents`）：

```
0.2 s  PROTECTED -> ENABLE   全部关节闭环，目标跟随当前角，靠重力前馈托住
0.5 s  -> HOST               所有关节闭环在当前角，J1 目标 +20 deg
3.5 s                        J1 目标回 0
8.0 s  结束
```

ENABLE 只是过渡：每周期把目标设成当前观测（PID 误差恒为 0），全靠重力前馈顶着。
仿真里关节摩擦有限，停久了会慢慢漂，所以 0.5 s 就切 HOST。

## 和上位机（Host/armhost）对接

```
Host/armhost (pyserial) ──► /tmp/armsim-host ──► pty ──► HostLink ──► Host ──► Arm
```

```bash
./build/armsim --host              # 开链路，等上位机；Ctrl-C 退出
```

```python
from armhost.comm import ArmLink, SerialConfig, SerialTransport
link = ArmLink(SerialTransport(SerialConfig(port="/tmp/armsim-host", baudrate=921600)))
link.start()
link.set_host_target(1, 20.0, closed_loop=True).result(1.0)   # J1 转到 20 deg
```

`--host` 下臂直接进入 HOST 状态，不跑脚本、不刷终端；所有关节一开始都不闭环，
只有重力前馈托着，等上位机发 `SET_HOST_TARGET`。仿真强制按实时节拍跑，`--fast` 无效。

| 方向 | 消息 | 说明 |
| --- | --- | --- |
| 上行 | `JOINT_STATE` | 100 Hz，7 关节的角度、速度、力矩、目标、指令力矩、编码器、状态标志 |
| 上行 | `ACK` / `PARAM_VALUE` / `DEVICE_INFO` | 命令应答；同一 (type, seq) 重发时重放上次应答 |
| 下行 | `HEARTBEAT` | 只刷新链路存活时间 |
| 下行 | `DEVICE_INFO_REQ` | `fw_name` 为 `ArmSim` |
| 下行 | `PARAM_READ` | 状态频率、限位、PID 增益、DH、方向、零点（只读） |
| 下行 | `SET_HOST_TARGET` | 单关节目标 + 是否闭环，或全部退出闭环 |

未实现的消息（力矩直控、急停、参数写等）回 `UNSUPPORTED`。上位机掉线超过 300 ms，
`Host::Step` 会让所有关节退出闭环，与实机一致。

与实机的差别：

- 链路超时按**墙钟**算，`JOINT_STATE` 里的时间戳是**仿真时间**。
- 应答直接写链路，不经发送队列。
- 电机状态只有使能/失能两种，`error_id`、温度恒为 0。
- 零点参数是 16 bit 编码器计数，由 rad 零点折算（仿真里恒为 0）。

## 接自己的算法

1. **换仿真输入**（最常用）：改 `main.cpp` 的 `kInputEvents`，或在循环里直接调
   `arm.SetState()` / `arm.SetHostTarget()`。
2. **换上层控制**：在 `Algorithm/` 加算法类，在 `Arm::ControlJoints()` 里替换或叠加 PID 输出；
   也可以在 `main.cpp` 里直接用 `JointActuator::SetTorque()` 下发力矩，跳过 `Arm`。

新增源文件记得加到 `CMakeLists.txt` 的 `add_executable(armsim ...)`。

## 仿真专用参数

`DIY/robot.xml` 来自 URDF，质量/惯量/运动学没改。下面几项不属于 URDF，在 `main.cpp` 顶部调：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `kJointArmature` | `1e-3 kg*m^2` | 减速器折算到关节的转子惯量 |
| `kJointDamping` | `1.5 N*m*s/rad` | 粘性阻尼 |
| `kJointFrictionLoss` | `0.15 N*m` | 库仑摩擦，让远端关节能停住不漂 |
| `kTorqueLimitNm` | `20 N*m` | 写进物理引擎前的安全限幅（另有一层 `total_output_max`） |

不加前三项时，远端关节（J6 惯量约 `1.5e-5 kg*m^2`）纯力矩控制会抖、慢漂。全部置 0 就是“纯 MJCF”。

其他对齐点：

- 增益、限位、DH、beta 全部来自实机（`main.cpp` 的 `MakeArmConfig()`），PID 误差仍是度，输出仍是 N·m。
- 仿真里零点 0、方向 +1：模型关节和电机同轴，不需要实机的装配标定值。
- J0 绕竖直轴，重力矩恒为 0，重力前馈的 mask 里关掉了 J0，和实机一致。
- `GravityModel` 只保留 12 维 `beta` 版本；实机里用于核对辨识结果的 28 维 `pi` 版本没有搬过来。

没有电机电流环、CAN 延迟、编码器噪声和温度模型；反馈是理想值（只做了 16 bit 量化）。

## 相关

- 模型来源：`../mujoco/DIY/robot.xml`（由 URDF 生成，见 `../mujoco/BigArm/make_mjcf.py`）
- 实机工程：`Real/DIYcontroller`
- DH 参数与帧定义：`../mujoco/DIY/dh_params.md`
