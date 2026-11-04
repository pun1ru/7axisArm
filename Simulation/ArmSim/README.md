# ArmSim — 七轴臂 MuJoCo 仿真框架

把 `Real/DIYcontroller` 的控制代码搬到 PC 上跑：算法、模块、状态机基本原样保留，
只把最底下的“CAN 收发 + 电机”换成 MuJoCo。改算法不用碰物理引擎，换模型不用碰算法。

```
Algorithm/   机械臂算法        PID、改进 DH、正运动学、重力前馈
Module/      arm 对象          Arm_Init / Arm_Run，状态机、关节闭环、目标管理
Driver/      电机驱动          Motor_t 风格接口，收发换成读写 SimPlant
System/      系统层            MuJoCo 启停、任务调度、模型参数、仿真输入
main.cpp     命令行入口
```

## 分层

### Algorithm — 只依赖数学
| 文件 | 内容 | 对应实机 |
| --- | --- | --- |
| `pid.h/.cpp` | `PIDStruct`、`PIDInitialize`、`PIDUpdate` | `LibSE/Algorithm/PID` |
| `dh.h/.cpp` | 改进 DH（Craig）：`T = Rx(α)Tx(a)Rz(q+θoff)Tz(d)` | `LibSE/Algorithm/Kinematics` |
| `gravity.h/.cpp` | 12 维 `beta` 重力模型 `tau = Y(q)·beta` | `LibSE/Algorithm/ArmDynamics` |
| `position_tran.h` | Mat3/Euler/Quat/Trans4 工具，纯头文件 | `LibSE/Algorithm/math` |

这一层没有任何硬件、MuJoCo、RTOS 依赖，单元测试直接调。

### Module — 七轴臂对象
`Arm_t` 的划分和实机一致：`config`（电机 / 状态 / 参数）+ 运行时状态。
每周期 `Arm_Run()` 固定四步：

```
观测(从 Motor_t.feedback 拿角度) -> 正运动学 -> 重力前馈 -> 关节 PID -> 写 motorcmd
```

参数（DH、限位、增益、beta）不写死在模块里，由 System 层填 `ArmConfig_t`，
和实机 `module_init.c` 的习惯一致。

### Driver — 电机封装
`Motor_t / MotorCmd_t / Feedback_t / MotorOps_t` 与 `LibSE/Drivers/Motor/Motor.h`
同名同用法，`sendcommand/enable/disable/protect/requestfeedback` 都在。
区别只有实现：

- 真机：`sendcommand` 组 CAN 帧发出去，反馈由 CAN 中断填。
- 仿真：`sendcommand` 只把力矩缓存到 `SimMotor_t.command_torque`；
  `SimMotor_WriteToPlant()` 在物理步进前统一写 `qfrc_applied`；
  `SimMotor_UpdateFeedback()` 从 `SimPlant` 读 `qpos/qvel` 并模拟 16 bit 编码器、
  单圈/多圈角、RPM、力矩反馈。

反馈字段、单位、零点/方向/减速比的处理方式都和真机相同，所以 Module 层看不出区别。

### System — 仿真系统
- `sim_plant.*`：MuJoCo 的唯一入口（`mj_loadXML` / `mj_makeData` / `mj_step`）。
  按名字找 `joint1..jointN` 和 `tcp` site；力矩走 `qfrc_applied`，**不需要 MJCF 里有 actuator**，
  所以 `DIY/robot.xml` 这种 `nu=0` 的模型能直接跑。
- `system.cpp`：实例化 `SimPlant_t`、7 个 `SimMotor_t`、`Arm_t`；填 DH/限位/增益/重力参数；
  一个单线程任务表按周期跑（替代 FreeRTOS）：

  | 任务 | 周期 | 作用 |
  | --- | --- | --- |
  | `ArmTask` | 2 ms | 刷新电机反馈 → `Arm_Run` |
  | `PlantTask` | 2 ms | 清力矩 → 写 7 个电机力矩 → `mj_step` |
  | `MonitorTask` | 100 ms | 打印关节角、TCP、最大力矩、warning 数 |

  `System_InputRun()` 是仿真的“遥控器/上位机”，演示时间线写在这里。

## 构建与运行

依赖 MuJoCo 3.x SDK（默认 `/opt/mujoco`，含 `include/mujoco/mujoco.h` 和 `lib/libmujoco.so`）。

```bash
cmake -S . -B build -DMUJOCO_ROOT=/opt/mujoco
cmake --build build -j
./build/armsim                 # 8 s，按实时节拍
./build/armsim --fast          # 不限速，尽快跑完
./build/armsim 4.0             # 跑 4 s（第二位置参数是秒数）
./build/armsim --xml ../mujoco/BigArm/robot.xml 5.0
```

默认模型是 `../mujoco/DIY/robot.xml`（CMake 会注入绝对路径）。

默认时间线：

```
0.2 s  PROTECTED -> ENABLE   全部关节闭环，目标跟随当前角，靠重力前馈托住
0.5 s  -> HOST               所有关节闭环在当前角，J1 目标 +20 deg
3.5 s                        J1 目标回 0
8.0 s  结束
```

> ENABLE 只是过渡。这个状态每个周期把目标设成当前观测（PID 误差恒为 0），
> 全靠重力前馈顶着；仿真里减速器摩擦有限，停久了会慢慢漂，所以 0.5 s 就切 HOST。

## 接自己的算法

两种方式，按改动大小选：

1. **换仿真输入**（最常用）：改 `System/system.cpp` 的 `System_InputRun()`，
   或者在 `ArmTask` 里调 `Arm_SetHostTarget()` / `Arm_SetState()`。
   算法还是在 `Arm_Run` 里跑，物理和驱动都不用动。
2. **换上层控制**：在 `Algorithm/` 加文件，在 `Module/arm.cpp` 的
   `Arm_JointControl()` 里替换或叠加 PID 输出；也可以在 `System/` 新加一个任务，
   直接通过 `SimPlant_SetJointTorque()` 下发力矩，跳过 Module。

新增源文件记得加到 `CMakeLists.txt` 的 `add_executable(armsim ...)`。

## 保真度说明

`DIY/robot.xml` 来自 URDF，质量/惯量/运动学没改。下面几项是**仿真专用、不属于 URDF**，
在 `System/system_define.h` 里调：

| 参数 | 默认 | 作用 |
| --- | --- | --- |
| `SIM_JOINT_ARMATURE` | `1e-3 kg*m^2` | 减速器折算到关节的转子惯量 |
| `SIM_JOINT_DAMPING` | `1.5 N*m*s/rad` | 粘性阻尼，实机齿轮箱/反电势的近似 |
| `SIM_JOINT_FRICTIONLOSS` | `0.15 N*m` | 库仑摩擦，让远端关节能停住不漂 |
| `SIM_TORQUE_LIMIT_NM` | `20 N*m` | 写进物理引擎前的安全限幅（另有一层模块内 `total_output_max`） |

不加这三项时，远端关节（J6 惯量约 `1.5e-5 kg*m^2`）纯力矩控制会抖、慢漂；
加完后阶跃响应单调、能停住，和实机手感接近。全部置 0 就是“纯 MJCF”。

其他对齐点：

- **增益、限位、DH、beta 全部来自实机**，PID 误差仍是“度”，输出仍是 N·m。
- 仿真电机的零点 = 0、方向 = +1、`inverse = false`、减速比 = 1：
  模型关节和电机同轴，不需要实机的装配标定值。
- 重力前馈的 `beta` 来自实机辨识。用 MuJoCo 的 `qfrc_bias` 对比过：
  符号一致、量级接近（例如 `q=[0,10,0,...]` 时 J1 bias `-0.323`、模型 `-0.392`），
  残差在仿真阻尼/摩擦的量级内。
- J0 绕竖直轴，重力矩恒为 0，重力输出 mask 里关掉了 J0，和实机一致。

已知差异：没有电机电流环、CAN 延迟、编码器噪声和温度模型；
仿真反馈是理想值（只做了 16 bit 量化）。需要时在 Driver 层加，不影响上层。

## 相关

- 模型来源：`../mujoco/DIY/robot.xml`（由 URDF 生成，见 `../mujoco/BigArm/make_mjcf.py`）
- 实机工程：`Real/DIYcontroller`
- DH 参数与帧定义：`../mujoco/DIY/dh_params.md`
