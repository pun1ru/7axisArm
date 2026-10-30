# 01 上位机整体架构

> 对应 README 第 1 项。本文只定架构和接口边界，不含 UI 代码。

## 0. 需求边界

### 已确定（来自 README 与 `Real/DIYcontroller` 工程）

| 项 | 依据 |
|---|---|
| 语言 / 框架：Python + PySide6 + pyqtgraph + PySerial | README |
| 跨 Windows / Linux | README |
| 关节数 7（J0~J6） | `arm.h` `ARM_JOINT_COUNT 7U` |
| 关节角单位 deg，口径 (-180, 180]，方向已按 `direction` 折算 | `ArmObserver_t.joint_deg` |
| 电机反馈有角速度 rad/s、力矩 N·m、温度、状态码、错误码 | `Feedback_t` |
| DH 为改进 DH，7 行 `{alpha_deg, a_m, d_m, theta_offset_deg}` | `module_init.c` |
| 控制任务周期 2 ms（500 Hz） | `TASK_ARM_PERIOD_MS 2U` |
| 机械臂状态机 PROTECTED / ENABLE / FEEDBACK / MASTER（主机角色） | `ArmState_t` |
| 力矩真正经 CAN 下发由单片机按遥控状态决定，上位机只"设置" | README #2 |

### 需要你补充（未补充前按"建议值"实现，均可在配置里改）

| # | 待确认 | 建议值 | 理由 |
|---|---|---|---|
| Q1 | 物理链路：UART 还是 USB CDC？UART 用哪个口？ | UART7 / USART2/3/10 任一空闲口，921600 8N1 | 这四个口工程里已配 921600 且未绑定解析器 |
| Q2 | 状态上报频率 | 200 Hz（UART）/ 500 Hz（USB） | 见协议文档带宽计算 |
| Q3 | 上位机力矩设定在哪个 Arm 状态下生效 | 仅 `ARM_ENABLE` 且遥控允许时，叠加或替代 PID 输出由固件决定 | 现有状态机里没有"上位机力矩"模式 |
| Q4 | 各关节力矩硬上限 | 固件编译期常量，上位机只能在其以内设 | 安全 |
| Q5 | MuJoCo 接入方式：同进程 Python API 还是独立进程 | 先同进程（`mujoco` 包），接口按可换成进程间设计 | 部署最简单 |
| Q6 | 是否需要控制模式之外的位置/速度指令 | 暂不做 | README 只要求力矩设置 |

## 1. 分层与模块

```
┌──────────────────────────── UI 层 (PySide6, 主线程) ─────────────────────────────┐
│  MainWindow  JointTable  ScopeWidget(pyqtgraph)  View3D  CommPanel  RecordPanel   │
└──────────────▲──────────────────────────▲──────────────────────────▲────────────┘
               │ Qt 信号 / 只读快照          │                          │ 命令(方法调用)
┌──────────────┴──────────── 应用服务层 ─────┴──────────────────────────┴────────────┐
│  RobotStateStore(状态模型)  WaveformBuffer(环形缓存)  Recorder/Player  CommandService│
└──────────────▲─────────────────────────────────────────────────────▲──────────────┘
               │ RobotSample                                         │ Command
┌──────────────┴──────────────── 后端抽象  RobotBackend ──────────────┴──────────────┐
│     SerialBackend(真实硬件)        MujocoBackend(仿真)        MockBackend(模拟)     │
└──────────────▲──────────────────────────────▲─────────────────────────────────────┘
┌──────────────┴──────── 通信层 ──────────────┐   ┌─────── MuJoCo 适配器 ─────────┐
│ ArmLink: 发送队列 / 序号 / 应答匹配 / 统计     │   │ 模型加载、步进线程、关节映射      │
│ FrameParser: 粘包拆包重同步                    │   └───────────────────────────────┘
│ protocol.messages: 编解码（与固件结构体同步）   │
│ SerialTransport: PySerial / 虚拟串口           │
└──────────────────────────────────────────────┘
```

| 模块 | 包路径 | 职责 | 不能做的事 |
|---|---|---|---|
| 串口驱动层 | `armhost.comm.transport` | 端口枚举、打开/关闭、参数、读写线程、异常恢复 | 不理解协议 |
| 协议编解码层 | `armhost.protocol` | CRC、帧格式、消息 dataclass、流式解析 | 不碰 IO、不依赖 Qt |
| 通信抽象层 | `armhost.comm.link` | 序号、发送队列、ACK 匹配、心跳、统计、原始 hex 日志 | 不依赖 Qt |
| 机器人状态模型 | `armhost.model` | 统一 `JointState` / `RobotSample`，最新值 + 历史 | 不做 IO |
| 后端接口 | `armhost.backend` | 真实 / 仿真 / 模拟可切换 | 不直接操作 UI |
| 波形模块 | `armhost.ui.scope` | 定时批量从环形缓存取数绘图 | 不逐点建对象 |
| 三维显示 | `armhost.ui.view3d` + `armhost.kinematics` | DH 正运动学 + 渲染 | 不读串口 |
| MuJoCo 适配器 | `armhost.sim.mujoco_adapter` | 加载 `Simulation/mujoco/robot.xml`、步进、映射关节 | UI 不得直接改 `mjData` |

依赖方向严格自上而下；`protocol` 和 `comm` 不依赖 PySide6（第 3 项已按此实现，可脱离 GUI 单测）。

## 2. 可切换后端接口

```python
class RobotBackend(Protocol):
    name: str
    def start(self) -> None: ...
    def stop(self) -> None: ...                       # 必须让机器人进入安全状态再返回
    def is_running(self) -> bool: ...
    def subscribe(self, cb: Callable[[RobotSample], None]) -> Callable[[], None]: ...
    def set_joint_torque(self, items: dict[int, float], hold_ms: int) -> Future[Ack]: ...
    def estop(self, latch: bool = True) -> Future[Ack]: ...
    def read_param(self, pid: int, index: int) -> Future[ParamValue]: ...
    def write_param(self, pid: int, index: int, value) -> Future[Ack]: ...
    def capabilities(self) -> BackendCaps: ...        # 关节数、能否写参数、是否真实硬件
```

* `SerialBackend` = `ArmLink` + 状态转换；`MujocoBackend` 在仿真线程里把 `qpos/qvel/qfrc_actuator` 转成同样的 `RobotSample`；`MockBackend` 产生正弦数据。
* 切换后端：先 `old.stop()`（发急停/清零），再 `new.start()`；UI 只认 `RobotBackend`。

## 3. 线程模型

| 线程 | 内容 | 与其它线程的交互 |
|---|---|---|
| GUI 主线程 | Qt 事件循环，`QTimer` 30 Hz 刷新波形、10 Hz 刷新表格、30 Hz 刷新三维 | 只读快照 / 从环形缓存批量取 |
| 串口读线程 | `serial.read()` 阻塞（超时 50 ms）→ `FrameParser.feed()` → 回调 | 回调里只做解码 + 入队，绝不调 Qt 控件 |
| 串口写线程 | 从 `queue.Queue` 取帧写串口 | 写错误计数并触发重连 |
| Link 定时线程 | 心跳、ACK 超时、重发 | 线程安全的 pending 表（`threading.Lock`） |
| 仿真线程 | MuJoCo `mj_step`，按实时或倍速 | 同样产出 `RobotSample` |
| 记录线程 | CSV 写盘（队列消费） | 避免磁盘 IO 卡 GUI |

跨线程通信：
* **后端 → UI**：`RobotSample` 进入 `RobotStateStore`（加锁写最新值 + `collections.deque(maxlen)` 历史）；UI 定时器主动拉取，**不是每帧发一个 Qt 信号**。低频事件（连接状态、ACK、错误事件）用 Qt 信号（`QueuedConnection`）。
* **UI → 后端**：方法调用把命令放入发送队列，返回 `concurrent.futures.Future`，UI 用回调或轮询拿结果，不阻塞。

## 4. 统一数据结构与单位

```python
@dataclass(frozen=True, slots=True)
class JointState:
    index: int
    angle_deg: float          # 关节角，deg，(-180,180]，+ 为关节 +q 方向（固件已按 direction 折算）
    velocity_dps: float       # 关节角速度，deg/s，同向
    torque_nm: float          # 电机反馈实际力矩，N·m，同向
    torque_cmd_nm: float      # 本周期下发的力矩指令，N·m
    target_deg: float         # 角度环目标，deg
    temperature_c: int
    motor_status: MotorStatus # 与固件 MotorStatus_t 同值
    error_id: int
    online: bool

@dataclass(frozen=True, slots=True)
class RobotSample:
    mcu_time_us: int          # MCU 采样时刻（固件 µs 计数，32 位回绕已展开为 64 位）
    host_time_ns: int         # 上位机收到该帧的 time.monotonic_ns()
    seq: int
    arm_state: ArmState
    flags: int
    joints: tuple[JointState, ...]
```

时间戳两条线**不得混用**：曲线横轴默认用 `mcu_time_us`（反映真实采样间隔），接收频率和延迟统计用 `host_time_ns`。仿真后端的 `mcu_time_us` 填仿真时间。

## 5. 目录结构

```
Host/
├─ README.md                 需求（原文）
├─ docs/01_architecture.md   本文
├─ docs/02_protocol.md       串口协议
├─ firmware_ref/             STM32 端参考实现（不进 Real，由你决定是否移植）
├─ armhost/
│  ├─ protocol/              crc.py frame.py messages.py parser.py
│  ├─ comm/                  transport.py link.py mock.py stats.py hexlog.py
│  ├─ model/  backend/  kinematics/  sim/  ui/   （第 4~6 项实现）
├─ tests/                    pytest
└─ examples/                 运行示例
```

## 6. 异常处理与日志

* 标准 `logging`，logger 名按包（`armhost.comm.transport` 等），文件 `logs/armhost.log` 轮转 5×5 MB。
* 原始收发字节单独 logger `armhost.raw`，默认关闭，UI 打开后写入环形 hex 缓冲 + 可选文件。
* 串口异常（拔线、`SerialException`）：读写线程捕获 → 状态置 `DISCONNECTED` → 按退避（0.5/1/2/5 s）重连；**重连前所有未确认命令的 Future 以 `LinkLost` 失败**，不会重发力矩命令。
* 协议错误（CRC、长度、版本）只计数 + 低频日志，不抛给 UI。
* 回调异常被吞并记录，不能杀死读线程。

## 7. 面向后续扩展

| 扩展 | 预留点 |
|---|---|
| 轨迹控制 | `CommandService` 层生成时间序列 → 周期调用 `set_joint_torque`；协议预留消息类型段 0x30~0x3F |
| 参数调节 | 协议 `PARAM_READ/WRITE`，参数表在 `protocol/params.py`，UI 自动生成表单 |
| 数据回放 | `Recorder` 写 CSV（含两条时间戳），`ReplayBackend` 实现 `RobotBackend` |
| 仿真同步 | `MujocoBackend` 可订阅真实后端的 `RobotSample`，把实测角写入 `qpos` 做影子显示 |

## 8. 分阶段计划与验收

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| P1 | 架构 + 协议文档 | 你确认 Q1~Q6 |
| P2 | `protocol` + `comm` + 模拟设备 + 单测 | `pytest` 全绿；模拟设备 200 Hz 下 10 min 无丢帧；注入噪声后能重同步 |
| P3 | 固件侧移植 `firmware_ref` | 真机 `DEVICE_INFO` 握手成功；拔线 300 ms 内上位机力矩设定失效 |
| P4 | 监视界面 + 波形 + CSV | 500 Hz 数据下 GUI ≥ 30 FPS，CPU < 30% |
| P5 | 三维显示 | 与 `Simulation/mujoco` 同一关节角下末端位置误差 < 1 mm |
| P6 | 整合 + MuJoCo + 打包 | README #6 验收清单全部通过 |
