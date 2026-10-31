# 06 整合、部署与验收

> 对应 README 第 6 项。

## 1. 整合前检查出的冲突与缺失（及处理）

| # | 冲突 / 缺失 | 处理 |
|---|---|---|
| 1 | 固件 `observer.joint_deg` 已改为**多圈角**，并新增 `joint_abs_deg`；协议文档原写 angle_deg 为 (-180,180] | 上位机按多圈角处理（三维用三角函数，滑块显示时折到 ±180）。**协议文档第 2、5 节需要你确认后改口径**；`joint_abs_deg` 目前没上报，需要的话在关节记录末尾追加（向前兼容） |
| 2 | 固件 `ArmJointCtrl_t` 新增 `limit_min_deg / limit_max_deg`（多圈锁存区间） | 协议未上报，不影响现有功能；需要时同样追加到记录末尾 |
| 3 | 固件 `ARM_LIMIT_Jx`、`ARM_GAIN_Jx` 已改成实测值 | 上位机不缓存这些值，读参数时以设备为准 |
| 4 | 界面只认 MockBackend / SerialBackend | 新增 `SimBackend`（仿真），和其它后端同一接口 |
| 5 | 没有配置文件 | 新增 `config/settings.toml`（只读，标准库 tomllib），坏文件弹窗提示并用默认值 |
| 6 | 未捕获异常会被 Qt 吞掉 | 全局 `sys.excepthook` + `threading.excepthook`：写日志并弹窗 |
| 7 | `Real/` 下的固件参考实现仍未移植 | 不动 `Real/`，见第 7 节 |

已通过测试的代码（协议、通信、波形）**没有重写**，只做了接入。

## 2. 依赖方向

```
ui (PySide6) ──► controller ──► model.backend ──► comm ──► protocol
     │                              │
     │                              └─► model.sim ──► mujoco（可选）
     └──► kinematics（纯 numpy）
```

* UI 不 import `serial`、不碰 `SerialTransport`；只通过 `RobotBackend` 的方法（`estop`、`read_dh`、`set_joint_torque`…）下指令。
* UI 不碰 `mjData`：仿真只暴露 `set_targets / set_joint_torque / set_mode / estop`，由仿真线程在步进之间写入。
* `protocol`、`comm`、`kinematics`、`model` 都不依赖 Qt，可单独测试。

## 3. 数据源

| 数据源 | 用途 | 是否需要硬件 |
|---|---|---|
| 模拟设备 | 走完整协议栈（编码→虚拟串口→解析），安全语义与固件约定一致 | 否 |
| 模拟设备（含噪声） | 注入噪声、翻转位、丢字节、随机拆包，验证重同步 | 否 |
| 仿真 | 装了 mujoco 用 `robot.xml` 真实动力学，否则内置简化模型；hold 模式 PD + 重力补偿 | 否 |
| 串口 | 真实机械臂 | 是 |
| 回放 | CSV 按 MCU 时间节奏重放 | 否 |

## 4. 安装与运行

### Windows

```bat
cd Host
py -3.11 -m pip install -r requirements.lock   :: 推荐 Python 3.11
pip install mujoco                      :: 可选
start_mock.bat                          :: 双击也行：模拟设备演示
start.bat --sim                         :: 仿真
start.bat --port COM5                   :: 真机
```

`start.bat` 会优先选择 `py -3.11`。不要用 Python 3.14 + PySide6 6.11 运行实时波形：
这组组合在 pyqtgraph 重绘时会偶发 Qt 原生 access violation，Python 层无法捕获。

崩溃记录都在 `logs/`：

- `crash.log`：faulthandler 的原始原生堆栈；
- `crash_history.log`：下次启动时归档的历史原生崩溃；
- `armhost.log`：上次异常退出摘要、未捕获异常和正常通信日志。

串口：设备管理器 → 端口（COM 和 LPT）查看编号；USB 转串口需要装对应驱动（CH340 / CP210x）。

### Linux

```bash
cd Host
python3 -m pip install -r requirements.lock
sudo apt install libxcb-cursor0          # PySide6 6.5+ 在 Ubuntu 上需要
bash start.sh --mock
```

串口权限：

```bash
sudo usermod -aG dialout $USER           # 重新登录生效；Arch 用 uucp 组
ls -l /dev/ttyUSB* /dev/ttyACM*
```

`start.sh` 检测到不在 dialout 组时会提醒。

### 打包（PyInstaller）

```bash
pip install pyinstaller
pyinstaller -n armhost --windowed --add-data "config;config" -p . armhost/app.py      # Windows
pyinstaller -n armhost --windowed --add-data "config:config" -p . armhost/app.py      # Linux
```

产物在 `dist/armhost/`。仿真要用 MuJoCo 时，把 `Simulation/mujoco/` 放到 `dist/` 旁边，并加 `--collect-all mujoco`。没装 mujoco 也能打包，仿真自动用内置模型。

## 5. 配置、日志、错误提示

| 项 | 位置 |
|---|---|
| 配置 | `config/settings.toml`（串口、缓存时长、刷新率、模拟/仿真参数、日志级别） |
| DH 表 | `config/dh.json`（界面里"保存为默认"后生成） |
| 日志 | `logs/armhost.log`，5 MB × 5 轮转；原始收发字节用 logger `armhost.raw`（DEBUG） |
| 错误提示 | 配置错误、连接失败、仿真异常、未捕获异常都会弹窗，同时写日志 |

## 6. 验证结果

`python -m unittest discover -s tests`：83 个测试，1 个跳过（没装 mujoco）。

| README #6 要求 | 测试 |
|---|---|
| 串口断开 | `test_comm.test_unplug_and_reconnect`、`test_not_connected_fails_fast` |
| 损坏数据帧 | `test_protocol.TestParser.*`、`test_comm.TestLinkWithFaults` |
| 数据突发 | `test_comm.TestBurst.test_500hz`、`test_ui.test_refresh_cost` |
| 长时间运行 / 缓存上限 | `test_sim_system.test_long_run_buffer_bounded`、`test_model.TestBuffer` |
| 仿真停止 | `test_sim_system.test_divergence_reported`（发散后报错并停止，不卡界面） |
| 模式切换 | `test_sim_system.test_mode_switch_cycle`（模拟↔仿真 反复切换，线程不泄漏、数据不串台） |
| 真机安全状态 | `test_sim_system.test_switch_away_sends_estop`、`test_comm.test_device_link_timeout_clears_torque`、`test_torque_never_retried` |
| 关窗口时线程退出 | `test_ui` 的 tearDown + `test_mode_switch_cycle` 的线程计数 |

## 7. 系统验收清单（需要在真机上逐项确认）

- [ ] `start_mock.bat` 双击能打开，波形、关节表、三维都在动
- [ ] 数据源选"仿真"，三维里机械臂保持在零位附近，不下垂
- [ ] 真机：连接后状态面板显示设备信息、MCU 采样频率与固件设定一致
- [ ] 真机：拔掉串口线，状态变为 reconnecting，插回后自动恢复，无需重启程序
- [ ] 真机：手动转动 J0~J6，三维里对应关节同方向转动（核对 direction）
- [ ] 真机：零位姿态下，三维显示为全伸直
- [ ] 急停按钮按下后，事件页出现"急停已确认"
- [ ] 关闭窗口：设备收到急停 + 清零，程序 2 秒内退出，任务管理器里没有残留 python 进程
- [ ] 记录 1 分钟 CSV 并回放，曲线与记录时一致

## 8. 线程与资源检查

| 线程 | 退出方式 |
|---|---|
| serial-rx / serial-tx / link-timer | `ArmLink.stop()`：先发急停 + 清零，再 join（超时 1 s，均为 daemon） |
| sim | `SimBackend.stop()`：置事件后 join |
| replay | `ReplayBackend.stop()` |
| csv-writer | `CsvRecorder.stop()`：写完队列里的剩余行再退出 |
| GUI 定时器 | `closeEvent` 里全部 stop |

缓存上限：波形环形缓存（默认 30000 点）、事件 deque 1000、hex 记录 2000、记录队列 20000（满了丢弃并计数）、串口发送队列 256（断线时清空，不补发）。
