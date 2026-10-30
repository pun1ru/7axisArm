# 05 三维姿态显示

> 对应 README 第 5 项。代码：`armhost/kinematics/dh.py`（正运动学）、`armhost/ui/view3d.py`（渲染）、`armhost/ui/pose_panel.py`（面板）。

## 1. 方案评估与选型

| 方案 | 依赖 / 部署成本 | 适用场景 | 对本项目 |
|---|---|---|---|
| Qt Quick 3D | PySide6 自带，但要写 QML，Python↔QML 传数据要绕一层；需要 GPU/RHI 后端 | 带材质、光照的产品级 3D 界面 | 只画线和坐标系，成本偏高；远程桌面 / 虚拟机上 RHI 不稳定 |
| VTK | `pip install vtk` 约 100 MB，打包体积大 | 科学可视化、点云、网格 | 功能远超需求，部署最重 |
| pyqtgraph.opengl | 需要 PyOpenGL；依赖显卡驱动 | 中等规模 3D 曲线 / 散点 | 可行，但无 GPU 环境、offscreen 测试会失败；中文标签麻烦 |
| MuJoCo 自带 viewer | 需要 mujoco；独立窗口 | 看仿真模型 | 不能嵌进 Qt 窗口，不能显示实机数据 |
| **QPainter 自绘透视投影（选用）** | **零额外依赖** | 几十条线段的线框 | 任何环境都能跑（含远程桌面、无显卡），offscreen 可截图测试，中文直接画 |

当前场景只有几十条线段（连杆、坐标轴、网格），QPainter 每帧 < 5 ms。以后要显示 STL 网格时再换成 Qt Quick 3D / OpenGL：渲染器只依赖 `DHTable + q_deg`，替换只动 `view3d.py`。

## 2. 坐标与单位（已与固件逐项核对）

* **DH 约定**：改进 DH（Craig），与固件 `dh.c` 完全相同：`T_i = RotX(α)·TransX(a)·RotZ(q_i+θ偏置)·TransZ(d)`，第 i 行对应关节 Ji。
* **帧编号**：帧 0 = 基座 O₀；关节 Ji 位于帧 i+1 的原点、绕帧 i+1 的 z 轴旋转。
* **单位**：DH 表与协议都用 deg / m，只在算三角函数时转 rad。
* **零位与方向**：用协议里的 `angle_deg`，即固件 `observer.joint_deg`——已减零点、已乘 `direction`。**固件已改为多圈角**，三角函数天然处理，不需要折叠。
* **末端姿态**：`euler_zyx_deg` 逐行照抄固件 `Matrix_ToEuler`，(roll, pitch, yaw) 与固件 `observer.euler_deg` 同口径。
* **测试**：`tests/test_kinematics.py` 核对 q=0 时各帧原点与 `Simulation/mujoco/dh_params.md` 第 7.3 节一致（62.6 / 210.6 / 310.6 mm），轴向与第 3 节一致（z₁=+Z, z₂=−Y …）。

## 3. DH 参数表

启动时按顺序取：

1. `config/dh.json`（在"DH 参数"页点过"保存为默认"）；
2. 单片机工程 `Real/DIYcontroller/App/module_init.c` 里的 `#define ARM_DH_Jx`（**只读**，不修改固件）；
3. 内置副本（只拷了 Host 目录时）。

"DH 参数"页可以直接改表（默认改完立即生效），也可以恢复单片机默认、导入/导出 JSON，或"从设备读取"（PARAM_READ 0x0400，需要固件实现参数读写；模拟设备已实现）。

## 4. 操作

| 操作 | 效果 |
|---|---|
| 左键拖动 | 旋转视角 |
| 右键 / 中键拖动 | 平移 |
| 滚轮 | 缩放 |
| 双击 | 复位视角 |
| 单击关节球 | 选中该关节，左上角显示它的角度、目标、角速度、力矩、状态 |
| 等轴 / 前视 / 侧视 / 俯视 | 预设视角 |

显示开关：连杆、关节坐标系（总开关 + 基座/J0~J6 逐个开关）、关节转轴、关节名称、地面网格、世界坐标轴、坐标轴长度。

## 5. 数据源解耦

面板只接收一个"取最新样本"的函数，不知道数据来自串口、模拟设备、仿真还是回放：

* **实时数据**：当前连接的数据源；
* **手动滑块**：离线核对 DH 表；"取当前实时姿态"把实机姿态复制到滑块上。

## 6. 渲染频率

面板定时器 30 Hz（`config/settings.toml` 的 `ui.view3d_fps`）拉一次关节角；角度、视角、开关都没变时**不重绘**。三维面板不可见（切到波形页）时完全不计算。渲染只在 GUI 线程，不影响串口线程和波形刷新。
