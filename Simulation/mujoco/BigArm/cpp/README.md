# BigArm C++ 控制程序骨架

在 C++ 里加载 `robot.xml`、每 2 ms 读一次状态、算出 8 个关节的力矩并写回
MuJoCo。**只替换 `main.cpp` 里 `Controller::update()` 的实现**，就能接入自己的
算法；模型加载、关节/执行器查找、实时步进这些管线都不用动。

```
cpp/
├── main.cpp         # 控制循环 + 示例控制器
├── CMakeLists.txt
└── README.md
```

## 编译运行

Linux（MuJoCo SDK 在 `/opt/mujoco`）：

```bash
cd Simulation/mujoco/BigArm/cpp
cmake -S . -B build -DMUJOCO_ROOT=/opt/mujoco
cmake --build build
./build/bigarm_ctrl ../robot.xml 5     # 跑 5 秒，参数缺省为 robot.xml / 5 s
```

Windows（先解压 `mujoco-3.x.x-windows-x86_64.zip`，假设在 `E:/tools/mujoco`）：

```powershell
cd Simulation\mujoco\BigArm\cpp
cmake -S . -B build -DMUJOCO_ROOT=E:/tools/mujoco -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
copy E:\tools\mujoco\bin\mujoco.dll build\Release\
.\build\Release\bigarm_ctrl.exe ..\robot.xml 5
```

`mujoco.dll` 必须和 exe 放在一起（或把 `bin` 加进 `PATH`），否则运行时找不到。

正常输出（示例控制器保持初始位姿，`tau1 = tau2 ≈ -7.793 N·m` 是重力矩）：

```
t= 0.00  q1= 0.0000 q2= 0.0000 q3= 0.0000 q4= 0.0000  tau1=  -7.793 tau2=  -7.793
t= 3.00  q1=-0.0000 q2= 0.0000 q3=-0.0000 q4=-0.0000  tau1=  -7.793 tau2=  -7.793
```

## 接入自己的算法

`RobotState` 是控制器能看到的全部信息，`update()` 返回 `joint1..joint8` 的力矩：

```cpp
struct RobotState {
  std::array<double, kN> q;      // 关节角 [rad]
  std::array<double, kN> dq;     // 角速度 [rad/s]
  std::array<double, kN> ddq;    // 角加速度 [rad/s^2]
  std::array<double, kN> bias;   // C(q,dq) + g(q)，保持不动所需的力矩
};

class Controller {
 public:
  virtual void reset(const RobotState&) {}                            // 开始时调用一次
  virtual std::array<double, kN> update(const RobotState&, double t) = 0;
};
```

两种接法：

1. **改示例类**：直接改 `PdGravityController::update()` 的公式。
2. **加新类**：继承 `Controller`，在 `main()` 里把
   `PdGravityController controller(kp, kd);` 换成你的类。算法本身不依赖
   MuJoCo 类型，移植到真机时只需要把读状态/写力矩换成你的驱动即可。

```cpp
class MyController : public Controller {
 public:
  void reset(const RobotState& s) override { q0_ = s.q; }
  std::array<double, kN> update(const RobotState& s, double t) override {
    std::array<double, kN> tau{};
    // ... 你的计算 ...
    return tau;
  }
 private:
  std::array<double, kN> q0_{};
};
```

## 必须知道的约定

- **`ctrl` 是力矩**，不是位置。`<motor>` 没有位置环，`ctrl=0` 时臂会在重力下
  掉下来。示例用 `bias` 做重力/科氏补偿，再叠加 PD。
- **力矩会被限幅**：每个关节 ±100 N·m（URDF effort）。示例写 `ctrl` 前显式
  限幅了一次，模型内部也会再限一次。
- **`qpos`/`qvel` 是当前状态；`jointN_pos`/`jointN_vel` 传感器滞后一步**。
  `mj_step` 在积分前采样传感器，所以步进后读 `sensordata` 得到的是步初状态。
  示例直接读 `d->qpos`/`d->qvel`，避免反馈里多一拍延迟——实测同样的 PD 用
  传感器反馈会发散。
- **`qfrc_bias` / `qacc` 也比 `qpos` 滞后一步**（它们是上一步积分前算的）。
  在 2 ms 下影响很小；需要严格对齐时，在循环开头调用一次
  `mj_forward(m, d)`。MuJoCo 没有 `jointacc` 传感器，角加速度就是 `d->qacc`。
- **PD 增益不能随便给**。力矩是显式积分，增益要满足 2 ms 步长：示例按
  `kp = I·ωn²`、`kd = 2ζ·I·ωn`（`I` 取质量矩阵对角元）取 `ωn = 10 rad/s`、
  `ζ = 1`。这个模型 `ωn` 超过约 20 rad/s 就会数值发散，最初的
  `kp={60,...}` 就是因此炸掉的。改初始位姿后惯量会变，重算一次即可。
- 夹爪执行器 `act_gripper` 没有在示例里驱动，`ctrl=0` 表示两指保持不动；
  需要夹取时把它当作第 9 个执行器，`ctrl` 是腱张力 [N]（正值夹紧）。

## 实时性与可视化

`main()` 用 `sleep_until` 按模型 `timestep=0.002` 实时步进。如果只想尽快算完，
把第 4 步的 pacing 删掉即可。

这个程序是 headless 的。想边跑边看，可以另开一个终端：

```bash
python -m mujoco.viewer --mjcf=Simulation/mujoco/BigArm/robot_scene.xml
```

不过那是 MuJoCo 自己的物理，不会同步这里的算法。要在 viewer 里看自己的算法，
把控制循环换成 `mujoco.viewer.launch_passive` 的 Python 版，或在 C++ 里用
`simulate` 库（需要 GLFW，当前 SDK 没有）。
