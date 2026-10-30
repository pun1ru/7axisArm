"""仿真数据源（README #5/#6 的"MuJoCo 适配器"）。

两种实现，同一个接口 SimBackend（RobotBackend 子类）：
  * MujocoEngine：装了 `mujoco` 包时加载 Simulation/mujoco/robot.xml，真实动力学；
  * KinematicEngine：没装时的纯 Python 替身，二阶关节模型 + 简单重力项，保证没有 MuJoCo 也能演示全部界面。

约束（README #6 第 2 条）：UI 不直接改 MuJoCo 内部状态。外部只能通过
  set_targets(q_deg)、set_joint_torque({j: τ}, hold_ms)、set_mode("hold" | "free")、estop()
这几个方法下指令，由仿真线程在步进之间统一写入 mjData。

关节映射：robot.xml 的 joint1..joint7 按顺序对应 J0..J6，q_mj = q_dh（rad↔deg 在这里换算）。
robot.xml 各 body 的 z 轴与 DH z 轴同向（dh_params.md 第 3 节轴向表），零位一致；如以后模型改了，
改 JOINT_SIGN / JOINT_OFFSET_DEG 即可。
"""

from __future__ import annotations

import logging
import math
import threading
import time
from concurrent.futures import Future
from pathlib import Path

import numpy as np

from ..protocol import Ack, AckResult
from ..protocol.enums import ArmState, JointFlag, MotorStatus, StatusFlag
from ..protocol.msg_report import JointRecord
from .backend import RobotBackend
from .state import RobotSample

log = logging.getLogger("armhost.sim")

REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL_XML = REPO_ROOT / "Simulation" / "mujoco" / "robot.xml"
JOINT_NAMES = tuple(f"joint{i + 1}" for i in range(7))
JOINT_SIGN = np.ones(7)
JOINT_OFFSET_DEG = np.zeros(7)

try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except ImportError:
    HAVE_MUJOCO = False


# ====================================================================== 引擎
class KinematicEngine:
    """没有 MuJoCo 时的替身：每个关节 J·q̈ = τ − b·q̇ − g·sin(q)，半隐式欧拉。"""
    name = "内置简化模型"

    def __init__(self, n: int = 7, dt: float = 0.001) -> None:
        self.n, self.dt = n, dt
        self.q = np.zeros(n)
        self.dq = np.zeros(n)
        self.inertia = np.full(n, 0.01)
        self.damping = np.full(n, 0.05)
        self.gravity = np.array([0.0, 0.25, 0.0, 0.12, 0.0, 0.04, 0.0])[:n]     # 只有水平轴受重力
        self.tau_applied = np.zeros(n)
        self.time = 0.0

    def gravity_torque(self) -> np.ndarray:
        return self.gravity * np.sin(self.q)

    def step(self, tau: np.ndarray) -> None:
        self.tau_applied = tau
        ddq = (tau - self.damping * self.dq - self.gravity_torque()) / self.inertia
        self.dq += ddq * self.dt
        self.q += self.dq * self.dt
        self._steps = getattr(self, "_steps", 0) + 1
        self.time = self._steps * self.dt          # 按步数算，不累加浮点误差

    def reset(self) -> None:
        self.q[:] = 0
        self.dq[:] = 0
        self.time = 0.0
        self._steps = 0


class MujocoEngine:
    """MuJoCo 适配：只在仿真线程里碰 mjModel / mjData。"""
    name = "MuJoCo"

    def __init__(self, xml: Path = MODEL_XML) -> None:
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.qadr = np.array([self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in JOINT_NAMES])
        self.vadr = np.array([self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                              for n in JOINT_NAMES])
        self.n = len(JOINT_NAMES)
        self.tau_applied = np.zeros(self.n)
        mujoco.mj_forward(self.model, self.data)

    @property
    def q(self) -> np.ndarray:
        return self.data.qpos[self.qadr]

    @property
    def dq(self) -> np.ndarray:
        return self.data.qvel[self.vadr]

    @property
    def time(self) -> float:
        return float(self.data.time)

    def gravity_torque(self) -> np.ndarray:
        return self.data.qfrc_bias[self.vadr].copy()          # 重力 + 科氏，保持姿态要抵消它

    def step(self, tau: np.ndarray) -> None:
        self.tau_applied = tau
        self.data.qfrc_applied[:] = 0
        self.data.qfrc_applied[self.vadr] = tau
        self.mj.mj_step(self.model, self.data)

    def reset(self) -> None:
        self.mj.mj_resetData(self.model, self.data)
        self.mj.mj_forward(self.model, self.data)


def make_engine(prefer_mujoco: bool = True):
    if prefer_mujoco and HAVE_MUJOCO and MODEL_XML.exists():
        try:
            return MujocoEngine()
        except Exception as e:                    # 模型加载失败不影响演示
            log.warning("MuJoCo 加载失败，改用内置简化模型: %s", e)
    return KinematicEngine()


# ====================================================================== 后端
class SimBackend(RobotBackend):
    """仿真数据源。产出的 RobotSample 与真机同构，界面不用区分。

    控制律（与固件语义对齐）：
      hold 模式：τ = Kp·(q_target − q) − Kd·q̇ + 重力补偿 + 上位机力矩
      free 模式：τ = 上位机力矩（只受重力、阻尼）
      急停：上位机力矩清零并拒绝；hold 模式仍保持，避免仿真臂掉下来误导观察
    """
    name = "sim"
    is_hardware = False
    can_estop = True

    def __init__(self, rate_hz: int = 200, realtime: float = 1.0, prefer_mujoco: bool = True,
                 engine=None) -> None:
        super().__init__()
        self.engine = engine or make_engine(prefer_mujoco)
        self.n = self.engine.n
        # 采样周期必须是仿真步长的整数倍；不整除时取最接近的，并按实际频率报告
        self.steps_per_sample = max(1, int(1.0 / (rate_hz * self.engine.dt) + 0.5))
        self.rate_hz = 1.0 / (self.steps_per_sample * self.engine.dt)
        self.realtime = realtime
        self.mode = "hold"
        self.kp = np.full(self.n, 8.0)
        self.kd = np.full(self.n, 0.3)
        self.torque_limit = np.full(self.n, 2.0)
        self._target = np.zeros(self.n)
        self._host_tau = np.zeros(self.n)
        self._host_deadline = np.zeros(self.n)
        self._estop = False
        self._cmd_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._seq = 0
        self.steps = 0
        self.label = f"{self.engine.name} {self.rate_hz:.0f} Hz"

    # ------------------------------------------------------------ 指令（任意线程调用）
    def set_mode(self, mode: str) -> None:
        if mode not in ("hold", "free"):
            raise ValueError(mode)
        with self._cmd_lock:
            self.mode = mode

    def set_targets(self, q_deg) -> None:
        with self._cmd_lock:
            self._target[:] = np.radians(np.asarray(q_deg, float)[: self.n]) * JOINT_SIGN[: self.n]

    def set_joint_torque(self, torques: dict[int, float], hold_ms: int = 100) -> Future:
        f: Future = Future()
        with self._cmd_lock:
            if self._estop:
                f.set_result(Ack(0, 0x20, AckResult.REJECT_ESTOP, 0))
                return f
            now = time.monotonic()
            mask = 0
            for j, t in torques.items():
                lim = self.torque_limit[j]
                c = max(-lim, min(lim, t))
                mask |= (c != t) << j
                self._host_tau[j] = c
                self._host_deadline[j] = now + min(hold_ms or 100, 500) / 1000
        f.set_result(Ack(0, 0x20, AckResult.OK_CLAMPED if mask else AckResult.OK, mask))
        return f

    def estop(self, latch: bool = True) -> Future:
        with self._cmd_lock:
            self._estop = latch
            if latch:
                self._host_tau[:] = 0
                self._host_deadline[:] = 0
        f: Future = Future()
        f.set_result(Ack(0, 0x22, AckResult.OK, 0))
        self._emit_event("info", f"仿真{'急停锁存' if latch else '解除急停'}")
        return f

    # ------------------------------------------------------------ 生命周期
    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim", daemon=True)
        self._thread.start()
        self._emit_event("info", f"仿真启动：{self.label}，模型 {MODEL_XML.name if self.engine.name == 'MuJoCo' else '无'}")

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(1.0)
        self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------ 仿真线程
    def _control(self) -> np.ndarray:
        e = self.engine
        now = time.monotonic()
        with self._cmd_lock:
            expired = self._host_deadline < now
            self._host_tau[expired] = 0
            host = self._host_tau.copy()
            target = self._target.copy()
            mode = self.mode
        tau = host.copy()
        if mode == "hold":
            tau += self.kp * (target - e.q) - self.kd * e.dq + e.gravity_torque()
        return tau

    def _run(self) -> None:
        e = self.engine
        steps_per_sample = self.steps_per_sample
        wall0, sim0 = time.monotonic(), e.time
        try:
            while not self._stop.is_set():
                for _ in range(steps_per_sample):
                    e.step(self._control())
                    self.steps += 1
                if not np.all(np.isfinite(e.q)):
                    raise FloatingPointError("仿真发散（q 出现 NaN/Inf）")
                self._emit_sample(self._sample())
                ahead = (e.time - sim0) / max(self.realtime, 1e-3) - (time.monotonic() - wall0)
                if ahead > 0:
                    self._stop.wait(ahead)
                elif ahead < -0.5:                   # 跟不上就放弃追赶，避免越积越多
                    wall0, sim0 = time.monotonic(), e.time
        except Exception as ex:
            log.exception("仿真线程异常退出")
            self._emit_event("sim_error", str(ex))
        finally:
            self._emit_event("sim_stopped", self.engine.name)

    def _sample(self) -> RobotSample:
        e = self.engine
        q = np.degrees(e.q * JOINT_SIGN[: self.n]) + JOINT_OFFSET_DEG[: self.n]
        dq = np.degrees(e.dq * JOINT_SIGN[: self.n])
        tau = e.tau_applied * JOINT_SIGN[: self.n]
        with self._cmd_lock:
            host_valid = self._host_deadline > time.monotonic()
            tgt = np.degrees(self._target)
            estop = self._estop
        flags = StatusFlag.RC_ONLINE | StatusFlag.HOST_LINK_OK
        if estop:
            flags |= StatusFlag.ESTOP_LATCHED
        if host_valid.any():
            flags |= StatusFlag.HOST_TORQUE_ACTIVE
        joints = tuple(JointRecord(float(q[i]), float(dq[i]), float(tau[i]), float(tgt[i]), float(tau[i]), 0,
                                   MotorStatus.ENABLED, 0, 25,
                                   JointFlag.ONLINE | (JointFlag.HOST_TORQUE_VALID if host_valid[i] else 0),
                                   self._seq & 0xFFFF)
                       for i in range(self.n))
        self._seq += 1
        return RobotSample(int(round(e.time * 1e6)), time.monotonic_ns(), self._seq & 0xFFFF, ArmState.ENABLE,
                           flags, 0xFFFF, joints)

    def status(self) -> dict:
        return {"backend": self.name, "port": self.label, "state": "running" if self.running else "stopped",
                "link_alive": self.running, "engine": self.engine.name, "mode": self.mode,
                "sim_time_s": self.engine.time, "steps": self.steps}
