"""右侧状态面板：串口状态、设备状态、时间与频率、收发统计。5 Hz 由主窗口推数据进来。"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFormLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

from ..model import RobotSample, TimingStats
from ..protocol.enums import ArmState, MsgType, StatusFlag
from . import theme


def _dot(color: str) -> str:
    return f"<span style='color:{color}'>●</span>"


class _Form(QGroupBox):
    def __init__(self, title: str, rows: list[tuple[str, str, str]]) -> None:
        super().__init__(title)
        f = QFormLayout(self)
        f.setContentsMargins(8, 6, 8, 6)
        f.setVerticalSpacing(3)
        self.v: dict[str, QLabel] = {}
        for key, label, tip in rows:
            t = QLabel(label)
            t.setObjectName("title")
            lb = QLabel("—")
            lb.setObjectName("value")
            lb.setTextFormat(Qt.RichText)
            lb.setToolTip(tip)
            t.setToolTip(tip)
            f.addRow(t, lb)
            self.v[key] = lb

    def set(self, key: str, text: str) -> None:
        lb = self.v[key]
        if lb.text() != text:
            lb.setText(text)


class StatusPanel(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.conn = _Form("连接", [
            ("source", "数据源", ""),
            ("state", "串口", "connected / reconnecting / closed"),
            ("link", "链路", "300 ms 内是否收到有效帧"),
            ("device", "设备", "DEVICE_INFO 握手结果"),
        ])
        self.dev = _Form("设备状态", [
            ("arm", "Arm 状态", "ArmState_t"),
            ("rc", "遥控", "DT7 在线 / 拨杆"),
            ("flags", "标志", "status_flags"),
            ("cmd_age", "上位机指令", "距上一条有效力矩设定"),
        ])
        self.time = _Form("时间", [
            ("mcu_t", "MCU 采样时刻", "帧内 mcu_time_us，已展开回绕"),
            ("host_t", "上位机接收时刻", "time.monotonic，本机时钟"),
            ("mcu_rate", "MCU 采样频率", "由相邻 MCU 时间戳求得"),
            ("mcu_dt", "最大采样间隔", "MCU 侧丢样或卡顿会变大"),
            ("host_rate", "上位机接收频率", "实际到达本机的帧率"),
            ("host_gap", "最长到达间隔", "USB/串口缓冲导致成批到达时变大"),
            ("jitter", "传输抖动", "(接收时刻 − 采样时刻) 的标准差；两台机器时钟没同步，所以只看抖动不看绝对延迟"),
        ])
        self.stats = _Form("收发统计", [
            ("rx", "有效帧", ""),
            ("lost", "丢帧（按序号）", "SEQ 不连续推算出的丢帧"),
            ("crc", "校验错误", "CRC16 / 帧头 / 长度 / 版本"),
            ("disc", "丢弃字节", "重新同步时跳过的字节"),
            ("tx", "发送 / 丢弃", "断线时待发帧直接丢弃"),
            ("ack", "应答超时", ""),
            ("reconn", "重连次数", ""),
            ("buf", "缓存", "环形缓存占用 / 容量"),
        ])
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        for w in (self.conn, self.dev, self.time, self.stats):
            lay.addWidget(w)
        lay.addStretch(1)

    def update_all(self, st: dict, sample: RobotSample | None, timing: TimingStats,
                   buf_size: int, buf_cap: int) -> None:
        c = self.conn
        c.set("source", f"{st.get('backend', '—')}  {st.get('port', '')}")
        state = st.get("state", "—")
        col = theme.OK if state in ("connected", "replay", "running") else theme.WARN if state == "reconnecting" else theme.ERR
        c.set("state", f"{_dot(col)} {state}")
        alive = st.get("link_alive", False)
        c.set("link", f"{_dot(theme.OK if alive else theme.ERR)} {'正常' if alive else '无数据'}")
        info = st.get("device_info")
        c.set("device", f"{info.fw_name} v{info.proto_version >> 4}.{info.proto_version & 15} "
                        f"{info.joint_count} 关节 {info.state_rate_hz} Hz" if info else "—")

        d = self.dev
        sysst = st.get("system")
        if sample is not None:
            try:
                arm = ArmState(sample.arm_state).name
            except ValueError:
                arm = str(sample.arm_state)
            d.set("arm", f"{_dot(theme.OK if sample.arm_state == ArmState.ENABLE else theme.MUTED)} {arm}")
            f = StatusFlag(sample.status_flags)
            rc_on = StatusFlag.RC_ONLINE in f
            sw = f"  L{sysst.rc_switch_left} R{sysst.rc_switch_right}" if sysst else ""
            d.set("rc", f"{_dot(theme.OK if rc_on else theme.ERR)} {'在线' if rc_on else '离线'}{sw}")
            parts = []
            if StatusFlag.ESTOP_LATCHED in f:
                parts.append(f"<span style='color:{theme.ERR}'>急停锁存</span>")
            if StatusFlag.HOST_LINK_OK in f:
                parts.append("链路OK")
            if StatusFlag.HOST_TORQUE_ACTIVE in f:
                parts.append(f"<span style='color:{theme.WARN}'>上位机力矩生效</span>")
            if StatusFlag.CTRL_ALIGN_PENDING in f:
                parts.append("对齐中")
            d.set("flags", " ".join(parts) or "—")
            age = sample.host_cmd_age_ms
            d.set("cmd_age", "从未" if age >= 0xFFFF else f"{age} ms 前")
            self.time.set("mcu_t", f"{sample.mcu_time_us / 1e6:12.6f} s")
            self.time.set("host_t", f"{sample.host_time_ns / 1e9:12.6f} s")

        t = self.time
        t.set("mcu_rate", f"{timing.mcu_rate_hz:7.1f} Hz")
        t.set("mcu_dt", f"{timing.mcu_dt_max_ms:7.2f} ms")
        t.set("host_rate", f"{timing.host_rate_hz:7.1f} Hz")
        t.set("host_gap", f"{timing.host_gap_max_ms:7.2f} ms")
        t.set("jitter", f"{timing.jitter_ms:7.2f} ms")

        s = self.stats
        p = st.get("parser", {})
        tr = st.get("transport", {})
        rx = st.get("rx_by_type", {}).get(int(MsgType.JOINT_STATE), 0)
        s.set("rx", f"{p.get('frames_ok', 0)}（状态帧 {rx}）" if p else "—")
        s.set("lost", str(st.get("lost_frames", "—")))
        errs = sum(p.get(k, 0) for k in ("crc_errors", "header_errors", "length_errors", "version_errors"))
        s.set("crc", f"{_dot(theme.ERR) if errs else ''} {errs}" if p else "—")
        s.set("disc", f"{p.get('bytes_discarded', 0)} B" if p else "—")
        s.set("tx", f"{tr.get('tx_frames', 0)} / {tr.get('tx_dropped', 0)}" if tr else "—")
        s.set("ack", str(st.get("ack_timeouts", "—")))
        s.set("reconn", str(tr.get("reconnects", "—")) if tr else "—")
        s.set("buf", f"{buf_size} / {buf_cap}")
