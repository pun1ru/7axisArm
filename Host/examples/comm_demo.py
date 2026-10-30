"""通信模块运行示例。

    python examples/comm_demo.py --list               # 枚举串口
    python examples/comm_demo.py                      # 模拟设备
    python examples/comm_demo.py --noise              # 模拟设备 + 噪声/丢字节/拆包
    python examples/comm_demo.py --port COM5          # 真实设备（Linux: /dev/ttyACM0）
    python examples/comm_demo.py --port COM5 --hex    # 同时打印原始十六进制
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from armhost.comm import (ArmLink, FaultConfig, SerialConfig, SerialTransport, SimDevice,  # noqa: E402
                          list_serial_ports, make_mock_transport)
from armhost.protocol import MsgType  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--noise", action="store_true")
    ap.add_argument("--hex", action="store_true")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--torque", type=float, default=0.0, help="演示：给 J0 设力矩 N*m（默认 0 不发）")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

    if a.list:
        for p in list_serial_ports():
            print(f"{p.device:12s} {p.description}  [{p.hwid}]")
        return 0

    dev = None
    if a.port:
        tr = SerialTransport(SerialConfig(port=a.port, baudrate=a.baud))
    else:
        faults = FaultConfig(noise_prob=0.05, corrupt_prob=0.02, drop_prob=0.02, chunk_max=31, seed=0) \
            if a.noise else FaultConfig()
        tr, dev, _ = make_mock_transport(SimDevice(faults=faults))
        dev.start()

    link = ArmLink(tr)
    link.hexlog.enabled = a.hex
    link.subscribe(MsgType.EVENT, lambda r: print("EVENT", r.msg))
    link.start()
    try:
        if not tr.wait_connected(3.0):
            print("串口打不开", file=sys.stderr)
            return 1
        try:
            print("DEVICE_INFO", link.request_device_info().result(1.0))
        except Exception as e:
            print("DEVICE_INFO 无应答:", e)

        end = time.monotonic() + a.seconds
        while time.monotonic() < end:
            if a.torque:
                link.set_joint_torque({0: a.torque}, hold_ms=100, want_ack=False)
            time.sleep(0.5)
            print("-" * 100)
            print(link.mirror.format_c())
            st = link.snapshot_stats()
            hz = st["rate_hz"].get(int(MsgType.JOINT_STATE), 0.0)
            p = st["parser"]
            print(f"state={st['state']} alive={st['link_alive']} JOINT_STATE {hz:.0f} Hz | "
                  f"ok={p['frames_ok']} crc_err={p['crc_errors']} hdr_err={p['header_errors']} "
                  f"lost={st['lost_frames']} discarded={p['bytes_discarded']}B")
            if a.hex:
                for rec in link.hexlog.snapshot(last=3):
                    print(rec.format()[:160])
    finally:
        link.stop()
        if dev:
            dev.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
