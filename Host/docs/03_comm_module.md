# 03 通信模块

> 对应 README 第 3 项。不依赖 PySide6，可以单独使用和测试。

## 文件结构

```
Host/
├─ armhost/
│  ├─ protocol/            协议层（纯 Python，没有 IO）
│  │  ├─ crc.py            CRC-8/SMBUS、CRC-16/CCITT-FALSE
│  │  ├─ frame.py          帧格式、encode_frame
│  │  ├─ parser.py         FrameParser：粘包/半包/错误帧/重同步/半帧超时
│  │  ├─ enums.py          消息类型、ArmState、MotorStatus 等（数值与固件一致）
│  │  ├─ msg_link.py       HEARTBEAT / DEVICE_INFO
│  │  ├─ msg_report.py     ACK / JOINT_STATE / SYSTEM_STATUS / EVENT
│  │  ├─ msg_cmd.py        SET_JOINT_TORQUE / CLEAR_TORQUE / ESTOP / PARAM_*
│  │  ├─ params.py         参数表
│  │  └─ messages.py       注册表、decode_frame
│  └─ comm/
│     ├─ transport.py      SerialTransport：端口枚举、连接/断开、配置、读写线程、自动重连
│     ├─ link.py           ArmLink：序号、发送、应答匹配、超时重试、心跳、订阅
│     ├─ mirror.py         与固件同名的镜像结构体 Arm_t / Feedback_t …
│     ├─ stats.py          SeqTracker、McuClock、LinkStats
│     ├─ hexlog.py         原始十六进制环形记录
│     └─ mock.py           SimDevice + MockSerial：模拟固件，可注入噪声/丢字节/拔线
├─ firmware_ref/           STM32 端参考实现（不在 Real 里）
├─ tests/                  46 个单元测试
└─ examples/comm_demo.py   运行示例
```

## 线程

| 线程 | 内容 |
|---|---|
| `serial-rx` | 阻塞 read（20 ms 超时）→ FrameParser → 解码 → 更新 mirror → 回调订阅者 |
| `serial-tx` | 从有界队列取帧写串口；断线时队列清空，**不补发** |
| `link-timer` | 10 ms tick：心跳、应答超时与重试、链路存活判断 |
| 调用方（GUI 主线程） | 只调用非阻塞方法，拿到 `Future`；读 `mirror.snapshot()` / `snapshot_stats()` |

订阅回调在 `serial-rx` 线程中执行，GUI 层要转成 Qt 信号或写进缓存，不能直接操作控件。

## 用法

```python
from armhost.comm import ArmLink, SerialConfig, SerialTransport, list_serial_ports
from armhost.protocol import MsgType

print(list_serial_ports())
link = ArmLink(SerialTransport(SerialConfig(port="COM5", baudrate=921600)))
link.subscribe(MsgType.JOINT_STATE, lambda r: buffer.append((r.host_time_ns, r.msg)))
link.start()
info = link.request_device_info().result(1.0)
print(link.read_param(0x0300, 0).result(1.0).value)           # J0 上位机力矩限幅 N·m（只读，固件编译期常量）
ack = link.set_joint_torque({0: 0.3}, hold_ms=100).result(0.5)
print(link.mirror.format_c())                              # 按固件变量名打印
print(link.snapshot_stats())                                # 收发、丢帧、CRC 错误、频率
link.stop()                                                 # 发急停 + 清零后关闭
```

没有硬件时把 transport 换成 `make_mock_transport()` 返回的模拟设备即可。

## 运行

```bash
pip install pyserial            # 测试与模拟设备只依赖标准库，真实串口需要 pyserial
python -m unittest discover -s tests -v          # 或 pytest tests
python examples/comm_demo.py --list
python examples/comm_demo.py                     # 模拟设备
python examples/comm_demo.py --noise --hex       # 注入噪声并查看原始十六进制
python examples/comm_demo.py --port COM5         # 真实设备（Linux 用 /dev/ttyUSB0，需要在 dialout 组里）
```

## 测试覆盖

| 文件 | 内容 |
|---|---|
| `test_protocol.py` | CRC 校验值；帧字段偏移；逐字节输入、多帧粘包、垃圾字节、HCRC/CRC/长度/版本错误、好帧藏在坏帧里、半帧超时；所有消息往返编解码；向前兼容（加长记录） |
| `test_comm.py` | 序号回绕/丢帧/重启；时间戳展开；握手；200/500 Hz 数据流；力矩 ACK 与限幅；有效期到期；急停拒绝；参数读写/只读/越界；应答超时重试；**力矩命令不重试**；设备侧链路超时清零；拔线重连；噪声环境下持续工作 |
| `test_c_header_sync.py` | `firmware_ref/arm_proto.h` 的常量、枚举、结构体尺寸与 Python 定义一致 |
