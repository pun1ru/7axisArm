"""启动上位机。

    python -m armhost.app                 # 打开后选择数据源再点"连接"
    python -m armhost.app --mock          # 直接连模拟设备
    python -m armhost.app --sim           # 直接连仿真（MuJoCo / 内置模型）
    python -m armhost.app --port COM5     # 直接连真实设备
    python -m armhost.app --replay logs/arm_xxx.csv
配置：config/settings.toml；日志：logs/armhost.log
"""

from __future__ import annotations

import argparse
import faulthandler
import logging
import logging.handlers
import os
import sys
import threading
import traceback
from pathlib import Path

from .crashlog import begin_session, note_abnormal, versions

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"
log = logging.getLogger("armhost")
_crash_file = None


def setup_logging(level: str = "INFO") -> None:
    global _crash_file
    LOG_DIR.mkdir(exist_ok=True)
    root = logging.getLogger()
    root.setLevel(level)
    fmt = logging.Formatter("%(asctime)s %(levelname)-7s %(name)s %(threadName)s: %(message)s")
    fh = logging.handlers.RotatingFileHandler(LOG_DIR / "armhost.log", maxBytes=5 << 20, backupCount=5,
                                              encoding="utf-8")
    fh.setFormatter(fmt)
    root.addHandler(fh)
    if sys.stderr is not None:
        sh = logging.StreamHandler()
        sh.setFormatter(fmt)
        root.addHandler(sh)
    try:
        _crash_file = open(LOG_DIR / "crash.log", "a", encoding="utf-8")
        faulthandler.enable(file=_crash_file, all_threads=True)
    except OSError:
        _crash_file = None


def install_excepthook() -> None:
    """未捕获异常：写日志 + 弹窗提示，不让 Qt 静默吞掉，也不让后台线程无声死掉"""
    def show(text: str) -> None:
        from PySide6.QtCore import QTimer
        from PySide6.QtWidgets import QApplication, QMessageBox
        if QApplication.instance() is None:
            return
        QTimer.singleShot(0, lambda: QMessageBox.critical(
            None, "程序错误", f"{text[-1500:]}\n\n完整信息见 {LOG_DIR / 'armhost.log'}"))

    def hook(et, ev, tb):
        if issubclass(et, KeyboardInterrupt):
            sys.__excepthook__(et, ev, tb)
            return
        text = "".join(traceback.format_exception(et, ev, tb))
        note_abnormal(f"{et.__name__}: {ev}")
        log.critical("未捕获异常\n%s", text)
        show(text)

    def thread_hook(args):
        hook(args.exc_type, args.exc_value, args.exc_traceback)

    sys.excepthook = hook
    threading.excepthook = thread_hook


def main(argv: list[str] | None = None) -> int:
    from . import config
    ap = argparse.ArgumentParser(description="七轴机械臂上位机")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--mock", action="store_true", help="启动后连接模拟设备")
    g.add_argument("--sim", action="store_true", help="启动后连接仿真")
    g.add_argument("--port", help="启动后连接串口")
    g.add_argument("--replay", help="启动后回放 CSV")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--rate", type=int, default=None, help="模拟设备上报频率 Hz")
    ap.add_argument("--log", default=None)
    ap.add_argument("--check", action="store_true", help="只检查依赖与界面能否构造，然后退出（给启动脚本自检用）")
    a = ap.parse_args(argv)

    try:
        cfg = config.load()
        cfg_err = None
    except config.ConfigError as e:
        cfg, cfg_err = config.DEFAULTS, str(e)
    setup_logging(a.log or cfg["log"]["level"])
    install_excepthook()
    begin_session(LOG_DIR, log)
    v = versions()
    log.info("armhost starting pid=%d python=%s pyside=%s pyqtgraph=%s numpy=%s",
             os.getpid(), v.get("python"), v.get("PySide6"), v.get("pyqtgraph"), v.get("numpy"))

    from PySide6.QtWidgets import QApplication, QMessageBox
    app = QApplication(sys.argv[:1])
    app.setApplicationName("armhost")
    from .ui import theme
    theme.apply(app)
    if cfg_err:
        log.error(cfg_err)
        QMessageBox.warning(None, "配置文件", f"{cfg_err}\n\n本次使用默认配置启动。")

    from .controller import MonitorController
    from .model.backend import MockBackend, SerialBackend
    from .model.sim import SimBackend
    from .ui.main_window import MonitorWindow
    ctrl = MonitorController(buffer_capacity=int(cfg["ui"]["buffer_seconds"] * 500))
    w = MonitorWindow(ctrl, config=cfg)
    if a.check:
        w.close()
        print("armhost check OK")
        return 0
    w.show()
    baud = a.baud or cfg["serial"]["baudrate"]
    port = a.port or (cfg["serial"]["port"] if not (a.mock or a.sim or a.replay) else "")
    if a.mock:
        w._switch(MockBackend(rate_hz=a.rate or cfg["mock"]["rate_hz"]))
    elif a.sim:
        s = cfg["sim"]
        w._switch(SimBackend(s["rate_hz"], s["realtime"], s["prefer_mujoco"]))
    elif port:
        w._switch(SerialBackend.open_port(port, baud))
    elif a.replay:
        w._start_replay(a.replay, 1.0, False)
    try:
        return app.exec()
    finally:
        log.info("armhost event loop stopped")
        logging.shutdown()


if __name__ == "__main__":
    sys.exit(main())
