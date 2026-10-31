"""Host 会话状态与原生崩溃归档。

Qt/PySide 的原生崩溃不能被 Python ``except`` 捕获，只能由 faulthandler 把
堆栈追加到 ``logs/crash.log``。这里记录每次会话是否正常结束；下次启动时，
把上一次会话新增的 crash.log 内容写进 armhost.log 和 crash_history.log。
"""

from __future__ import annotations

import atexit
import json
import logging
import os
import sys
from datetime import datetime
from importlib import metadata
from pathlib import Path

_state_path: Path | None = None
_crash_path: Path | None = None
_dirty_reason: str | None = None


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _read_state(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError, TypeError):
        return {}


def _write_state(path: Path, state: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    try:
        tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        tmp.replace(path)
    except OSError:
        pass


def _read_from(path: Path, offset: int) -> str:
    try:
        with path.open("rb") as f:
            f.seek(max(0, offset))
            return f.read().decode("utf-8", errors="replace")
    except OSError:
        return ""


def versions() -> dict[str, str]:
    versions = {"python": sys.version.split()[0]}
    for package in ("PySide6", "pyqtgraph", "numpy", "pyserial"):
        try:
            versions[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            versions[package] = "?"
    return versions


def note_abnormal(reason: str) -> None:
    """记录 Python 未捕获异常，正常退出时不要覆盖成 clean。"""
    global _dirty_reason
    _dirty_reason = reason


def _append_history(log_dir: Path, text: str, state: dict) -> None:
    header = (f"\n===== native crash archived {_now()} "
              f"pid={state.get('pid', '?')} started={state.get('started_at', '?')} =====\n")
    try:
        with (log_dir / "crash_history.log").open("a", encoding="utf-8") as f:
            f.write(header)
            f.write(text)
            if not text.endswith("\n"):
                f.write("\n")
    except OSError:
        pass


def begin_session(log_dir: Path, logger: logging.Logger) -> None:
    """开始会话；若上次会话异常结束，先把新增堆栈写入日志。"""
    global _state_path, _crash_path, _dirty_reason
    log_dir.mkdir(parents=True, exist_ok=True)
    _state_path = log_dir / "session.json"
    _crash_path = log_dir / "crash.log"
    _dirty_reason = None

    previous = _read_state(_state_path)
    crash_offset = _crash_path.stat().st_size if _crash_path.exists() else 0
    if previous and not previous.get("clean", False):
        try:
            old_offset = max(0, int(previous.get("crash_offset", 0) or 0))
        except (TypeError, ValueError):
            old_offset = 0
        new_text = _read_from(_crash_path, old_offset)
        if new_text.strip():
            logger.error("上次会话异常退出（pid=%s, 启动于 %s），原生崩溃记录：\n%s",
                         previous.get("pid", "?"), previous.get("started_at", "?"), new_text.rstrip())
            _append_history(log_dir, new_text, previous)
        else:
            logger.warning("上次会话未正常结束（pid=%s, 启动于 %s），crash.log 没有新增堆栈；"
                           "可能是进程被结束、断电或系统异常",
                           previous.get("pid", "?"), previous.get("started_at", "?"))

    state = {
        "pid": os.getpid(),
        "started_at": _now(),
        "clean": False,
        "crash_offset": crash_offset,
        "versions": versions(),
        "argv": sys.argv[:],
    }
    _write_state(_state_path, state)
    atexit.register(mark_clean_exit)


def mark_clean_exit() -> None:
    """正常退出时更新会话状态；异常退出要求进程正常执行 atexit 才会走到这里。"""
    if _state_path is None:
        return
    state = _read_state(_state_path)
    if not state or state.get("pid") != os.getpid():
        return
    state["clean"] = _dirty_reason is None
    state["ended_at"] = _now()
    if _dirty_reason is not None:
        state["dirty_reason"] = _dirty_reason
    _write_state(_state_path, state)
