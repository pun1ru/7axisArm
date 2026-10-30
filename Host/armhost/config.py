"""读取 config/settings.toml（标准库 tomllib，只读）。缺文件或缺项都回落到默认值，坏文件给出明确错误。"""

from __future__ import annotations

import tomllib
from copy import deepcopy
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
SETTINGS_FILE = CONFIG_DIR / "settings.toml"

DEFAULTS: dict = {
    "serial": {"port": "", "baudrate": 921600},
    "ui": {"buffer_seconds": 60, "scope_fps": 30, "view3d_fps": 30, "default_window_s": 10},
    "mock": {"rate_hz": 200},
    "sim": {"rate_hz": 200, "prefer_mujoco": True, "realtime": 1.0},
    "log": {"level": "INFO"},
}


class ConfigError(ValueError):
    pass


def load(path: Path = SETTINGS_FILE) -> dict:
    cfg = deepcopy(DEFAULTS)
    if not path.exists():
        return cfg
    try:
        user = tomllib.loads(path.read_text(encoding="utf-8"))
    except tomllib.TOMLDecodeError as e:
        raise ConfigError(f"{path.name} 格式错误：{e}") from None
    for sec, vals in user.items():
        if sec not in cfg or not isinstance(vals, dict):
            raise ConfigError(f"{path.name}: 未知配置段 [{sec}]")
        for k, v in vals.items():
            if k not in cfg[sec]:
                raise ConfigError(f"{path.name}: [{sec}] 没有 {k} 这一项")
            want = type(cfg[sec][k])
            if want is float and isinstance(v, int):
                v = float(v)
            if not isinstance(v, want):
                raise ConfigError(f"{path.name}: [{sec}].{k} 应为 {want.__name__}，实际 {v!r}")
            cfg[sec][k] = v
    return cfg
