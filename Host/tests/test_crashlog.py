"""Host crash.log 归档与会话状态测试。"""

import json
import logging
import tempfile
import unittest
from pathlib import Path

from armhost import crashlog


class TestCrashLog(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.log_dir = Path(self.tmp.name)
        self.logger = logging.getLogger(f"test.crashlog.{id(self)}")
        self.logger.propagate = False

    def tearDown(self):
        crashlog.mark_clean_exit()
        self.tmp.cleanup()

    def _previous(self, offset, clean=False):
        (self.log_dir / "session.json").write_text(json.dumps({
            "pid": 1234,
            "started_at": "2026-10-31T12:00:00+08:00",
            "clean": clean,
            "crash_offset": offset,
        }), encoding="utf-8")

    def test_archives_new_native_stack(self):
        old = "old crash\n"
        crash = self.log_dir / "crash.log"
        crash.write_text(old, encoding="utf-8")
        self._previous(len(old.encode()))
        crash.write_text(old + "Windows fatal exception: access violation\n", encoding="utf-8")

        with self.assertLogs(self.logger, level="ERROR") as cm:
            crashlog.begin_session(self.log_dir, self.logger)

        self.assertIn("上次会话异常退出", "\n".join(cm.output))
        self.assertIn("access violation", (self.log_dir / "crash_history.log").read_text(encoding="utf-8"))
        crashlog.mark_clean_exit()
        state = json.loads((self.log_dir / "session.json").read_text(encoding="utf-8"))
        self.assertTrue(state["clean"])

    def test_clean_exit_is_not_reported(self):
        old = "old crash\n"
        crash = self.log_dir / "crash.log"
        crash.write_text(old, encoding="utf-8")
        self._previous(len(old.encode()), clean=True)
        crash.write_text(old + "new data\n", encoding="utf-8")

        crashlog.begin_session(self.log_dir, self.logger)

        self.assertFalse((self.log_dir / "crash_history.log").exists())


if __name__ == "__main__":
    unittest.main()
