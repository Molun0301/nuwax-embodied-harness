#!/usr/bin/env python3
"""更新调度读取的离线契约检查；门控只证明并发顺序，不测量磁盘性能。"""

import asyncio
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import httpx
from fastapi import FastAPI
from gateways import updates

REAL_SLEEP = asyncio.sleep

if len(sys.argv) > 2 and sys.argv[1] == "--source":
    source = Path(sys.argv[2])
    sys.argv[1:3] = []
    candidate = ModuleType("update_schedule_candidate")
    exec(compile(source.read_text(), str(source), "exec"), candidate.__dict__)
    updates = candidate


class StopLoop(BaseException):
    pass


class UpdateScheduleTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory()))
        with patch.object(updates.updater, "git", return_value="a" * 40):
            self.service = updates.UpdateService(SimpleNamespace(), self.root)
        self.directory = self.root / "runtime_data/updates"
        self.directory.mkdir(parents=True)
        (self.directory / "preferences.json").write_text('{"auto_check": false}')
        (self.directory / "check.json").write_text('{"checked_at": 7}')
        self.service.check = Mock(return_value={"available": True, "latest_revision": "b" * 40})
        self.service.install = Mock()

    async def stop_sleep(self, delay):
        if delay == 0:
            return await REAL_SLEEP(0)
        self.assertEqual(delay, 30)
        raise StopLoop()

    async def one_cycle(self):
        with patch.object(updates.asyncio, "sleep", self.stop_sleep):
            with self.assertRaises(StopLoop):
                await self.service.run()

    async def test_both_reads_preserve_order_on_one_worker(self):
        original = updates.updater.read_json
        calls = []

        def read(path, *args):
            calls.append((Path(path).name, threading.get_ident()))
            return original(path, *args)

        with patch.object(updates.updater, "read_json", read):
            await self.one_cycle()
        self.assertEqual([name for name, _ in calls], ["preferences.json", "check.json"])
        self.assertEqual(calls[0][1], calls[1][1])
        self.assertNotEqual(calls[0][1], threading.get_ident())
        self.service.check.assert_not_called()

    async def check_responsive_read(self, filename):
        loop = asyncio.get_running_loop()
        entered, finished = asyncio.Event(), asyncio.Event()
        release, watchdog_stop = threading.Event(), threading.Event()
        original = updates.updater.read_json
        observations = []

        def read(path, *args):
            if Path(path).name == filename:
                loop.call_soon_threadsafe(entered.set)
                try:
                    if not release.wait(5):
                        raise RuntimeError("测试读取未释放")
                finally:
                    loop.call_soon_threadsafe(finished.set)
            return original(path, *args)

        def watchdog():
            if not watchdog_stop.wait(2):
                release.set()

        app = FastAPI()

        @app.get("/peer")
        async def peer():
            observations.append(not release.is_set())
            return {"ok": True}

        async def request():
            await entered.wait()
            async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test") as client:
                response = await client.get("/peer")
                self.assertEqual(response.status_code, 200)
            release.set()

        guard = threading.Thread(target=watchdog)
        guard.start()
        try:
            with patch.object(updates.updater, "read_json", read):
                await asyncio.wait_for(asyncio.gather(self.one_cycle(), request()), 6)
                await asyncio.wait_for(finished.wait(), 2)
        finally:
            release.set()
            watchdog_stop.set()
            guard.join(3)
        self.assertFalse(guard.is_alive())
        self.assertEqual(observations, [True], "同一事件循环的HTTP请求必须在门控读取释放前完成")

    async def test_preferences_read_does_not_block_peer_http(self):
        await self.check_responsive_read("preferences.json")

    async def test_check_state_read_does_not_block_peer_http(self):
        await self.check_responsive_read("check.json")

    async def test_close_cancels_schedule_without_following_actions(self):
        loop = asyncio.get_running_loop()
        entered, finished = asyncio.Event(), asyncio.Event()
        release = threading.Event()
        original = updates.updater.read_json
        (self.directory / "preferences.json").write_text('{"auto_check": true, "auto_install": true}')

        def read(path, *args):
            if Path(path).name == "check.json":
                loop.call_soon_threadsafe(entered.set)
                try:
                    if not release.wait(5):
                        raise RuntimeError("测试读取未释放")
                    return original(path, *args)
                finally:
                    loop.call_soon_threadsafe(finished.set)
            return original(path, *args)

        with patch.object(updates.updater, "read_json", read):
            self.service.start()
            try:
                await asyncio.wait_for(entered.wait(), 2)
                await asyncio.wait_for(self.service.close(), 2)
                self.assertTrue(self.service.task.cancelled())
                self.assertFalse(finished.is_set(), "取消调度不等于物理读取已经结束")
            finally:
                release.set()
                await asyncio.wait_for(finished.wait(), 2)
                await self.service.close()
        self.service.check.assert_not_called()
        self.service.install.assert_not_called()

    async def run_cycles(self, preferences, last, count=1, check_result=None):
        sleeps = []
        if check_result is not None:
            self.service.check.return_value = check_result

        async def sleep(delay):
            if delay == 0:
                return await REAL_SLEEP(0)
            sleeps.append(delay)
            if len(sleeps) == count:
                raise StopLoop()

        with patch.object(self.service, "preferences", side_effect=preferences), patch.object(
            updates.updater, "read_json", **({"side_effect": last} if isinstance(last, list) else {"return_value": last})
        ), patch.object(updates.time, "time", return_value=10000), patch.object(
            updates.asyncio, "sleep", sleep
        ):
            with self.assertRaises(StopLoop):
                await self.service.run()
        self.assertEqual(sleeps, [30] * count)

    async def test_disabled_check_still_uses_normal_sleep(self):
        await self.run_cycles([{"auto_check": False}], {})
        self.service.check.assert_not_called()
        self.service.install.assert_not_called()

    async def test_recent_check_does_not_run(self):
        await self.run_cycles([{"auto_check": True, "interval_minutes": 15}], {"checked_at": 9900})
        self.service.check.assert_not_called()

    async def test_due_check_keeps_next_attempt_backoff(self):
        preferences = {"auto_check": True, "interval_minutes": 15}
        await self.run_cycles([preferences, preferences], {}, count=2)
        self.service.check.assert_called_once_with()
        self.service.install.assert_not_called()

    async def test_preferences_are_reread_next_cycle(self):
        await self.run_cycles([{"auto_check": False}, {"auto_check": True}], {}, count=2)
        self.service.check.assert_called_once_with()

    async def test_auto_install_only_when_available_target_differs(self):
        for available, transaction, expected in ((True, {}, 1), (False, {}, 0),
                                                  (True, {"target_revision": "b" * 40}, 0)):
            with self.subTest(available=available, transaction=transaction):
                self.service.check.reset_mock()
                self.service.install.reset_mock()
                await self.run_cycles([{"auto_check": True, "auto_install": True}], {}, check_result={
                    "available": available, "latest_revision": "b" * 40, "transaction": transaction,
                })
                self.assertEqual(self.service.install.call_count, expected)
                if expected:
                    self.service.install.assert_called_once_with("b" * 40)

    async def test_read_error_logs_and_retries_after_normal_sleep(self):
        with patch.object(updates.LOG, "warning") as warning:
            await self.run_cycles([OSError("读取失败"), {"auto_check": True}], {}, count=2)
        warning.assert_called_once()
        self.assertEqual(warning.call_args.kwargs["extra"]["event"], "system.update.deferred")
        self.service.check.assert_called_once_with()

    async def test_check_state_read_error_retries_next_cycle(self):
        with patch.object(updates.LOG, "warning") as warning:
            await self.run_cycles([{"auto_check": True}] * 2, [ValueError("状态格式错误"), {}], count=2)
        warning.assert_called_once()
        self.service.check.assert_called_once_with()

    async def test_missing_files_keep_default_detection_only(self):
        for filename in ("preferences.json", "check.json"):
            (self.directory / filename).unlink()
        with patch.object(updates.time, "time", return_value=10000):
            await self.one_cycle()
        self.service.check.assert_called_once_with()
        self.service.install.assert_not_called()

    async def test_check_error_preserves_next_attempt_backoff(self):
        self.service.check.side_effect = OSError("检查失败")
        preferences = {"auto_check": True, "interval_minutes": 15}
        with patch.object(updates.LOG, "warning") as warning:
            await self.run_cycles([preferences, preferences], {}, count=2)
        warning.assert_called_once()
        self.service.check.assert_called_once_with()
        self.service.install.assert_not_called()


if __name__ == "__main__":
    unittest.main(verbosity=2)
