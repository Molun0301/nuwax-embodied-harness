"""智能体等待直接消费当前任务状态；等待不阻塞取消、超时与同一事件循环。"""
import asyncio
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.capability import Capability
from core.tasks import TaskState
from gateways.agent.service import AgentToolService
from gateways.invoke import invoke
from scripts.diagnostics.current_fixtures import task_platform, finished


class AgentWaitIsolationTests(unittest.IsolatedAsyncioTestCase):
    async def test_wait_returns_real_terminal_result_without_blocking_event_loop(self):
        with task_platform() as platform:
            release = threading.Event()
            def handler(p, ctx, args):
                while not release.is_set():
                    ctx.wait(.01)
                return {"value": "真实处理结果"}
            platform.capabilities.register(Capability("wait_example", "等待", "等待", {"type":"object"}, "task", handler))
            service = AgentToolService(platform)
            task = service.invoke("wait_example", {})["task"]
            work = asyncio.create_task(service.wait_task_async(task["task_id"], 2))
            await asyncio.sleep(.01)
            self.assertFalse(work.done())
            self.assertEqual(service.get_task(task["task_id"])["task"]["state"], "running")
            release.set()
            result = await work
            self.assertEqual(result["task"]["state"], "succeeded")
            self.assertEqual(result["task"]["result"], {"value":"真实处理结果"})
            service.close()

    async def test_wait_timeout_does_not_cancel_the_task(self):
        with task_platform() as platform:
            platform.capabilities.register(Capability("wait_example", "等待", "等待", {"type":"object"}, "task",
                lambda p,c,a: c.wait(10)))
            task_id = invoke(platform,"wait_example",{},"check")["task"]["task_id"]
            service = AgentToolService(platform)
            self.assertEqual((await service.wait_task_async(task_id,.01))["task"]["state"],"running")
            platform.tasks.cancel(task_id)
            self.assertEqual((await service.wait_task_async(task_id,2))["task"]["state"],"cancelled")
            service.close()

    async def test_terminal_is_published_after_cancel_cleanup(self):
        with task_platform() as platform:
            cleaning, release = threading.Event(), threading.Event()
            def handler(p,ctx,args):
                try:ctx.wait(10)
                finally:cleaning.set(); release.wait(2)
            platform.capabilities.register(Capability("wait_example","等待", "等待",{"type":"object"},"task",handler))
            task_id=invoke(platform,"wait_example",{},"check")["task"]["task_id"]
            await asyncio.sleep(.01)
            platform.tasks.cancel(task_id)
            self.assertTrue(await asyncio.get_running_loop().run_in_executor(None, cleaning.wait, 1))
            self.assertEqual(platform.tasks.status(task_id).state,TaskState.CANCELLING)
            release.set()
            self.assertEqual((await AgentToolService(platform).wait_task_async(task_id,2))["task"]["state"],"cancelled")


if __name__=="__main__":unittest.main(verbosity=2)
