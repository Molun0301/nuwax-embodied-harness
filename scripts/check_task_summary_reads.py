"""当前任务列表保留活动任务和有界完成历史，结果来自唯一任务状态。"""
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.capability import Capability
from core.tasks import TaskState
from gateways.agent.service import AgentToolService
from gateways.invoke import invoke
from scripts.diagnostics.current_fixtures import task_platform,finished

class TaskSummaryChecks(unittest.TestCase):
    def test_history_order_retention_and_complete_result(self):
        with task_platform(history_limit=2) as p:
            p.capabilities.register(Capability("work","样例", "样例",{"type":"object"},"task",lambda p,c,a:a))
            ids=[]
            for value in range(3):
                task_id=invoke(p,"work",{"value":value},"check")["task"]["task_id"]
                finished(p,task_id);ids.append(task_id)
            rows=AgentToolService(p).list_tasks(2)
            self.assertEqual([r["task_id"] for r in rows["history_tasks"]],list(reversed(ids[1:])))
            self.assertEqual(rows["history_tasks"][0]["result"],{"value":2})
            self.assertEqual(rows["active_tasks"],[])
            self.assertIsNone(p.tasks.status(ids[0]))

class TaskTimeoutChecks(unittest.TestCase):
    def test_timeout_keeps_latest_reported_reason_or_stage_after_cleanup(self):
        for stage, message in (("waiting_navigation", "等待路线：前方地面支持不足"), ("等待地图更新", "")):
            with self.subTest(stage=stage), task_platform() as p:
                cleaning, release = threading.Event(), threading.Event()
                def handler(platform, ctx, args):
                    ctx.progress(.1, "starting", "旧阶段原因")
                    ctx.progress(.4, stage, message, waiting=True)
                    try:
                        ctx.wait(10)
                    finally:
                        cleaning.set()
                        release.wait()
                p.capabilities.register(Capability("work", "样例", "样例", {"type":"object"}, "task", handler, timeout_sec=.1))
                task_id = invoke(p, "work", {}, "check")["task"]["task_id"]
                try:
                    self.assertTrue(cleaning.wait(2), "到期后必须先完成处理器收尾")
                    current = p.tasks.status(task_id)
                    self.assertEqual(current.state, TaskState.CANCELLING)
                    self.assertEqual(current.message, "期限")
                    self.assertIsNone(current.finished_at)
                finally:
                    release.set()
                status = finished(p, task_id)
                self.assertEqual(status.state, TaskState.TIMED_OUT)
                self.assertEqual(status.cancel_reason, "期限")
                self.assertIn("已超时", status.message)
                self.assertIn(message or stage, status.message)
                self.assertNotIn("旧阶段原因", status.message)
                self.assertEqual(status.progress, .4)
                self.assertEqual(status.stage, stage)
                self.assertEqual(status.data, {"waiting": True})

    def test_timeout_without_progress_keeps_timeout_message(self):
        with task_platform() as p:
            p.capabilities.register(Capability("work", "样例", "样例", {"type":"object"}, "task", lambda p,c,a:c.wait(10), timeout_sec=.02))
            task_id = invoke(p, "work", {}, "check")["task"]["task_id"]
            status = finished(p, task_id)
            self.assertEqual(status.state, TaskState.TIMED_OUT)
            self.assertEqual(status.message, "任务已超时。")

if __name__=="__main__":unittest.main(verbosity=2)
