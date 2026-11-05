"""当前任务列表保留活动任务和有界完成历史，结果来自唯一任务状态。"""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.capability import Capability
from gateways.agent.service import AgentToolService
from gateways.invoke import invoke
from scripts.diagnostics.current_fixtures import task_platform,finished

class TaskSummaryChecks(unittest.TestCase):
    def test_history_order_retention_and_complete_result(self):
        with task_platform(history_limit=2) as p:
            p.capabilities.register(Capability("work","样例",{"type":"object"},"task",lambda p,c,a:a))
            ids=[]
            for value in range(3):
                task_id=invoke(p,"work",{"value":value},"check")["task"]["task_id"]
                finished(p,task_id);ids.append(task_id)
            rows=AgentToolService(p).list_tasks(2)
            self.assertEqual([r["task_id"] for r in rows["history_tasks"]],list(reversed(ids[1:])))
            self.assertEqual(rows["history_tasks"][0]["result"],{"value":2})
            self.assertEqual(rows["active_tasks"],[])
            self.assertIsNone(p.tasks.status(ids[0]))

if __name__=="__main__":unittest.main(verbosity=2)
