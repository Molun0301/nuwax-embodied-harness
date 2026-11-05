"""任务准备完成后再次检查取消和期限，已取消任务不派发业务动作。"""
from pathlib import Path
import sys
import threading
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from core.capability import Capability
from core.tasks import TaskState
from gateways.invoke import invoke
from scripts.diagnostics.current_fixtures import task_platform,finished

class PreparationChecks(unittest.TestCase):
    def test_cancel_during_ensure_never_dispatches_handler(self):
        for timeout in (False,True):
            with self.subTest(timeout=timeout),task_platform() as p:
                entered,release,dispatched=threading.Event(),threading.Event(),threading.Event()
                def ensure(names,budget):entered.set();release.wait(2)
                p.ensure=ensure
                p.capabilities.register(Capability("prepare","准备",{"type":"object"},"task",
                    lambda *a:dispatched.set(),needs=("geometry",),timeout_sec=.03 if timeout else 2))
                task_id=invoke(p,"prepare",{},"check")["task"]["task_id"]
                self.assertTrue(entered.wait(1))
                if timeout:
                    self.assertTrue(p.tasks._contexts[task_id].cancelled.wait(1))
                else:p.tasks.cancel(task_id)
                release.set()
                self.assertEqual(finished(p,task_id).state,TaskState.TIMED_OUT if timeout else TaskState.CANCELLED)
                self.assertFalse(dispatched.is_set())

if __name__=="__main__":unittest.main(verbosity=2)
