"""保留策略后台执行真实删除，关闭等待当前清理结束。"""
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.artifacts import ArtifactKind,ArtifactRetentionPolicy
from services.artifact_store import LocalArtifactStore
from services.artifact_maintenance import ArtifactMaintenanceWorker

class MaintenanceChecks(unittest.TestCase):
    def test_request_performs_retention_and_close_reaps_worker(self):
        with tempfile.TemporaryDirectory(prefix="a2d-maintenance-") as directory:
            store=LocalArtifactStore(directory,"")
            refs=[store.save_bytes(kind=ArtifactKind.IMAGE,mime_type="image/png",data=b"image") for _ in range(3)]
            worker=ArtifactMaintenanceWorker(store,policy=ArtifactRetentionPolicy(max_count=1))
            worker.start();worker.request()
            deadline=time.monotonic()+2
            while store.build_summary().artifact_count!=1 and time.monotonic()<deadline:time.sleep(.002)
            worker.close()
            self.assertEqual([r.artifact_id for r in store.list_refs()],[refs[-1].artifact_id])
            self.assertFalse(worker.status()["thread_alive"])
    def test_close_waits_for_active_cleanup_without_false_retry(self):
        entered,release,closed=threading.Event(),threading.Event(),threading.Event()
        class Store:
            def cleanup(self,policy,**kwargs):entered.set();release.wait(2)
        worker=ArtifactMaintenanceWorker(Store(),policy=ArtifactRetentionPolicy())
        worker.start();worker.request();self.assertTrue(entered.wait(1))
        thread=threading.Thread(target=lambda:(worker.close(),closed.set()));thread.start()
        try:self.assertFalse(closed.wait(.02))
        finally:release.set();thread.join(2)
        self.assertTrue(closed.is_set());self.assertFalse(worker.status()["thread_alive"])

if __name__=="__main__":unittest.main(verbosity=2)
