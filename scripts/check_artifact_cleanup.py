"""临时制品真实读写、删除与保留策略检查，仅使用临时目录。"""
from pathlib import Path
import sys
import tempfile
import unittest
from datetime import timedelta
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from contracts.artifacts import ArtifactKind,ArtifactRetentionPolicy
from contracts.base import utc_now
from core.errors import ArtifactNotFoundError,GatewayError
from services.artifact_store import LocalArtifactStore

class ArtifactChecks(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(prefix="a2d-artifact-");self.addCleanup(self.tmp.cleanup)
        self.store=LocalArtifactStore(self.tmp.name,"http://synthetic.invalid")
    def save(self,data=b"data"):
        return self.store.save_bytes(kind=ArtifactKind.IMAGE,mime_type="image/png",data=data)
    def test_read_reopen_and_explicit_delete(self):
        ref=self.save(b"actual bytes")
        self.assertEqual(LocalArtifactStore(self.tmp.name,"").read_bytes(ref.artifact_id),b"actual bytes")
        self.store.delete_artifact(ref.artifact_id)
        with self.assertRaises(ArtifactNotFoundError):self.store.read_bytes(ref.artifact_id)
    def test_batch_count_and_byte_pressure_keep_remaining_files_readable(self):
        refs=[self.save(bytes([number])*4) for number in range(4)]
        result=self.store.cleanup(ArtifactRetentionPolicy(max_count=1,max_total_bytes=4,cleanup_batch_size=2))
        self.assertEqual(result.removed_artifact_ids,[r.artifact_id for r in refs[:2]])
        self.assertEqual(result.freed_bytes,8)
        self.assertEqual(result.summary.artifact_count,2)
        self.assertEqual(self.store.read_bytes(refs[-1].artifact_id),bytes([3])*4)
    def test_expiry_uses_stored_timestamp_and_strict_cutoff(self):
        ref=self.save();path=next(Path(self.tmp.name).glob("**/*.meta.json"));now=utc_now()
        path.write_text(ref.model_copy(update={"timestamp":now-timedelta(days=2)}).model_dump_json())
        self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy(retention_days=2),as_of=now).removed_count,0)
        self.assertEqual(self.store.cleanup(ArtifactRetentionPolicy(retention_days=2),as_of=now+timedelta(microseconds=1)).removed_artifact_ids,[ref.artifact_id])
    def test_orphan_is_preserved_and_invalid_sidecar_is_reported(self):
        root=Path(self.tmp.name);(root/"unknown.bin").write_bytes(b"unknown")
        ref=self.save();next(root.glob("**/*.meta.json")).write_text("not json")
        with self.assertRaises(ValueError):self.store.cleanup(ArtifactRetentionPolicy(max_count=1))
        self.assertEqual((root/"unknown.bin").read_bytes(),b"unknown")

if __name__=="__main__":unittest.main(verbosity=2)
