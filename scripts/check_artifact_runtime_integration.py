"""真实HTTP制品元数据、完整下载及范围下载，制品按普通文件资源关闭。"""
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fastapi import FastAPI
from fastapi.testclient import TestClient
from core.errors import ArtifactNotFoundError
from contracts.artifacts import ArtifactKind,ArtifactRetentionPolicy
from gateways.http.api import build_http_router
from core.settings import load_config
from services.artifact_service import ArtifactService
from services.artifact_store import LocalArtifactStore

class RuntimeChecks(unittest.TestCase):
    def test_formal_http_metadata_raw_range_and_missing(self):
        with tempfile.TemporaryDirectory(prefix="a2d-artifact-http-") as directory:
            store=LocalArtifactStore(directory,"");service=ArtifactService(store,retention_policy=ArtifactRetentionPolicy())
            try:
                ref=store.save_bytes(kind=ArtifactKind.IMAGE,mime_type="image/png",data=b"0123456789")
                app=FastAPI();app.include_router(build_http_router(SimpleNamespace(artifacts=service,config=load_config()),None))
                with TestClient(app) as client:
                    self.assertEqual(client.get(f"/artifacts/{ref.artifact_id}/meta").json()["artifact_id"],ref.artifact_id)
                    self.assertEqual(client.get(f"/artifacts/{ref.artifact_id}").content,b"0123456789")
                    response=client.get(f"/artifacts/{ref.artifact_id}",headers={"Range":"bytes=2-5"})
                    self.assertEqual(response.status_code,206);self.assertEqual(response.content,b"2345")
                    store.delete_artifact(ref.artifact_id)
                    with self.assertRaises(ArtifactNotFoundError):client.get(f"/artifacts/{ref.artifact_id}")
            finally:service.close()

if __name__=="__main__":unittest.main(verbosity=2)
