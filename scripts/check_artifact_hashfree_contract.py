#!/usr/bin/env python3
"""制品无摘要合同、旧侧车和客户端下载的离线合成回归。"""
from __future__ import annotations

import io
import json
import socket
import struct
import sys
import tempfile
import unittest
from email.message import Message
from pathlib import Path
from urllib.error import HTTPError
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from clients.robot_client import RobotClient, RobotClientError
from contracts.artifacts import ArtifactKind, ArtifactRef
from core.errors import GatewayError
from services.artifact_store import LocalArtifactStore


ARTIFACT_ID = "art_image_20261002T000000Z_0123abcd"


class Response(io.BytesIO):
    def __init__(self, data, *, length=None, media_type="application/octet-stream", chunk_size=65536, fail_after=None):
        super().__init__(data)
        self.headers = Message()
        self.headers["Content-Type"] = media_type
        if length is not None:
            self.headers["Content-Length"] = str(length)
        self.chunk_size = chunk_size
        self.fail_after = fail_after
        self.reads = 0

    def read(self, size=-1):
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after:
            raise socket.timeout("synthetic interrupted transfer")
        return super().read(min(size, self.chunk_size) if size >= 0 else self.chunk_size)


class Opener:
    def __init__(self, metadata, response):
        self.metadata = metadata
        self.response = response
        self.requests = []

    def open(self, request, **kwargs):
        self.requests.append(request)
        if request.full_url.endswith("/meta"):
            return Response(json.dumps(self.metadata).encode(), media_type="application/json")
        if isinstance(self.response, BaseException):
            raise self.response
        return self.response


class ArtifactReferenceChecks(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-artifact-contract-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.store = LocalArtifactStore(str(self.root), "http://synthetic.invalid")

    def save(self, data=b"synthetic bytes"):
        return self.store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=data)

    def test_new_reference_and_sidecar_omit_digest_without_changing_identity(self):
        first, second = self.save(), self.save()
        self.assertNotEqual(first.artifact_id, second.artifact_id)
        self.assertNotIn("sha256", first.model_dump())
        self.assertFalse(hasattr(first, "sha256"))
        metadata_path = next(self.root.glob(f"**/{first.artifact_id}.meta.json"))
        self.assertNotIn("sha256", json.loads(metadata_path.read_text()))
        self.assertEqual(first.size_bytes, len(b"synthetic bytes"))
        self.assertEqual(self.store.read_bytes(first.artifact_id), b"synthetic bytes")

    def test_old_sidecar_and_reference_roundtrip_ignore_digest_without_rewriting_source(self):
        ref = self.save()
        path = next(self.root.glob(f"**/{ref.artifact_id}.meta.json"))
        old = ref.model_dump(mode="json")
        old["sha256"] = "obsolete and intentionally unverified"
        path.write_text(json.dumps(old))
        before = path.read_bytes()
        reopened = LocalArtifactStore(str(self.root), "http://synthetic.invalid")
        loaded = reopened.get_ref(ref.artifact_id)
        self.assertEqual(loaded.model_dump(), ref.model_dump())
        self.assertEqual(path.read_bytes(), before)
        for value in (None, "", "old text"):
            old["sha256"] = value
            parsed = ArtifactRef.model_validate(old)
            self.assertIn("sha256", old)
            self.assertNotIn("sha256", parsed.model_dump_json())
            self.assertEqual(ArtifactRef.model_validate_json(parsed.model_dump_json()), parsed)



    def test_failed_sidecar_publication_does_not_leave_half_artifact(self):
        import os
        original = os.replace

        def fail_metadata(source, target):
            if str(target).endswith(".meta.json"):
                raise OSError("synthetic metadata publication failure")
            return original(source, target)

        with patch("services.artifact_store.os.replace", side_effect=fail_metadata):
            with self.assertRaises(OSError):
                self.save()
        self.assertEqual([path for path in self.root.rglob("*") if path.is_file()], [])









class ArtifactDownloadChecks(unittest.TestCase):
    def test_success_downloads_exact_bytes_with_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/"image.bin"
            client=RobotClient("http://synthetic.invalid")
            metadata={"artifact_id":ARTIFACT_ID,"size_bytes":1400}
            client._opener=Opener(metadata,Response(b"image bytes"*200,chunk_size=7))
            result=client.download_artifact(ARTIFACT_ID,output)
            self.assertEqual(output.read_bytes(),b"image bytes"*200)
            self.assertEqual(result,{"artifact":metadata,"path":str(output)})

    def test_existing_destination_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)/"image.bin";output.write_bytes(b"existing")
            client=RobotClient("http://synthetic.invalid")
            client._opener=Opener({},Response(b"replacement"))
            with self.assertRaises(FileExistsError):client.download_artifact(ARTIFACT_ID,output)
            self.assertEqual(output.read_bytes(),b"existing")

if __name__=="__main__":unittest.main(verbosity=2)
