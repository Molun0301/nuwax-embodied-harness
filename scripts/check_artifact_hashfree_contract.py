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
from gateways.agent.feedback import collect_artifacts
from services.artifact_store import LocalArtifactStore
from scripts.diagnostics.semantic_map_development import recording_artifact
from scripts.diagnostics import semantic_map_development


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

    def test_other_schema_rules_remain_and_old_feedback_references_are_still_extracted(self):
        ref = self.save()
        old = {**ref.model_dump(mode="json"), "sha256": "old field"}
        refs, count, complete = collect_artifacts({"artifact": old})
        self.assertEqual((count, complete), (1, True))
        self.assertEqual(refs[0]["artifact_id"], ref.artifact_id)
        self.assertNotIn("sha256", refs[0])
        for changes in ({"size_bytes": -1}, {"unexpected": True}, {"artifact_id": "../other"}):
            with self.assertRaises(ValueError):
                ArtifactRef.model_validate({**old, **changes})

    def test_reader_hold_and_collision_rules_survive_digest_removal(self):
        ref = self.save()
        with self.store.read_artifact(ref.artifact_id) as (loaded, stream):
            self.assertEqual(loaded.artifact_id, ref.artifact_id)
            with self.assertRaises(GatewayError) as caught:
                self.store.delete_artifact(ref.artifact_id)
            self.assertEqual(caught.exception.error_code, "artifact_in_use")
            self.assertEqual(stream.read(), b"synthetic bytes")
        with patch("services.artifact_store.build_artifact_id", return_value=ref.artifact_id):
            with self.assertRaises(GatewayError) as caught:
                self.save(b"replacement")
            self.assertEqual(caught.exception.error_code, "artifact_id_collision")
        self.assertEqual(self.store.read_bytes(ref.artifact_id), b"synthetic bytes")

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

    def import_recording(self, data, *, kind=ArtifactKind.MAP_SNAPSHOT):
        with self.store.publication_scope(owner="synthetic.recording") as owner:
            ref = recording_artifact(self.store, source_path=self.root / "source.bin", kind=kind,
                mime_type="application/octet-stream", data=data, extension=".bin",
                map_name="synthetic", frame_id="map", owner=owner)
            with self.assertRaises(GatewayError) as caught:
                self.store.delete_artifact(ref.artifact_id)
            self.assertEqual(caught.exception.error_code, "artifact_in_use")
            return ref

    def test_repeated_recording_input_reuses_persisted_owner_without_reimport_identity(self):
        for kind in (ArtifactKind.MAP_SNAPSHOT, ArtifactKind.POINTCLOUD):
            first = self.import_recording(b"first input", kind=kind)
            second = self.import_recording(b"first input", kind=kind)
            self.assertEqual(second.artifact_id, first.artifact_id)
            self.assertEqual("offline-recording:" + second.artifact_id, "offline-recording:" + first.artifact_id)
        self.assertEqual(len(self.store.list_refs()), 2)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_same_path_size_but_changed_bytes_gets_new_recording_identity(self):
        first = self.import_recording(b"first input")
        changed = self.import_recording(b"other input")
        self.assertEqual(first.size_bytes, changed.size_bytes)
        self.assertNotEqual(first.artifact_id, changed.artifact_id)
        self.assertEqual(self.store.read_bytes(first.artifact_id), b"first input")
        self.assertEqual(self.import_recording(b"other input").artifact_id, changed.artifact_id)

    def test_missing_recording_artifact_recovers_with_new_stable_identity(self):
        first = self.import_recording(b"first input")
        # 仅删除本测试临时目录中的数据文件，模拟旧侧车仍在但内容已不可读。
        self.store.get_file_path(first.artifact_id).unlink()
        restored = self.import_recording(b"first input")
        self.assertNotEqual(restored.artifact_id, first.artifact_id)
        self.assertEqual(self.import_recording(b"first input").artifact_id, restored.artifact_id)
        self.assertEqual(self.store.read_bytes(restored.artifact_id), b"first input")

    def test_recording_server_retains_only_inputs_and_releases_on_exit_or_error(self):
        occupancy = self.root / "occupancy.json"
        occupancy.write_text(json.dumps({"ts": 1, "frame_id": "map", "origin": [0, 0, 0],
            "origin_orientation": [0, 0, 0, 1], "width": 1, "height": 1,
            "resolution": 0.1, "data": [0]}))
        cloud = self.root / "cloud.bin"
        cloud.write_bytes(struct.pack("<4f", 1, 2, 3, 0))

        def check_inputs_and_unrelated_save():
            status = self.store.get_holder_status()
            self.assertEqual(status["holder_count"], 1)
            self.assertEqual(status["protected_artifact_count"], 2)
            self.assertEqual(status["owners"], ["diagnostic.offline_recording.inputs"])
            for ref in self.store.list_refs():
                with self.assertRaises(GatewayError) as caught:
                    self.store.delete_artifact(ref.artifact_id)
                self.assertEqual(caught.exception.error_code, "artifact_in_use")
            unrelated = self.save(b"unrelated later save")
            self.assertEqual(self.store.get_holder_status()["protected_artifact_count"], 2)
            self.store.delete_artifact(unrelated.artifact_id)
            return {}

        for server_error in (False, True):
            with self.subTest(server_error=server_error):
                runtime = MagicMock(artifact_store=self.store)
                runtime.scene_mapping_service.run_once.side_effect = lambda **kwargs: check_inputs_and_unrelated_save()

                def serve(*args, **kwargs):
                    check_inputs_and_unrelated_save()
                    if server_error:
                        raise RuntimeError("synthetic server failure")

                with patch.object(semantic_map_development, "load_config", return_value=MagicMock()), \
                     patch.object(semantic_map_development, "create_customrobot_assembly"), \
                     patch.object(semantic_map_development, "GatewayRuntime", return_value=runtime), \
                     patch.object(semantic_map_development, "create_gateway_app"), \
                     patch.object(semantic_map_development.uvicorn, "run", side_effect=serve), \
                     patch.object(sys, "argv", ["semantic-map-development", "--occupancy", str(occupancy),
                         "--pointcloud", str(cloud), "--output-dir", str(self.root / "output")]):
                    if server_error:
                        with self.assertRaisesRegex(RuntimeError, "synthetic server failure"):
                            semantic_map_development.main()
                    else:
                        semantic_map_development.main()
                runtime.stop.assert_called_once_with()
                self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
                self.assertEqual(self.store.get_holder_status()["protected_artifact_count"], 0)


class ArtifactDownloadChecks(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-artifact-download-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / "download.bin"

    def client(self, data=b"synthetic bytes", *, metadata=None, response=None, length=None, **options):
        meta = {"artifact_id": ARTIFACT_ID, "kind": "image", "mime_type": "image/jpeg", "size_bytes": len(data)}
        meta.update(metadata or {})
        client = RobotClient("http://synthetic.invalid", token="synthetic-token")
        client._opener = Opener(meta, response if response is not None else Response(data, length=length, **options))
        return client

    def assert_no_partial(self):
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob(".robot-artifact-*")), [])

    def test_success_streams_bytes_and_returns_no_digest_or_integrity_claim(self):
        data = b"synthetic bytes" * 100
        client = self.client(data, length=len(data), chunk_size=7)
        result = client.download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(self.output.read_bytes(), data)
        self.assertEqual((result["artifact_id"], result["size_bytes"]), (ARTIFACT_ID, len(data)))
        self.assertNotIn("sha256", result)
        self.assertNotIn("sha256", result["source"])
        self.assertGreater(client._opener.response.reads, 2)
        self.assertEqual(list(self.root.glob(".robot-artifact-*")), [])

    def test_new_client_accepts_old_optional_digest_and_ignores_external_uri(self):
        client = self.client(metadata={"sha256": "wrong obsolete value", "uri": "http://other.invalid/private"})
        result = client.download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(result["source"]["artifact_id"], ARTIFACT_ID)
        self.assertNotIn("sha256", result["source"])
        self.assertTrue(all(request.full_url.startswith("http://synthetic.invalid/artifacts/") for request in client._opener.requests))
        self.assertTrue(all(request.get_header("Authorization") == "Bearer synthetic-token" for request in client._opener.requests))
        # 老客户端只在可选字段为真时比较；新引用省略该字段，不提供伪校验值。
        self.assertIsNone(result["source"].get("sha256"))

    def test_zero_byte_download_and_missing_content_length_are_supported(self):
        result = self.client(b"").download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(result["size_bytes"], 0)
        self.assertEqual(self.output.read_bytes(), b"")

    def test_declared_length_truncation_leaves_no_file(self):
        with self.assertRaises(RobotClientError) as caught:
            self.client(length=100).download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(caught.exception.payload["code"], "artifact_unavailable")
        self.assert_no_partial()

    def test_metadata_size_mismatch_leaves_no_file(self):
        with self.assertRaises(RobotClientError) as caught:
            self.client(metadata={"size_bytes": 100}).download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(caught.exception.payload["code"], "artifact_size_mismatch")
        self.assert_no_partial()

    def test_transfer_budget_and_midstream_timeout_leave_no_file(self):
        for client, limit, code in (
            (self.client(), 4, "artifact_too_large"),
            (self.client(chunk_size=2, fail_after=1), 100, "artifact_unavailable"),
        ):
            with self.assertRaises(RobotClientError) as caught:
                client.download_artifact(ARTIFACT_ID, self.output, max_bytes=limit)
            self.assertEqual(caught.exception.payload["code"], code)
            self.assert_no_partial()

    def test_existing_and_symlink_destinations_are_not_overwritten(self):
        self.output.write_bytes(b"user file")
        with self.assertRaises(RobotClientError) as caught:
            self.client().download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(caught.exception.payload["code"], "output_exists")
        self.assertEqual(self.output.read_bytes(), b"user file")
        link = self.root / "link.bin"
        link.symlink_to(self.output)
        with self.assertRaises(RobotClientError):
            self.client().download_artifact(ARTIFACT_ID, link)
        self.assertTrue(link.is_symlink())

    def test_exclusive_publication_race_keeps_the_competing_file(self):
        def race(_source, target):
            Path(target).write_bytes(b"competing file")
            raise FileExistsError("synthetic destination race")

        with patch("clients.robot_client.os.link", side_effect=race):
            with self.assertRaises(RobotClientError) as caught:
                self.client().download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(caught.exception.payload["code"], "output_exists")
        self.assertEqual(self.output.read_bytes(), b"competing file")
        self.assertEqual(list(self.root.glob(".robot-artifact-*")), [])

    def test_metadata_identity_error_and_http_failure_never_publish(self):
        client = self.client(metadata={"artifact_id": "other"})
        with self.assertRaises(RobotClientError) as caught:
            client.download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(caught.exception.payload["code"], "invalid_artifact")
        self.assertEqual(len(client._opener.requests), 1)
        error = HTTPError("http://synthetic.invalid/artifacts/" + ARTIFACT_ID, 404, "missing", {}, io.BytesIO())
        with self.assertRaises(RobotClientError) as caught:
            self.client(response=error).download_artifact(ARTIFACT_ID, self.output)
        self.assertEqual(caught.exception.payload["code"], "artifact_unavailable")
        self.assert_no_partial()


if __name__ == "__main__":
    unittest.main(verbosity=2)
