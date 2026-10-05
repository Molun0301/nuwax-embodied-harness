#!/usr/bin/env python3
"""专属维护、健康读取、下载生命周期及关闭的离线合成回归。"""
from __future__ import annotations

import asyncio
import gc
import sys
import tempfile
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.artifacts import ArtifactCleanupResult, ArtifactKind, ArtifactRetentionPolicy
from contracts.image import ImageEncoding, ImageFrame
from contracts.runtime_views import ServiceHealthSummary
from core import StateStore
from core.errors import GatewayError
from gateways.http.api import ArtifactFileResponse
from gateways.runtime import GatewayRuntime
from services.artifact_service import ArtifactService
from services.artifact_store import LocalArtifactStore
from services.memory.artifact_references import MemoryArtifactReferences
from services.memory.repository import MemoryRepository
from services.observation_service import ObservationService


class RuntimeChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-artifact-runtime-")
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.store = LocalArtifactStore(str(root / "artifacts"), "http://diagnostic.invalid")
        self.repo = MemoryRepository(str(root / "memory.db"))
        self.addCleanup(self.repo.close)
        self.bridge = MemoryArtifactReferences(self.repo, self.store)
        self.addCleanup(self.bridge.close)
        self.service = ArtifactService(self.store, retention_policy=ArtifactRetentionPolicy(max_count=1))
        self.addCleanup(self.service.close)
        self.assertTrue(self.service.start_maintenance())

    def frame(self):
        return ImageFrame(camera_id="synthetic", frame_id="camera", width_px=1, height_px=1,
                          encoding=ImageEncoding.JPEG, data=b"synthetic-image")

    def save(self):
        return self.store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"synthetic-image")

    def health_runtime(self):
        runtime = object.__new__(GatewayRuntime)
        runtime.artifact_service, runtime.artifact_store = self.service, self.store
        runtime._started, runtime._closed = True, False
        runtime.config = SimpleNamespace(gateway=SimpleNamespace(public_base_url="http://diagnostic.invalid"))
        runtime.robot = SimpleNamespace(manifest=SimpleNamespace(robot_name="synthetic", robot_model="test"), get_status=dict)
        runtime.robot_state_service = SimpleNamespace(get_latest_snapshot=lambda: None)
        runtime.localization_service = SimpleNamespace(get_latest_snapshot=lambda: None, get_active_session=lambda: None)
        runtime.mapping_service = SimpleNamespace(get_latest_snapshot=lambda: None)
        runtime.navigation_service = SimpleNamespace(get_latest_navigation_context=lambda: None, get_latest_exploration_context=lambda: None)
        runtime.perception_video_runtime = SimpleNamespace(get_status=lambda: None)
        runtime.map_workspace_service = SimpleNamespace(get_active_workspace_identity=lambda: None, get_active_workspace=lambda: None)
        runtime.platform_geometry_runtime = SimpleNamespace(get_runtime_status=dict, get_live_mapping_diagnostics=dict)
        runtime.audio_service = SimpleNamespace(get_cached_volume_state=dict)
        runtime.task_manager = SimpleNamespace(list_active_tasks=list)
        runtime.event_bus = SimpleNamespace(latest_cursor=lambda: 0)
        runtime.stream_bus = SimpleNamespace(latest_cursor=lambda: 0, status=dict)
        runtime.resource_lock_manager = SimpleNamespace(snapshot=tuple)
        runtime.get_event_log_status = dict
        runtime._runtime_geometry_asset_summary = lambda *a: {}
        runtime._runtime_mapping_profile_summary = lambda *a: {}
        runtime._runtime_last_relocalization_summary = lambda *a: {}
        runtime._runtime_navigation_control_policy_summary = lambda *a: {}
        runtime._runtime_localization_session_summary = lambda *a: {}
        runtime._build_service_health_summary = lambda **kw: ServiceHealthSummary()
        return runtime

    def test_blocked_scan_capture_commit_and_health_stay_responsive(self):
        self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        scan = self.store._scan_entries
        def blocked_scan():
            entered.set()
            self.assertTrue(release.wait(5))
            return scan()
        with patch.object(self.store, "_scan_entries", side_effect=blocked_scan), self.service.publication_scope(owner="capture") as publication:
            first = self.service.save_image_frame(self.frame())
            self.assertTrue(entered.wait(2))
            with ThreadPoolExecutor(max_workers=2) as pool:
                def capture():
                    with self.service.publication_scope(owner="parallel.capture") as pending:
                        artifact = self.service.save_image_frame(self.frame())
                        return artifact, pending.retain(owner="latest")
                future = pool.submit(capture)
                try:
                    second, latest = future.result(timeout=1)
                    self.addCleanup(latest.release)
                    self.assertTrue(self.store.get_file_path(second.artifact_id).exists())
                    with patch.object(self.store, "build_summary", side_effect=AssertionError("health scanned")), patch.object(self.bridge, "_read_index", side_effect=AssertionError("health queried references")):
                        health = pool.submit(self.health_runtime().get_health_summary).result(timeout=1)
                    self.assertTrue(health["artifact_maintenance"]["maintenance_pending"])
                    self.assertEqual(health["artifact_summary"]["scope"], "unknown")
                    self.assertIsNone(health["artifact_summary"]["artifact_count"])
                    self.assertFalse(release.is_set())
                    self.assertIn(first.artifact_id, publication.artifact_ids)
                finally:
                    release.set()
            self.assertTrue(self.service._maintenance_worker.wait_idle(3))

    def test_blocked_reference_provider_does_not_block_capture_or_health(self):
        self.save()
        self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        read_index = self.bridge._read_index
        def blocked_provider():
            entered.set()
            self.assertTrue(release.wait(5))
            return read_index()
        with patch.object(self.bridge, "_read_index", side_effect=blocked_provider):
            self.service._maintenance_worker.request()
            self.assertTrue(entered.wait(2))
            with self.service.publication_scope(owner="capture"), ThreadPoolExecutor(max_workers=1) as pool:
                try:
                    artifact = pool.submit(self.service.save_image_frame, self.frame()).result(timeout=1)
                    health = pool.submit(self.health_runtime().get_health_summary).result(timeout=1)
                    self.assertTrue(self.store.get_file_path(artifact.artifact_id).exists())
                    self.assertEqual(health["artifact_maintenance"]["worker"]["active_requests"], 1)
                finally:
                    release.set()
            self.assertTrue(self.service._maintenance_worker.wait_idle(3))

    def test_actual_observation_capture_returns_owned_context_during_blocked_scan(self):
        provider = SimpleNamespace(provider_name="synthetic", provider_version="1", is_available=lambda: True,
                                   capture_image=lambda _camera: self.frame(), get_camera_info=lambda _camera: None)
        observations = ObservationService(provider_owner=SimpleNamespace(providers=provider),
                                          artifact_service=self.service, state_store=StateStore(), history_limit=1)
        self.addCleanup(observations.close)
        self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        scan = self.store._scan_entries
        def blocked():
            entered.set()
            self.assertTrue(release.wait(5))
            return scan()
        with patch.object(self.store, "_scan_entries", side_effect=blocked), ThreadPoolExecutor(max_workers=1) as pool:
            self.service._maintenance_worker.request()
            self.assertTrue(entered.wait(2))
            try:
                context = pool.submit(observations.capture_observation).result(timeout=1)
                self.assertIs(observations.get_latest_observation(), context)
                self.assertTrue(self.store.get_file_path(context.image_artifact.artifact_id).exists())
            finally:
                release.set()
            self.assertTrue(self.service._maintenance_worker.wait_idle(3))
        with self.assertRaises(GatewayError) as error:
            self.store.delete_artifact(context.image_artifact.artifact_id)
        self.assertEqual(error.exception.error_code, "artifact_in_use")

    def test_health_cached_snapshot_is_labeled_and_explicit_summary_stays_fresh(self):
        self.save()
        self.service._maintenance_worker.request()
        self.assertTrue(self.service._maintenance_worker.wait_idle(3))
        self.save()
        with patch.object(self.store, "_scan_entries", side_effect=AssertionError("automatic scan")):
            health = self.health_runtime().get_health_summary()
            governance = self.health_runtime().get_runtime_governance_summary()
        self.assertEqual(health["artifact_summary"]["artifact_count"], 1)
        self.assertEqual(health["artifact_summary"]["freshness"], "unverified_since_snapshot")
        self.assertIsNotNone(health["artifact_summary"]["snapshot_time"])
        self.assertEqual(governance.metadata["artifact_summary_scope"], "last_completed_scan")
        self.assertEqual(self.service.get_summary().artifact_count, 2)

    def test_status_compaction_never_copies_unbounded_ids(self):
        class NoIteration(list):
            def __iter__(self):
                raise AssertionError("status walked ID list")
            def __deepcopy__(self, memo):
                raise AssertionError("status copied ID list")
        self.store._maintenance_status = {"status": "deferred", "pending_count": 5,
            "skipped_artifact_ids": NoIteration(["x"]), "result": {"removed_count": 0,
            "protected_count": 1000000, "protected_artifact_ids": NoIteration(["x"]),
            "removed_artifact_ids": NoIteration(["x"])}}
        status = self.service.get_maintenance_status()
        self.assertEqual(status["pending_count"], 5)
        self.assertNotIn("protected_artifact_ids", status["result"])

    def test_worker_not_ready_saves_with_visible_unserviced_debt(self):
        self.service.close()
        self.bridge.close()
        service = ArtifactService(self.store, retention_policy=ArtifactRetentionPolicy(max_count=1))
        self.addCleanup(service.close)
        self.assertFalse(service.start_maintenance())
        with patch.object(self.store, "cleanup", side_effect=AssertionError("sync fallback")):
            artifact = service.save_image_frame(self.frame())
        self.assertTrue(self.store.get_file_path(artifact.artifact_id).exists())
        self.assertEqual(service.get_maintenance_status()["unserviced_saves"], 1)

    def test_real_save_failure_propagates_but_enqueue_failure_keeps_committed_save(self):
        with patch.object(self.store, "save_bytes", side_effect=OSError("disk failure")), self.assertRaisesRegex(OSError, "disk failure"):
            self.service.save_image_frame(self.frame())
        self.assertEqual(self.service.get_maintenance_status()["failed_saves"], 1)
        with patch.object(self.service._maintenance_worker, "request", side_effect=RuntimeError("mailbox failure")), self.assertLogs("services.artifact_service", level="ERROR"):
            artifact = self.service.save_image_frame(self.frame())
        self.assertTrue(self.store.get_file_path(artifact.artifact_id).exists())
        self.assertEqual(self.service.get_maintenance_status()["unserviced_saves"], 1)

    def test_stalled_provider_close_retains_memory_and_late_callback_cannot_delete(self):
        first, second = self.save(), self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        read_index = self.bridge._read_index
        def blocked_provider():
            entered.set()
            self.assertTrue(release.wait(5))
            return read_index()
        runtime = object.__new__(GatewayRuntime)
        runtime.artifact_service = self.service
        calls = []
        runtime.memory_service = SimpleNamespace(close=lambda: calls.append("memory") or True)
        original_close = runtime._close_artifact_dependencies
        runtime._close_artifact_dependencies = lambda: original_close(0.01)
        with patch.object(self.bridge, "_read_index", side_effect=blocked_provider):
            self.service._maintenance_worker.request()
            self.assertTrue(entered.wait(2))
            try:
                with self.assertRaisesRegex(RuntimeError, "Artifact shutdown incomplete"):
                    runtime._close_memory_service()
                self.assertFalse(calls)
                self.assertTrue(self.service.get_maintenance_status()["worker"]["thread_alive"])
            finally:
                release.set()
            self.assertTrue(self.service.close(timeout=2))
        self.assertTrue(self.store.get_file_path(first.artifact_id).exists())
        self.assertTrue(self.store.get_file_path(second.artifact_id).exists())
        runtime._close_memory_service()
        self.assertEqual(calls, ["memory"])

    def test_never_started_dispatch_closes_artifact_owners_before_dependencies(self):
        runtime = object.__new__(GatewayRuntime)
        order = []
        runtime.artifact_service = SimpleNamespace(close=lambda **kw: order.append("maintenance") or True)
        runtime.perception_video_runtime = SimpleNamespace(close=lambda **kw: order.append("video") or True)
        runtime.observation_service = SimpleNamespace(close=lambda **kw: order.append("observation") or True)
        runtime.perception_service = SimpleNamespace(close=lambda **kw: order.append("perception") or True)
        runtime.scene_mapping_service = SimpleNamespace(close=lambda: order.append("mapping"))
        runtime._close_mapping_recording = lambda: None
        runtime._close_runtime_event_bridge = lambda _timeout: order.append("event_bridge") or True
        self.assertTrue(runtime._close_dispatch_resources())
        self.assertEqual(order[:5], ["maintenance", "video", "observation", "perception", "mapping"])

    def test_started_stop_fences_maintenance_before_video_and_mapping_teardown(self):
        from scripts.check_memory_service_regressions import ServiceRegressionChecks
        fixture = ServiceRegressionChecks()
        runtime, order, _events = fixture.runtime_stub(True, [True])
        runtime.artifact_service = SimpleNamespace(close=lambda **kw: order.append("artifact_fence") or True)
        runtime.stop()
        self.assertLess(order.index("tasks"), order.index("platform_geometry_runtime"))
        self.assertLess(order.index("platform_geometry_runtime"), order.index("artifact_fence"))
        self.assertLess(order.index("artifact_fence"), order.index("perception_video_runtime"))
        self.assertLess(order.index("artifact_fence"), order.index("scene_mapping_service"))
        self.assertLess(order.index("artifact_fence"), order.index("memory"))

    def test_policy_updates_keep_worker_and_service_in_acknowledged_order(self):
        entered, release, second_started = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)
        update = self.service._maintenance_worker.update_policy
        def blocked(policy):
            if policy.max_count == 2:
                entered.set()
                self.assertTrue(release.wait(3))
            return update(policy)
        def later_update():
            second_started.set()
            self.service.update_retention_policy(ArtifactRetentionPolicy(max_count=100))
        with patch.object(self.service._maintenance_worker, "update_policy", side_effect=blocked), ThreadPoolExecutor(max_workers=2) as pool:
            first = pool.submit(self.service.update_retention_policy, ArtifactRetentionPolicy(max_count=2))
            self.assertTrue(entered.wait(2))
            second = pool.submit(later_update)
            try:
                self.assertTrue(second_started.wait(2))
                self.assertFalse(second.done())
            finally:
                release.set()
            first.result(3)
            second.result(3)
        self.assertEqual(self.service._retention_policy.max_count, 100)
        self.assertEqual(self.service._maintenance_worker._policy.max_count, 100)
        for _ in range(3):
            self.save()
        self.service.save_image_frame(self.frame())
        self.assertTrue(self.service._maintenance_worker.wait_idle(3))
        self.assertEqual(self.store.build_summary().artifact_count, 4)

    def test_large_cleanup_result_serialization_does_not_hold_save_lock(self):
        self.save()
        self.save()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        serialize = ArtifactCleanupResult.model_dump
        def blocked(result, *args, **kwargs):
            entered.set()
            self.assertTrue(release.wait(4))
            return serialize(result, *args, **kwargs)
        with patch.object(ArtifactCleanupResult, "model_dump", blocked), ThreadPoolExecutor(max_workers=1) as pool:
            self.service._maintenance_worker.request()
            self.assertTrue(entered.wait(2))
            try:
                artifact = pool.submit(self.service.save_image_frame, self.frame()).result(timeout=1)
                self.assertTrue(self.store.get_file_path(artifact.artifact_id).exists())
                self.assertFalse(release.is_set())
            finally:
                release.set()
            self.assertTrue(self.service._maintenance_worker.wait_idle(3))

    def test_busy_producer_retains_memory_dependency(self):
        runtime = object.__new__(GatewayRuntime)
        runtime.artifact_service = self.service
        runtime.perception_video_runtime = SimpleNamespace(close=lambda **kw: False)
        closed = []
        runtime.memory_service = SimpleNamespace(close=lambda: closed.append(True) or True)
        with self.assertRaisesRegex(RuntimeError, "Artifact shutdown incomplete"):
            runtime._close_memory_service()
        self.assertFalse(closed)
        self.assertEqual(runtime._artifact_shutdown_pending, ["perception_video_runtime"])

    def test_read_scope_covers_open_and_releases_before_owned_bytes_use(self):
        artifact = self.save()
        with self.store.read_artifact(artifact.artifact_id) as (ref, stream):
            with self.assertRaises(GatewayError):
                self.store.delete_artifact(artifact.artifact_id)
            data = stream.read()
            self.assertEqual(ref.artifact_id, artifact.artifact_id)
        self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(data, b"synthetic-image")
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_http_response_retains_during_delayed_asgi_and_preserves_range(self):
        artifact = self.save()
        response = ArtifactFileResponse(self.store, artifact.artifact_id)
        with self.assertRaises(GatewayError):
            self.store.delete_artifact(artifact.artifact_id)
        messages = []
        async def receive():
            return {"type": "http.request", "body": b"", "more_body": False}
        async def send(message):
            messages.append(message)
        scope = {"type": "http", "method": "GET", "headers": [(b"range", b"bytes=0-3")], "extensions": {"http.response.pathsend": {}}}
        asyncio.run(response(scope, receive, send))
        self.assertEqual(messages[0]["status"], 206)
        self.assertEqual(b"".join(message.get("body", b"") for message in messages), b"synt")
        self.assertFalse(any(message["type"] == "http.response.pathsend" for message in messages))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.store.delete_artifact(artifact.artifact_id)

    def test_http_cancel_send_failure_and_discard_release_exactly_once(self):
        artifact = self.save()
        scope = {"type": "http", "method": "GET", "headers": []}
        async def receive():
            return {"type": "http.disconnect"}
        for failure in (RuntimeError("send failed"), asyncio.CancelledError()):
            response = ArtifactFileResponse(self.store, artifact.artifact_id)
            async def send(_message, failure=failure):
                raise failure
            with self.assertRaises(type(failure)):
                asyncio.run(response(scope, receive, send))
            self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        response = ArtifactFileResponse(self.store, artifact.artifact_id)
        del response
        gc.collect()
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_http_active_body_is_protected_and_body_cancellation_releases(self):
        artifact = self.save()
        response = ArtifactFileResponse(self.store, artifact.artifact_id)
        async def receive():
            return {"type": "http.request", "body": b""}
        async def send(message):
            if message["type"] == "http.response.body":
                with self.assertRaises(GatewayError) as error:
                    self.store.delete_artifact(artifact.artifact_id)
                self.assertEqual(error.exception.error_code, "artifact_in_use")
                raise asyncio.CancelledError()
        with self.assertRaises(asyncio.CancelledError):
            asyncio.run(response({"type": "http", "method": "GET", "headers": []}, receive, send))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
