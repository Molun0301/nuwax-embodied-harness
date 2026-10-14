#!/usr/bin/env python3
"""Offline producer/queue ownership regressions; only temporary synthetic artifacts."""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.artifacts import ArtifactKind, ArtifactRetentionPolicy
from contracts.base import utc_now
from contracts.image import ImageEncoding, ImageFrame
from contracts.perception import Observation
from contracts.runtime_views import PerceptionContext, SceneSummary
from core import StateNamespace, StateStore
from core.errors import GatewayError
from services.artifact_service import ArtifactService
from services.artifact_store import ArtifactHold, LocalArtifactStore
from services.observation_service import ObservationService
from services.perception.base import DetectionBundle, DetectorBackendSpec, DisabledSegmentationBackend
from services.perception.service import PerceptionService
from services.perception.video_runtime import PerceptionVideoRuntime


class Detector:
    default_backend_name = "synthetic"
    nms_iou_threshold = 0.5

    def __init__(self):
        self.calls = 0

    def detect(self, *_args, **_kwargs):
        self.calls += 1
        return DetectionBundle(), DetectorBackendSpec(name="synthetic", backend_kind="synthetic")


class BlockingSegmentation(DisabledSegmentationBackend):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.resume = threading.Event()

    def segment(self, *args, **kwargs):
        self.entered.set()
        if not self.resume.wait(5):
            raise AssertionError("synthetic segmentation timed out")
        return super().segment(*args, **kwargs)


class Memory:
    def __init__(self):
        self.library = "one"
        self.entered = threading.Event()
        self.resume = threading.Event()
        self.calls = []

    def is_enabled(self):
        return True

    def get_active_library_name(self):
        return self.library

    def remember_current_scene(self, **kwargs):
        self.calls.append(kwargs["perception_context"])
        self.entered.set()
        if not self.resume.wait(5):
            raise AssertionError("synthetic memory timed out")


class ProducerChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-producer-owners-")
        self.addCleanup(self.temp.cleanup)
        self.store = LocalArtifactStore(self.temp.name, "http://synthetic.invalid")
        self.artifacts = ArtifactService(self.store, retention_policy=ArtifactRetentionPolicy())
        self.addCleanup(self.artifacts.close)
        self.state = StateStore()
        self.detector = Detector()
        self.frame = ImageFrame(camera_id="front_camera", frame_id="camera", width_px=2, height_px=2,
                                encoding=ImageEncoding.JPEG, data=b"synthetic")
        self.provider = SimpleNamespace(provider_name="synthetic", provider_version="1", is_available=lambda: True,
                                        capture_image=lambda _camera: self.frame, get_camera_info=lambda _camera: None)
        self.source = SimpleNamespace(get_latest_image=lambda _camera: self.frame,
                                      status_snapshot=lambda: {"available": True})
        self.services = []
        self.threads = []
        self.addCleanup(self.finish)

    def finish(self):
        for thread, resume, _outcome in self.threads:
            if resume is not None:
                resume.set()
            thread.join(5)
            self.assertFalse(thread.is_alive())
        for service in reversed(self.services):
            self.assertTrue(service.close(timeout=1))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def observe(self, limit=2):
        service = ObservationService(provider_owner=SimpleNamespace(providers=self.provider),
                                     artifact_service=self.artifacts, state_store=self.state, history_limit=limit)
        self.services.append(service)
        return service

    def perceive(self, *, limit=1, segmentation=None):
        service = PerceptionService(provider_owner=None, artifact_service=self.artifacts, state_store=self.state,
                                    detector_pipeline=self.detector, video_frame_source=self.source,
                                    history_limit=limit, segmentation_backend=segmentation)
        self.services.append(service)
        return service

    def video(self, perception=None):
        memory = Memory()
        runtime = PerceptionVideoRuntime(perception_service=perception or self.perceive(), state_store=self.state,
                                         memory_service=memory, enabled=True, auto_start=False, camera_id="front_camera",
                                         interval_sec=0.05, failure_backoff_sec=0.1)
        self.services.append(runtime)
        self.addCleanup(memory.resume.set)
        return runtime, memory

    def save(self):
        return self.store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"synthetic")

    def context(self, artifact):
        return PerceptionContext(camera_id="front_camera",
                                 observation=Observation(observation_id="obs_front_camera_20261001T000000Z_12345678",
                                                         frame_id="camera", summary="synthetic",
                                                         artifact_ids=[artifact.artifact_id]),
                                 scene_summary=SceneSummary(headline="synthetic"), image_artifact=artifact,
                                 pipeline_name="synthetic", detector_backend="synthetic", tracker_backend="synthetic")

    def spawn(self, fn, *, resume=None):
        outcome = {}

        def run():
            try:
                outcome["result"] = fn()
            except BaseException as exc:  # noqa: BLE001 - report worker failures to the test thread
                outcome["error"] = exc

        thread = threading.Thread(target=run)
        self.threads.append((thread, resume, outcome))
        thread.start()
        return thread, outcome

    def join(self, thread, outcome):
        thread.join(5)
        self.assertFalse(thread.is_alive())
        if "error" in outcome:
            raise outcome["error"]
        return outcome.get("result")

    def protected(self, artifact):
        with self.assertRaises(GatewayError) as caught:
            self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(caught.exception.error_code, "artifact_in_use")

    def schedule(self, runtime, context, **kwargs):
        return runtime._schedule_memory_write(context=context, current_pose=kwargs.pop("current_pose", None), reason="synthetic", now=utc_now(),
                                              force=True, allow_memory_write=True, **kwargs)

    def test_observation_save_publication_gap_is_owned(self):
        service = self.observe()
        entered, resume = threading.Event(), threading.Event()
        original = self.state.read_latest
        saved = []
        original_save = self.artifacts.save_image_frame

        def save(*args, **kwargs):
            result = original_save(*args, **kwargs)
            saved.append(result)
            return result

        def read(*args):
            entered.set()
            self.assertTrue(resume.wait(5))
            return original(*args)

        with patch.object(self.artifacts, "save_image_frame", side_effect=save), patch.object(self.state, "read_latest", side_effect=read):
            thread, outcome = self.spawn(service.capture_observation, resume=resume)
            self.assertTrue(entered.wait(2))
            self.protected(saved[0])
            self.assertEqual(self.store.get_holder_status()["holder_count"], 1)
            resume.set()
            self.join(thread, outcome)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)

    def test_observation_bounded_slots_and_context_reader(self):
        service = self.observe(limit=1)
        first = service.capture_observation()
        context, hold = service.acquire_latest_observation(owner="reader")
        self.assertIs(context, first)
        second = service.capture_observation()
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)
        self.protected(first.image_artifact)
        hold.release()
        self.store.delete_artifact(first.image_artifact.artifact_id)
        self.protected(second.image_artifact)
        self.assertTrue(service.close())
        self.assertIsNone(self.state.read(StateNamespace.OBSERVATION, "front_camera").value)

    def test_observation_close_cancels_late_publication_and_retains_active_save(self):
        service = self.observe()
        entered, resume = threading.Event(), threading.Event()
        saved = []
        original = self.artifacts.save_image_frame

        def save(*args, **kwargs):
            artifact = original(*args, **kwargs)
            saved.append(artifact)
            entered.set()
            self.assertTrue(resume.wait(5))
            return artifact

        with patch.object(self.artifacts, "save_image_frame", side_effect=save):
            thread, outcome = self.spawn(service.capture_observation, resume=resume)
            self.assertTrue(entered.wait(2))
            self.assertFalse(service.close(timeout=0))
            self.protected(saved[0])
            resume.set()
            thread.join(5)
        self.assertEqual(outcome["error"].error_code, "observation_closed")
        self.assertTrue(service.close())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_perception_saved_artifact_protected_while_waiting_for_pipeline(self):
        service = self.perceive()
        saved, entered = [], threading.Event()
        original = self.artifacts.save_image_frame

        def save(*args, **kwargs):
            result = original(*args, **kwargs)
            saved.append(result)
            entered.set()
            return result

        with service._full_pipeline_lock, patch.object(self.artifacts, "save_image_frame", side_effect=save):
            thread, outcome = self.spawn(service.perceive_current_scene)
            self.assertTrue(entered.wait(2))
            self.protected(saved[0])
        self.join(thread, outcome)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)

    def test_existing_artifact_acquired_before_model_lock(self):
        service = self.perceive()
        artifact = self.save()
        entered = threading.Event()
        original = self.artifacts.acquire_artifacts

        def acquire(*args, **kwargs):
            self.assertFalse(service._pipeline_lock.locked())
            result = original(*args, **kwargs)
            entered.set()
            return result

        with service._full_pipeline_lock, patch.object(self.artifacts, "acquire_artifacts", side_effect=acquire):
            thread, outcome = self.spawn(lambda: service.process_image(self.frame, image_artifact=artifact))
            self.assertTrue(entered.wait(2))
            self.protected(artifact)
            self.assertEqual(self.detector.calls, 0)
        self.join(thread, outcome)

    def test_missing_existing_artifact_rejected_before_model(self):
        service = self.perceive()
        artifact = self.save()
        self.store.delete_artifact(artifact.artifact_id)
        with self.assertRaises(GatewayError) as caught:
            service.process_image(self.frame, image_artifact=artifact)
        self.assertEqual(caught.exception.error_code, "artifact_publication_missing")
        self.assertEqual(self.detector.calls, 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_unknown_external_artifact_remains_unresolved_compatible(self):
        service = self.perceive()
        with tempfile.TemporaryDirectory(prefix="neh-external-") as other_root:
            other = LocalArtifactStore(other_root, "http://external.invalid")
            artifact = other.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"external")
            service.process_image(self.frame, image_artifact=artifact)
        self.assertNotIn(artifact.artifact_id, self.store._state.local_pins)
        self.assertIn(artifact.artifact_id, self.store._state.pins)

    def test_empty_fast_and_full_frames_never_query_artifacts(self):
        service = self.perceive()
        with patch.object(self.artifacts, "publication_scope", side_effect=AssertionError("empty scope")), \
             patch.object(self.artifacts, "acquire_artifacts", side_effect=AssertionError("empty lookup")), \
             patch.object(self.artifacts, "hold_proven_artifacts", side_effect=AssertionError("empty proof")), \
             patch.object(self.store, "_find_meta_path", side_effect=AssertionError("empty filesystem")):
            fast = service.process_image_fast(self.frame)
            self.assertIsNone(service.acquire_context_artifacts(fast, owner="empty"))
            service.process_image(self.frame)
            service.perceive_current_scene(store_artifact=False)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_full_slot_survives_fast_latest_and_history_eviction(self):
        service = self.perceive(limit=1)
        first = service.perceive_current_scene()
        self.frame = self.frame.model_copy(update={"timestamp": utc_now()})
        service.process_image_fast(self.frame)
        self.assertIs(service.get_latest_full_perception(), first)
        self.assertEqual(self.store._state.pins[first.image_artifact.artifact_id], 1)
        second = service.perceive_current_scene()
        self.assertNotIn(first.image_artifact.artifact_id, self.store._state.pins)
        self.assertEqual(self.store._state.pins[second.image_artifact.artifact_id], 3)

    def test_slow_full_model_allows_fast_context_and_keeps_older_full_owner(self):
        segmentation = BlockingSegmentation()
        service = self.perceive(segmentation=segmentation)
        artifact = self.save()
        thread, outcome = self.spawn(lambda: service.process_image(self.frame, image_artifact=artifact), resume=segmentation.resume)
        self.assertTrue(segmentation.entered.wait(2))
        newer = self.frame.model_copy(update={"timestamp": utc_now()})
        fast = service.process_image_fast(newer)
        self.protected(artifact)
        segmentation.resume.set()
        full = self.join(thread, outcome)
        self.assertIs(service.get_latest_perception(), fast)
        self.assertIs(service.get_latest_full_perception(), full)
        self.assertEqual(self.store._state.pins[artifact.artifact_id], 2)

    def test_perception_close_timeout_retains_slow_operation_and_blocks_late_publish(self):
        segmentation = BlockingSegmentation()
        service = self.perceive(segmentation=segmentation)
        artifact = self.save()
        thread, outcome = self.spawn(lambda: service.process_image(self.frame, image_artifact=artifact), resume=segmentation.resume)
        self.assertTrue(segmentation.entered.wait(2))
        self.assertFalse(service.close(timeout=0))
        self.assertEqual(service.get_lifecycle_status()["active_operations"], 1)
        self.protected(artifact)
        with self.assertRaises(GatewayError):
            service.process_image_fast(self.frame)
        segmentation.resume.set()
        thread.join(5)
        self.assertEqual(outcome["error"].error_code, "perception_closed")
        self.assertTrue(service.close())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_independent_reader_and_model_copy_do_not_share_owner(self):
        service = self.perceive()
        context = service.perceive_current_scene()
        _, first = service.acquire_latest_perception(owner="reader", full=True)
        copied = context.model_copy(deep=True)
        second = service.acquire_context_artifacts(copied, owner="copy_reader")
        self.assertIsNot(first, second)
        self.assertTrue(service.close())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)
        first.release()
        self.protected(context.image_artifact)
        second.release()
        self.store.delete_artifact(context.image_artifact.artifact_id)

    def test_queue_pending_replacement_releases_exact_old_owner(self):
        runtime, memory = self.video()
        first, second, third = [self.context(self.save()) for _ in range(3)]
        self.assertTrue(self.schedule(runtime, first))
        self.assertTrue(memory.entered.wait(2))
        self.assertTrue(self.schedule(runtime, second))
        self.assertTrue(self.schedule(runtime, third))
        self.assertNotIn(second.image_artifact.artifact_id, self.store._state.pins)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)
        self.protected(first.image_artifact)
        self.protected(third.image_artifact)
        memory.resume.set()
        runtime._memory_write_thread.join(5)
        self.assertEqual(memory.calls, [first, third])
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_queue_library_change_releases_pending_without_write(self):
        runtime, memory = self.video()
        first, second = [self.context(self.save()) for _ in range(2)]
        self.schedule(runtime, first)
        self.assertTrue(memory.entered.wait(2))
        self.schedule(runtime, second)
        memory.library = "two"
        memory.resume.set()
        runtime._memory_write_thread.join(5)
        self.assertEqual(memory.calls, [first])
        self.assertEqual(runtime._last_memory_write_error, "memory_library_changed")
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_queue_stop_timeout_keeps_inflight_and_releases_pending(self):
        runtime, memory = self.video()
        first, second = [self.context(self.save()) for _ in range(2)]
        self.schedule(runtime, first)
        self.assertTrue(memory.entered.wait(2))
        self.schedule(runtime, second)
        thread = runtime._memory_write_thread
        runtime.stop(timeout=0)
        self.assertIs(runtime._memory_write_thread, thread)
        self.assertTrue(thread.is_alive())
        self.assertEqual(runtime.get_lifecycle_status()["inflight_artifact_count"], 1)
        self.assertEqual(runtime.get_lifecycle_status()["pending_artifact_count"], 0)
        self.assertNotIn(second.image_artifact.artifact_id, self.store._state.pins)
        self.protected(first.image_artifact)
        self.assertFalse(runtime.close(timeout=0))
        memory.resume.set()
        thread.join(5)
        self.assertTrue(runtime.close())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_queue_thread_start_failure_releases_owner(self):
        runtime, _memory = self.video()
        context = self.context(self.save())
        with patch("services.perception.video_runtime.threading.Thread.start", side_effect=RuntimeError("start failed")), \
             self.assertRaisesRegex(RuntimeError, "start failed"):
            self.schedule(runtime, context)
        self.assertFalse(runtime.get_lifecycle_status()["memory_write_in_flight"])
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_queue_rejection_after_acquire_releases_owner(self):
        service = self.perceive()
        runtime, _memory = self.video(service)
        context = self.context(self.save())
        original = service.acquire_context_artifacts

        def acquire(*args, **kwargs):
            hold = original(*args, **kwargs)
            runtime._stop_event.set()
            return hold

        with patch.object(service, "acquire_context_artifacts", side_effect=acquire):
            self.assertFalse(self.schedule(runtime, context))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_queue_empty_fast_context_never_acquires_artifacts(self):
        service = self.perceive()
        runtime, memory = self.video(service)
        context = service.process_image_fast(self.frame)
        memory.resume.set()
        with patch.object(service, "acquire_context_artifacts", side_effect=AssertionError("empty queue lookup")):
            self.assertTrue(self.schedule(runtime, context))
            runtime._memory_write_thread.join(5)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_reusable_stop_keeps_perception_slots_owned(self):
        service = self.perceive()
        context = service.perceive_current_scene()
        runtime, _memory = self.video(service)
        runtime.stop(timeout=0)
        self.assertIs(service.get_latest_perception(), context)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)
        self.protected(context.image_artifact)

    def test_video_terminal_close_fences_new_work(self):
        runtime, _memory = self.video()
        self.assertTrue(runtime.close(timeout=0))
        context = self.context(self.save())
        self.assertFalse(self.schedule(runtime, context))
        with self.assertRaises(GatewayError):
            runtime.start()
        with self.assertRaises(GatewayError):
            runtime.run_once()
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_observation_failed_slot_retain_releases_partial_owners(self):
        service = self.observe()
        original = ArtifactHold.retain

        def retain(hold, *, owner):
            if owner == "observation.history":
                raise RuntimeError("retain failed")
            return original(hold, owner=owner)

        with patch.object(ArtifactHold, "retain", retain), self.assertRaisesRegex(RuntimeError, "retain failed"):
            service.capture_observation()
        self.assertIsNone(service.get_latest_observation())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_state_publication_failure_keeps_previous_slots(self):
        for service, capture, getter in (
            (self.observe(), "capture_observation", "get_latest_observation"),
            (self.perceive(), "perceive_current_scene", "get_latest_perception"),
        ):
            old = getattr(service, capture)()
            old_count = self.store.get_holder_status()["holder_count"]
            with patch.object(self.state, "write", side_effect=RuntimeError("state unavailable")), \
                 self.assertRaisesRegex(RuntimeError, "state unavailable"):
                getattr(service, capture)()
            self.assertIs(getattr(service, getter)(), old)
            self.assertEqual(self.store.get_holder_status()["holder_count"], old_count)
            self.protected(old.image_artifact)

    def test_owned_context_handoff_uses_only_memory_proof(self):
        service = self.perceive()
        context = service.perceive_current_scene()
        with patch.object(self.artifacts, "acquire_artifacts", side_effect=AssertionError("owned context lookup")), \
             patch.object(self.store, "_find_meta_path", side_effect=AssertionError("owned file lookup")):
            first = service.acquire_context_artifacts(context, owner="queue")
            _, second = service.acquire_latest_perception(owner="reader")
        first.release()
        second.release()
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)

    def test_latest_reader_acquires_before_concurrent_replacement(self):
        service = self.observe(limit=1)
        old = service.capture_observation()
        entered, resume, writer_entered = threading.Event(), threading.Event(), threading.Event()
        original_retain = ArtifactHold.retain
        original_capture = self.provider.capture_image

        def retain(hold, *, owner):
            if owner == "reader":
                entered.set()
                self.assertTrue(resume.wait(5))
            return original_retain(hold, owner=owner)

        def capture(*args):
            writer_entered.set()
            return original_capture(*args)

        with patch.object(ArtifactHold, "retain", retain), patch.object(self.provider, "capture_image", side_effect=capture):
            reader, reader_outcome = self.spawn(lambda: service.acquire_latest_observation(owner="reader"), resume=resume)
            self.assertTrue(entered.wait(2))
            writer, writer_outcome = self.spawn(service.capture_observation)
            # The writer cannot even enter producer admission while the selection/retain fence is held.
            self.assertFalse(writer_entered.is_set())
            resume.set()
            context, hold = self.join(reader, reader_outcome)
            self.join(writer, writer_outcome)
        self.assertIs(context, old)
        self.assertEqual(self.store._state.pins[old.image_artifact.artifact_id], 1)
        hold.release()
        self.assertNotIn(old.image_artifact.artifact_id, self.store._state.pins)

    def test_perception_close_cancels_waiting_pipeline_before_detector(self):
        service = self.perceive()
        artifact = self.save()
        acquired = threading.Event()
        original = self.artifacts.acquire_artifacts

        def acquire(*args, **kwargs):
            hold = original(*args, **kwargs)
            acquired.set()
            return hold

        with service._full_pipeline_lock, patch.object(self.artifacts, "acquire_artifacts", side_effect=acquire):
            thread, outcome = self.spawn(lambda: service.process_image(self.frame, image_artifact=artifact))
            self.assertTrue(acquired.wait(2))
            self.assertFalse(service.close(timeout=0))
            self.protected(artifact)
        thread.join(5)
        self.assertEqual(outcome["error"].error_code, "perception_closed")
        self.assertEqual(self.detector.calls, 0)
        self.assertTrue(service.close())

    def test_queue_preserves_keyframe_over_continuous_candidate(self):
        runtime, memory = self.video()
        first, keyframe, continuous = [self.context(self.save()) for _ in range(3)]
        self.schedule(runtime, first)
        self.assertTrue(memory.entered.wait(2))
        self.schedule(runtime, keyframe)
        self.assertFalse(self.schedule(runtime, continuous, continuous=True, current_pose=object()))
        self.assertNotIn(continuous.image_artifact.artifact_id, self.store._state.pins)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 2)
        memory.resume.set()
        runtime._memory_write_thread.join(5)
        self.assertEqual(memory.calls, [first, keyframe])

    def test_queue_memory_failure_releases_owner_once(self):
        runtime, memory = self.video()
        context = self.context(self.save())
        releases = []
        original = ArtifactHold.release

        def release(hold, **kwargs):
            if hold.owner == "perception.video_queue":
                releases.append(hold)
            return original(hold, **kwargs)

        with patch.object(memory, "remember_current_scene", side_effect=RuntimeError("memory failed")), \
             patch.object(ArtifactHold, "release", release):
            self.assertTrue(self.schedule(runtime, context))
            runtime._memory_write_thread.join(5)
        self.assertEqual(len(releases), 1)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_queue_acquisition_race_keeps_close_incomplete(self):
        service = self.perceive()
        runtime, _memory = self.video(service)
        context = self.context(self.save())
        entered, resume = threading.Event(), threading.Event()
        original = service.acquire_context_artifacts

        def acquire(*args, **kwargs):
            hold = original(*args, **kwargs)
            entered.set()
            self.assertTrue(resume.wait(5))
            return hold

        with patch.object(service, "acquire_context_artifacts", side_effect=acquire):
            thread, outcome = self.spawn(lambda: self.schedule(runtime, context), resume=resume)
            self.assertTrue(entered.wait(2))
            self.assertFalse(runtime.close(timeout=0))
            self.assertEqual(runtime.get_lifecycle_status()["active_operations"], 1)
            self.protected(context.image_artifact)
            resume.set()
            self.assertFalse(self.join(thread, outcome))
        self.assertTrue(runtime.close())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_video_manual_operation_blocks_terminal_close(self):
        runtime, _memory = self.video()
        entered, resume = threading.Event(), threading.Event()

        def run(**kwargs):
            entered.set()
            self.assertTrue(resume.wait(5))
            return "finished"

        with patch.object(runtime, "_run_once_locked", side_effect=run):
            thread, outcome = self.spawn(runtime.run_once, resume=resume)
            self.assertTrue(entered.wait(2))
            self.assertFalse(runtime.close(timeout=0))
            self.assertEqual(runtime.get_lifecycle_status()["active_operations"], 1)
            resume.set()
            self.assertEqual(self.join(thread, outcome), "finished")
        self.assertTrue(runtime.close())

    def test_queue_replacement_stays_bounded_and_restart_waits_for_live_writer(self):
        runtime, memory = self.video()
        self.schedule(runtime, self.context(self.save()))
        self.assertTrue(memory.entered.wait(2))
        worker = runtime._memory_write_thread
        for _ in range(30):
            self.assertTrue(self.schedule(runtime, self.context(self.save())))
            self.assertEqual(self.store.get_holder_status()["holder_count"], 2)
            self.assertIs(runtime._memory_write_thread, worker)
        runtime.stop(timeout=0)
        self.assertFalse(runtime.start().running)
        self.assertTrue(runtime._stop_event.is_set())
        self.assertIs(runtime._memory_write_thread, worker)
        memory.resume.set()
        worker.join(5)
        with patch.object(runtime, "_run_loop", return_value=None):
            runtime.start()
            runtime._thread.join(5)
        self.assertFalse(runtime._stop_event.is_set())
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
