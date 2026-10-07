#!/usr/bin/env python3
"""Offline perception publication ordering, callback isolation and ownership checks."""

from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from contracts.artifacts import ArtifactRetentionPolicy
from contracts.image import ImageEncoding, ImageFrame
from core import StateStore, StreamBus
from core.errors import GatewayError
from services.artifact_service import ArtifactService
from services.artifact_store import LocalArtifactStore
from services.perception.base import DetectionBundle, DetectorBackendSpec, DisabledSegmentationBackend
from services.perception.service import PerceptionService


class Detector:
    default_backend_name = "synthetic"
    spec = DetectorBackendSpec(name="synthetic", backend_kind="synthetic")

    def __init__(self):
        self.calls = []
        self.thresholds = None

    def detect(self, image, *_args, **_kwargs):
        self.calls.append(image)
        return DetectionBundle(), self.spec

    def list_backends(self):
        return (self.spec,)

    def get_backend(self, _name):
        return self

    def update_thresholds(self, **kwargs):
        self.thresholds = kwargs


class CallbackInterrupt(BaseException):
    pass


class DispatchChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-perception-dispatch-")
        self.store = LocalArtifactStore(self.temp.name, "http://synthetic.invalid")
        self.artifacts = ArtifactService(self.store, retention_policy=ArtifactRetentionPolicy())
        self.bus = StreamBus()
        self.detector = Detector()
        self.service = PerceptionService(
            provider_owner=None, artifact_service=self.artifacts, state_store=StateStore(),
            detector_pipeline=self.detector, stream_bus=self.bus,
        )
        self.first = ImageFrame(
            camera_id="synthetic_camera", frame_id="camera", width_px=2, height_px=2,
            encoding=ImageEncoding.JPEG, data=b"synthetic", metadata={"source_sequence": 1},
        )
        self.second = self.first.model_copy(update={
            "timestamp": self.first.timestamp + timedelta(seconds=1), "metadata": {"source_sequence": 2},
        })
        self.gates, self.threads = [], []

    def tearDown(self):
        for gate in self.gates:
            gate.set()
        for thread, _outcome in self.threads:
            thread.join(3)
            self.assertFalse(thread.is_alive(), "perception callback deadlocked")
        self.assertTrue(self.service.close(timeout=1))
        self.bus.shutdown()
        self.artifacts.close()
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)
        self.temp.cleanup()

    def gate(self):
        event = threading.Event()
        self.gates.append(event)
        return event

    def spawn(self, call):
        outcome = {}

        def run():
            try:
                outcome["result"] = call()
            except BaseException as exc:  # noqa: BLE001 - Surface callback interruptions to the test owner.
                outcome["error"] = exc

        thread = threading.Thread(target=run, daemon=True)
        self.threads.append((thread, outcome))
        thread.start()
        return thread, outcome

    def join(self, worker):
        thread, outcome = worker
        thread.join(3)
        self.assertFalse(thread.is_alive())
        if "error" in outcome:
            raise outcome["error"]
        return outcome.get("result")

    def subscribe(self, callback):
        return self.bus.subscribe(callback, topics=("video.detections",))

    def assert_progress_while_callback_blocked(self, *, full, next_full=False):
        entered, release, reserved, dispatch_entered = self.gate(), self.gate(), self.gate(), self.gate()
        received, lock_states = [], []

        def callback(frame):
            received.append(frame)
            lock_states.append((self.service._pipeline_lock.locked(), self.service._full_pipeline_lock.locked()))
            if len(received) == 1:
                entered.set()
                if not release.wait(5):
                    raise AssertionError("synthetic callback gate timed out")

        original_prepare = self.bus.prepare_publication
        original_dispatch = self.bus.dispatch_prepared

        def prepare(frame):
            publication = original_prepare(frame)
            if frame.payload.get("source_sequence") == 2:
                reserved.set()
            return publication

        def dispatch(publication):
            if publication.frame.payload.get("source_sequence") == 2:
                dispatch_entered.set()
            return original_dispatch(publication)

        self.subscribe(callback)
        method = self.service.process_image if full else self.service.process_image_fast
        next_method = self.service.process_image if next_full else self.service.process_image_fast
        with patch.object(self.bus, "prepare_publication", side_effect=prepare), \
                patch.object(self.bus, "dispatch_prepared", side_effect=dispatch):
            first = self.spawn(lambda: method(self.first))
            self.assertTrue(entered.wait(2))
            second = self.spawn(lambda: next_method(self.second))
            tuning = self.spawn(lambda: self.service.update_detector_thresholds(SimpleNamespace(
                confidence_threshold=0.5, iou_threshold=0.4, max_detections=10,
            )))
            # A gated callback remains paused throughout this assertion. This
            # proves lock independence, rather than asserting wall-clock speed.
            self.assertTrue(reserved.wait(1), "next inference is blocked by a video.detections callback")
            self.assertTrue(dispatch_entered.wait(1))
            self.join(tuning)
            self.assertEqual(len(self.detector.calls), 2)
            self.assertEqual(self.detector.thresholds["max_detections"], 10)
            self.assertEqual(self.service.get_latest_perception().metadata["source_sequence"], 2)
            self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 2)
            self.assertTrue(first[0].is_alive())
            self.assertTrue(second[0].is_alive(), "synchronous callback completion must still be awaited")
            release.set()
            self.join(first)
            self.join(second)
        self.assertEqual([frame.payload["source_sequence"] for frame in received], [1, 2])
        self.assertLess(received[0].cursor, received[1].cursor)
        self.assertEqual(lock_states, [(False, False), (False, False)])

    def test_fast_callback_does_not_own_pipeline(self):
        self.assert_progress_while_callback_blocked(full=False)

    def test_full_callback_does_not_own_pipeline(self):
        self.assert_progress_while_callback_blocked(full=True)

    def test_full_callback_allows_next_full_model_stages(self):
        self.assert_progress_while_callback_blocked(full=True, next_full=True)

    def test_later_dispatcher_preserves_reserved_order_without_replay(self):
        entered, release = self.gate(), self.gate()
        received = []
        self.subscribe(received.append)
        original_dispatch = self.bus.dispatch_prepared

        def dispatch(publication):
            if publication.frame.payload["source_sequence"] == 1:
                entered.set()
                if not release.wait(5):
                    raise AssertionError("synthetic dispatch gate timed out")
            return original_dispatch(publication)

        with patch.object(self.bus, "dispatch_prepared", side_effect=dispatch):
            first = self.spawn(lambda: self.service.process_image_fast(self.first))
            self.assertTrue(entered.wait(2))
            second = self.join(self.spawn(lambda: self.service.process_image_fast(self.second)))
            self.assertTrue(first[0].is_alive())
            self.assertEqual([frame.payload["source_sequence"] for frame in received], [1, 2])
            self.assertLess(received[0].cursor, received[1].cursor)
            self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 1)
            self.assertIs(self.service.get_latest_perception(), second)
            release.set()
            self.join(first)
        self.assertEqual(len(received), 2)

    def test_same_thread_callback_can_reenter_fast_and_full_with_fifo_delivery(self):
        received, nested, callback_threads = [], [], []

        def callback(frame):
            received.append(frame)
            callback_threads.append(threading.get_ident())
            if len(received) == 1:
                nested.append(self.service.process_image_fast(self.second))
                nested.append(self.service.process_image(self.second))

        self.subscribe(callback)
        first = self.join(self.spawn(lambda: self.service.process_image(self.first)))
        self.assertEqual([frame.payload["observation_id"] for frame in received], [
            first.observation.observation_id, *(item.observation.observation_id for item in nested),
        ])
        self.assertEqual([frame.cursor for frame in received], sorted(frame.cursor for frame in received))
        self.assertEqual(len(set(callback_threads)), 1)
        self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 0)

    def test_fast_cache_does_not_repeat_preparation_or_callback(self):
        received = []
        self.subscribe(received.append)
        first = self.service.process_image_fast(self.first)
        with patch.object(self.bus, "prepare_publication", side_effect=AssertionError("duplicate publication")):
            self.assertIs(self.service.process_image_fast(self.first), first)
        self.assertEqual(len(received), 1)
        self.assertEqual(len(self.detector.calls), 1)

    def test_close_during_full_callback_retains_operation_and_artifacts(self):
        entered, release = self.gate(), self.gate()

        def callback(_frame):
            entered.set()
            if not release.wait(5):
                raise AssertionError("synthetic callback gate timed out")

        self.subscribe(callback)
        artifact = self.artifacts.save_image_frame(self.first)
        worker = self.spawn(lambda: self.service.process_image(self.first, image_artifact=artifact))
        self.assertTrue(entered.wait(2))
        self.assertFalse(self.service.close(timeout=0))
        status = self.service.get_lifecycle_status()
        self.assertEqual(status["active_operations"], 1)
        self.assertTrue(status["closing"])
        self.assertFalse(status["closed"])
        self.assertIsNotNone(self.service.get_latest_full_perception())
        with self.assertRaises(GatewayError) as caught:
            self.service.process_image_fast(self.second)
        self.assertEqual(caught.exception.error_code, "perception_closed")
        with self.assertRaises(GatewayError) as caught:
            self.store.delete_artifact(artifact.artifact_id)
        self.assertEqual(caught.exception.error_code, "artifact_in_use")
        self.assertEqual(self.store.get_holder_status()["holder_count"], 4)
        release.set()
        self.join(worker)
        self.assertTrue(self.service.close(timeout=1))
        self.assertEqual(self.store.get_holder_status()["holder_count"], 0)

    def test_callback_interruption_releases_operation_and_does_not_replay(self):
        received = []

        def callback(frame):
            received.append(frame)
            if len(received) == 1:
                raise CallbackInterrupt("synthetic callback interruption")

        self.subscribe(callback)
        artifact = self.artifacts.save_image_frame(self.first)
        with self.assertRaises(CallbackInterrupt):
            self.service.process_image(self.first, image_artifact=artifact)
        self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)
        self.assertFalse(self.service._pipeline_lock.locked())
        self.assertFalse(self.service._full_pipeline_lock.locked())
        self.service.process_image_fast(self.second)
        self.assertEqual([frame.payload["source_sequence"] for frame in received], [1, 2])

    def test_callback_exception_is_isolated_and_later_overlay_is_delivered(self):
        received = []

        def callback(frame):
            received.append(frame)
            if len(received) == 1:
                raise ValueError("synthetic callback failure")

        self.subscribe(callback)
        first = self.service.process_image_fast(self.first)
        second = self.service.process_image_fast(self.second)
        self.assertEqual([frame.payload["observation_id"] for frame in received], [
            first.observation.observation_id, second.observation.observation_id,
        ])
        self.assertIsNotNone(self.bus.latest_frame("diagnostics.stream_bus.subscription_failed"))
        self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 0)

    def test_prepare_failure_releases_locks_and_operation(self):
        with patch.object(self.bus, "prepare_publication", side_effect=RuntimeError("synthetic closed bus")), \
                self.assertRaisesRegex(RuntimeError, "synthetic closed bus"):
            self.service.process_image(self.first)
        self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 0)
        self.assertFalse(self.service._pipeline_lock.locked())
        self.assertFalse(self.service._full_pipeline_lock.locked())
        self.assertIsNotNone(self.service.process_image_fast(self.second))

    def test_stale_full_completion_cannot_replace_newer_fast_overlay(self):
        entered, release = self.gate(), self.gate()
        segmentation = DisabledSegmentationBackend()
        original_segment = segmentation.segment

        def segment(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise AssertionError("synthetic segmentation gate timed out")
            return original_segment(*args, **kwargs)

        segmentation.segment = segment
        self.service._segmentation_backend = segmentation
        received = []
        self.subscribe(received.append)
        worker = self.spawn(lambda: self.service.process_image(self.first))
        self.assertTrue(entered.wait(2))
        fast = self.service.process_image_fast(self.second)
        release.set()
        full = self.join(worker)
        self.assertIs(self.service.get_latest_perception(), fast)
        self.assertIs(self.service.get_latest_full_perception(), full)
        self.assertEqual([frame.payload["source_sequence"] for frame in received], [2])


if __name__ == "__main__":
    unittest.main(verbosity=2)
