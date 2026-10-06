#!/usr/bin/env python3
"""Offline inference ownership: deterministic fake workers plus real spawn/SHM fake models."""

from __future__ import annotations

import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np

from contracts.image import ImageEncoding, ImageFrame
from core import StateStore
from services.perception.detectors import DetectorPipeline, YoloWorldDetectorBackend
from services.perception.inference_owner import InferenceProcessOwner
from services.perception.service import PerceptionService


class FakeWorker:
    def __init__(self, *, boot=True, startup_gate=None, predict_gate=None, stop_gate=None):
        self.boot, self.startup_gate, self.predict_gate, self.stop_gate = boot, startup_gate, predict_gate, stop_gate
        self.started, self.predicted, self.stopping = threading.Event(), threading.Event(), threading.Event()
        self.is_ready = False
        self.has_pending_resources = False
        self.failure_reason = None
        self.last_inference_ms = 1.0
        self.fail_prediction = False
        self.thresholds = {}
        self.stop_calls = 0

    def start_nonblocking(self):
        self.has_pending_resources = True
        self.started.set()
        return True

    def wait_until_ready(self, *, cancel_event):
        if self.startup_gate is not None:
            while not self.startup_gate.wait(0.001):
                if cancel_event.is_set():
                    return False
        self.is_ready = self.boot
        if not self.boot:
            self.failure_reason = "synthetic_boot_failure"
        return self.boot

    def predict(self, image, **kwargs):
        self.predicted.set()
        if self.predict_gate is not None:
            assert self.predict_gate.wait(3), "synthetic prediction gate timed out"
        if self.fail_prediction:
            self.failure_reason = "synthetic_prediction_failure"
            self.is_ready = False
            return None
        return (np.array([[int(image[0, 0, 0]), 0, 1, 1]]), np.array([0.8]), np.array([0]), {0: "cup"}, None)

    def update_thresholds(self, **kwargs):
        self.thresholds.update(kwargs)

    def stop(self):
        self.stopping.set()
        self.stop_calls += 1
        if self.stop_gate is not None:
            assert self.stop_gate.wait(3), "synthetic stop gate timed out"
        self.is_ready = False
        self.has_pending_resources = False


class RecoveryChecks(unittest.TestCase):
    def setUp(self):
        self.owners, self.gates, self.threads = [], [], []
        self.image = np.ones((2, 2, 3), np.uint8)

    def tearDown(self):
        for gate in self.gates:
            gate.set()
        for thread in self.threads:
            thread.join(3)
            self.assertFalse(thread.is_alive())
        for owner in self.owners:
            self.assertTrue(owner.close(3), owner.status())

    def gate(self):
        event = threading.Event()
        self.gates.append(event)
        return event

    def owner(self, workers, backoff=0.03):
        self.created, self.kwargs, self.creation_times = [], [], []

        def factory(source, **kwargs):
            worker = workers[len(self.created)]
            self.created.append(worker)
            self.kwargs.append(kwargs)
            self.creation_times.append(time.monotonic())
            return worker

        owner = InferenceProcessOwner("synthetic", retry_backoff_sec=backoff, worker_factory=factory)
        self.owners.append(owner)
        self.assertTrue(owner.start_nonblocking())
        return owner

    def eventually(self, predicate):
        end = time.monotonic() + 3
        while not predicate():
            self.assertLess(time.monotonic(), end, "synthetic state transition timed out")
            time.sleep(0.001)

    def spawn(self, call):
        thread = threading.Thread(target=call)
        self.threads.append(thread)
        thread.start()
        return thread

    def test_repeated_start_failure_recovers_on_same_owner_with_backoff(self):
        owner = self.owner([FakeWorker(boot=False), FakeWorker(boot=False), FakeWorker()])
        lifecycle = owner._thread
        self.eventually(lambda: owner.status()["state"] == "ready")
        self.assertEqual(owner.status()["start_attempts"], 3)
        self.assertIs(owner._thread, lifecycle)
        self.assertTrue(all(b - a >= 0.025 for a, b in zip(self.creation_times, self.creation_times[1:])))
        self.assertTrue(all(not w.has_pending_resources for w in self.created[:-1]))
        self.assertIsNotNone(owner.predict(self.image))

    def test_concurrent_callers_cannot_create_extra_lifecycle_or_skip_backoff(self):
        owner = self.owner([FakeWorker(boot=False), FakeWorker()], backoff=0.2)
        self.eventually(lambda: owner.status()["state"] == "backoff")
        lifecycle = owner._thread
        calls = [self.spawn(lambda: (owner.start_nonblocking(), owner.predict(self.image))) for _ in range(8)]
        for thread in calls:
            thread.join(1)
            self.assertFalse(thread.is_alive())
        self.assertEqual(len(self.created), 1)
        self.assertIs(owner._thread, lifecycle)

    def test_prediction_failure_preserves_process_mode_and_never_calls_local_model(self):
        a, b = FakeWorker(), FakeWorker()
        owner = self.owner([a, b])
        self.eventually(lambda: owner.status()["state"] == "ready")
        backend = YoloWorldDetectorBackend(weights="synthetic.engine")
        backend._inference_worker = owner
        a.fail_prediction = True
        with patch.object(backend, "_get_model", side_effect=AssertionError("implicit local model")):
            with self.assertRaises(RuntimeError):
                backend._predict(self.image)
            self.assertTrue(backend._use_worker_process)
            self.eventually(lambda: owner.status()["start_attempts"] == 2 and owner.status()["state"] == "ready")
            self.assertEqual(len(backend._predict(self.image)), 1)
        self.assertEqual(backend._last_inference_execution, "worker_process")

    def test_idle_worker_crash_recovers_without_prediction_request(self):
        a, b = FakeWorker(), FakeWorker()
        owner = self.owner([a, b])
        self.eventually(lambda: owner.status()["state"] == "ready")
        a.is_ready = False
        self.eventually(lambda: owner.status()["start_attempts"] == 2 and owner.status()["state"] == "ready")
        self.assertFalse(a.has_pending_resources)

    def test_late_old_generation_success_cannot_replace_new_generation_state(self):
        gate = self.gate()
        a, b = FakeWorker(predict_gate=gate), FakeWorker()
        owner = self.owner([a, b])
        self.eventually(lambda: owner.status()["state"] == "ready")
        result = []
        thread = self.spawn(lambda: result.append(owner.predict(self.image)))
        self.assertTrue(a.predicted.wait(1))
        a.is_ready = False
        self.eventually(lambda: owner.status()["start_attempts"] == 2 and owner.status()["state"] == "ready")
        gate.set()
        thread.join(1)
        self.assertEqual(result, [None])
        self.assertIs(owner._worker, b)
        self.assertEqual(owner.status()["state"], "ready")

    def test_close_during_startup_is_terminal_and_reaps(self):
        worker = FakeWorker(startup_gate=self.gate())
        owner = self.owner([worker])
        self.assertTrue(worker.started.wait(1))
        self.assertTrue(owner.close(1))
        self.assertFalse(owner.start_nonblocking())
        self.assertFalse(worker.has_pending_resources)

    def test_large_retry_backoff_does_not_delay_ready_worker_close(self):
        worker = FakeWorker()
        owner = self.owner([worker], backoff=10)
        self.eventually(lambda: owner.status()["state"] == "ready")
        self.assertTrue(owner.close(0.5), owner.status())
        self.assertTrue(worker.stopping.is_set())
        self.assertFalse(worker.has_pending_resources)

    def test_known_prediction_failure_reaps_without_waiting_large_retry_backoff(self):
        worker = FakeWorker()
        owner = self.owner([worker], backoff=10)
        self.eventually(lambda: owner.status()["state"] == "ready")
        worker.fail_prediction = True
        self.assertIsNone(owner.predict(self.image))
        self.assertTrue(worker.stopping.wait(0.5))
        self.eventually(lambda: owner.status()["state"] == "backoff")
        self.assertEqual(len(self.created), 1)
        self.assertTrue(owner.close(0.5))

    def test_repeated_close_preserves_completed_terminal_status(self):
        owner = self.owner([FakeWorker()])
        self.eventually(lambda: owner.status()["state"] == "ready")
        for _ in range(3):
            self.assertTrue(owner.close(1))
            self.assertEqual(owner.status()["state"], "stopped")
            self.assertFalse(owner.status()["lifecycle_alive"])
            self.assertFalse(owner.status()["worker_owned"])
            self.assertFalse(owner.start_nonblocking())

    def test_close_during_backoff_does_not_restart(self):
        owner = self.owner([FakeWorker(boot=False)], backoff=1)
        self.eventually(lambda: owner.status()["state"] == "backoff")
        self.assertTrue(owner.close(1))
        self.assertEqual(len(self.created), 1)

    def test_close_inflight_retains_resources_until_reaped_and_rejects_late_result(self):
        gate = self.gate()
        worker = FakeWorker(predict_gate=gate, stop_gate=gate)
        owner = self.owner([worker])
        self.eventually(lambda: owner.status()["state"] == "ready")
        result = []
        thread = self.spawn(lambda: result.append(owner.predict(self.image)))
        self.assertTrue(worker.predicted.wait(1))
        self.assertFalse(owner.close(0))
        self.assertTrue(worker.has_pending_resources)
        self.assertIsNone(owner.predict(self.image))
        gate.set()
        thread.join(1)
        self.assertTrue(owner.close(1))
        self.assertEqual(result, [None])

    def test_unreaped_worker_is_retained_and_no_replacement_can_start(self):
        class RetainedWorker(FakeWorker):
            allow_reap = False

            def stop(self):
                self.stop_calls += 1
                self.stopping.set()
                if self.allow_reap:
                    super().stop()

        worker = RetainedWorker(boot=False)
        owner = self.owner([worker, FakeWorker()], backoff=0.1)
        self.assertTrue(worker.stopping.wait(1))
        self.assertFalse(owner.close(0))
        self.assertTrue(owner.status()["worker_owned"])
        self.assertEqual(len(self.created), 1)
        worker.allow_reap = True
        self.assertTrue(owner.close(1))
        self.assertEqual(len(self.created), 1)

    def test_thresholds_reach_current_and_next_worker(self):
        a, b = FakeWorker(), FakeWorker()
        owner = self.owner([a, b])
        self.eventually(lambda: owner.status()["state"] == "ready")
        owner.update_thresholds(confidence_threshold=0.6, iou_threshold=0.4, max_detections=5)
        self.assertEqual(a.thresholds["confidence_threshold"], 0.6)
        a.is_ready = False
        self.eventually(lambda: owner.status()["start_attempts"] == 2 and owner.status()["state"] == "ready")
        self.assertEqual(self.kwargs[1]["confidence_threshold"], 0.6)

    def test_detection_metadata_exposes_transient_recovery_and_dynamic_limitation(self):
        owner = self.owner([FakeWorker(startup_gate=self.gate())])
        backend = YoloWorldDetectorBackend(weights="synthetic.engine")
        backend._inference_worker = owner
        frame = ImageFrame(
            camera_id="generated",
            frame_id="generated",
            width_px=2,
            height_px=2,
            encoding=ImageEncoding.BGR8,
            data=self.image.tobytes(),
        )
        with patch.object(backend, "_get_model", side_effect=AssertionError("local fallback")):
            bundle = backend.detect(frame)
        self.assertEqual(bundle.metadata["stage_status"], "not_ready")
        self.assertTrue(bundle.metadata["use_worker_process"])
        self.assertEqual(bundle.metadata["worker_recovery"]["state"], "starting")
        dynamic = YoloWorldDetectorBackend(weights="synthetic.pt")
        self.assertFalse(dynamic._worker_status_metadata()["supports_worker_process"])

    def test_explicit_in_process_mode_is_unchanged(self):
        backend = YoloWorldDetectorBackend(weights="synthetic.engine", use_worker_process=False)
        with patch.object(
            backend, "_get_model", return_value=SimpleNamespace(predict=lambda *a, **k: iter(["result"]))
        ):
            self.assertEqual(backend._predict(self.image), ["result"])
        self.assertEqual(backend._last_inference_execution, "in_process")

    def test_service_close_owns_pipeline_shutdown_and_retains_incomplete_owner(self):
        gate = self.gate()
        worker = FakeWorker(stop_gate=gate)
        owner = self.owner([worker])
        self.eventually(lambda: owner.status()["state"] == "ready")
        backend = YoloWorldDetectorBackend(weights="synthetic.engine")
        backend._inference_worker = owner
        pipeline = DetectorPipeline([backend], default_backend_name=backend.spec.name)
        service = PerceptionService(
            provider_owner=None,
            artifact_service=SimpleNamespace(),
            state_store=StateStore(),
            detector_pipeline=pipeline,
        )
        self.assertFalse(service.close(0))
        self.assertTrue(service.get_lifecycle_status()["closing"])
        self.assertFalse(service.get_lifecycle_status()["closed"])
        gate.set()
        self.assertTrue(service.close(2))
        self.assertFalse(worker.has_pending_resources)


_FAKE_MODEL = """import numpy as np, types, sys, time, os
from pathlib import Path
stub=types.ModuleType("services.perception.detectors")
stub._ensure_torchvision_nms_fallback=lambda:None
sys.modules["services.perception.detectors"]=stub
class YOLO:
 def __init__(self, source):
  self.mode=Path(source).read_text()
  if self.mode=="boot_error": raise ValueError("synthetic bootstrap failure")
  if self.mode=="slow_start": time.sleep(.3)
 def predict(self, image, **kwargs):
  if self.mode=="crash": os._exit(17)
  if self.mode=="slow": time.sleep(.3)
  boxes=types.SimpleNamespace(xyxy=np.array([[int(image[0,0,0]),0,1,1]]),conf=np.array([kwargs["conf"]]),cls=np.array([0]))
  yield types.SimpleNamespace(boxes=boxes,names={0:"generated"},masks=None)
"""


class RealProcessChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="neh-inference-recovery-")
        self.root = Path(self.temp.name)
        (self.root / "ultralytics.py").write_text(_FAKE_MODEL)
        self.model = self.root / "synthetic.engine"
        self.model.write_text("normal")
        sys.path.insert(0, self.temp.name)
        self.owner = InferenceProcessOwner(str(self.model), retry_backoff_sec=0.05)
        self.image = np.ones((2, 2, 3), np.uint8)
        self.assertTrue(self.owner.start_nonblocking())

    def tearDown(self):
        self.assertTrue(self.owner.close(8), self.owner.status())
        sys.path.remove(self.temp.name)
        self.temp.cleanup()

    def eventually(self, predicate):
        deadline = time.monotonic() + 8
        while not predicate():
            self.assertLess(time.monotonic(), deadline, self.owner.status())
            time.sleep(0.005)

    def ready(self):
        self.eventually(lambda: self.owner.status()["state"] == "ready")

    def test_spawn_shm_prediction_cleanup_and_close(self):
        self.ready()
        worker = self.owner._worker
        pid = worker._process.pid
        self.assertNotEqual(pid, os.getpid())
        self.assertEqual(worker._request_queue._maxsize, 2)
        result = self.owner.predict(self.image)
        self.assertEqual(result[0][0, 0], 1)
        name = worker._shm.name
        self.assertTrue(self.owner.close(5))
        self.assertFalse(worker.has_pending_resources)
        from multiprocessing.shared_memory import SharedMemory

        with self.assertRaises(FileNotFoundError):
            SharedMemory(name=name)

    def test_real_child_crash_recovers_without_parent_fallback(self):
        self.ready()
        first = self.owner._worker
        first._process.terminate()
        first._process.join(2)
        self.eventually(lambda: self.owner.status()["start_attempts"] >= 2 and self.owner.status()["state"] == "ready")
        self.assertIsNot(self.owner._worker, first)
        self.assertFalse(first.has_pending_resources)
        self.assertIsNotNone(self.owner.predict(self.image))

    def test_real_startup_failure_then_model_becomes_available(self):
        self.assertTrue(self.owner.close(5))
        self.model.write_text("boot_error")
        self.owner = InferenceProcessOwner(str(self.model), retry_backoff_sec=0.1)
        self.owner.start_nonblocking()
        self.eventually(lambda: self.owner.status()["state"] == "backoff")
        self.assertIn("synthetic bootstrap failure", self.owner.failure_reason)
        self.model.write_text("normal")
        self.ready()
        self.assertIsNotNone(self.owner.predict(self.image))

    def test_public_detector_recovers_asynchronously_and_only_uses_child_model(self):
        self.assertTrue(self.owner.close(5))
        backend = YoloWorldDetectorBackend(weights=str(self.model), worker_retry_backoff_sec=0.05)
        frame = ImageFrame(
            camera_id="generated",
            frame_id="generated",
            width_px=2,
            height_px=2,
            encoding=ImageEncoding.BGR8,
            data=np.zeros((2, 2, 3), np.uint8).tobytes(),
        )
        with patch.object(backend, "_get_model", side_effect=AssertionError("implicit parent model")):
            first = backend.detect(frame)
            self.owner = backend._inference_worker
            self.assertEqual(first.metadata["stage_status"], "not_ready")
            self.ready()
            result = backend.detect(frame)
        self.assertEqual(result.metadata["stage_status"], "completed")
        self.assertEqual(result.metadata["inference_execution"], "worker_process")
        self.assertTrue(result.metadata["use_worker_process"])
        self.assertEqual(result.metadata["worker_recovery"]["state"], "ready")

    def test_real_prediction_timeout_reaps_old_shm_before_restart(self):
        self.assertTrue(self.owner.close(5))
        self.model.write_text("slow")
        self.owner = InferenceProcessOwner(str(self.model), retry_backoff_sec=0.05)
        self.owner.start_nonblocking()
        self.ready()
        first = self.owner._worker
        self.assertIsNone(self.owner.predict(self.image, timeout_sec=0.03))
        self.model.write_text("normal")
        self.eventually(lambda: self.owner.status()["start_attempts"] >= 2 and self.owner.status()["state"] == "ready")
        self.assertFalse(first.has_pending_resources)
        self.assertIsNotNone(self.owner.predict(self.image))

    def test_real_close_during_startup_retains_owner_until_reaped(self):
        self.assertTrue(self.owner.close(5))
        self.model.write_text("slow_start")
        self.owner = InferenceProcessOwner(str(self.model), retry_backoff_sec=0.05)
        self.owner.start_nonblocking()
        self.eventually(lambda: self.owner._worker is not None and self.owner._worker._process is not None)
        self.owner.close(0)
        self.assertTrue(self.owner.close(5))
        self.assertFalse(self.owner.status()["worker_owned"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
