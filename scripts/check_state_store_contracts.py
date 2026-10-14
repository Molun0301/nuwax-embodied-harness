#!/usr/bin/env python3
"""Offline latest-state/cache contracts; synthetic image bytes and temporary storage only."""
from __future__ import annotations

import sys
import tempfile
import threading
import unittest
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.state_store import StateNamespace, StateStore


class StateStoreContracts(unittest.TestCase):
    def setUp(self):
        self.store = StateStore()
        self.namespace = StateNamespace.ROBOT_STATE
        self.time = datetime(2026, 10, 4, tzinfo=timezone.utc)

    def write(self, key, second, value=None, namespace=None):
        return self.store.write(namespace or self.namespace, key, value if value is not None else key,
                                source="offline", updated_at=self.time + timedelta(seconds=second))

    def latest(self):
        latest = self.store.read_latest(self.namespace)
        entries = self.store.snapshot(self.namespace)
        self.assertTrue(any(latest is entry for entry in entries))
        self.assertEqual(latest.updated_at, max(entry.updated_at for entry in entries))
        return latest

    def test_empty_namespace(self):
        self.assertIsNone(self.store.read_latest(self.namespace))
        self.assertEqual(self.store.snapshot(self.namespace), ())

    def test_overwriting_latest_with_older_timestamp_selects_current_other_key(self):
        self.write("a", 2, "discarded")
        expected = self.write("b", 1)
        replacement = self.write("a", 0, "replacement")
        self.assertIs(self.store.read(self.namespace, "a"), replacement)
        self.assertIs(self.latest(), expected)

    def test_regressing_latest_still_newer_than_other_key(self):
        self.write("a", 3)
        self.write("b", 1)
        replacement = self.write("a", 2)
        self.assertIs(self.latest(), replacement)

    def test_single_entry_regression_returns_replacement(self):
        self.write("a", 3)
        self.assertIs(self.latest(), self.store.read(self.namespace, "a"))
        replacement = self.write("a", 1)
        self.assertIs(self.latest(), replacement)

    def test_out_of_order_nonlatest_does_not_replace_current_latest(self):
        expected = self.write("a", 5)
        self.write("b", 3)
        self.write("b", 1)
        self.write("c", 4)
        self.assertIs(self.latest(), expected)

    def test_equal_timestamp_replacement_and_promotion(self):
        self.write("a", 2)
        second = self.write("b", 2)
        self.assertIs(self.latest(), second)
        promoted = self.write("a", 2)
        self.assertIs(self.latest(), promoted)
        replacement = self.write("a", 2)
        self.assertIs(self.latest(), replacement)

    def test_latest_regression_to_equal_timestamp_prefers_new_write(self):
        self.write("b", 2)
        self.write("a", 3)
        replacement = self.write("a", 2)
        self.assertIs(self.latest(), replacement)

    def test_latest_regression_below_tied_peers_returns_a_current_maximum(self):
        first = self.write("b", 2)
        self.write("c", 2)
        self.write("a", 3)
        self.write("a", 1)
        # Once the former winner is replaced, either unchanged maximum has
        # the same timestamp; stable namespace insertion order is sufficient.
        self.assertIs(self.latest(), first)

    def test_namespaces_remain_independent(self):
        expected = self.write("a", 2)
        self.write("a", 10, namespace=StateNamespace.OBSERVATION)
        self.write("a", 0, namespace=StateNamespace.OBSERVATION)
        self.assertIs(self.latest(), expected)
        other = self.store.read_latest(StateNamespace.OBSERVATION)
        self.assertIs(other, self.store.read(StateNamespace.OBSERVATION, "a"))

    def test_concurrent_regressing_writers_preserve_actual_latest_entry(self):
        count = 4
        barrier = threading.Barrier(count)
        failures = []
        def writer(index):
            try:
                for generation in range(20):
                    self.write(str(index), 100 + generation + index)
                    barrier.wait(timeout=3)
                    self.write(str(index), index)
                    barrier.wait(timeout=3)
                    if index == 0:
                        self.assertEqual(self.latest().key, str(count - 1))
                    barrier.wait(timeout=3)
            except BaseException as error:  # noqa: BLE001 - relay worker failure
                failures.append(error)
                barrier.abort()
        threads = [threading.Thread(target=writer, args=(index,)) for index in range(count)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(5)
        self.assertFalse(any(thread.is_alive() for thread in threads))
        self.assertEqual(failures, [])

    def test_observation_summary_uses_current_robot_state_after_regression(self):
        from contracts.artifacts import ArtifactRetentionPolicy
        from contracts.image import ImageEncoding, ImageFrame
        from services.artifact_service import ArtifactService
        from services.artifact_store import LocalArtifactStore
        from services.observation_service import ObservationService

        self.write("a", 2, NS(mode=NS(value="discarded-mode")))
        self.write("b", 1, NS(mode=NS(value="current-mode")))
        self.write("a", 0, NS(mode=NS(value="older-mode")))
        frame = ImageFrame(camera_id="front_camera", frame_id="camera", width_px=2, height_px=2,
                           encoding=ImageEncoding.JPEG, data=b"synthetic")
        provider = NS(provider_name="offline", provider_version="1", is_available=lambda: True,
                      capture_image=lambda camera: frame, get_camera_info=lambda camera: None)
        with tempfile.TemporaryDirectory(prefix="neh-state-store-") as directory:
            artifacts = ArtifactService(LocalArtifactStore(directory, "http://synthetic.invalid"),
                                        retention_policy=ArtifactRetentionPolicy())
            service = ObservationService(provider_owner=NS(providers=provider), artifact_service=artifacts,
                                         state_store=self.store, history_limit=1)
            try:
                context = service.capture_observation(camera_id="front_camera")
                self.assertIn("current-mode", context.observation.summary)
                self.assertNotIn("discarded-mode", context.observation.summary)
            finally:
                self.assertTrue(service.close(timeout=1))
                artifacts.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
