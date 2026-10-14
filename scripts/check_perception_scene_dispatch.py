#!/usr/bin/env python3
"""Offline scene-event ordering, full-model independence and freshness checks."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import check_perception_dispatch_isolation as overlay_checks

from contracts.events import RuntimeEventCategory
from core import EventBus
from core.errors import GatewayError


class SceneDispatchChecks(unittest.TestCase):
    gate = overlay_checks.DispatchChecks.gate
    spawn = overlay_checks.DispatchChecks.spawn
    join = overlay_checks.DispatchChecks.join
    subscribe = overlay_checks.DispatchChecks.subscribe

    def setUp(self):
        overlay_checks.DispatchChecks.setUp(self)
        self.events = EventBus()
        self.service._event_bus = self.events

    def tearDown(self):
        try:
            overlay_checks.DispatchChecks.tearDown(self)
        finally:
            self.assertTrue(self.events.shutdown(), self.events.status())

    def scene_subscribe(self, callback):
        return self.events.subscribe(callback, event_types=("perception.scene_ready",))

    def event(self, name):
        return self.events.build_event(name, category=RuntimeEventCategory.SYSTEM, source="synthetic")

    def test_scene_callback_allows_next_full_analysis_to_reserve(self):
        entered, release, second_reserved = self.gate(), self.gate(), self.gate()
        received = []

        def callback(event):
            received.append(event)
            if len(received) == 1:
                entered.set()
                if not release.wait(5):
                    raise AssertionError("synthetic scene callback gate timed out")

        original_prepare = self.events.prepare
        prepared = []

        def prepare(event):
            publication = original_prepare(event)
            prepared.append(publication)
            if len(prepared) == 2:
                second_reserved.set()
            return publication

        self.scene_subscribe(callback)
        with patch.object(self.events, "prepare", side_effect=prepare):
            first = self.spawn(lambda: self.service.process_image(self.first))
            self.assertTrue(entered.wait(2))
            second = self.spawn(lambda: self.service.process_image(self.second))
            self.assertTrue(second_reserved.wait(1), "scene subscriber still blocks the next full model")
            self.assertEqual(len(self.detector.calls), 2)
            self.assertEqual(self.service.get_latest_full_perception().metadata["source_sequence"], 2)
            self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 2)
            self.assertTrue(first[0].is_alive())
            self.assertTrue(second[0].is_alive())
            release.set()
            first_context, second_context = self.join(first), self.join(second)
        self.assertEqual([event.subject_id for event in received], [
            first_context.observation.observation_id, second_context.observation.observation_id,
        ])
        self.assertLess(received[0].cursor, received[1].cursor)

    def test_newer_full_completion_after_old_scene_dispatch_suppresses_old_overlay(self):
        entered, release = self.gate(), self.gate()
        overlays = []
        self.subscribe(overlays.append)
        original_dispatch = self.events.dispatch_prepared
        first_dispatch = []

        def dispatch(publication):
            result = original_dispatch(publication)
            if not first_dispatch:
                first_dispatch.append(publication)
                entered.set()
                if not release.wait(5):
                    raise AssertionError("synthetic post-scene gate timed out")
            return result

        with patch.object(self.events, "dispatch_prepared", side_effect=dispatch):
            first = self.spawn(lambda: self.service.process_image(self.first))
            self.assertTrue(entered.wait(2))
            second = self.join(self.spawn(lambda: self.service.process_image(self.second)))
            self.assertIs(self.service.get_latest_full_perception(), second)
            self.assertEqual([frame.payload["source_sequence"] for frame in overlays], [2])
            release.set()
            self.join(first)
        self.assertEqual([frame.payload["source_sequence"] for frame in overlays], [2])

    def test_identity_check_and_overlay_reservation_exclude_newer_full_commit(self):
        scene_entered, scene_release = self.gate(), self.gate()
        second_segment_entered, second_segment_release = self.gate(), self.gate()
        overlay_entered, overlay_release = self.gate(), self.gate()
        second_publish_attempted, second_published = self.gate(), self.gate()
        original_segment = self.service._segmentation_backend.segment
        original_publish = self.service._publish_context
        original_prepare = self.bus.prepare_publication
        overlays = []
        self.subscribe(overlays.append)

        def callback(event):
            if not scene_entered.is_set():
                scene_entered.set()
                if not scene_release.wait(5):
                    raise AssertionError("synthetic scene gate timed out")

        def segment(frame, **kwargs):
            if frame.metadata["source_sequence"] == 2:
                second_segment_entered.set()
                if not second_segment_release.wait(5):
                    raise AssertionError("synthetic segmentation gate timed out")
            return original_segment(frame, **kwargs)

        def publish(context, **kwargs):
            if context.metadata["source_sequence"] == 2:
                second_publish_attempted.set()
                result = original_publish(context, **kwargs)
                second_published.set()
                return result
            return original_publish(context, **kwargs)

        def prepare(frame):
            if frame.payload["source_sequence"] == 1:
                overlay_entered.set()
                if not overlay_release.wait(5):
                    raise AssertionError("synthetic overlay reservation gate timed out")
            return original_prepare(frame)

        self.scene_subscribe(callback)
        with patch.object(self.service._segmentation_backend, "segment", side_effect=segment), \
                patch.object(self.service, "_publish_context", side_effect=publish), \
                patch.object(self.bus, "prepare_publication", side_effect=prepare):
            first = self.spawn(lambda: self.service.process_image(self.first))
            self.assertTrue(scene_entered.wait(2))
            second = self.spawn(lambda: self.service.process_image(self.second))
            self.assertTrue(second_segment_entered.wait(2))
            scene_release.set()
            self.assertTrue(overlay_entered.wait(2))
            acquired = self.service._context_condition.acquire(blocking=False)
            if acquired:
                self.service._context_condition.release()
            self.assertFalse(acquired, "latest identity was not protected through cursor reservation")
            second_segment_release.set()
            self.assertTrue(second_publish_attempted.wait(2))
            self.assertFalse(second_published.is_set())
            overlay_release.set()
            self.join(first)
            self.join(second)
        self.assertTrue(second_published.is_set())
        self.assertEqual([frame.payload["source_sequence"] for frame in overlays], [1, 2])
        self.assertLess(overlays[0].cursor, overlays[1].cursor)

    def test_unrelated_event_backlog_does_not_hold_full_model_lock_and_keeps_fifo(self):
        entered, release, scene_reserved = self.gate(), self.gate(), self.gate()
        seen, scenes = [], []
        original_prepare = self.events.prepare

        def callback(event):
            seen.append(event)
            if event.event_type == "synthetic.first":
                entered.set()
                if not release.wait(5):
                    raise AssertionError("synthetic unrelated callback timed out")

        def prepare(event):
            publication = original_prepare(event)
            if event.event_type == "perception.scene_ready":
                scenes.append(publication)
                scene_reserved.set()
            return publication

        self.events.subscribe(callback)
        with patch.object(self.events, "prepare", side_effect=prepare):
            unrelated = self.spawn(lambda: self.events.publish(self.event("synthetic.first")))
            self.assertTrue(entered.wait(2))
            first = self.spawn(lambda: self.service.process_image(self.first))
            self.assertTrue(scene_reserved.wait(2))
            middle = self.events.prepare(self.event("synthetic.middle"))
            middle_dispatch = self.spawn(lambda: self.events.dispatch_prepared(middle))
            scene_reserved.clear()
            second = self.spawn(lambda: self.service.process_image(self.second))
            self.assertTrue(scene_reserved.wait(1), "unrelated event blocks full model progress")
            self.assertEqual(len(scenes), 2)
            release.set()
            for worker in (unrelated, first, middle_dispatch, second):
                self.join(worker)
        self.assertEqual([event.event_type for event in seen], [
            "synthetic.first", "perception.scene_ready", "synthetic.middle", "perception.scene_ready",
        ])
        self.assertEqual([event.cursor for event in seen], sorted(event.cursor for event in seen))

    def test_scene_callback_can_reenter_full_and_preserves_queued_event(self):
        seen, nested = [], []
        overlays = []
        self.subscribe(overlays.append)

        def callback(event):
            seen.append(event)
            if len(seen) == 1:
                nested.append(self.service.process_image(self.second))

        self.scene_subscribe(callback)
        first = self.join(self.spawn(lambda: self.service.process_image(self.first)))
        self.assertEqual([event.subject_id for event in seen], [
            first.observation.observation_id, nested[0].observation.observation_id,
        ])
        self.assertEqual([frame.payload["source_sequence"] for frame in overlays], [2])
        self.assertEqual(self.events.status()["prepared_publications"], 0)

    def test_close_during_scene_callback_retains_operation_and_artifact(self):
        entered, release = self.gate(), self.gate()

        def callback(_event):
            entered.set()
            if not release.wait(5):
                raise AssertionError("synthetic scene callback timed out")

        self.scene_subscribe(callback)
        artifact = self.artifacts.save_image_frame(self.first)
        first = self.spawn(lambda: self.service.process_image(self.first, image_artifact=artifact))
        self.assertTrue(entered.wait(2))
        self.assertFalse(self.service.close(timeout=0))
        self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 1)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 4)
        with self.assertRaises(GatewayError):
            self.store.delete_artifact(artifact.artifact_id)
        with self.assertRaises(GatewayError):
            self.service.process_image(self.second)
        release.set()
        self.join(first)
        self.assertTrue(self.service.close(timeout=1))

    def test_dispatch_interruption_abandons_reservation_and_releases_artifact(self):
        seen = []
        self.scene_subscribe(seen.append)
        artifact = self.artifacts.save_image_frame(self.first)
        with patch.object(self.events, "dispatch_prepared", side_effect=overlay_checks.CallbackInterrupt("synthetic interrupt")), \
                self.assertRaises(overlay_checks.CallbackInterrupt):
            self.service.process_image(self.first, image_artifact=artifact)
        self.assertEqual(self.events.status()["prepared_publications"], 0)
        self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 0)
        self.assertEqual(self.store.get_holder_status()["holder_count"], 3)
        second = self.service.process_image(self.second)
        self.assertEqual([event.subject_id for event in seen], [second.observation.observation_id])

    def test_callback_baseexception_is_isolated_without_losing_next_scene(self):
        seen = []

        def callback(event):
            seen.append(event)
            if len(seen) == 1:
                raise overlay_checks.CallbackInterrupt("synthetic callback exit")

        self.scene_subscribe(callback)
        self.service.process_image(self.first)
        self.service.process_image(self.second)
        self.assertEqual(len(seen), 2)
        self.assertEqual(self.events.status()["callback_failures"], 1)
        self.assertEqual(self.events.status()["prepared_publications"], 0)

    def test_event_prepare_failure_releases_full_lock_and_operation(self):
        with patch.object(self.events, "prepare", side_effect=RuntimeError("synthetic preparation failure")), \
                self.assertRaisesRegex(RuntimeError, "synthetic preparation failure"):
            self.service.process_image(self.first)
        self.assertEqual(self.service.get_lifecycle_status()["active_operations"], 0)
        self.assertFalse(self.service._full_pipeline_lock.locked())
        self.assertEqual(self.events.status()["prepared_publications"], 0)
        self.assertIsNotNone(self.service.process_image(self.second))


if __name__ == "__main__":
    unittest.main(verbosity=2)
