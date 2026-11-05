#!/usr/bin/env python3
"""Offline audio cache/health ownership checks; no hardware or external calls."""

import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.audio.service import AudioService




class AudioCache(unittest.TestCase):
    def service(self, robot=None):
        return AudioService(config=NS(volume=NS(auto_enable_vui_switch=False)), robot=robot or NS())

    def test_unknown_software_read_does_not_invent_observation(self):
        service = self.service()
        for state in (service.get_cached_volume_state(), service.get_volume_state()):
            self.assertIsNone(state["volume"])
            self.assertIsNone(state["observed_at"])
            self.assertEqual(state["freshness"], "unknown")

    def test_software_set_clamps_and_publishes_real_software_state(self):
        service = self.service()
        self.assertEqual(service.set_volume(2)["volume"], 1.0)
        self.assertEqual(service.set_volume(-1)["volume"], 0.0)
        self.assertIsNotNone(service.get_cached_volume_state()["observed_at"])


    def test_cache_and_explicit_results_own_nested_device_data(self):
        raw = {"volume": 0.3, "details": {"level": 3}}
        service = self.service(NS(get_vui_volume_info=lambda: raw))
        result = service.get_volume_state()
        raw["details"]["level"] = 9
        result["robot_volume"]["details"]["level"] = 8
        state = service.get_cached_volume_state()
        self.assertEqual(state["robot_volume"]["details"]["level"], 3)
        state["robot_volume"]["details"]["level"] = 7
        self.assertEqual(service.get_cached_volume_state()["robot_volume"]["details"]["level"], 3)

    def test_failed_get_keeps_last_value_and_time_honestly_stale(self):
        get = Mock(return_value={"volume": 0.4})
        service = self.service(NS(get_vui_volume_info=get))
        old = service.get_volume_state()
        get.side_effect = RuntimeError("offline")
        failed = service.get_volume_state()
        self.assertEqual(failed["volume"], 0.4)
        self.assertEqual(failed["observed_at"], old["observed_at"])
        self.assertEqual(failed["freshness"], "stale_after_error")
        self.assertEqual(failed["last_error"], "offline")
        get.side_effect = None
        get.return_value = {"volume": 0.5}
        new = service.get_volume_state()
        self.assertEqual(new["last_error"], "")
        self.assertNotIn("robot_volume_error", new)

    def test_failed_initial_read_or_set_stays_unknown(self):
        robot = NS(
            get_vui_volume_info=Mock(side_effect=RuntimeError("offline")),
            set_vui_volume_ratio=Mock(side_effect=RuntimeError("offline")),
        )
        service = self.service(robot)
        for state in (service.get_volume_state(), service.set_volume(0.8)):
            self.assertIsNone(state["volume"])
            self.assertIsNone(state["observed_at"])
            self.assertEqual(state["freshness"], "unknown")
            self.assertFalse(state["refresh_in_progress"])

    def test_failed_set_never_caches_requested_volume(self):
        robot = NS(
            get_vui_volume_info=lambda: {"volume": 0.2}, set_vui_volume_ratio=Mock(side_effect=RuntimeError("failed"))
        )
        service = self.service(robot)
        old = service.get_volume_state()
        state = service.set_volume(0.9)
        self.assertEqual(state["volume"], 0.2)
        self.assertEqual(state["observed_at"], old["observed_at"])
        self.assertEqual(state["freshness"], "stale_after_error")

    def test_missing_or_invalid_readback_never_caches_request(self):
        for raw in (None, {}, {"volume": float("nan")}, {"volume": float("inf")}, {"volume": True}, {"volume": 2}):
            with self.subTest(raw=raw):
                service = self.service(NS(set_vui_volume_ratio=lambda *args, raw=raw: raw))
                state = service.set_volume(0.8)
                self.assertIsNone(state["volume"])
                self.assertIsNone(state["observed_at"])
                self.assertTrue(state["last_error"])

    def test_blocked_set_preserves_cached_observation_until_real_readback(self):
        entered, resume, cache_finished = threading.Event(), threading.Event(), threading.Event()
        results, errors = [], []

        def set_value(*args):
            entered.set()
            assert resume.wait(2), "synthetic setter gate timed out"
            return {"volume": 0.6}

        service = self.service(NS(get_vui_volume_info=lambda: {"volume": 0.2}, set_vui_volume_ratio=set_value))
        old = service.get_volume_state()

        def set_volume():
            try:
                service.set_volume(0.9)
            except BaseException as error:  # noqa: BLE001 - Forward every test-thread failure.
                errors.append(error)

        def cached():
            results.append(service.get_cached_volume_state())
            cache_finished.set()

        setter, reader = threading.Thread(target=set_volume), threading.Thread(target=cached)
        setter.start()
        try:
            self.assertTrue(entered.wait(1))
            reader.start()
            self.assertTrue(cache_finished.wait(1))
            self.assertEqual(results[0]["volume"], 0.2)
            self.assertEqual(results[0]["observed_at"], old["observed_at"])
            self.assertTrue(results[0]["refresh_in_progress"])
        finally:
            resume.set()
            setter.join(2)
            if reader.ident is not None:
                reader.join(2)
        self.assertFalse(setter.is_alive())
        self.assertFalse(reader.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(service.get_cached_volume_state()["volume"], 0.6)
        self.assertFalse(service.get_cached_volume_state()["refresh_in_progress"])

    def test_concurrent_explicit_get_set_serializes_device_without_blocking_cache(self):
        entered, resume, set_entered, cache_finished = (
            threading.Event(),
            threading.Event(),
            threading.Event(),
            threading.Event(),
        )
        order = []

        def get():
            order.append("get")
            entered.set()
            assert resume.wait(2)
            return {"volume": 0.2}

        def set_value(*args):
            order.append("set")
            set_entered.set()
            return {"volume": 0.8}

        service = self.service(NS(get_vui_volume_info=get, set_vui_volume_ratio=set_value))
        a = threading.Thread(target=service.get_volume_state)
        b = threading.Thread(target=lambda: service.set_volume(0.8))
        c = threading.Thread(target=lambda: (service.get_cached_volume_state(), cache_finished.set()))
        a.start()
        try:
            self.assertTrue(entered.wait(1))
            b.start()
            c.start()
            self.assertTrue(cache_finished.wait(1))
            self.assertFalse(set_entered.is_set())
        finally:
            resume.set()
            a.join(2)
            b.join(2)
            c.join(2)
        self.assertFalse(any(t.is_alive() for t in (a, b, c)))
        self.assertEqual(order, ["get", "set"])
        self.assertEqual(service.get_cached_volume_state()["volume"], 0.8)


if __name__ == "__main__":
    unittest.main(verbosity=2)
