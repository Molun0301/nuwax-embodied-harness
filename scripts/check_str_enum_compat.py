#!/usr/bin/env python3
"""Verify string-enum parity and warning protection on Python 3.8 and newer."""
from __future__ import annotations

import builtins
import json
import logging
import sys
import unittest
from enum import Enum
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.compat import StrEnum


def fallback_enum():
    """Exercise the actual compatibility source even on native-StrEnum Python."""
    original_import = builtins.__import__

    def legacy_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "enum" and "StrEnum" in fromlist:
            raise ImportError("exercise pre-3.11 fallback")
        return original_import(name, globals, locals, fromlist, level)

    namespace = {"__builtins__": dict(vars(builtins), __import__=legacy_import)}
    source = Path(__file__).resolve().parents[1] / "contracts" / "compat.py"
    exec(compile(source.read_text(), str(source), "exec"), namespace)
    return namespace["StrEnum"]


class EnumCompatibility(unittest.TestCase):
    def check_strings(self, base):
        class Severity(base):
            WARNING = "warning"
            ERROR = "error"

        value = Severity.WARNING
        self.assertEqual(str(value), "warning")
        self.assertEqual("%s" % value, "warning")
        self.assertEqual(f"{value}", "warning")
        self.assertEqual(format(value, ">10"), "   warning")
        self.assertEqual(repr(value), "<Severity.WARNING: 'warning'>")
        self.assertEqual(json.dumps({"severity": value}), '{"severity": "warning"}')
        self.assertEqual(value, "warning")
        self.assertEqual(hash(value), hash("warning"))
        self.assertIs(Severity("warning"), value)
        self.assertIs(Severity.__members__["WARNING"], value)
        self.assertEqual(list(Severity), [Severity.WARNING, Severity.ERROR])
        self.assertIsInstance(value, str)
        self.assertIsInstance(value, Enum)

    def test_runtime_selected_enum(self):
        self.check_strings(StrEnum)

    def test_forced_legacy_fallback(self):
        self.check_strings(fallback_enum())


class WarningProtection(unittest.TestCase):
    def test_warning_and_error_keep_feedback_and_reserved_capacity(self):
        from contracts.events import RuntimeEventCategory, RuntimeEventSeverity
        from core.capability_registry import CapabilityRegistry
        from core.event_bus import EventBus
        from core.logging_utils import LogRateLimiter
        from core.stream_bus import StreamBus
        from gateways import runtime as runtime_module
        from gateways.feedback_messages import is_feedback_event
        from gateways.runtime import GatewayRuntime
        from gateways.runtime_event_log import RuntimeEventLog

        class Sink(logging.Handler):
            def __init__(self):
                super().__init__()
                self.records = []

            def emit(self, record):
                self.records.append(record)

        for severity in (RuntimeEventSeverity.WARNING, RuntimeEventSeverity.ERROR):
            with self.subTest(severity=severity.value):
                import tempfile
                from tests.conftest import create_isolated_gateway
                directory = tempfile.TemporaryDirectory(prefix="nuwax-enum-feedback-")
                self.addCleanup(directory.cleanup)
                runtime = create_isolated_gateway(Path(directory.name))
                self.addCleanup(runtime.stop)
                gaps = []
                selected_log = RuntimeEventLog(runtime._write_runtime_event_log,
                    gap_writer=gaps.append, max_items=1, reserved_items=1)
                binding = patch.object(runtime, "_runtime_event_log", selected_log)
                binding.start()
                self.addCleanup(binding.stop)
                logger = logging.Logger("check.runtime", logging.INFO)
                feedback_logger = logging.Logger("check.feedback", logging.INFO)
                runtime_sink, feedback_sink = Sink(), Sink()
                logger.addHandler(runtime_sink)
                feedback_logger.addHandler(feedback_sink)
                try:
                    with patch.object(runtime_module, "RUNTIME_LOGGER", logger), \
                            patch.object(runtime_module, "FEEDBACK_LOGGER", feedback_logger):
                        # Ordinary telemetry cannot use the one reserved slot.
                        ordinary = runtime.event_bus.build_event(
                            "localization.snapshot_ready", category=RuntimeEventCategory.ROBOT,
                            source="compat_check", payload={"source_name": "lidar"})
                        runtime.event_bus.publish(ordinary)
                        event = ordinary.model_copy(update={"severity": severity})
                        self.assertTrue(is_feedback_event(event))
                        self.assertEqual(str(event.category), "robot")
                        roundtrip = type(event).model_validate_json(event.model_dump_json())
                        self.assertIs(roundtrip.severity, severity)
                        self.assertEqual(str(roundtrip.severity), severity.value)
                        stored = runtime.event_bus.publish(event)
                        self.assertTrue(runtime._runtime_event_log.close())
                    status = runtime._runtime_event_log.status()
                    self.assertEqual(status["accepted"], 1)
                    self.assertEqual(status["delivered"], 1)
                    self.assertEqual(status["telemetry_dropped"], 1)
                    self.assertEqual(status["protected_dropped"], 0)
                    self.assertEqual(status["writer_failed"], 0)
                    self.assertEqual(sum(gap["missing_count"] for gap in gaps), 1)
                    self.assertEqual(sum(gap["protected_missing_count"] for gap in gaps), 0)
                    self.assertEqual(len(runtime_sink.records), 1)
                    self.assertEqual(len(feedback_sink.records), 1)
                    level = logging.WARNING if severity is RuntimeEventSeverity.WARNING else logging.ERROR
                    self.assertEqual(runtime_sink.records[0].levelno, level)
                    self.assertEqual(feedback_sink.records[0].levelno, level)
                    self.assertEqual(feedback_sink.records[0].feedback["severity"], severity.value)
                    self.assertEqual(runtime_sink.records[0].cursor, stored.cursor)
                    self.assertEqual(len(runtime.event_bus.replay()), 2)
                finally:
                    self.assertTrue(runtime._runtime_event_log.close())
                    self.assertTrue(runtime.event_bus.shutdown())
                    runtime.stream_bus.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
