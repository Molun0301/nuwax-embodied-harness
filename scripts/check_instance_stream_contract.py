#!/usr/bin/env python3
"""Instance SSE snapshots preserve the producer contract; no models or hardware."""
from __future__ import annotations

import asyncio
import json
import sys
import threading
import unittest
from datetime import datetime, timezone
from decimal import Decimal
from enum import Enum
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from uuid import UUID

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from contracts.geometry import Pose
from contracts.spatial_memory import InstanceDynamicState, InstanceLifecycleState, SemanticInstance
from contracts.streams import StreamFrame
from gateways.ws.data_streams import (
    _encode_ws_frame, _format_sse, _format_sse_keepalive, _format_sse_payload,
    _strict_json_dumps,
)
from services.memory.instance_association_service import InstanceAssociationService
from services.memory.service import MemoryService


NOW = datetime(2026, 1, 1, tzinfo=timezone.utc)
LIBRARY = "synthetic-instance-library"


def memory_fixture(count=500):
    from core import StateStore
    owner = MemoryService(localization_service=None, mapping_service=None,
        observation_service=None, perception_service=None, state_store=StateStore(),
        text_embedding_model="disabled", image_embedding_model="disabled")
    owner._memory_enabled = True
    owner._active_library_name = LIBRARY
    owner._durable_index.activate(LIBRARY, load_history=False)
    for index in range(count):
        instance = SemanticInstance(
            instance_id="ins_synthetic_20260101T000000Z_%08x" % index,
            anchor_id="anc_synthetic_20260101T000000Z_%08x" % index,
            label="cabinet" if index % 2 else "chair", display_name="合成物品\n%d" % index,
            pose=Pose(frame_id="map", position={"x": index / 8., "y": index % 7 / 4., "z": .5}),
            timestamp=NOW, first_seen_ts=NOW, last_seen_ts=NOW, map_name=LIBRARY,
            confidence=.875, observation_count=index + 1,
            lifecycle_state=InstanceLifecycleState.ACTIVE if index % 3 else InstanceLifecycleState.STALE,
            dynamic_state=InstanceDynamicState.STATIC,
            metadata={"memory_library_name": LIBRARY, "projection_method": "depth",
                      "projection_confidence": .8, "support_point_count": 32,
                      "navigable": True, "observation_only": bool(index % 5 == 0)},
        )
        owner._semantic_instances_by_id[instance.instance_id] = instance
    return owner


def snapshot_fixture(count=500, limit=500):
    owner = memory_fixture(count)
    try:
        with patch("services.memory.service.utc_now", return_value=NOW):
            payload = owner.get_semantic_instances_snapshot(limit=limit)
    finally:
        owner.close()
    payload.pop("_next_update_at")  # SemanticInstancePublisher consumes this internal deadline.
    return payload


def frame_for(payload, topic="memory.instances"):
    return StreamFrame(topic=topic, source="memory_service", payload=payload,
                       frame_id="map", cursor=9007199254740993, status="available", timestamp=NOW)


def sse_value(encoded):
    return json.loads(next(line[6:] for line in encoded.split("\n") if line.startswith("data: ")))


class InstanceStreamContractChecks(unittest.TestCase):
    def test_500_source_rows_keep_every_id_and_field_in_ws_and_sse(self):
        owner = memory_fixture()
        self.addCleanup(owner.close)
        with patch("services.memory.service.utc_now", return_value=NOW):
            payload = owner.get_semantic_instances_snapshot()
            agent_rows = owner.list_semantic_instances(library_name=LIBRARY)
        payload.pop("_next_update_at")
        self.assertEqual(payload["instances"], agent_rows)
        frame = frame_for(payload)
        ws = json.loads(_encode_ws_frame(frame))["frame"]
        sse = sse_value(_format_sse_payload(frame))
        self.assertEqual(len(sse["payload"]["instances"]), 500)
        self.assertEqual(sse, ws)
        self.assertEqual(sse["payload"], payload)
        for expected, received in zip(agent_rows, sse["payload"]["instances"]):
            with self.subTest(instance_id=expected["instance_id"]):
                self.assertEqual(received, expected)
                self.assertEqual(set(received), set(expected))
        self.assertEqual(sse["cursor"], "9007199254740993")
        self.assertTrue(_format_sse_payload(frame).startswith("id: 9007199254740993\nevent: stream.frame\n"))

    def test_source_limit_and_explicit_truncation_remain_authoritative(self):
        for limit in (500, 1000):
            with self.subTest(limit=limit):
                payload = snapshot_fixture(count=1001, limit=limit)
                actual = sse_value(_format_sse_payload(frame_for(payload)))["payload"]
                self.assertEqual(len(actual["instances"]), limit)
                self.assertEqual(actual, payload)
                self.assertEqual((len(actual["instances"]), actual["total"], actual["limit"], actual["truncated"]),
                                 (limit, 1001, limit, True))

    def test_empty_and_unavailable_snapshots_match_ws(self):
        for payload in ({"instances": [], "total": 0, "limit": 500, "truncated": False,
                         "library_name": None, "active_library_name": None},
                        {"available": False, "message": "空间记忆快照读取失败。"}):
            with self.subTest(payload=payload):
                frame = frame_for(payload)
                self.assertEqual(sse_value(_format_sse_payload(frame)), json.loads(_encode_ws_frame(frame))["frame"])

    def test_existing_supported_scalar_types_and_multiline_text_match_ws(self):
        class Example(Enum):
            ITEM = "item"
        payload = {"instances": [{"instance_id": "legacy", "timestamp": NOW,
                   "decimal": Decimal("1.25"), "path": Path("synthetic/file"),
                   "uuid": UUID(int=1), "enum": Example.ITEM, "text": "中文\r\nnext\u2028line"}]}
        frame = frame_for(payload)
        encoded = _format_sse_payload(frame)
        self.assertEqual(len([line for line in encoded.split("\n") if line.startswith("data: ")]), 1)
        self.assertEqual(sse_value(encoded), json.loads(_encode_ws_frame(frame))["frame"])

    def test_noncanonical_instance_diagnostics_keep_bounded_fallback(self):
        cycle = []; cycle.append(cycle)
        for extra in (float("nan"), float("inf"), b"x" * 100000, cycle, object()):
            with self.subTest(type=type(extra).__name__):
                frame = frame_for({"instances": [{"instance_id": "diagnostic", "extra": extra}]})
                self.assertEqual(sse_value(_format_sse_payload(frame)), json.loads(_strict_json_dumps(frame)))
                self.assertLess(len(_format_sse_payload(frame)), 40000)

    def test_other_json_topics_still_have_node_item_and_text_budgets(self):
        oversized = {"rows": [{"id": str(i), "text": "x" * 9000} for i in range(600)]}
        for topic in ("memory.scene_spaces", "diagnostic.generic", "robot.state"):
            with self.subTest(topic=topic):
                frame = frame_for(oversized, topic=topic)
                actual = _format_sse_payload(frame)
                self.assertEqual(sse_value(actual), json.loads(_strict_json_dumps(frame)))
                self.assertLess(len(actual), 100000)
                self.assertIn("truncated", actual)

    def test_generic_mappings_cannot_bypass_budgets_by_claiming_topic_name(self):
        raw = {"topic": "memory.instances", "payload": {"instances": list(range(500))}}
        actual = _format_sse("stream.frame", raw)
        self.assertEqual(sse_value(actual), json.loads(_strict_json_dumps(raw)))
        self.assertIn("<truncated:item_limit>", actual)

    def test_deep_or_failing_model_diagnostics_retain_old_sse_projection(self):
        class BrokenModel:
            def model_dump(self, **kwargs):
                raise RuntimeError("synthetic dump failure")
        nested = "leaf"
        for _ in range(10000):
            nested = [nested]
        for extra in (nested, BrokenModel()):
            with self.subTest(type=type(extra).__name__):
                frame = frame_for({"instances": [{"instance_id": "diagnostic", "extra": extra}]})
                actual = _format_sse_payload(frame)
                self.assertEqual(sse_value(actual), json.loads(_strict_json_dumps(frame)))
                actual.encode("utf-8")
                self.assertLess(len(actual), 40000)

    def test_public_surrogate_name_uses_json_escape_without_utf8_send_failure(self):
        payload = snapshot_fixture(count=1)
        payload["instances"][0]["display_name"] = "name-\ud800-end"
        encoded = _format_sse_payload(frame_for(payload))
        wire = encoded.encode("utf-8")
        self.assertEqual(sse_value(wire.decode("utf-8"))["payload"], payload)
        self.assertIn("\\ud800", encoded)

    def test_legal_long_instance_text_is_complete_not_a_byte_budget_claim(self):
        payload = snapshot_fixture(count=1)
        payload["instances"][0]["display_name"] = "完整名称" * 40000
        frame = frame_for(payload)
        actual = _format_sse_payload(frame)
        self.assertEqual(sse_value(actual)["payload"], payload)
        self.assertEqual(sse_value(actual), json.loads(_encode_ws_frame(frame))["frame"])
        actual.encode("utf-8")

    def test_streaming_response_emits_utf8_body_for_complete_and_diagnostic_frames(self):
        from starlette.responses import StreamingResponse
        class BrokenModel:
            def model_dump(self, **kwargs):
                raise RuntimeError("synthetic dump failure")
        nested = "leaf"
        for _ in range(10000):
            nested = [nested]
        normal = snapshot_fixture(count=500)
        surrogate = snapshot_fixture(count=1)
        surrogate["instances"][0]["display_name"] = "name-\ud800-end"
        long_text = snapshot_fixture(count=1)
        long_text["instances"][0]["display_name"] = "完整名称" * 40000
        cases = [normal, surrogate, long_text,
                 {"instances": [{"instance_id": "deep", "extra": nested}]},
                 {"instances": [{"instance_id": "broken", "extra": BrokenModel()}]}]
        async def emit(frame):
            messages = []
            async def body():
                yield _format_sse_payload(frame)
            async def receive():
                await asyncio.Event().wait()
            async def send(message):
                messages.append(message)
            response = StreamingResponse(body(), media_type="text/event-stream")
            await response({"type": "http", "asgi": {"version": "3.0", "spec_version": "2.4"},
                            "method": "GET", "headers": []}, receive, send)
            self.assertEqual(messages[0]["status"], 200)
            self.assertFalse(messages[-1].get("more_body", False))
            wire = b"".join(message.get("body", b"") for message in messages)
            return sse_value(wire.decode("utf-8"))
        for index, payload in enumerate(cases):
            with self.subTest(case=index):
                frame = frame_for(payload)
                actual = asyncio.run(emit(frame))
                if index < 3:
                    self.assertEqual(actual["payload"], payload)
                else:
                    self.assertEqual(actual, json.loads(_strict_json_dumps(frame)))

    def test_existing_log_text_exception_and_keepalive_are_unchanged(self):
        payload = {"message": "x" * 20000, "exception": "y" * 20000}
        frame = frame_for(payload, topic="logs.entry")
        self.assertEqual(sse_value(_format_sse_payload(frame))["payload"], payload)
        self.assertEqual(sse_value(_format_sse_keepalive(9007199254740993)), {"cursor": "9007199254740993"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
