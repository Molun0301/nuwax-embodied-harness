#!/usr/bin/env python3
"""机器人网页状态的离线修订回归；不启动服务或连接设备。"""
from __future__ import annotations

import importlib.util
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from contracts.robot_state import BatteryState, RobotControlMode, RobotState, SafetyState
from core.state_store import StateNamespace, StateStore
from core.stream_bus import StreamBus

# 仅加载被测服务文件，不触发 services 包中其它服务的装配。
SPEC = importlib.util.spec_from_file_location("robot_state_revision_subject", ROOT / "services/robot_state_service.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
RobotStateService = MODULE.RobotStateService


class Publications:
    def __init__(self):
        self.frames = []

    def build_event(self, event_type, **fields):
        return {"event_type": event_type, **fields}

    def publish(self, topic_or_event, **fields):
        self.frames.append({"topic": topic_or_event, **fields} if fields else topic_or_event)


class RobotStateRevisionChecks(unittest.TestCase):
    def setUp(self):
        self.robot = SimpleNamespace(
            manifest=SimpleNamespace(robot_name="synthetic", robot_model="test"),
            get_status=dict, get_adapter_health_statuses=list, providers=None,
        )
        self.store, self.events, self.streams = StateStore(), Publications(), Publications()
        self.service = self.new_service()
        self.robot_state = RobotState(robot_id="synthetic", frame_id="base_link", mode=RobotControlMode.IDLE,
                                      battery=BatteryState(percentage=70), safety=SafetyState(can_move=True))

    def new_service(self):
        return RobotStateService(self.robot, state_store=self.store, event_bus=self.events, stream_bus=self.streams)

    def ingest(self, value=None, source="provider"):
        return self.service.ingest_robot_state(value or self.robot_state, source_name=source)

    def test_semantic_equality_keeps_revision_and_does_not_publish(self):
        self.ingest()
        self.ingest(self.robot_state.model_copy(deep=True))
        self.assertEqual(len(self.streams.frames), 1)
        self.assertEqual(len(self.events.frames), 1)
        metadata = self.streams.frames[0]["metadata"]
        self.assertTrue(metadata["source_epoch"])
        self.assertEqual(self.streams.frames[0]["revision"], f"robot-state:{metadata['source_epoch']}:1")
        self.assertEqual(metadata["source_epoch"], self.events.frames[0]["payload"]["source_epoch"])
        self.assertEqual(set(metadata), {"robot_name", "mode", "diagnostic_count", "revision", "source_epoch", "change_policy"})

    def test_changed_semantics_increment_revision_in_the_same_epoch(self):
        self.ingest()
        self.ingest(self.robot_state.model_copy(update={"battery": BatteryState(percentage=60)}))
        first, second = self.streams.frames
        self.assertEqual((first["metadata"]["revision"], second["metadata"]["revision"]), (1, 2))
        self.assertNotEqual(first["revision"], second["revision"])
        self.assertEqual(first["metadata"]["source_epoch"], second["metadata"]["source_epoch"])
        self.assertEqual(second["payload"].robot_state.battery.percentage, 60)

    def test_owner_restart_allocates_an_independent_epoch(self):
        self.ingest()
        self.service = self.new_service()
        self.ingest()
        first, second = self.streams.frames
        self.assertEqual(first["metadata"]["revision"], second["metadata"]["revision"])
        self.assertNotEqual(first["revision"], second["revision"])
        self.assertNotEqual(first["metadata"]["source_epoch"], second["metadata"]["source_epoch"])

    def test_source_change_remains_a_semantic_update(self):
        self.ingest(source="first-provider")
        self.ingest(source="second-provider")
        self.assertEqual([frame["metadata"]["revision"] for frame in self.streams.frames], [1, 2])

    def test_volatile_updates_keep_complete_latest_state_without_extra_web_publications(self):
        for sequence in range(1000):
            state = self.robot_state.model_copy(update={"metadata": {"sequence": sequence}})
            self.ingest(state)
        self.assertEqual(len(self.streams.frames), 1)
        self.assertEqual(self.service.get_latest_snapshot().robot_state.metadata["sequence"], 999)
        self.assertEqual(self.store.read(StateNamespace.ROBOT_STATE, "synthetic").value.metadata["sequence"], 999)

    def test_stream_snapshot_replay_and_wire_keep_owner_revision_without_digest_fields(self):
        from gateways.ws.data_streams import _encode_ws_frame, _format_sse_payload

        stream = StreamBus()
        self.addCleanup(stream.shutdown)
        self.service = RobotStateService(self.robot, state_store=self.store, stream_bus=stream)
        self.ingest()
        first = stream.latest_frame("robot.state")
        first_replay = stream.replay_snapshot(after_cursor=0, topics=("robot.state",))
        self.assertEqual([frame.revision for frame in first_replay.frames], [first.revision])
        self.ingest(self.robot_state.model_copy(update={"battery": BatteryState(percentage=65)}))
        latest = stream.latest_frame("robot.state")
        replay = stream.replay_snapshot(after_cursor=0, topics=("robot.state",))
        # robot.state 延续总线原有 latest-only 合并，不把本次身份变更扩成队列行为变更。
        self.assertEqual([frame.revision for frame in replay.frames], [latest.revision])
        self.assertEqual(replay.latest_frames[0].revision, latest.revision)
        for frame in (first, latest):
            wire = json.loads(_encode_ws_frame(frame))["frame"]
            self.assertEqual(wire["revision"], frame.revision)
            self.assertEqual(wire["metadata"], frame.metadata)
            self.assertEqual(wire["payload"]["robot_state"]["battery"]["percentage"],
                             frame.payload.robot_state.battery.percentage)
            self.assertIn(frame.revision, _format_sse_payload(frame))


if __name__ == "__main__":
    unittest.main(verbosity=2)
