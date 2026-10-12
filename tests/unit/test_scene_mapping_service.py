"""用真实记忆仓储验证语义闭环、旧结果隔离与停止。"""
from dataclasses import replace
from types import SimpleNamespace
import threading
import time

import pytest

from contracts.base import utc_now
from contracts.geometry import Pose, Vector3
from contracts.maps import OccupancyGrid
from contracts.naming import build_observation_id
from contracts.perception import Observation
from contracts.runtime_views import LocalizationSnapshot, MapSnapshot, PerceptionContext, SceneSummary, SceneObjectSummary
from contracts.scene_decision import SceneDecisionResult
from contracts.spatial_memory import SceneSpaceReviewStatus
from core.errors import GatewayError
from core.settings import SceneMappingConfig
from services.memory.service import MemoryService
from services.memory.scene_mapping import SceneMappingService, CATEGORIES


def make_snapshot(*, epoch="a", sequence=1, name="lab", change=False):
    data = [100] * 3600
    for y in range(5, 55):
        for x in range(5, 55):
            data[y * 60 + x] = 0
    if change:
        for y in range(5, 20):
            for x in range(5, 20):
                data[y * 60 + x] = 100
    return MapSnapshot(map_name=name, source_name="test", source_epoch=epoch, update_sequence=sequence,
        occupancy_grid=OccupancyGrid(map_id=name, frame_id="map", width=60, height=60,
                                    resolution_m=.1, origin=Pose(frame_id="map"), data=data))


class Backend:
    def __init__(self):
        self.calls = []
        self.hook = None
        self.closed = False
        self.winner = "office"

    def decide(self, request):
        self.calls.append(request)
        if self.hook:
            self.hook()
        return SceneDecisionResult(category_scores={key: .95 if key == self.winner else .01 for key in CATEGORIES},
            score_kind="nli_entailment", model_id="test-model", raw={"note": "真实契约的受控决策替身"})

    def close(self):
        self.closed = True


@pytest.fixture
def setup(tmp_path):
    holder = {"map": make_snapshot(), "pose": Pose(frame_id="map", position=Vector3(x=2, y=2))}
    mapping = SimpleNamespace(get_latest_snapshot=lambda: holder["map"])
    localization = SimpleNamespace(get_active_session=lambda: SimpleNamespace(map_name="lab", pose_available=True,
             session_id="test-session", status=SimpleNamespace(value="ready")),
             get_latest_snapshot=lambda: LocalizationSnapshot(source_name="test", current_pose=holder["pose"]))
    memory = MemoryService(localization_service=localization, mapping_service=mapping,
        observation_service=SimpleNamespace(), perception_service=SimpleNamespace(),
        state_store=SimpleNamespace(write=lambda *a, **kw: None),
        memory_db_path=str(tmp_path / "memory.sqlite"), text_embedding_model="disabled", image_embedding_model="disabled")
    memory.activate_memory_library(library_name="lab")
    backend = Backend()
    config = SceneMappingConfig(enabled=True, auto_apply=False, model="test-model", min_evidence_views=2)
    service = SceneMappingService(memory_service=memory, config=config, backend=backend)
    yield holder, memory, backend, service
    service.close()
    assert memory.close()


def remember(memory, holder, x=2, *, epoch=None, timestamp=None):
    now = timestamp or utc_now()
    pose = Pose(frame_id="map", position=Vector3(x=x, y=2), timestamp=now)
    observation_id = build_observation_id("front")
    context = PerceptionContext(camera_id="front", observation=Observation(observation_id=observation_id,
        frame_id="camera", summary="可见电脑与办公桌"),
        scene_summary=SceneSummary(headline="可见独立办公桌、显示器与办公椅", objects=[SceneObjectSummary(label="desk", count=3)]),
        pipeline_name="test", detector_backend="test", tracker_backend="test",
        metadata={"frame_timestamp": now.isoformat(), "frame_timestamp_source": "image_frame.timestamp"})
    snapshot = holder["map"] if epoch is None else make_snapshot(epoch=epoch)
    return memory._create_scene_memory_entry(title="场景观察", summary=None, tags=[], linked_location=None,
        camera_id="front", metadata={}, perception_context=context, use_observation_snapshot=True,
        observation_pose=pose, use_spatial_snapshot=True, map_snapshot=snapshot,
        localization_snapshot=LocalizationSnapshot(source_name="test", current_pose=pose), expected_library_name="lab")


@pytest.mark.parametrize("category,label", [("office", "办公室"), ("living_room", "客厅"),
                                           ("bedroom", "卧室"), ("bathroom", "卫生间")])
def test_real_memory_to_candidate_apply_and_cold_restore(setup, category, label):
    holder, memory, backend, service = setup
    # 验证类别通过真实仓储与冷加载；模型识别正确性由真实图像联调验证。
    backend.winner = category
    for x in (2, 3):
        remember(memory, holder, x)
    result = service.run_once()
    assert result["region_count"] == 1 and result["processed_ids"] and not result["applied_ids"]
    row = service.get_map()["regions"][0]
    assert row["primary_label"] is None and row["review_status"] == "candidate"
    assert row["metadata"]["last_decision"]["score_kind"] == "nli_entailment"
    assert len(row["observation_samples"]) == 2
    service.run_once(apply_labels=True)
    saved = memory._repository.load_scene_spaces(library_name="lab")[0]
    assert saved.primary_label == label and saved.metadata["navigable"] is False
    assert memory._scene_space_navigation_pose(saved) is None
    memory._scene_spaces_by_id.clear()
    memory._load_persisted_memory(library_name="lab")
    assert service.get_map()["regions"][0]["primary_label"] == label
    assert backend.calls[0].state["evidence"][0]["visual_summary"]


def test_no_evidence_and_wrong_epoch_never_infer(setup):
    holder, memory, backend, service = setup
    service.run_once(apply_labels=True)
    assert not backend.calls
    current = holder["map"]
    holder["map"] = make_snapshot(epoch="old")
    remember(memory, holder)
    holder["map"] = current
    result = service.run_once(apply_labels=True)
    assert not backend.calls and result["skipped"][0]["reason"] == "no_valid_observation_evidence"


def test_repeated_same_view_does_not_satisfy_coverage(setup):
    holder, memory, backend, service = setup
    remember(memory, holder, 2)
    remember(memory, holder, 2.1)
    result = service.run_once(apply_labels=True)
    assert backend.calls and not result["applied_ids"]
    assert service.get_map()["regions"][0]["metadata"]["last_decision"]["independent_view_count"] == 1


def test_unknown_never_adopted(setup):
    holder, memory, backend, service = setup
    backend.winner = "unknown"
    for x in (2, 3):
        remember(memory, holder, x)
    assert not service.run_once(apply_labels=True)["applied_ids"]


def test_map_switch_during_model_rejects_late_result(setup):
    holder, memory, backend, service = setup
    remember(memory, holder)
    backend.hook = lambda: holder.update(map=make_snapshot(epoch="new"))
    with pytest.raises(GatewayError, match="作用域"):
        service.run_once(apply_labels=True)
    assert all(space.primary_label is None for space in memory._repository.load_scene_spaces(library_name="lab"))
    assert service.get_map()["regions"][0]["review_status"] == "stale"


def test_same_name_memory_reactivation_rejects_late_result(setup):
    holder, memory, backend, service = setup
    remember(memory, holder)
    def reactivate():
        with memory._instances_lock:
            memory._clear_active_memory_state()
    backend.hook = reactivate
    with pytest.raises(GatewayError):
        service.run_once(apply_labels=True)
    assert all(space.primary_label is None for space in memory._repository.load_scene_spaces(library_name="lab"))


def test_changed_geometry_discards_result(setup):
    holder, memory, backend, service = setup
    remember(memory, holder)
    backend.hook = lambda: holder.update(map=make_snapshot(sequence=2, change=True))
    result = service.run_once(apply_labels=True)
    assert not result["completed"] and result["failures"][0]["code"] == "scene_geometry_changed"


def test_content_unchanged_revision_advance_can_commit(setup):
    holder, memory, backend, service = setup
    for x in (2, 3):
        remember(memory, holder, x)
    backend.hook = lambda: holder.update(map=make_snapshot(sequence=2))
    assert service.run_once(apply_labels=True)["applied_ids"]


def test_explicit_label_revision_idempotence_and_protection(setup):
    holder, memory, backend, service = setup
    service.run_once()
    row = service.get_map()["regions"][0]
    args = dict(scene_space_id=row["scene_space_id"], map_name="lab",
                expected_revision=row["metadata"]["scene_revision"], label="研发办公室", request_id="r1", requested_by="test")
    updated = memory.update_scene_mapping_label(**args)
    assert memory.update_scene_mapping_label(**args) == updated
    with pytest.raises(GatewayError) as conflict:
        memory.update_scene_mapping_label(**{**args, "request_id": "r2"})
    assert conflict.value.error_code == "scene_revision_conflict"
    for x in (2, 3):
        remember(memory, holder, x)
    result = service.run_once(apply_labels=True)
    assert not backend.calls and result["skipped"][0]["reason"] == "explicit_label_protected"
    assert service.get_map()["regions"][0]["primary_label"] == "研发办公室"


def test_user_label_during_model_is_not_overwritten(setup):
    holder, memory, backend, service = setup
    for x in (2, 3):
        remember(memory, holder, x)
    def rename():
        row = service.get_map()["regions"][0]
        memory.update_scene_mapping_label(scene_space_id=row["scene_space_id"], map_name="lab",
            expected_revision=row["metadata"]["scene_revision"], label="会议室", request_id="r3", requested_by="test")
    backend.hook = rename
    result = service.run_once(apply_labels=True)
    assert result["failures"][0]["code"] == "scene_revision_conflict"
    assert service.get_map()["regions"][0]["primary_label"] == "会议室"


def test_stop_and_bounded_single_worker_reject_late_result(setup):
    holder, memory, backend, service = setup
    remember(memory, holder)
    entered, release = threading.Event(), threading.Event()
    backend.hook = lambda: (entered.set(), release.wait(3))
    service.start()
    assert entered.wait(2)
    with pytest.raises(GatewayError) as busy:
        service.run_once()
    assert busy.value.error_code == "scene_mapping_busy"
    assert service.stop(wait_sec=.01)["stop_pending"]
    release.set()
    service.stop(wait_sec=2)
    assert not service.status()["running"]
    assert all(space.primary_label is None for space in memory._repository.load_scene_spaces(library_name="lab"))


def test_locate_reports_geometry_and_unknown(setup):
    holder, memory, backend, service = setup
    service.run_once()
    assert service.locate()["status"] == "located"
    holder["pose"] = Pose(frame_id="map", position=Vector3(x=0, y=0))
    assert service.locate()["status"] == "unknown"


def test_accepted_capture_cannot_follow_new_map(setup):
    holder, memory, backend, service = setup
    accepted = memory.capture_scene_mapping_state()
    holder["map"] = make_snapshot(epoch="changed_after_acceptance")
    with pytest.raises(GatewayError) as changed:
        service.run_once(expected_capture=accepted)
    assert changed.value.error_code == "scene_scope_changed"
    assert not memory._repository.load_scene_spaces(library_name="lab")


def test_existing_review_name_is_also_protected(setup):
    holder, memory, backend, service = setup
    service.run_once()
    row = service.get_map()["regions"][0]
    memory.review_scene_space(row["scene_space_id"], primary_label="用户命名")
    for x in (2, 3):
        remember(memory, holder, x)
    service.run_once(apply_labels=True)
    assert not backend.calls
    assert service.get_map()["regions"][0]["primary_label"] == "用户命名"


def test_stop_also_fences_explicit_refresh(setup):
    holder, memory, backend, service = setup
    remember(memory, holder)
    backend.hook = lambda: service.stop(wait_sec=0)
    with pytest.raises(GatewayError) as stopped:
        service.run_once(apply_labels=True)
    assert stopped.value.error_code == "scene_mapping_stopped"
    assert all(space.primary_label is None for space in memory._repository.load_scene_spaces(library_name="lab"))


def test_unchanged_background_cycle_does_not_rewrite_region(setup):
    holder, memory, backend, service = setup
    service.run_once()
    first = service.get_map()["regions"][0]
    service.run_once()
    assert service.get_map()["regions"][0]["metadata"]["scene_revision"] == first["metadata"]["scene_revision"]


def test_category_catalog_change_refreshes_cached_decision(setup, monkeypatch):
    holder, memory, backend, service = setup
    remember(memory, holder)
    service.run_once(background=True)
    assert len(backend.calls) == 1
    unchanged = service.run_once(background=True)
    assert len(backend.calls) == 1
    assert unchanged["skipped"][0]["reason"] == "evidence_unchanged"
    monkeypatch.setitem(CATEGORIES, "living_room", "客厅：休闲与会客区域，需实际图像证据。")
    refreshed = service.run_once(background=True)
    assert len(backend.calls) == 2 and refreshed["processed_ids"]


def test_images_require_matching_capture_time_and_camera(setup, tmp_path):
    from datetime import timedelta
    from contracts.artifacts import ArtifactKind
    from services.artifact_store import LocalArtifactStore
    holder, memory, backend, service = setup
    entry = remember(memory, holder)
    store = LocalArtifactStore(str(tmp_path / "artifacts"), "http://local")
    memory._artifact_store = store
    old = store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/png", data=b"old",
        metadata={"camera_id": "other", "frame_timestamp": (utc_now() - timedelta(days=1)).isoformat()}, extension=".png")
    matched = store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/png", data=b"matched",
        metadata={"camera_id": "front", "frame_timestamp": entry.metadata["frame_timestamp"]}, extension=".png")
    entry.artifact_ids = [old.artifact_id, matched.artifact_id]
    images = service._images([entry])
    import base64
    assert len(images) == 1 and base64.b64decode(images[0].split(",")[1]) == b"matched"


def test_stale_vector_job_cannot_overwrite_new_label(setup):
    holder, memory, backend, service = setup
    assert memory._stop_vectorization_worker()
    service.run_once()
    outbox = memory._durable_index.outbox
    worker = outbox.start_worker()
    try:
        old_job = outbox.claim_next(worker)
        assert old_job is not None
        row = service.get_map()["regions"][0]
        memory.update_scene_mapping_label(scene_space_id=row["scene_space_id"], map_name="lab",
            expected_revision=row["metadata"]["scene_revision"], label="新名字", request_id="vec", requested_by="test")
        calls = []
        result = outbox.complete(old_job, publish=lambda tx, claim: calls.append(claim))
        assert not result.published and calls == []
        assert memory._repository.load_scene_spaces(library_name="lab")[0].primary_label == "新名字"
    finally:
        outbox.stop_worker(worker)


def test_multiview_conflict_prevents_automatic_adoption(setup):
    from services.memory.scene_mapping import _consistent_visual_votes
    _, _, _, _service = setup
    result = SceneDecisionResult(category_scores={key: .95 if key == "office" else .01 for key in CATEGORIES},
        score_kind="nli_entailment", model_id="test", raw={"views": [
            {"nli": {key: {"probabilities": {"entailment": .95 if key == winner else .01}}
                     for key in CATEGORIES}} for winner in ("office", "meeting_room")]})
    assert _consistent_visual_votes(result, ["view1", "view2"], "office") is False


def test_missing_jev_key_preserves_readable_status_and_no_worker(setup):
    holder, memory, backend, service = setup
    service.config = SceneMappingConfig(enabled=True, backend="jev_api", base_url="https://api.typesafe.ai/v1", model="jev-1.13.0")
    service._backend = None
    with pytest.raises(GatewayError) as unavailable:
        service.start()
    assert unavailable.value.error_code == "scene_decision_credentials_missing"
    status = service.status()
    assert not status["running"] and status["last_error"]["code"] == "scene_decision_credentials_missing"
    assert "api_key" not in status


@pytest.mark.parametrize("outcome", ["complete", "cancel", "stop", "map_change"])
def test_explicit_refresh_waits_for_busy_worker_and_obeys_task_scope(setup, outcome):
    from concurrent.futures import ThreadPoolExecutor
    from core.task_manager import TaskCancelledError

    holder, memory, backend, service = setup
    remember(memory, holder)
    capture = memory.capture_scene_mapping_state()
    entered, release, waiting, cancelled = (threading.Event() for _ in range(4))

    def hold_first_call():
        if len(backend.calls) == 1:
            entered.set()
            assert release.wait(3)

    def ensure_active():
        waiting.set()
        if cancelled.is_set():
            raise TaskCancelledError("等待中的语义刷新已取消")

    backend.hook = hold_first_call
    service.start()
    pool = ThreadPoolExecutor(max_workers=1)
    try:
        assert entered.wait(2)
        pending = pool.submit(service.run_once, cancel_check=ensure_active, expected_capture=capture)
        assert waiting.wait(1), "显式刷新应保留任务并进入可取消等待"
        assert not pending.done()
        if outcome == "cancel":
            cancelled.set()
            with pytest.raises(TaskCancelledError):
                pending.result(timeout=1)
        elif outcome == "stop":
            service.stop(wait_sec=0)
            with pytest.raises(GatewayError) as error:
                pending.result(timeout=1)
            assert error.value.error_code == "scene_mapping_stopped"
        elif outcome == "map_change":
            holder["map"] = make_snapshot(epoch="changed-while-waiting")
            release.set()
            with pytest.raises(GatewayError) as error:
                pending.result(timeout=2)
            assert error.value.error_code == "scene_scope_changed"
        else:
            release.set()
            assert pending.result(timeout=2)["completed"]
            assert len(backend.calls) == 2
    finally:
        cancelled.set()
        release.set()
        pool.shutdown(wait=True)
        service.stop(wait_sec=2)


def test_public_refresh_task_timeout_during_busy_worker_does_not_run_later(setup):
    from core.task_manager import TaskManager
    from gateways.scene_mapping import _handle
    from contracts.tasks import TaskState

    holder, memory, backend, service = setup
    remember(memory, holder)
    entered, release = threading.Event(), threading.Event()
    backend.hook = lambda: (entered.set(), release.wait(3))
    service.start()
    manager = TaskManager(max_workers=1)
    try:
        assert entered.wait(2)
        runtime = SimpleNamespace(scene_mapping_service=service, memory_service=memory, task_manager=manager)
        accepted = _handle(runtime, "refresh", {"timeout_sec": 1}, requested_by="test")
        manager.wait_for_task(accepted.task_id, timeout=2)
        assert manager.get_task_status(accepted.task_id).state == TaskState.TIMEOUT
        release.set()
        assert len(backend.calls) == 1
    finally:
        release.set()
        manager.shutdown(wait=True)
        service.stop(wait_sec=2)


@pytest.mark.parametrize("activation_metadata", [
    {"backend_mode": "mapping_localization"},
    {"geometry_activation": {"backend_mode": "native_mapping_backend_checkpoint", "backend_switch_applied": True}},
    {},
])
def test_activated_semantics_survive_navigation_live_map_and_clear_on_switch(setup, activation_metadata):
    from services.mapping.service import MappingService
    from contracts.navigation import NavigationGoal
    holder, memory, backend, service = setup
    mapping = MappingService(provider_owner=SimpleNamespace(), state_store=SimpleNamespace(write=lambda *a, **kw: None))
    memory._mapping_service = mapping
    saved = holder["map"]
    def publish(snapshot, **metadata):
        return mapping.ingest_external_snapshot(map_name=snapshot.map_name, source_name=snapshot.source_name,
            source_epoch=snapshot.source_epoch, occupancy_grid=snapshot.occupancy_grid,
            metadata=metadata)
    publish(saved, source_kind="map_asset_bundle", **activation_metadata)
    service.run_once()
    original = service.get_map()
    assert original["region_count"] == 1
    region = memory.current_managed_scene_spaces(mapping.get_semantic_snapshot())[0]
    goal = NavigationGoal(goal_id="区域目标", map_name="lab", target_pose=holder["pose"])
    named = memory.resolve_semantic_region_goal(region.scene_space_id, map_name="lab")
    assert named.target_pose is not None and named.require_goal_orientation is False
    captured = memory.resolve_navigation_region(goal)
    assert captured and captured.scene_space_id == region.scene_space_id
    live = publish(make_snapshot(epoch="navigation", change=True), semantic_reference_identity=original["map_binding"])
    assert live.source_epoch == "navigation"
    assert mapping.get_semantic_snapshot().source_epoch == saved.source_epoch
    assert mapping.get_semantic_snapshot().occupancy_grid == saved.occupancy_grid
    assert live.metadata["semantic_map_binding"] == original["map_binding"]
    assert service.get_map() == original
    assert memory.navigation_region_contains(captured, holder["pose"])
    assert not memory.navigation_region_contains(captured, Pose(frame_id="map", position=Vector3(x=20,y=20)))
    mapping.clear_latest_snapshot()
    assert mapping.get_semantic_snapshot() is None
    publish(make_snapshot(epoch="new", change=True))
    assert all(row["review_status"] == "stale" for row in service.get_map()["regions"])
    assert not memory.navigation_region_contains(captured, holder["pose"])


def test_enabling_adoption_reprocesses_cached_candidate_and_persists_visible_name(setup):
    holder, memory, backend, service = setup
    for x in (2, 3):
        remember(memory, holder, x)
    first = service.run_once(background=True)
    assert first['processed_ids'] and not first['applied_ids']
    row = service.get_map()['regions'][0]
    assert row['metadata']['last_decision']['adoption_blockers'] == ['automatic_adoption_disabled']
    service.run_once(background=True)
    assert len(backend.calls) == 1
    service.config = replace(service.config, auto_apply=True)
    adopted = service.run_once(background=True)
    assert adopted['applied_ids'] and len(backend.calls) == 2
    row = service.get_map()['regions'][0]
    assert row['primary_label'] == '办公室'
    assert row['metadata']['label_source'] == 'model'
    assert row['review_status'] == 'auto_labeled'
    assert any(node['name'] == '办公室' for node in service.graph.query()['nodes'])
    memory._scene_spaces_by_id.clear()
    memory._load_persisted_memory(library_name='lab')
    assert service.get_map()['regions'][0]['primary_label'] == '办公室'
    service.run_once(background=True)
    assert len(backend.calls) == 2


def test_automatic_and_gateway_label_share_commit_notification_and_explicit_priority(setup):
    from gateways.scene_mapping import _handle
    holder, memory, backend, service = setup
    events = []
    service.stream_bus = SimpleNamespace(publish=lambda topic, **value: events.append((topic, value)))
    service.config = replace(service.config, auto_apply=True)
    for x in (2, 3):
        remember(memory, holder, x)
    service.run_once(background=True)
    assert len(events) == 1 and events[-1][1]['payload']['regions'][0]['primary_label'] == '办公室'
    before = service.get_map()['regions'][0]
    args = dict(scene_space_id=before['scene_space_id'], map_name='lab', expected_revision=before['metadata']['scene_revision'],
                label='接待与等候区', request_id='agent_label', requested_by='agent')
    result = _handle(SimpleNamespace(scene_mapping_service=service), 'label',
                     {k:v for k,v in args.items() if k != 'requested_by'}, requested_by='agent')
    saved = result['scene_space']
    assert saved.primary_label == '接待与等候区'
    assert saved.metadata['scene_revision'] == before['metadata']['scene_revision'] + 1
    assert saved.metadata['label_source'] == 'explicit_tool_update' and saved.metadata['label_locked']
    assert saved.review_status == SceneSpaceReviewStatus.AUTO_LABELED
    assert len(events) == 2 and events[-1][1]['payload']['regions'][0]['primary_label'] == '接待与等候区'
    service.update_label(**args)
    assert len(events) == 2  # 幂等重试没有第二次写入/修订通知。
    backend.winner = 'kitchen'
    service.run_once(background=True)
    assert len(backend.calls) == 1
    memory._scene_spaces_by_id.clear()
    memory._load_persisted_memory(library_name='lab')
    row = service.get_map()['regions'][0]
    assert row['primary_label'] == '接待与等候区' and row['metadata']['label_locked']
    assert any(node['name'] == '接待与等候区' for node in service.graph.query()['nodes'])


def test_changed_adoption_threshold_rechecks_candidate_without_relaxing_evidence(setup):
    holder, memory, backend, service = setup
    for x in (2, 3):remember(memory, holder, x)
    service.config = replace(service.config, auto_apply=True, min_label_score=.99)
    result = service.run_once(background=True)
    assert not result['applied_ids']
    assert 'label_score_below_threshold' in service.get_map()['regions'][0]['metadata']['last_decision']['adoption_blockers']
    service.config = replace(service.config, min_label_score=.9)
    assert service.run_once(background=True)['applied_ids']
    assert len(backend.calls) == 2


def test_common_label_commit_rechecks_protection_and_concurrent_idempotence(setup):
    holder, memory, backend, service = setup
    service.run_once()
    capture = memory.capture_scene_mapping_state()
    old = memory.current_managed_scene_spaces(holder['map'])[0]
    revision = old.metadata['scene_revision']
    notifications = []
    args = dict(label='智能体明确命名', source='explicit_tool_update', expected_revision=revision,
                request_id='concurrent-repeat', requested_by='agent', on_committed=lambda:notifications.append(True))
    saved = memory.commit_scene_mapping_label(capture, old, **args)
    duplicate = memory.commit_scene_mapping_label(capture, old, **args)
    assert saved == duplicate and len(notifications) == 1
    with pytest.raises(GatewayError) as error:
        memory.commit_scene_mapping_label(capture, saved, label='办公室', source='model',
                                         expected_revision=saved.metadata['scene_revision'])
    assert error.value.error_code == 'scene_label_protected'
    with pytest.raises(GatewayError) as error:
        memory.commit_scene_mapping_label(capture, old, **{**args, 'label':'同ID不同意图'})
    assert error.value.error_code == 'scene_request_conflict'


@pytest.mark.parametrize('legacy', [False, True])
def test_adopted_label_survives_new_single_view_and_cold_restore(setup, monkeypatch, legacy):
    from datetime import timedelta
    import services.memory.scene_mapping as mapping_module

    holder, memory, backend, service = setup
    start = utc_now()
    monkeypatch.setattr(mapping_module, 'utc_now', lambda: start)
    backend.winner = 'kitchen'
    for x in (2, 3):
        remember(memory, holder, x, timestamp=start)
    assert service.run_once(apply_labels=True)['applied_ids']
    accepted = memory._repository.load_scene_spaces(library_name='lab')[0]
    if legacy:
        # 模拟本次修复之前落库的合法已采纳记录，没有新增依据快照。
        accepted.metadata.pop('adopted_label_evidence', None)
        capture = memory.capture_scene_mapping_state()
        memory.commit_scene_mapping_spaces(capture, [accepted], expected_revisions={
            accepted.scene_space_id: accepted.metadata['scene_revision']})
    memory._scene_spaces_by_id.clear()
    memory._load_persisted_memory(library_name='lab')
    before_revision = service.get_map()['regions'][0]['metadata']['scene_revision']

    later = start + timedelta(seconds=service.config.max_evidence_age_sec + 2)
    monkeypatch.setattr(mapping_module, 'utc_now', lambda: later)
    remember(memory, holder, 2.2, timestamp=later)
    backend.winner = 'bedroom'
    result = service.run_once(apply_labels=True)
    assert result['processed_ids'] and not result['applied_ids']
    row = service.get_map()['regions'][0]
    assert row['primary_label'] == accepted.primary_label
    assert row['review_status'] == 'auto_labeled'
    assert row['metadata']['label_source'] == 'model'
    assert row['metadata']['category'] == 'kitchen'
    assert row['metadata']['scene_revision'] > before_revision
    assert row['metadata']['last_decision']['independent_view_count'] == 1
    assert row['metadata']['last_decision']['adopted'] is False
    assert 'independent_views_insufficient' in row['metadata']['last_decision']['adoption_blockers']
    assert row['candidate_labels'][0]['normalized_label'] == 'bedroom'
    evidence = row['metadata']['adopted_label_evidence']
    assert evidence['decision']['adopted'] is True
    assert evidence['decision']['independent_view_count'] == 2
    assert len(evidence['observation_samples']) == 2
    memory._scene_spaces_by_id.clear()
    memory._load_persisted_memory(library_name='lab')
    assert service.get_map()['regions'][0]['metadata']['adopted_label_evidence'] == evidence
    assert any(node['name'] == accepted.primary_label for node in service.graph.query()['nodes'])

    # 足够的新独立视角仍可修正旧自动名称，无新增分数比较门禁。
    remember(memory, holder, 3.3, timestamp=later)
    assert service.run_once(apply_labels=True)['applied_ids']
    new = service.get_map()['regions'][0]
    assert new['primary_label'] == '卧室'
    assert new['metadata']['adopted_label_evidence']['category'] == 'bedroom'
    assert len(new['metadata']['adopted_label_evidence']['observation_samples']) == 2


@pytest.mark.parametrize('weak_reason', ['low_score', 'adoption_disabled', 'unknown'])
def test_adopted_label_is_not_revoked_by_nonadopting_decision(setup, weak_reason):
    holder, memory, backend, service = setup
    for x in (2, 3):
        remember(memory, holder, x)
    service.run_once(apply_labels=True)
    if weak_reason == 'low_score':
        service.config = replace(service.config, min_label_score=.99)
    elif weak_reason == 'unknown':
        backend.winner = 'unknown'
    result = service.run_once(apply_labels=weak_reason != 'adoption_disabled')
    row = service.get_map()['regions'][0]
    assert result['processed_ids'] and not result['applied_ids']
    assert row['primary_label'] == '办公室' and row['review_status'] == 'auto_labeled'
    assert row['metadata']['last_decision']['adoption_blockers']
    assert row['metadata']['adopted_label_evidence']['decision']['category_scores']['office'] == .95


def _named_location(memory, *, name='接待台', aliases=('前台',), epoch='a', source='test', frame='map', library='lab'):
    from contracts.memory import TaggedLocation
    from contracts.naming import build_location_id
    row = TaggedLocation(location_id=build_location_id("test"), name=name, normalized_name=name,
        aliases=list(aliases), pose=Pose(frame_id=frame), map_name='lab', localization_session_id='old-session',
        metadata={'memory_library_name':library, 'map_source_name':source, 'map_source_epoch':epoch})
    memory._repository.upsert_tagged_location(row)
    return row


@pytest.mark.parametrize('query', ['接待台', '前台'])
def test_exact_same_map_name_resolves_across_localization_sessions(setup, query):
    _, memory, _, _ = setup
    row = _named_location(memory)
    found = memory.resolve_tagged_location(query, map_name='lab', localization_session_id='new-session')
    assert found.location_id == row.location_id and found.localization_session_id == 'old-session'
    assert memory._repository.load_tagged_locations(library_name='lab')[0].localization_session_id == 'old-session'


@pytest.mark.parametrize('mismatch', ['epoch', 'source', 'frame', 'missing_epoch', 'ambiguous', 'fuzzy', 'no_map'])
def test_exact_same_map_fallback_rejects_insufficient_or_ambiguous_scope(setup, mismatch):
    _, memory, _, _ = setup
    _named_location(memory, **({'epoch':'other'} if mismatch=='epoch' else {'source':'other'} if mismatch=='source'
        else {'frame':'odom'} if mismatch=='frame' else {'epoch':None} if mismatch=='missing_epoch' else {}))
    if mismatch=='ambiguous':_named_location(memory)
    assert memory.resolve_tagged_location('接待' if mismatch=='fuzzy' else '接待台',
        map_name=None if mismatch=='no_map' else 'lab', localization_session_id='new-session') is None


def test_same_map_name_fallback_rejects_library_aba_during_query(setup, monkeypatch):
    _, memory, _, _ = setup
    _named_location(memory)
    query = memory.query_location
    def changed(*args, **kwargs):
        result = query(*args, **kwargs)
        memory.activate_memory_library(library_name='lab')
        return result
    monkeypatch.setattr(memory, 'query_location', changed)
    assert memory.resolve_tagged_location('接待台', map_name='lab', localization_session_id='new-session') is None


def test_label_preparation_and_notification_do_not_hold_structural_lock(setup, monkeypatch):
    _, memory, _, service = setup
    service.run_once()
    row = service.get_map()['regions'][0]
    prepare = memory._prepare_managed_scene_space
    observed = []
    def checked(*args, **kwargs):
        assert not memory._instances_lock._is_owned()
        observed.append('prepare')
        return prepare(*args, **kwargs)
    monkeypatch.setattr(memory, '_prepare_managed_scene_space', checked)
    def notify():
        assert not memory._instances_lock._is_owned()
        observed.append('notify')
    memory.update_scene_mapping_label(scene_space_id=row['scene_space_id'], map_name='lab',
        expected_revision=row['metadata']['scene_revision'], label='新名字', request_id='outside-lock', requested_by='test',
        on_committed=notify)
    assert observed == ['prepare', 'notify']


def test_label_sql_failure_rolls_back_record_cache_and_notification(setup, monkeypatch):
    _, memory, _, service = setup
    service.run_once()
    before = memory._repository.load_scene_spaces(library_name='lab')[0]
    commit = memory._repository._commit_prepared_scene_space
    events = []
    def broken(plan):
        commit(plan)
        raise RuntimeError('injected failure after SQL before transaction completion')
    monkeypatch.setattr(memory._repository, '_commit_prepared_scene_space', broken)
    with pytest.raises(RuntimeError, match='injected failure'):
        memory.update_scene_mapping_label(scene_space_id=before.scene_space_id, map_name='lab',
            expected_revision=before.metadata['scene_revision'], label='不能留下', request_id='rollback', requested_by='test',
            on_committed=lambda:events.append(True))
    assert memory._repository.load_scene_spaces(library_name='lab')[0] == before
    assert memory._scene_spaces_by_id[before.scene_space_id] == before
    assert events == []


def test_label_concurrent_writer_wins_during_unlocked_preparation(setup, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    _, memory, _, service = setup
    service.run_once()
    before = memory._repository.load_scene_spaces(library_name='lab')[0]
    prepare = memory._prepare_managed_scene_space
    entered = threading.Event()
    def checked(space, **kwargs):
        if space.primary_label == '迟到模型' and not entered.is_set():
            entered.set()
            with ThreadPoolExecutor(max_workers=1) as pool:
                pool.submit(memory.update_scene_mapping_label, scene_space_id=before.scene_space_id,
                    map_name='lab', expected_revision=before.metadata['scene_revision'], label='并发明确名称',
                    request_id='concurrent-write', requested_by='test').result(timeout=3)
        return prepare(space, **kwargs)
    monkeypatch.setattr(memory, '_prepare_managed_scene_space', checked)
    events = []
    with pytest.raises(GatewayError) as exc:
        memory.commit_scene_mapping_label(memory.capture_scene_mapping_state(), before, label='迟到模型',
            source='model', expected_revision=before.metadata['scene_revision'], on_committed=lambda:events.append(True))
    assert exc.value.error_code == 'scene_revision_conflict'
    assert memory._repository.load_scene_spaces(library_name='lab')[0].primary_label == '并发明确名称'
    assert not events


def test_historical_replay_uses_v18_records_and_keeps_source_unchanged(setup, tmp_path, monkeypatch):
    import importlib.util
    import json
    import sqlite3
    import sys
    from pathlib import Path
    from contracts.artifacts import ArtifactKind
    from services.artifact_store import LocalArtifactStore
    holder, memory, _, _ = setup
    source = tmp_path/'historical'
    source.mkdir()
    bundle = source/'maps/住宅演示地图/current/bundle.json'
    bundle.parent.mkdir(parents=True)
    bundle.write_text(json.dumps({'snapshot':holder['map'].model_dump(mode='json')}, ensure_ascii=False))
    store = LocalArtifactStore(str(source/'artifacts'), 'http://historical.invalid')
    entries=[]
    for x in (2, 3):
        entry=remember(memory, holder, x)
        ref=store.save_bytes(kind=ArtifactKind.IMAGE, mime_type='image/png', data=b'synthetic-image',
            metadata={'camera_id':'front', 'frame_timestamp':entry.metadata['frame_timestamp']}, extension='.png')
        entries.append(entry.model_copy(update={'artifact_ids':[ref.artifact_id]}))
    with sqlite3.connect(str(source/'memory.db')) as db:
        db.execute('CREATE TABLE semantic_memories(map_name TEXT,payload_json TEXT)')
        db.executemany('INSERT INTO semantic_memories VALUES (?,?)', [('lab',entry.model_dump_json()) for entry in entries])
    originals={str(p.relative_to(source)):p.read_bytes() for p in source.rglob('*') if p.is_file()}
    script=Path(__file__).resolve().parents[2]/'scripts/diagnostics/区域自动命名重放.py'
    spec=importlib.util.spec_from_file_location('historical_replay_test', str(script))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    class ReplayBackend(Backend):
        def decide(self, request):
            result=super().decide(request)
            result.raw['views']=[{'nli':{key:{'probabilities':{'entailment':.95 if key=='office' else .01}}
                for key in CATEGORIES}} for _ in request.images]
            return result
        def describe(self):return {'backend':'controlled-test'}
    monkeypatch.setattr(module, 'create_scene_decision_backend', lambda **kwargs:ReplayBackend())
    output=tmp_path/'derived'
    monkeypatch.setattr(sys, 'argv', [str(script),'--source',str(source),'--output',str(output)])
    module.main()
    summary=json.loads((output/'验证汇总.json').read_text())
    assert summary['valid_photo_entries']==2 and summary['model_calls']==1
    assert summary['applied_ids'] and summary['explicit_protection_cold_restore_verified']
    assert {str(p.relative_to(source)):p.read_bytes() for p in source.rglob('*') if p.is_file()}==originals
    with pytest.raises(ValueError, match='拒绝覆盖'):module.main()


def test_identical_concurrent_label_request_is_idempotent_after_unlocked_preparation(setup, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    _, memory, _, service = setup
    service.run_once()
    before=memory._repository.load_scene_spaces(library_name='lab')[0]
    prepare=memory._prepare_managed_scene_space
    entered=threading.Event()
    events=[]
    arguments=dict(scene_space_id=before.scene_space_id, map_name='lab',
        expected_revision=before.metadata['scene_revision'], label='同一请求', request_id='same-concurrent',
        requested_by='agent', on_committed=lambda:events.append(True))
    def checked(space, **kwargs):
        if not entered.is_set():
            entered.set()
            with ThreadPoolExecutor(max_workers=1) as pool:
                pool.submit(memory.update_scene_mapping_label, **arguments).result(timeout=3)
        return prepare(space, **kwargs)
    monkeypatch.setattr(memory, '_prepare_managed_scene_space', checked)
    result=memory.update_scene_mapping_label(**arguments)
    assert result.primary_label=='同一请求' and len(events)==1
    assert result.metadata['scene_revision']==before.metadata['scene_revision']+1
