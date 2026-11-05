"""新库中的语义证据、人工命名、采纳策略及在途取消行为。"""
from dataclasses import replace
from types import SimpleNamespace
from datetime import timedelta
import sqlite3
import threading
import time
import pytest

from contracts.base import utc_now
from contracts.geometry import Pose,Vector3
from contracts.scene_decision import SceneDecisionResult
from core.errors import GatewayError
from core.settings import SceneMappingConfig
from services.memory.semantic_map.scene_mapping import CATEGORIES
from tests.test_a2c_semantic_store import View,remember as basic_remember,new_memory,Backend
from services.memory.semantic_map.scene_mapping import SceneMappingService
from services.geometry.state import MapUpdate,OccupancyGrid
import numpy as np


@pytest.fixture
def setup(tmp_path):
    view=View()
    view.cells=np.full((60,60),100,dtype=np.int8)
    view.cells[5:55,5:55]=0
    view.latest_map=lambda:MapUpdate(OccupancyGrid(view.name,60,60,.1,0,0,'map',view.revision,view.cells.tobytes()),view.cells.tobytes())
    memory=new_memory(tmp_path,view);backend=Backend()
    service=SceneMappingService(memory_service=memory,geometry_state=view,backend=backend,
        config=SceneMappingConfig(enabled=True,auto_apply=False,model='test-model',min_evidence_views=2))
    try:yield SimpleNamespace(view=view,memory=memory,backend=backend,service=service,root=tmp_path)
    finally:service.close();memory.close()


def remember(setup,x=2,*,timestamp=None):
    entry=basic_remember(setup,x)
    updates={'pose':entry.pose.model_copy(update={'position':entry.pose.position.model_copy(update={'y':2})})}
    if timestamp is not None:
        updates['timestamp']=timestamp
        updates['pose']=updates['pose'].model_copy(update={'timestamp':timestamp})
        updates['metadata']={**entry.metadata,'frame_timestamp':timestamp.isoformat()}
    entry=entry.model_copy(update=updates)
    setup.memory.write(lambda store:store.put('memory',entry))
    return entry

def test_unknown_never_adopted(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    backend.winner = "unknown"
    for x in (2, 3):
        remember(setup, x)
    assert not service.run_once(apply_labels=True)["applied_ids"]

def test_user_label_during_model_is_not_overwritten(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    for x in (2, 3):
        remember(setup, x)
    def rename():
        row = service.get_map()["regions"][0]
        service.update_label(scene_space_id=row["scene_space_id"], map_name="实验室",
            expected_revision=row["metadata"]["scene_revision"], label="会议室", requested_by="test")
    backend.hook = rename
    result = service.run_once(apply_labels=True)
    assert result["failures"][0]["code"] == "scene_revision_conflict"
    assert service.get_map()["regions"][0]["primary_label"] == "会议室"

def test_stop_and_bounded_single_worker_reject_late_result(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    remember(setup)
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
    assert all(space.primary_label is None for space in memory.records("scene_space"))

def test_locate_reports_geometry_and_unknown(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    service.run_once()
    assert service.locate()["status"] == "located"
    setup.view.position = replace(setup.view.position, x=20, y=20)
    assert service.locate()["status"] == "unknown"

def test_existing_review_name_is_also_protected(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    service.run_once()
    row = service.get_map()["regions"][0]
    memory.review_scene_space(row["scene_space_id"], map_name="实验室", primary_label="用户命名")
    for x in (2, 3):
        remember(setup, x)
    service.run_once(apply_labels=True)
    assert service.get_map()["regions"][0]["primary_label"] == "用户命名"

def test_stop_also_fences_explicit_refresh(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    remember(setup)
    backend.hook = lambda: service.stop(wait_sec=0)
    with pytest.raises(GatewayError) as stopped:
        service.run_once(apply_labels=True)
    assert stopped.value.error_code == "scene_mapping_stopped"
    assert all(space.primary_label is None for space in memory.records("scene_space"))

def test_unchanged_background_cycle_does_not_rewrite_region(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    service.run_once()
    first = service.get_map()["regions"][0]
    service.run_once()
    assert service.get_map()["regions"][0]["metadata"]["scene_revision"] == first["metadata"]["scene_revision"]

def test_multiview_conflict_prevents_automatic_adoption(setup):
    from services.memory.semantic_map.scene_mapping import _consistent_visual_votes
    _service = setup.service
    result = SceneDecisionResult(category_scores={key: .95 if key == "office" else .01 for key in CATEGORIES},
        score_kind="nli_entailment", model_id="test", raw={"views": [
            {"nli": {key: {"probabilities": {"entailment": .95 if key == winner else .01}}
                     for key in CATEGORIES}} for winner in ("office", "meeting_room")]})
    assert _consistent_visual_votes(result, ["view1", "view2"], "office") is False

def test_missing_jev_key_preserves_readable_status_and_no_worker(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    service.config = SceneMappingConfig(enabled=True, backend="jev_api", base_url="https://api.typesafe.ai/v1", model="jev-1.13.0")
    service._backend = None
    with pytest.raises(GatewayError) as unavailable:
        service.start()
    assert unavailable.value.error_code == "scene_decision_credentials_missing"
    status = service.status()
    assert not status["running"] and status["last_error"]["code"] == "scene_decision_credentials_missing"
    assert "api_key" not in status

def test_enabling_adoption_reprocesses_cached_candidate_and_persists_visible_name(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    for x in (2, 3):
        remember(setup, x)
    first = service.run_once(background=True)
    assert first['processed_ids'] and not first['applied_ids']
    row = service.get_map()['regions'][0]
    assert row['metadata']['last_decision']['adoption_blockers'] == ['automatic_adoption_disabled']
    service.run_once(background=True)
    service.config = replace(service.config, auto_apply=True)
    adopted = service.run_once(background=True)
    assert adopted['applied_ids']
    row = service.get_map()['regions'][0]
    assert row['primary_label'] == '办公室'
    assert row['metadata']['label_source'] == 'model'
    assert row['review_status'] == 'auto_labeled'
    assert any(node['name'] == '办公室' for node in service.graph.query()['nodes'])
    assert service.get_map()['regions'][0]['primary_label'] == '办公室'
    service.run_once(background=True)

def test_changed_adoption_threshold_rechecks_candidate_without_relaxing_evidence(setup):
    memory, backend, service = setup.memory, setup.backend, setup.service
    for x in (2, 3):remember(setup, x)
    service.config = replace(service.config, auto_apply=True, min_label_score=.99)
    result = service.run_once(background=True)
    assert not result['applied_ids']
    assert 'label_score_below_threshold' in service.get_map()['regions'][0]['metadata']['last_decision']['adoption_blockers']
    service.config = replace(service.config, min_label_score=.9)
    assert service.run_once(background=True)['applied_ids']

def test_adopted_label_survives_new_single_view_and_cold_restore(setup, monkeypatch):
    from datetime import timedelta
    import services.memory.semantic_map.scene_mapping as mapping_module

    memory, backend, service = setup.memory, setup.backend, setup.service
    start = utc_now()
    monkeypatch.setattr(mapping_module, 'utc_now', lambda: start)
    backend.winner = 'kitchen'
    for x in (2, 3):
        remember(setup, x, timestamp=start)
    assert service.run_once(apply_labels=True)['applied_ids']
    accepted = memory.records('scene_space')[0]
    before_revision = service.get_map()['regions'][0]['metadata']['scene_revision']

    later = start + timedelta(seconds=service.config.max_evidence_age_sec + 2)
    monkeypatch.setattr(mapping_module, 'utc_now', lambda: later)
    remember(setup, 2.2, timestamp=later)
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
    assert cold_map(setup)['regions'][0]['metadata']['adopted_label_evidence'] == evidence
    assert any(node['name'] == accepted.primary_label for node in service.graph.query()['nodes'])

    # 足够的新独立视角仍可修正旧自动名称，无新增分数比较门禁。
    remember(setup, 3.3, timestamp=later)
    assert service.run_once(apply_labels=True)['applied_ids']
    new = service.get_map()['regions'][0]
    assert new['primary_label'] == '卧室'
    assert new['metadata']['adopted_label_evidence']['category'] == 'bedroom'
    assert len(new['metadata']['adopted_label_evidence']['observation_samples']) == 2

@pytest.mark.parametrize('weak_reason', ['low_score', 'adoption_disabled', 'unknown'])
def test_adopted_label_is_not_revoked_by_nonadopting_decision(setup, weak_reason):
    memory, backend, service = setup.memory, setup.backend, setup.service
    for x in (2, 3):
        remember(setup, x)
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

def test_no_evidence_does_not_adopt_and_category_catalog_refreshes_decision(setup,monkeypatch):
    result=setup.service.run_once(apply_labels=True)
    assert not result['applied_ids'] and result['skipped'][0]['reason']=='no_valid_observation_evidence'
    remember(setup)
    setup.service.run_once(background=True)
    unchanged=setup.service.run_once(background=True)
    assert unchanged['skipped'][0]['reason']=='evidence_unchanged'
    monkeypatch.setitem(CATEGORIES,'living_room','客厅：休闲与会客区域，需实际图像证据。')
    assert setup.service.run_once(background=True)['processed_ids']


def test_label_sql_failure_rolls_back_record_and_next_write_can_succeed(setup,monkeypatch):
    from services.memory.store import MemoryStore
    setup.service.run_once()
    before=setup.memory.records('scene_space')[0]
    put=MemoryStore.put
    def broken(store,kind,record):
        result=put(store,kind,record)
        if kind=='scene_space':raise RuntimeError('injected failure after SQL before transaction completion')
        return result
    with monkeypatch.context() as patched:
        patched.setattr(MemoryStore,'put',broken)
        with pytest.raises(RuntimeError,match='injected failure'):
            setup.service.update_label(scene_space_id=before.scene_space_id,map_name='实验室',
                expected_revision=before.metadata['scene_revision'],label='不能留下',requested_by='test')
    assert setup.memory.records('scene_space')[0]==before
    saved=setup.service.update_label(scene_space_id=before.scene_space_id,map_name='实验室',
        expected_revision=before.metadata['scene_revision'],label='事务后可写',requested_by='test')
    assert saved.primary_label=='事务后可写'


def test_empty_map_response_keeps_all_public_fields(setup):
    setup.view.cells[:]=100
    setup.service.run_once()
    result=setup.service.get_map()
    assert result['regions']==[] and result['region_count']==0
    assert result['map_binding']['map_name']=='实验室'
    assert result['navigation_checked'] is False


def test_committed_region_updates_publish_detached_full_json(setup):
    setup.service.run_once()
    before=setup.service.get_map();row=before['regions'][0]
    frames=[]
    setup.service.stream_bus=SimpleNamespace(publish=lambda topic,**kw:frames.append((topic,kw['payload'])))
    for i in range(4):
        setup.service.update_label(scene_space_id=row['scene_space_id'],map_name='实验室',
            expected_revision=row['metadata']['scene_revision']+i,label='区域%d'%i,requested_by='test')
        assert frames[-1][0]=='memory.scene_spaces'
        assert frames[-1][1]==setup.service.get_map()
    assert [frame[1]['regions'][0]['primary_label'] for frame in frames]==['区域%d'%i for i in range(4)]
    assert before['regions'][0]['primary_label'] is None


def cold_map(setup):
    memory=new_memory(setup.root,setup.view)
    service=SceneMappingService(memory_service=memory,geometry_state=setup.view,
        config=SceneMappingConfig(enabled=False),backend=Backend())
    try:return service.get_map()
    finally:service.close();memory.close()


@pytest.mark.parametrize('outcome',['complete','cancel','stop','map_change'])
def test_explicit_refresh_waits_for_busy_worker_and_obeys_task_scope(setup,outcome):
    from concurrent.futures import ThreadPoolExecutor
    from core.tasks import Cancelled
    remember(setup)
    capture=setup.service.memory.capture_scene_mapping_state()
    entered,release,waiting,cancelled=(threading.Event() for _ in range(4))
    def held():entered.set();assert release.wait(3)
    def check():
        waiting.set()
        if cancelled.is_set():raise Cancelled('等待中的语义刷新已取消')
    setup.backend.hook=held
    setup.service.start()
    with ThreadPoolExecutor(max_workers=1) as pool:
        try:
            assert entered.wait(2)
            pending=pool.submit(setup.service.run_once,cancel_check=check,expected_capture=capture)
            assert waiting.wait(1) and not pending.done()
            if outcome=='cancel':
                cancelled.set()
                with pytest.raises(Cancelled):pending.result(timeout=1)
            elif outcome=='stop':
                setup.service.stop(wait_sec=0)
                with pytest.raises(GatewayError) as error:pending.result(timeout=1)
                assert error.value.error_code=='scene_mapping_stopped'
            elif outcome=='map_change':
                setup.view.switch('changed-while-waiting');setup.memory.write(lambda store:None)
                release.set()
                with pytest.raises(GatewayError) as error:pending.result(timeout=2)
                assert error.value.error_code=='scene_scope_changed'
            else:
                release.set()
                assert pending.result(timeout=2)['completed']
        finally:
            cancelled.set();release.set();setup.service.stop(wait_sec=2)


def test_public_refresh_deadline_while_waiting_does_not_adopt_later(setup):
    from core.capability import CapabilityTable
    from core.event_bus import EventBus
    from core.tasks import TaskRunner,TaskState
    from gateways.invoke import invoke
    from gateways.capabilities.memory import CAPABILITIES
    remember(setup)
    entered,release=threading.Event(),threading.Event()
    setup.backend.hook=lambda:(entered.set(),release.wait(3))
    setup.service.start()
    events=EventBus()
    platform=SimpleNamespace(scene_mapping=setup.service,memory=setup.memory,events=events,
        ensure=lambda *_:None,capabilities=CapabilityTable(),robot=SimpleNamespace(manifest=SimpleNamespace(robot_name='test')))
    for cap in CAPABILITIES:platform.capabilities.register(cap)
    platform.tasks=TaskRunner(platform)
    try:
        assert entered.wait(2)
        task=invoke(platform,'refresh_semantic_map',{'timeout_sec':1,'apply_labels':True},'test')['task']
        end=time.monotonic()+2
        while time.monotonic()<end and platform.tasks.status(task['task_id']).state in (TaskState.RUNNING,TaskState.CANCELLING):
            threading.Event().wait(.01)
        assert platform.tasks.status(task['task_id']).state is TaskState.TIMED_OUT
        release.set();setup.service.stop(wait_sec=2)
        assert all(space.primary_label is None for space in setup.memory.records('scene_space'))
    finally:
        release.set();platform.tasks.shutdown();events.shutdown();setup.service.stop(wait_sec=2)
