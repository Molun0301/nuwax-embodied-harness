#!/usr/bin/env python3
"""Offline graph source generations, bounded captures and geometry ownership."""
from __future__ import annotations
from pathlib import Path
from datetime import datetime, timezone, timedelta
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.base import utc_now
from contracts.geometry import Pose
from contracts.maps import OccupancyGrid
from contracts.runtime_views import MapSnapshot
from contracts.spatial_memory import SceneSpaceReviewStatus
from scripts import check_memory_provider_modes as helpers
from scripts.check_memory_storage_transactions import records
from services.memory.room_segmentation import RoomSegmentationService
from services.memory.scene_mapping_state import snapshot_binding
from services.memory.sqlite_storage import SQLiteTransaction
from services.memory.scene_mapping import SceneMappingService, _evidence_inputs
from services.memory.scene_graph import SceneGraphService


def snapshot(sequence=1,epoch='epoch'):
    cells=[100]*400
    for y in range(1,19):
        for x in range(1,19): cells[y*20+x]=0
    return MapSnapshot(map_name='library',source_name='synthetic',source_epoch=epoch,
        frame_id='map',update_sequence=sequence,occupancy_grid=OccupancyGrid(map_id='library',frame_id='map',
        width=20,height=20,resolution_m=.1,origin=Pose(frame_id='map'),data=cells))


class SceneGraphSourceChecks(unittest.TestCase):
    def fixture(self,path=':memory:'):
        service=helpers.ProviderModeChecks.fixture(self,path=path)
        current=snapshot(); holder=SimpleNamespace(current=current)
        service._mapping_service=SimpleNamespace(get_latest_snapshot=lambda:holder.current,get_semantic_snapshot=lambda:holder.current)
        return service,holder

    def scene(self,service,current):
        scene=RoomSegmentationService().segment(current,map_name='library')[0]
        scene.metadata.update(scene_mapping_managed=True,map_binding=snapshot_binding(current),memory_library_name='library')
        return service._repository.upsert_scene_space(scene)

    def test_geometry_revision_is_owner_assigned_and_label_updates_reuse_it(self):
        service,holder=self.fixture(); scene=self.scene(service,holder.current)
        generation=service._repository.scene_graph_generations('library')
        changed=scene.model_copy(update={'primary_label':'Office','metadata':{**scene.metadata,'geometry_revision':'forged'}},deep=True)
        written=service._repository.upsert_scene_space(changed)
        self.assertEqual(written.metadata['geometry_revision'],scene.metadata['geometry_revision'])
        self.assertEqual(service._repository.scene_graph_generations('library'),(generation[0]+1,generation[1]))
        changed=written.model_copy(deep=True); changed.boundary.polygon_points[0].x+=.1
        newer=service._repository.upsert_scene_space(changed)
        self.assertNotEqual(newer.metadata['geometry_revision'],written.metadata['geometry_revision'])
        self.assertEqual(service._repository.scene_graph_generations('library'),(generation[0]+2,generation[1]+1))

    def test_bound_scope_and_eligibility_change_geometry_generation(self):
        service,holder=self.fixture(); scene=self.scene(service,holder.current)
        previous=service._repository.scene_graph_generations('library')
        rejected=service._repository.upsert_scene_space(scene.model_copy(update={'review_status':SceneSpaceReviewStatus.REJECTED}))
        self.assertEqual(service._repository.scene_graph_generations('library')[1],previous[1]+1)
        self.assertEqual(rejected.metadata['geometry_revision'],scene.metadata['geometry_revision'])
        restored=service._repository.upsert_scene_space(rejected.model_copy(update={'review_status':SceneSpaceReviewStatus.CANDIDATE}))
        altered=restored.model_copy(update={'metadata':{**restored.metadata,'map_binding':snapshot_binding(snapshot(2))}})
        service._repository.upsert_scene_space(altered)
        self.assertEqual(service._repository.scene_graph_generations('library')[1],previous[1]+3)

    def test_source_counters_cover_relevant_writes_only_and_rollback(self):
        service,holder=self.fixture(); repo=service._repository
        self.assertEqual(repo.scene_graph_generations('library'),(0,0))
        repo.upsert_tagged_location(records()[0][3]); repo.upsert_observation_event(records()[3][3])
        self.assertEqual(repo.scene_graph_generations('library'),(0,0))
        repo.upsert_semantic_memory(records()[1][3]); repo.upsert_semantic_instance(records()[2][3])
        before=repo.scene_graph_generations('library'); self.assertEqual(before,(2,0))
        with self.assertRaises(RuntimeError),repo.storage.transaction():
            self.scene(service,holder.current)
            self.assertEqual(repo.scene_graph_generations('library'),(3,1))
            raise RuntimeError('rollback')
        self.assertEqual(repo.scene_graph_generations('library'),before)
        repo.delete_semantic_memories([records()[1][3].memory_id])
        self.assertEqual(repo.scene_graph_generations('library'),(3,0))

    def test_cold_capture_reads_own_writes_without_publishing_cache(self):
        service,holder=self.fixture(); repo=service._repository
        with repo.storage.transaction():
            scene=self.scene(service,holder.current)
            repo.upsert_semantic_memory(records()[1][3])
            self.assertFalse(service._scene_spaces_by_id)
            capture=service.capture_scene_graph_state()
            self.assertEqual(capture['spaces'][0].scene_space_id,scene.scene_space_id)
            self.assertEqual(capture['entry_total'],1)

    def test_delayed_callback_does_not_hide_committed_sources(self):
        service,holder=self.fixture(); callbacks=[]
        with patch.object(SQLiteTransaction,'after_commit',autospec=True,side_effect=lambda tx,cb,**kw:callbacks.append(cb)):
            scene=self.scene(service,holder.current)
            service._repository.upsert_semantic_memory(records()[1][3])
        self.assertFalse(service._scene_spaces_by_id)
        capture=service.capture_scene_graph_state()
        self.assertEqual(capture['spaces'][0].scene_space_id,scene.scene_space_id)
        self.assertEqual(capture['entry_total'],1)
        for cb in callbacks:cb()
        with self.assertRaises(Exception):service.assert_scene_graph_capture(capture)

    def test_other_owner_source_write_invalidates_stamp_and_cold_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/'memory.db'); first,holder=self.fixture(path); second,_=self.fixture(path)
            before=first.capture_scene_graph_stamp()
            second._repository.upsert_semantic_memory(records()[1][3])
            with self.assertRaises(Exception):first.assert_scene_graph_capture(before)
            self.assertEqual(first.capture_scene_graph_state()['entry_total'],1)

    def test_stamp_does_not_scan_or_decode_sources(self):
        service,_=self.fixture()
        with patch.object(service._repository,'_load_record',side_effect=AssertionError('decoded')), \
                patch.object(service._repository,'capture_scene_graph_sources',side_effect=AssertionError('capture')):
            one=service.capture_scene_graph_stamp(); two=service.capture_scene_graph_stamp()
            self.assertEqual(one['stamp_key'],two['stamp_key'])

    def test_source_decode_occurs_outside_owned_reader_lock(self):
        service,holder=self.fixture(); self.scene(service,holder.current)
        original=service._repository._load_record
        def decode(*args,**kwargs):
            self.assertFalse(service._repository.storage._lock._is_owned())
            return original(*args,**kwargs)
        with patch.object(service._repository,'_load_record',side_effect=decode):
            self.assertEqual(len(service.capture_scene_graph_state()['spaces']),1)

    def test_lifecycle_deadline_and_live_only_overlay_are_captured(self):
        service,_=self.fixture(); instance=records()[2][3].model_copy(update={'last_seen_ts':utc_now()})
        service._repository.upsert_semantic_instance(instance)
        old=service._semantic_instances_by_id[instance.instance_id]
        live=old.model_copy(update={'display_name':'live position'},deep=True)
        service._publish_live_instance(live,expected=old)
        capture=service.capture_scene_graph_state()
        self.assertEqual(capture['instances'][0]['display_name'],'live position')
        self.assertGreater(capture['valid_until'],utc_now().timestamp())
        self.assertEqual(capture['instance_total'],1)

    def test_history_disabled_hides_pre_activation_rows(self):
        service,holder=self.fixture(); self.scene(service,holder.current)
        service._repository.upsert_semantic_memory(records()[1][3])
        with service._instances_lock:
            service._clear_active_memory_state()
            service._durable_index.load_history=False
        self.assertEqual(service.capture_scene_graph_state()['entry_total'],0)
        self.assertFalse(service.capture_scene_graph_state()['spaces'])

    def test_room_overlap_preserves_legacy_id_and_geometry_revision(self):
        service,holder=self.fixture(); room=self.scene(service,holder.current)
        room.scene_space_id='ssp_geometry_legacy_19700101T000000Z_00000001'
        room.metadata.pop('geometry_revision')
        segmenter=RoomSegmentationService(); next_rooms=segmenter.segment(holder.current,map_name='library',existing_spaces=[room])
        self.assertEqual(next_rooms[0].scene_space_id,room.scene_space_id)
        self.assertTrue(getattr(next_rooms[0],'_geometry_unchanged'))
        self.assertNotIn('geometry_digest',next_rooms[0].metadata)
        again=segmenter.segment(holder.current,map_name='library',existing_spaces=next_rooms)
        self.assertEqual(again[0].metadata['geometry_revision'],next_rooms[0].metadata['geometry_revision'])

    def test_legacy_label_survives_unchanged_geometry_binding(self):
        service,holder=self.fixture(); room=self.scene(service,holder.current)
        room.metadata.pop('geometry_revision');room.primary_label='reviewed label'
        candidates=RoomSegmentationService().segment(holder.current,map_name='library',existing_spaces=[room])
        owner=SceneMappingService(memory_service=service,config=SimpleNamespace(min_area_m2=2,doorway_width_m=1.2,max_cells=2000000))
        bound=owner._bind_regions({'spaces':[room],'binding':snapshot_binding(holder.current)},candidates)
        self.assertEqual(bound[0].primary_label,'reviewed label')

    def test_real_graph_reuses_geometry_for_label_change_and_not_visible_membership(self):
        service,holder=self.fixture(); room=self.scene(service,holder.current)
        graph=SceneGraphService(memory_service=service,max_cells=2000000)
        first=graph.query(); old_geometry=graph._geometry
        updated=room.model_copy(update={'primary_label':'New label'})
        service._repository.upsert_scene_space(updated)
        second=graph.query()
        self.assertIs(graph._geometry,old_geometry)
        self.assertNotEqual(first['graph_revision'],second['graph_revision'])
        self.assertEqual(second['nodes'][0]['name'],'New label')
        with patch.object(service,'capture_scene_graph_state',side_effect=AssertionError('warm source capture')):
            self.assertEqual(graph.query()['graph_revision'],second['graph_revision'])

    def test_hidden_scene_delayed_admission_rebuilds_geometry_once(self):
        service,holder=self.fixture();service._durable_index.load_history=False
        callbacks=[]
        with patch.object(SQLiteTransaction,'after_commit',autospec=True,side_effect=lambda tx,cb,**kw:callbacks.append(cb)):
            room=self.scene(service,holder.current)
        graph=SceneGraphService(memory_service=service,max_cells=2000000)
        self.assertFalse(graph.query()['nodes']);empty_geometry=graph._geometry
        for callback in callbacks:callback()
        current=graph.query();self.assertEqual(current['coverage']['region_count'],1)
        self.assertIsNot(graph._geometry,empty_geometry)
        occupied=graph._geometry
        service._repository.upsert_scene_space(room.model_copy(update={'primary_label':'Visible label'}))
        graph.query();self.assertIs(graph._geometry,occupied)

    def test_graph_from_rolled_back_writer_does_not_poison_committed_cache(self):
        service,holder=self.fixture();graph=SceneGraphService(memory_service=service,max_cells=2000000)
        initial=graph.query()
        with self.assertRaises(RuntimeError),service._repository.storage.transaction():
            self.scene(service,holder.current)
            self.assertEqual(graph.query()['coverage']['region_count'],1)
            raise RuntimeError('rollback')
        self.assertEqual(graph.query()['graph_revision'],initial['graph_revision'])
        self.assertFalse(graph.query()['nodes'])

    def test_refresh_assigns_room_revision_once_without_repeated_source_writes(self):
        service,holder=self.fixture()
        owner=SceneMappingService(memory_service=service,config=SimpleNamespace(min_area_m2=2,doorway_width_m=1.2,max_cells=2000000))
        first_capture,first=owner._refresh_regions(lambda:None)
        generation=service._repository.scene_graph_generations('library')
        second_capture,second=owner._refresh_regions(lambda:None)
        self.assertEqual(first[0].scene_space_id,second[0].scene_space_id)
        self.assertEqual(first[0].metadata['geometry_revision'],second[0].metadata['geometry_revision'])
        self.assertEqual(service._repository.scene_graph_generations('library'),generation)

    def test_geometry_membership_change_between_libraries_invalidates_both(self):
        from services.memory.repository import MemoryRepository
        service,holder=self.fixture();scene=self.scene(service,holder.current)
        standalone=MemoryRepository(storage=service._repository.storage);self.addCleanup(standalone.close)
        before=standalone.scene_graph_generations('library')
        moved=scene.model_copy(update={'metadata':{**scene.metadata,'memory_library_name':'other'}})
        standalone.upsert_scene_space(moved)
        self.assertEqual(standalone.scene_graph_generations('library')[1],before[1]+1)
        self.assertEqual(standalone.scene_graph_generations('other')[1],1)

    def test_unversioned_legacy_room_delete_or_reject_invalidates_geometry(self):
        for action in ('delete','reject'):
            with self.subTest(action=action):
                service,holder=self.fixture();room=self.scene(service,holder.current)
                with service._repository.storage.transaction() as tx:
                    tx.execute("UPDATE scene_spaces SET graph_geometry_revision='' WHERE scene_space_id=?",(room.scene_space_id,))
                graph=SceneGraphService(memory_service=service,max_cells=2000000)
                self.assertEqual(graph.query()['coverage']['region_count'],1)
                geometry=graph._geometry
                if action=='delete':service._repository.delete_scene_spaces([room.scene_space_id])
                else:service._repository.upsert_scene_space(room.model_copy(update={'review_status':SceneSpaceReviewStatus.REJECTED}))
                result=graph.query();self.assertEqual(result['coverage']['region_count'],0)
                self.assertIsNot(graph._geometry,geometry)
                self.assertEqual(graph._geometry.region_ids,[])

    def test_cold_bounds_order_offset_instances_before_limiting(self):
        from services.memory.repository import MemoryRepository
        service,_=self.fixture(); raw=MemoryRepository(storage=service._repository.storage);self.addCleanup(raw.close)
        base=records()[2][3]
        older=datetime(2026,1,2,tzinfo=timezone(timedelta(hours=14)))
        newer=datetime(2026,1,1,23,tzinfo=timezone(timedelta(hours=-12)))
        with raw.storage.transaction():
            for index in range(2001):
                item=base.model_copy(update={'instance_id':'ins_fixture_20260101T000000Z_%08x' % index,
                    'last_seen_ts':newer if index==2000 else older})
                raw.upsert_semantic_instance(item)
        capture=service.capture_scene_graph_state()
        self.assertEqual(capture['instance_total'],2001)
        self.assertEqual(len(capture['instances']),1000)
        self.assertEqual(capture['instances'][0]['instance_id'],'ins_fixture_20260101T000000Z_%08x' % 2000)

    def test_memory_selection_uses_current_payload_timestamp(self):
        service,_=self.fixture(); repo=service._repository;first=records()[1][3]
        second=first.model_copy(update={'memory_id':first.memory_id.replace('00000001','00000002'),
            'timestamp':datetime(2026,2,1,tzinfo=timezone.utc)})
        repo.upsert_semantic_memory(first);repo.upsert_semantic_memory(second)
        changed=repo.load_semantic_memories()[0].model_copy(update={'timestamp':datetime(2026,3,1,tzinfo=timezone.utc)})
        repo.upsert_semantic_memory(changed)
        self.assertEqual(service.capture_scene_graph_state()['entries'][0].memory_id,first.memory_id)
        self.assertEqual(service.capture_scene_mapping_state()['entries'][0].memory_id,first.memory_id)

    def test_timestamp_collation_preserves_microseconds_offsets_and_invalid_order(self):
        from services.memory.scene_revisions import compare_scene_timestamps
        self.assertEqual(compare_scene_timestamps('2026-01-01T14:00:00.123456+14:00','2026-01-01T00:00:00.123456Z'),0)
        self.assertLess(compare_scene_timestamps('2026-01-01T00:00:00.123455Z','2026-01-01T00:00:00.123456Z'),0)
        self.assertLess(compare_scene_timestamps('malformed','2026-01-01T00:00:00Z'),0)
        # Valid extreme-offset contracts cannot overflow Python datetime normalization.
        self.assertLess(compare_scene_timestamps('0001-01-01T00:00:00+14:00','0001-01-01T00:00:00Z'),0)
        with patch('services.memory.scene_revisions._TIMESTAMP_ADAPTER.validate_python',side_effect=RuntimeError('unexpected')):
            with self.assertRaisesRegex(RuntimeError,'unexpected'):
                compare_scene_timestamps('x','y')

    def test_collation_is_registered_for_file_reader_and_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            service,_=self.fixture(str(Path(directory)/'memory.db'))
            sql="SELECT CASE WHEN ? COLLATE NEH_SCENE_TIMESTAMP > ? THEN 1 ELSE 0 END"
            values=('2026-01-01T00:00:00.000002Z','2026-01-01T00:00:00.000001Z')
            with service._repository.storage.transaction() as tx:self.assertEqual(tx.execute(sql,values).fetchone()[0],1)
            with service._repository.storage.read_snapshot() as tx:self.assertEqual(tx.execute(sql,values).fetchone()[0],1)

    def test_legacy_raw_live_overlay_has_exact_source_token_and_no_double_count(self):
        from services.memory.repository import MemoryRepository
        service,_=self.fixture();raw=MemoryRepository(storage=service._repository.storage);self.addCleanup(raw.close)
        original=records()[2][3];raw.upsert_semantic_instance(original)
        with raw.storage.transaction() as tx:
            tx.execute("UPDATE semantic_instances SET graph_source_revision='' WHERE instance_id=?",(original.instance_id,))
        loaded=service._repository.load_semantic_instances()[0]
        with service._instances_lock:service._cache_artifact_record('instance',loaded)
        live=loaded.model_copy(update={'display_name':'live'})
        service._publish_live_instance(live,expected=loaded)
        first=service.capture_scene_graph_state()
        self.assertEqual(first['instance_total'],1);self.assertEqual(first['instances'][0]['display_name'],'live')
        raw.upsert_semantic_instance(original.model_copy(update={'display_name':'new durable'}))
        second=service.capture_scene_graph_state()
        self.assertEqual(second['instance_total'],1);self.assertEqual(second['instances'][0]['display_name'],'new durable')
        raw.delete_semantic_instances([original.instance_id])
        third=service.capture_scene_graph_state()
        self.assertEqual(third['instance_total'],1);self.assertEqual(third['instances'][0]['display_name'],'live')

    def test_legacy_evidence_generation_changes_only_with_memory_sources(self):
        from services.memory.repository import MemoryRepository
        service,holder=self.fixture();raw=MemoryRepository(storage=service._repository.storage);self.addCleanup(raw.close)
        original=records()[1][3];raw.upsert_semantic_memory(original)
        first=_evidence_inputs(service.capture_scene_mapping_state()['entries'],'model')
        self.assertEqual(first['entries'][0]['source_revision'][0],'memory_source')
        self.scene(service,holder.current)
        second=_evidence_inputs(service.capture_scene_mapping_state()['entries'],'model')
        self.assertEqual(first,second)
        raw.upsert_semantic_memory(original.model_copy(update={'summary':'changed legacy evidence'}))
        third=_evidence_inputs(service.capture_scene_mapping_state()['entries'],'model')
        self.assertNotEqual(first,third)

    def test_standalone_update_overrides_live_with_existing_index_sidecar(self):
        from services.memory.repository import MemoryRepository
        service,_=self.fixture();service._repository.upsert_semantic_instance(records()[2][3])
        old=service._semantic_instances_by_id[records()[2][3].instance_id]
        service._publish_live_instance(old.model_copy(update={'display_name':'old live'}),expected=old)
        raw=MemoryRepository(storage=service._repository.storage);self.addCleanup(raw.close)
        raw.upsert_semantic_instance(records()[2][3].model_copy(update={'display_name':'new standalone source'}))
        capture=service.capture_scene_graph_state()
        self.assertEqual(capture['instances'][0]['display_name'],'new standalone source')
        self.assertEqual(capture['instance_total'],1)
        raw.delete_semantic_instances([old.instance_id])
        self.assertEqual(service.capture_scene_graph_state()['instance_total'],1)

    def test_live_overlay_is_applied_before_bounded_durable_selection(self):
        service,_=self.fixture();base=records()[2][3];moment=utc_now()
        with service._repository.storage.transaction():
            for index in range(1001):
                service._repository.upsert_semantic_instance(base.model_copy(update={
                    'instance_id':'ins_overlay_20260101T000000Z_%08x' % index,
                    'last_seen_ts':moment+timedelta(seconds=2000-index)}))
        for index in range(1000):
            identifier='ins_overlay_20260101T000000Z_%08x' % index
            original=service._semantic_instances_by_id[identifier]
            service._publish_live_instance(original.model_copy(update={'last_seen_ts':moment-timedelta(seconds=100)}),expected=original)
        capture=service.capture_scene_graph_state()
        self.assertEqual(capture['instance_total'],1001)
        self.assertEqual(len(capture['instances']),1000)
        self.assertEqual(capture['instances'][0]['instance_id'],'ins_overlay_20260101T000000Z_%08x' % 1000)

    def test_managed_commit_rejects_standalone_source_change_with_same_scene_revision(self):
        from services.memory.repository import MemoryRepository
        service,holder=self.fixture();self.scene(service,holder.current)
        captured=service.capture_scene_mapping_state()['spaces'][0]
        raw=MemoryRepository(storage=service._repository.storage);self.addCleanup(raw.close)
        raw.upsert_scene_space(captured.model_copy(update={'primary_label':'authoritative replacement'}))
        before=service._repository.scene_graph_generations('library')
        stale=captured.model_copy(update={'primary_label':'late model label'})
        with self.assertRaises(Exception) as raised:
            service._persist_managed_scene_space(stale,expected_revision=int(captured.metadata.get('scene_revision',0)))
        self.assertEqual(getattr(raised.exception,'error_code',None),'scene_revision_conflict')
        self.assertEqual(service._repository.load_scene_spaces()[0].primary_label,'authoritative replacement')
        self.assertEqual(service._repository.scene_graph_generations('library'),before)

    def test_segmentation_capture_uses_committed_scenes_before_cache_callback(self):
        service,holder=self.fixture(); callbacks=[]
        with patch.object(SQLiteTransaction,'after_commit',autospec=True,side_effect=lambda tx,cb,**kw:callbacks.append(cb)):
            written=self.scene(service,holder.current)
        self.assertFalse(service._scene_spaces_by_id)
        captured=service.capture_scene_mapping_state()['spaces'][0]
        self.assertEqual(captured.scene_space_id,written.scene_space_id)
        self.assertEqual(captured._scene_graph_source_revision,written._scene_graph_source_revision)

    def test_batch_retires_uncached_authoritative_predecessor_atomically(self):
        from services.memory.repository import MemoryRepository
        service,holder=self.fixture();raw=MemoryRepository(storage=service._repository.storage);self.addCleanup(raw.close)
        old=RoomSegmentationService().segment(holder.current,map_name='library')[0]
        old.metadata.update(scene_mapping_managed=True,map_binding=snapshot_binding(holder.current),memory_library_name='library')
        raw.upsert_scene_space(old)
        self.assertFalse(service._scene_spaces_by_id)
        capture=service.capture_scene_mapping_state()
        replacement=type(old).model_validate(old.model_dump()).model_copy(update={'scene_space_id':'ssp_replacement_20260101T000000Z_00000001'})
        service.commit_scene_mapping_spaces(capture,[replacement],expected_revisions={replacement.scene_space_id:0},retire_missing=True)
        rows={room.scene_space_id:room for room in service._repository.load_scene_spaces()}
        self.assertEqual(rows[old.scene_space_id].review_status,SceneSpaceReviewStatus.STALE)
        graph=SceneGraphService(memory_service=service,max_cells=2000000)
        self.assertEqual(graph.query()['coverage']['region_count'],1)

    def test_later_batch_conflict_rolls_back_earlier_candidate_and_retirement(self):
        from services.memory.repository import MemoryRepository
        service,holder=self.fixture();first=self.scene(service,holder.current)
        second=type(first).model_validate(first.model_dump()).model_copy(update={'scene_space_id':'ssp_second_20260101T000000Z_00000001','primary_label':'old B'})
        service._repository.upsert_scene_space(second)
        capture=service.capture_scene_mapping_state();by_id={room.scene_space_id:room for room in capture['spaces']}
        raw=MemoryRepository(storage=service._repository.storage);self.addCleanup(raw.close)
        raw.upsert_scene_space(by_id[second.scene_space_id].model_copy(update={'primary_label':'external B'}))
        before=service._repository.scene_graph_generations('library')
        candidates=[by_id[first.scene_space_id].model_copy(update={'primary_label':'partial A'}),
                    by_id[second.scene_space_id].model_copy(update={'primary_label':'partial B'})]
        with self.assertRaises(Exception) as raised:
            service.commit_scene_mapping_spaces(capture,candidates,expected_revisions={room.scene_space_id:int(room.metadata.get('scene_revision',0)) for room in capture['spaces']})
        self.assertEqual(getattr(raised.exception,'error_code',None),'scene_revision_conflict')
        rows={room.scene_space_id:room for room in service._repository.load_scene_spaces()}
        self.assertEqual(rows[first.scene_space_id].primary_label,first.primary_label)
        self.assertEqual(rows[second.scene_space_id].primary_label,'external B')
        self.assertEqual(service._repository.scene_graph_generations('library'),before)
        self.assertNotEqual(service._scene_spaces_by_id[first.scene_space_id].primary_label,'partial A')

    def test_batch_prepares_geometry_and_index_outside_writer_and_commits_once(self):
        with tempfile.TemporaryDirectory() as directory:
            service,holder=self.fixture(str(Path(directory)/'memory.db'))
            old=self.scene(service,holder.current);capture=service.capture_scene_mapping_state()
            candidate=capture['spaces'][0].model_copy(update={'primary_label':'batch label'})
            statements=[];service._repository.storage._connection.set_trace_callback(statements.append)
            self.addCleanup(lambda:service._repository.storage._connection.set_trace_callback(None))
            original=service._repository._prepare_scene_space_write
            original_index=service._durable_index.prepare
            def prepare(*args,**kwargs):
                self.assertFalse(service._repository.storage.in_transaction)
                self.assertFalse(service._instances_lock._is_owned())
                return original(*args,**kwargs)
            def index_prepare(*args,**kwargs):
                self.assertFalse(service._repository.storage.in_transaction)
                return original_index(*args,**kwargs)
            with patch.object(service._repository,'_prepare_scene_space_write',side_effect=prepare),patch.object(service._durable_index,'prepare',side_effect=index_prepare):
                written=service.commit_scene_mapping_spaces(capture,[candidate],expected_revisions={old.scene_space_id:0})
            self.assertEqual(written[0].primary_label,'batch label')
            self.assertEqual(sum(statement=='COMMIT' for statement in statements),1)

    def test_label_continuity_ignores_representative_point_change_as_before(self):
        service,holder=self.fixture();old=self.scene(service,holder.current)
        old=old.model_copy(deep=True);old.primary_label='office'
        old.boundary.centroid.position.x+=.05
        regions=RoomSegmentationService().segment(holder.current,map_name='library',existing_spaces=[old])
        self.assertTrue(getattr(regions[0],'_geometry_unchanged'))
        self.assertNotEqual(regions[0].metadata['geometry_revision'],old.metadata['geometry_revision'])
        owner=SceneMappingService(memory_service=service,config=SimpleNamespace(min_area_m2=2,doorway_width_m=1.2,max_cells=2000000))
        result=owner._bind_regions({'spaces':[old],'binding':snapshot_binding(holder.current)},regions)
        self.assertEqual(result[0].primary_label,'office')

    def test_evidence_stamp_is_small_and_uses_owned_source_versions(self):
        service,_=self.fixture(); entry=records()[1][3]
        service._repository.upsert_semantic_memory(entry)
        loaded=service._repository.load_semantic_memories()[0]
        stamp=_evidence_inputs([loaded],'model-a')
        self.assertEqual(stamp['version'],2);self.assertEqual(stamp['entries'][0]['memory_id'],entry.memory_id)
        self.assertEqual(stamp['entries'][0]['source_revision'][0],'business')
        self.assertEqual(_evidence_inputs([loaded],'model-a'),stamp)

if __name__=='__main__':unittest.main(verbosity=2)
