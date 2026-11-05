#!/usr/bin/env python3
"""Offline object-memory acceptance: real ingest, SQLite reopen, query and HTTP.

Synthetic observations only. No network, robot, real model, repository mutation,
content digest, or build is required. The only disabled service behavior is the
background vector/refinement workers; synchronous ingestion stays unmodified.

Run with the repository Python 3.8 environment:
  python scripts/diagnostics/object_memory_integration_20261014.py --output-dir /tmp/neh-object-memory
The JSON report records every case and any forbidden application hash attempt.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts'), str(ROOT / 'tests')]

# Initialize test/framework dependencies before the application hash fence.
from fastapi import FastAPI
from fastapi.testclient import TestClient
from contracts.geometry import Pose, Quaternion, Vector3
from contracts.perception import BoundingBox2D, Detection2D, Detection3D, Observation
from contracts.runtime_views import PerceptionContext,SceneSummary
from contracts.spatial_memory import InstanceDynamicState, SemanticInstance
from scripts.memory_test_vectors import ExplicitTextEmbedder
from services.memory.service import MemoryService
from services.memory.store import MemorySnapshot
from core.capability import CapabilityTable
from services.geometry.state import SessionPhase
from core import StateStore

NOW = datetime(2026, 10, 14, 4, 0, tzinfo=timezone.utc)
AXES = ('x', 'y', 'z')


def position(x=2.0, y=1.0, z=0.7, *, frame='map'):
    return Pose(frame_id=frame, position=Vector3(x=x, y=y, z=z))


def box_points(center, size):
    return [[center[0] + sx * size[0], center[1] + sy * size[1], center[2] + sz * size[2]]
            for sx in (-0.5, 0.5) for sy in (-0.5, 0.5) for sz in (-0.5, 0.5)]


def detection3d(label='umbrella', *, x=2.0, y=1.0, z=0.7, size=(0.9, 0.9, 1.1), yaw=0.0):
    pose = Pose(frame_id='map', position=Vector3(x=x, y=y, z=z),
                orientation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    dimensions = {} if size is None else dict(zip(('size_x_m', 'size_y_m', 'size_z_m'), size))
    return Detection3D(label=label, score=.97, pose=pose, **dimensions)


def detection2d(label='umbrella', *, x=2.0, y=1.0, z=0.7, size=(0.9, 0.9, 1.1),
                camera='front_camera', track='umbrella_track', pixel=(100., 100.), fallback=False):
    attributes = {} if fallback else {'support_points_map': box_points((x, y, z), size)}
    attributes.update({'has_structured_segmentation': True, 'mask_center_px': list(pixel),
                       'segmentation_score': .98, 'predicted_iou': .98, 'stability_score': .98})
    return Detection2D(label=label, score=.97, bbox=BoundingBox2D(x_px=pixel[0]-10, y_px=pixel[1]-10,
                       width_px=20., height_px=20.), camera_id=camera, track_id=track, attributes=attributes)


class ObjectMemoryIntegration(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='neh-object-memory-')
        self.addCleanup(self.temporary.cleanup)
        self.database = str(Path(self.temporary.name) / 'memory.db')
        self.sequence = 0
        self.thread_errors = []
        hook = patch('threading.excepthook', side_effect=lambda event: self.thread_errors.append(repr(event.exc_value)))
        hook.start()
        self.addCleanup(hook.stop)
        self.addCleanup(lambda: self.assertEqual(self.thread_errors, []))

    def service(self,*,library='house'):
        class Geometry:
            def __init__(self):self.name=library;self.callbacks=[]
            def snapshot(self):return SimpleNamespace(map_name=self.name,phase=SessionPhase.MAPPING)
            def subscribe(self,callback):
                self.callbacks.append(callback);return lambda:self.callbacks.remove(callback)
            def pose(self):return None
            def latest_map(self):return None
            def change(self,name):
                self.name=name
                for callback in tuple(self.callbacks):callback(self.snapshot())
        geometry=Geometry();embedder=ExplicitTextEmbedder(dimension=16)
        service=MemoryService(geometry_state=geometry,maps_root=Path(self.database).with_suffix('')/'maps',default_root=Path(self.database).with_suffix('')/'default',
            observation_service=None,perception_service=None,state_store=StateStore(),text_embedder=embedder,
            embedding_model='lexical-only',embedding_dimension=16,image_embedding_model='disabled')
        service._diagnostic_geometry=geometry
        self.addCleanup(service.close)
        return service

    def change_library(self,service,name):
        service._diagnostic_geometry.change(name)
        service.wait_for_library(name,SimpleNamespace(check=lambda:None,remaining=lambda:2))

    def context(self, detections, *, camera='front_camera', sequence=None):
        self.sequence += 1
        index = self.sequence if sequence is None else sequence
        return PerceptionContext(camera_id=camera, observation=Observation(
            observation_id='obs_integration_20261014T040000Z_%08x' % index, frame_id='map',
            detections_2d=[item for item in detections if isinstance(item, Detection2D)],
            detections_3d=[item for item in detections if isinstance(item, Detection3D)]),
            scene_summary=SceneSummary(headline='Synthetic observed objects'), pipeline_name='offline',
            detector_backend='fixture', tracker_backend='fixture')

    def ingest(self,service,detections,*,second=0,camera='front_camera',pose=None,context=None):
        return service.ingest_perception_observation(map_name=service.current_library,
            perception_context=context or self.context(detections,camera=camera),current_pose=pose or position(0,0,0),now=NOW+timedelta(seconds=second))

    def settle(self, service, detections, *, start=0, **kwargs):
        result = None
        for second in (start, start + 5, start + 10):
            result = self.ingest(service, detections, second=second, **kwargs)
        return result

    def stored(self,service,*,library=None):
        name=library or service.current_library
        return list(MemorySnapshot.from_directory(service.maps_root/name/'memory',name=name).read('instance'))

    def assert_size(self, row, expected):
        extent = row['object_extent']
        self.assertIsNotNone(extent)
        for axis, value in zip(AXES, expected):
            self.assertAlmostEqual(extent['size_m'][axis], value, places=5)
        self.assertGreater(len(extent['footprint_xy']), 0)
        self.assertLessEqual(len(extent['footprint_xy']), 8)
        return extent

    def legacy(self, *, suffix='00000001', x=2.0, frame='map', metadata=None, attributes=None):
        return SemanticInstance(instance_id='ins_legacy_20261014T040000Z_' + suffix,
            anchor_id='anc_legacy_20261014T040000Z_' + suffix, label='cup', map_name='house',
            pose=position(x, frame=frame), first_seen_ts=NOW, last_seen_ts=NOW, observation_count=3,
            dynamic_state=InstanceDynamicState.STATIC,
            attributes=attributes or {}, metadata={'memory_library_name': 'house',
                'projection_method': 'map_point_samples', 'projection_confidence': .97,
                'pose_quality_level': 'high', **(metadata or {})})

    def test_detection3d_persists_reopens_and_public_query_returns_dimensions(self):
        first = self.service()
        _, _, ids = self.settle(first, [detection3d(yaw=math.pi / 4)])
        self.assertEqual(len(set(ids)), 1)
        stored = self.stored(first)
        self.assertEqual(len(stored), 1)
        identifier = stored[0].instance_id
        self.assertEqual(stored[0].observation_count, 3)
        self.assert_size(first.list_semantic_instances()[0], (.9, .9, 1.1))
        first.close()
        second = self.service()
        rows = second.list_semantic_instances()
        self.assertEqual([row['instance_id'] for row in rows], [identifier])
        extent = self.assert_size(rows[0], (.9, .9, 1.1))
        self.assertEqual(extent['source'], 'detection3d')
        self.assertAlmostEqual(extent['orientation']['z'], math.sin(math.pi / 8))
        matches = second.query_semantic_memory('umbrella', similarity_threshold=.1).matches
        self.assertEqual([match.record_id for match in matches], [identifier])
        self.assertEqual(matches[0].semantic_instance.observation_count, 3)
        self.assertEqual(second.get_semantic_instances_snapshot()['instances'][0]['object_extent'], extent)

    def test_same_umbrella_new_view_and_next_day_restart_updates_one_record(self):
        first = self.service()
        self.settle(first, [detection2d(pixel=(100, 100))])
        before = self.stored(first)
        self.assertEqual(len(before), 1)
        identifier = before[0].instance_id
        first.close()
        second = self.service()
        _, outcomes, ids = self.ingest(second, [detection2d(x=2.02, camera='rear_camera',
            track='new_tracker_id', pixel=(900, 600))], camera='rear_camera', second=18*3600)
        self.assertEqual(ids, [identifier])
        self.assertFalse(outcomes[0].created)
        self.assertEqual(len(self.stored(second)), 1)
        self.assertEqual(self.stored(second)[0].observation_count, 4)
        self.assertEqual(len(second.list_semantic_instances()), 1)
        self.assert_size(second.list_semantic_instances()[0], (.96, .93, 1.11))

    def test_nearby_cups_seen_separately_and_reordered_batch_stay_distinct(self):
        service = self.service()
        left = detection3d('cup', x=1.0, size=(.08, .08, .12))
        right = detection3d('cup', x=1.10, size=(.08, .08, .12))
        _, _, left_ids = self.settle(service, [left])
        _, _, right_ids = self.settle(service, [right], start=20)
        self.assertNotEqual(left_ids, right_ids)
        _, _, reordered = self.ingest(service, [right, left], second=40)
        self.assertEqual(reordered, [right_ids[0], left_ids[0]])
        stored = self.stored(service)
        self.assertEqual(len(stored), 2)
        self.assertEqual(sorted(record.observation_count for record in stored), [4, 4])
        for row in service.list_semantic_instances(label='cup'):
            self.assert_size(row, (.08, .08, .12))
        service.close()
        reopened = self.service()
        self.assertEqual({row['instance_id'] for row in reopened.list_semantic_instances()}, set(reordered))
        _, _, next_day = self.ingest(reopened, [
            detection3d('cup', x=1.105, size=(.08, .08, .12)),
            detection3d('cup', x=1.005, size=(.08, .08, .12)),
        ], second=18*3600)
        self.assertEqual(next_day, reordered)
        self.assertEqual(sorted(record.observation_count for record in self.stored(reopened)), [5, 5])

    def test_fallback_keeps_previous_position_and_dimensions_through_restart(self):
        service = self.service()
        _, _, ids = self.settle(service, [detection2d()])
        before = service.list_semantic_instances()[0]
        _, _, weak_ids = self.ingest(service, [detection2d(fallback=True)], second=11, pose=position(30, 30, 0))
        self.assertEqual(weak_ids, [])
        row = service.list_semantic_instances()[0]
        self.assertEqual(row['pose'], before['pose'])
        self.assertEqual(row['object_extent'], before['object_extent'])
        self.assertFalse(row['observation_only'])
        self.assertEqual(len(self.stored(service)), 1)
        service.close()
        reopened = self.service()
        self.assertEqual(reopened.list_semantic_instances()[0]['object_extent'], before['object_extent'])

    def test_weak_visible_support_does_not_shrink_detector_dimensions(self):
        service = self.service()
        self.settle(service, [detection3d()])
        identifier = self.stored(service)[0].instance_id
        _, _, ids = self.ingest(service, [detection2d(size=(.25, .25, .2), track='new_track')], second=15)
        self.assertEqual(ids, [identifier])
        extent = self.assert_size(service.list_semantic_instances()[0], (.9, .9, 1.1))
        self.assertEqual(extent['source'], 'detection3d')
        self.assertEqual(len(self.stored(service)), 1)

    def test_unknown_metric_dimensions_stay_unknown(self):
        service = self.service()
        self.settle(service, [detection3d(size=None)])
        self.assertEqual(len(self.stored(service)), 1)
        self.assertIsNone(service.list_semantic_instances()[0]['object_extent'])
        service.close()
        reopened = self.service()
        self.assertIsNone(reopened.list_semantic_instances()[0]['object_extent'])

    def test_two_camera_same_batch_publishes_final_count_dimensions_and_persists_once(self):
        service = self.service()
        front = detection2d(size=(.8, .8, 1.), camera='front_camera', track='front')
        rear = detection2d(size=(.9, .9, 1.1), camera='rear_camera', track='rear', pixel=(800, 500))
        context = None
        identifier = None
        for index, second in enumerate((0, 5, 10), 1):
            context = self.context([front, rear])
            _,_,ids=self.ingest(service,[],context=context,second=second)
            self.assertEqual(len(ids), 2)
            self.assertEqual(ids[0], ids[1])
            identifier = ids[0]
            # 同源双相机只计一次确认；第三个独立源观察后才对外显示并持久化。
            stored = self.stored(service)
            self.assertEqual(len(stored), 0 if index < 3 else 1)
            if stored:
                self.assertEqual(stored[0].observation_count, index * 2)
            if index<3:
                self.assertEqual(service.list_semantic_instances(), [])
                continue
            row = service.list_semantic_instances()[0]
            self.assertEqual(row['observation_count'], index * 2)
            self.assert_size(row, (.93, .93, 1.11))
        service.close()
        reopened = self.service()
        self.assertEqual(self.stored(reopened)[0].instance_id, identifier)
        self.assertEqual(self.stored(reopened)[0].observation_count, 6)
        self.assert_size(reopened.list_semantic_instances()[0], (.93, .93, 1.11))
        self.ingest(reopened, [], context=context, second=11)
        self.assertEqual(self.stored(reopened)[0].observation_count, 6)

    def test_paired_3d_and_2d_same_camera_track_count_once_and_keep_detector_extent(self):
        service = self.service()
        detector = detection3d('cup', size=(.08, .08, .12)).model_copy(update={
            'attributes':{'camera_id':'front_camera', 'track_id':'shared_track'}})
        support = detection2d('cup', size=(.07, .07, .1), track='shared_track')
        self.settle(service, [detector, support])
        rows = service.list_semantic_instances(label='cup')
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['observation_count'], 3)
        extent = self.assert_size(rows[0], (.08, .08, .12))
        self.assertEqual(extent['source'], 'detection3d')
        identifier = rows[0]['instance_id']
        self.assertEqual(len(self.stored(service)), 1)
        service.close()
        reopened = self.service()
        row = reopened.list_semantic_instances()[0]
        self.assertEqual(row['instance_id'], identifier)
        self.assertEqual(row['observation_count'], 3)
        self.assertEqual(row['object_extent'], extent)

    def test_unpaired_same_camera_modalities_do_not_collapse_separate_detections(self):
        for case, first_track, second_track in (('absent', None, None), ('different', 'left', 'right')):
            with self.subTest(case=case):
                self.database = str(Path(self.temporary.name) / (case + '.db'))
                service = self.service()
                attributes = {'camera_id':'front_camera'}
                if first_track is not None:
                    attributes['track_id'] = first_track
                detector = detection3d('cup', size=(.08, .08, .12)).model_copy(update={'attributes':attributes})
                # 两只8厘米杯子相距10厘米，几何不重合；缺跟踪号本身不再代表不同物体。
                support = detection2d('cup', x=2.1, size=(.08, .08, .12), track=second_track)
                self.settle(service, [detector, support])
                rows = service.list_semantic_instances(label='cup')
                self.assertEqual(len(rows), 2)
                self.assertEqual(len(self.stored(service)), 2)
                self.assertEqual(sorted(row['observation_count'] for row in rows), [3, 3])
                self.assertEqual(len({row['instance_id'] for row in rows}), 2)
                positions=sorted(record.pose.position.x for record in self.stored(service))
                self.assertAlmostEqual(positions[1]-positions[0],.1,places=6)
                service.close()

    def test_same_source_replay_does_not_inflate_observation_count(self):
        service = self.service()
        self.settle(service, [detection3d()])
        context = self.context([detection3d()])
        self.ingest(service, [], context=context, second=20)
        count = self.stored(service)[0].observation_count
        self.ingest(service, [], context=context, second=21)
        self.assertEqual(len(self.stored(service)), 1)
        self.assertEqual(self.stored(service)[0].observation_count, count)
        service.close()
        reopened = self.service()
        self.ingest(reopened, [], context=context, second=22)
        self.assertEqual(self.stored(reopened)[0].observation_count, count)

    def test_identical_coordinates_in_two_libraries_are_isolated(self):
        service = self.service()
        _, _, first_ids = self.settle(service, [detection3d('cup', size=(.08, .08, .12))])
        self.change_library(service,'garage')
        _, _, second_ids = self.settle(service, [detection3d('cup', size=(.08, .08, .12))], start=20)
        self.assertNotEqual(first_ids, second_ids)
        self.assertEqual([row['instance_id'] for row in service.list_semantic_instances()], second_ids)
        self.assertEqual([row['instance_id'] for row in service.list_semantic_instances(library_name='house')], first_ids)
        self.change_library(service,'house')
        self.assertEqual([row['instance_id'] for row in service.list_semantic_instances()], first_ids)

    def test_foreign_frame_history_is_not_absorbed_or_deleted(self):
        service = self.service()
        old = self.legacy(frame='odom')
        service.write(lambda store:store.put('instance',old))
        self.change_library(service,'house')
        _, _, ids = self.settle(service, [detection3d('cup', size=(.08, .08, .12))])
        self.assertNotEqual(ids, [old.instance_id])
        self.assertEqual(len(self.stored(service)), 2)
        saved_old = next(row for row in self.stored(service) if row.instance_id == old.instance_id)
        self.assertEqual(saved_old.pose.frame_id, 'odom')
        self.assertEqual(saved_old.observation_count, 3)






    def test_nonuniform_point_support_keeps_measured_box_center_separate_from_navigation_pose(self):
        service = self.service()
        base = detection2d()
        points = box_points((2., 1., .7), (.9, .9, 1.1))
        first = base.model_copy(update={'attributes':{**base.attributes,
            'support_points_map':points + [[1.55, 1., .7]]}})
        self.settle(service, [first])
        row = service.list_semantic_instances()[0]
        self.assertAlmostEqual(row['pose']['x'], 1.55)
        self.assertAlmostEqual(row['object_extent']['center']['x'], 1.995)
        second = base.model_copy(update={'attributes':{**base.attributes,
            'support_points_map':points + [[2.45, 1., .7]]}})
        self.ingest(service, [second], second=20)
        row = service.list_semantic_instances()[0]
        self.assertAlmostEqual(row['pose']['x'], 2.45)
        self.assertAlmostEqual(row['object_extent']['center']['x'], 1.995)
        self.assert_size(row, (.93, .93, 1.11))

    def test_http_route_preserves_extent_and_filter_after_database_reopen(self):
        from gateways.http.api import build_http_router
        from gateways.capabilities.memory import CAPABILITIES
        service=self.service();self.settle(service,[detection3d()]);service.close();reopened=self.service()
        runtime=SimpleNamespace(memory=reopened,capabilities=CapabilityTable(),robot=SimpleNamespace(manifest=SimpleNamespace(robot_name='customrobot')),ensure=lambda names,timeout:None)
        for cap in CAPABILITIES:runtime.capabilities.register(cap)
        access = SimpleNamespace(authorize_capability=lambda value: None)
        # Router initialization owns unrelated settings lifecycle resources; use
        # a harmless in-memory fixture while keeping the actual route unchanged.
        with patch('gateways.settings.SettingsApplication', return_value=SimpleNamespace(start=lambda:None, close=lambda:None)):
            router = build_http_router(runtime, access)
        app = FastAPI()
        app.include_router(router)
        with TestClient(app) as client:
            response = client.get('/api/memory/instances?label=umbrella&limit=1')
            self.assertEqual(response.status_code, 200, response.text)
            body = response.json()
            self.assertEqual(len(body['instances']),1)
            self.assert_size(body['instances'][0], (.9, .9, 1.1))
            self.assertEqual(client.get('/api/memory/instances?label=cup').json()['instances'], [])


class EvidenceResult(unittest.TextTestResult):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cases = []
        self.started = {}
    def startTest(self, test):
        self.started[test.id()] = time.monotonic()
        super().startTest(test)
    def record(self, test, outcome, detail=''):
        self.cases.append({'name':test.id(), 'outcome':outcome,
            'elapsed_seconds':time.monotonic()-self.started[test.id()], 'detail':detail})
    def addSuccess(self, test):
        self.record(test, 'passed')
        super().addSuccess(test)
    def addError(self, test, err):
        self.record(test, 'error', self._exc_info_to_string(err, test))
        super().addError(test, err)
    def addFailure(self, test, err):
        self.record(test, 'failed', self._exc_info_to_string(err, test))
        super().addFailure(test, err)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--test', action='append', default=[], help='Select exact unittest method names')
    options = parser.parse_args()
    selected = options.test or unittest.defaultTestLoader.getTestCaseNames(ObjectMemoryIntegration)
    suite = unittest.TestSuite(ObjectMemoryIntegration(name) for name in selected)
    started_at = datetime.now(timezone.utc).isoformat()
    started = time.monotonic()
    result=unittest.TextTestRunner(verbosity=2,resultclass=EvidenceResult).run(suite)
    report = {'python':sys.version, 'executable':sys.executable, 'argv':sys.argv,
        'started_at_utc':started_at, 'finished_at_utc':datetime.now(timezone.utc).isoformat(),
        'elapsed_seconds':time.monotonic()-started, 'tests_run':result.testsRun,
        'successful':result.wasSuccessful(), 'cases':result.cases,
        'scope':'Synthetic offline ingest, real projection/association/SQLite persistence/reload and public APIs; no robot or browser rendering'}
    if options.output_dir:
        options.output_dir.mkdir(parents=True, exist_ok=True)
        destination = options.output_dir / 'object_memory_integration.json'
        destination.write_text(json.dumps(report, indent=2) + '\n')
        print('REPORT:', destination)
    return 0 if report['successful'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
