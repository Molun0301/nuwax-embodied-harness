#!/usr/bin/env python3
"""Synthetic offline recording input ownership and provenance regression checks."""
from __future__ import annotations

import io
import json
import os
import sqlite3
import struct
import sys
import tarfile
import tempfile
import unittest
from contextlib import contextmanager, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from contracts.artifacts import ArtifactKind
from core.errors import GatewayError
from scripts.diagnostics import recording_inputs
from scripts.diagnostics import semantic_map_recording as recording
from services.artifact_store import LocalArtifactStore


class RecordingInputChecks(unittest.TestCase):
    def setUp(self):
        temporary_root = os.environ.get('NEH_TEST_WORK_DIR')
        if temporary_root:
            Path(temporary_root).mkdir(parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(prefix='neh-recording-inputs-', dir=temporary_root)
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / 'output'
        self.output.mkdir()
        self.cloud = self.root / 'cloud.xyzt'
        self.cloud.write_bytes(np.asarray([[1.5, 1.5, 0., 0.], [2.5, 2.5, 0., 0.]], dtype='<f4').tobytes())
        self.occupancy = self.root / 'occupancy.json'
        self.raw = {'ts': 1., 'frame_id': 'map', 'origin': [0., 0., 0.],
                    'origin_orientation': [0., 0., 0., 1.], 'width': 4, 'height': 4,
                    'resolution': 1., 'data': [0] * 16}
        self.occupancy.write_text(json.dumps(self.raw))
        self.trajectory = self.root / 'trajectory.json'
        self.trajectory.write_text(json.dumps([{'timestamp_ns': 1000000000, 'p': [1.5, 1.5, 0.]}]))
        self.paired = self.root / 'paired.bin'
        self.paired.write_bytes(struct.pack('<Q7dI', 1000000000, 1.5, 1.5, 0., 0., 0., 0., 1., 2)
                               + np.asarray([[1.5, 1.5, 0.], [2.5, 2.5, 0.]], dtype='<f4').tobytes())
        self.manifest = self.root / 'manifest.json'
        self.manifest.write_text(json.dumps({'variants': [{'id': 'fixture', 'point_file': self.cloud.name,
            'trajectory_file': self.trajectory.name, 'point_sha256': 'unsupported-historical-value'}]}))
        self.database = self.root / 'source.sqlite'
        with sqlite3.connect(self.database) as connection:
            connection.execute('CREATE TABLE records(kind TEXT)')
            connection.execute("INSERT INTO records VALUES('pose')")
        self.archive = self.root / 'source.tar'
        with tarfile.open(self.archive, 'w') as archive:
            value = b'synthetic recording';member = tarfile.TarInfo('note.txt');member.size = len(value)
            archive.addfile(member, io.BytesIO(value))
        self.args = SimpleNamespace(occupancy=self.occupancy, output=self.output, version='fixture', artifact_dir=None)
        self.store = LocalArtifactStore(str(self.output / 'input-artifacts'), '')

    def command(self, **extra):
        result = ['semantic-recording', '--manifest', str(self.manifest), '--version', 'fixture',
            '--occupancy', str(self.occupancy), '--trajectory', str(self.trajectory), '--paired-scans', str(self.paired),
            '--source-recording', str(self.database), '--source-archive', str(self.archive), '--output', str(self.output)]
        for name, value in extra.items():
            result.extend(['--' + name.replace('_', '-'), str(value)])
        return result

    def run_recording(self, *, render=None, **extra):
        with patch.object(sys, 'argv', self.command(**extra)), patch.object(recording, 'render_map', side_effect=render), \
                patch('subprocess.Popen', side_effect=AssertionError('No native process')), redirect_stdout(io.StringIO()):
            recording.main()
        return json.loads((self.output / '验证结果.json').read_text())

    def test_exact_repeat_reuses_all_input_ids_and_epoch(self):
        first = self.run_recording();second = self.run_recording()
        self.assertEqual(first['input_artifact_ids'], second['input_artifact_ids'])
        self.assertEqual(first['source_epoch'], second['source_epoch'])
        self.assertEqual(len(self.store.list_refs()), 3)
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)

    def test_same_size_restored_mtime_changed_cloud_gets_distinct_id(self):
        first = self.run_recording();before = self.cloud.stat();data = bytearray(self.cloud.read_bytes())
        data[:4] = struct.pack('<f', 1.25);self.cloud.write_bytes(data)
        os.utime(self.cloud, ns=(before.st_atime_ns, before.st_mtime_ns))
        second = self.run_recording()
        self.assertNotEqual(first['pointcloud_artifact_id'], second['pointcloud_artifact_id'])
        self.assertEqual(first['occupancy_artifact_id'], second['occupancy_artifact_id'])
        self.assertEqual(first['source_epoch'], second['source_epoch'])
        self.assertEqual(self.store.read_bytes(first['pointcloud_artifact_id'])[:4], struct.pack('<f', 1.5))

    def test_changed_occupancy_gets_new_epoch_while_old_input_remains(self):
        first = self.run_recording();before = self.occupancy.read_bytes()
        changed = before.replace(b'"ts": 1.0', b'"ts": 2.0');self.assertEqual(len(changed), len(before))
        self.occupancy.write_bytes(changed);second = self.run_recording()
        self.assertNotEqual(first['source_epoch'], second['source_epoch'])
        self.assertEqual(self.store.read_bytes(first['occupancy_artifact_id']), before)

    def test_missing_artifact_creates_new_owned_identity(self):
        first = self.run_recording();self.store.get_file_path(first['occupancy_artifact_id']).unlink()
        second = self.run_recording();third = self.run_recording()
        self.assertNotEqual(first['source_epoch'], second['source_epoch'])
        self.assertEqual(second['source_epoch'], third['source_epoch'])

    def test_shared_artifact_root_reuses_identity_across_output_dirs(self):
        shared = self.root / 'shared-inputs'
        first = self.run_recording(artifact_dir=shared)
        self.output = self.root / 'other-output';self.output.mkdir()
        second = self.run_recording(artifact_dir=shared)
        self.assertEqual(first['input_artifact_ids'], second['input_artifact_ids'])
        self.assertEqual(first['source_epoch'], second['source_epoch'])

    def test_capture_reads_each_input_once_and_parses_owned_bytes(self):
        original = Path.read_bytes;seen = []
        def read(path):
            seen.append(path)
            return original(path)
        with patch.object(Path, 'read_bytes', read), recording.capture_recording_inputs(self.args, self.cloud) as (_, refs, cloud, raw):
            self.assertEqual(cloud, original(self.cloud));self.assertEqual(raw, self.raw)
            self.assertEqual(set(refs), {'pointcloud', 'occupancy', 'segmentation_source'})
        self.assertEqual(seen.count(self.cloud), 1);self.assertEqual(seen.count(self.occupancy), 1)
        self.assertEqual(seen.count(ROOT / 'services/memory/room_segmentation.py'), 1)

    def test_path_changes_after_capture_do_not_change_used_bytes(self):
        cloud = self.cloud.read_bytes();occupancy = self.occupancy.read_bytes()
        original = recording.capture_recording_inputs
        @contextmanager
        def mutate_paths(*args, **kwargs):
            with original(*args, **kwargs) as captured:
                self.cloud.write_bytes(b'invalid later cloud')
                self.occupancy.write_bytes(b'invalid later occupancy')
                yield captured
        with patch.object(recording, 'capture_recording_inputs', mutate_paths):
            report = self.run_recording()
        self.assertEqual(report['pointcloud_count'], 2)
        self.assertEqual(report['grid_cells'], 16)
        self.assertEqual(self.store.read_bytes(report['pointcloud_artifact_id']), cloud)
        self.assertEqual(self.store.read_bytes(report['occupancy_artifact_id']), occupancy)
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)

    def test_reuse_handoff_retains_inputs_after_reader_and_publication_exit(self):
        self.run_recording()
        with recording.capture_recording_inputs(self.args, self.cloud) as (store, refs, _, _):
            self.assertEqual(store.get_holder_status()['protected_artifact_count'], 3)
            for ref in refs.values():
                with self.assertRaises(GatewayError) as caught:store.delete_artifact(ref.artifact_id)
                self.assertEqual(caught.exception.error_code, 'artifact_in_use')
            unrelated = store.save_bytes(kind=ArtifactKind.OTHER, mime_type='text/plain', data=b'later', extension='.txt')
            store.delete_artifact(unrelated.artifact_id)
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)

    def test_inputs_remain_held_during_render_and_release_after_failure(self):
        def render(*args):
            self.assertEqual(self.store.get_holder_status()['protected_artifact_count'], 3)
            raise RuntimeError('synthetic render failure')
        with self.assertRaisesRegex(RuntimeError, 'synthetic render failure'):self.run_recording(render=render)
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)
        self.assertEqual(self.store.get_holder_status()['protected_artifact_count'], 0)

    def test_capture_partial_save_failure_releases_publication_owner(self):
        original = recording.recording_artifact;calls = []
        def save(*args, **kwargs):
            calls.append(kwargs['kind'])
            if len(calls) == 2:raise RuntimeError('synthetic import failure')
            return original(*args, **kwargs)
        with patch.object(recording, 'recording_artifact', side_effect=save):
            with self.assertRaisesRegex(RuntimeError, 'synthetic import failure'):
                with recording.capture_recording_inputs(self.args, self.cloud):pass
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)
        self.assertEqual(self.store.get_holder_status()['protected_artifact_count'], 0)

    def test_legacy_digest_is_ignored_not_presented_as_verified(self):
        result = self.run_recording()
        self.assertEqual(result['schema_version'], 2)
        self.assertEqual(result['manifest_content_binding'], 'not_verified_path_selection_only')
        self.assertEqual(result['input_kind'], 'captured_offline_recording_inputs')
        self.assertNotIn('point_sha256', result['selected_variant'])
        self.assertFalse(any('sha256' in key for key in result))
        self.assertIn('not_loaded_module_proof', result['segmentation_source_evidence'])
        self.assertEqual(result['region_count'], 1)
        self.assertEqual(result['geometry_checks']['overlapping_region_cells'], 0)
        self.assertEqual(result['geometry_checks']['nonfree_cells_assigned'], 0)
        self.assertEqual(result['pairing']['trajectory_timestamp_matches'], 1)
        self.assertFalse(result['classification_accuracy_verified'])

    def test_manifest_without_old_digest_is_supported(self):
        value = json.loads(self.manifest.read_text());value['variants'][0].pop('point_sha256')
        self.manifest.write_text(json.dumps(value));self.assertEqual(self.run_recording()['pointcloud_count'], 2)

    def test_pairing_position_mismatch_still_rejects_and_releases_inputs(self):
        self.trajectory.write_text(json.dumps([{'timestamp_ns':1000000000, 'p':[2., 1.5, 0.]}]))
        with self.assertRaisesRegex(ValueError, '同坐标来源'):self.run_recording()
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)

    def test_pairing_timestamp_mismatch_still_rejects(self):
        self.trajectory.write_text(json.dumps([{'timestamp_ns':2000000000, 'p':[1.5, 1.5, 0.]}]))
        with self.assertRaisesRegex(ValueError, '同坐标来源'):self.run_recording()

    def test_non_axis_aligned_orientation_still_rejects(self):
        self.raw['origin_orientation'] = [0., 0., .1, .995];self.occupancy.write_text(json.dumps(self.raw))
        with self.assertRaisesRegex(ValueError, '轴对齐'):self.run_recording()
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)

    def test_geometry_budget_validation_remains(self):
        with self.assertRaisesRegex(ValueError, '最大单元数'):self.run_recording(max_cells=1)
        self.assertEqual(self.store.get_holder_status()['holder_count'], 0)

    def test_truncated_scan_payload_still_rejects(self):
        self.paired.write_bytes(self.paired.read_bytes()[:-1])
        with self.assertRaisesRegex(ValueError, '点载荷不完整'):self.run_recording()

    def test_compatibility_reexport_uses_exact_same_helper(self):
        from scripts.diagnostics import semantic_map_development
        self.assertIs(semantic_map_development.recording_artifact, recording_inputs.recording_artifact)


if __name__ == '__main__':
    unittest.main(verbosity=2)
