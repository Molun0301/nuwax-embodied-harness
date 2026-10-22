"""Source-only receipt propagation regressions; no native/runtime device needed."""
from tests.conftest import create_geometry_runtime
import tempfile
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contracts.geometry import Pose
from contracts.maps import CostMap, OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from contracts.runtime_views import MapSnapshot
from core import StateStore, StreamBus
from core.errors import GatewayError
from services.geometry.runtime import PlatformGeometryRuntime, _live_map_layer_descriptor
from services.mapping import service as mapping
from services.mapping.service import MappingService, _committed_occupancy_identity, _receipt_source_epoch


def identity(revision=7, *, commit=11, representation='effective', nonce='ab' * 16):
    return {'kind': 'committed_occupancy', 'representation': representation, 'provenance': {
        'schema_version': 2, 'commit_id': {'boot_nonce': nonce, 'sequence': commit},
        'session_id': 'mapping-session', 'session_generation': 3, 'occupancy_revision': revision,
        'source_timestamp_ns': 123456789, 'backend_kind': 'voxel', 'backend_generation': 2, 'frame_id': 'map'}}


def grid(receipt=None, cells=b'\x00\x64'):
    metadata = {} if receipt is None else {'content_identity': receipt, 'map_provenance': receipt['provenance'],
                                         'raw_representation': 2, 'mapping_session_id': 'mapping-session',
                                         'session_generation': 3, 'map_name': 'test-map'}
    return OccupancyGrid(map_id='test-map', frame_id='map', width=len(cells), height=1, resolution_m=.1,
                         timestamp=datetime.fromtimestamp(.123456789, tz=timezone.utc),
                         origin=Pose(frame_id='map'), data=PackedOccupancyCells(cells), metadata=metadata)


def metadata(receipt):
    return {'source_kind': 'platform_live_map_provider', 'map_snapshot_kind': 'live_streaming',
            'content_identity': receipt, 'map_provenance': receipt['provenance'],
            'update_sequence': receipt['provenance']['occupancy_revision'], 'raw_representation': 2}


def runtime(test_case):
    directory = tempfile.TemporaryDirectory(prefix="geometry-receipt-")
    test_case.addCleanup(directory.cleanup)
    value = create_geometry_runtime(directory.name)
    value.config = NS(stack='test', mapping=NS(occupancy_projection='test', timestamp_quality=NS(max_tf_age_sec=1)),
                      navigation=NS(costmap_source='test'), build_metadata=lambda: {})
    value._build_parameter_summary = lambda: {}
    value.describe_backend = lambda: {}
    value._map_stream_tracker = {}
    return value


def live_snapshot(value, receipt=None, cost=None):
    receipt = receipt or identity()
    layer = grid(receipt)
    return value._build_live_map_snapshot(live_occupancy=layer, live_cost_map=cost,
        occupancy_descriptor=_live_map_layer_descriptor(layer),
        cost_map_descriptor=_live_map_layer_descriptor(cost), live_status={}, seed_bundle=None,
        mapping_context={'session_id': 'mapping-session', 'session_generation': 3, 'map_name': 'test-map'})


class RuntimeReceiptTests(unittest.TestCase):
    def test_descriptor_uses_receipt_without_hashing_or_reading_cells(self):
        receipt = identity()
        class NoCells:
            def __iter__(self):
                raise AssertionError('descriptor scanned grid cells')
        layer = NS(map_id='test-map', frame_id='map', data=NoCells(), metadata={'content_identity': receipt})
        with patch('hashlib.sha256', side_effect=AssertionError('live hash')):
            descriptor = _live_map_layer_descriptor(layer)
        self.assertEqual(descriptor['revision'], 7)
        self.assertEqual(descriptor['content_identity'], receipt)
        self.assertNotIn('content_hash', descriptor)
        receipt['provenance']['commit_id']['sequence'] = 999
        self.assertEqual(descriptor['content_identity']['provenance']['commit_id']['sequence'], 11)

    def test_unbound_revision_and_hash_never_become_live_authority(self):
        layer = NS(map_id='test-map', frame_id='map', revision=8, metadata={'content_hash': 'legacy'})
        descriptor = _live_map_layer_descriptor(layer, content_revision=8)
        self.assertIsNone(descriptor['revision'])
        self.assertIsNone(descriptor['content_identity'])
        self.assertEqual(descriptor['identity_status'], 'legacy_unbound')
        self.assertEqual(_live_map_layer_descriptor(None)['identity_status'], 'absent')

    def test_missing_zero_and_mismatched_receipts_are_rejected(self):
        for field in ('session_id', 'session_generation', 'occupancy_revision', 'source_timestamp_ns',
                      'backend_kind', 'backend_generation', 'frame_id'):
            receipt = identity()
            receipt['provenance'].pop(field)
            with self.subTest(field=field), self.assertRaises(ValueError):
                _committed_occupancy_identity(receipt)
        for value in (0, True, '7'):
            receipt = identity()
            receipt['provenance']['occupancy_revision'] = value
            with self.assertRaises(ValueError):
                _committed_occupancy_identity(receipt)
        with self.assertRaises(ValueError):
            _live_map_layer_descriptor(grid(identity()), content_revision=99)
        with self.assertRaises(ValueError):
            _live_map_layer_descriptor(grid(identity()), mapping_identity_override=('other', 'test-map', 3))

    def test_runtime_preserves_actual_revision_composite_source_time_and_bytes(self):
        value = runtime(self)
        cost = CostMap(map_id='test-map', frame_id='map', width=2, height=1, resolution_m=.1,
                       origin=Pose(frame_id='map'), data=[1., 2.])
        layer = grid(identity())
        with patch('hashlib.sha256', side_effect=AssertionError('live hash')):
            snapshot = value._build_live_map_snapshot(live_occupancy=layer, live_cost_map=cost,
                occupancy_descriptor=_live_map_layer_descriptor(layer),
                cost_map_descriptor=_live_map_layer_descriptor(cost), live_status={}, seed_bundle=None,
                mapping_context={'session_id': 'mapping-session', 'session_generation': 3, 'map_name': 'test-map'})
        self.assertEqual(snapshot.update_sequence, 7)
        self.assertEqual(snapshot.metadata['content_revision'], 7)
        self.assertIsNone(snapshot.metadata['cost_map_content_revision'])
        self.assertEqual(snapshot.metadata['layer_content_identities']['cost_map']['identity_status'], 'legacy_unbound')
        self.assertEqual(snapshot.source_epoch, _receipt_source_epoch(identity()))
        self.assertEqual(snapshot.metadata['timestamp_ns'], 123456789)
        self.assertEqual(snapshot.occupancy_grid.timestamp, layer.timestamp)
        self.assertIs(snapshot.occupancy_grid.data, layer.data)
        self.assertNotIn('content_hash', snapshot.metadata)
        with self.assertRaises(ValueError):
            value._build_live_map_snapshot(live_occupancy=grid(), live_cost_map=None,
                occupancy_descriptor=_live_map_layer_descriptor(grid()), cost_map_descriptor=_live_map_layer_descriptor(None),
                live_status={}, seed_bundle=None, mapping_context={})

    def test_tracker_distinguishes_receipt_progress_from_physical_change(self):
        value = runtime(self)
        now = datetime.now(timezone.utc).isoformat()
        def observe(receipt, count, changed):
            value._update_stream_tracker({'occupancy_message_count': count, 'latest_occupancy_timestamp': now,
                'latest_occupancy_received_at': now, 'occupancy_metrics': {'content_identity': receipt,
                    'content_changed': changed, 'known_cell_count': 2}})
        observe(identity(), 1, True)
        changed_at = value._map_stream_tracker['last_content_change_at']
        observe(identity(19, commit=30), 2, False)
        self.assertEqual(value._map_stream_tracker['content_revision'], 19)
        self.assertTrue(value._map_stream_tracker['receipt_progress'])
        self.assertFalse(value._map_stream_tracker['content_changed'])
        self.assertEqual(value._map_stream_tracker['last_content_change_at'], changed_at)
        observe(identity(19, commit=30), 2, True)
        self.assertFalse(value._map_stream_tracker['receipt_progress'])
        self.assertFalse(value._map_stream_tracker['content_changed'])
        value._update_stream_tracker({})
        self.assertIsNone(value._map_stream_tracker['content_revision'])


class MappingReceiptTests(unittest.TestCase):
    def setUp(self):
        self.bus = StreamBus()
        self.addCleanup(self.bus.shutdown)
        self.owner = NS(is_map_snapshot_source_current=lambda **kw: True)
        self.service = MappingService(provider_owner=self.owner, state_store=StateStore(), stream_bus=self.bus)

    def admit(self, receipt=None, cells=b'\x00\x64', **kwargs):
        receipt = receipt or identity()
        return self.service.ingest_external_snapshot(map_name='test-map', source_name='geometry_runtime',
            source_epoch=_receipt_source_epoch(receipt), occupancy_grid=grid(receipt, cells),
            metadata=metadata(receipt), **kwargs)

    def test_actual_receipt_propagates_replays_drop_and_conflicts_stay_outside_lock(self):
        receipt = identity()
        first = self.admit(receipt)
        self.assertEqual(first.update_sequence, 7)
        self.assertEqual(first.metadata['content_identity'], receipt)
        frame = self.bus.latest_frame('map.frame')
        self.assertEqual(frame.metadata['content_identity'], receipt)
        self.assertEqual(frame.payload.metadata['content_revision'], 7)
        compare = mapping._same_map_layer_payload
        def unlocked(*args):
            self.assertFalse(self.service._lock._is_owned())
            self.assertFalse(self.service._publication_lock._is_owned())
            return compare(*args)
        with patch.object(mapping, '_same_map_layer_payload', side_effect=unlocked):
            self.assertIs(self.admit(), first)
            with self.assertRaises(GatewayError) as conflict:
                self.admit(cells=b'\x64\x00')
        self.assertEqual(conflict.exception.error_code, 'map_snapshot_revision_conflict')
        receipt['provenance']['commit_id']['sequence'] = 99
        self.assertEqual(first.metadata['content_identity']['provenance']['commit_id']['sequence'], 11)

    def test_same_body_new_receipt_advances_actual_revision_not_local_clock(self):
        self.admit()
        newer = self.admit(identity(24, commit=35))
        self.assertEqual(newer.update_sequence, 24)
        self.assertEqual(newer.metadata['content_revision'], 24)
        with self.assertRaises(GatewayError):
            self.admit(identity(23, commit=34))
        with self.assertRaises(GatewayError):
            self.admit(identity(24, commit=36))
        restarted = self.admit(identity(1, commit=1, nonce='cd' * 16))
        self.assertEqual(restarted.update_sequence, 1)

    def test_required_live_receipt_does_not_fall_back_to_synthetic_counter(self):
        with self.assertRaises(GatewayError) as missing:
            self.service.ingest_external_snapshot(map_name='test-map', source_name='geometry_runtime',
                occupancy_grid=grid(), metadata={'source_kind': 'platform_live_map_provider'})
        self.assertEqual(missing.exception.error_code, 'map_snapshot_receipt_missing')
        legacy = self.service.ingest_external_snapshot(map_name='test-map', source_name='asset', source_epoch='saved',
            occupancy_grid=grid(), update_sequence_override=5,
            metadata={'source_kind': 'map_asset_bundle', 'map_snapshot_kind': 'live_streaming', 'content_hash': 'old-format'})
        self.assertEqual(legacy.update_sequence, 5)
        self.assertIsNone(legacy.metadata['content_revision'])
        self.assertNotIn('content_identity', legacy.metadata)

    def test_raw_shared_bytes_have_separate_view_identity_without_synthetic_revision(self):
        receipt = identity()
        layer = grid(receipt)
        raw_receipt = identity(representation='raw_physical')
        options = dict(map_name='test-map', source_name='geometry_runtime', source_epoch=_receipt_source_epoch(receipt),
            occupancy_grid=layer, metadata=metadata(receipt), stream_occupancy_grid=layer,
            stream_occupancy_source='raw_physical', stream_occupancy_metadata={'content_identity': raw_receipt})
        accepted = self.service.ingest_external_snapshot(**options)
        displayed = self.bus.latest_frame('map.frame').payload
        self.assertNotEqual(displayed.source_epoch, accepted.source_epoch)
        self.assertEqual(displayed.update_sequence, 7)
        self.assertEqual(displayed.metadata['content_identity']['representation'], 'raw_physical')
        self.assertEqual(displayed.metadata['layer_content_identities']['occupancy']['content_identity'], raw_receipt)
        self.assertEqual(accepted.metadata['content_identity']['representation'], 'effective')
        self.assertEqual(self.service._display_sequence, 0)
        self.assertIs(self.service.ingest_external_snapshot(**options), accepted)
        options['stream_occupancy_metadata'] = {}
        with self.assertRaises(GatewayError):
            self.service.ingest_external_snapshot(**options)

    def test_provider_push_and_runtime_poll_share_the_same_identity(self):
        accepted = self.admit()
        snapshot = live_snapshot(runtime(self))
        snapshot._source_owner_id = id(self.owner)
        self.owner.get_map_snapshot = lambda: snapshot
        with patch.object(mapping, '_capture_map_layer', side_effect=AssertionError('trusted refresh copied')):
            self.assertIs(self.service.refresh(), accepted)

    def test_frame_metadata_never_sums_independent_layer_revisions(self):
        frame_metadata = self.service._build_map_frame_metadata(
            {'occupancy_content_revision': 5, 'cost_map_content_revision': 8}, update_sequence=100)
        self.assertIsNone(frame_metadata['content_revision'])
        self.assertEqual(frame_metadata['occupancy_content_revision'], 5)
        self.assertEqual(frame_metadata['cost_map_content_revision'], 8)


if __name__ == '__main__':
    unittest.main(verbosity=2)
