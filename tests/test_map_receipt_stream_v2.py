"""Offline live-map receipt admission; no RPC, hardware, or content hashes."""
import hashlib
import threading
import unittest
from unittest.mock import patch
from services.geometry import geometry_runtime_provider as module

pb = module.pb2


def update(revision=1, sequence=None, data=b'\xff\x64', timestamp=11, nonce=b'N'*16):
    return pb.MapUpdateNotification(schema_version=2, session_id='source', map_name='room', session_generation=3,
        meta=pb.OccupancyGridMeta(width=2, height=1, resolution=0.125, origin_x=-1, origin_y=2,
            frame_id='map', revision=revision, timestamp_ns=timestamp), data=data, size=len(data), last_seen_ms=7,
        raw_representation=pb.RAW_MAP_REPRESENTATION_SAME_AS_EFFECTIVE,
        occupancy_receipt=pb.CanonicalOccupancyCommitReceipt(
            commit_id=pb.MappingCommitIdentity(boot_nonce=nonce, sequence=revision if sequence is None else sequence),
            session_id='source', session_generation=3, occupancy_revision=revision, source_timestamp_ns=timestamp,
            backend_kind='voxel', backend_generation=2))


def provider():
    value = module.GeometryRuntimeProvider(None)
    value._map_state_lock = threading.Lock()
    value._map_connection_epoch = 1
    value._mapping_transition_epoch = 1
    value._mapping_identity = ('source', 'room', 3)
    value._mapping_transition_inflight = False
    value._stopping = False
    return value


class MapReceiptStreamTests(unittest.TestCase):
    def test_admission_without_hash_and_metadata_cached_atomically(self):
        p = provider()
        with patch.object(hashlib, 'sha256', side_effect=AssertionError('application hash')):
            result = p._map_update_to_payload(update())
        grid = result['occupancy_grid']
        self.assertIsNot(grid, p._last_occupancy_grid)
        self.assertIs(grid.data, p._last_occupancy_grid.data)
        self.assertEqual(grid.metadata['map_provenance']['commit_id']['sequence'], 1)
        self.assertEqual(grid.metadata['timestamp_ns'], 11)
        self.assertIs(grid.data.raw, p._last_map_update_evidence[2][2])
        self.assertNotIn('content_hash', result['metadata'])

    def test_cached_getter_reuses_bytes_and_isolates_mutable_shells(self):
        p = provider(); result = p._map_update_to_payload(update())
        with patch.object(p, 'get_occupancy_grid_data', side_effect=AssertionError('unneeded unary read')):
            returned = p.get_occupancy_grid()
        self.assertIs(returned.data, p._last_occupancy_grid.data)
        returned.metadata['map_provenance']['commit_id']['sequence'] = 999
        result['occupancy_grid'].data = [0, 0]
        self.assertEqual(p._last_occupancy_grid.data.raw, b'\xff\x64')
        self.assertEqual(p._last_occupancy_grid.metadata['map_provenance']['commit_id']['sequence'], 1)

    def test_unbound_unary_cannot_replace_stream_owner(self):
        p = provider(); p._last_occupancy_grid = None
        payload = dict(width=2, height=1, resolution=.125, origin_x=0, origin_y=0,
                       frame_id='map', timestamp_ns=11, data=b'\xff\x64', revision=1)
        with patch.object(p, 'get_occupancy_grid_data', return_value=payload):
            grid = p.get_occupancy_grid()
        self.assertEqual(grid.metadata['identity_status'], 'unbound_unary')
        self.assertIsNone(p._last_occupancy_grid)

    def test_identical_replay_does_no_grid_or_count_work(self):
        p = provider(); p._map_update_to_payload(update())
        old = p._last_map_update_evidence
        with patch.object(p, '_occupancy_grid_from_components', side_effect=AssertionError('replay rebuilt grid')):
            self.assertIsNone(p._map_update_to_payload(update()))
        self.assertIs(old, p._last_map_update_evidence)

    def test_equal_receipt_mutations_retain_current(self):
        for mutate in (lambda u: setattr(u, 'data', b'\x00\x64'),
                       lambda u: setattr(u.meta, 'origin_x', 4),
                       lambda u: setattr(u, 'last_seen_ms', 8),
                       lambda u: setattr(u, 'raw_representation', pb.RAW_MAP_REPRESENTATION_EFFECTIVE_FALLBACK)):
            p = provider(); p._map_update_to_payload(update()); old = p._last_occupancy_grid
            candidate = update(); mutate(candidate)
            with self.assertRaises(ValueError): p._map_update_to_payload(candidate)
            self.assertIs(old, p._last_occupancy_grid)

    def test_new_evidence_keeps_physical_changed_false(self):
        p = provider(); p._map_update_to_payload(update())
        result = p._map_update_to_payload(update(4, 8, timestamp=29))
        self.assertFalse(result['metadata']['occupancy_metrics']['content_changed'])
        self.assertEqual(result['metadata']['content_revision'], 4)
        self.assertEqual(result['metadata']['timestamp_ns'], 29)
        self.assertEqual(result['metadata']['occupancy_message_count'], 2)

    def test_rollback_partial_and_owner_changes_rejected(self):
        p = provider(); p._map_update_to_payload(update(4, 8))
        for candidate in (update(3, 9), update(5, 7), update(4, 9), update(5, 8), update(5, 9, nonce=b'A'*16)):
            with self.assertRaises(ValueError): p._map_update_to_payload(candidate)

    def test_new_connection_admits_unordered_nonce(self):
        p = provider(); p._map_update_to_payload(update(4, 8, nonce=b'Z'*16))
        p._map_connection_epoch += 1
        result = p._map_update_to_payload(update(1, 1, nonce=b'A'*16))
        self.assertEqual(result['metadata']['map_provenance']['commit_id']['boot_nonce'], (b'A'*16).hex())

    def test_explicit_raw_and_fallback(self):
        p = provider(); candidate = update(); candidate.raw_representation = pb.RAW_MAP_REPRESENTATION_SEPARATE_RAW; candidate.raw_data = b'\xff\x00'
        result = p._map_update_to_payload(candidate)
        self.assertEqual(result['display_occupancy_grid'].data.raw, b'\xff\x00')
        self.assertEqual(result['display_occupancy_grid'].metadata['content_identity']['representation'], 'raw_physical')
        candidate = update(2); candidate.raw_representation = pb.RAW_MAP_REPRESENTATION_EFFECTIVE_FALLBACK
        result = p._map_update_to_payload(candidate)
        self.assertEqual(result['display_occupancy_source'], 'effective_fallback')

    def test_fallback_display_tracks_effective_body_changes(self):
        p = provider(); first = update(); first.raw_representation = pb.RAW_MAP_REPRESENTATION_EFFECTIVE_FALLBACK
        p._map_update_to_payload(first)
        next_update = update(2, data=b'\x00\x64'); next_update.raw_representation = pb.RAW_MAP_REPRESENTATION_EFFECTIVE_FALLBACK
        result = p._map_update_to_payload(next_update)
        self.assertTrue(result['display_occupancy_metadata']['occupancy_metrics']['content_changed'])

    def test_version_skew_missing_receipt_and_malformed_geometry(self):
        for mutate in (lambda u: setattr(u, 'schema_version', 1), lambda u: u.ClearField('occupancy_receipt'),
                       lambda u: setattr(u.occupancy_receipt.commit_id, 'boot_nonce', b'\0'*16),
                       lambda u: setattr(u, 'raw_representation', 0), lambda u: setattr(u.meta, 'frame_id', '/map'),
                       lambda u: setattr(u.meta, 'resolution', float('nan'))):
            candidate = update(); mutate(candidate)
            with self.assertRaises(ValueError): provider()._map_update_to_payload(candidate)

    def test_concurrent_newer_admission_prevents_old_candidate_install(self):
        p = provider(); p._map_update_to_payload(update())
        materialize = p._occupancy_grid_from_components
        entered = False
        def race(**kwargs):
            nonlocal entered
            if not entered:
                entered = True
                p._map_update_to_payload(update(3, 5, timestamp=33))
            return materialize(**kwargs)
        with patch.object(p, '_occupancy_grid_from_components', side_effect=race):
            with self.assertRaisesRegex(ValueError, 'stale'):
                p._map_update_to_payload(update(2, 4, timestamp=22))
        self.assertEqual(p._last_occupancy_grid_revision, 3)
        self.assertEqual(p._last_occupancy_grid.metadata['timestamp_ns'], 33)

    def test_transition_during_materialization_never_installs_candidate(self):
        p = provider(); p._map_update_to_payload(update()); old = p._last_occupancy_grid
        materialize = p._occupancy_grid_from_components
        def transition(**kwargs):
            p._mapping_transition_epoch += 1
            return materialize(**kwargs)
        with patch.object(p, '_occupancy_grid_from_components', side_effect=transition):
            with self.assertRaisesRegex(ValueError, 'transition'):
                p._map_update_to_payload(update(2))
        self.assertIs(p._last_occupancy_grid, old)

    def test_transition_and_stopped_reject_before_materialization(self):
        for field in ('_stopping', '_mapping_transition_inflight'):
            p = provider(); setattr(p, field, True)
            with patch.object(p, '_occupancy_grid_from_components', side_effect=AssertionError('materialized rejected update')):
                with self.assertRaises(ValueError): p._map_update_to_payload(update())

    def test_bootstrap_identity_updates_existing_transition_fence(self):
        p = provider(); p._mapping_identity = None
        result = p._map_update_to_payload(update())
        self.assertEqual(p._mapping_transition_epoch, 2)
        self.assertEqual(result.delivery_token, p._map_delivery_token_locked())
        self.assertIsNone(p._map_update_to_payload(update()))

if __name__ == '__main__': unittest.main()
