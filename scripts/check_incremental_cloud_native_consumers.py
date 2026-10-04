#!/usr/bin/env python3
"""Structural guard for native metadata-only consumers; not a Rust build/test."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'services/geometry/nuwax_geometry_runtime/src'
LCM = (SRC / 'lcm_runtime.rs').read_text()
SERVER = (SRC / 'server.rs').read_text()


def section(source, start, end):
    return source[source.index(start):source.index(end, source.index(start))]


class NativeIncrementalConsumerContracts(unittest.TestCase):
    def test_committed_identity_is_payload_free(self):
        for name, end in [('pub(crate) struct CommittedMappingCloudMap', 'async fn ingest_mapping_points_with_backend'),
                          ('struct CanonicalCloudPublication', 'pub(crate) struct NavigationMap3DPublishContext')]:
            body = section(LCM, name, end)
            self.assertIn('CommittedCloudMapMetadata', body)
            self.assertNotIn('xyz_f32_le', body)

    def test_guard_reads_only_metadata(self):
        for start, end in [('fn canonical_committed_cloud_publication_matches_writer', 'struct LegacyGlobalMapDemandConfig'),
                           ('fn latest_canonical_cloud_publication', 'async fn publish_latest_canonical_cloud_map'),
                           ('async fn ensure_canonical_mapping_cloud_identity', 'async fn publish_canonical_mapping_cloud_while_lifecycle_fenced'),
                           ('async fn ensure_read_only_mapping_identity', 'async fn registered_lio_correction_input')]:
            body = section(LCM, start, end)
            self.assertIn('committed_metadata()', body)
            self.assertNotIn('shared_committed_snapshot()', body)
            self.assertNotIn('xyz_f32_le', body)
        equality = section(LCM, 'pub(crate) fn same_cloud_metadata', 'fn export_canonical_cloud_payload')
        self.assertIn('producer_epoch', equality)
        self.assertIn('layout_generation', equality)

    def test_fine_navigation_never_copies_then_clears(self):
        body = section(LCM, 'fn build_navigation_map_3d_publication_frame', 'fn cloud_metadata_matches_notification')
        self.assertIn('if publication.fine.is_some()', body)
        self.assertNotIn('cloud.data.clear()', body)
        self.assertNotIn('publication.xyz_f32_le', body)

    def test_export_is_gated_and_validated_before_snapshot(self):
        body = section(LCM, 'fn export_canonical_cloud_payload', 'fn canonical_cloud_payload_matches')
        self.assertLess(body.index('same_cloud_metadata'), body.index('shared_committed_snapshot()'))
        self.assertLess(body.index('last_cloud_receipt()'), body.index('shared_committed_snapshot()'))
        publish = section(LCM, 'async fn publish_latest_canonical_cloud_map_with_navigation', 'struct ExpectedRelocalizationMode')
        self.assertIn('legacy_full_demand.is_some() || legacy_navigation', publish)
        self.assertNotIn('std::fs::', publish)
        self.assertNotIn('std::env::', publish)

    def test_demand_is_owned_and_activation_based(self):
        self.assertIn('"legacy-global-map-demand"', LCM)
        self.assertIn('legacy_global_map_demand_future(handle.clone()', LCM)
        self.assertIn('Duration::from_millis(500)', LCM)
        config = section(LCM, 'struct LegacyGlobalMapDemandConfig', 'fn legacy_global_map_demand_future')
        self.assertNotIn('NUWAX_PERSON_FOLLOW_AUTOSTART', config)
        for key in ['subscriptions_ready', 'boot_id', 'starttime', 'NUWAX_GEOMETRY_LEGACY_GLOBAL_MAP_FULL']:
            self.assertIn(key, config)
        env = (ROOT / '.env.example').read_text()
        self.assertIn('NUWAX_GEOMETRY_LEGACY_GLOBAL_MAP_FULL=false', env)
        for relative in ['scripts/start_gateway.sh', 'deploy/systemd/nuwax-geometry-runtime.service']:
            self.assertIn('NUWAX_PERSON_FOLLOW_READY_FILE', (ROOT / relative).read_text())

    def test_native_behavior_tests_and_explicit_export_preserved(self):
        for name in ['fine_navigation_without_full_demand_never_materializes_cumulative_xyz',
                     'follower_demand_watcher_replays_static_map_once_and_stops_full_updates',
                     'legacy_full_demand_requires_explicit_opt_in_or_live_follower_identity',
                     'canonical_cloud_publication_identity_includes_receipt_epoch_and_layout']:
            self.assertIn('fn ' + name, LCM)
        self.assertIn('full_snapshot_materialized()', LCM)
        self.assertIn('snapshot: crate::pointcloud::CommittedCloudMapSnapshot', SERVER)
        self.assertIn('xyz_sha256: crate::map_checksum::sha256_hex(&snapshot.xyz_f32_le)', SERVER)
        self.assertNotIn('xyz_f32_le: target.xyz_f32_le.clone().into()', SERVER)


if __name__ == '__main__':
    unittest.main(verbosity=2)
