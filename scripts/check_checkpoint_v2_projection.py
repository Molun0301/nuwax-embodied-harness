#!/usr/bin/env python3
"""Exact socket/process-free checkpoint-v2 and assigned-storage qualification.

The one fork/abrupt-exit checkpoint test is intentionally excluded. This runner
loads real source from its own checkout and rejects stale repository imports.
"""
from contextlib import ExitStack
import hashlib
import importlib.util
import os
from pathlib import Path
import platform
import socket
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SELECTION = {'tests/test_checkpoint_identity_v2.py': ('IdentityV2Tests.test_actual_proto_export_store_restart_reopen_reupload',
                                          'IdentityV2Tests.test_explicit_backend_and_occupancy_members_reject_skew_and_conflicts',
                                          'IdentityV2Tests.test_legacy_wire_rejects_new_member_fields_before_hash_or_artifact_reads',
                                          'IdentityV2Tests.test_public_export_rejects_missing_member_before_artifact_download',
                                          'IdentityV2Tests.test_saved_member_inventory_roundtrips_without_inventing_identity',
                                          'IdentityV2Tests.test_pre_stage_a_saved_manifest_fixture_reopens_exact_member_ids',
                                          'IdentityV2Tests.test_asset_identity_is_strict_immutable_and_not_a_digest',
                                          'IdentityV2Tests.test_assigned_activation_wire_contains_one_canonical_checkpoint_body',
                                          'IdentityV2Tests.test_assigned_activation_metadata_rejects_before_body_materialization',
                                          'IdentityV2Tests.test_same_id_retry_streams_and_reuses_input',
                                          'IdentityV2Tests.test_reopen_checks_expected_assigned_identity_without_legacy_fallback',
                                          'IdentityV2Tests.test_v2_activation_gate_precedes_rpc_and_asset_reads',
                                          'IdentityV2Tests.test_v2_pcd_materialization_streams_retained_cloud_member',
                                          'IdentityV2Tests.test_v2_native_database_uses_assigned_identity_column',
                                          'IdentityV2Tests.test_runtime_v2_activation_gate_precedes_native_activator',
                                          'IdentityV2Tests.test_explicit_v2_preparation_keeps_assigned_metadata_and_exact_members',
                                          'IdentityV2Tests.test_actual_v2_preparation_finalizes_and_reopens_assigned_asset',
                                          'IdentityV2Tests.test_finalization_default_stays_legacy_until_activation_contract_is_paired',
                                          'IdentityV2Tests.test_same_checkpoint_identity_conflicts_before_overwrite',
                                          'IdentityV2Tests.test_artifact_id_descriptor_conflict_across_checkpoint',
                                          'IdentityV2Tests.test_download_requires_full_reference_and_correct_offset',
                                          'IdentityV2Tests.test_explicit_version_rejects_old_response_before_download',
                                          'IdentityV2Tests.test_mixed_identity_and_source_conflicts',
                                          'IdentityV2Tests.test_frozen_payload_constructs_without_digest',
                                          'IdentityV2Tests.test_fault_phases_keep_previous_selector_and_retry',
                                          'IdentityV2Tests.test_corrupt_live_selector_preserves_uncertain_gc_set',
                                          'IdentityV2Tests.test_single_freeze_per_large_reopened_member',
                                          'IdentityV2Tests.test_legacy_read_preserves_literal_aliases_and_missing_canonical_bytes',
                                          'IdentityV2Tests.test_gc_keeps_union_of_v1_v2_paths_without_payload_reads',
                                          'IdentityV2Tests.test_fine_and_dense_same_id_changed_bytes_fail',
                                          'IdentityV2Tests.test_symlink_hardlink_and_traversal_members_are_rejected',
                                          'IdentityV2Tests.test_publish_does_not_overwrite_unknown_existing_artifact',
                                          'IdentityV2Tests.test_sealed_reupload_replays_all_chunks',
                                          'IdentityV2Tests.test_default_export_remains_explicitly_legacy',
                                          'IdentityV2Tests.test_distinct_roles_cannot_reuse_artifact_id',
                                          'IdentityV2Tests.test_upload_rejects_forged_reference_and_changed_bytes_before_transfer',
                                          'IdentityV2Tests.test_canonical_role_budgets_fail_before_publication',
                                          'IdentityV2Tests.test_assigned_nil_ids_rejected_and_legacy_ids_stay_opaque',
                                          'IdentityV2Tests.test_multichunk_sealed_and_partial_reupload_use_retained_frontier',
                                          'IdentityV2Tests.test_upload_checks_initial_and_final_frontier',
                                          'IdentityV2Tests.test_cloud_target_kind_is_empty_and_unadmitted_owner_cannot_upload'),
 'tests/test_assigned_asset_storage.py': ('AssignedAssetStorageTests.test_reopen_exact_identity_and_same_id_exact_request_is_idempotent',
                                          'AssignedAssetStorageTests.test_same_id_changed_body_or_inventory_is_rejected_without_overwrite',
                                          'AssignedAssetStorageTests.test_different_assigned_id_never_overwrites_current',
                                          'AssignedAssetStorageTests.test_assigned_finalize_preserves_legacy_current_in_independent_namespace',
                                          'AssignedAssetStorageTests.test_concurrent_first_publishers_preserve_exact_one_owner',
                                          'AssignedAssetStorageTests.test_pre_and_post_rename_faults_restart_and_retry',
                                          'AssignedAssetStorageTests.test_postpublication_metadata_failure_is_recoverable_from_durable_owner',
                                          'AssignedAssetStorageTests.test_cleanup_refuses_replacement_directory_or_symlink',
                                          'AssignedAssetStorageTests.test_cleanup_replacement_after_owner_pin_is_restored_without_deletion',
                                          'AssignedAssetStorageTests.test_cleanup_deletes_pinned_quarantine_and_preserves_new_public_name',
                                          'AssignedAssetStorageTests.test_missing_mixed_manifest_versions_fail_before_bundle_body_read',
                                          'AssignedAssetStorageTests.test_mixed_checkpoint_source_is_rejected_before_candidate_serialization',
                                          'AssignedAssetStorageTests.test_contracts_never_issue_identity_or_reinterpret_digest',
                                          'AssignedAssetStorageTests.test_owned_snapshot_shares_immutable_cells_and_detaches_descriptors',
                                          'AssignedAssetStorageTests.test_equal_copy_updates_never_install_mutable_external_aliases',
                                          'AssignedAssetStorageTests.test_capture_and_validate_assigned_snapshot_without_fingerprint_hash',
                                          'AssignedAssetStorageTests.test_assigned_previous_identity_requires_catalog_owner_and_legacy_writer_is_gated',
                                          'LegacyAssetStorageCompatibilityTests.test_literal_pred1_bundle_and_manifest_reopen_and_repeat_finalize',
                                          'LegacyAssetStorageCompatibilityTests.test_legacy_current_rejects_assigned_expectation_before_hashing')}
SOURCES = ('services/geometry/runtime.py', 'services/mapping/asset_store.py',
           'tests/test_checkpoint_identity_v2.py', 'tests/test_assigned_asset_storage.py')


class NoSocket(socket.socket):
    def __new__(cls, *args, **kwargs):
        raise AssertionError('socket construction forbidden')


def verify_imports():
    for name, module in tuple(sys.modules.items()):
        raw = getattr(module, '__file__', None)
        if not raw:
            continue
        declared = ROOT.joinpath(*name.split('.'))
        paths = (declared.with_suffix('.py'), declared / '__init__.py')
        expected = next((path for path in paths if path.is_file()), None)
        if expected is not None and Path(raw).resolve() != expected.resolve():
            raise AssertionError('repository import outside reviewed tree: ' + name)
    for name in ('services.geometry.runtime', 'services.geometry.geometry_runtime_provider',
                 'services.geometry.mapping_backend_checkpoint_store', 'services.mapping.asset_store'):
        module = sys.modules[name]
        expected = ROOT.joinpath(*name.split('.')).with_suffix('.py')
        if Path(module.__file__).resolve() != expected:
            raise AssertionError('owner source mismatch: ' + name)
        print('SOURCE', name, expected, flush=True)


def main():
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT))
    before = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCES}
    print('INTERPRETER', sys.executable, sys.version, flush=True)
    print('SOURCE_SHA256', before, flush=True)
    # Python 3.8 uuid imports query uname; prime stdlib discovery before test guards.
    platform.uname()
    with ExitStack() as guards:
        guards.enter_context(patch.object(socket, 'socket', NoSocket))
        guards.enter_context(patch.object(subprocess, 'Popen', side_effect=AssertionError('process forbidden')))
        if hasattr(os, 'fork'):
            guards.enter_context(patch.object(os, 'fork', side_effect=AssertionError('fork excluded from selection')))
        import grpc
        import lcm
        for name in ('secure_channel', 'insecure_channel'):
            guards.enter_context(patch.object(grpc, name, side_effect=AssertionError('real grpc forbidden')))
        guards.enter_context(patch.object(lcm, 'LCM', side_effect=AssertionError('real LCM forbidden')))
        from services.geometry import runtime, geometry_runtime_provider
        guards.enter_context(patch.object(runtime.PlatformGeometryRuntime, '__init__', side_effect=AssertionError('live runtime forbidden')))
        guards.enter_context(patch.object(runtime.GeometryRuntimeSubprocess, 'start', side_effect=AssertionError('native launch forbidden')))
        guards.enter_context(patch.object(geometry_runtime_provider.GeometryRuntimeProvider, '__init__', side_effect=AssertionError('live provider forbidden')))
        suite = unittest.TestSuite()
        for index, (relative, selectors) in enumerate(SELECTION.items()):
            path = ROOT / relative
            spec = importlib.util.spec_from_file_location('checkpoint_projection_case_%d' % index, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            for selector in selectors:
                cls, method = selector.split('.', 1)
                suite.addTest(getattr(module, cls)(method))
        if suite.countTestCases() != 60:
            raise AssertionError('exact 60-test selection changed')
        verify_imports()
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        verify_imports()
        after = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCES}
        if before != after:
            raise AssertionError('source changed during execution')
        return int(not result.wasSuccessful() or bool(result.skipped) or result.testsRun != 60)


if __name__ == '__main__':
    raise SystemExit(main())
