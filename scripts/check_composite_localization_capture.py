#!/usr/bin/env python3
"""Exact socket/process-free composite localization sampling qualification.

The fresh-interpreter contract case is excluded. All selected cases load real
source from this checkout; stale repository imports and skipped tests fail.
"""
from contextlib import ExitStack
import hashlib
import importlib.util
import os
from pathlib import Path
import socket
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SELECTION = {'tests/test_composite_localization_capture.py': ('ProviderCompositeCaptureTests.test_real_protobuf_decode_tree_then_pose_and_static_edge_merge',
                                                  'ProviderCompositeCaptureTests.test_pre_admission_rejects_disconnected_stopping_transition_or_missing_stub',
                                                  'ProviderCompositeCaptureTests.test_missing_rpc_rejects_without_starting_partial_read',
                                                  'ProviderCompositeCaptureTests.test_rpc_failure_discards_both_values_without_retry',
                                                  'ProviderCompositeCaptureTests.test_same_identity_transition_aba_invalidates_tree_and_pose_waits',
                                                  'ProviderCompositeCaptureTests.test_reconnect_and_stub_replacement_never_mix_stubs',
                                                  'ProviderCompositeCaptureTests.test_connection_epoch_change_rejects_even_with_same_stub_and_identity',
                                                  'ProviderCompositeCaptureTests.test_stopping_disconnection_or_open_transition_during_either_rpc_rejects_pair',
                                                  'ProviderCompositeCaptureTests.test_static_edge_list_is_pinned_before_either_rpc_wait',
                                                  'BundleCompositeCaptureTests.test_real_provider_composite_runs_outside_runtime_and_map_locks',
                                                  'BundleCompositeCaptureTests.test_generic_backend_preserves_tree_then_pose_compatibility',
                                                  'BundleCompositeCaptureTests.test_composite_rejection_and_exception_never_fall_back',
                                                  'BundleCompositeCaptureTests.test_pending_stopping_disabled_or_no_backend_reject_before_capture',
                                                  'BundleCompositeCaptureTests.test_generic_capture_pins_backend_across_tree_and_pose_waits',
                                                  'BundleCompositeCaptureTests.test_composite_backend_replacement_rejects_without_retrying_replacement',
                                                  'BundleCompositeCaptureTests.test_attachment_aba_pending_stopping_and_disabled_during_capture_reject',
                                                  'BundleCompositeCaptureTests.test_same_metadata_reinstall_and_equal_generation_replacement_reject',
                                                  'ServiceCompositeCaptureTests.test_generic_provider_retains_sequential_compatibility',
                                                  'ServiceCompositeCaptureTests.test_composite_success_rejection_and_exception_never_fall_back',
                                                  'ServiceCompositeCaptureTests.test_actual_assigned_preparation_uses_composite_without_publication',
                                                  'ServiceCompositeCaptureTests.test_actual_preparation_drops_invalidated_pair_without_sequential_retry'),
 'tests/test_assigned_localization_session.py': ('AssignedSessionContractTests.test_import_identity_and_legacy_serialization',
                                                 'AssignedSessionContractTests.test_typed_roundtrip_preserves_owners_and_raw_bits',
                                                 'AssignedSessionContractTests.test_alias_isolation_and_validated_copy',
                                                 'AssignedSessionContractTests.test_rejects_mixed_missing_unapplied_and_forged_owner_fields',
                                                 'AssignedSessionContractTests.test_legacy_owner_assignment_and_bool_copy_fail_before_mutation',
                                                 'AssignedSessionContractTests.test_descriptive_security_and_protocol_digests_are_not_owner_authority',
                                                 'AssignedSessionContractTests.test_typed_candidate_body_is_rejected_before_any_serializer_or_copy',
                                                 'AssignedSessionContractTests.test_payload_objects_are_rejected_without_copy_or_serialization',
                                                 'AssignedSessionServiceTests.test_no_refresh_activation_works_without_provider_and_does_not_make_ready',
                                                 'AssignedSessionServiceTests.test_provider_properties_and_metadata_are_captured_before_guards',
                                                 'AssignedSessionServiceTests.test_provider_nested_body_is_rejected_before_serialization_or_copy',
                                                 'AssignedSessionServiceTests.test_provider_geometry_projection_preserves_contract_validation_and_alias_isolation',
                                                 'AssignedSessionServiceTests.test_valid_captured_pose_can_be_ready_without_relocalization_terminal',
                                                 'AssignedSessionServiceTests.test_terminal_without_pose_and_wrong_frame_never_becomes_ready',
                                                 'AssignedSessionServiceTests.test_same_metadata_new_runtime_installation_is_rejected_before_commit',
                                                 'AssignedSessionServiceTests.test_stale_session_reference_epoch_and_owner_do_not_mutate',
                                                 'AssignedSessionServiceTests.test_newer_session_during_blocked_provider_capture_is_preserved',
                                                 'AssignedSessionServiceTests.test_wrong_guarded_owner_and_terminal_fail_before_any_mutation',
                                                 'AssignedSessionServiceTests.test_active_drainer_cannot_publish_deferred_record_under_outer_guards',
                                                 'AssignedSessionServiceTests.test_out_of_order_release_keeps_fifo_old_event_identity_and_actual_token',
                                                 'AssignedSessionServiceTests.test_reentrant_callback_newer_session_is_never_restored',
                                                 'AssignedSessionServiceTests.test_exception_and_cancellation_after_commit_release_after_guards_unwind',
                                                 'AssignedSessionServiceTests.test_commit_failure_rolls_back_evicted_history_in_constant_space',
                                                 'AssignedSessionServiceTests.test_failure_after_each_shared_mutation_rolls_back_and_unblocks_fifo',
                                                 'AssignedSessionServiceTests.test_callback_exception_or_cancellation_does_not_strand_ready_events',
                                                 'AssignedSessionServiceTests.test_prepare_failure_has_no_side_effect_and_lazy_metadata_is_rejected',
                                                 'AssignedSessionServiceTests.test_legacy_mutators_cannot_strip_assigned_owner',
                                                 'AssignedSessionServiceTests.test_legacy_default_calls_still_publish_and_clear'),
 'tests/test_runtime_assigned_relocalization.py': ('AssignedRuntimeConsumerTests.test_snapshot_captures_mutable_aliases_and_shares_heavy_cell_owners',
                                                   'AssignedRuntimeConsumerTests.test_equal_metadata_new_install_invalidates_old_snapshot_and_target',
                                                   'AssignedRuntimeConsumerTests.test_legacy_unsealed_snapshot_read_keeps_generation_fence_without_hash',
                                                   'AssignedRuntimeConsumerTests.test_closed_native_availability_is_preserved_for_receipt_projection',
                                                   'AssignedRuntimeConsumerTests.test_partial_or_retired_diagnostics_are_preserved_but_cannot_bind',
                                                   'AssignedRuntimeConsumerTests.test_unsupported_mixed_missing_and_skewed_activation_states_fail_closed',
                                                   'AssignedRuntimeConsumerTests.test_only_exact_typed_finalized_assigned_asset_can_bind',
                                                   'AssignedRuntimeConsumerTests.test_dispatch_uses_retained_alias_and_canonical_terminal_not_conveniences',
                                                   'AssignedRuntimeConsumerTests.test_dispatch_detaches_caller_request_and_state_aliases_before_rpc',
                                                   'AssignedRuntimeConsumerTests.test_non_normalized_replay_or_mismatched_target_fails_before_dispatch',
                                                   'AssignedRuntimeConsumerTests.test_dispatch_completion_racing_equal_replacement_is_not_current',
                                                   'AssignedRuntimeConsumerTests.test_terminal_wrong_request_or_legacy_envelope_is_rejected',
                                                   'AssignedRuntimeConsumerTests.test_historical_query_survives_new_missing_and_unavailable_current_owner',
                                                   'AssignedRuntimeConsumerTests.test_query_never_reads_current_target_and_keeps_durable_origin',
                                                   'AssignedRuntimeConsumerTests.test_query_seals_exact_evidence_before_validation_alias_mutation',
                                                   'AssignedRuntimeConsumerTests.test_every_query_status_requires_exact_whole_normalized_request_echo',
                                                   'AssignedRuntimeConsumerTests.test_query_status_terminal_presence_and_whole_terminal_request_are_checked',
                                                   'AssignedRuntimeConsumerTests.test_private_current_result_guard_holds_only_existing_lock_and_rechecks',
                                                   'AssignedRuntimeConsumerTests.test_unavailable_current_target_allows_query_but_not_current_installation'),
 'tests/test_map_receipt_stream_v2.py': ('MapReceiptStreamTests.test_admission_without_hash_and_metadata_cached_atomically',
                                         'MapReceiptStreamTests.test_cached_getter_reuses_bytes_and_isolates_mutable_shells',
                                         'MapReceiptStreamTests.test_unbound_unary_cannot_replace_stream_owner',
                                         'MapReceiptStreamTests.test_identical_replay_does_no_grid_or_count_work',
                                         'MapReceiptStreamTests.test_equal_receipt_mutations_retain_current',
                                         'MapReceiptStreamTests.test_new_evidence_keeps_physical_changed_false',
                                         'MapReceiptStreamTests.test_rollback_partial_and_owner_changes_rejected',
                                         'MapReceiptStreamTests.test_new_connection_admits_unordered_nonce',
                                         'MapReceiptStreamTests.test_explicit_raw_and_fallback',
                                         'MapReceiptStreamTests.test_fallback_display_tracks_effective_body_changes',
                                         'MapReceiptStreamTests.test_version_skew_missing_receipt_and_malformed_geometry',
                                         'MapReceiptStreamTests.test_concurrent_newer_admission_prevents_old_candidate_install',
                                         'MapReceiptStreamTests.test_transition_during_materialization_never_installs_candidate',
                                         'MapReceiptStreamTests.test_transition_and_stopped_reject_before_materialization',
                                         'MapReceiptStreamTests.test_bootstrap_identity_updates_existing_transition_fence'),
 'tests/test_checkpoint_identity_v2.py': ('IdentityV2Tests.test_v2_activation_gate_precedes_rpc_and_asset_reads',
                                          'IdentityV2Tests.test_runtime_v2_activation_gate_precedes_native_activator',
                                          'IdentityV2Tests.test_finalization_default_stays_legacy_until_activation_contract_is_paired',
                                          'IdentityV2Tests.test_default_export_remains_explicitly_legacy')}
SOURCES = ('tests/test_composite_localization_capture.py', 'services/geometry/geometry_runtime_provider.py', 'services/geometry/runtime.py', 'services/localization/service.py', 'tests/test_assigned_localization_session.py', 'tests/test_runtime_assigned_relocalization.py', 'tests/test_map_receipt_stream_v2.py', 'tests/test_checkpoint_identity_v2.py', 'tests/test_assigned_relocalization_contract.py')


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
                 'services.localization.service', 'services.mapping.asset_store'):
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
            spec = importlib.util.spec_from_file_location('composite_capture_case_%d' % index, path)
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            spec.loader.exec_module(module)
            for selector in selectors:
                cls, method = selector.split('.', 1)
                suite.addTest(getattr(module, cls)(method))
        if suite.countTestCases() != 87:
            raise AssertionError('exact 87-test selection changed')
        verify_imports()
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        verify_imports()
        after = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCES}
        if before != after:
            raise AssertionError('source changed during execution')
        return int(not result.wasSuccessful() or bool(result.skipped) or result.testsRun != 87)


if __name__ == '__main__':
    raise SystemExit(main())
