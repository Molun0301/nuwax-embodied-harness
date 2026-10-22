"""Focused full-request journal boundaries for the public assigned loader.

Only native metadata is retained. No runtime process, network, payload hashing,
or unrelated catalog/component consolidation tests are exercised here.
"""
import json
import hashlib
import uuid
import tempfile
import unittest
from copy import deepcopy
from unittest.mock import Mock, patch

from tests.test_assigned_activation_catalog import receipt as old_receipt

from contracts.map_activation import AssignedActivationReceipt
from contracts.map_relocalization import current_assigned_relocalization_policy
from contracts.map_workspace import AssetIdentityV2
from services.mapping import catalog as module

ASSET = AssetIdentityV2(asset_id="a" * 32)
CHECKPOINT = "b" * 32


def activation_request():
    return {"activation_request_id": "activate:public.1", "session_id": "installed:session.2",
                "map_name": "room", "asset_identity": ASSET.model_dump(), "checkpoint_id": CHECKPOINT,
                "expected_runtime_generation": 3}


def activation_receipt(request=None, *, epoch=9, source_session=None):
    request = deepcopy(request or activation_request())
    value = old_receipt(request["activation_request_id"], epoch=epoch)
    for destination in (value, value["activation"], value["target"]["activation"]):
        destination["request"] = deepcopy(request)
    value["activation"]["runtime_generation"] = request["expected_runtime_generation"]
    value["target"]["activation"]["runtime_generation"] = request["expected_runtime_generation"]
    value["target"]["occupancy_receipt"]["session_id"] = request["session_id"]
    if source_session is not None:
        value["target"]["source_cloud_member"]["occupancy_receipt"]["session_id"] = source_session
    return AssignedActivationReceipt.admit(value)


def predecessor():
    return {"session_id": "old:session", "map_name": "previous", "session_generation": 11,
                "runtime_generation": 3, "mapping_backend_kind": "voxel"}


class AssignedWorkspaceJournalTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.repo = module.MapCatalogRepository(self.directory.name)
        self.request = activation_request()
        self.owner = activation_receipt()
        self.target = {"target_map_name": "room", "target_asset_identity": ASSET, "target_checkpoint_id": CHECKPOINT}
        self.identity = dict(self.target, activation_request_id=self.request["activation_request_id"],
                             activation_request=self.request)

    def reopen(self):
        return module.MapCatalogRepository(self.directory.name)

    def begin(self, **changes):
        args = dict(self.target, previous_map_name=None, load_memory_history=False,
                    activation_request=self.request)
        args.update(changes)
        return self.repo.begin_assigned_map_activation(**args)

    def phase(self, phase, expected=("prepared",), **changes):
        args = dict(self.identity, expected_phases=expected)
        args.update(changes)
        return self.repo.update_assigned_map_activation_phase(phase, **args)

    def evidence(self, **changes):
        args = dict(self.identity, activation_receipt=self.owner,
                    relocalization_policy_v2=current_assigned_relocalization_policy(),
                    relocalization_target_available=True, activation_receipt_disposition=2,
                    activation_receipt_disposition_name="sealed", expected_phases=("backend_dispatching",))
        args.update(changes)
        return self.repo.record_assigned_map_activation_evidence(**args)

    def applied(self, **changes):
        self.begin(**changes)
        self.phase("backend_dispatching")
        return self.evidence()

    def complete(self, **changes):
        args = dict(self.identity, activation_receipt=self.owner, expected_phases=("backend_applied",),
                    relocalization_target_available=True, retain_journal=True)
        args.update(changes)
        return self.repo.complete_assigned_map_activation_durable(**args)

    def clear(self, **changes):
        args = dict(self.identity, expected_phases=("backend_applied",))
        args.update(changes)
        return self.repo.clear_assigned_map_activation(**args)

    def test_original_request_is_fsynced_before_dispatch_and_never_generated(self):
        events = []
        real_fsync = module.os.fsync

        def fsync(fd):
            real_fsync(fd)
            events.append("fsync")

        with patch.object(module.os, "fsync", side_effect=fsync), \
                patch.object(uuid, "uuid4", side_effect=AssertionError("regenerated request")), \
                patch.object(hashlib, "sha256", side_effect=AssertionError("payload hash")):
            journal = self.begin()
            self.assertEqual(events, ["fsync", "fsync"])
            self.phase("backend_dispatching")
            events.append("dispatch")
        self.assertEqual(events, ["fsync"] * 4 + ["dispatch"])
        self.assertEqual(journal["activation_request"], self.request)
        self.assertEqual(self.reopen().get_map_activation()["activation_request"], self.request)
        self.assertIsNone(journal["predecessor"])
        self.assertIsNone(journal["backend_evidence"])

    def test_failed_admission_fsync_prevents_caller_dispatch(self):
        for failing_call in (1, 2):
            with self.subTest(fsync=failing_call), tempfile.TemporaryDirectory() as directory:
                repository = module.MapCatalogRepository(directory)
                dispatch = Mock()
                real = module.os.fsync
                count = 0

                def fsync(fd, *, failing_call=failing_call, real=real):
                    nonlocal count
                    count += 1
                    if count == failing_call:
                        raise OSError("injected admission durability failure")
                    return real(fd)

                with patch.object(module.os, "fsync", side_effect=fsync), self.assertRaises(OSError):
                    repository.begin_assigned_map_activation(**self.target, previous_map_name=None,
                        load_memory_history=False, activation_request=self.request)
                    dispatch()
                dispatch.assert_not_called()

    def test_full_request_reopen_changed_fields_and_distinct_native_ids(self):
        original = self.begin()
        before = self.repo._activation_journal_path.read_bytes()
        changes = {
            "activation_request_id": "activate:changed", "session_id": "installed:changed", "map_name": "changed",
            "asset_identity": {"asset_id": "e" * 32}, "checkpoint_id": "f" * 32, "expected_runtime_generation": 4,
        }
        for key, value in changes.items():
            changed = dict(self.request, **{key: value})
            with self.subTest(field=key), self.assertRaises(ValueError):
                self.begin(activation_request=changed)
            self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)
        self.assertEqual(self.reopen().begin_assigned_map_activation(**self.target,
            previous_map_name=None, load_memory_history=False, activation_request=self.request), original)
        original["activation_request"]["session_id"] = "mutated:return"
        self.assertEqual(self.reopen().get_map_activation()["activation_request"], self.request)
        self.phase("backend_dispatching")
        self.evidence()
        completed = self.complete()
        self.assertNotEqual(completed["activation_request"]["session_id"], completed["activation_request_id"])
        self.assertEqual(self.reopen().get_last_successful_map_activation_receipt(), completed)

    def test_known_metadata_only_journal_retires_but_incomplete_current_record_fails(self):
        old = {"schema_version": 3, "previous_map_name": None, "target_map_name": "room",
            "target_asset_identity": ASSET.model_dump(), "target_checkpoint_id": CHECKPOINT,
            "activation_request_id": "mapload:" + "1" * 32, "load_memory_history": False, "phase": "prepared"}
        self.repo._activation_journal_path.write_text(json.dumps(old))
        self.assertIsNone(self.reopen().get_map_activation())
        self.assertFalse(self.repo._activation_journal_path.exists())
        current = self.begin()
        for field in ("activation_request", "predecessor", "backend_evidence"):
            damaged = dict(current)
            damaged.pop(field)
            self.repo._activation_journal_path.write_text(json.dumps(damaged))
            before = self.repo._activation_journal_path.read_bytes()
            with self.assertRaises(ValueError):
                self.reopen().get_map_activation()
            self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)

    def test_whole_request_is_required_for_every_full_journal_mutation(self):
        self.begin()
        for operation in (
            lambda: self.phase("backend_dispatching", activation_request=None),
            lambda: self.clear(expected_phases=("prepared",), activation_request=None),
            lambda: self.complete(expected_phases=("prepared",), activation_request=None),
        ):
            with self.subTest(operation=operation), self.assertRaises(ValueError):
                operation()
        self.assertEqual(self.repo.get_map_activation()["phase"], "prepared")

    def test_predecessor_is_exact_and_stop_phases_have_expected_phase_cas(self):
        previous = predecessor()
        self.begin(previous_map_name="previous", predecessor=previous)
        previous["session_generation"] = 999
        stopped = self.phase("predecessor_stop_dispatching")
        self.assertEqual(stopped["predecessor"], predecessor())
        stopped = self.phase("predecessor_stopped", ("predecessor_stop_dispatching",))
        before = self.repo._activation_journal_path.read_bytes()
        with self.assertRaisesRegex(ValueError, "phase changed"):
            self.phase("recovering", ("prepared",))
        self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)
        self.assertEqual(self.reopen().get_map_activation(), stopped)
        dispatched = self.phase("backend_dispatching", ("predecessor_stopped",))
        self.assertEqual(dispatched["activation_request"], self.request)
        self.assertEqual(dispatched["predecessor"], predecessor())
        before = self.repo._activation_journal_path.read_bytes()
        with self.assertRaisesRegex(ValueError, "cannot regress"):
            self.phase("prepared", ("backend_dispatching",))
        self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)

    def test_predecessor_descriptor_rejects_mutable_revisions_and_bad_generations(self):
        mutations = [dict(predecessor(), revision=1), dict(predecessor(), session_generation=True),
                     dict(predecessor(), runtime_generation=0), dict(predecessor(), mapping_backend_kind="unknown")]
        for candidate in mutations:
            with self.subTest(candidate=candidate), self.assertRaises(ValueError):
                self.begin(predecessor=candidate)
        self.begin()
        with self.assertRaisesRegex(ValueError, "predecessor"):
            self.phase("predecessor_stop_dispatching")
        self.assertEqual(self.repo.get_map_activation()["phase"], "prepared")



    def test_exact_backend_evidence_survives_reopen_and_ack_replay_label(self):
        original = self.applied()
        evidence = original["backend_evidence"]
        self.assertEqual(evidence["activation_receipt_v2"], self.owner.to_dict())
        self.assertEqual(evidence["relocalization_policy_v2"], current_assigned_relocalization_policy())
        self.assertEqual(set(evidence), {"activation_receipt_schema_version", "activation_receipt_v2",
            "activation_receipt_disposition", "activation_receipt_disposition_name",
            "relocalization_target_available", "relocalization_policy_v2"})
        before = self.repo._activation_journal_path.read_bytes()
        with patch.object(module, "_atomic_write_text", side_effect=AssertionError("rewrote exact evidence")):
            replay = self.evidence(expected_phases=("backend_applied",),
                activation_receipt_disposition=3, activation_receipt_disposition_name="sealed_replay")
        self.assertEqual(replay, original)
        self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)
        original["backend_evidence"]["relocalization_policy_v2"]["schema_version"] = 99
        self.assertEqual(self.reopen().get_map_activation(), replay)
        recovered = self.phase("recovering", ("backend_applied",))
        self.assertEqual(recovered["backend_evidence"], replay["backend_evidence"])
        reconciling = self.phase("reconciling", ("recovering",))
        self.assertEqual(self.evidence(expected_phases=("reconciling",),
            activation_receipt_disposition=3, activation_receipt_disposition_name="sealed_replay"), reconciling)
        with self.assertRaises(ValueError):
            self.phase("backend_dispatching", ("reconciling",))

    def test_changed_receipt_policy_availability_or_mixed_fields_fail_without_rewrite(self):
        self.applied()
        before = self.repo._activation_journal_path.read_bytes()
        policy = current_assigned_relocalization_policy()
        mutations = [{"activation_receipt": activation_receipt(epoch=10)},
                     {"relocalization_target_available": False},
                     {"relocalization_policy_v2": dict(policy, source_policy_version=4)},
                     {"relocalization_policy_v2": dict(policy, source_policy_version=True)},
                     {"relocalization_policy_v2": None}]
        for change in mutations:
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.evidence(expected_phases=("backend_applied",), **change)
            self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)
        corrupted = json.loads(before)
        corrupted["backend_evidence"]["cloud_payload"] = "not allowed"
        self.repo._activation_journal_path.write_text(json.dumps(corrupted))
        with self.assertRaisesRegex(ValueError, "fields"):
            self.reopen().get_map_activation()

    def test_failed_evidence_persistence_leaves_original_dispatch_fence(self):
        self.begin()
        dispatched = self.phase("backend_dispatching")
        before = self.repo._activation_journal_path.read_bytes()
        with patch.object(module, "_atomic_write_text", side_effect=OSError("injected evidence failure")), \
                self.assertRaises(OSError):
            self.evidence()
        self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)
        self.assertEqual(self.reopen().get_map_activation(), dispatched)
        self.assertEqual(self.evidence()["phase"], "backend_applied")

    def test_exact_evidence_retry_settles_failed_directory_fsync_before_adoption(self):
        self.begin()
        self.phase("backend_dispatching")
        with patch.object(module, "_fsync_directory", side_effect=OSError("evidence directory fsync failed")), \
                self.assertRaises(OSError):
            self.evidence()
        retained = self.reopen().get_map_activation()
        self.assertEqual(retained["phase"], "backend_applied")
        before = self.repo._activation_journal_path.read_bytes()
        with patch.object(module, "_atomic_write_text", side_effect=AssertionError("rewrote evidence")), \
                patch.object(module, "_fsync_directory", wraps=module._fsync_directory) as fsync:
            self.assertEqual(self.evidence(expected_phases=("backend_applied",)), retained)
        fsync.assert_called_once_with(self.repo.root_dir)
        self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)

    def test_receipt_failure_retains_full_journal_then_explicit_clear_completes(self):
        applied = self.applied()
        with patch.object(module, "_atomic_write_text", side_effect=OSError("injected receipt failure")), \
                self.assertRaises(OSError):
            self.complete()
        self.assertEqual(self.reopen().get_map_activation(), applied)
        self.assertIsNone(self.repo.get_last_successful_map_activation_receipt())
        with self.assertRaisesRegex(ValueError, "durable evidence"):
            self.clear()
        receipt = self.complete()
        self.assertEqual(self.reopen().get_map_activation(), applied)
        self.assertEqual(receipt["activation_request"], self.request)
        self.assertEqual(receipt["relocalization_policy_v2"], current_assigned_relocalization_policy())
        self.clear()
        self.assertIsNone(self.reopen().get_map_activation())
        self.assertEqual(self.reopen().get_last_successful_map_activation_receipt(), receipt)
        self.assertEqual(self.complete(idempotent_replay=True), receipt)

    def test_receipt_directory_fsync_failure_keeps_recoverable_journal_and_receipt(self):
        applied = self.applied()
        with patch.object(module, "_fsync_directory", side_effect=OSError("injected directory fsync failure")), \
                self.assertRaises(OSError):
            self.complete()
        self.assertEqual(self.reopen().get_map_activation(), applied)
        raw = self.repo._activation_receipt_path.read_bytes()
        self.assertIsNotNone(self.reopen().get_last_successful_map_activation_receipt())
        self.complete(idempotent_replay=True)
        self.assertEqual(self.repo._activation_receipt_path.read_bytes(), raw)
        self.clear()
        self.assertIsNone(self.reopen().get_map_activation())

    def test_full_completion_and_clear_cannot_discard_unresolved_or_changed_ownership(self):
        self.begin()
        self.phase("backend_dispatching")
        with self.assertRaisesRegex(ValueError, "durable backend evidence"):
            self.complete(expected_phases=("backend_dispatching",))
        with self.assertRaisesRegex(ValueError, "unresolved"):
            self.clear(expected_phases=("backend_dispatching",))
        self.evidence()
        before = self.repo._activation_journal_path.read_bytes()
        for changes in ({"activation_receipt": activation_receipt(epoch=10)},
                        {"relocalization_target_available": False},
                        {"activation_request": dict(self.request, expected_runtime_generation=4)}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.complete(**changes)
        self.assertEqual(self.repo._activation_journal_path.read_bytes(), before)
        self.assertFalse(self.repo._activation_receipt_path.exists())

    def test_full_receipt_mismatched_request_is_not_trusted_and_old_receipt_has_no_inference(self):
        self.applied()
        self.complete()
        original = json.loads(self.repo._activation_receipt_path.read_text())
        wrong = deepcopy(original)
        wrong["activation_request"]["session_id"] = "other:session"
        self.repo._activation_receipt_path.write_text(json.dumps(wrong))
        with self.assertRaises(ValueError):
            self.reopen().get_last_successful_map_activation_receipt()
        old_shape = {key: value for key, value in original.items()
                     if key not in {"activation_request", "relocalization_policy_v2"}}
        self.repo._activation_receipt_path.write_text(json.dumps(old_shape))
        with self.assertRaises(ValueError):
            self.reopen().get_last_successful_map_activation_receipt()

    def test_native_source_session_does_not_gain_new_journal_length_or_alphabet_cap(self):
        self.begin()
        self.phase("backend_dispatching")
        owner = activation_receipt(source_session="源地图/" + "x" * 70_000)
        journal = self.evidence(activation_receipt=owner)
        self.assertEqual(journal["backend_evidence"]["activation_receipt_v2"], owner.to_dict())
        self.assertEqual(self.reopen().get_map_activation(), journal)


if __name__ == "__main__":
    unittest.main()
