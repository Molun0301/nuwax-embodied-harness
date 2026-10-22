"""Isolated assigned-session contracts, guarded metadata CAS and FIFO release.

No runtime constructor, native process, network/RPC, robot or map-body read.
These tests are source-only until the parent grants the timing clearance.
"""
from collections import deque
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import unittest
from unittest.mock import patch
from pydantic import BaseModel, model_serializer

from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.map_activation import AssignedActivationReceipt
from contracts.map_identity import AssetIdentityV2
from contracts.map_relocalization import AssignedRelocalizationRequest, AssignedRelocalizationTerminal
from contracts.map_workspace import AssetIdentityV2 as WorkspaceAssetIdentity
from contracts.runtime_views import LocalizationSession, LocalizationSessionStatus, LocalizationSnapshot, RelocalizationCandidate
from core.errors import GatewayError
from services.localization import service as localization_module
from services.localization.service import LocalizationService
from services.localization.state_machine import LocalizationStateMachine

_fixture_path = Path(__file__).resolve().with_name('test_assigned_relocalization_contract.py')
_fixture_spec = importlib.util.spec_from_file_location('_assigned_session_fixtures', _fixture_path)
_fixtures = importlib.util.module_from_spec(_fixture_spec)
_fixture_spec.loader.exec_module(_fixtures)
STAMP = datetime(2026, 1, 1, tzinfo=timezone.utc)


def activation(asset='a'):
    request = _fixtures.request_dict()
    request['assigned_cloud_v2']['target']['activation']['request']['asset_identity']['asset_id'] = asset * 32
    request['assigned_cloud_v2']['target']['activation']['request']['activation_request_id'] = 'mapload:' + asset
    return AssignedRelocalizationRequest.admit(request).activation_receipt


def applied_terminal():
    return AssignedRelocalizationTerminal.admit(_fixtures.terminal_dict(5))


def assigned_session(owner=None, terminal=None, **updates):
    owner = owner or (terminal.request.activation_receipt if terminal else activation())
    values = dict(timestamp=STAMP, identity_schema_version=2, session_id='local', map_name=owner.map_name,
                  asset_identity=owner.asset_identity, activation_receipt_v2=owner,
                  relocalization_terminal_v2=terminal, frame_id='map')
    values.update(updates)
    return LocalizationSession(**values)


class Cancelled(BaseException):
    pass


class Store:
    def __init__(self, service, guards):
        self.service, self.guards = service, guards
        self.writes = []
        self.on_write = None

    def write(self, namespace, key, value, *, source, metadata):
        assert not self.service._lock._is_owned(), 'callback under localization lock'
        assert not self.service._session_publication_lock.locked(), 'callback under publication lock'
        assert not any(lock._is_owned() for lock in self.guards), 'callback under an originating outer guard'
        self.writes.append((key, value, deepcopy(metadata)))
        if self.on_write:
            self.on_write(key, value)

    @property
    def sessions(self):
        return [value for key, value, _ in self.writes if key == 'active_session']


class Provider:
    def __init__(self, service, guards):
        self.service, self.guards = service, guards
        self.calls = []
        self.backend = {'tracking_state': 'tracking', 'map_loaded': True, 'quality_score': .8,
                        'nested': {'value': 1}}
        self.pose = Pose(frame_id='map', timestamp=STAMP)
        self.tree = FrameTree(root_frame_id='map', timestamp=STAMP)
        self.block = None

    def check(self, name):
        assert not self.service._lock._is_owned()
        assert not any(lock._is_owned() for lock in self.guards)
        self.calls.append(name)

    @property
    def provider_name(self):
        self.check('name')
        return 'fake_mapping_provider'

    @property
    def backend_type(self):
        self.check('backend_type')
        return 'mapping_backend'

    def is_available(self):
        self.check('available')
        return True

    def get_localization_state(self):
        self.check('diagnostics')
        return self.backend

    def get_frame_tree(self):
        self.check('tree')
        if self.block:
            self.block()
        return self.tree

    def get_current_pose(self):
        self.check('pose')
        return self.pose


class AssignedSessionContractTests(unittest.TestCase):
    def test_import_identity_and_legacy_serialization(self):
        self.assertIs(WorkspaceAssetIdentity, AssetIdentityV2)
        session = LocalizationSession(timestamp=STAMP, session_id='legacy', map_name='room')
        doc = session.model_dump(mode='json')
        for key in ('identity_schema_version', 'asset_identity', 'activation_receipt_v2', 'relocalization_terminal_v2'):
            self.assertNotIn(key, doc)
        self.assertEqual(LocalizationSession.model_validate(doc).model_dump(mode='json'), doc)
        session.metadata['legacy_mutable'] = True
        self.assertTrue(session.metadata['legacy_mutable'])
        self.assertEqual(session.model_copy(update={'source_name': 'legacy-copy'}).source_name, 'legacy-copy')

    def test_import_orders_in_fresh_interpreters(self):
        modules = ['runtime_views', 'map_workspace', 'map_activation', 'map_relocalization', 'map_identity']
        root = str(Path(__file__).resolve().parents[1])
        for first in modules:
            order = [first] + [name for name in modules if name != first]
            code = '; '.join('import contracts.' + name for name in order)
            code += '; assert contracts.map_workspace.AssetIdentityV2 is contracts.map_identity.AssetIdentityV2'
            result = subprocess.run([sys.executable, '-c', code], cwd=root, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_typed_roundtrip_preserves_owners_and_raw_bits(self):
        terminal = applied_terminal()
        session = assigned_session(terminal=terminal, status=LocalizationSessionStatus.READY, pose_available=True)
        doc = session.model_dump(mode='json')
        self.assertEqual(doc['activation_receipt_v2'], terminal.request.activation_receipt.to_dict())
        self.assertEqual(doc['relocalization_terminal_v2'], terminal.to_dict())
        self.assertNotIn('_encoded', json.dumps(doc))
        reopened = LocalizationSession.model_validate(doc)
        self.assertTrue(reopened.relocalization_terminal_v2.exact_replay_of(terminal))
        for copied in (session.model_copy(), session.model_copy(deep=True), deepcopy(session)):
            self.assertIs(copied.activation_receipt_v2, session.activation_receipt_v2)
            self.assertIs(copied.relocalization_terminal_v2, terminal)

    def test_alias_isolation_and_validated_copy(self):
        metadata = {'nested': {'items': [1]}}
        geometry = {'quality_metrics': {'status': 'ready'}}
        candidates = [{'backend': 'fake', 'metadata': {'nested': [1]}}]
        session = assigned_session(metadata=metadata, geometry_asset=geometry, relocalization_candidates=candidates)
        candidates[0]['metadata']['nested'].append(2)
        session.relocalization_candidates[0].metadata['nested'].append(3)
        self.assertEqual(session.relocalization_candidates[0].metadata['nested'], [1])
        metadata['nested']['items'].append(2)
        geometry['quality_metrics']['status'] = 'failed'
        session.metadata['nested']['items'].append(3)
        session.geometry_asset['quality_metrics']['status'] = 'failed'
        self.assertEqual(session.metadata['nested']['items'], [1])
        self.assertEqual(session.geometry_asset['quality_metrics']['status'], 'ready')
        for key, value in [('map_name', 'other'), ('identity_schema_version', 1), ('asset_identity', AssetIdentityV2(asset_id='b' * 32)), ('activation_receipt_v2', activation('b'))]:
            with self.subTest(key=key), self.assertRaises((TypeError, ValueError)):
                session.model_copy(update={key: value}, deep=True)
        with self.assertRaises(TypeError):
            session.status = LocalizationSessionStatus.READY
        with self.assertRaises(ValueError):
            session.model_copy(update={'metadata': {'target_receipt_sha256': 'a' * 64}})
        updated = session.model_copy(update={'metadata': {'another': [1]}})
        updated.metadata['another'].append(2)
        self.assertEqual(updated.metadata, {'another': [1]})
        self.assertEqual(session.metadata, {'nested': {'items': [1]}})

    def test_rejects_mixed_missing_unapplied_and_forged_owner_fields(self):
        base = assigned_session().model_dump(mode='python')
        cases = [dict(identity_schema_version=True), dict(identity_schema_version=3), dict(asset_identity=None),
                 dict(activation_receipt_v2=None), dict(asset_manifest_sha256='a' * 64), dict(map_name='other'),
                 dict(asset_identity={'asset_id': 'b' * 32}), dict(status='ready', pose_available=False),
                 dict(metadata={'mapping_session_id': 'legacy'}), dict(geometry_asset={'asset_manifest_sha256': None}),
                 dict(geometry_asset={'map_name': 'other'}), dict(metadata={'asset_identity': {'asset_id': 'b' * 32}})]
        for change in cases:
            with self.subTest(change=change), self.assertRaises(ValueError):
                LocalizationSession.model_validate({**base, **change})
        rejected = AssignedRelocalizationTerminal.admit(_fixtures.terminal_dict())
        with self.assertRaises(ValueError):
            assigned_session(terminal=rejected)
        with self.assertRaises(ValueError):
            assigned_session(owner=activation('b'), terminal=applied_terminal())
        forged = AssetIdentityV2(asset_id='a' * 32).model_copy(update={'asset_id': 'bad'})
        with self.assertRaises(ValueError):
            assigned_session(asset_identity=forged)
        legacy = LocalizationSession(session_id='legacy', map_name='room')
        with self.assertRaises(TypeError):
            legacy.model_copy(update={'activation_receipt_v2': activation()})

    def test_legacy_owner_assignment_and_bool_copy_fail_before_mutation(self):
        legacy = LocalizationSession(timestamp=STAMP, session_id='legacy', map_name='room', metadata={'kept': [1]})
        before = legacy.model_dump(mode='python')
        for name, value in (('identity_schema_version', 2), ('identity_schema_version', True),
                            ('asset_identity', activation().asset_identity),
                            ('activation_receipt_v2', activation()), ('relocalization_terminal_v2', applied_terminal())):
            with self.subTest(name=name, value_type=type(value)), self.assertRaises(TypeError):
                setattr(legacy, name, value)
            self.assertEqual(legacy.model_dump(mode='python'), before)
            self.assertIs(type(legacy.identity_schema_version), int)
            self.assertEqual(legacy.identity_schema_version, 1)
        for deep in (False, True):
            with self.assertRaises(TypeError):
                legacy.model_copy(update={'identity_schema_version': True}, deep=deep)
            self.assertEqual(legacy.model_dump(mode='python'), before)
        legacy.source_name = 'ordinary-legacy-assignment'
        self.assertEqual(legacy.source_name, 'ordinary-legacy-assignment')

    def test_descriptive_security_and_protocol_digests_are_not_owner_authority(self):
        metadata = {'certificate_sha256': 'c' * 64, 'firmware_sha256': 'f' * 64,
                    'protocol': {'fingerprint_sha256': 'd' * 64, 'schema_fingerprint': 'caf86342b91bf47d'}}
        session = assigned_session(metadata=metadata)
        self.assertEqual(session.metadata, metadata)
        for key in ('asset_manifest_sha256', 'activation_request_identity_sha256', 'target_receipt_sha256',
                    'committed_occupancy_sha256', 'source_policy_sha256', 'terminal_proof_sha256'):
            with self.subTest(key=key), self.assertRaises(ValueError):
                assigned_session(metadata={'nested': {key: 'a' * 64}})

    def test_typed_candidate_body_is_rejected_before_any_serializer_or_copy(self):
        calls = []
        class Body(BaseModel):
            @model_serializer
            def serialize_body(self):
                calls.append('serialize')
                raise AssertionError('candidate body serialized')
            def __deepcopy__(self, memo):
                calls.append('deepcopy')
                raise AssertionError('candidate body copied')
        body = Body()
        candidates = [
            RelocalizationCandidate(metadata={'nested': {'body': body}}),
            RelocalizationCandidate(covariance={'body': body}),
            RelocalizationCandidate(asset_refs=[{'body': body}]),
            RelocalizationCandidate().model_copy(update={'pose': body}),
            RelocalizationCandidate().model_copy(update={'fitness': body}),
            RelocalizationCandidate(pose=Pose(frame_id='map', timestamp=STAMP).model_copy(update={'position': body})),
        ]
        with patch.object(RelocalizationCandidate, 'model_dump', side_effect=AssertionError('generic candidate serializer invoked')):
            for candidate in candidates:
                for field in ('relocalization_candidates', 'last_relocalization'):
                    value = [candidate] if field == 'relocalization_candidates' else candidate
                    with self.subTest(field=field), self.assertRaises(ValueError):
                        assigned_session(**{field: value})
        self.assertEqual(calls, [])
        # Normal typed pose/candidate admission detaches nested scalar views and
        # also avoids the generic candidate serializer entirely.
        candidate = RelocalizationCandidate(pose=Pose(frame_id='map', timestamp=STAMP), metadata={'nested': [1]})
        with patch.object(RelocalizationCandidate, 'model_dump', side_effect=AssertionError('generic candidate serializer invoked')):
            admitted = assigned_session(relocalization_candidates=[candidate])
        candidate.pose.position.x = 9
        candidate.metadata['nested'].append(2)
        self.assertEqual(admitted.relocalization_candidates[0].pose.position.x, 0)
        self.assertEqual(admitted.relocalization_candidates[0].metadata['nested'], [1])

    def test_payload_objects_are_rejected_without_copy_or_serialization(self):
        class Body:
            def __deepcopy__(self, memo):
                raise AssertionError('body copied')
            def model_dump(self, **kwargs):
                raise AssertionError('body serialized')
        for value in ({'object': Body()}, {'occupancy_grid': {'data': []}}, {'cloud': []}, {'candidate_payload_v3': ''}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                assigned_session(metadata=value)


class AssignedSessionServiceTests(unittest.TestCase):
    def setUp(self):
        self.service = service = LocalizationService.__new__(LocalizationService)
        service._lock = threading.RLock()
        service._session_publication_lock = threading.Lock()
        service._session_publication_queue = deque()
        service._session_publication_draining = False
        service._session_publication_drainer_token = None
        service._session_event_read_view = threading.local()
        service._history = deque(maxlen=2)
        service._session_history = deque(maxlen=2)
        service._latest_snapshot = None
        service._active_session = None
        service._session_mutation_epoch = 0
        service._state_machine = LocalizationStateMachine()
        service._last_diagnostics = None
        service._event_bus = service._stream_bus = None
        self.guards = [threading.RLock() for _ in range(3)]  # E, C, R
        self.store = service._state_store = Store(service, self.guards)
        self.provider = Provider(service, self.guards)
        service._find_localization_provider = lambda: self.provider
        self.installed_target = object()

    @contextmanager
    def guarded(self, expected_target):
        with self.guards[0], self.guards[1], self.guards[2]:
            if expected_target is not self.installed_target:
                raise ValueError('exact runtime installation changed')
            yield

    def prepare(self, owner=None, terminal=None, **kwargs):
        expected, epoch = self.service.get_active_session_token()
        owner = owner or (terminal.request.activation_receipt if terminal else activation())
        return self.service.prepare_assigned_session_update(
            expected_session=expected, expected_mutation_epoch=epoch,
            activation_receipt_v2=owner, relocalization_terminal_v2=terminal, frame_id='map', **kwargs)

    def commit(self, prepared, scope, target=None, **overrides):
        values = dict(guarded_activation_receipt_v2=prepared.session.activation_receipt_v2,
                      guarded_relocalization_terminal_v2=prepared.session.relocalization_terminal_v2,
                      publication_scope=scope)
        values.update(overrides)
        with self.guarded(self.installed_target if target is None else target):
            return self.service._compare_and_enqueue_prepared_assigned_session(prepared, **values)

    def install(self, owner=None, terminal=None, **kwargs):
        prepared = self.prepare(owner, terminal, **kwargs)
        with self.service.assigned_session_publication_scope() as scope:
            result = self.commit(prepared, scope)
        return result

    def state(self):
        service = self.service
        return (service.get_active_session_token(), tuple(service._session_history), tuple(service._history),
                dict(service._state_machine.__dict__), service._latest_snapshot, tuple(service._session_publication_queue))

    def test_no_refresh_activation_works_without_provider_and_does_not_make_ready(self):
        with patch.object(self.service, '_get_localization_provider', side_effect=AssertionError('provider touched')):
            result = self.install()
        self.assertTrue(result.accepted)
        self.assertEqual(result.committed_epoch, 1)
        self.assertEqual(result.session.status, LocalizationSessionStatus.LOCALIZING)
        self.assertFalse(result.session.pose_available)
        self.assertIsNone(result.session.relocalization_terminal_v2)
        self.assertEqual(self.provider.calls, [])
        self.assertEqual(self.store.sessions, [result.session])

    def test_provider_properties_and_metadata_are_captured_before_guards(self):
        metadata = {'nested': {'value': 1}}
        geometry = {'quality_metrics': {'status': 'ready'}}
        def block():
            metadata['nested']['value'] = 99
            geometry['quality_metrics']['status'] = 'failed'
            self.provider.backend['nested']['value'] = 99
        self.provider.block = block
        prepared = self.prepare(terminal=applied_terminal(), metadata=metadata, geometry_asset=geometry, refresh=True)
        self.provider.pose = self.provider.pose.model_copy(update={"position": self.provider.pose.position.model_copy(update={"x": 99})})
        with self.service.assigned_session_publication_scope() as scope:
            with patch.object(self.service, '_get_localization_provider', side_effect=AssertionError('provider inside commit')):
                result = self.commit(prepared, scope)
        self.assertTrue(result.session.pose_available)
        self.assertEqual(result.session.metadata['nested']['value'], 1)
        self.assertEqual(result.session.metadata['diagnostics']['metadata']['backend_state']['nested']['value'], 1)
        self.assertEqual(self.service.get_latest_snapshot().current_pose.position.x, 0)
        self.assertTrue({'available', 'name', 'backend_type', 'diagnostics', 'tree', 'pose'} <= set(self.provider.calls))
        shared = self.service.get_latest_snapshot()
        self.assertIs(self.service.get_latest_snapshot(), shared)
        snapshot = shared.model_copy(deep=True)
        snapshot.metadata['asset_identity']['asset_id'] = 'b' * 32
        snapshot.current_pose.position.x = 8
        self.assertEqual(self.service.get_latest_snapshot().current_pose.position.x, 0)
        self.assertEqual(self.service.get_latest_snapshot().metadata['asset_identity']['asset_id'], 'a' * 32)

    def test_provider_nested_body_is_rejected_before_serialization_or_copy(self):
        calls = []
        class Body(BaseModel):
            @model_serializer
            def serialize_body(self):
                calls.append('serialize')
                raise AssertionError('provider body serialized')
            def __deepcopy__(self, memo):
                calls.append('deepcopy')
                raise AssertionError('provider body copied')
        body = Body()
        pose = Pose(frame_id='map', timestamp=STAMP)
        tree = FrameTree(root_frame_id='map', timestamp=STAMP)
        transform = Transform(parent_frame_id='map', child_frame_id='body', timestamp=STAMP)
        inputs = [
            (pose.model_copy(update={'position': body}), tree),
            (pose.model_copy(update={'orientation': body}), tree),
            (pose.model_copy(update={'position': Vector3.model_construct(x=body)}), tree),
            (pose.model_copy(update={'timestamp': body}), tree),
            (pose.model_copy(update={'metadata': {'body': body}}), tree),
            (pose, tree.model_copy(update={'transforms': [body]})),
            (pose, tree.model_copy(update={'transforms': body})),
            (pose, tree.model_copy(update={'root_frame_id': body})),
            (pose, tree.model_copy(update={'transforms': [transform.model_copy(update={'translation': body})]})),
            (pose, tree.model_copy(update={'transforms': [transform.model_copy(update={'rotation': Quaternion.model_construct(w=body)})]})),
            (pose, tree.model_copy(update={'transforms': [transform.model_copy(update={'authority': body})]})),
        ]
        before = self.state()
        # Even serialization of a valid companion object must wait until both
        # captures have passed field screening and normal contract validation.
        with patch.object(Pose, 'model_dump_json', side_effect=AssertionError('pose serialized before screening')), \
             patch.object(FrameTree, 'model_dump_json', side_effect=AssertionError('tree serialized before screening')):
            for current_pose, frame_tree in inputs:
                self.provider.pose, self.provider.tree = current_pose, frame_tree
                with self.subTest(pose_keys=current_pose.__dict__.keys(), tree_keys=frame_tree.__dict__.keys()), self.assertRaises(ValueError):
                    self.prepare(refresh=True)
                self.assertEqual(self.state(), before)
        self.assertEqual(calls, [])

    def test_provider_geometry_projection_preserves_contract_validation_and_alias_isolation(self):
        transform = Transform(parent_frame_id='map', child_frame_id='body', timestamp=STAMP)
        self.provider.tree = FrameTree(root_frame_id='map', transforms=[transform], timestamp=STAMP)
        prepared = self.prepare(refresh=True)
        self.provider.pose = self.provider.pose.model_copy(update={"position": self.provider.pose.position.model_copy(update={"x": 99})})
        self.provider.tree.transforms[0].translation.x = 99
        with self.service.assigned_session_publication_scope() as scope:
            result = self.commit(prepared, scope)
        self.assertTrue(result.session.pose_available)
        snapshot = self.service.get_latest_snapshot()
        self.assertEqual(snapshot.current_pose.position.x, 0)
        self.assertEqual(snapshot.frame_tree.transforms[0].translation.x, 0)
        self.provider.tree = FrameTree(root_frame_id='map', timestamp=STAMP).model_copy(
            update={'transforms': [Transform.model_construct(parent_frame_id='map', child_frame_id='')]})
        before = self.state()
        with self.assertRaises(ValueError):
            self.prepare(refresh=True)
        self.assertEqual(self.state(), before)

    def test_valid_captured_pose_can_be_ready_without_relocalization_terminal(self):
        result = self.install(refresh=True)
        self.assertTrue(result.accepted)
        self.assertEqual(result.session.status, LocalizationSessionStatus.READY)
        self.assertTrue(result.session.pose_available)
        self.assertIsNone(result.session.relocalization_terminal_v2)
        self.assertIsNotNone(self.service.get_latest_snapshot().current_pose)
        self.assertEqual(self.service._state_machine.current_state, 'ready')
        contract = assigned_session(status=LocalizationSessionStatus.READY, pose_available=True)
        self.assertIsNone(contract.relocalization_terminal_v2)

    def test_terminal_without_pose_and_wrong_frame_never_becomes_ready(self):
        self.provider.pose = None
        self.assertFalse(self.install(terminal=applied_terminal(), refresh=True).session.pose_available)
        self.provider.pose = Pose(frame_id='odom', timestamp=STAMP)
        self.assertFalse(self.install(terminal=applied_terminal(), refresh=True).session.pose_available)

    def test_same_metadata_new_runtime_installation_is_rejected_before_commit(self):
        target = self.installed_target
        self.provider.block = lambda: setattr(self, 'installed_target', object())
        prepared = self.prepare(terminal=applied_terminal(), refresh=True)
        before = self.state()
        with self.service.assigned_session_publication_scope() as scope:
            with self.assertRaisesRegex(ValueError, 'runtime installation'):
                self.commit(prepared, scope, target=target)
        self.assertEqual(self.state(), before)
        self.assertEqual(self.store.sessions, [])

    def test_stale_session_reference_epoch_and_owner_do_not_mutate(self):
        a = self.install().session
        stale = self.prepare()
        b = self.install(activation('b')).session
        before = self.state()
        with self.service.assigned_session_publication_scope() as scope:
            result = self.commit(stale, scope)
        self.assertFalse(result.accepted)
        self.assertIs(result.session, b)
        self.assertEqual(self.state(), before)
        self.assertIsNot(a, b)
        # Reference mismatch remains authoritative even for equal local labels.
        same_id = self.prepare()
        replacement = b.model_copy()
        with self.service._lock:
            self.service._active_session = replacement
        before = self.state()
        with self.service.assigned_session_publication_scope() as scope:
            result = self.commit(same_id, scope)
        self.assertFalse(result.accepted)
        self.assertEqual(self.state(), before)
        # ABA return to the same owner still has a newer mutation epoch.
        stale = self.prepare()
        self.service.clear_active_session()
        self.install(activation('b'))
        before = self.state()
        with self.service.assigned_session_publication_scope() as scope:
            self.assertFalse(self.commit(stale, scope).accepted)
        self.assertEqual(self.state(), before)

    def test_newer_session_during_blocked_provider_capture_is_preserved(self):
        entered, resume = threading.Event(), threading.Event()
        prepared_values, errors = [], []
        def block():
            entered.set()
            if not resume.wait(5):
                raise AssertionError('provider capture not resumed')
        self.provider.block = block
        def prepare_worker():
            try:
                prepared_values.append(self.prepare(terminal=applied_terminal(), refresh=True))
            except BaseException as error:
                errors.append(error)
        thread = threading.Thread(target=prepare_worker)
        thread.start()
        self.assertTrue(entered.wait(5))
        try:
            newer = self.install(activation('b')).session
            resume.set()
            thread.join(5)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            before = self.state()
            with self.service.assigned_session_publication_scope() as scope:
                result = self.commit(prepared_values[0], scope)
            self.assertFalse(result.accepted)
            self.assertIs(result.session, newer)
            self.assertEqual(self.state(), before)
        finally:
            resume.set()
            thread.join(5)

    def test_wrong_guarded_owner_and_terminal_fail_before_any_mutation(self):
        prepared = self.prepare(terminal=applied_terminal(), refresh=True)
        before = self.state()
        with self.service.assigned_session_publication_scope() as scope:
            result = self.commit(prepared, scope, guarded_activation_receipt_v2=activation('b'))
            self.assertFalse(result.accepted)
            result = self.commit(prepared, scope, guarded_relocalization_terminal_v2=None)
            self.assertFalse(result.accepted)
        self.assertEqual(self.state(), before)

    def test_active_drainer_cannot_publish_deferred_record_under_outer_guards(self):
        entered, resume, done = threading.Event(), threading.Event(), threading.Event()
        errors = []
        def callback(key, session):
            if key == 'active_session' and session is not None and session.identity_schema_version == 1:
                entered.set()
                if not resume.wait(5):
                    raise AssertionError('test drainer was not resumed')
        self.store.on_write = callback
        def start_legacy():
            try:
                self.service.ensure_active_session(map_name='legacy', frame_id='map')
            except BaseException as error:
                errors.append(error)
            finally:
                done.set()
        thread = threading.Thread(target=start_legacy)
        thread.start()
        self.assertTrue(entered.wait(5))
        try:
            prepared = self.prepare()
            with self.service.assigned_session_publication_scope() as scope:
                with self.guarded(self.installed_target):
                    result = self.service._compare_and_enqueue_prepared_assigned_session(
                        prepared, guarded_activation_receipt_v2=prepared.session.activation_receipt_v2,
                        guarded_relocalization_terminal_v2=None, publication_scope=scope)
                    resume.set()
                    self.assertTrue(done.wait(5))
                    self.assertEqual(len(self.store.sessions), 1)
                    self.assertFalse(self.service._session_publication_draining)
                    self.assertFalse(result.publication_record.ready)
                self.assertEqual(len(self.store.sessions), 1)
            self.assertEqual(self.store.sessions[-1], result.session)
            self.assertFalse(self.service._session_publication_queue)
        finally:
            resume.set()
            thread.join(5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])

    def test_out_of_order_release_keeps_fifo_old_event_identity_and_actual_token(self):
        first_scope = self.service.assigned_session_publication_scope()
        scope_a = first_scope.__enter__()
        a = self.commit(self.prepare(activation('a')), scope_a)
        second_scope = self.service.assigned_session_publication_scope()
        scope_b = second_scope.__enter__()
        b = self.commit(self.prepare(activation('b')), scope_b)
        seen = []
        def callback(key, session):
            if key == 'active_session':
                seen.append((self.service.get_active_session(), self.service.get_active_session_token()))
                if session is a.session:
                    stale = self.service.prepare_assigned_session_update(
                        expected_session=session, expected_mutation_epoch=a.committed_epoch,
                        activation_receipt_v2=session.activation_receipt_v2, frame_id='map')
                    with self.service.assigned_session_publication_scope() as scope:
                        self.assertFalse(self.commit(stale, scope).accepted)
        self.store.on_write = callback
        self.service.release_assigned_session_publication(b)
        self.assertEqual(self.store.sessions, [])
        first_scope.__exit__(None, None, None)
        second_scope.__exit__(None, None, None)
        self.assertEqual(self.store.sessions, [a.session, b.session])
        self.assertIs(seen[0][0], a.session)
        self.assertIs(seen[0][1][0], b.session)
        metadata = [item[2] for item in self.store.writes if item[0] == 'active_session']
        self.assertEqual([item['session_mutation_epoch'] for item in metadata], [1, 2])
        self.assertEqual([item['asset_identity']['asset_id'] for item in metadata], ['a' * 32, 'b' * 32])
        self.service.release_assigned_session_publication(a)
        self.assertEqual(len(self.store.sessions), 2)

    def test_reentrant_callback_newer_session_is_never_restored(self):
        replacement = []
        def callback(key, session):
            if key == 'active_session' and not replacement:
                replacement.append(True)
                # Second thread must acquire each originating guard and S/P.
                done = threading.Event()
                def probe():
                    with self.guards[0], self.guards[1], self.guards[2], self.service._lock, self.service._session_publication_lock:
                        done.set()
                thread = threading.Thread(target=probe)
                thread.start()
                self.assertTrue(done.wait(5))
                thread.join(5)
                replacement.append(self.install(activation('b')).session)
        self.store.on_write = callback
        original = self.install(activation('a'))
        self.assertIs(self.service.get_active_session_token()[0], replacement[-1])
        self.assertEqual(self.store.sessions, [original.session, replacement[-1]])

    def test_exception_and_cancellation_after_commit_release_after_guards_unwind(self):
        for error_type in (RuntimeError, Cancelled):
            with self.subTest(error_type=error_type):
                before = len(self.store.sessions)
                with self.assertRaises(error_type):
                    with self.service.assigned_session_publication_scope():
                        with self.guarded(self.installed_target):
                            raise error_type('before commit')
                self.assertEqual(len(self.store.sessions), before)
                with self.assertRaises(error_type):
                    with self.service.assigned_session_publication_scope() as scope:
                        prepared = self.prepare()
                        with self.guarded(self.installed_target):
                            result = self.service._compare_and_enqueue_prepared_assigned_session(
                                prepared, guarded_activation_receipt_v2=prepared.session.activation_receipt_v2,
                                guarded_relocalization_terminal_v2=None, publication_scope=scope)
                            self.assertFalse(result.publication_record.ready)
                            raise error_type('after commit')
                self.assertEqual(len(self.store.sessions), before + 1)
                self.assertFalse(self.service._session_publication_queue)

    def test_commit_failure_rolls_back_evicted_history_in_constant_space(self):
        self.install(terminal=applied_terminal(), refresh=True)
        self.install(terminal=applied_terminal(), refresh=True)
        prepared = self.prepare(terminal=applied_terminal(), refresh=True)
        before = self.state()
        original = self.service._record_snapshot_locked
        def append_then_fail(snapshot):
            original(snapshot)
            raise Cancelled('inside commit')
        with patch.object(self.service, '_record_snapshot_locked', side_effect=append_then_fail):
            with self.assertRaises(Cancelled):
                with self.service.assigned_session_publication_scope() as scope:
                    self.commit(prepared, scope)
        self.assertEqual(self.state(), before)
        self.assertFalse(self.service._session_publication_queue)

    def test_failure_after_each_shared_mutation_rolls_back_and_unblocks_fifo(self):
        # These injections execute no external provider; each raises only once,
        # after the named write, so rollback itself remains executable.
        phases = ('queue_append', 'active_session', 'epoch', 'session_history',
                  'machine_state', 'last_diagnostics', 'latest_snapshot',
                  'snapshot_history', 'publication_payload', 'commit_mark')
        for phase in phases:
            with self.subTest(phase=phase):
                self.setUp()
                self.install(terminal=applied_terminal(), refresh=True)
                self.install(terminal=applied_terminal(), refresh=True)
                prepared = self.prepare(terminal=applied_terminal(), refresh=True)
                before = self.state()
                service = self.service
                armed = [True]

                def fail_once():
                    if armed[0]:
                        armed[0] = False
                        raise Cancelled('after ' + phase)

                class FailingService(LocalizationService):
                    def __setattr__(self, name, value):
                        super().__setattr__(name, value)
                        targets = {'active_session': '_active_session', 'epoch': '_session_mutation_epoch',
                                   'last_diagnostics': '_last_diagnostics', 'latest_snapshot': '_latest_snapshot'}
                        if name == targets.get(phase):
                            fail_once()

                class FailingDeque(deque):
                    def append(self, value):
                        super().append(value)
                        fail_once()

                class FailingState(dict):
                    def update(self, *args, **kwargs):
                        super().update(*args, **kwargs)
                        fail_once()

                class FailingRecord(localization_module._SessionPublicationRecord):
                    def __setattr__(self, name, value):
                        super().__setattr__(name, value)
                        if (phase == 'publication_payload' and name == 'snapshot_publication' and value is not None
                                or phase == 'commit_mark' and name == 'cancelled' and value is False):
                            fail_once()

                if phase in ('active_session', 'epoch', 'last_diagnostics', 'latest_snapshot'):
                    service.__class__ = FailingService
                if phase in ('queue_append', 'session_history', 'snapshot_history'):
                    name = {'queue_append': '_session_publication_queue', 'session_history': '_session_history',
                            'snapshot_history': '_history'}[phase]
                    old = getattr(service, name)
                    setattr(service, name, FailingDeque(old, maxlen=old.maxlen))
                if phase == 'machine_state':
                    service._state_machine.__dict__ = FailingState(service._state_machine.__dict__)
                record_type = FailingRecord if phase in ('publication_payload', 'commit_mark') else localization_module._SessionPublicationRecord
                with patch.object(localization_module, '_SessionPublicationRecord', record_type):
                    with self.assertRaises(Cancelled):
                        with service.assigned_session_publication_scope() as scope:
                            self.commit(prepared, scope)
                self.assertFalse(armed[0], 'injection was not reached')
                self.assertEqual(self.state(), before)
                self.assertFalse(service._session_publication_draining)
                # Failure must not leave a hidden unready head blocking the next
                # independently captured operation.
                self.assertTrue(self.install(terminal=applied_terminal(), refresh=True).accepted)

    def test_callback_exception_or_cancellation_does_not_strand_ready_events(self):
        for error_type in (RuntimeError, Cancelled):
            with self.subTest(error_type=error_type):
                self.setUp()
                context = self.service.assigned_session_publication_scope()
                scope = context.__enter__()
                a = self.commit(self.prepare(activation('a')), scope)
                b = self.commit(self.prepare(activation('b')), scope)
                def callback(key, session):
                    if session is a.session:
                        raise error_type('callback failure')
                self.store.on_write = callback
                if error_type is Cancelled:
                    with self.assertRaises(Cancelled):
                        context.__exit__(None, None, None)
                else:
                    context.__exit__(None, None, None)
                self.assertEqual(self.store.sessions, [a.session, b.session])
                self.assertFalse(self.service._session_publication_queue)
                self.assertFalse(self.service._session_publication_draining)

    def test_prepare_failure_has_no_side_effect_and_lazy_metadata_is_rejected(self):
        before = self.state()
        self.provider.block = lambda: (_ for _ in ()).throw(Cancelled('prepare cancelled'))
        with self.assertRaises(Cancelled):
            self.prepare(terminal=applied_terminal(), refresh=True)
        self.assertEqual(self.state(), before)
        self.provider.block = None
        self.provider.backend['lazy'] = object()
        with self.assertRaises(ValueError):
            self.prepare(terminal=applied_terminal(), refresh=True)
        self.assertEqual(self.state(), before)

    def test_legacy_mutators_cannot_strip_assigned_owner(self):
        session = self.install().session
        before = self.state()
        for name in ('ensure_active_session', 'refresh_active_session', 'relocalize_active_session'):
            with self.subTest(name=name), self.assertRaises(GatewayError):
                getattr(self.service, name)(map_name='room', frame_id='map')
        self.assertIs(self.service.refresh_current_session(), session)
        token = self.service.get_active_session_token()
        self.assertFalse(self.service.compare_and_set_active_session(
            expected_session_id=session.session_id, expected_live_mapping_identity=None,
            expected_mutation_epoch=token[1], map_name='room', refresh=False)[0])
        with self.assertRaises(GatewayError):
            self.service._set_active_session(LocalizationSession(session_id='legacy', map_name='room'))
        with self.assertRaises(GatewayError):
            self.service._set_active_session(session)
        self.assertEqual(self.state(), before)
        self.assertFalse(self.service.compare_and_clear_active_session(
            expected_session_id=session.session_id, expected_live_mapping_identity=None,
            expected_mutation_epoch=token[1])[0])
        self.assertTrue(self.service.compare_and_clear_active_session(
            expected_session_id=session.session_id, expected_live_mapping_identity=None,
            expected_mutation_epoch=token[1], expected_session=session,
            expected_activation_receipt_v2=session.activation_receipt_v2)[0])
        self.assertIsNone(self.service.get_active_session())

    def test_legacy_default_calls_still_publish_and_clear(self):
        first = self.service.ensure_active_session(map_name='legacy', frame_id='map')
        self.assertEqual(first.identity_schema_version, 1)
        self.assertIs(self.service.ensure_active_session(map_name='legacy', frame_id='map'), first)
        token = self.service.get_active_session_token()
        accepted, second = self.service.compare_and_set_active_session(
            expected_session_id=first.session_id, expected_live_mapping_identity=None,
            expected_mutation_epoch=token[1], map_name='other', frame_id='map', refresh=False)
        self.assertTrue(accepted)
        self.assertEqual(second.identity_schema_version, 1)
        self.service.clear_active_session()
        self.assertIsNone(self.store.sessions[-1])
        self.assertEqual(self.service.get_active_session_token()[1], 3)


if __name__ == '__main__':
    unittest.main()
