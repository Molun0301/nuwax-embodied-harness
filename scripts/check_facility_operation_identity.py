#!/usr/bin/env python3
"""设施意图独立身份、持久化窗口和未知结果恢复的离线回归。"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager, nullcontext
from datetime import datetime, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import stat
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch
import uuid

import httpx
import pydantic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
ATTEMPTS = []


@contextmanager
def no_hash():
    def forbidden(*args, **kwargs):
        ATTEMPTS.append(True)
        raise AssertionError('禁止执行应用摘要或名称UUID')
    with ExitStack() as stack:
        for name in ('new', 'md5', 'sha1', 'sha224', 'sha256', 'sha384', 'sha512',
                     'sha3_224', 'sha3_256', 'sha3_384', 'sha3_512', 'shake_128', 'shake_256', 'blake2b', 'blake2s'):
            stack.enter_context(patch.object(hashlib, name, side_effect=forbidden))
        for name in ('uuid3', 'uuid5'):
            stack.enter_context(patch.object(uuid, name, side_effect=forbidden))
        yield


# 只载入需要的真实模块，不执行聚合__init__以免启动全服务或读取真实settings。
for name in ('contracts', 'providers', 'services', 'services.navigation', 'services.mapping'):
    package = types.ModuleType(name)
    package.__path__ = [str(ROOT.joinpath(*name.split('.')))]
    sys.modules[name] = package
with no_hash():
    execution = importlib.import_module('services.navigation.building_execution')
    http_provider = importlib.import_module('services.navigation.facilities.http_provider')
    from contracts.building_navigation import (BuildingTopology, FacilityOperation, FloorConnection,
        FloorExecutionState, FloorNavigationRequest, FloorPose, FloorRecord)
    from providers.facilities import FacilityOutcomeUnknown, FacilityUnavailable

WORK = Path(tempfile.gettempdir()) / 'neh-facility-operation-identity-checks'
IDENTITY = ('operation_id', 'session_id', 'action', 'target_floor_id')


def identity(operation):
    return tuple(getattr(operation, name) for name in IDENTITY)


class Crash(BaseException):
    """绕过普通异常收尾，模拟进程在指定窗口退出。"""


class Ports:
    def __init__(self):
        self.ensure_active = Mock()
        self.progress = Mock()
        self.stop_navigation = Mock(return_value={'zeroed': True})
        self.interrupt_barrier = Mock()

    def remaining_sec(self):
        return 10.0


class Provider:
    supported_connection_kinds = ('elevator',)
    provider_name = 'fake'

    def __init__(self, case):
        self.case = case
        self.submissions = []
        self.queries = []
        self.receipts = {}
        self.query_states = []
        self.mode = 'success'

    def is_available(self):
        return True

    def submit_operation(self, facility_id, operation, **kwargs):
        # 从真实磁盘重读，不以调用方内存对象冒充已持久化意图。
        saved = self.case.service.journal.load()
        self.case.assertEqual(identity(saved.pending_operation), identity(operation))
        self.case.assertEqual(saved.facility_session_id, operation.session_id)
        self.submissions.append(operation)
        if self.mode == 'crash_before_apply':
            raise Crash()
        receipt = operation.model_copy(update={'state': 'succeeded'})
        self.receipts[identity(operation)] = receipt
        if self.mode == 'crash_after_apply':
            raise Crash()
        if self.mode == 'unknown':
            raise FacilityOutcomeUnknown('合成超时')
        return receipt

    def query_operation(self, facility_id, operation, **kwargs):
        self.queries.append(operation)
        if self.query_states:
            state = self.query_states.pop(0)
            if isinstance(state, BaseException):
                raise state
            return operation.model_copy(update={'state': state})
        return self.receipts.get(identity(operation), operation.model_copy(update={'state': 'not_found'}))


class FacilityIdentityChecks(unittest.TestCase):
    def setUp(self):
        WORK.mkdir(parents=True, exist_ok=True)
        directory = tempfile.TemporaryDirectory(prefix='fixture-', dir=WORK)
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        pose = FloorPose(x=0, y=0)
        self.floors = (FloorRecord(floor_id='f1', display_name='一层', map_name='map1'),
                       FloorRecord(floor_id='f2', display_name='二层', map_name='map2'))
        self.connection = FloorConnection(connection_id='lift', from_floor_id='f1', to_floor_id='f2',
            provider_id='fake', facility_id='synthetic', waiting_pose=pose, boarding_pose=pose,
            arrival_pose=pose, exit_pose=pose, docking_calibration_ref='fixture-calibration', expected_transfer_sec=10)
        topology = BuildingTopology(building_id='building', display_name='合成楼', revision=1,
            floors=self.floors, connections=(self.connection,))
        maps = types.SimpleNamespace(root_dir=self.root, map_activation_guard=nullcontext)
        self.catalog = types.SimpleNamespace(maps=maps, get=lambda _: topology, validate_frozen=lambda _: topology)
        self.request = FloorNavigationRequest(request_id='request', building_id='building', topology_revision=1,
            source_floor_id='f1', target_floor_id='f2', target_map_name='map2', create_target_map=True,
            target_pose=pose, route=('lift',))
        self.provider = Provider(self)
        self.service = execution.BuildingNavigationService(self.catalog, {'fake': self.provider})
        self.ports = Ports()
        self.addCleanup(patch.stopall)
        patch.object(execution.time, 'sleep', return_value=None).start()

    def state(self, *, pending=False, status='unknown', session=None):
        # 合成旧ID直接写常量；绝不重算历史SHA。
        session = session or 'a' * 64
        operation = FacilityOperation(operation_id='b' * 64, session_id=session, action='call',
            target_floor_id='f1', state=status) if pending else None
        return FloorExecutionState(request=self.request, current_floor_id='f1', stage='calling',
            facility_session_id=session, pending_operation=operation)

    def invoke(self, state=None):
        state = state or self.state()
        self.service._operation(state, self.connection, 'call', 'f1', self.ports)
        return state

    def reload(self):
        self.service = execution.BuildingNavigationService(self.catalog, {'fake': self.provider})
        return self.service.journal.load()

    def no_allocation(self):
        return patch.object(execution.uuid, 'uuid4', side_effect=AssertionError('不得重新分配已有身份'))


    def test_new_operation_allocates_once_and_is_durable_before_submit(self):
        with patch.object(execution.uuid, 'uuid4', wraps=uuid.uuid4) as assigned:
            state = self.invoke()
            self.assertEqual(assigned.call_count, 1)
        self.assertRegex(state.pending_operation.operation_id, r'^facility-operation-[0-9a-f]{32}$')
        self.assertEqual(self.service.journal.load().pending_operation, state.pending_operation)
        self.assertEqual(len(self.provider.submissions), 1)

    def test_separate_new_operations_have_independent_ids(self):
        first = self.invoke().pending_operation.operation_id
        state = self.state()
        second = self.invoke(state).pending_operation.operation_id
        self.assertNotEqual(first, second)

    def test_session_is_saved_once_before_reserve_and_preserved_after_reload(self):
        state = self.state(); state.stage = 'reserving'; state.facility_session_id = None
        with patch.object(self.service, '_operation', side_effect=Crash()), patch.object(execution.uuid, 'uuid4', wraps=uuid.uuid4) as assigned:
            with self.assertRaises(Crash): self.service._run_connection(state, self.connection, *self.floors, self.ports)
            self.assertEqual(assigned.call_count, 1)
        session = self.service.journal.load().facility_session_id
        self.assertRegex(session, r'^facility-session-[0-9a-f]{32}$')
        state = self.reload()
        with self.no_allocation(), patch.object(self.service, '_operation', side_effect=Crash()):
            with self.assertRaises(Crash): self.service._run_connection(state, self.connection, *self.floors, self.ports)
        self.assertEqual(self.service.journal.load().facility_session_id, session)
        self.assertFalse(self.provider.submissions)

    def test_save_failure_prevents_rpc(self):
        with patch.object(self.service.journal, 'save', side_effect=OSError('fixture write failed')):
            with self.assertRaises(OSError): self.invoke()
        self.assertFalse(self.provider.submissions)

    def test_file_fsync_failure_prevents_rpc(self):
        with patch.object(os, 'fsync', side_effect=OSError('fixture file fsync failed')):
            with self.assertRaises(OSError): self.invoke()
        self.assertFalse(self.provider.submissions)
        self.assertIsNone(self.service.journal.load())

    def test_directory_fsync_failure_prevents_rpc_even_if_replaced(self):
        real = os.fsync
        def fail_directory(fd):
            if stat.S_ISDIR(os.fstat(fd).st_mode): raise OSError('fixture directory fsync failed')
            return real(fd)
        with patch.object(os, 'fsync', side_effect=fail_directory):
            with self.assertRaises(OSError): self.invoke()
        self.assertFalse(self.provider.submissions)
        self.assertIsNotNone(self.service.journal.load().pending_operation)

    def test_crash_after_intent_before_rpc_recovers_original_id(self):
        self.ports.progress.side_effect = Crash()
        with self.assertRaises(Crash): self.invoke()
        saved = self.reload(); original = identity(saved.pending_operation)
        self.assertFalse(self.provider.submissions)
        self.ports.progress.side_effect = None
        with self.no_allocation(): self.invoke(saved)
        self.assertEqual(identity(self.provider.queries[0]), original)
        self.assertEqual(identity(self.provider.submissions[0]), original)

    def test_cancel_after_intent_before_rpc_does_not_submit(self):
        self.ports.ensure_active.side_effect = RuntimeError('合成取消')
        with self.assertRaises(RuntimeError): self.invoke()
        self.assertFalse(self.provider.submissions)
        self.assertIsNotNone(self.service.journal.load().pending_operation)

    def test_crash_during_rpc_before_apply_retries_same_id_only_after_not_found(self):
        self.provider.mode = 'crash_before_apply'
        with self.assertRaises(Crash): self.invoke()
        saved = self.reload(); original = identity(saved.pending_operation)
        self.provider.mode = 'success'
        with self.no_allocation(): self.invoke(saved)
        self.assertEqual(len(self.provider.queries), 1)
        self.assertEqual([identity(x) for x in self.provider.submissions], [original, original])

    def test_crash_during_rpc_after_apply_queries_without_reexecution(self):
        self.provider.mode = 'crash_after_apply'
        with self.assertRaises(Crash): self.invoke()
        saved = self.reload()
        with self.no_allocation(): self.invoke(saved)
        self.assertEqual(len(self.provider.submissions), 1)
        self.assertEqual(len(self.provider.queries), 1)
        self.assertEqual(saved.pending_operation.state, 'succeeded')

    def test_crash_after_receipt_before_result_save_queries_original(self):
        save = self.service.journal.save
        def fail_result(state):
            if state.pending_operation and state.pending_operation.state == 'succeeded': raise Crash()
            save(state)
        with patch.object(self.service.journal, 'save', side_effect=fail_result):
            with self.assertRaises(Crash): self.invoke()
        saved = self.reload()
        self.assertEqual(saved.pending_operation.state, 'unknown')
        with self.no_allocation(): self.invoke(saved)
        self.assertEqual(len(self.provider.submissions), 1)
        self.assertEqual(saved.pending_operation.state, 'succeeded')

    def test_unknown_submit_response_queries_without_reexecution(self):
        self.provider.mode = 'unknown'
        state = self.invoke()
        self.assertEqual(state.pending_operation.state, 'succeeded')
        self.assertEqual(len(self.provider.submissions), 1)
        self.assertEqual(len(self.provider.queries), 1)

    def test_unknown_accepted_running_are_query_only_across_reloaded_legacy_state(self):
        self.service.journal.save(self.state(pending=True))
        saved = self.reload(); original = identity(saved.pending_operation)
        self.provider.query_states = ['unknown', 'accepted', 'running', 'succeeded']
        with self.no_allocation(): self.invoke(saved)
        self.assertFalse(self.provider.submissions)
        self.assertEqual([identity(x) for x in self.provider.queries], [original] * 4)

    def test_query_timeout_preserves_unknown_and_never_submits(self):
        saved = self.state(pending=True); self.service.journal.save(saved)
        self.provider.query_states = [FacilityUnavailable('合成查询超时')]
        with self.no_allocation(), self.assertRaises(FacilityUnavailable): self.invoke(saved)
        self.assertFalse(self.provider.submissions)
        self.assertEqual(self.service.journal.load().pending_operation.state, 'unknown')

    def test_repeated_matching_not_found_and_unknown_resubmits_only_same_ids(self):
        saved = self.state(pending=True); self.service.journal.save(saved)
        self.provider.mode = 'unknown'; self.provider.query_states = ['not_found', 'not_found', 'succeeded']
        with self.no_allocation(): self.invoke(saved)
        self.assertEqual(len(self.provider.submissions), 2)
        self.assertEqual({identity(x) for x in self.provider.submissions + self.provider.queries}, {identity(saved.pending_operation)})

    def test_pending_identity_mismatch_blocks_without_allocation_or_rpc(self):
        for field, value in [('session_id', 'other'), ('action', 'release'), ('target_floor_id', 'f2')]:
            with self.subTest(field=field):
                state = self.state(pending=True)
                state.pending_operation = state.pending_operation.model_copy(update={field: value})
                with self.no_allocation(), self.assertRaises(FacilityOutcomeUnknown): self.invoke(state)
        self.assertFalse(self.provider.submissions or self.provider.queries)

    def test_existing_success_returns_and_existing_failure_does_not_reallocate(self):
        with self.no_allocation():
            self.invoke(self.state(pending=True, status='succeeded'))
            with self.assertRaises(FacilityUnavailable): self.invoke(self.state(pending=True, status='failed'))
        self.assertFalse(self.provider.submissions or self.provider.queries)

    def test_stage_change_clears_only_successful_operation(self):
        state = self.state(pending=True)
        with self.assertRaises(FacilityOutcomeUnknown): self.service._save(state, self.ports, 'waiting_entry')
        self.assertIsNotNone(state.pending_operation)
        state.pending_operation = state.pending_operation.model_copy(update={'state': 'succeeded'})
        self.service._save(state, self.ports, 'waiting_entry')
        self.assertIsNone(self.service.journal.load().pending_operation)

    def test_explicit_resume_failed_allocates_new_operation_and_keeps_session(self):
        old = self.state(pending=True, status='failed'); self.service.journal.save(old)
        def connection(state, *args):
            self.invoke(state)
            raise Crash()
        with patch.object(self.service, '_run_connection', side_effect=connection):
            with self.assertRaises(Crash): self.service.run(self.request, self.ports, resume=True)
        saved = self.service.journal.load()
        self.assertEqual(saved.attempt, 1)
        self.assertEqual(saved.facility_session_id, old.facility_session_id)
        self.assertNotEqual(saved.pending_operation.operation_id, old.pending_operation.operation_id)
        self.assertEqual(len(self.provider.submissions), 1)

    def test_explicit_resume_unknown_keeps_legacy_operation_and_session(self):
        old = self.state(pending=True); self.service.journal.save(old)
        self.provider.query_states = ['succeeded']
        def connection(state, *args):
            self.invoke(state)
            raise Crash()
        with self.no_allocation(), patch.object(self.service, '_run_connection', side_effect=connection):
            with self.assertRaises(Crash): self.service.run(self.request, self.ports, resume=True)
        saved = self.service.journal.load()
        self.assertEqual(saved.attempt, 1)
        self.assertEqual(identity(saved.pending_operation), identity(old.pending_operation))
        self.assertFalse(self.provider.submissions)

    def test_resume_requires_confirmed_stop_before_facility_activity(self):
        self.service.journal.save(self.state(pending=True))
        self.ports.stop_navigation.return_value = {'zeroed': False}
        with self.no_allocation(), self.assertRaises(RuntimeError): self.service.run(self.request, self.ports, resume=True)
        self.assertFalse(self.provider.submissions or self.provider.queries)

    def test_execution_file_lock_allows_only_one_concurrent_owner(self):
        self.service.journal.save(self.state(pending=True))
        other = execution.BuildingNavigationService(self.catalog, {'fake': self.provider})
        with self.service.journal.execution_guard(), ThreadPoolExecutor(max_workers=1) as pool:
            pending = pool.submit(other.run, self.request, self.ports, resume=True)
            with self.assertRaisesRegex(RuntimeError, '已有跨层'): pending.result(timeout=2)
        self.assertFalse(self.provider.submissions or self.provider.queries)
        with other.journal.execution_guard(): pass

    def http(self, handler):
        client = httpx.Client(transport=httpx.MockTransport(handler), trust_env=False)
        self.addCleanup(client.close)
        return http_provider.HttpFacilityProvider('fake', 'https://facility.invalid', client=client)

    def test_http_contract_accepts_new_and_legacy_opaque_id_formats(self):
        requests = []
        expected = None
        def transport(request):
            requests.append(request)
            if request.method == 'PUT': self.assertEqual(json.loads(request.content), expected.model_dump(mode='json', exclude={'state', 'message'}))
            return httpx.Response(200, json=expected.model_copy(update={'state': 'succeeded'}).model_dump(mode='json'))
        provider = self.http(transport)
        for operation_id, session_id in [('facility-operation-' + '1' * 32, 'facility-session-' + '2' * 32), ('b' * 64, 'a' * 64)]:
            expected = FacilityOperation(operation_id=operation_id, session_id=session_id, action='reserve', state='unknown')
            provider.submit_operation('synthetic', expected); provider.query_operation('synthetic', expected)
            for request in requests[-2:]:
                self.assertIn('/sessions/' + session_id + '/operations/' + operation_id, request.url.path)
        self.assertEqual([request.method for request in requests], ['PUT', 'GET', 'PUT', 'GET'])

    def test_http_wrong_not_found_receipt_never_authorizes_replay(self):
        for field, value in [('operation_id', 'other'), ('session_id', 'other'), ('action', 'release'), ('target_floor_id', 'f2')]:
            with self.subTest(field=field):
                calls = []; state = self.state(pending=True)
                def transport(request):
                    calls.append(request.method)
                    receipt = state.pending_operation.model_copy(update={'state': 'not_found', field: value})
                    return httpx.Response(404, json=receipt.model_dump(mode='json'))
                self.service.providers['fake'] = self.http(transport)
                with self.no_allocation(), self.assertRaises(FacilityOutcomeUnknown): self.invoke(state)
                self.assertEqual(calls, ['GET'])

    def test_http_matching_not_found_retries_exact_durable_identity(self):
        state = self.state(pending=True); self.service.journal.save(state)
        expected = identity(state.pending_operation)
        calls = []
        def transport(request):
            calls.append(request)
            self.assertEqual(identity(self.service.journal.load().pending_operation), expected)
            reply = 'not_found' if request.method == 'GET' else 'succeeded'
            return httpx.Response(404 if reply == 'not_found' else 200,
                json=state.pending_operation.model_copy(update={'state': reply}).model_dump(mode='json'))
        self.service.providers['fake'] = self.http(transport)
        with self.no_allocation(): self.invoke(state)
        self.assertEqual([request.method for request in calls], ['GET', 'PUT'])
        self.assertEqual(calls[0].url, calls[1].url)
        self.assertEqual(identity(self.service.journal.load().pending_operation), expected)
        self.assertEqual(self.service.journal.load().pending_operation.state, 'succeeded')

    def test_http_bare_not_found_or_malformed_reply_never_authorizes_replay(self):
        for response in [httpx.Response(404, json={}), httpx.Response(404, content=b'')]:
            calls = []
            def transport(request): calls.append(request.method); return response
            self.service.providers['fake'] = self.http(transport)
            with self.no_allocation(), self.assertRaises(FacilityUnavailable): self.invoke(self.state(pending=True))
            self.assertEqual(calls, ['GET'])


if __name__ == '__main__':
    with no_hash():
        result = unittest.main(verbosity=2, exit=False)
    print('Forbidden digest/name-UUID attempts:', len(ATTEMPTS))
    raise SystemExit(0 if result.result.wasSuccessful() and not ATTEMPTS else 1)
