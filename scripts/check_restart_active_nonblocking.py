#!/usr/bin/env python3
"""Offline restart validation: preserve ordering/errors without blocking peer HTTP requests."""
from __future__ import annotations

import asyncio
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import httpx
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from core import config_profiles, config_values
from core.errors import GatewayError
from gateways.http import api
from gateways import settings, settings_restart

REAL_IS_ACTIVE = settings.SettingsApplication._is_active

# A pinned historical source can be checked without replacing the working tree.
if len(sys.argv) > 2 and sys.argv[1] == '--source':
    source = Path(sys.argv[2])
    sys.argv[1:3] = []
    candidate = ModuleType('restart_validation_candidate')
    exec(compile(source.read_text(), str(source), 'exec'), candidate.__dict__)
    api = candidate


class RestartActiveTests(unittest.IsolatedAsyncioTestCase):
    def fixture(self, *, active=True, location=True, error=None, gated=False,
                revision_error=None):
        stack = ExitStack()
        self.addCleanup(stack.close)
        directory = Path(stack.enter_context(tempfile.TemporaryDirectory()))
        marker = directory / 'active-profile.txt'
        marker.write_text('selected', encoding='utf-8')
        requested = {'path': directory / 'selected.env'}
        trace = []
        started = threading.Event()
        release = threading.Event()
        peer_finished = threading.Event()
        evidence = {}
        loop_thread = threading.get_ident()
        owner = object()

        def filesystem_backed_probe():
            trace.append('active.enter')
            evidence['thread'] = threading.get_ident()
            evidence['file'] = marker.read_text(encoding='utf-8')
            if gated:
                started.set()
                # Independent controlled releaser also permits the historical
                # event-loop-blocking implementation to finish without deadlock.
                evidence['released'] = release.wait(5)
            if error is not None:
                raise error
            trace.append('active.exit')
            return requested if active else {'path': directory / 'other.env'}

        application = SimpleNamespace(_active_location=filesystem_backed_probe,
                                      revision_owner=owner, start=lambda: None,
                                      close=lambda: None)
        application._is_active = REAL_IS_ACTIVE.__get__(application)

        def resolve(**kwargs):
            trace.append('location')
            self.assertEqual(kwargs['robot'], 'robot')
            self.assertEqual(kwargs['profile'], 'selected')
            return requested if location else None

        def revision(actual, token, **kwargs):
            trace.append('revision')
            self.assertIs(actual, requested)
            self.assertEqual(token, 'revision-token')
            self.assertIs(kwargs['revision_owner'], owner)
            if revision_error is not None:
                raise revision_error

        receipt = {'request_id': 'offline-restart', 'state': 'queued'}

        def queue(root):
            trace.append('queue')
            return receipt

        def dispatch(root, result):
            trace.append('dispatch')
            self.assertIs(result, receipt)

        def policy():
            evidence['peer_before_release'] = not release.is_set()
            trace.append('peer')
            peer_finished.set()
            return {'auth': 'disabled'}

        stack.enter_context(patch.object(settings, 'SettingsApplication', return_value=application))
        stack.enter_context(patch.object(config_values, 'configuration_location', side_effect=resolve))
        stack.enter_context(patch.object(config_profiles, 'check_revision', side_effect=revision))
        stack.enter_context(patch.object(settings_restart, 'queue_restart', side_effect=queue))
        stack.enter_context(patch.object(settings_restart, 'dispatch_restart', side_effect=dispatch))
        app = FastAPI()

        @app.exception_handler(GatewayError)
        async def gateway_error(request, exc):
            return JSONResponse({'code': exc.error_code}, status_code=exc.http_status)

        app.include_router(api.build_http_router(SimpleNamespace(), SimpleNamespace(export_policy=policy)))
        client = httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url='http://offline')
        self.addAsyncCleanup(client.aclose)
        return SimpleNamespace(client=client, trace=trace, evidence=evidence,
                               started=started, release=release, peer_finished=peer_finished,
                               loop_thread=loop_thread, receipt=receipt)

    async def restart(self, fixture):
        return await fixture.client.post('/api/config/restart', json={
            'robot': 'robot', 'profile': 'selected', 'revision': 'revision-token'})

    async def test_slow_active_filesystem_does_not_block_peer_http(self):
        f = self.fixture(gated=True)

        def releaser():
            if f.started.wait(5):
                # Timeout is an emergency escape for the negative baseline,
                # never a performance or pass/fail latency threshold.
                f.evidence['peer_observed_while_blocked'] = f.peer_finished.wait(2)
            f.release.set()

        guard = threading.Thread(target=releaser, name='offline-release-guard')
        guard.start()
        pending = asyncio.create_task(self.restart(f))
        try:
            while not f.started.is_set():
                if pending.done():
                    pending.result()
                    self.fail('active check was not reached')
                await asyncio.sleep(0)
            peer = await f.client.get('/api/security/policy')
            self.assertEqual(peer.status_code, 200)
            response = await pending
        finally:
            f.release.set()
            await asyncio.gather(pending, return_exceptions=True)
            guard.join(6)
        self.assertFalse(guard.is_alive())
        self.assertNotEqual(f.evidence['thread'], f.loop_thread)
        self.assertTrue(f.evidence['peer_before_release'])
        self.assertTrue(f.evidence['peer_observed_while_blocked'])
        self.assertTrue(f.evidence['released'])
        self.assertEqual(f.evidence['file'], 'selected')
        self.assertEqual(response.json(), f.receipt)
        self.assertEqual(f.trace, ['location', 'active.enter', 'peer', 'active.exit',
                                   'revision', 'queue', 'dispatch'])

    async def test_success_preserves_receipt_and_order(self):
        f = self.fixture()
        response = await self.restart(f)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), f.receipt)
        self.assertEqual(f.trace, ['location', 'active.enter', 'active.exit', 'revision', 'queue', 'dispatch'])

    async def test_inactive_keeps_409_and_never_queues(self):
        f = self.fixture(active=False)
        response = await self.restart(f)
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json(), {'code': 'configuration_not_active'})
        self.assertEqual(f.trace, ['location', 'active.enter', 'active.exit'])

    async def test_absent_location_skips_active_and_revision(self):
        f = self.fixture(location=False)
        response = await self.restart(f)
        self.assertEqual(response.json(), f.receipt)
        self.assertEqual(f.trace, ['location', 'queue', 'dispatch'])

    async def test_active_errors_propagate_unchanged_without_queue(self):
        for error in (OSError('offline io'), ValueError('invalid active path'),
                      TypeError('invalid active input'), config_profiles.ConfigurationConflict('stale active')):
            with self.subTest(error=type(error).__name__):
                f = self.fixture(error=error)
                with self.assertRaises(type(error)) as raised:
                    await self.restart(f)
                self.assertIs(raised.exception, error)
                self.assertEqual(f.trace, ['location', 'active.enter'])

    async def test_revision_conflict_never_queues(self):
        f = self.fixture(revision_error=config_profiles.ConfigurationConflict('stale'))
        response = await self.restart(f)
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.json(), {'code': 'configuration_conflict'})
        self.assertEqual(f.trace, ['location', 'active.enter', 'active.exit', 'revision'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
