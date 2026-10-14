#!/usr/bin/env python3
"""Offline Python 3.8 checks for executor context and terminal-name compatibility.

Run: python3 -B scripts/check_python38_runtime_compatibility.py
Creates FastAPI apps with fake controllers, invoking registered endpoints directly.
No HTTP listener, model, device, subprocess, or external service is started.
"""
import asyncio
from contextvars import ContextVar
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fastapi import Request
from services.geometry import geometry_runtime_provider as geometry
from services.scene_decision.backends import SceneDecisionError
from services.scene_decision.local_server import create_app

TRACE = ContextVar('runtime_qualification_trace', default='unset')
VALID_BODY = json.dumps({
    'state': {'offline': True},
    'categories': {'unknown': 'Unknown', 'room': 'Room'},
    'images': [],
}).encode('utf-8')


def request(body=VALID_BODY, headers=None):
    values = [(b'content-type', b'application/json')]
    if headers:
        values.extend(headers)
    sent = False

    async def receive():
        nonlocal sent
        if sent:
            return {'type': 'http.disconnect'}
        sent = True
        return {'type': 'http.request', 'body': body, 'more_body': False}

    return Request({'type': 'http', 'method': 'POST', 'path': '/v1/scene-decisions',
                    'headers': values}, receive)


def endpoint(app, path='/v1/scene-decisions'):
    return next(route.endpoint for route in app.routes if route.path == path)


class Controller:
    def __init__(self):
        self.started = 0
        self.closed = 0
        self.decisions = []
        self.error = None
        self.close_error = None
        self.entered = threading.Event()
        self.release = threading.Event()
        self.block_decision = False
        self.block_close = False
        self.thread = None
        self.context = None
        self.released = None

    def start(self):
        self.started += 1

    def health(self):
        return {'ready': True}

    def observe(self, should_block):
        self.thread = threading.get_ident()
        self.context = TRACE.get()
        TRACE.set('worker-only')
        self.entered.set()
        if should_block:
            self.released = self.release.wait(3)

    def decide(self, value):
        self.decisions.append(value)
        self.observe(self.block_decision)
        if self.error is not None:
            raise self.error
        return {'offline': True, 'trace': self.context}

    def close(self):
        self.closed += 1
        self.observe(self.block_close)
        if self.close_error is not None:
            raise self.close_error


class SceneExecutorChecks(unittest.IsolatedAsyncioTestCase):
    async def wait_until_entered(self, controller):
        # The worker waits on release. This coroutine must progress concurrently.
        for unused in range(2000):
            if controller.entered.is_set():
                return
            await asyncio.sleep(0.001)
        self.fail('fake controller did not enter within bounded observation window')

    async def test_decide_is_nonblocking_and_copies_context_without_leaking(self):
        controller = Controller()
        controller.block_decision = True
        app = create_app(controller)
        token = TRACE.set('request-context')
        work = asyncio.create_task(endpoint(app)(request()))
        try:
            await self.wait_until_entered(controller)
            self.assertFalse(work.done())
            self.assertNotEqual(controller.thread, threading.get_ident())
            self.assertEqual(controller.context, 'request-context')
            self.assertEqual(TRACE.get(), 'request-context')
        finally:
            controller.release.set()
            response = await work
            TRACE.reset(token)
        self.assertTrue(controller.released)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(json.loads(response.body), {'offline': True, 'trace': 'request-context'})
        self.assertEqual(controller.decisions[0].state, {'offline': True})

    async def test_decide_error_mapping_and_unexpected_error_propagation(self):
        controller = Controller()
        app = create_app(controller)
        for error, status, code in (
            (SceneDecisionError('synthetic busy', error_code='scene_decision_busy', http_status=429),
             429, 'scene_decision_busy'),
            (SceneDecisionError('synthetic timeout', error_code='scene_decision_timeout', http_status=504),
             504, 'scene_decision_timeout'),
            (ValueError('synthetic invalid request'), 422, 'invalid_request'),
        ):
            with self.subTest(error=type(error).__name__, status=status):
                controller.error = error
                response = await endpoint(app)(request())
                self.assertEqual(response.status_code, status)
                self.assertEqual(json.loads(response.body)['error'], code)
        controller.error = RuntimeError('unexpected controller failure')
        with self.assertRaisesRegex(RuntimeError, 'unexpected controller failure'):
            await endpoint(app)(request())

    async def test_admission_errors_do_not_call_controller(self):
        controller = Controller()
        secured = create_app(controller, token='offline-token')
        response = await endpoint(secured)(request())
        self.assertEqual(response.status_code, 401)
        app = create_app(controller)
        response = await endpoint(app)(request(body=b'invalid-json'))
        self.assertEqual(response.status_code, 422)
        self.assertEqual(controller.decisions, [])

    async def test_lifespan_close_is_nonblocking_and_copies_context(self):
        controller = Controller()
        controller.block_close = True
        app = create_app(controller)
        token = TRACE.set('shutdown-context')

        async def lifecycle():
            async with app.router.lifespan_context(app):
                self.assertEqual(controller.started, 1)
                self.assertEqual(controller.closed, 0)
            self.assertEqual(TRACE.get(), 'shutdown-context')

        work = asyncio.create_task(lifecycle())
        try:
            await self.wait_until_entered(controller)
            self.assertFalse(work.done())
            self.assertEqual(controller.closed, 1)
            self.assertNotEqual(controller.thread, threading.get_ident())
            self.assertEqual(controller.context, 'shutdown-context')
            self.assertEqual(TRACE.get(), 'shutdown-context')
        finally:
            controller.release.set()
            await work
            TRACE.reset(token)
        self.assertTrue(controller.released)

    async def test_lifespan_closes_after_body_error_and_propagates_close_error(self):
        controller = Controller()
        app = create_app(controller)
        with self.assertRaisesRegex(RuntimeError, 'body failure'):
            async with app.router.lifespan_context(app):
                raise RuntimeError('body failure')
        self.assertEqual(controller.closed, 1)
        controller.close_error = OSError('close failure')
        with self.assertRaisesRegex(OSError, 'close failure'):
            async with app.router.lifespan_context(app):
                pass
        self.assertEqual(controller.closed, 2)


class TerminalNameChecks(unittest.TestCase):
    def terminal(self, disposition):
        return SimpleNamespace(disposition=disposition, correction_revision=37,
                               request=SimpleNamespace(body_key='assigned_cloud_v2'),
                               to_dict=lambda: {'assigned_cloud_v2': {'detail': 'offline detail'},
                                                'preserved': 'unchanged'})

    def test_all_actual_enum_names_have_unchanged_payload(self):
        prefix = 'RELOCALIZE_DISPOSITION_'
        entries = geometry.pb2.RelocalizeDisposition.items()
        self.assertTrue(entries)
        for name, number in entries:
            with self.subTest(name=name):
                self.assertTrue(name.startswith(prefix))
                result = geometry._assigned_terminal_payload(self.terminal(number))
                self.assertEqual(result, {
                    'assigned_cloud_v2': {'detail': 'offline detail'},
                    'preserved': 'unchanged',
                    'success': number in geometry._RELOCALIZE_SUCCESS_DISPOSITIONS,
                    'disposition': number,
                    'disposition_name': name[len(prefix):].lower(),
                    'correction_revision': 37,
                    'detail': 'offline detail',
                })

    def test_prefix_removed_exactly_once_only_at_start(self):
        cases = (
            ('RELOCALIZE_DISPOSITION_APPLIED', 'applied'),
            ('APPLIED', 'applied'),
            ('XRELOCALIZE_DISPOSITION_APPLIED', 'xrelocalize_disposition_applied'),
            ('RELOCALIZE_DISPOSITION_RELOCALIZE_DISPOSITION_APPLIED', 'relocalize_disposition_applied'),
            ('RELOCALIZE_DISPOSITION_', ''),
            ('relocalize_disposition_APPLIED', 'relocalize_disposition_applied'),
            ('', ''),
        )
        for name, expected in cases:
            with self.subTest(name=name), patch.object(geometry.pb2.RelocalizeDisposition, 'Name', return_value=name):
                result = geometry._assigned_terminal_payload(self.terminal(1))
                self.assertEqual(result['disposition_name'], expected)

    def test_unknown_enum_still_rejected(self):
        with self.assertRaises(ValueError):
            geometry._assigned_terminal_payload(self.terminal(999999))


if __name__ == '__main__':
    unittest.main(verbosity=2)
