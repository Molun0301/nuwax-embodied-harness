#!/usr/bin/env python3
"""Run only the six cancellation and six prepared-publication regressions.

Use the real project source next to this script. No transport, process, or
other test classes are admitted by this runner.
"""
from contextlib import ExitStack
import hashlib
import importlib.util
from pathlib import Path
import socket
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
FOCUSED = (
    'test_attempted_item_is_not_replayed_and_next_publication_progresses',
    'test_concurrent_accepted_publisher_is_released',
    'test_reentrant_publication_keeps_fifo_without_duplicate_delivery',
    'test_first_interruption_survives_later_interruptions_and_ordinary_error',
    'test_shutdown_releases_queued_waiter_without_delivering_it',
    'test_old_rethrow_cannot_clear_a_new_drainer_after_empty_handoff',
)
PREPARED = (
    'test_no_prepare_callbacks_and_abandoned_work_adopted_by_next_dispatch',
    'test_shutdown_completes_pending_and_foreign_owner_rejected',
    'test_unclaimed_preparations_bounded_before_failed_cursor_allocation',
    'test_cross_thread_dispatch_and_reentrant_publish_preserve_order',
    'test_pending_shutdown_releases_waiters_without_running_their_callbacks',
    'test_async_latest_only_coalescing_and_gap_reporting_survive_prepare',
)
SOURCES = ('core/stream_bus.py', 'tests/test_stream_bus_cancellation.py',
           'scripts/check_map_snapshot_revisions.py')


class NoSocket(socket.socket):
    def __new__(cls, *args, **kwargs):
        raise AssertionError('socket construction forbidden in this component suite')


def verify_imports():
    for name, module in tuple(sys.modules.items()):
        filename = getattr(module, '__file__', None)
        if not filename:
            continue
        declared = ROOT.joinpath(*name.split('.'))
        candidates = (declared.with_suffix('.py'), declared / '__init__.py')
        expected = next((path for path in candidates if path.is_file()), None)
        if expected is not None and Path(filename).resolve() != expected.resolve():
            raise AssertionError('repository module imported from wrong tree: ' + name)
    actual = Path(sys.modules['core.stream_bus'].__file__).resolve()
    if actual != ROOT / 'core/stream_bus.py':
        raise AssertionError('real restored StreamBus was not imported')
    print('SOURCE', actual, flush=True)


def load(name, relative):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    if Path(module.__file__).resolve() != path.resolve():
        raise AssertionError('test source path changed')
    return module


def main():
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(ROOT))
    before = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCES}
    print('INTERPRETER', sys.executable, sys.version, flush=True)
    print('SOURCE_SHA256', before, flush=True)
    with ExitStack() as guards:
        guards.enter_context(patch.object(socket, 'socket', NoSocket))
        guards.enter_context(patch.object(subprocess, 'Popen', side_effect=AssertionError('process forbidden')))
        import grpc
        import lcm
        for name in ('insecure_channel', 'secure_channel'):
            guards.enter_context(patch.object(grpc, name, side_effect=AssertionError('real grpc forbidden')))
        guards.enter_context(patch.object(lcm, 'LCM', side_effect=AssertionError('real LCM forbidden')))
        focused = load('stream_cancellation_fixture', 'tests/test_stream_bus_cancellation.py')
        prepared = load('stream_prepared_fixture', 'scripts/check_map_snapshot_revisions.py')
        suite = unittest.TestSuite()
        for module, cls_name, methods in (
            (focused, 'StreamBusCancellationTests', FOCUSED),
            (prepared, 'PreparedChecks', PREPARED),
        ):
            case = getattr(module, cls_name)
            for method in methods:
                suite.addTest(case(method))
        if suite.countTestCases() != 12:
            raise AssertionError('exact test selection changed')
        verify_imports()
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        verify_imports()
        after = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCES}
        if before != after:
            raise AssertionError('reviewed source changed during execution')
        return int(not result.wasSuccessful() or bool(result.skipped) or result.testsRun != 12)


if __name__ == '__main__':
    raise SystemExit(main())
