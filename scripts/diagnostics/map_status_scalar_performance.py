"""Offline paired legacy status/handler measurement with explicit fixture vectors.

Requires an unedited baseline map_workspace.py source copy. Runs bounded,
interleaved arms over one real file-backed MemoryService and map catalog.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import statistics
import sys
import tempfile
import time
from contextlib import ExitStack
from types import MethodType, SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'tests'), str(ROOT / 'scripts')]
# Existing framework-only initialization occurs before the application guard.
from scripts.diagnostics.verify_python_guarded_integration import protected, counts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rows', type=int, default=1000)
    args = parser.parse_args()
    assert 1 <= args.rows <= 5000
    with ExitStack() as guard:
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                guard.enter_context(patch.object(hashlib, name, protected(name, getattr(hashlib, name))))
        from test_map_status_scalar import MapStatusScalarChecks
        from contracts.map_workspace import MapWorkspace
        from contracts.memory import MemoryStoreSummary
        from scripts.check_memory_storage_transactions import NOW
        from skills import map_workspace
        from skills.handlers.localization_skills import handle_list_maps
        tree = ast.parse(args.baseline_source.read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'MapWorkspaceService')
        method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '_sync_asset')
        namespace = dict(vars(map_workspace))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(args.baseline_source), 'exec'), namespace)
        methods = {'baseline': namespace['_sync_asset'], 'candidate': map_workspace.MapWorkspaceService._sync_asset}
        case = MapStatusScalarChecks()
        results = []
        setup = time.perf_counter()
        try:
            with tempfile.TemporaryDirectory(prefix='neh-status-perf-') as directory:
                memory = case.prepared(args.rows, path=str(Path(directory) / 'memory.db'))
                owner = case.owner(memory)
                owner._repository.save_map_asset(case.asset())
                owner._repository.set_active_map_name('library')
                for number in range(7):
                    owner._repository.save_map_asset(case.asset(map_name='inactive%d' % number,
                        display_name='Inactive %d' % number, active=False))
                setup_seconds = time.perf_counter() - setup
                summaries = memory.get_summary
                snapshot = memory._durable_index.query_snapshot
                library_list = memory.list_memory_libraries
                with patch('services.memory.service.MemoryStoreSummary', lambda **kw: MemoryStoreSummary(timestamp=NOW, **kw)), \
                        patch('skills.map_workspace.MapWorkspace', lambda **kw: MapWorkspace(timestamp=NOW, **kw)):
                    # Fix only generated summary/workspace clocks, preserving source and asset timestamps.
                    for scope in ('list_maps', 'handle_list_maps'):
                        outputs = []
                        for arm in ('baseline', 'candidate', 'candidate', 'baseline', 'baseline', 'candidate'):
                            owner._sync_asset = MethodType(methods[arm], owner)
                            calls = {'summary': 0, 'snapshot_rows': 0, 'snapshot_pages': 0, 'library_list': 0}
                            def counted_summary():
                                calls['summary'] += 1
                                return summaries()
                            def counted_snapshot(visible, **kw):
                                calls['snapshot_rows'] += len(visible)
                                calls['snapshot_pages'] += 1
                                return snapshot(visible, **kw)
                            def counted_libraries(*a, **kw):
                                calls['library_list'] += 1
                                return library_list(*a, **kw)
                            with patch.object(memory, 'get_summary', counted_summary), \
                                    patch.object(memory._durable_index, 'query_snapshot', counted_snapshot), \
                                    patch.object(memory, 'list_memory_libraries', counted_libraries):
                                started = time.perf_counter()
                                cpu = time.process_time()
                                result = (owner.list_maps(refresh_localization=False) if scope == 'list_maps' else
                                          handle_list_maps(SimpleNamespace(map_workspace_service=owner), {}))
                                cpu_seconds = time.process_time() - cpu
                                seconds = time.perf_counter() - started
                            document = ([asset.model_dump(mode='json') for asset in result] if scope == 'list_maps' else
                                {'maps': [asset.model_dump(mode='json') for asset in result['maps']],
                                 'active_workspace': result['active_workspace'].model_dump(mode='json')})
                            outputs.append(document)
                            results.append(dict(scope=scope, arm=arm, wall_seconds=seconds, cpu_seconds=cpu_seconds, **calls))
                        assert all(output == outputs[0] for output in outputs), scope + ' complete output differs'
                with memory._repository.storage.read_snapshot() as tx:
                    synchronous = tx.execute('PRAGMA synchronous').fetchone()[0]
        finally:
            case.doCleanups()
        document = dict(python=sys.version, rows=args.rows, assets=8, active_schema=1,
            setup_seconds=setup_seconds, sqlite_synchronous=synchronous,
            baseline_source=str(args.baseline_source), complete_output_equal=True,
            clocks='Only generated MemoryStoreSummary and MapWorkspace timestamps fixed to existing NOW fixture',
            runs=results, guard=counts)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(document, indent=2) + '\n')
        for scope in ('list_maps', 'handle_list_maps'):
            print(scope, {arm: statistics.median(row['wall_seconds'] for row in results if row['scope'] == scope and row['arm'] == arm)
                          for arm in methods})
        print('GUARD', counts)


if __name__ == '__main__':
    main()
