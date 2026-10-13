"""Bounded integration selection; no application content hashes permitted."""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from contextlib import ExitStack
from unittest.mock import patch

# Framework initialization probes predate application/test imports.
import pytest
import fastapi
import starlette._compat
import starlette.responses

ROOT = Path(__file__).resolve().parents[2]
TARGETS = Path(__file__).resolve().with_name('python-guarded-integration-20261013-targets.txt')
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'tests'), str(ROOT)]
DESELECT = [
    'tests/test_assigned_asset_storage.py::LegacyAssetStorageCompatibilityTests::test_literal_pred1_bundle_and_manifest_reopen_and_repeat_finalize',
]
counts = {'application_hash_attempts': [], 'framework_metadata_etag_calls': 0}

def protected(name, original):
    def invoke(*args, **kwargs):
        caller = sys._getframe(1)
        upstream = caller.f_back
        if (name == 'md5' and caller.f_code.co_name == 'md5_hexdigest'
                and caller.f_code.co_filename == starlette._compat.__file__
                and upstream is not None
                and upstream.f_code is starlette.responses.FileResponse.set_stat_headers.__code__
                and args and args[0] == upstream.f_locals['etag_base'].encode()):
            counts['framework_metadata_etag_calls'] += 1
            return original(*args, **kwargs)
        counts['application_hash_attempts'].append({
            'name': name, 'file': caller.f_code.co_filename, 'line': caller.f_lineno})
        raise AssertionError('application content hash prohibited: ' + name)
    return invoke

class Report:
    def __init__(self):
        self.nodes = []
        self.deselected = []
        self.reports = []
    def pytest_collection_finish(self, session):
        self.nodes = [item.nodeid for item in session.items]
        assert len(self.nodes) == len(set(self.nodes)), 'duplicate collected node IDs'
    def pytest_deselected(self, items):
        self.deselected.extend(item.nodeid for item in items)
    def pytest_runtest_logreport(self, report):
        if report.when == 'call' or report.outcome != 'passed':
            self.reports.append({'nodeid': report.nodeid, 'phase': report.when,
                                 'outcome': report.outcome,
                                 'detail': str(report.longrepr) if report.longrepr else ''})

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='Explicit directory for result JSON; no implicit repository output')
    parser.add_argument('--collect-only', action='store_true')
    options = parser.parse_args()
    evidence = options.output_dir.expanduser().resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    os.chdir(str(ROOT))
    collect = options.collect_only
    label = 'collect' if collect else 'execution'
    targets = TARGETS.read_text().splitlines()
    args = ['-q', '-p', 'no:cacheprovider'] + targets
    for node in DESELECT:
        args.extend(['--deselect', node])
    if collect:
        args.append('--collect-only')
    plugin = Report()
    before = subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip()
    started = time.monotonic()
    code = 2
    try:
        with ExitStack() as guards:
            for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
                if hasattr(hashlib, name):
                    guards.enter_context(patch.object(hashlib, name, protected(name, getattr(hashlib,name))))
            wrappers = [
                ('services.geometry.checkpoint_payload', 'payload_digest'),
                ('services.geometry.mapping_backend_checkpoint_store', 'payload_digest'),
                ('services.geometry.geometry_runtime_provider', 'payload_digest'),
                ('services.mapping.asset_store', '_sha256_bytes'),
            ]
            for module_name, attr in wrappers:
                module = importlib.import_module(module_name)
                guards.enter_context(patch.object(module, attr, protected(module_name+'.'+attr, getattr(module,attr))))
            code = int(pytest.main(args, plugins=[plugin]))
    finally:
        result = dict(python=sys.version, executable=sys.executable, pytest_args=args,
            head_before=before, head_after=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            elapsed_seconds=time.monotonic()-started, returncode=code,
            selected_nodes=plugin.nodes, deselected_nodes=plugin.deselected, reports=plugin.reports, **counts)
        evidence.joinpath(label+'.json').write_text(json.dumps(result, indent=2)+'\n')
        print('GUARD SUMMARY:', json.dumps(counts))
    return code

if __name__ == '__main__':
    raise SystemExit(main())
