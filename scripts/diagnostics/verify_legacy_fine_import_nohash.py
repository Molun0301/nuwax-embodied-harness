"""Run focused legacy-fine import/parser tests with application hashing disabled."""

import hashlib
import sys
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch


def main():
    root = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(root), str(root / 'tests')]
    from services.geometry import (
        checkpoint_payload, mapping_backend_checkpoint_store, geometry_runtime_provider)

    with ExitStack() as guard:
        for name in hashlib.algorithms_guaranteed | {'new', 'file_digest'}:
            if hasattr(hashlib, name):
                guard.enter_context(patch.object(hashlib, name,
                    side_effect=AssertionError('application hash prohibited')))
        for module in (checkpoint_payload, mapping_backend_checkpoint_store, geometry_runtime_provider):
            guard.enter_context(patch.object(module, 'payload_digest',
                side_effect=AssertionError('payload digest prohibited')))
        suite = unittest.defaultTestLoader.loadTestsFromNames([
            'test_legacy_cloud_import', 'test_checkpoint_identity_v2', 'test_fine_archive_vector'])
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
