# Guarded Python integration — 2026-10-13

## Result and provenance

Python **3.8.10: 1,160 passed, 0 failed, 0 skipped, 1 explicitly deselected**, one existing httpx raw-upload deprecation warning. Pytest elapsed **62.13 seconds**; outer runner elapsed 62.807 seconds; exit 0.

Execution HEAD before and after was exactly `3073f85e731842668dafbb7ebca223d2236ae863`. Subsequent native Scan/RSS commits do not change the Python production code qualified here. This result belongs to the original execution, not a later rerun. Concurrent native work was outside this Python qualification. No production files or test expectations were modified by the verification.

[Original execution JSON](python-guarded-integration-20261013-results.json) preserves the interpreter, arguments, before/after HEAD, exact per-node results and guard counters unchanged. Workspace absolute paths in that historical JSON identify the original execution; the checked-in runner below is portable.

## Exact selection and exclusions

The [57-target list](../../scripts/diagnostics/python-guarded-integration-20261013-targets.txt) is the deduplicated union of:

- The earlier [652-case and 572-case selections](../验证证据/overnight-python-integration-20261013.md)
- Seven [artifact metadata regression modules](../验证证据/performance-artifact-metadata-access.md), previously reporting 177 cases
- Eight [AR1 qualification modules](../验证证据/ar1-assigned-import-proposal-20261013.md), previously reporting 143 cases

Historical counts are not added together. Actual final collection found 1,161 cases, with 1,160 unique selected node IDs; each has a passing call-phase report in the preserved JSON. The artifact runtime module overlaps the earlier broad selection and runs once.

The sole exclusion is:

```text
tests/test_assigned_asset_storage.py::LegacyAssetStorageCompatibilityTests::test_literal_pred1_bundle_and_manifest_reopen_and_repeat_finalize
```

That fixture explicitly computes `hashlib.sha256` on the literal legacy manifest and exercises legacy digest verification. It was deselected before execution to honor the no-content-hash instruction. No expected digest was computed. Earlier quiet logs do not individually prove whether this case ran; their full-module commands and saved arguments show inclusion. This audit finding does not retroactively invalidate or expand their recorded results.

The final run **includes and passes** `test_abrupt_exit_after_atomic_install_can_restart_and_retry`. An initial collect-only pass retained an old fork exclusion; review of newer AR1 verbose evidence established that the case had passed in this environment, so it was included in the final execution. Historical fork limitations are preserved as historical evidence, not claimed as a current blocker. This does not establish real UDS qualification.

## Hash guard and framework boundary

The [guarded runner](../../scripts/diagnostics/verify_python_guarded_integration.py) preloads pytest/FastAPI framework initialization, then guards all available hashlib guaranteed algorithms plus `new` and `file_digest` **before selected application or test imports**. Known checkpoint-payload, checkpoint-store, geometry-provider and legacy asset-store digest wrappers are also guarded before fixtures. A prohibited invocation raises instead of computing a digest.

Observed application hash attempts: **0**.

One narrow exception preserves Starlette's existing HTTP `FileResponse.set_stat_headers` metadata ETag behavior. The exception requires the exact `_compat.md5_hexdigest` caller, the exact `set_stat_headers` code object, and input equal to that frame's encoded `etag_base`. It hashes mtime-size metadata, not file payload. It is never manually invoked or used for fixtures, expected values or provenance.

Observed existing framework metadata ETag calls: **4**, counted separately. This result does not claim that no framework hash executes. The exception preserves original HTTP response behavior rather than changing production code to bypass a regression.

## Portable reproduction

Use a project Python 3.8.10 environment with its dependencies installed. The runner derives the repository root from its own path and reads the adjacent frozen target list. It requires an explicit output directory; there is no default result write inside the repository.

From the repository root, choose an evidence directory outside the checkout:

```sh
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python -B scripts/diagnostics/verify_python_guarded_integration.py \
  --output-dir ../python-guarded-integration-evidence --collect-only
python -B scripts/diagnostics/verify_python_guarded_integration.py \
  --output-dir ../python-guarded-integration-evidence
```

Capture stdout separately if a console log is wanted. `collect.json` and `execution.json` are separate files; rerunning a mode replaces that mode's file in the explicitly selected output directory. The runner sets scripts/tests/root import paths and disables pytest's cache plugin. Moving it into the repository changes only argument/path/output handling; the original execution runner remains unchanged in workspace evidence beside its logs.

The portable runner was checked by collection only on Python 3.8.10: **1,160 selected / 1,161 collected, 1 deselected**, in 1.69 seconds, with zero application hash attempts and zero framework ETag calls. Its ordered selected node IDs and exclusion exactly match the original execution JSON. The 1,160-test execution was not repeated or attributed to this path adaptation.

## Limits

This is a bounded offline Python integration pass on Python 3.8.10 only. It is not a full repository suite, other-Python-version pass, robot/deployment qualification, real UDS check, native build, network service test, or system CPU/latency measurement. The RSS and Scan benchmark CPU lanes were released before execution. The existing httpx warning remains unchanged.

Original workspace evidence: `../neh-evidence/python-integration-guarded-20261013/`, including the unchanged execution runner, target list, logs, JSON and the initial two-exclusion collection record.
