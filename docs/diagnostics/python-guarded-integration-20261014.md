# Updated guarded Python integration — 2026-10-14

## Result

Python **3.8.10: 1,227 passed, 0 failed, 0 skipped, 1 explicitly deselected**, with one existing httpx raw-upload deprecation warning. Pytest elapsed **65.42 seconds**; outer guarded runner elapsed **66.061 seconds**; exit **0**. Exactly one actual integration execution was performed; no collect-only or repeat execution is represented here.

All 1,227 selected node IDs are unique and have a passing call-phase result in the [execution evidence](python-guarded-integration-20261014-results.json). This is an updated bounded offline integration qualification, not a full repository suite or performance benchmark.

## Current scope

The [shared explicit target list](../../scripts/diagnostics/python-guarded-integration-20261013-targets.txt) now has **59 unique targets**. The original 57 targets were preserved, and two modules added:

- `tests/test_map_status_scalar.py`: **10 passed**, including the new map-status scalar regressions
- `scripts/check_installer_naming_identity.py`: **57 passed**, including current SDK source/build reuse and managed dependency binding cases

The installer module was **absent** from the prior 57-target list. It is now explicitly included once. Its Git, CMake, subprocess and network operations are fakes; it does not rerun the separately qualified real CMake scripts. No unrelated hash-producing fixtures or broad repository discovery were added. The increase from the historical 1,160 selected cases is exactly these 67 added cases.

The [2026-10-13 report](python-guarded-integration-20261013.md) and its execution JSON remain historical evidence for their original scope and HEAD. The dated filename of the shared target list was retained for runner compatibility; the frozen targets and arguments in each run's JSON identify its actual scope.

## Hash guard and exact exclusion

The [guarded runner](../../scripts/diagnostics/verify_python_guarded_integration.py) was not changed. It initializes pytest/FastAPI framework imports first, then guards all available guaranteed hashlib algorithms plus `new` and `file_digest` before selected application/test imports, and guards the four known payload/asset digest wrappers.

Observed application hash attempts: **0**. No application or manual content digest was computed for fixtures, assertions, evidence or provenance. Installer cases additionally prohibit their own hash/name-UUID calls and real command/network execution.

The existing narrow framework exception is unchanged: only Starlette `_compat.md5_hexdigest`, called from the exact `FileResponse.set_stat_headers` code object, with input exactly equal to that frame's encoded `etag_base`, may compute its existing mtime-size metadata ETag. Observed framework metadata ETag calls: **4**. This is metadata behavior, not a claim that all framework hashes are absent.

The sole explicit exclusion is unchanged:

```text
tests/test_assigned_asset_storage.py::LegacyAssetStorageCompatibilityTests::test_literal_pred1_bundle_and_manifest_reopen_and_repeat_finalize
```

That legacy pred1 fixture explicitly computes a content digest. It was deselected before execution; no expected digest was calculated. No additional exclusions or weakened assertions were introduced.

## Source and dirty-worktree provenance

- Execution HEAD before: `35ac9003d55699678646aeaf998a351359698a6f`
- Execution HEAD after: `041b6f186b032e8995ef4c5d87712854f2b38fbe`
- The intervening commit added only the read-only relocalization diagnostic report, result JSON and two probe files; it did not change production Python, selected tests or the guarded runner
- Therefore the qualified application/test source is the committed `35ac900` state, including prior map-status scalar changes and the SDK installer changes; this is not a claim that the worktree was clean or HEAD remained fixed

Exact HEAD delta:

```text
A	docs/diagnostics/relocalization-pair-reuse-audit-20261014-results.json
A	docs/diagnostics/relocalization-pair-reuse-audit-20261014.md
A	tools/diagnostics/relocalization_pair_reuse_probe.py
A	tools/diagnostics/relocalization_pair_reuse_probe.rs
```

Dirty manifest immediately before execution:

```text
 M docs/performance-progress-20261014.md
 M docs/公开文档/系统性能优化进度与后续计划.md
 M scripts/diagnostics/python-guarded-integration-20261013-targets.txt
?? docs/diagnostics/relocalization-pair-reuse-audit-20261014-results.json
?? tools/diagnostics/relocalization_pair_reuse_probe.py
?? tools/diagnostics/relocalization_pair_reuse_probe.rs
```

Dirty manifest after execution, before creating this report and result JSON:

```text
 M docs/performance-progress-20261014.md
 M docs/公开文档/系统性能优化进度与后续计划.md
 M scripts/diagnostics/python-guarded-integration-20261013-targets.txt
```

The two performance Markdown files belong to the concurrent report worker. The localization diagnostic files belonged to the concurrent audit worker and were committed during this execution. This verification changed only the shared target list and created this dated report/result JSON; no production files, test expectations, Git state, toolchain or shell profile were edited by this verifier.

## Reproduction and limits

From the repository root, with the existing Python 3.8.10 dependencies:

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
/workspace/scratch/71cdb9d17732/neh-toolchain/venv-py3810/bin/python -B \
  scripts/diagnostics/verify_python_guarded_integration.py \
  --output-dir ../neh-evidence/python-integration-guarded-20261014
```

The localization audit worker confirmed its CPU lane clear since 00:38:40 UTC before this run. Elapsed time is qualification runtime, not a controlled benchmark or performance comparison. Raw execution JSON, stdout and before/after manifests are preserved in `../neh-evidence/python-integration-guarded-20261014/`; the checked-in result JSON retains all raw runner fields and adds the explicit qualification metadata.

This result does not qualify other Python versions, a full repository suite, deployment/robot operation, real UDS, native builds, network services, or system CPU/latency. Real CMake qualification remains separate. The pre-existing httpx warning was not changed.
