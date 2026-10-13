# Overnight Python integration — 2026-10-13

## Latest canonical-contract verification

On baseline `c1d251f`, with the approved test-only canonical-contract alignments below and unchanged production code, the repeated combined suite plus `tests/test_dense_legacy_epoch.py` reports **609 passed, 0 failed, 1 deselected, 1 warning in 41.03 seconds**. This is a bounded offline suite pass, not full deployment or robot qualification. The earlier 593/5/1 result remains recorded below for traceability.

## Initial scope and result

Actual interpreter: Python 3.8.10 from the existing workspace toolchain. Reviewed production snapshot: `7bfe8f4`, including the frozen limited-scene-listing implementation. Three test-only fixture corrections listed below were present during the final run. No production edits were made by this verification task.

**Initial combined run: 593 passed, 5 failed, 1 deselected, 1 warning, 39.34 seconds. This was not a full-suite pass.**

The run combines real task-manager lifecycle/receipts, HTTP task projections, MCP envelope encoding, cached health/artifact ownership, memory durable integration and limited listing, perception dispatch/snapshots, logs, assigned checkpoints, dense bounds and saved display. Tests use synthetic data and temporary test fixtures, without paid models or real robots.

An additional bounded cross-boundary probe used one real TaskManager through idle → running → succeeded. Health active count, HTTP task list and capability task list agreed at every stable state; the updater refused active work. This probe is not an additional pytest case and is not added to the 593 count.

## Reproduction

From the repository root:

```sh
source ../neh-toolchain/env.sh
PYTHONPATH="$PWD/scripts:$PWD" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -B -m pytest -q \
  scripts/check_task_summary_reads.py scripts/check_task_active_index.py \
  scripts/check_mcp_response_encoding.py scripts/check_audio_health_cache.py \
  scripts/check_artifact_runtime_integration.py scripts/check_memory_durable_integration.py \
  scripts/check_memory_scene_graph_revisions.py scripts/check_memory_limited_scene_listing.py \
  scripts/check_perception_scene_dispatch.py scripts/check_log_index_hashfree.py \
  scripts/check_log_index_sqlite_compat.py scripts/check_python38_runtime_compatibility.py \
  scripts/check_self_update.py::ServiceTests scripts/check_task_receipt_evidence.py \
  scripts/check_task_history_cleanup.py scripts/check_task_call_identity.py \
  tests/test_assigned_exploration_checkpoint_store.py tests/test_runtime_assigned_save_preparation.py \
  tests/test_assigned_cloud_exchange.py tests/test_dense_archive_bounds.py \
  tests/test_scene_graph_image_sampling.py tests/unit/test_yolo_detection_contract.py \
  tests/test_perception_snapshot_v18.py tests/test_assigned_saved_cloud_sources.py \
  tests/test_assigned_saved_display_gateway.py tests/integration/test_runtime_input_bridge.py \
  tests/test_checkpoint_identity_v2.py tests/test_assigned_asset_storage.py -k 'not abrupt'
```

The scripts directory must be on PYTHONPATH because some existing script modules import sibling scripts by bare name. The initial collection without it failed on two imports and ran no tests; those collection errors are not product failures or passes.

Additional checks:

```sh
bash scripts/ci/no_ros2_residual.sh
bash scripts/check_layering.sh
python -B scripts/ci/check_pep604.py services/geometry/__init__.py \
  services/geometry/loop_closure_state.py services/geometry/runtime.py \
  services/geometry/external_debug_observer.py gateways scripts/deploy_installer install.py
python -B scripts/check_python38_runtime_compatibility.py
```

These checks passed, including 8 standalone Python compatibility tests. The workflow's tracked fixture/mock/integration ignore-policy check also passed: git check-ignore returned 1 with zero ignored tracked paths. Its command was `git ls-files -z -- tests/fixtures/ tests/mocks/ tests/integration/`, feeding `git check-ignore --no-index -z --stdin`.

## Approved fixture corrections

1. `tests/test_assigned_saved_cloud_sources.py`: removed a patch of the nonexistent `dense_cloud.payload_digest` attribute. Existing hashlib algorithm/new/file_digest prohibitions and the actual checkpoint_payload.payload_digest prohibition remain. No create=True mock or substitute hashing was introduced. All 21 previously setup-blocked saved-display tests now execute and pass.
2. `scripts/check_python38_runtime_compatibility.py`: terminal fixture supplies `request.body_key='assigned_cloud_v2'`, matching the existing terminal contract. Exact payload and enum assertions remain unchanged. The formerly missing field generated 24 subtest errors; the corrected standalone run passes all 8 tests.
3. `tests/test_assigned_cloud_exchange.py`: unsupported backend rejection expects the existing `unsupported assigned checkpoint backend` message, preserving the same exception type and refusal operation.

Before these corrections, the original 19-target slice reported 396 passed / 25 failed. A separate lifecycle/checkpoint slice reported 143 passed / 1 failed / 1 deselected. The final run includes additional targets; these counts are not directly comparable and must not be added together.

## Five initial failures and exact causes

- `RuntimeAssignedSavePreparationTests.test_v2_rejects_missing_context_before_or_after_export`: expects the obsolete phrase `cloud backend`. The existing runtime rejects missing context with `assigned checkpoint export requires an active native backend` (or backend drift after export). Rejection remains active.
- `RuntimeAssignedSavePreparationTests.test_v2_rejects_noncloud_backend_before_export`: its first input is `slam_toolbox`, which is now explicitly supported alongside voxel and raytracing. The synthetic exporter returns voxel, so the existing runtime rejects backend drift after export, not every planar backend before export. The test's blanket no-export assumption is stale.
- `AssignedCheckpointExportOriginTests.test_export_rejects_planar_or_unknown_reply_before_download`: turns a cloud response into `slam_toolbox` without supplying graph_artifact. Existing native planar admission rejects the missing required member before downloading; the old cloud-only message does not match.
- `LegacySavedCloudTests.test_legacy_dense_keeps_manifest_reload_and_digest_epoch`: fixture lacks required activation_generation, producing KeyError before reading. It also expects a newly computed body-digest epoch. Existing production instead uses `saved-legacy:<backend>:<retained manifest alias>:<activation generation>` and does not recompute a body hash.
- `AssignedAssetStorageTests.test_assigned_finalize_rejects_legacy_current_before_hash_or_body_read`: expects an old legacy current directory to prevent first assigned publication. Existing production publishes the separately owned assigned asset instead. This pre-existing semantic expectation remains unchanged.

These causes predate the reviewed performance changes: the runtime save-preparation, provider terminal/export logic and relevant failing fixture bodies are unchanged from the recovered baseline `65d15c5`; the baseline dense reader already lacks payload_digest and already requires activation_generation. The recent dense change only replaces axis-bound reductions. No production behavior was modified to satisfy stale tests.

## Limits and evidence

The abrupt-fork checkpoint test is explicitly deselected. No real UDS qualification, hardware/model calls, full deployment build, full repository pytest run or end-to-end running gateway was performed. MCP checks establish exact envelope/body fidelity and error/budget behavior, not every live capability. A deprecation warning from httpx raw upload syntax remains.

Evidence is retained in the workspace sibling directory `../neh-evidence/overnight-python-integration-20261013/`, including final-combined.log, guardrails.log, python38-final.log, cross-boundary.log and its check_cross_boundary.py probe. Test fixtures may create ephemeral temporary directories internally; retained reports and logs live in the workspace evidence directory.


## Approved canonical-contract alignment and rerun

No production files were changed. Rather than relaxing admission or ownership behavior to fit the historical expectations:

- The legacy dense fixture now supplies the actual activation generation and checks the retained manifest alias/generation epoch directly. It verifies one-load continuation, unchanged payload, reinstall reload and rejection of the previous epoch. A prior body-hash calculation was removed. Both this fixture and `test_dense_legacy_epoch.py` now guard actual hashlib algorithms and checkpoint_payload.payload_digest; the latter no longer uses a create=True spy on a nonexistent adapter.
- The assigned publication test checks the current independent namespace contract: exact assigned identity and state, separate assigned-id directory, explicit reopen after repository restart, exact occupancy payload, and unchanged legacy directory inode, manifest bytes, membership and default selection. Existing no-hashing, no-legacy-load and no-whole-body-clone guards remain. The checkpoint projection runner's matching selector was renamed too; all 60 selectors were verified to resolve without executing that runner's source hashing.
- Save preparation now separately tests unsupported/missing backend refusal before export, supported planar-context/cloud-export mismatch after exactly one export, and context disappearance before versus after export. All assert no staged files survive.
- Provider tests distinguish a planar reply missing graph_artifact from an unknown backend. The unknown-backend test additionally prohibits entry into envelope validation, proving the supported-backend gate rejects it before parsing members or transferring bytes. The production gate is explicit in `export_mapping_backend_checkpoint`; this is not an inferred error-string contract.

Focused reproduction:

```sh
source ../neh-toolchain/env.sh
PYTHONPATH="$PWD/scripts:$PWD" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -B -m pytest -q \
  tests/test_dense_legacy_epoch.py tests/test_assigned_saved_cloud_sources.py \
  tests/test_assigned_asset_storage.py tests/test_runtime_assigned_save_preparation.py
```

Result: **63 passed in 1.90 seconds**. Then repeat the full command above, adding `tests/test_dense_legacy_epoch.py` before `-k 'not abrupt'`: **609 passed, 1 deselected, 1 warning in 41.03 seconds**. The count increase from 598 executed cases to 609 is nine legacy-epoch cases plus two newly split rejection tests; no earlier run counts are summed. All five initial failures are resolved by canonical test expectations, with no production edits. The abrupt-fork exclusion and all other scope limits remain.

Workspace evidence: `canonical-focused.log` and `canonical-combined.log` in the same evidence directory. `git diff --check` passes for these changes.
