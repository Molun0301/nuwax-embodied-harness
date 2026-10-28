# Active task index, 2026-10-13

## Scope and lifecycle audit

The scalar-helper stage showed approximately 20 ms of lock-held retained-record scanning at 50,000 terminal tasks. This stage adds one insertion-ordered dictionary of active TaskRecord references, using the existing RLock. Only list_active_tasks, capture_task_list, count_active_tasks and has_active_tasks consume it. Public copies remain outside the lock. No task history, result, receipt or resource-quarantine evidence is deleted.

Membership owners:

- submit registration: add the same accepted record to both dictionaries under the existing lock
- submit rollback after successful resource cleanup: remove from both dictionaries; a failed cleanup retains the accepted recovery owner in both
- _set_terminal_state: remove immediately after replacing status, before event construction, publication or resource callbacks; repeated terminal calls do not reopen membership

Gated execution and _set_running only change ACCEPTED to RUNNING. Progress, metadata, interrupt request and barrier acknowledgement writes preserve task state. Quarantine changes and late releases never reopen terminal status. Explicit history cleanup only removes reclaimable terminal records. There is no TaskManager restore/reopen API or other production registration path. Capability pause/resume creates new tasks through submit. cancel_all_active_tasks and quarantine scanning are deliberately unchanged.

The existing TaskManager owns task-state writes; direct mutation of its private record table is not an alternate production API. Synthetic summary and measurement fixtures now establish both private tables consistently; lifecycle tests use real submit/transition paths and compare index identity/order with a fresh nonterminal projection of retained records. No new defensive checks, background cleanup or synchronization primitives were added.

## Measurement

Command: `source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh; python scripts/measure_task_active_reads.py`

Python 3.8.10; 11 repeats per method, 200/5,000/50,000 terminal records × 0/5/50 active. Real public methods, real TaskRecord/TaskStatus objects, separate dictionary-scan/key-lookup/history-visit instrumentation and timed ordinary dictionaries. capture_task_list performs exactly 20 history visits and 20 keyed record lookups; list/count/bool perform none. Scalar methods visit no active entries either. Final fixtures use deterministic unique suffixes and assert exact fixture cardinality; one initial random-ID run was discarded after a collision reduced the 50-active fixture to 49. No production task-ID behavior changed.

Full data is in task-active-index-results-20261013.json; pre-index list/capture and scalar-helper measurements are in task-active-scalar-results-20261013.json. Baseline record-visit counts confirm their stated cardinalities.

| Terminal | Active | baseline list ms | indexed list ms | indexed list lock ms | count ms | count lock ms | active visits |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 200 | 0 | 0.0147 | 0.0023 | 0.000631 | 0.000811 | 0.000270 | 0 |
| 200 | 5 | 0.0864 | 0.0702 | 0.000981 | 0.000831 | 0.000271 | 5 |
| 200 | 50 | 0.7117 | 0.6287 | 0.003605 | 0.000821 | 0.000270 | 50 |
| 5000 | 0 | 0.6102 | 0.0023 | 0.000631 | 0.000832 | 0.000261 | 0 |
| 5000 | 5 | 0.5711 | 0.0700 | 0.001002 | 0.000821 | 0.000270 | 5 |
| 5000 | 50 | 1.1977 | 0.6137 | 0.003465 | 0.000821 | 0.000271 | 50 |
| 50000 | 0 | 19.5506 | 0.0024 | 0.000621 | 0.000892 | 0.000301 | 0 |
| 50000 | 5 | 20.3515 | 0.0704 | 0.000992 | 0.000842 | 0.000271 | 5 |
| 50000 | 50 | 22.4505 | 0.6645 | 0.004046 | 0.000811 | 0.000260 | 50 |

At 50,000 terminal records, active list selection visits only 0/5/50 active records and holds the lock for about 0.00062/0.00099/0.00405 ms. Count and boolean queries visit neither table and hold the lock for about 0.00026–0.00031 ms. capture_task_list still selects its requested bounded history tail and copies requested summaries; at 50 active its measured total is 3.11 ms versus 25.56 ms baseline. Public status copying remains proportional to the number of active statuses returned. Timings are diagnostic and environment-specific, not hard performance test thresholds.

## Verification

Python 3.8.10 verification (404 checks passed):

- check_task_active_index: 9; real registration/future window, pending cancellation, normal completion/order, rollback success/failure, deferred cancel/quarantine, timeout/late result, terminal-dispatch exception, no retained-record scanning, owned summary payload and concurrent completion/list copying
- check_task_summary_reads: 22
- check_task_history_cleanup: 20
- check_task_receipt_evidence: 35
- check_task_call_identity: 12
- check_event_bus_contracts: 32
- check_audio_health_cache: 11
- check_artifact_runtime_integration: 17
- check_self_update: 55
- check_agent_wait_status_reads: 16
- check_agent_terminal_receipts: 21
- check_agent_terminal_recovery: 20
- check_navigation_failure_messages: 3
- check_navigation_activation_control: 24
- pytest tests/unit/test_manipulation_http_tasks.py tests/unit/test_scene_mapping_service.py: 107
- git diff --check: passed

The existing pinned jsonschema==4.19.2 and websockets==10.4 requirements were installed into the workspace Python 3.8.10 environment to enable the integration imports; no requirement files changed. The artifact-runtime suite exposed a baseline-existing teardown test double with the old zero-argument/nonboolean _close_mapping_recording contract. Its failure was reproduced using the pre-scalar-commit script; the isolated one-line test-only fix accepts the timeout and returns True. Runtime shutdown behavior was not changed.

The manipulation tests use their synthetic loopback HTTP server; no real robot, motion or model endpoint was called. No full-project or deployment verification is claimed.
