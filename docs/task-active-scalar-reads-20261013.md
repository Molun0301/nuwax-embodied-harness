# Active task scalar reads, 2026-10-13

## Change and boundaries

Health summaries now read count_active_tasks(); update admission reads has_active_tasks(). Both hold the existing task-manager RLock and use the same nonterminal predicate as list_active_tasks(). Neither copies public status objects. Boolean queries stop at the first active record.

Public list ownership, ordering, quarantine classification, retained records, receipt generation, cleanup policy and lifecycle writes are unchanged. Terminal quarantined tasks remain excluded from the active predicate, exactly as before. No active index is included in this stage.

## Offline measurement

Run: `source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh; python scripts/measure_task_active_reads.py`

Python 3.8.10; 11 timed repetitions per method; actual TaskManager methods and unique TaskRecord/TaskStatus objects. Active records follow terminal records to expose worst-case boolean scanning. A separate instrumented pass counts actual record visits; timings use an ordinary dictionary with an RLock wrapper measuring acquisition-to-release hold time. Synthetic tasks do not execute runners, model calls or robot I/O. Full baseline and scalar-stage data: `task-active-scalar-results-20261013.json`.

Same-run medians below separate copying savings from retained-history scan cost. Milliseconds are environment-specific diagnostics, not test thresholds.

| Terminal | Active | list ms | count ms | bool ms | count lock ms | count visits | bool visits |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 200 | 0 | 0.0148 | 0.0137 | 0.0157 | 0.0130 | 200 | 200 |
| 200 | 5 | 0.0857 | 0.0142 | 0.0171 | 0.0136 | 205 | 201 |
| 200 | 50 | 0.6716 | 0.0180 | 0.0158 | 0.0173 | 250 | 201 |
| 5000 | 0 | 0.5069 | 0.4668 | 0.5275 | 0.4641 | 5000 | 5000 |
| 5000 | 5 | 0.5646 | 0.4595 | 0.5062 | 0.4579 | 5005 | 5001 |
| 5000 | 50 | 1.6600 | 0.5421 | 0.5360 | 0.5392 | 5050 | 5001 |
| 50000 | 0 | 18.7663 | 19.8482 | 20.5801 | 19.8276 | 50000 | 50000 |
| 50000 | 5 | 20.0656 | 20.3300 | 19.8917 | 20.3086 | 50005 | 50001 |
| 50000 | 50 | 20.1067 | 19.7358 | 21.3041 | 19.7173 | 50050 | 50001 |

At 200 retained terminal records and 50 active tasks, the count avoids approximately 0.65 ms of copying. At 50,000 retained terminal records, scanning still holds the task lock for approximately 20 ms even with zero active tasks. This stage therefore does not claim history-independent active-query latency. A separately reviewed active index would be needed to remove that scan.

## Verification

- `scripts/check_task_summary_reads.py`: 22 passed, including scalar no-copy, all task states, quarantined terminal exclusion, boolean short circuit, real running/terminal transitions and concurrent locked transition
- `scripts/check_task_history_cleanup.py`: 20 passed
- `scripts/check_audio_health_cache.py`: 11 passed
- `scripts/check_self_update.py`: 55 passed
- `scripts/check_artifact_runtime_integration.py`: 16/17 passed; unrelated pre-existing teardown fixture fails because its zero-argument `_close_mapping_recording` lambda receives the current timeout argument (line 253). The task-manager health double is updated; teardown fixture was unchanged in this stage. A later isolated test-only compatibility fix accepts the timeout and returns True; all 17 then pass
- Existing health test doubles updated to implement the scalar TaskManager methods
- `git diff --check`: passed

No full-project, hardware, live model or deployment verification is claimed.
