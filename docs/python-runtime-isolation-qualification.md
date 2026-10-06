# Python runtime isolation and latest-state qualification

Fresh offline verification on 2026-10-04, using source baseline
`85f22475f837a19d266d6dfb37231d421d2d5dc1` plus the StateStore correction in
this commit. These results do not inherit historical archive test counts.

## Latest-state correction

Previously, writing `a@t+2`, `b@t+1`, then replacing `a@t` left
`read_latest()` returning the discarded `a@t+2`, although both `read(a)`
and `snapshot()` exposed the replacement. Observation capture uses this
cache for its robot-mode summary, so it could describe superseded state.

Only replacement of the cached latest key with a regressing timestamp now
recomputes the maximum over current entries, under the existing namespace
lock. A newly written entry wins an equal-timestamp tie, matching normal
writes. Tied surviving peers use stable namespace insertion order; the
store does not track their historical write order. Ordinary writes retain
the existing namespace copy-on-write cost with no additional scan, lock,
copy, counter or identity hash. `read_latest()` remains O(1).

Lock-free reads during an overlapping write retain their previous
old-or-new semantics. This change does not promise an atomic snapshot
across separate `read`, `read_latest` and `snapshot` calls.

## Reproduction and results

The same final 11-case `check_state_store_contracts.py` suite produced
8 failures / 3 passes on unmodified baseline source and 11 passes with the
correction. Coverage includes single-entry and multi-entry timestamp
regression, nonlatest writes, equal timestamps, namespace independence,
phased concurrent completed-write reads, and real ObservationService
capture using synthetic image bytes and temporary local artifact storage.

Ten existing suites passed on baseline (226 cases). After the correction,
all those suites, the 11 new regressions and 30 artifact producer lifetime
cases passed: **12 scripts, 267 passed cases, zero failures/errors/skips**.
None of the selected functional scripts was blocked or left unrun.

| Script suffix (`scripts/check_<suffix>.py`) | Cases |
| --- | ---: |
| event_bus_contracts | 32 |
| runtime_event_log_isolation | 24 |
| agent_wait_isolation | 27 |
| agent_wait_status_reads | 16 |
| agent_terminal_receipts | 21 |
| agent_terminal_store_split | 30 |
| agent_terminal_recovery | 20 |
| agent_terminal_rollout_policy | 17 |
| robot_state_revisions | 6 |
| task_receipt_evidence | 33 |
| artifact_producer_lifetimes | 30 |
| state_store_contracts | 11 |

Environment: Linux 6.18.44 x86_64, CPython 3.12.14, Pydantic 2.10.6,
FastAPI 0.115.14, SQLite 3.53.1. An existing Python environment was reused;
no packages, SDKs or native build caches were installed/copied.

Exact interpreter used:
`/workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env/bin/python`.
Worktree: `/workspace/scratch/71cdb9d17732/neh-python-runtime-qualification`.
For every suffix above, from that worktree:

```sh
PYTHONDONTWRITEBYTECODE=1 /workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env/bin/python scripts/check_<suffix>.py
```

`git diff --check` passed. Ruff passes for the new script. Combined Ruff
checking of the two Python files reports one pre-existing I001 unsorted
import block in `core/state_store.py`; running Ruff against `git show
85f22475:core/state_store.py` produces the same warning. Imports were not
reformatted as part of this correctness change.

## Isolation and recovery evidence

- The actual runtime event bridge with a gated logging handler lets the
  publisher, waiter, terminal consumer and unrelated publication finish
  while output remains blocked. Failed writer/gap-writer fixtures preserve
  progress and expose health counters. Count/byte bounds include in-flight
  work; saturation does not conceal live terminal state.
- Event retention expiry, replay gaps, bounded/coalesced queues,
  reentrancy, callback failure and bounded shutdown/drain are covered.
- Async wait uses in-process ASGI transport, including disconnect,
  cancellation, shutdown and notification coalescing. Real TaskManager
  checks verify that ignored progress does not repeatedly project receipts.
- Receipt persistence checks cover task/bus progress during blocked
  storage, bounded mailbox reconciliation, capped retry backoff, reuse of
  prepared content, late terminal evidence and no action replay.
- Real temporary SQLite files in DELETE and WAL modes verify split-store
  terminal writer versus intent writer isolation and combined-store
  contention. Missing/unavailable/wrong-pair targets remain explicit
  outages rather than stale-history fallback.
- Fifteen split-store child-process exit phases cover intent, pair
  publication, legacy seed and candidate commit/reopen. Previously known
  pending/unknown/recorded action authority is never reacquired.
- Rollout checks deliberately mock external build/network/lifecycle work
  and execute only an inert test-owned shell launcher/config target. They
  qualify storage creation/reader policy, not deployment integration.

## Limits

No permission failure occurred in these selected fixtures. No real robot,
camera, model/provider, paid model call, network MCP transport, native
build or performance benchmark was exercised. These process-exit fixtures
do not establish power-loss/media durability or production-filesystem
behavior. Full historical product rollback and post-C1 integrated runtime
requalification remain separate gates. No remote write was performed by
this qualification.
