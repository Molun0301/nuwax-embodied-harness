# Perception overlay dispatch isolation qualification

Qualified on 2026-10-07 against local base `eefd52a329b7976038697a4b1047a23a76692c62`.
This is an uncommitted, CPU-only component change. No hardware, GPU, external model,
motion, deployment, commit or push was performed.

## Change and ownership

`services/perception/service.py` reserves `video.detections` publications through
the existing `StreamBus.prepare_publication` under the detector/tracker ordering
lock. It calls `dispatch_prepared` after releasing both perception pipeline locks.
`scripts/check_perception_dispatch_isolation.py` provides the offline regressions.
There are no new locks, queues, threads or image copies.

The public processing operation and any input-artifact hold remain active through
callback dispatch. New publications retain synchronous completion semantics.
Fast cached calls return the original context without republishing. The existing
full-versus-fast freshness check, overlay payload and cursor ordering are retained.

The full-model lock still serializes full processing; the detector/tracker lock
still serializes shared detector and tracker state. The inference worker's separate
prediction lock still owns its single reused shared-memory image buffer and the
matching response-queue read. Removing that lock would permit image overwrite or
response misattribution and was outside this change.

## Evidence

Before editing production code, both event-gated fast/full callback tests failed:
a second inference could not reserve its overlay while the first subscriber was
paused. The same cases pass with the candidate. This establishes a removed blocking
dependency, not a measured model or end-to-end speedup.

- New component suite: 11/11 passed; 20 repeated suites: 220/220 passed
- Existing producer/artifact-lifetime suite: 30/30 passed
- Existing StreamBus preparation/cancellation suite: 12/12 passed
- Independent read-only review approved; independently reran 11 and 12 checks
- New script Ruff clean; service retains the same 14 baseline Ruff diagnostics
- Both changed Python files pass Python 3.8 AST and PEP604 checks
- `git diff --check` passes

Cases cover blocked-callback inference and threshold-update progress, second full
pipeline completion through its model stages, same-thread fast/full callback
reentry, deliberately inverted dispatcher arrival with FIFO delivery, no cached
republication, stale-full suppression, failed preparation, ordinary callback
exceptions, callback `BaseException`, and close during an artifact-backed callback.

Reproduce with the project's dependency-equipped Python environment:

```
python scripts/check_perception_dispatch_isolation.py
python scripts/check_artifact_producer_lifetimes.py
python scripts/check_stream_bus_cancellation.py
python scripts/ci/check_pep604.py services/perception/service.py scripts/check_perception_dispatch_isolation.py
```

## Exact remaining limits

- Synchronous publishers still wait for subscriber completion. Subsequent inference
  can progress; the test does not claim all public calls return immediately
- `perception.scene_ready` EventBus callbacks still run inside the full-pipeline
  lock. A slow callback can delay another full request, and full-processing reentry
  from that callback can deadlock. This existing behavior remains out of scope
- StreamBus uses one shared dispatch queue. Another publisher can adopt prepared
  overlay work, including a logging publisher reached while a model lock is held.
  This change removes the direct overlay-dispatch lock hold, not every possible
  callback/model-lock interaction across publishers
- Cached perception/status reads already used separate short state locks. Task
  cancellation does not take these perception locks; this change adds no
  per-request interruption of running model inference
- Validation is synthetic component evidence. Hardware throughput, application
  performance and real model execution were not measured
