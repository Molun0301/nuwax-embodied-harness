# Inference process recovery ownership

Fixed-vocabulary engine inference with `use_worker_process=True` keeps that
requested mode after failures. It never silently loads a heavy local model in
place of its process worker. A failed or starting worker produces a transient
`DetectionBundle` with `availability: unavailable` / `stage_status: not_ready`.

One detector-owned lifecycle thread starts, waits for readiness, reaps and
restarts one worker generation. Callers do not wait for model bootstrap or reap a
failed process; an already-ready prediction still uses the existing synchronous
request/response boundary and SHM serialization lock. Request and response queue
capacity remains 2. No additional frame backlog or image hash is introduced.

Recovery retries and idle liveness checks use the existing
`perception.stream_runtime.failure_backoff_sec` setting. Model startup still uses
the worker's existing readiness deadline. At most one old generation can own
process/IPC resources; failure to reap retains that owner and prevents spawning a
replacement. Late results cannot update a replacement generation. Threshold
updates apply to both the current worker and the next generation.

`worker_recovery` result metadata exposes state, failure reason, start attempts,
worker ownership and lifecycle-thread liveness. `use_worker_process` remains the
requested policy; `supports_worker_process` reports whether the selected backend
actually supports it.

`PerceptionService.close` fences new work and closes detector lifecycle owners
before releasing dependencies. A timeout returns false while in-flight work or
unreaped resources remain owned. Close during startup/backoff stops retries;
close during inference fences its late result and waits for safe cleanup. Close
is terminal: a later start does not create another owner.

## Execution boundaries and limits

HTTP capability calls, MCP calls and agent requests already offload synchronous
handlers to a threadpool. Background perception and prewarm have their own
threads. Earlier parent-PID probes proved loss of process isolation; they did not
prove event-loop blocking or task starvation.

Explicit `use_worker_process=False` retains existing in-process behavior on
those executors. Dynamic-vocabulary `.pt` inference still uses its existing
in-process path and reports `supports_worker_process: false`. Moving mutable
vocabularies into the child requires a separate protocol change. This phase does
not claim that support or actual GPU/model quality.

## Offline verification

```sh
PYTHONDONTWRITEBYTECODE=1 python scripts/check_inference_recovery.py
PYTHONDONTWRITEBYTECODE=1 python scripts/check_artifact_producer_lifetimes.py
```

The focused fixture combines deterministic fake lifecycle workers with real
multiprocessing spawn, queues and shared memory using a temporary fake local
model module. It covers repeated startup failure/recovery, child exit, timeout,
concurrent callers, retry pacing, stale-generation results, unreaped-generation
retention, threshold continuity, public detector status and service shutdown.
No real model, GPU, camera, microphone, robot or network is used.
