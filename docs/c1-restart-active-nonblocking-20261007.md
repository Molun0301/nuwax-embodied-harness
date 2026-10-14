# Restart active-profile I/O off the event loop

Date: 2026-10-07. Baseline: `9bce2b99cc29d9d5a1392c15d9c508135ac2ed86`.

The async `/api/config/restart` route called filesystem-backed `_is_active`
directly, between already offloaded configuration operations. It now awaits
the existing `run_in_threadpool` for this one operation. No executor, lock,
validation condition, retry or background restart behavior is added.

The direct thread-pool primitive is intentional: the adjacent `call_settings`
wrapper also reclassifies exceptions. This change preserves the original active
probe's exception propagation rather than introducing that behavior change.

## Qualification

- Six new offline ASGI tests pass. The real `_is_active` method uses a gated
  test-owned filesystem probe; an independent lightweight HTTP request completes
  on the same event loop while that probe remains blocked in a worker thread.
- The fixed baseline fails this concurrency test because the probe runs on the
  event-loop thread. Its other five contract tests still pass.
- Success receipt/order, inactive-profile 409, absent-location bypass, original
  exception identity and revision-conflict/no-queue behavior remain covered.
- Twenty-eight existing configuration-revision tests pass.

The test's independent release thread has a timeout solely to let the failing
baseline terminate; this is not a latency benchmark. All restart queue/dispatch
effects are fixtures. No service restart, user configuration write, network
request or running server is needed.

Run `scripts/check_restart_active_nonblocking.py` in the existing project Python
environment. Its optional `--source` path checks a captured historical API file
without replacing the working tree.

This fixes one infrequent configuration-restart I/O blocking point. It does not
claim faster memory queries, general thread-pool isolation or whole-gateway
latency improvements; unrelated requests can still contend for the existing
bounded shared pool.
