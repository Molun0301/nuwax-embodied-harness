# Stream and event delivery audit, 2026-10-13

Scope: existing checkout and branch, read-only production source audit. No production edits, Git actions, hashes, application hashing tests, devices, runtime construction, or network use. Benchmark fixture and raw output are retained under the workspace evidence directory stream-delivery-20261013; selected results are in docs/diagnostics/stream-delivery-audit-20261013-results.json.

## Concrete findings

1. StreamBus.prepare_publication and EventBus.prepare only shallow-copy envelopes with the assigned cursor. They retain payload references. No whole payload serialization or deep copy occurs under publication locks. StreamBus replay snapshots scan retained references under the lock, bounded by configured retention; no evidence justifies a more complicated replay index now.
2. Both buses serialize synchronous callbacks to preserve accepted publication order/completion. Both have a single async consumer; a blocking asynchronous subscriber can delay another, explicitly documented by EventBus. Actual WS/SSE stream callbacks hand off to bounded queues instead of performing encoding/network I/O. Gateway event subscriptions likewise enqueue via ClientQueue. Replacing bus dispatch with thread-per-subscriber would materially expand scheduling/shutdown semantics without a demonstrated production bottleneck.
3. ClientQueue and _ClientStreamQueue already bound payloads and coalesce cross-thread wakeups. Stream queue replacement and priority selection are indexed; no per-frame full queue copy remains. Slow socket sends are awaited per connection, so they do not block another connection's socket owner, though the shared event loop still executes some encoders.
4. Events SSE and WS each run build_public_runtime_event per client in the threadpool, followed by their own transport serialization. Projection includes bounded redaction and final size enforcement. This repeated work is real; removing redaction or size checks would change external safety/API behavior.
5. Localization trajectory, pose and frame-tree use native Pydantic JSON encoding on the event loop. Encoding is repeated for every WS client. The realistic default trajectory is 100 poses (core/settings.py localization_history_limit), not an unbounded history. The probe measures that source-derived size and 1/4/8 client CPU batch costs, not network or actual event-loop delay.
6. Generic identity-based memoization is not sound: normal StreamFrame and RuntimeEvent remain mutable, and frozen Pose still contains mutable Vector3/Quaternion. The probe verifies mutation is visible on the next current encoding. Existing InstanceSnapshotFrame explicitly transfers ownership; its specialized cache is justified by a stronger contract. Canonical point clouds already have shared encoding and packed maps already reuse envelopes.
7. Packed map envelope miss concatenation happens under _MAP_ENCODE_LOCK, but hits are O(1) reference delivery and the retained cache is bounded by bytes and entries. Moving the copy out of this lock could duplicate simultaneous misses; no observed contention justifies a single-flight state machine here.

## Measurement limits and stopping condition

Use actual _encode_ws_frame and build_public_runtime_event on fixed synthetic canonical contracts. Primary single-client results, 4/8 concurrent projection and sequential event-loop encoder fanout only as scaling context. Direct projection CPU excludes its final SSE/WS JSON serialization; trajectory cost includes wire JSON encoding. Threadpool batch includes the actual run_in_threadpool operation used by the gateway, but is not a full ASGI/socket benchmark. No event-loop-lag claim. If baseline is modest and no ownership-preserving narrow optimization appears, stop without speculative production changes.

## Source references

- core/stream_bus.py: prepare_publication, _drain_dispatch_queue, _replay_snapshot_locked, _async_consumer_loop
- core/event_bus.py: prepare, _deliver_publication, subscribe, _async_consumer_loop
- gateways/ws/events.py: sse_events, websocket_events
- gateways/ws/data_streams.py: _ClientStreamQueue, _pack_map_envelope, _encode_ws_frame, _send_ws_frame
- gateways/ws/client_queue.py: ClientQueue
- gateways/ws/instance_encoding.py: InstanceSnapshotEncoder
- contracts/streams.py: StreamFrame, InstanceSnapshotFrame
- contracts/geometry.py: Pose, Vector3, Quaternion
- services/localization/service.py: _record_snapshot_locked, _publish_snapshot
- core/settings.py: RuntimeDataConfig.localization_history_limit

## Measured result

Bounded run completed at approximately 21:35:35 UTC, Python 3.8.10, without concurrent authorized timing work. 60 direct batches/case and 30 threadpool batches/case. These are small synthetic microbenchmarks on the cloud host, not device or end-to-end latency claims; no production traffic distribution is established.

| Case | Clients | Direct median / p95 ms | Threadpool median / p95 ms |
| --- | ---: | ---: | ---: |
| 100-pose trajectory WS encoding | 1 | 0.219 / 0.357 | n/a |
| 100-pose trajectory WS encoding | 4 | 0.857 / 1.017 | n/a |
| 100-pose trajectory WS encoding | 8 | 1.717 / 2.054 | n/a |
| Task progress public projection | 1 | 0.357 / 0.421 | 0.653 / 1.022 |
| Task progress public projection | 4 | 1.389 / 1.918 | 2.571 / 2.958 |
| Task progress public projection | 8 | 2.943 / 4.949 | 5.030 / 5.996 |
| Synthetic 32-item robot public projection | 1 | 2.170 / 2.517 | 2.644 / 3.002 |
| Synthetic 32-item robot public projection | 4 | 8.995 / 10.331 | 10.798 / 12.078 |
| Synthetic 32-item robot public projection | 8 | 17.788 / 19.347 | 21.503 / 26.588 |

Trajectory encoded size is 15,106 UTF-8 bytes. Projected event serialized sizes measured after projection were 653 and 1,483 bytes; transport wrapping not included. Warm-up is the initial untimed direct call; initial threadpool worker startup is included in first batch and affects maxima. Mutation checks passed for both trajectory nested vector and event payload.

Recommendation: no production change. Default single-client trajectory encoding is small. Public event projection deserves attention only if actual high-rate/fanout telemetry is established, and then requires an explicit immutable public projection ownership boundary rather than generic memoization of mutable events. Preserve redaction, bounded projection, replay/gaps and callback ordering. No additional measurements or speculative rewrites are needed for this audit.

## Reproduction

With the project Python 3.8.10 environment active, run from the checkout root:

```sh
python scripts/diagnostics/stream_delivery_probe.py > delivery-results.json
```

Optional positive integer arguments are `--direct-batches` (default 60) and `--threadpool-batches` (default 30). The 1/4/8 client cases and 100-pose fixture are fixed. The probe constructs fixtures and runs measurements only inside its main guard; importing it does not run the benchmark. The committed probe was refactored from the measured evidence script to add CLI arguments and import safety, then AST-parsed without a timing rerun. Existing result JSON remains the original measured output.
