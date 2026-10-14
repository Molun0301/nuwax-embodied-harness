# Deterministic incremental point-cloud replay

Date: 2026-10-13 UTC. Production code unchanged. Test: [check_incremental_pointcloud_replay.mjs](../../scripts/check_incremental_pointcloud_replay.mjs). Machine-readable [results](incremental-pointcloud-replay-20261013-results.json).

## Result

`node scripts/check_incremental_pointcloud_replay.mjs` passed (exit 0) against the real JavaScript connection, packet parser/assembler, Three.js objects and incremental source-page layer. No browser, renderer, WebGL, network, device or GPU was involved. These are correctness/resource-count properties, not FPS, latency, heap/GC or hardware-performance measurements.

- 230 accepted transactions, 12 reconnects, 24 deliberately injected faults producing expected resyncs
- Maximum 36,878 points, 37 source pages, 3 render slabs
- Maximum incomplete accepted transaction payload: 393,216 bytes / 32 chunks
- Maximum source-array allocation: 589,824 bytes (3 slabs × 16,384 points × 12 bytes)
- 3,322,224 copied coordinates and 3,483,399 scanned coordinates across the entire replay, including full resets
- 21,020,052 scheduled source upload bytes across the entire replay; this is production accounting, not observed GPU traffic
- 97 geometries created, 97 disposed exactly once; both materials disposed exactly once; zero retained source slabs, pending transfer, accepted assembler base or timers after disposal

The final source was rerun after adding exact scanned-coordinate assertions. Raw command output is retained in the task evidence directory under `incremental-pointcloud-replay/replay.log` and its exit status in `replay.exit`.

## Scope and oracle

A fixed PRNG seed (`0x19a742d`) selects point edits. The independent reference is a plain flat Float32Array manipulated by append, direct replacement, dense-slot swap-delete, and tail truncation. It deliberately makes full copies inside the test only. Production modules receive wire-format v2 packets with at most four pages per packet, not oracle arrays. Every accepted transaction is compared page-by-page with the independent reference, and assembler acknowledgement and point count are checked after complete acceptance.

Twelve bounded rounds exercise incremental appends, replacements, swap-delete compaction, count-only truncation, generation/layout reset, an interrupted multipacket transfer, reconnect resume metadata, late old-socket message/close/error events, deliberately out-of-order packets, stale-generation reset rejection, and recovery by a valid full reset. A final empty transaction and idempotent disposal close resource ownership.

This is the mutable **live** v2 protocol. Assigned saved assets require immutable complete resets, so they are intentionally not used as mutable replay fixtures. Packet schema is parsed before normal delivery. Reordered delivery is explicit fault injection: ordered WebSockets do not normally reorder messages, and the expected resync is not a production bug. A reconnect permits a fresh producer baseline; the test does not misclassify that documented behavior as stale acceptance. Old-socket events are checked at the real connection's socket/epoch guard.

## Incremental and retention assertions

- Parser chunk arrays remain views of their original packet buffers, without assembly-time whole-cloud XYZ flattening
- Accepted source pages match every reference coordinate and share their slab backing array
- Surviving source page/slab object identities are stable for non-reset updates
- Copy count equals the payload's changed coordinates exactly; metadata scan count equals changed coordinates plus only a shortened surviving tail page when required
- Count-only truncation copies and schedules zero XYZ upload bytes
- Untouched slabs have no newly scheduled position update ranges; incremental scheduled bytes are bounded by changed coordinates plus fresh slab capacity
- Source page/slab/mesh counts match current point count; created-minus-disposed geometry count matches currently retained slabs
- Histogram total equals current point count, extent-tree nodes have a page/capacity bound, pending upload collections are empty after apply, and no derived resources are created by this height-style replay
- Incomplete packet payload/chunk counts remain bounded by that transaction, do not advance the accepted base, and are discarded on reconnect or resync

Only height-style source storage is covered here; existing style/voxel/derived-view checks remain separate. Object reachability counts do not prove native/browser garbage collection, and upload accounting does not measure graphics-driver behavior. The fake socket intentionally uses deterministic events and timers; it does not establish network throughput or browser event-loop performance. No production defect was found, so no production guard or optimization was added.
