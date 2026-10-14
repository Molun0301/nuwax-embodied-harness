# Owned occupancy payload encoding qualification

Date: 2026-10-07. Source baseline: `9b1190ca8054f9b58acc07975fde4827a5d808a8`.

## Change and reachability

`GetOccupancyGridData` and `GetRawOccupancyGridData` already own a cloned
`MapState`. Consume its `Vec<i8>` when producing `Vec<u8>` instead of borrowing
the vector and collecting a second allocation. The helper uses safe Rust:
`data.into_iter().map(|value| value as u8).collect()`.

The original snapshot clone, validation, metadata, revision wait and error
ordering remain unchanged. No additional checks, locks, cache, hash, unsafe
conversion or transport changes are introduced. Borrowed checkpoint encoding
and metadata-only RPCs are outside this change.

The effective-grid RPC is reachable through an unbound unary map-snapshot
fallback, including activation waiting without a matching receipt-bound owner.
The raw-grid RPC is reachable through legacy/planar map finalization. Normal
bound push-stream reads and assigned 3D map saving bypass these paths. This is
not a steady point-cloud streaming optimization.

## Evidence

- Rust 1.88.0, x86_64: standalone opt-level 0 and 3 probes retain the input
  allocation pointer, capacity, length and exact bytes in eight cases: empty,
  allocated empty, singleton, all 256 bit patterns, spare capacity and 1/4/16 MiB
  buffers. Optimized probe assembly transfers the vector descriptor without a
  per-byte loop. This is a tested compiler/target result, not a Rust API guarantee
  for all toolchains or architectures.
- Incremental geometry test build passed using the existing native SDK cache.
  Geometry profile: opt-level 0, debug assertions and overflow checks enabled;
  existing MLS/runtime dependency opt-level 3 overrides were retained. Features
  `default` and `real-lcm-bridge`. This is not a production release benchmark.
- Seven new tests passed: one allocation/byte-pattern test and six explicitly
  in-process RPC tests. The latter compare frozen old RPC bodies, complete
  metadata and output, original source isolation, malformed-input/error ordering,
  captured-revision waiting while shared state advances, and missing descriptors.
- Two existing Python cached/unary regressions passed:
  `test_cached_getter_reuses_bytes_and_isolates_mutable_shells` and
  `test_unbound_unary_cannot_replace_stream_owner`.
- Six unchanged real-LCM tests were attempted separately. All failed at fixture
  construction with missing multicast route / `Network is unreachable` /
  `lcm_create` failure, before their assertions. The in-process fixtures do not
  substitute for these integration tests, and no network settings were changed.

## Limits and reproduction

No whole-application latency, CPU, ARM/Go2 or real-LCM integration improvement
is claimed. The shared map still needs its original snapshot clone; the
optimization removes only the subsequent owned payload conversion allocation
on the qualified toolchain. Serialization and network transport remain.

Run the geometry test binary with
`server::owned_occupancy_bytes_tests:: --test-threads=1` after an incremental
build. The original integration selectors remain under `occupancy_rpc_tests`
and `lifecycle_tests::start_mapping_deferred_style_commit_returns_aligned_data_and_descriptor`.
Keep their outcomes separate from the new in-process module.

The deferred relocalization/ground/support/clearance project is not modified.
