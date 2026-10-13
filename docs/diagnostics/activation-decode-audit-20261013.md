# Assigned activation canonical decode: source audit and bounded diagnostic

## Decision and scope

One redundant native allocation/construction is demonstrated, but **unconditional
retention is not recommended**: multiple activation tasks can hold prepared sets
before obtaining transition ownership, without an established server-wide count
bound. **No production optimization is implemented in this diagnostic.**
The test compares the existing validation-plus-decode pair with a single decoder
result, not two complete activation implementations. It cannot establish an
end-to-end map-switch speedup, writer-lock latency reduction, or robot CPU result.

Changes here are a `cfg(test)` child module in `pointcloud.rs`, an explicitly
ignored diagnostic, a Python 3.8-compatible process runner and this evidence.
No protocol, cache, lock, receipt, collision, source or fence contract changed.

## Actual source path and frequency

The ordinary assigned path is `skills/map_workspace.py::activate_map` →
`skills/assigned_map_activation.py::_activate` →
`GeometryRuntime.dispatch_prepared_assigned_map` →
`GeometryRuntimeProvider.activate_assigned_platform_map` → native
`server.rs::activate_map_r1`.

For a new successful voxel/raytracing installation:

1. After typed checkpoint admission, `server.rs` calls
   `CloudMapShmWriter::validate_checkpoint` under `cloud_map_writer.lock()`
2. `pointcloud.rs::validate_checkpoint` calls `decode_checkpoint_voxels` and
   discards the returned `HashSet<VoxelKey>`
3. The activation subsequently prepares fine/dense archives, enters lifecycle
   admission, restores the backend and installs occupancy
4. `install_checkpoint_with_sources`, again under the writer lock, calls the
   same decoder for the same immutable canonical target, before installation

Each decoder allocates a set with `point_count` requested capacity, decodes all
XYZ coordinates, computes voxel keys, and checks size, finite/range validity,
SHM capacity, duplicates and nonempty geometry. This is **two complete set
constructions per successful cold activation**, not a per-frame cost or every
RPC: early terminal-replay paths return before this work. A later conflict can
also discard preflight work without installing it. Deployed map-size and
activation-frequency distributions were not measured.

Key source locations at audit time: `server.rs:2329–2340,3210–3228`,
`pointcloud.rs:1095–1102,1168–1209,1396–1456`.

## Constraints for any later implementation

- `CanonicalCloudTargetV2` privately owns canonical bytes inside an `Arc`;
  cloning retains that exact owner. Both `assigned_checkpoint` and
  `ActivationTargetSource::Assigned` keep it through the activation awaits
- A prepared set should belong to this one activation/target and be consumed
  once. It must not be added to the checkpoint interner or permanent receipt
  ledger, and must not be matched merely by a user-supplied ID
- `max_points` is initialized to `CM_MAX_POINTS`. The only production assignment
  after initialization copies it into private scan work; there is no public
  runtime setter. Tests lower it. Installation must still validate its actual
  writer capacity rather than assume the earlier writer is identical
- Preserve preflight malformed/duplicate/range rejection before backend
  mutation. Installation currently checks positive session generation, revision
  availability, geometry/capacity, occupancy receipt, then optional fine-cloud
  construction, before changing state. Reusing geometry must retain current
  receipt, revision, capacity, owner and publication checks in their original
  relative order
- Cloud `ActivateMap` spawns `activate_map_r1` in an existing Tokio task
  (`server.rs:6672`). Client RPC cancellation does not cancel that task. A local
  prepared set would survive until that task returns, including continued
  recovery after caller cancellation. Early return/task exit drops it. No new
  detached task or retention policy is justified
- `PreparedCloudAccumulator` is **not a drop-in replacement**: it owns dynamic
  filtering, ordering, fine/dense cloud state and history flags, and
  `install_prepared_accumulator` has different preconditions. Using it directly
  would also bypass the current no-fine reconstruction path. A later change
  should stay limited to the already-decoded keys and exact target ownership
- The set would remain live during archive/backend preparation. Its longer
  lifetime can increase real peak memory even though one construction is
  removed. The diagnostic below measures that lifetime cost; it does not
  establish the actual overlap with production archive allocations

No-fine installation was neither changed nor exercised by the helper diagnostic.
A production candidate would need that path, owner mismatch, cancellation/replay,
late capacity changes, malformed geometry, duplicate voxels, receipts, and
installation/publication regression coverage before acceptance.

### Concurrent preparation makes the retention tradeoff unfavorable without more evidence

The preflight and fine/dense preparation occur **before**
`mapping_transition.lock()`. The production transition timeout is 30 seconds;
it starts only after archive preparation, so it is not a bound on total prepared
set lifetime. The test build uses a separate 1-second transition timeout, but
this helper does not exercise either timeout.

The Python provider's `_begin_mapping_transition` rejects overlapping operations
within that provider. It is not a server-wide bound on independent clients or
direct native RPC callers. Native cloud `ActivateMap` spawns one task per call.

- LCM `serve_geometry_rpc` uses `RpcBroker::serve_many` with the default
  `RpcServerConfig`. `nuwax-rpc` permits **50 concurrent handlers per method**
  and then leaves requests queued. This limits ordinary active LCM handlers,
  not all possible native entry points. Fifty live sets at the measured 1M
  size would represent approximately **1.27 GiB** of additional set storage.
  This is a possible concurrency illustration, not an observed workload
- The production gRPC `Server::builder()` at `main.rs:2024` sets message sizes,
  but does not configure an application concurrency limit. The pinned tonic
  0.12 source initializes `concurrency_limit` and `max_concurrent_streams` to
  `None`. No global activation-task bound across callers/connections was found
- The canonical SHM maximum is **1,398,101 points** from 16 MiB / 12. That bounds
  each accepted cloud, not the number of simultaneous prepared sets. The
  measured 1M fixture stays below it

The present discard behavior lets preflight sets be freed sequentially under
the writer lock. Retention would let them accumulate outside that lock while
waiting for archive preparation and lifecycle ownership. Original canonical
payloads already have their own lifetime; that does not justify adding another
approximately 26 MiB of set storage per large pending request.

Simply moving preflight after transition ownership changes whether invalid
geometry is rejected immediately versus waiting/timing out, and extends
transition-lock ownership into preparation. Retaining the original preflight
and decoding again after acquiring the transition preserves ordering, but
does not remove the duplicate traversal. Neither change is silently acceptable.
The helper timings therefore do not justify production retention. No new
semaphore, cache or lifecycle framework was added to make the optimization fit.

### Minimal conditional design, not approved or implemented

If a later whole-activation study establishes a favorable and bounded lifetime,
the narrowly scoped design would touch three production files:

1. `activation_identity.rs`: a crate-private exact-owner comparison using
   `Arc::ptr_eq`; content equality or matching IDs are insufficient
2. `pointcloud.rs`: a private, non-Clone prepared-key value holding the exact
   `CanonicalCloudTargetV2` and decoded keys, constructed by the existing
   decoder and consumed once. Assigned installation would derive voxel size,
   point count and bytes from that retained owner rather than accepting
   independent scalar/body arguments. It would verify the expected exact owner
   and current writer capacity, and retain all final revision/receipt checks.
   The legacy entry and no-fine reconstruction behavior would remain unchanged
3. `server.rs`: carry that value only for the assigned branch and route only that
   branch through the typed installation entry. Early terminal replay would
   still return before preparation; all existing transition/ledger ordering
   would remain. No bool validation bypass or inter-request cache is needed

This design does **not** solve the concurrency/lifetime issue above and is not
recommended for implementation on the present evidence. The actual full
activation peak-memory and queue-wait tradeoff remains unmeasured.

## Other audit findings

- Explicit Python activation already carries `_AssignedActivationRead`, keeping
  one owned checkpoint through preparation. Current startup restore does too:
  `restore_active_workspace` passes `_for_activation=True` and propagates the
  returned owner. The October 9 retained-read report's exclusion of startup
  restore describes its historical version
- `validate_checkpoint_bundle` already short-circuits admitted immutable schema-2
  ownership; it does not rescan all bodies on each call
- Native fine then dense preparation has a real dependency: dense indexing uses
  the prepared fine archive's `voxel_size` when fine is present. Substituting the
  canonical cloud size to run both concurrently would change behavior
- State queries, durable journal steps and provider-origin checks bracket actual
  predecessor stop, backend dispatch and installation. No safe redundant wait
  was established there

## Diagnostic method

Build: Cargo/Rust 1.88.0, original release profile (`opt-level=3`, LTO, one codegen
unit), existing cached target, `--locked --offline -j2`. The incremental build
succeeded in **8m34s**, with existing warnings preserved in its log. No profile
override, clean build, legacy checksum test or broader suite was run.

Both existing map/archive no-hash sentinels are enabled by RAII **before fixture
construction** and remain active on the synchronous test thread. The synthetic
canonical target is admitted through `CanonicalCloudTargetV2::admit` using typed
literal checkpoint/member/receipt provenance, no payload digest. Its exact
12.5 cm voxel centers form a square grid, at most 125 m per side. Sizes are
1,000, 100,000 and 1,000,000 canonical points; the last stays within the current
1,398,101-point SHM capacity. This is canonical downsampled geometry, not a
claim about dense full-scan point counts.

Each size has **three separate process pairs**, with arm and size order rotated.
Each process records seven timed samples:

- Baseline: actual `writer.validate_checkpoint` followed by actual
  `decode_checkpoint_voxels`
- Retained opportunity: one actual `decode_checkpoint_voxels`, returning its set

All output keys are checked outside timing against the independently generated
grid, including count, coordinate bounds and linear index. An `Arc` clone keeps
the same canonical byte allocation. Each process lowers the writer capacity
below the valid target count afterward and verifies that both existing
validation and decoder reject with the same capacity error. **All 18 processes
passed.** These are repeated diagnostic executions, not 18 unique tests.

The single-decode arm does not implement future installation or its currentness
checks. Timings omit RPC, file/archive work, backend restoration, conversion to
`VoxelIndex`, fine construction, publication and locks. Results are a local
opportunity measurement, not production A/B results.

## Timing results

Each cell lists the median of seven decoder samples from each of three fresh
processes, in milliseconds.

| Canonical points | Existing validate + decode | Single retained decode |
| ---: | --- | --- |
| 1,000 | 0.033531 / 0.033911 / 0.033720 | 0.017336 / 0.017466 / 0.016886 |
| 100,000 | 5.498181 / 5.713605 / 5.113229 | 2.622199 / 2.728379 / 2.671083 |
| 1,000,000 | 137.300909 / 147.752408 / 141.969429 | 69.007408 / 65.157576 / 75.830747 |

Whole-process wall time includes launch, fixture generation/admission, memory
sampling, a separate lifetime illustration, seven timing samples and key checks.
It is **not one activation's latency**. At 1M points the baseline process wall
times were 1167.894 / 1266.877 / 1221.688 ms; retained-arm processes were
596.958 / 587.994 / 696.700 ms. The JSON preserves each process's wall, user and
system time, complete decoder samples and process maximum RSS.

## Allocation footprint and retained lifetime

No allocator override or allocation counter was installed. The source performs
two versus one `HashSet` constructions; that is not a measured global allocation
call count. Actual returned capacities were 1,792 / 114,688 / 1,835,008 slots for
the three sizes. Optional read-only GNU `mallinfo2`, `/proc/self/status` and
`wait4` maximum RSS are preserved. GNU in-use/mapped bytes are allocator
footprint, not exact live Rust object bytes or cumulative allocated bytes;
samples are not atomic and kernel RSS/HWM accounting can differ slightly.

For the first 1M-point pair:

| Phase | Baseline RSS KiB | Retained RSS KiB |
| --- | ---: | ---: |
| Immutable target and writer | 25,836 | 25,900 |
| After preflight | 25,912 | 52,604 |
| Arbitrary 8 MiB intervening scratch live | 34,104 | 60,800 |
| Decoder result and scratch live | 60,728 | 60,800 |
| Result and scratch dropped | 60,728 | 25,976 |

The retained arm's GNU mapped-byte increase from owner-only to after-preflight
was **27,267,072 bytes**. Baseline had already freed that set at the same point.
The retained lifetime cost is therefore material. Baseline later reused heap
storage while retained used a separate mapping, illustrating allocator behavior;
the final RSS difference is not a production memory-saving claim.

The 8 MiB scratch is explicitly **arbitrary lifetime illustration**, not an
actual fine/dense archive or backend-restore allocation, and not a prediction of
activation peak RSS. Similar maximum RSS in this toy overlap does not rule out
a larger production peak when the retained set overlaps real preparation work.
No trim, garbage collection intervention or cache clearing was used.

## Evidence and reproduction

Structured results: [activation-decode-audit-20261013-results.json](activation-decode-audit-20261013-results.json).
The diagnostic source is
`services/geometry/nuwax_geometry_runtime/src/activation_decode_diagnostic_tests.rs`.

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
CARGO_BUILD_JOBS=2 CMAKE_BUILD_PARALLEL_LEVEL=1 cargo test \
  --release --locked --offline -j2 \
  --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \
  --bin nuwax-geometry-runtime \
  pointcloud::activation_decode_diagnostic_tests::assigned_activation_decode_pair_diagnostic \
  --no-run
python scripts/measure_activation_decode.py TEST_BINARY OUTPUT_DIRECTORY
```

The runner invokes only the exact ignored diagnostic with `--test-threads=1`.
Original build, runner and all 18 per-process logs, plus the copied test binary
and original runner, remain in the workspace evidence directory
`neh-evidence/activation-decode-20261013/diagnostic-v1/`. No earlier log was
overwritten. Measurement completed at approximately 2026-10-13 23:48:53 UTC.
