# Recent-obstacle ingestion baseline — 2026-10-13

## Result

No production optimization was applied. Three new ordinary contract tests passed; the one ignored bounded attribution diagnostic passed in 2.79 seconds. The baseline supports **not pursuing the XYZ iterator/error-drain refactor yet**: at the admitted 100,000-point maximum, separate XYZ materialization costs 0.820–0.846 ms and the finite scan 0.143–0.155 ms. A simpler two-pass parser removes its 2.4 MB temporary but would decode coordinates twice; no candidate was implemented or measured.

Full `RecentSourceState::consume` medians at 100,000 points range from 5.572–6.001 ms for repeated columns to 35.059–35.867 ms for dispersed columns. The separately executed original recent-frame builder costs 4.504–4.757 ms and 20.566–20.662 ms respectively. These measurements identify a larger construction lead, **not its internal cause or a proven optimization**. Do not infer allocator or destruction cost by subtracting separate probes.

## Exact scope

- The parent supplied source `79bb33f` when assigning this audit. The worker did not read or write Git; this is not an independently verified build commit. Its only source edits are the new test module and its `cfg(test)` declaration
- Real production `RecentSourceState::consume`, codec, `NavScheduler` input/ACK and `SurfaceFollower`; existing `RecordingMotion`, no active goal, no `PlannerWorker`, no LCM/socket, sensor, actuator or physical robot. The diagnostic asserts no motion-port calls before shutdown
- Release build of only `nuwax-nav-stack --lib`, Rust 1.88.0, original workspace release profile, `--locked --offline -j2`, features `mls,guard-runtime,surface-recovery-qualification`. Build completed in 1m31s; no clean, full-workspace build or profile change
- One fixed run: 1k/10k/100k points × 64 repeated versus dispersed XY columns × zero retention versus eight-frame target. Each cohort has eight warmups and seven measured samples. Separate stages alternate before/after the complete call
- Points are generated outside timing, one quarter marked ground with the remaining points non-ground. Encoding and fixture creation are outside timing. Source timestamps are acquired afresh before each synthetic packet; all 15 inputs per cohort must remain within the existing five-second retention window
- Full timing ends after scheduler ACK and state/watermark update. Stage attribution uses the same encoded body but executes separately with its own retained snapshot. Stage boundaries are nonoverlapping; old stage-map retirement, scheduler work and ACK are outside those stage intervals. The stage totals are not a decomposition of the full-call samples
- XYZ payload estimates use `N * size_of::<Point3>()`, not allocator counters, retained bytes or RSS. No field deployment distribution, input rate, hard-real-time bound or system CPU benefit is established

## Baseline medians

All columns below are milliseconds. The four rightmost columns are **separate-stage** measurements.

| Points | Distribution | Frame target | Full consume | Codec decode | XYZ materialize | Finite scan | Recent-frame build |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1000 | 64 repeated columns | 1 | 0.061 | 0.003 | 0.008 | 0.001 | 0.047 |
| 1000 | 64 repeated columns | 8 | 0.061 | 0.003 | 0.008 | 0.001 | 0.046 |
| 1000 | dispersed | 1 | 0.165 | 0.014 | 0.008 | 0.001 | 0.127 |
| 1000 | dispersed | 8 | 0.171 | 0.013 | 0.008 | 0.001 | 0.123 |
| 10000 | 64 repeated columns | 1 | 0.542 | 0.011 | 0.092 | 0.014 | 0.414 |
| 10000 | 64 repeated columns | 8 | 0.539 | 0.011 | 0.085 | 0.014 | 0.409 |
| 10000 | dispersed | 1 | 1.957 | 0.013 | 0.079 | 0.014 | 1.514 |
| 10000 | dispersed | 8 | 2.231 | 0.013 | 0.079 | 0.014 | 1.446 |
| 100000 | 64 repeated columns | 1 | 5.572 | 0.126 | 0.820 | 0.143 | 4.504 |
| 100000 | 64 repeated columns | 8 | 6.001 | 0.137 | 0.846 | 0.143 | 4.757 |
| 100000 | dispersed | 1 | 35.059 | 0.174 | 0.830 | 0.148 | 20.566 |
| 100000 | dispersed | 8 | 35.867 | 0.182 | 0.841 | 0.155 | 20.662 |

Compact medians/ranges and precise scope are in [results JSON](recent-ingestion-baseline-20261013-results.json). Raw sample JSON and build/test logs remain in the execution workspace `recent-ingestion-audit-20261013`; they are not evidence of a remote publication. No content hashes were computed.

## Source/lock audit

- `surface_dataflow.rs` launches only one SurfaceGeometryCache build at a time; its expensive cache lock is used by that build closure, not by the input/cancel event loop. No demonstrated concurrent waiter is helped by removing that mutex
- `PlanningOwner` already computes and retires replaced payloads outside its exchange lock
- `PlannerWorker` already runs geometry and path derivation outside its input lock. Residual boundary/display copying and bounded delta merging exist, but this audit does not establish a high-value lock redesign
- `lcm_dataflow.rs` handles recent sources inside the guarded-input event handler. Its synchronous decode/build work delays the next event intake in that handler; it does not directly lock the independent scheduler/worker threads

## Contracts exercised

`recent_ingestion_diagnostics.rs` adds three ordinary tests:

1. Legal packed f32 and mixed f32/f64 layouts, both endian modes, reordered/extra fields, per-point padding and multiple contiguous rows; original ground member indices, obstacle-overrides-ground for a shared voxel, and exact output comparison against the original builder using decoded scalar values
2. Replay/source-generation rollback, domain/frame mismatch, wire truncation, count mismatch, row padding rejection and field extent rejection; missing z before unsupported x type; early voxel-range error followed by late NaN still reports invalid admitted points; NaN/Inf and range rejection leave the accepted snapshot, watermark and enabled state unchanged
3. Single-frame and eight-frame retention through a ninth publication, then source-generation reset. The no-cloud clear branch is unchanged and is not claimed by the new tests

Current codec already enforces positive and exact dimensions, emitted count, contiguous rows, data length, field extents and header/source identity. It does not require XYZ fields or reject nonfinite coordinate values. Runtime selects the first matching named XYZ fields and supports float32/float64 with declared offsets/endianness. No reusable borrowed XYZ iterator was found. These supported layouts must not be silently narrowed.

## Commands

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
cargo test --manifest-path services/nuwax-workspace/Cargo.toml \
  --release --locked --offline -j2 -p nuwax-nav-stack --lib \
  --features mls,guard-runtime,surface-recovery-qualification --no-run --message-format=json
./stack-test-binary recent_source::ingestion_diagnostics:: --nocapture --test-threads=1
./stack-test-binary \
  lcm_dataflow::surface_dataflow::recent_source::ingestion_diagnostics::baseline_complete_recent_ingestion_cost \
  --exact --ignored --nocapture --test-threads=1
```

## Open construction lead, not implemented

The original `surface/recent.rs` inserts every voxel into `BTreeMap<XY, Vec<Z>>`, records ground/obstacle membership in original input order, then sorts/deduplicates each column. A possible test-only candidate is to collect owned voxel keys, sort/deduplicate them, group by XY, and construct the same BTreeMap while leaving ground point ordering, errors and retention unchanged.

This still needs one Vec per distinct XY, adds a flat 12N-byte voxel buffer, and can regress short or highly repeated inputs. Rust 1.88 `BTreeMap::from_iter` also collects and stable-sorts grouped entries before bulk construction; see the [pinned official source](https://github.com/rust-lang/rust/blob/1.88.0/library/alloc/src/collections/btree/map.rs#L2157-L2170). No internal allocation count or speedup is claimed. Any candidate needs exact internal-column/ground-point equality, malformed-input ordering checks and paired repeated/dispersed comparisons before production consideration.
