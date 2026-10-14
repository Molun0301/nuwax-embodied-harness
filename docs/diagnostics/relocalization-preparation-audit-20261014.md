# Relocalization synchronous preparation audit — 2026-10-14

## Result and scope

The actual source-sealing helper has measurable synchronous work before registration is sent to `spawn_blocking`. In the declared **4,000,000-point synthetic target capacity cohort**, full preparation took median **95.93–100.13 ms** across the three source sizes. The 100k target cohorts took **1.60–2.34 ms**, and the 1M target cohorts **5.03–13.70 ms**. These measurements justify looking at large-target preparation; they do not establish urgency, typical field latency or a need for new concurrency.

**No production behavior changed.** The only source change is a cfg(test) include plus an ignored diagnostic that invokes the actual private helpers and real admitted owners. No ICP, map activation RPC, actor queue-latency measurement, physical robot, persistent cache, extra worker or production offload was added. The earlier [candidate-pair reuse audit](relocalization-pair-reuse-audit-20261014.md) remains a separate negative result.

## Exact active path and limits

`lcm_runtime.rs:11440–11455` obtains the existing mapping-lifecycle guard for an active-mode cloud and sends `submit_with_lifecycle_guard`. The replay path does likewise (`:11674–11689`). `correction_coordinator.rs:8994` uses the guard's presence as the cutoff; **there is no mandatory 50k-point gate**. The unused-as-production-gate 50k constant and historical name `MinimumAggregateReached` must not override the actual branch. Source policy bytes begin their numeric bound list with zero (`:3532`; matching Python policy in `contracts/map_relocalization.py`).

| Boundary | Existing value | Measurement interpretation |
| --- | ---: | --- |
| Per-frame source | 100,000 points / 2,400,000 f64 payload bytes | 50k and 100k cohorts each use one valid guarded frame |
| Accepted frames | 8 | unchanged; three frames used in the aggregate-bound cohort |
| Aggregate source | 250,000 points / 6,000,000 f64 payload bytes | 100k + 100k + 50k is a supported helper-bound control, not ordinary immediate guarded ingress |
| Emitted source | at most 100,000 points / 2,400,000 payload bytes | existing deterministic stratified reduction, unchanged |
| Source wire envelope | 25,600,000 bounded bytes | existing validation, no wider frame format |
| Canonical target | 120 MiB = 125,829,120 XYZ-f32 bytes | exactly 10,485,760 points at the existing maximum |
| Decoded theoretical target payload | 240 MiB = 251,658,240 f64 bytes | theoretical limit only; not allocated for this audit |

Actual target cohorts were 100k, 1M and 4M finite points. The latter two match sizes already used in prior activation/dense-admission diagnostics. Their coordinates are a literal binary-representable lattice; these are explicitly synthetic capacity inputs, **not inferred or guessed user maps**. The environment reported approximately 9.0 GB available memory before building; the theoretical target maximum was not exercised.

## What runs synchronously

The active actor performs the following before `spawn_blocking` (`correction_coordinator.rs:9008–9046`):

1. Require the original cutoff lifecycle guard
2. Sort the owned accepted-frame list by the existing complete frame key
3. Validate the boundary's admitted owner and exact match with the latest accepted frame
4. Validate aggregate byte bounds, transform every body point by that frame's captured `odom_from_body`, check transformed coordinates and exact aggregate count
5. Apply the existing index rule `floor(i × aggregate_count / emitted_count)` if reduction is required
6. Decode every target coordinate from canonical little-endian f32 to f64, retaining finite/count validation
7. Convert source and target Vecs into the existing `Arc<[Point3<f64>]>` owners
8. Produce existing source receipts, clone Arc handles and capture anchor/registration metadata for computation

The source transformation is body→odometry. Anchor/registration transforms are consumed later by the unchanged compute function; preparation does not apply them twice.

`CorrectionPoints` owns a private `Arc` around the admitted Vec with cached finiteness; metadata clones do not clone its point payload. The current boundary and accepted frame share those owners, so source ownership checks use their exact identity fast paths. `CanonicalCloudTargetV2` owns private immutable canonical bytes through an Arc. The snapshot, source owners and existing owned lifecycle lease can all remain owned while work executes elsewhere; immutability alone does not settle actor-state/cancellation semantics.

## Measurements

Each target size ran in its own fresh process, with source order 50k / 100k / 250k. Each cohort has **2 warmups and 5 recorded samples**. Target admission, real source admission, expected arrays, boundary metadata clones and lease acquisition occur outside timers. All nine cohorts completed, yielding 45 recorded samples.

Full preparation calls the unmodified `seal_relocalization_source`, then the existing source-receipt projection and compute metadata/Arc-handle capture. It excludes earlier actor ledger/admission checks, actor scheduling, registration, and output verification. The timer adds a few clock reads for its nested measurements; it is not a transport or complete-runtime timing.

| Target points | Source points / frames | Full preparation median | Recorded range |
| ---: | ---: | ---: | ---: |
| 100,000 | 50,000 / 1 | 1.600 ms | 1.486–2.612 ms |
| 100,000 | 100,000 / 1 | 1.972 ms | 1.651–2.238 ms |
| 100,000 | 250,000 / 3 | 2.339 ms | 1.890–3.084 ms |
| 1,000,000 | 50,000 / 1 | 5.027 ms | 4.752–5.226 ms |
| 1,000,000 | 100,000 / 1 | 5.893 ms | 5.621–6.244 ms |
| 1,000,000 | 250,000 / 3 | 13.705 ms | 13.387–14.238 ms |
| 4,000,000 | 50,000 / 1 | 95.933 ms | 93.541–99.593 ms |
| 4,000,000 | 100,000 / 1 | 100.133 ms | 96.240–115.233 ms |
| 4,000,000 | 250,000 / 3 | 98.315 ms | 98.032–111.353 ms |

Independent stage samples call the actual decode/reduction helpers and actual Arc conversion. A separate test helper copies the original transformation loop exactly; full preparation always uses the actual production sealing function, including its ownership checks. In the 4M target cohort, independent decode medians were **37.01–39.27 ms**, and target Vec→Arc-slice handoff **58.27–59.81 ms**. Source transform medians were about **0.21 ms / 0.42 ms / 1.05 ms** at 50k / 100k / 250k respectively. Reduction was near zero when the Vec could move unchanged, and about **0.87 ms** at 250k for that target process. Post-seal receipt/handle work was only several microseconds.

**These independent timers are not additive attribution and are not subtracted from the full timer.** For example, some 1M independent decode/Arc samples are slower than the full preparation timer because execution/allocation history differs. This audit does not infer exact allocator or page-fault causes from that discrepancy. All original values, nested timers and process order are retained in [structured results](relocalization-preparation-audit-20261014-results.json).

## Lifetime and memory

Observed Vec capacities in the separate stage calls:

- Transformed source: 1,200,000 / 2,400,000 / 6,000,000 bytes at 50k / 100k / 250k
- Reduced source: 1,200,000 / 2,400,000 / **3,145,728 capacity bytes** respectively; the 250k input emits 100k points, and its final Arc slice has 2,400,000 payload bytes
- Decoded target Vec: exactly 2,400,000 / 24,000,000 / 96,000,000 capacity bytes at 100k / 1M / 4M
- Corresponding target Arc slices have those same payload lengths; canonical byte-slice lengths are 1,200,000 / 12,000,000 / 48,000,000

Vec→Arc-slice conversion creates the Arc allocation and copies elements before the Vec allocation is released. A large target therefore has a transient interval containing the decoded Vec and Arc payload, while the immutable canonical bytes remain retained. These values describe known arrays; they exclude headers, allocator rounding, canonical Vec spare capacity, source owner buffers, receipt structures and other process state. No atomic whole-process live-payload peak was measured.

Linux process VmHWM observed across all source cohorts was **38,596 / 96,160 / 272,368 KiB** for the 100k / 1M / 4M target processes. Those are process high-water observations, including fixture/admission/verification and allocator behavior, not an isolated preparation allocation peak. VmRSS and prior HWM are preserved per sample; neither is interchangeable with capacity × element size. Teardown timing releases the measured output and lease while fixture source/target owners remain alive, so it is not a complete production-owner destruction cost.

## Equality, errors and lease checks

Every successful sample checks all emitted source coordinate bits against sorted per-frame transforms and the exact stratified index rule, and every target coordinate bit against canonical f32→f64 decoding. It checks original source Arc identity, original checkpoint ownership, handle sharing, retained lifecycle lock while output is alive, and explicit release afterward. Inputs are deliberately presented to the helper in reverse frame-list order to exercise its original sorting.

Each cohort also checks the original errors for missing lifecycle guard, declared aggregate-count mismatch, aggregate 250,001-point byte-bound rejection, and boundary-owner mismatch. No invalid data is made valid or injected into an immutable owner. The two existing tests `relocalization_source_seal_uses_each_frame_odom_and_retains_boundary_guard` and `relocalization_source_hard_bounds_fail_closed` passed unchanged.

Test accounting: **2 existing tests passed**, plus **one new ignored diagnostic explicitly executed in three target-cohort processes**. The three invocations are not three new independent tests. No entire native suite or ignored suite was run.

## Actor and cancellation interpretation

`run_relocalization_runtime_loop` cannot return to its select loop while `handle_frame` is sealing. Consequently this actor delays terminal queries, new/joined admissions, waiter-detach cancellation handling, shutdown handling, target polling, frame receipt and completion-event handling. The target poll is nominally 10 ms, but no dropped-tick or queue-delay count was measured. Other Geometry tasks/RPCs can run on other runtime workers; this is not a whole-process freeze claim.

The cutoff frame's reply is sent before sealing, but the existing lifecycle lease remains owned through registration and canonical finalization. The producer next tries to reacquire that mapping lock. An offload would not itself release that lock or allow a waiting map activation to proceed.

Cancellation and deadlines constrain a later design:

- A dropped command waiter sends a detach event; only actor handling can durably settle its cancellation. The helper itself does not poll command cancellation
- Current selection takes accumulation out of `execution`, but that temporary absence is unobservable to other actor messages because the call is synchronous. An asynchronous preparation must preserve Busy/single-active ownership before yielding
- Logical/accumulation deadlines apply while accumulating. After computation handoff, the select loop does not impose a new deadline. Moving preparation must not silently change that boundary
- Current Computing state retains source receipts used to report cancellation/stop evidence. Those receipts are currently produced after successful sealing; a worker-side seal can fail before that evidence exists. Error precedence and evidence phase need explicit preservation
- The original lease must survive preparation/compute/cancellation retention and reach the unchanged finalization/release boundary. Immutable inputs make movement possible, but they do not justify early lease release or returning incomplete source evidence

No production concurrency recommendation is adopted by this report. The bounded large-target result is a reason to compare a small ownership-preserving option, not to introduce a new queue/cache or claim that spawn_blocking automatically improves activation time.

## Build, reproducibility and retained mistakes

Fixture: [`relocalization_preparation_probe.rs`](../../services/geometry/nuwax_geometry_runtime/src/relocalization_preparation_probe.rs), included only inside the existing cfg(test) coordinator tests module. Actual build:

```sh
cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --release --locked --offline -j2 --bin nuwax-geometry-runtime --no-run --message-format=json
```

It uses the existing Geometry release profile: opt-level 3, full LTO, one codegen unit, no override. The correct binary build passed in **511.944 s**. Its Cargo JSON artifact confirms test=true, opt_level=3 and default/real-lcm-bridge features. The executable was copied to a frozen local artifact before test selection and execution; no content hash was calculated.

- Initial launcher `/usr/bin/time` was unavailable (exit 127); Cargo did not start for that attempt. Python monotonic timing replaced only the launcher
- Initial `--lib` selection compiled dependencies in **51.152 s** but did not contain `correction_coordinator`; a subsequent fresh lib invocation took **0.241 s**. Artifact inspection caught the wrong target before tests were claimed. Only the later binary artifact ran the fixture
- Before the correct binary build, fixture-only boundary/lease clones were moved outside the timer. No production helper changed
- Existing exact tests took **0.012 / 0.016 s** whole-process wall time. The explicit 100k / 1M / 4M diagnostic processes took **0.130 / 0.685 / 4.785 s**; these include setup/verification and are not preparation latencies
- Local evidence directory: `relocalization-preparation-audit-20261014`, beside the checkout. It retains generated Cargo logs/artifact metadata, frozen executable/test source, process logs and launcher/build history. Repository structured evidence contains all 45 samples and complete process/build metadata

No Git operation, manual hash, profile change, paid model, user-computer access or production algorithm/safety change was performed by this worker. Source thresholds, scoring, heading acceptance, point limits, frame/point order and target freshness checks are unchanged.

## Read-only next-step assessment

Moving sealing into the existing single compute closure is ownership-feasible, but not a mechanical scheduling edit: the actor currently installs `Computing { source_receipts, ... }` only after a successful seal. Before yielding, an offload would need to retain single-active/Busy state; cancellation, shutdown and a preparation error would need an explicit receipt/evidence phase and unchanged error precedence. Leaving the actor as Accumulating would also change which deadline/target-poll branches run. No such production change is recommended from this measurement alone.

A smaller alternative deserves a bounded A/B first: the private `SealedRelocalizationSource::target_points` currently uses `Arc<[Point3<f64>]>`. Its only production uses are cloning a handle for the existing compute closure and reading a slice. Storing `Arc<Vec<Point3<f64>>>` via `Arc::new(target_points)` could retain the decoded Vec allocation instead of copying into an Arc slice. All observed target Vec capacities equal their point counts, so this target-only proposal does not inherit the reduced source Vec's spare-capacity tradeoff. It would preserve the current actor, lease, cutoff and error order. This is a read-only proposal, **not an implemented or measured candidate**; the independent handoff timer cannot be subtracted to promise a full-preparation gain. Any candidate still needs complete-helper A/B, pointer/owner and error equivalence, and relevant actor regressions under the same existing release profile.
