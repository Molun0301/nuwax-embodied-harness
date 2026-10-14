# Relocalization target handoff qualification — 2026-10-14

## Result

The two-line private target-owner change removes the decoded target's Vec→Arc-slice coordinate copy. In the paired actual-helper diagnostic, the 1M-target/100k-source median decreased from **6.080 to 2.738 ms**; the 4M synthetic target/100k-source median decreased from **99.732 to 38.284 ms**. All three paired process medians improved in these cohorts. The 100k-target/250k-source aggregate control is mixed: **1.976584→1.996333 ms**, including one **1.156155 candidate/baseline ratio**. This is not a universal speedup claim.

All **19 exact functional tests** and **18 diagnostic process invocations** passed. The non-cfg production binary-target check also passed. The original [preparation baseline](relocalization-preparation-audit-20261014.md) and [negative pair-reuse audit](relocalization-pair-reuse-audit-20261014.md) remain independent and unchanged by this qualification.

## Production scope and consumers

Only two production lines change in `correction_coordinator.rs`:

- `SealedRelocalizationSource.target_points`: `Arc<[Point3<f64>]>` → `Arc<Vec<Point3<f64>>>`
- Successful seal construction: `Arc::from(target_points)` → `Arc::new(target_points)`

The source remains `Arc<[Point3<f64>]>`. The new regression include is cfg(test)-only. The sealed struct is private, with no public ABI, repr/serialization or FFI layout contract. Its only production consumer clones the target Arc for the existing compute closure; `target_points.as_ref()` coerces to the same borrowed point slice at `run_relocalization_compute`. No algorithm or target values change. There is no new cache or extended request lifetime, concurrency, worker, guard, source-owner change, scoring/heading change or error-policy change.

## Reproducible inputs and method

For point index i, the literal finite lattice is `(i % 1024, (i / 1024) % 1024, i / (1024*1024)) * 0.125`, with integer division. Canonical targets contain these exact f32 XYZ values in little-endian order, built with the actual assigned-target factory and decoded by the production helper. These are declared synthetic capacity cohorts, not guessed field maps.

Sources are 50k and 100k single frames, plus the explicit 250k helper-bound control containing 100k+100k+50k frames. Each frame starts with the same lattice prefix. Frame j=1..3 uses identity rotation and translation `(j,-2*j,0.5)` and passes through actual source admission. Expected emitted coordinates are checked against admitted body arrays transformed by each captured odometry pose. Accepted frames are deliberately reversed; the actual helper sorts their complete keys. Existing reduction selects `floor(i*aggregate/emitted)` and emits at most 100k points. The 250k control is not normal immediate guarded ingress; no mandatory 50k production gate is implied.

The original baseline executable was frozen before editing; the original preparation probe remains byte-for-byte equal to its frozen copy. Both binaries use the existing Rust 1.88.0 release profile, opt-level=3, full LTO and codegen-units=1, default/real-lcm-bridge features. Three fixed fresh-process pairs run in **AB, BA, AB** order for each target. Each invocation runs source order 50k/100k/250k with two warmups and five samples each: **18 processes, 270 recorded samples, 135 per arm**. No rerun or sample is discarded.

Inputs, expected arrays, lease acquisition and boundary clones are outside timers. Full preparation measures the actual seal plus receipt projection, Arc handoff and captured compute metadata. It excludes ICP, actor scheduling, earlier ledger checks and output verification. Independent decode/transform/reduction/old Arc-slice stage timers remain unchanged for identical profiles; they are **not added or subtracted to attribute the full timer**.

## Full-preparation A/B results

Each headline is the median of three per-process medians. Ratios list the three actual paired candidate/baseline process medians.

| Target points | Source points | Baseline ms | Candidate ms | Paired ratios |
| ---: | ---: | ---: | ---: | --- |
| 100,000 | 50,000 | 1.570100 | 1.084438 | 0.703297 / 0.670647 / 0.647287 |
| 100,000 | 100,000 | 1.829131 | 0.742291 | 0.429198 / 0.402720 / 0.402087 |
| 100,000 | 250,000 | 1.976584 | 1.996333 | 1.156155 / 0.993564 / 0.991346 |
| 1,000,000 | 50,000 | 5.336256 | 2.474926 | 0.463794 / 0.445052 / 0.443412 |
| 1,000,000 | 100,000 | 6.079899 | 2.737593 | 0.427640 / 0.473309 / 0.450269 |
| 1,000,000 | 250,000 | 14.311506 | 4.230036 | 0.301699 / 0.284915 / 0.295569 |
| 4,000,000 | 50,000 | 97.453965 | 37.023635 | 0.384162 / 0.363872 / 0.388891 |
| 4,000,000 | 100,000 | 99.732179 | 38.283897 | 0.406057 / 0.351353 / 0.383867 |
| 4,000,000 | 250,000 | 99.202714 | 40.141183 | 0.443591 / 0.383362 / 0.397242 |

The small-target aggregate control's higher aggregate median is 19.749 microseconds, with one 15.6% paired regression and two slight improvements. It remains visible rather than being dismissed as noise. Large-target improvements are repeatable within this bounded synthetic experiment; these measurements do not establish typical robot, map-activation or complete relocalization latency.

## Contents, allocation and last-reader evidence

Every diagnostic sample checks every emitted source coordinate bit and every decoded target coordinate bit, including source order and reduction. It checks original admitted source identity, checkpoint ownership, Arc sharing, retained lifecycle lease and explicit release. Existing missing-guard, aggregate-count, over-bound and boundary-owner errors remain exercised.

The new test uses target counts 3 and 100k and explicitly includes `(-0.0,0.0,1.0)`. Direct production decode followed by the identical `Arc::new(decoded)` expression preserves the point data pointer, capacity and signed-zero bits. Clones share the allocation; a weak reference expires only after the last strong reader drops. Separately, the actual sealing helper is exercised with real admitted owners and a held lifecycle lease. The actual field type, target capacity and all coordinate bits are checked. The last target reader remains valid after the sealed context drops and after the original lease explicitly releases; source/checkpoint ownership and canonical target data remain unchanged. This complementary proof does not claim instrumentation of the pre-Arc pointer inside the full seal.

Target decoded capacities equal counts in all measured cohorts: **2,400,000 / 24,000,000 / 96,000,000 bytes**. The corresponding canonical byte-slice lengths are 1,200,000 / 12,000,000 / 48,000,000. Moving the Vec into Arc metadata retains its existing coordinate allocation and removes the old full-size copy plus the transient overlapping replacement coordinate buffer. Final coordinate payload length is unchanged; small Vec/Arc metadata remains. No longer-lived cache is introduced.

These payload capacities are not RSS or an atomic whole-process allocation peak. The unchanged diagnostic independently runs the original Vec→Arc-slice stage in **both arms**, so its process VmRSS/VmHWM includes that large allocation, fixture admission/verification and allocator behavior. Raw RSS fields are retained but cannot establish isolated candidate RSS savings. No allocator-call count or atomic whole-process peak was measured. The source's 250k reduction has 3,145,728 Vec capacity bytes for 2,400,000 emitted payload bytes; source representation is intentionally unchanged.

## Qualification and build accounting

The actual Geometry coordinator is compiled in the binary target, not its library. Candidate test build command: `cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --release --locked --offline -j2 --bin nuwax-geometry-runtime --no-run --message-format=json`. It passed in **511.637 seconds** using the existing profile/cache. No candidate compile or qualification failure occurred.

Final non-cfg check: `cargo check --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --release --locked --offline -j2 --bin nuwax-geometry-runtime --message-format=json`; passed in **30.952 seconds**. This checks production types/compilation; it is not a fully linked deployed-robot qualification.

The 19 exact functional tests below passed unchanged except the new ownership test. They are separate invocations totaling **1.090 seconds** of process wall time. The 18 explicit ignored-diagnostic invocations totaled **28.884 seconds**, including setup, assertions and independent stages; this is not full-preparation runtime. No entire test suite was claimed.

- `relocalization_target_vec_handoff_retains_allocation_and_last_reader`
- `relocalization_source_seal_uses_each_frame_odom_and_retains_boundary_guard`
- `relocalization_source_hard_bounds_fail_closed`
- `relocalization_local_consensus_rejects_sparse_wrong_heading`
- `relocalization_local_consensus_accepts_supported_seed`
- `relocalization_local_consensus_preserves_nonzero_anchor`
- `relocalization_local_consensus_rejects_tilted_estimate`
- `relocalization_local_consensus_accepts_sparse_and_partial_view`
- `relocalization_local_consensus_reuses_registration_without_accumulating_correction`
- `relocalization_runtime_claims_once_and_settles_canonical_apply`
- `relocalization_runtime_algorithm_reject_releases_cutoff_guard`
- `relocalization_phase_a_joins_late_blocking_compute_before_guard_release`
- `relocalization_runtime_without_guard_keeps_first_deadline`
- `relocalization_timer_preempts_simultaneously_ready_cutoff_frame`
- `relocalization_canonical_rejection_is_typed_and_sequence_continues`
- `relocalization_target_rollover_is_typed_preclaim_terminal`
- `relocalization_session_rollover_is_typed_preclaim_terminal`
- `relocalization_runtime_duplicate_is_ignored_and_same_key_conflict_fail_closes`
- `relocalization_compute_panic_stores_terminal_before_owner_failure`

The panic fixture intentionally panics inside compute and passes its existing terminal-before-owner-failure contract. The existing wrong-heading, supported-seed, tilt, anchor and repeated-registration tests remain unchanged. Passing them preserves covered behavior; it does not diagnose or fix the reported field heading reversal.

## Actor interpretation and provenance

Sealing still runs synchronously in the same actor and retains the same lease. Removing this copy shortens the measured synchronous section for the larger tested targets, so it reduces that section's opportunity to delay actor queries/admissions/cancel handling. Actual queue latency was not measured. It does not release the mapping lifecycle lock early or independently unblock map activation. Busy/admission/deadline/receipt/finalization and cancellation contracts remain on the original flow. No concurrency refactor is proposed.

Baseline checkpoint: `7a1ce35`. Frozen executable and literal prechange source are retained in `/workspace/scratch/71cdb9d17732/relocalization-target-handoff-audit-20261014`; the original baseline evidence is in `/workspace/scratch/71cdb9d17732/relocalization-preparation-audit-20261014`. Candidate build/check artifacts, all 37 qualification process logs, both frozen executables and full sample rows are retained there. [Structured results](relocalization-target-handoff-qualification-20261014-results.json) embed all 270 sample rows, process commands/exits/durations, profile artifacts, input formulas, ownership evidence and limitations. No manual content hash was created.
