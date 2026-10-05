# Dense observation-only source assembly attribution

Source baseline: observation-dispatch correction `8c52574bb442fd6864f052f0854f25a174ab9a3c`. Test-only scalar timers retain the original full-plane construction, acquisition timestamps, hit sequence, observation TTL, query/guard semantics and producer scheduling. MLS alone uses opt-level 3; runtime including the source fixture remains an unoptimized test build with debuginfo. This is not a deployed build profile.

`source-assembly-1cm-off-v1.log` remains a failed goal trial: after 1.201 s it delivers a current observation-expiry replan, with eight nonzero commands and .019989 m travel. Actual-edge and receipt-tail oracles pass. Only three delivered frames are available, so these are small-sample attribution measurements, not stable throughput estimates.

All three frames retain 180,000 occupied fine cells and 190 blocks. Measured median/max times:
- Full-plane nested-BTreeMap evidence grouping: 413.261 / 420.548 ms
- Observation block replacement: 18.239 / 98.105 ms, including the first geometry build
- Metadata stamping plus immutable snapshot construction: .00655 / .016744 ms

The original raw log labels the final stage `immutable_snapshot_publication`; the source label is corrected to `metadata_stamp_and_snapshot_construction` because channel publication is outside this timer. Replacement includes cleanup performed inside replace_block and its consuming loop. Channel send/drop and consumer work are excluded. Frame timings are copied as fixed scalars into the already bounded delivered-frame records; dropped frames are not included in these distributions. No source payload owner is added by the instrumentation.

The selected acquisition timestamps precede every measured stage. Achieved median delivered sample spacing is 718.861 ms here, versus configured 200 ms. The full-frame regrouping cost dominates this fixture's source preparation. This does not establish that the actual incremental scan producer rebuilds a complete plane. Actual geometry-change construction and observation-only metadata refresh must remain separate tests.

Next experiment: reuse only the immutable fixture's spatial grouping, while continuing to materialize every frame's genuinely sampled timestamps/hits and replace every original block. Retain the original builder as a differential reference, compare exact cell/evidence tuples including geometry changes, and give any faster builder separate benchmark names before changing default fixture setup. No downsampling, TTL extension or fabricated future samples is proposed.

## Current production path is already block-incremental

This source inspection is against `8c52574`, not C1's off-tree revision/wire migration:
- `surface_dataflow.rs:592–599` starts the existing blocking surface-build owner and calls `prepare_surface_map` (`292–306`)
- `surface_map.rs:91–117`, `SurfaceMapCache::build`, receives the fine manifest and calls `FineMapReader::observations` with the previous root/observation owner
- `surface_chunks.rs:15–48` compares manifests, shallow-clones the existing observation index, removes deleted blocks, skips unchanged block SHA references, and expands only changed blocks into per-block evidence maps before replace_block
- `surface_map.rs:128–139` reuses the physical snapshot when its existing geometry identity matches and changes only the observation wrapper when required
- Producer `services/geometry/nuwax_geometry_runtime/src/fine_cloud.rs`, `accumulate_inner`, converts actual incoming scan hits and calls `nuwax_map_chunks::Writer::observe`; `Writer::observe/load_dirty/flush/seal` in `crates/nuwax-map-chunks/src/writer.rs` modifies scanned-hit blocks and seals dirty blocks rather than regrouping every known voxel on each source refresh

Thus the measured complete-plane outer regrouping is not shared production code. Production still materializes changed per-block evidence maps and runs exact membership validation; its costs require separate actual delta workloads. A static pre-grouped fixture can isolate controller throughput under fresh dense evidence but cannot demonstrate a production receiver speedup. No changes to these production paths are included in this instrumentation phase.

## Separate prepared-layout fixture experiment

The original entrypoints still call the original whole-plane builder. Two separately named ignored experiments use a fixture-only PreparedEvidenceLayout: one retained immutable key Arc plus per-block sorted integer indices. It stores no evidence or permission and copies no pointcloud. Every actual sample still materializes all cells with that sample's timestamp and hit sequence, then calls the same block replacement and snapshot operations. Preparation happens before the first acquisition and is included in reported priming time. A different key owner is rejected, including copy-on-write membership changes; rebuilding the layout then matches the frozen reference. This is materialization/rebuild coverage, not a live changing-geometry stream test. Index storage scales with the input and is not a fixed byte bound.

`prepared-source-parity-v1.log` passes the exact complete-map comparison for empty, negative, multiblock and duplicate keys, zero/MAX evidence, changed owners, deletion/addition and preservation of the original immutable owner. The duplicate-key result is exact because every occurrence receives the same per-frame evidence. No production scan or receiver code changes.

Both original 60-second goal criteria remain **FAILED** in the named prepared arms, after all actual-edge and receipt-tail oracles pass:

| Trial | Actual motion and final pose error | Remaining failure |
|---|---|---|
| `prepared-source-1cm-off-v1.log` | 488 nonzero commands, .931770 m travel, position error .068230 m, yaw .839039 rad | Final yaw error .731757 rad exceeds .15 rad |
| `prepared-source-1cm-on-v1.log` | 467 nonzero commands, .852505 m travel, position error .147495 m, yaw .893877 rad | Final yaw error .676919 rad exceeds .15 rad |

Each delivers 301 real samples, with median spacing 200.000 ms and maximum 200.129 ms OFF / 201.594 ms ON, no dropped frames, and every frame retaining 180,000 cells in 190 blocks. One-time layout construction is 167.37/173.48 ms. Per-frame evidence materialization medians are 30.506/30.493 ms, block replacement 19.814/19.783 ms, metadata/snapshot construction .00628/.00643 ms. This demonstrates fresh full-coverage fixture acquisition at the configured 5 Hz in these runs; the raw cadence label is clarified in source to say the configured period alone is not evidence of achieved cadence.

There are zero delivered negatives or observation-dispatch expiry observations in either prepared run. All 488/467 delivered tails pass; guard-versus-receipt state differences remain 55/59, so the existing delayed-activation limitation is unchanged. Goal timeout cannot be relabeled success just because fresh evidence and useful motion are now available.

Current-step median/p95/max remains 34.513/72.635/108.157 ms OFF and 34.528/72.078/113.480 ms ON. In OFF, observed-sweep time is 31.323/69.921/105.704 ms versus projection 2.708/3.723/7.322 ms, measured over the same called steps with inclusive timers. This points to observation-sweep construction/checking as the next controller measurement target. It does not justify dropping samples or evidence checks. The prepared fixture's source improvement is not a production runtime speedup, full-goal qualification, or hard 20 Hz guarantee.
