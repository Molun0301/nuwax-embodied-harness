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
