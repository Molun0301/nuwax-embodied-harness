# Exploration remaining allocation/dispatch audit

Read-only follow-up, 2026-10-14 UTC. No additional production candidate retained. This is a source-based risk/prioritization note, not a deployed profiler or a diagnosis that a robot should move.

## Findings

1. **Selected-goal/status copies are small in the actual caller.** Runtime `build_controller_with_geometry` / `build_profile_controller` (`services/geometry/nuwax_geometry_runtime/src/explore_runtime.rs:1685,1707`) install wavefront or surface, not both. `surface_adapter.rs:788` and `wavefront_adapter.rs:952` retain `vec![selected_goal.clone()]`, not the complete candidate pool. `CommittedView::capture` (`explore_compute_backend.rs:111`) copies this committed summary for publication. There is no evidence here for replacing these ordinary owned values with new shared ownership, caches or locking.

2. **Ordered mission queries have distinct inputs and observable short-circuit behavior.** `explore_compute_backend.rs:621::ordered_mission_query` checks homeward, then outward, then return from the actual outward endpoint. Errors and cancellation checks gate subsequent calls. The return leg cannot be inferred from the original pose or by assuming route symmetry. Parallelizing these callbacks would perform work that existing failure/cancellation branches deliberately never dispatch. The earlier [planar route-copy probe](planar-mission-route-copy-probe.md) already bounds the map-copy cost; it was not repeated.

3. **Body-support reconstruction is a real deeper hotspot, with limited immediate reuse scope.** `nuwax-nav-stack/src/surface_oriented.rs:122–179` projects each action sample using a from-pose query view, then the `validate_motion=true` branch derives another from-pose query view during complete validation. `nuwax-nav-mls-planner/src/surface/body.rs:533–551` quickly shares the original view when both body and raw support are valid. In the inferred-footprint path it rebuilds bounded footprint/evidence columns, and an already rebased view returns to the original source before deriving a fresh pose-specific view. Source/bounds/occupied-volume checks remain essential. Most search expansions use `action_samples(..., false)` and validate after dominance checks, so same-sample local reuse mainly concerns connector/rounding branches, not every explored successor. This overlaps prior latest-body/query reuse work. It needs exact blind-contact, occupied-volume, drop, rotation and cache/witness tests before any new scope; no mutation or new timing was attempted here.

4. **Calibration validation is not an expensive map scan.** `ObservationModel::validate` (`nuwax-nav-types-messages/src/navigation_capabilities.rs:205`) is fixed-size scalar/pose validation, allocating an error string only on rejection. `estimate_view` calls it for each view, but the successful check neither allocates target collections nor scans map data. Public callers can provide arbitrary models; globally bypassing validation is not justified. The prior rejected [view filter](observation-filter-order-audit-20261014.md) and [ray axes](observation-stack-axes-audit-20261014.md) probes retain the relevant full-operation costs.

5. **The remaining clear redundant scan did not meet the performance bar.** Lazy scoring-grid inflation eliminates a scan/clone on no-scoring wavefront polls, but fixed full-selector measurements are flat/mixed on realistic free, filtered and selected-goal controls. Only the synthetic all-occupied immediate-stop map improves strongly. See [negative diagnostic](wavefront-lazy-inflation-audit-20261014.md). The production selector remains unchanged.

## Frequency and operational limits

The scheduler default is 200 ms (`explore_runtime.rs:78`), but active-child checks at approximately `:3577–3590`, source admission and same-pose/Pending shortcuts prevent inferring five complete selections per second. No deployed branch-frequency counters or sensor replay were available in this audit. The retained [private-wait frontier condition](surface-private-wait-frontier-20261014.md) accelerates an admitted changed-pose wait returning Pending; it does not directly accelerate actual goal selection or robot motion.

The useful next measurement, if this area remains operationally slow, is actual admitted-poll/connector branch incidence with existing diagnostics and a representative sensor replay. Do not create a persistent generation cache, parallel callback schedule or safety exception on the strength of these source-only observations.

## Final production check

From `services/nuwax-workspace`, with the existing toolchain env:

```sh
cargo check --offline --locked --release -j2 -p nuwax-exploration --features surface --lib
```

Exit **0**, 1.35 s. This is the non-cfg(test) library path, not a test binary or full link. `nuwax-exploration` has `default=[]`; `surface` enables its local `../nuwax-nav-mls-planner` dependency and retains unconditional local routing/types/base dependencies. No wavefront or runtime feature was requested. Workspace release remains opt3/thin-LTO/codegen-units=1, with nested `.cargo/config.toml` incremental=true. Existing missing-documentation warnings remain. Evidence: `/workspace/scratch/71cdb9d17732/neh-evidence/surface-private-wait-20261014/production-check.log`.
