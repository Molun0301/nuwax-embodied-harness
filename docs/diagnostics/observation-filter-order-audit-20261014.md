# Observation filter-order audit — 2026-10-14

## Decision

**Rejected: no production filter change.** Avoiding unused `atan2` work helps the synthetic all-out-of-range capacity case, but the fixed full-view measurements do not establish a consistent benefit for retained in-range/map-like/planar controls. No adaptive extra samples were run to rescue this outcome. [Results and all samples](observation-filter-order-audit-20261014-results.json).

## Candidate and exact contract

`nuwax-explore-routing/src/observation.rs::estimate_view` calculates azimuth and elevation for every finite first-occurrence target before combining range/FOV checks. The test-only candidate checks the same two range comparisons first, using `!(range >= min && range <= max)` to preserve NaN rejection. It then computes the existing planar-height or volume-elevation condition, and calculates azimuth only if those filters pass. It performs no precision substitution, new deduplication, threshold change, cached permission or budget change.

`observation_filter_tests.rs` retains the exact original reference and candidate. Comparisons cover exact returned errors, cell sets, ray counts, budget flags and ordered callback origin/target float bits. Cases include ±0, finite boundaries adjacent to min/max range, NaN/infinity, overflow from finite coordinates, invalid body/calibration, invalid-first versus finite-out-of-range-first duplicate semantics, 16/128/8192 ray budgets, and 48 full-pose/extrinsic cohorts in both input orders. The production implementation also matches the frozen reference. Ordinary routing tests report **14 passed / 0 failed / 1 ignored** at this checkpoint; the ignored new diagnostic explicitly passed separately.

## Fixed full-view measurements

Each sample completes 16 `estimate_view` calls, including actual `voxel_view_is_clear` queries at resolution 0.2 m and the unchanged 65,536-step budget. The occupancy callback contains a fixed wall segment. Construction and equivalence assertions are outside timing; estimates, eligibility/dedup work and ray queries are inside. Each cohort has exactly three paired measurements with alternating AB/BA/AB order in one process, after one output check per implementation. No concurrent benchmark/build ran.

| Synthetic cohort | Baseline median ms | Candidate median ms | Paired C/B ratios |
| --- | ---: | ---: | --- |
| In-range 1,024 targets | 138.406 | 140.128 | 1.009, 1.068, 0.967 |
| 16 m grid, 4,096 targets | 249.990 | 261.916 | 0.998, 1.004, 1.048 |
| 32 m grid, 16,384 targets | 307.011 | 307.184 | 0.963, 1.041, 0.927 |
| Planar 16 m grid | 57.376 | 59.783 | 1.010, 1.047, 1.003 |
| Vertical-rejected control | 7.687 | 7.660 | 0.785, 0.997, 1.018 |
| All-out-of-range capacity control | 29.815 | 25.634 | 0.860, 0.837, 0.829 |

Median paired ratios are not ratios of pooled medians. The consistently faster all-out-of-range result cannot substitute for benefit on views that actually cast rays. Map-like cohorts are synthetic spatial distributions, not captured production maps; their positions include exact voxel-boundary targets. Consequently these absolute timings are not a cell-center-only production throughput estimate. The diagnostic is the full shared view operation, not the full exploration decision or physical navigation.

## Reproduction and retained evidence

From `services/nuwax-workspace`, source the existing toolchain environment and build only:

```sh
cargo test --offline --locked --release -j2 -p nuwax-explore-routing --lib --no-run --message-format=json
```

Workspace release opt3/thin-LTO/codegen-units=1 is unchanged. The executable obtained from Cargo JSON was saved, then the equivalence test, ordinary routing suite and explicit `observation_filter_order_profile --ignored --nocapture --test-threads=1` were run. Evidence is retained in `/workspace/scratch/71cdb9d17732/neh-evidence/observation-filter-order-20261014/`: `test-build.jsonl`, `test-build.log`, `diagnostic-test`, `equivalence.log`, `routing-suite.log`, `paired-profile.log`, `initial-summary.json`.

No production change, new cache/lock/hash, geometry relaxation, source-health bypass or robot command is part of this experiment. The next separate stack-axis experiment concerns per-ray traversal scratch allocation and does not adopt this rejected filter reordering.
