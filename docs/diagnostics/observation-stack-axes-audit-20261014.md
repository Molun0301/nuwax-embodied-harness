# Observation ray stack-axis audit — 2026-10-14

## Decision

**Rejected: production ray traversal remains unchanged.** Replacing its per-step axis `Vec` with fixed stack storage did not yield a full-view benefit. The long tied-ray control regressed consistently, and three of four full-view cohorts had median paired ratios above 1. This is a separate experiment from the [rejected filter-order candidate](observation-filter-order-audit-20261014.md); neither production change is applied. [Raw fixed samples and results](observation-stack-axes-audit-20261014-results.json).

## Exact candidate and equivalence

`nuwax-explore-routing/src/observation.rs::voxel_view_is_clear` collects the at-most-three axes tied at the next ray crossing into a local `Vec`. The test-only alternative uses `[usize; 3]` and a populated slice. It preserves the original axis order, 1e-10 tie comparison, subset-mask enumeration order, occupancy callback order, coordinate updates, input validation and step budget. No precision shortcut, algorithmic change, cache, lock, hash or bounds relaxation was proposed.

`observation_axes_tests.rs` retains the exact original and candidate implementations. An ordinary test compares the boolean result **and every ordered occupancy callback cell** across 8,600 input/limit/blocker combinations. Coverage includes zero-length, tiny deltas around 1e-12, signed zero, two/three-axis ties, voxel boundaries, NaN/infinity/out-of-domain coordinates, invalid resolutions, zero/one/two/16/128 step budgets, early blockers and deterministic mixed-direction rays. Both candidate and unchanged production are checked against the frozen original. The final routing suite is **15 passed / 0 failed / 3 ignored**; both new axis timing diagnostics passed when explicitly run. The other ignore is the separate filter-order diagnostic.

## Fixed measurements

All samples use the unchanged workspace release profile. Each cohort has exactly three alternating AB/BA/AB pairs in one process. Direct samples make 10,000 complete ray calls. Full-view samples make 16 complete `estimate_view` calls, including deduplication, eligibility, sampling and actual voxel rays, with exact output comparison outside timing. Fixtures, output checks and printing are outside timing. No concurrent timing/build ran; no adaptive repetitions followed the negative result.

Full-view controls use synthetic **voxel-center targets** at 0.2 m spacing, full pose, real wall occupancy callbacks and the unchanged 65,536 ray-step cap. They are not captured robot maps. Small views use 400 targets/128 rays; other views use 6,400 targets/512 rays, including directional and planar modes.

| Complete view batch | Baseline median ms | Stack median ms | Paired C/B ratios |
| --- | ---: | ---: | --- |
| Small | 1.226 | 1.299 | 0.734, 1.098, 1.080 |
| Large omnidirectional | 18.350 | 18.980 | 0.990, 1.051, 1.037 |
| Large directional | 17.688 | 19.163 | 1.058, 1.083, 1.021 |
| Large planar | 15.951 | 15.323 | 0.928, 0.986, 0.981 |

| Direct ray batch | Baseline median ms | Stack median ms | Paired C/B ratios |
| --- | ---: | ---: | --- |
| Zero-length | 0.190 | 0.200 | 0.959, 0.728, 1.138 |
| Tiny one-axis | 0.347 | 0.269 | 0.772, 0.811, 0.559 |
| Long untied | 15.655 | 17.368 | 0.853, 1.124, 1.086 |
| Long three-axis tie | 26.566 | 35.766 | 1.328, 1.428, 1.372 |
| Early blocker | 0.348 | 0.283 | 2.097, 0.742, 0.816 |
| Eight-step budget exhausted | 3.144 | 4.012 | 1.342, 1.121, 1.350 |

Median paired ratios need not equal ratios of pooled medians. Tiny controls are noisy. The consistently slower tied and exhausted-budget cases, plus full-view regressions, outweigh the isolated planar/tiny benefits. Removing a syntactic `Vec` is not proof of a measured allocation reduction or speedup: no allocator instrumentation or generated-code causal attribution was performed.

## Reproduction and evidence

After sourcing the existing toolchain environment, build from `services/nuwax-workspace`:

```sh
cargo test --offline --locked --release -j2 -p nuwax-explore-routing --lib --no-run --message-format=json
```

Read the executable from Cargo JSON and save it. Run the ordinary suite with `--test-threads=1`; run `stack_axes_ray_profile` and `stack_axes_view_profile` explicitly with `--ignored --nocapture --test-threads=1`, serially. The fixed sample schedule is inside the retained test-only fixture.

Evidence remains in `/workspace/scratch/71cdb9d17732/neh-evidence/observation-filter-order-20261014/`: `axes-build.jsonl`, `axes-build.log`, `axes-diagnostic-test`, `axes-equivalence.log`, `axes-routing-suite.log`, `axes-ray-profile.log`, `axes-view-profile.log`, `axes-summary.json`, `production-reference-check.txt`.

The reference check confirms both production functions still exactly match their frozen originals; only test-module includes were added to `observation.rs`. No full exploration selection, full-workspace suite, sensor replay, physical navigation or real robot CPU test was run. No production behavior or navigation safety contract changed.
