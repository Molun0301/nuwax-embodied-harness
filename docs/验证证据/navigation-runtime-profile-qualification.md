# Runtime plus MLS optimized test-profile qualification

## Scope and build provenance

Measured navigation source is `7fb6bb88aec9f3302c4e95717b637e874e8c41e7`, with only a diagnostic source-profile label correction in the moving harness. No algorithm, production Cargo profile, installer or deployed binary changed.

The existing warmed target was used with:

```text
cargo test -p nuwax-nav-runtime --features body-query-profile \
  --config 'profile.test.package.nuwax-nav-mls-planner.opt-level=3' \
  --config 'profile.test.package.nuwax-nav-runtime.opt-level=3' --lib ...
```

`runtime-mls-opt3-compile-v1.log` contains the actual compiler invocations. Both MLS and runtime have `opt-level=3`, `debuginfo=2`, and `debug-assertions=on`; PCT and Scan retain their default unoptimized test settings. Cargo's generic “test profile [unoptimized + debuginfo]” summary does not describe these package overrides. Runtime has no build script.

This also optimizes the runtime unit-test sensor builder, ideal plant, wrappers and runtime-instantiated generics. Whole-run gains cannot be assigned solely to controller code. The label now refers readers to recorded Cargo flags instead of hardcoding “unoptimized runtime.”

The first build took 1m03s. It was not entirely a leaf-only rebuild: earlier nested wrappers had sourced the SDK environment twice, while this fresh shell sourced it once. The changed `PKG_CONFIG_PATH`/`CMAKE_PREFIX_PATH` strings caused two liblcm/GTSAM discovery scripts and cached CMake targets to rerun, followed by dependent Rust compilation. The verbose log records zero C/C++ object compilations; SDK/GTSAM artifacts remained cached. Subsequent runs used one consistently sourced shell. This overhead is preserved rather than described as zero native build-script activity.

All logs below are under `/workspace/shared/neh-validation/doorway/`.

## Unchanged moving criteria

All four 1 cm trials pass the original configured .15 m position / .15 rad final-yaw goal, every actual-edge and delivered-tail oracle, and the existing maximum 50 ms inner-step criterion. TTL remains one second. Each has zero delivered negatives, dispatch-expiry observations and dropped frames; every source frame contains 180,000 fine cells in 190 blocks.

| Source arm / local planning | Goal time s | Nonzero receipts | Position / yaw error | Step median / p95 / max ms |
|---|---:|---:|---|---|
| Prepared fixed layout / OFF | 21.850 | 327 | .061808 m / .141455 rad | 10.504 / 22.131 / 31.159 |
| Prepared fixed layout / ON | 24.500 | 316 | .144144 m / .121458 rad | 9.060 / 22.791 / 35.708 |
| Original full-plane grouping / OFF | 21.650 | 324 | .058356 m / .141360 rad | 10.798 / 24.625 / 34.065 |
| Original full-plane grouping / ON | 24.450 | 315 | .145809 m / .146471 rad | 8.975 / 22.888 / 40.553 |

Evidence: `runtime-mls-opt3-prepared-moving-v1.log` and `runtime-mls-opt3-original-moving-v1.log`, two passes each. All earlier failed builds/runs retain their original status; these results do not reclassify them.

The original-source arm actually delivers 110/124 frames with sample-spacing median/p95/max of 200.000418/200.031555/200.110889 ms OFF and 199.999826/200.015452/200.349369 ms ON. This is measured 5 Hz full-coverage, observation-only fixture acquisition, not merely a configured period. Geometry remains fixed; this does not qualify geometry churn or the production incremental ingress path.

Original full-plane evidence grouping now has 14.955/14.046 ms medians; block replacement remains 18.422/17.749 ms. Prepared materialization medians are 3.371/3.264 ms. The producer's code also became optimized, so the original source's earlier TTL-starvation failures cannot be attributed to production ingestion capacity from this fixture. Acquisition timestamps are still captured before construction and are never fabricated or shifted.

The original-source exact selected-guard source-age maxima are 184.702/185.100 ms. Guard-versus-receipt state mismatch counts remain 216/210 (prepared 218/211), preserving the previously documented changed-command spatial activation limitation. Passing time-based tail checks is not a physical-actuator guarantee.

Ordinary 5 cm factory OFF/ON full goals pass in `runtime-mls-opt3-5cm-factory-v1.log`. Held-planning continuity passes in `runtime-mls-opt3-5cm-held-v1.log`: five fresh held commands, later admission, 211 nonzero receipts and 1.066487 m travel, with all oracles and zero negative/expiry outcomes. Its sampled step median/p95/max is 1.075/1.783/3.795 ms. The full runtime suite passes 182 tests / 45 ignored / the same five known UDS tests excluded (`runtime-mls-opt3-runtime-all-v1.log`).

## Separate captured-step comparison

To separate a control-step effect from producer optimization, `runtime-mls-opt3-captured-pair-v1.log` alternates six independent processes per profile using the retained binaries:

- MLS-only optimized: `nuwax_nav_runtime-e9811cc98d0a10e4`
- MLS plus runtime optimized: `nuwax_nav_runtime-3375711efb31375e`

Both contain the same navigation algorithm; the diagnostic profile-label string differs. Their sizes/mtimes are recorded before and after. Each process reconstructs the identical captured state; sample 0 is the first control step after the same unmeasured setup/admission, including current-view refresh. Samples 1/2 are stationary warm repeats, not moving-loop evidence. This static-source fixture has observation TTL disabled.

First-step min/median/max is 25.745/26.778/29.007 ms for MLS-only and 24.910/25.539/28.360 ms for MLS plus runtime. Warm sample-1 medians are .671/.364 ms and sample-2 medians .630/.349 ms. Every run retains 62 motion checks, index 287, progress .4139866949152396 and the exact command `(0.11033333212307146, 0, 0.38829984385948857)`. This supports a narrower runtime-cost observation; it does not turn the larger changing-source run differences into a pure controller speedup estimate.

## Limits

These are sampled cloud tests with debug assertions and instrumentation, not a full release build, deployed robot measurement or hard 20 Hz proof. The moving harness uses the actual planner backend/worker and an ideal plant, not full NavScheduler-to-motion-port execution. Existing delayed-activation and downstream physical-completion limits remain open. No profile setting is persisted by this experiment.
