# Recent-guide fine-departure fallback regression — 2026-10-13

## Finding

The current real-guide pipeline already handles this independently witnessed body-sensitive fallback. Seven actual recent hints change the static straight PCT reference into a detour. Exported Scan cannot finish before local search; its existing fallback then succeeds in **3 expansions, 15 nodes, frontier peak 12**, returning **119 poses**, all independently body-reswept.

This is a positive regression, not a new navigation algorithm or a previously failing case that has been fixed. The earlier [static-guide priority ablation](local-search-priority-audit-20261013.md) remains a limitation of stale/misleading guidance. Its oracle successes and relaxed-bound failures must not be presented as improvements to the current recent-guide pipeline.

## Independently established geometry and witness

The fixture retains the parameterized `fine_escape_scene` construction: static rear pins at x=-0.29 m, a static forward wall at x=0.37 m, fine 0.01 m source observations, and a 0.60 m long body. The selected half-width is 0.0599 m (`half_width_cells=6`). This parameter lies between the existing thin- and wide-body variants; it is not their exact body width. Motion is the existing forward-only differential model, with in-place rotation allowed. Independent flat support is extended northward only so that a downstream obstacle and full detour fit within the surface.

Start is [0.011,0,0.4], facing east. The test asserts that all of the following collide at the start:

- In-place rotations ±0.08 and ±0.3 rad
- The 0.01 m curved departures with turn ±0.1 rad
- The 0.25 m straight primitive
- The original 0.2 m/±0.25 rad and 0.12 m/±0.45 rad arcs

A 0.01 m straight departure to [0.021,0,0.4] is independently witnessed before any search. The extended witness then goes north to y=0.45, west to x=-0.65, north to y=1.8, and east to the goal [0.021,1.8,0.4]. Every turn and translation uses the full motion validator; all 208 witness poses are independently reswept. The search result must retain the exact 1 cm departure, not replace it by a merged pose or larger primitive. Goal tolerance is 0.015 m; no final yaw is required.

Seven recent points span x=-0.3…0.3 at y=1.0,z=0.4. The test uses the production `RecentGuideOverlay` through its existing test-only source import, with the real body's radius. It does not inject the witness into priority or use the phase-one oracle mode.

## Gates and observed guide

The non-ignored profile regression checks:

1. Independent start/body constraints and complete swept witness
2. Nonempty actual recent hints
3. Successful PCT guide generation through production overlay
4. Overlay reference differs from the static reference
5. Exported Scan actually enters local search, measured after resetting the existing thread-local work counter
6. Success, independent result resweep, arrival, and retention of the exact 1 cm departure

Static reference: [[0.011,0,0],[0.021,1.8,0]].

Actual overlay reference, rounded here only for readability:

[[0.011,0,0],[-0.125,0.465,0],[-0.625,0.695,0],[-0.625,1.315,0],[0.021,1.8,0]].

The full precision result and reference are retained in [structured fixture results](recent-guide-fine-escape-results-20261013.json). Local-search entry is only reached in exported Scan after the projected polyline fails; the independent departure assertions establish the body-sensitive reason this is not an ordinary open-floor shortcut case. The test does not compare oracle or alternate heuristics.

## Scope and qualification

No production behavior, body legality, primitive set, cost, priority, deadline, expansion budget, or failure semantics changed. Source changes are a test helper's visibility, a `cfg(test)` counter reset, a profile-test module declaration, and the new positive regression. No pose quantization, hashes, locks, defensive guards, Git history changes, robot execution or local-computer run are part of this worker's change.

Raw first eligibility and final validation artifacts are under `/workspace/scratch/71cdb9d17732/fallback-fixture-20261013/`. The initial eligibility test already passed before it was strengthened into a non-ignored positive regression. This is one synthetic fixture, not general completeness or real-robot qualification. No elapsed-time comparison is claimed.

Final checks on the profile-gated source passed:

- Actual default-feature Scan tests: 50 passed, 0 failed, 1 ignored
- Profile-enabled Scan tests: 52 passed, 0 failed, 4 ignored
- Focused non-ignored exported fallback regression: passed, reproducing 3 expansions and 119 reswept poses
- Ordinary non-test `cargo check -p nuwax-nav-scan --lib --release`: passed; diagnostic overrides and work-reset hooks are excluded by `cfg(test)`

The first default-feature build exposed an optional-`serde_json` reference in the phase-one diagnostic module. That failure was preserved, and the module received the same profile gate as existing JSON diagnostics before the actual default and final profile suites above. The phase-one report explicitly distinguishes its earlier profile-enabled qualification from this actual default-feature check. No dependency was added. Existing upstream MLS compiler warnings remain; no full-workspace build was run.
