# Query-local recent-ground row reuse — 2026-10-13

## Scope and source audit

The frozen reference is the complete `check_body_pose` from
`bb1a14ffb24a91aeda1811a2b7332544df757bee`, verified byte-for-byte except its test-only
function name. The production change is confined to
`nuwax-nav-mls-planner/src/surface/body.rs`.

The modern production chain is:

1. `SurfaceFollower::refresh_current_body_view` derives current support from the
   original environment snapshot; `current_body_error` checks known ground
   relation and actual body pose
2. `with_body_motion_checks` constructs the existing geometry-only motion
   callback for both command selection and selected-command rechecks
3. `validate_body_motion_with_witness` retains its existing actual-pose samples,
   swept contact events, boundary checks, quantum, and result caches
4. A pose-result cache miss reaches `check_body_pose`, which derives the ground
   patch and checks recent frames before static fine/coarse occupancy
5. Each intersecting recent fine XY previously called `fine_ground_columns` for
   its entire enclosing coarse XY. That rebuilt a Vec of rows, batch-read the
   existing band cache under its mutex, and constructed static contact runs,
   even though this consumer read only one row's selected height

The old observation-footprint BTreeSet and per-cell freshness checker are now
`cfg(test)` compatibility/oracle paths, not modern production hot paths. This
work does not revive them. The production unassigned-column coverage BTreeSet,
current-ground provenance Vecs, existing geometry/cache locks and current-view
ownership are unchanged; this is not a claim that all allocations were removed.

## Change and invariants

A height-only XY row implements the existing `FineGroundValue` specialization.
Its height computation is shared with the complete static row; only unused
contact-membership construction is omitted. The pre-existing compact aggregate
`FineGroundHeight` type remains unchanged.

The recent loop retains only the last coarse column and its selected rows for
one complete pose query. All inputs to those rows are fixed during the loop:
pose, polygon/prism, refined patch, immutable observations, ground interpretation,
and current private clearing. Revisiting a nonadjacent coarse column recomputes
its rows; nothing is retained across poses or snapshots. No persistent cache,
new hash, lock, threshold, diagnostic check or planner policy is introduced.

Each fine XY still selects its own row. A missing row or `None` height is not
replaced by a coarse height; the original coarse fallback occurs only when the
entire selected-row set is absent. Frame order, fine XY order, ascending Z,
recent-before-static priority, first rejection and every prism predicate remain
unchanged. No source TTL, command deadline, replan cadence, motion quantum,
clearance tolerance or search budget is changed.

## Qualification

- Baseline MLS ordinary suite: **316 passed / 0 failed / 16 ignored**
- Final MLS ordinary suite: **320 passed / 0 failed / 16 ignored** (21.47 seconds)
- Final new focused semantics: **5 passed**; the separate ignored A/B benchmark
  was explicitly run and passed, not counted as an ordinary test
- Independent read-only review found no production semantic blocker; all four
  requested test-strengthening items were added and passed
- Rebuilt runtime `latest_recent_` selection: **6 passed**; `current_recheck_`
  selection: **4 passed**, with one overlap, therefore **9 distinct tests**
  covering original expiration, no reranking, current-state ownership, obstacle
  rejection and recent-source microjitter. This is not a full runtime suite
- `git diff --check` passed

The baseline already includes the frozen-oracle parity test and ignored benchmark;
the final count adds four ordinary semantic tests. Ignored tests are not passes.

The added ordinary tests include:

- 576 full-pose comparisons against the frozen complete query, retaining exact
  error and first-rejection fields; 384 clear and 192 rejected cases
- A 5 cm source with 5/7.5/10 cm coarse grids, no recent data, clear floor,
  missing-floor rejection, narrow door, persistent coarse-only data and recent
  obstacles, with 1/8 retained frames
- Measured and legacy rows, different heights within the same coarse XY,
  exact height-bit comparison, unknown slices and a full-pose exact-witness check
- Different successive frames, reuse across a frame boundary and competing
  blockers with an independently asserted first cell
- A guaranteed private current-support overlay with nonempty cleared voxels
- Pose changes, recent-source changes and actual observation-owner replacements

The missing-floor fixture is a rejection case, not proof of successful support
recovery: recent floor returns over unknown ground retain collision status.
A preliminary 20 cm coarse fixture failed the existing tread-resolution gate;
that unsupported fixture was replaced by 7.5 cm. No gate was relaxed and the
original failed run is preserved.

## Timing method and limits

Rust 1.88.0, offline/locked release, original workspace thin LTO and one codegen
unit; body-query profiling disabled. One Rayon thread and CPU affinity 0.
The final benchmark uses nine alternating AB/BA pairs, 512 complete pose calls
per arm per case, with 16 fixed nearby position/yaw values. Fixture creation is
outside timing. Every timed result and rejection is compared to the reference.

These are complete **pose-result-cache-miss queries with warm source/ground
caches**. They bypass the public pose-result cache deliberately so a cache hit
cannot hide query costs. They are not full command-selection, swept-motion,
robot-CPU, allocator/RSS or control-deadline measurements. Both arms include row
and temporary destruction. No per-allocation counter was installed. The reduced
Vec construction and lock acquisitions follow from eliminated repeated calls;
exact whole-process allocation counts are not claimed.

A separate pinned seven-pair A/A baseline preserved the unchanged algorithm and
showed several-percent noise (approximately 7% in some cases). Small changes in
negative controls are not presented as improvements. Equal 5 cm coarse/fine
resolution has no repeated coarse rows and showed no material speedup. Benefits
are concentrated where several recent fine columns share a coarse column.


## Final fixed-fixture measurements

Median microseconds per complete query, 10 cm coarse / 5 cm source:

| Case | Frames | Reference | Candidate | Reduction | Faster pairs |
| --- | ---: | ---: | ---: | ---: | ---: |
| clear | 1 | 47.88 | 31.24 | 34.8% | 9/9 |
| clear | 8 | 294.08 | 159.64 | 45.7% | 9/9 |
| door | 1 | 50.89 | 33.42 | 34.3% | 9/9 |
| door | 8 | 297.48 | 156.82 | 47.3% | 9/9 |
| obstacle | 1 | 17.18 | 9.46 | 44.9% | 9/9 |
| obstacle | 8 | 16.85 | 9.60 | 43.0% | 9/9 |

All six recent-rich combinations above improved in every pair, and an earlier
candidate run showed the same direction and approximate gain. Negative controls
are retained in the complete [results](body-recent-query-reuse-20261013-results.json):
no-recent was 13.97→14.32 µs, missing-floor rejection 27.79→27.57 µs and coarse-only
8.09→8.03 µs for one frame. These small changes do not establish a speedup or a
material regression. The equal 5 cm coarse/fine cases show no material benefit;
7.5 cm coarse cases improve less than the 10 cm recent-rich cases. No general
end-to-end or hardware speedup is inferred from this fixed-fixture result.

## Reproduction

Source `neh-toolchain/env.sh` adjacent to the checkout. From
`services/nuwax-workspace`:

```sh
cargo test --offline --locked --release -p nuwax-nav-mls-planner --lib
NEH_RECENT_PAIRS=9 NEH_RECENT_REPEATS=32 RAYON_NUM_THREADS=1 \
  taskset -c 0 target/release/deps/nuwax_nav_mls_planner-f2ede62f14d7a713 \
  recent_query_complete_pose_ab_benchmark --ignored --nocapture --test-threads=1
cargo test --offline --locked --release -p nuwax-nav-runtime --lib latest_recent_ -- --test-threads=1
cargo test --offline --locked --release -p nuwax-nav-runtime --lib current_recheck_ -- --test-threads=1
```

The executable suffix is local to this build. Local evidence is in
`neh-evidence/body-recent-query-20261013/`, including retained baseline/final
binaries, full logs, the original method and JSON timing summaries. No robot,
network service, paid call or Git push was used for this trial.
