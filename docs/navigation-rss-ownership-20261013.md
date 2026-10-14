# Sustained navigation RSS ownership diagnostics — 2026-10-13

## Result and limits

The two original five-minute synthetic-worker arms pass again: **49 goals OFF and 49 ON**, 607.989 seconds of live control-loop time, 11,675 nonzero receipts and 43,640 actual plant edges. This is additional diagnostic evidence, not a new production performance optimization or robot qualification.

The reproduced late-window RSS increases are **2,742.0 KiB OFF / 2,666.8 KiB ON**, while GNU allocator in-use-plus-mmap increases are only **44,252.8 / 117,067.2 bytes** over exactly the same windows. Allocator free-arena bytes increase by **3,198,550.4 / 1,962,472.0 bytes**. All persistent diagnostic vector capacities are constant across all 49 drains, and their lengths are zero after every clear.

After all fixture owners are dropped and worker/sensor joins complete, a single diagnostic `malloc_trim(0)` releases **24,776 / 21,072 KiB of RSS**, without changing allocator in-use bytes. This directly establishes a substantial reclaimable allocator/page-residency component in these processes. The data supports allocator/free-page retention as the main explanation of the reproduced RSS trend, rather than multi-MiB growth of still-allocated payload. It does **not** establish zero growth, exact ownership of the remaining 43/114 KiB window shift, a production leak absence proof, or retrospective attribution of every byte in the [earlier runs](navigation-sustained-worker-20261013.md).

No production allocator, cache, ownership or retention policy changed. Terminal trimming is an experimental intervention only, not a recommendation for idle-time/production trimming.

## What was measured

Only `nuwax-nav-runtime/src/route_sustained_worker_tests.rs` changes. Diagnostics are opt-in with `NEH_SUSTAINED_MEMORY_DIAGNOSTIC=1`; normal fixture behavior remains the default. The existing calibrated 5 cm source, five phases, 5 Hz real acquisitions, 20/40 Hz consumer/pose timing, non-ground remote return, stop acknowledgement and all motion/receipt/oracle acceptance checks remain intact.

At each acknowledged stopped boundary:

- **A:** before output/oracle, with receipt, edge and diagnostic histories alive
- **B:** after the unchanged independent oracle, with those histories still alive
- **C:** after existing receipt/edge/event/gap drops and all persistent diagnostic clears

Each checkpoint records `/proc/self/status` RSS, high-water and threads; `/proc/self/smaps_rollup` anonymous/private-dirty pages; and optional GNU `mallinfo2` arena, in-use, free, mmap bytes/count and top-releasable bytes. GNU allocator in-use plus mmap is `uordblks + hblkhd`; it includes allocator-specific accounting effects and is **not exact Rust live payload**. In particular, allocator tcache accounting and capacity allocations cannot be equated with live logical elements. RSS and virtual arena size also measure different things.

The sensor remains alive during A/B/C and can allocate concurrently. Samples across procfs and allocator counters are not atomic. These are stopped-boundary observations, not exact isolated oracle allocations. There are no new hot-loop allocation counters, custom global allocator, dependency additions, production cache clears or per-goal trim calls.

Terminal checkpoints separate worker join, sensor join/queue drop, map/boundary drop, diagnostic owner drops, and finally pre-trim versus post-trim. Eleven process threads remain at terminal samples; this is fixture-owner teardown, not process shutdown. Code, shared libraries, stacks and unrelated process-global state remain outside the interpretation of GNU in-use bytes.

Environment: GNU/Linux, glibc 2.41, Rust 1.88.0, Python 3.8.10. The source tree has no Runtime global allocator replacement; the only located replacement is an unrelated MLS example. `LD_PRELOAD`, `LD_AUDIT`, `MALLOC_ARENA_MAX` and `GLIBC_TUNABLES` were empty/unset; `/etc/ld.so.preload` was absent. GNU functions use optional dynamic symbol lookup; unsupported metrics report `None`. The GNU FFI exception to `deny(unsafe_code)` is confined to this test-only diagnostics submodule.

## Full series and exact window definition

Let `C[i]` denote the post-drop/clear checkpoint for zero-based goal leg `i`. Both full arms have 49 such checkpoints. The comparison is:

`mean(C[39:49]) - mean(C[5:15])`

That is, early one-based drains **6–15**, after skipping the first five drains, versus late drains **40–49**. It is not the mean of every drain after the fifth, and neither window is terminal teardown. The convention matches the earlier first-ten-after-five versus final-ten summary. All checkpoints, full start/end/ranges, per-phase summaries and raw vector rows are retained in the [structured results](diagnostics/navigation-rss-ownership-20261013-results.json), so interpretation need not depend on those two windows.

| Post-drain measure | OFF | ON |
|---|---:|---:|
| First / last RSS, KiB | 19,900 / 38,096 | 22,048 / 44,860 |
| All-drain RSS min–max, KiB | 19,900–42,132 | 22,048–47,904 |
| Early / late window RSS mean, KiB | 34,431.2 / 37,173.2 | 41,142.0 / 43,808.8 |
| Late minus early RSS, KiB | +2,742.0 | +2,666.8 |
| First / last allocator in-use+mmap, bytes | 13,094,000 / 13,501,552 | 15,981,312 / 16,664,864 |
| All-drain in-use+mmap min–max, bytes | 13,094,000–14,353,136 | 15,981,312–17,202,976 |
| Early / late in-use+mmap mean, bytes | 13,598,524.8 / 13,642,777.6 | 16,568,534.4 / 16,685,601.6 |
| Late minus early in-use+mmap, bytes | +44,252.8 | +117,067.2 |
| Late minus early free-arena bytes | +3,198,550.4 | +1,962,472.0 |

Anonymous-page mean changes equal the RSS window changes in these arms. This is consistent with allocator-backed residency rather than a growing mapped log file, but is not an independent owner census.

## Fixture history, capacities and oracle effects

Source inspection confirms that receipt/edge histories retain immutable snapshot owners within a leg and are dropped before C. `Probe` diagnostic vectors and recheck vectors are cleared, retaining capacity. Profiling thread-local records are fixed-size arrays, not growing histories. The source queue has capacity one. `--nocapture` writes directly to a file; the Python launcher loads each log only after that test process has exited, so its marker collection cannot explain that test process's RSS.

| Persistent diagnostic backing storage | Capacity bytes OFF | Capacity bytes ON |
|---|---:|---:|
| `control_samples` | 6,010,368 | 6,010,368 |
| `control_ms` | 38,528 | 38,528 |
| `map_update_ms` | 9,600 | 9,600 |
| `source_frames` | 96,000 | 96,000 |
| `source_installs` | 57,600 | 57,600 |
| `rechecks` | 28,800 | 28,800 |
| `endpoints` | 0 | 96 |
| **Total, unchanged at every drain** | **6,240,896** | **6,240,992** |

These are exact `capacity × size_of(element)` backing-store bytes, excluding nested allocations, allocator overhead and resident-page fractions. The buffers remain allocated despite zero logical elements at C. The large `control_samples` allocation is mostly reserved capacity; size alone is not evidence that all its pages are resident. Per-leg capacities are also constant: receipts 796,800 bytes, edges 385,280, events 144,000 and gaps 9,600. Those four buffers are dropped before C.

During A→B, allocator in-use+mmap rises by a median **1,746,400 / 1,968,912 bytes** OFF/ON and RSS by **3,136 / 3,584 KiB**. During B→C, allocator in-use+mmap falls by a median **21,225,824 / 20,577,888 bytes**, but median RSS change is **zero** in both arms. Historical snapshot queries and diagnostic output therefore sit on an important temporary-allocation path, while releasing those allocations does not usually return resident pages immediately. The concurrent sensor prevents assigning every byte of these deltas exclusively to oracle versus history ownership.

There is no evidence here of accumulating retained receipt/edge records or diagnostic vector capacity across legs. The original cache audit's entry-count bounds remain useful but are not transformed into byte bounds by this experiment.

## Terminal owner release and diagnostic trim

| Terminal checkpoint | OFF RSS, KiB | OFF in-use+mmap, bytes | ON RSS, KiB | ON in-use+mmap, bytes |
|---|---:|---:|---:|---:|
| Worker joined | 38,164 | 12,339,136 | 36,240 | 12,405,952 |
| Sensor joined, queue dropped | 38,164 | 12,217,920 | 36,240 | 12,284,416 |
| Map/boundary dropped | 34,840 | 6,507,056 | 31,944 | 6,508,512 |
| All fixture owners dropped, **pre-trim** | 34,672 | 263,328 | 31,776 | 264,432 |
| **Post-trim**, same owners already absent | 9,896 | 263,328 | 10,704 | 264,432 |

Both terminal trim calls return 1. Mmap allocation bytes are zero before and after terminal trim. In-use bytes remain unchanged while RSS falls by 24,776/21,072 KiB. The post-teardown 263/264 thousand-byte counts are process allocator statistics, not total process memory or all Rust live bytes. The remaining RSS does not by itself identify a leak.

## Motion validation and build evidence

| Arm | Goals | Live seconds | Accepted nonzero seconds | Receipts / edges |
|---|---:|---:|---:|---:|
| Short OFF | 5 | 27.061 | 26.445 | 529 / 1,949 |
| Short ON | 5 | 27.211 | 26.295 | 526 / 1,981 |
| Full OFF | 49 | 303.844 | 291.892 | 5,839 / 21,860 |
| Full ON | 49 | 304.146 | 291.747 | 5,836 / 21,780 |

Full OFF/ON maxima in milliseconds: inner step **11.312/8.766**, map install **0.725/0.593**, current-state recheck **29.619/6.056**, consumer gap **54.360/53.204**. Every unchanged 50 ms inner-step/map/recheck and 75 ms consumer-gap check passes. Every goal stays within its original 60-second ceiling and position/yaw limits. No source/diagnostic overflow, missing proof context or invalid proof-clock pair occurs. Worker/guard-basis mismatch counts are 4/2 and guard/receipt-state mismatch counts 2,794/2,817; raw lineage records remain in the logs. Passing these checks is not a claim that those raw comparison counts are zero.

The new optional-proc-field parser test passes (1 passed, 297 filtered). A full ordinary Runtime suite was not rerun as part of this RSS diagnostic task; preceding Runtime regression results belong to their separate recorded task.

The first diagnostic compile **failed** because Runtime denies unsafe code. Its log remains preserved. The retry adds only the reviewed test-module-local GNU FFI lint exception and passes. Release settings are unchanged: opt-level 3, thin LTO, one codegen unit, with `body-query-profile`. No manual hash computation occurred. Git source baseline: `0eed9d2aa2cc23cf1083ecb3b91f44fbd1a9361f`, plus the frozen diagnostic module. Later Scan source changes happened after the executable was copied and do not change this binary's lineage. All four arms were serial, after Geometry's lane release and before Scan's next native work.

## Reproduction and preserved files

Build: `cargo test --offline --locked --release -p nuwax-nav-runtime --features body-query-profile --lib --no-run --message-format=json`.

Run: `NEH_SUSTAINED_MEMORY_DIAGNOSTIC=1 python3 scripts/run_sustained_navigation.py --binary <exact copied executable> --output <new evidence directory> --stage all`.

Raw evidence root: `/workspace/scratch/71cdb9d17732/neh-evidence/navigation-rss-20261013/`.

- `build.jsonl`, `build.stderr`: retained initial lint failure
- `build-v2.jsonl`, `build-v2.stderr`, `build-v2.exit`: successful incremental retry
- `fixture-retry-v2.rs`, `runtime-rss-v2`, `binary-v2-provenance.json`: exact frozen fixture, copied Cargo executable and non-hash provenance
- `unit-v2.log`, `launcher-v2.log`, `launcher-v2.exit`: unit and serial launcher results
- `diagnostic-v2/{short,full}-{off,on}.log`, `diagnostic-v2/results.json`: all original raw output and exit results
- `analyze.py`, `analysis-v2.json`: Python 3.8-compatible analysis and full raw structured checkpoints

The remaining owner-attribution question is narrow: the approximately 43/114 KiB allocator window changes can reflect phase-dependent current maps/caches, source timing or allocator accounting. This bounded measurement does not isolate them. A broad allocator rewrite or production trim is not justified by the present evidence.
