# Browser MJPEG: detach the retained tail once per parsed batch

Date: 2026-10-14 UTC. Local source qualification; no commit/push claim.

## Baseline and scope

This is a separate second-stage measurement against `b97bdd9`, the stage-one selected-frame ownership change identified by the parent task. It is **not** another comparison against the earlier all-frame-copy implementation. The [first-stage report](mjpeg-parser-owned-selection-20261014.md) and its measured results remain unchanged. Benefits from these independently measured stages are not added together.

No Git or hashes were used in this audit. The stage-one source was read from the current checkout; the final fixture reconstructs that baseline by reversing exactly the two tail changes and asserts that its candidate reproduces the final production text.

## Exact change and ownership

Only two production changes in `gateways/http/static/js/console.js`:

1. A complete frame's `remaining` value uses `buffer.subarray(eoi + 2)` instead of `buffer.slice(eoi + 2)`
2. After the synchronous parse loop, `if (parsedAny) buffer = buffer.slice()` detaches the final retained tail before asynchronous rendering

The selected-frame `.slice()` introduced in stage one remains unchanged. Append behavior, no-complete-frame handling, incomplete-prefix trimming, frame-size checks, overflow behavior, metadata, errors and cancellation remain unchanged. No backing-buffer identity check, extra flags or cache was added.

Within one parse batch there is no await and neither append nor parsing mutates its input. Complete-frame tail views are used only to continue that synchronous parse. When any frame was completed, the final `buffer` is independently copied before rendering or the next read can yield. Mutating the earlier input chunk during display or the next read therefore cannot alter the retained continuation. When no frame completed, existing borrowing/trimming behavior is intentionally retained; this change does not introduce a stronger snapshot guarantee for that path.

The local input/result variables can still keep batch storage reachable during the current asynchronous operation; no immediate-GC, peak-memory or RSS claim is made. The ownership guarantee concerns selected/retained bytes used after an asynchronous boundary, not object lifetime determined by the JavaScript engine.

### Trimming caveat

The parser retains up to 16 KiB **before** an incomplete JPEG's SOI, plus the bytes after SOI. Its copied tail can therefore be much larger than 16 KiB. The fixture explicitly covers a declared 8 MiB JPEG with 4 MiB received, after a 20,000-byte prefix.

The simple candidate may copy an already detached trimmed tail again. However, the baseline first copied the full untrimmed suffix and then trimmed it, while the candidate trims a borrowed suffix and then detaches the final result. Counts in that large control are 8,428,871 → 8,425,034 copied bytes, nearly unchanged. The optimization does not remove both large copies; it does not claim this case is accelerated. An additional ownership-identity branch was deliberately not introduced.

## Final qualification

Three measured processes each passed 48 contract cases; final actual-source contracts also passed **48**, not an additional independent suite to add to earlier counts. Exact final-source equivalence to the measured in-memory candidate is asserted; timing was not repeated after production editing.

Coverage carries forward parser bytes/metadata/errors, malformed length/camera headers, embedded EOI with Content-Length, fragmented reads, consumer generation/cancellation/decode errors and actual rendering-function admission/generation checks with Node Blob and fake display.

New tail cases cover:

- Multiple complete frames followed by partial header or partial JPEG, with original input mutated during rendering and at the next read
- Fragmented continuation across 1/7/113/2048-byte chunks, with final image and sequence unchanged
- Empty tails; no complete frame; original small-prefix borrowing and large-prefix trimming
- Complete frame plus large garbage/incomplete prefix; detached 480 KiB incomplete image continued after external mutation
- Buffer-overflow trimming with a valid trailing JPEG and unchanged error when no SOI exists
- Separate 4 MiB incomplete-tail copy-count/timing control

Actual consumer functions use mock fetch/read/display boundaries. Rendering admission uses Node Blob but does not decode a real browser image.

## Fixed measurements

Three fresh Node v24.19.0 processes. Each arm/cohort has 20 warmups and 60 alternating AB/BA measured pairs. Exploration released its build/test CPU lane before timing and was notified when it ended. Synthetic JPEG-shaped payloads use explicit 4/64/256 KiB sizes and 1/4/8 complete frames per read batch. These are diagnostic cohorts, not measured live packet distributions.

The timed scope is synchronous real parser/append/selection and both ownership-copy boundaries. Fixture construction, copy instrumentation, network, asynchronous consumer scheduling, Blob construction, image decoding, DOM and GPU are excluded. Copied-byte counts count TypedArray.slice result bytes, not every allocation or memory traffic.

| JPEG bytes | Frames | Copied bytes, baseline → candidate | Median of process medians, ms |
| ---: | ---: | ---: | ---: |
| 4096 | 1 | 4098 → 4098 | 0.006731 → 0.006780 |
| 4096 | 4 | 29988 → 4098 | 0.024186 → 0.019500 |
| 4096 | 8 | 124904 → 4098 | 0.052088 → 0.036796 |
| 65536 | 1 | 65538 → 65538 | 0.010185 → 0.010125 |
| 65536 | 4 | 460074 → 65538 | 0.052029 → 0.023015 |
| 65536 | 8 | 1906692 → 65538 | 0.176487 → 0.039840 |
| 262144 | 1 | 262146 → 262146 | 0.023646 → 0.024097 |
| 262144 | 4 | 1836336 → 262146 | 0.151159 → 0.037016 |
| 262144 | 8 | 7608352 → 262146 | 0.700559 → 0.058919 |

The 256 KiB ×8 process pairs were 0.705165 → 0.058989, 0.700559 → 0.057326, 0.696713 → 0.058919 ms. All fixed multi-frame cohort medians improved in all three processes. The unchanged selected-frame copy plus final two-byte CRLF account for the 262,146-byte candidate total in that cohort.

### Controls and retained regressions

| Control | Copied bytes, baseline → candidate | Median of process medians, ms |
| --- | ---: | ---: |
| empty | 0 → 0 | 0.000641 → 0.000631 |
| no_complete_small | 0 → 0 | 0.002965 → 0.002965 |
| no_complete_prefix_trim | 16668 → 16668 | 0.110767 → 0.110777 |
| single_empty_tail | 4096 → 4096 | 0.020341 → 0.020361 |
| single_partial_tail | 4598 → 4598 | 0.008633 → 0.008653 |
| single_garbage_trim | 40482 → 36864 | 0.113922 → 0.113742 |
| single_incomplete_prefix_trim | 41266 → 37432 | 0.117778 → 0.117587 |
| single_large_incomplete_prefix_trim | 8428871 → 8425034 | 0.972087 → 0.962895 |

Single-frame 256 KiB process pairs were 0.024547 → 0.024788, 0.023646 → 0.024097, 0.023226 → 0.023376 ms: the candidate was slightly slower in all three. Single-frame 4 KiB likewise had small slowdowns in all three processes. These observations are retained, not relabeled as noise or proof of non-regression. Single-frame copy counts are unchanged.

No-frame controls have unchanged copy counts and small mixed timing differences. The large incomplete-tail control pairs were 0.972087 → 0.954621, 0.939810 → 0.962895, 1.122206 → 1.009015 ms; the second process regressed. Its copy reduction is tiny, so no meaningful benefit is claimed for that case. No-frame and single-frame controls remain distinct from the multi-frame finding.

## Decision and limits

Adopt the small tail-view/one-detachment change because it removes repeated large suffix copies in the evidenced multi-frame batching path while preserving retained-byte isolation. Do not extrapolate to FPS, browser decoding performance, end-to-end latency or robot CPU. Actual browser execution remains unqualified because of the previously observed process-singleton EPERM; Node results do not close that gap. No thresholds or safety settings changed.

## First-stage fixture compatibility

The earlier `mjpeg_parser_owned_selection.cjs` now explicitly removes the two known tail-stage edits before reconstructing either historical timing arm. This prevents a later invocation from silently measuring first-stage frame-copy savings on a different tail implementation. Its historical report and result JSON are unchanged, and their timing was not rerun.

The earlier fixture additionally supports `--current-contracts`, which uses the actual current consumer/parser against the historical first-stage baseline for output/lifecycle parity. Both historical reconstruction and actual-current contracts passed 29 cases. These overlap the 48-case tail suite and are not additive new coverage counts. The rendering function itself was unchanged by either tail edit.

- Historical first-stage contracts: `node scripts/diagnostics/mjpeg_parser_owned_selection.cjs --contracts-only`
- Actual-current first-stage contracts: `node scripts/diagnostics/mjpeg_parser_owned_selection.cjs --current-contracts`
- The default first-stage benchmark, if explicitly rerun later, retains its first-stage baseline/candidate by stripping these known tail changes

## Files and reproduction

- Fixture: `scripts/diagnostics/mjpeg_parser_owned_tail.cjs`
- [Structured results, controls and all samples](mjpeg-parser-owned-tail-20261014-results.json)
- Contracts only: `node scripts/diagnostics/mjpeg_parser_owned_tail.cjs --contracts-only`
- Fixed contracts/benchmark: `node scripts/diagnostics/mjpeg_parser_owned_tail.cjs`

No Git writes, hashes, builds, models, network or user-computer actions were performed. This report and fixture are separate from the first-stage artifact so their baselines and numbers remain explicit.
