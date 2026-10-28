# Browser MJPEG: copy only the selected frame

Date: 2026-10-14 UTC. Local source qualification; no commit/push claim.

## Narrow production change

In `gateways/http/static/js/console.js`, `extractNextJpeg` now returns a transient frame `subarray` instead of copying it. Its sole caller, `consumeMjpegStream`, already discards every complete frame except the last in each received batch. That caller now takes one `.slice()` immediately before asynchronous rendering. These are the only two production edits. Remaining-buffer copies, limits, validation, headers, error handling, decode admission and lifecycle are unchanged.

The transient typed-array views are mutable, **not immutable snapshots**. Reuse is justified by this private synchronous ownership interval: no await occurs between parsing and final selection; append/parser operations do not modify input; no other production caller retains the views. The final selected frame is independently copied before it crosses an async boundary. No view is retained in shared state, and no mutable SHM borrowing is introduced. Copying the remaining buffer is deliberately retained to avoid tying a tiny incomplete tail to an entire input allocation across reads.

The final source is text-equivalent to the measured in-memory candidate. The portable fixture reconstructs the original two-line baseline from final source and asserts that applying the two replacements reproduces final source exactly. It extracts actual parser, consumer and rendering-admission functions; it does not load the console application or contact a server.

## Caller and server audit

Go2 WebRTC encodes JPEG upstream, then sends it over UDS to the Rust relay. Python `VideoShmReader` takes an independent bytes snapshot. `_SharedMjpegFrameSource` builds one immutable multipart bytes object and shares it across browser clients; formatting happens outside its source lock and final reader join happens off the event loop. Browser `video.frame` publications carry metadata, not base64 images. No additional server-side encoding/cache/lock change is justified.

The perception source has its own reader and lifecycle. Unifying it with browser connection ownership would expand scope. Existing independent SHM snapshot ownership and server multipart sharing remain intact. See the earlier [MJPEG single-join audit](../验证证据/c1-mjpeg-single-join-20261008.md) and [stream audit](../验证证据/stream-delivery-audit-20261013.md).

## Qualification

The initial real-source fixture passed 26 contract cases in each of three measured processes. After timing, three tests additionally exercised the actual `renderDecodedVideoFrame` function with Node Blob and a fake display boundary: normal admission, stale generation at entry, and generation change during display with object-URL cleanup. The final production-source run passed **29 cases**, not 26+29 independent cases.

Coverage includes parser JPEG/metadata/error/remaining parity; one-byte and irregular chunk fragmentation; 1/4/8 complete frames; multiple complete frames plus incomplete tail; empty/no-complete inputs; prefix trimming; legacy no-Content-Length input; embedded EOI with a declared length; invalid lengths and camera escapes; independent selected bytes after the source buffer is mutated at the async boundary; generation changes and consumer cancellation/decode-error cleanup parity. Consumer fetch and display are mocked. These cases establish the bounded contracts, not all browser behavior.

## Copied bytes and timing

Each cohort contains synthetic JPEG-shaped data of the explicit size, enclosed in real-format multipart headers. Sizes are representative diagnostic choices, not a measured production traffic distribution. Three fresh Node v24.19.0 processes each ran 20 warmups per arm/cohort and 60 alternating AB/BA measured pairs. Localization's build/timing window was released before these measurements; it resumed afterward.

Timing measures the synchronous actual parser, append and selection/copy boundary. It excludes fixture generation, instrumentation, fetch, asynchronous consumer scheduling, Blob creation, actual image decoding, DOM/GPU and network. Counts include TypedArray.slice copies (frame plus unchanged tail copies), not every allocation, RSS or Blob/browser storage.

| JPEG bytes | Frames per batch | Slice-copy bytes, old → new | Median of three process medians, ms, old → new |
| ---: | ---: | ---: | ---: |
| 4096 | 1 | 4098 → 4098 | 0.007501 → 0.007621 |
| 4096 | 4 | 42276 → 29988 | 0.025329 → 0.022734 |
| 4096 | 8 | 153576 → 124904 | 0.086411 → 0.075584 |
| 65536 | 1 | 65538 → 65538 | 0.024487 → 0.024296 |
| 65536 | 4 | 656682 → 460074 | 0.076956 → 0.058889 |
| 65536 | 8 | 2365444 → 1906692 | 0.273793 → 0.218430 |
| 262144 | 1 | 262146 → 262146 | 0.026740 → 0.026460 |
| 262144 | 4 | 2622768 → 1836336 | 0.256246 → 0.174693 |
| 262144 | 8 | 9443360 → 7608352 | 0.960510 → 0.758195 |

Exactly `(frames − 1) × JPEG bytes` are no longer copied. All multi-frame per-process median comparisons improved in these fixed fixtures. Single-frame cohorts retain the same copy count; direction of small timing differences is mixed (64 KiB second process: 0.024106 → 0.024617 ms). Do not claim single-frame speedup or establish non-regression from these microbenchmarks.

The 256 KiB ×8 per-process pairs were 1.123296 → 0.864335, 0.960510 → 0.752576, 0.949203 → 0.758195 ms. This is a batch parser/copy result, not FPS, end-to-end latency or robot CPU improvement. Remaining-buffer copies still dominate some multi-frame cohorts; no additional restructuring was attempted.

Actual browser execution remains blocked by the previously observed process-singleton EPERM. Node qualification does not establish browser/GPU decoding, real network chunk distribution, sustained memory behavior or device performance. No thresholds or safety settings changed.

## Reproduction and evidence

- Fixture: `scripts/diagnostics/mjpeg_parser_owned_selection.cjs`
- [Full structured results and sample timings](mjpeg-parser-owned-selection-20261014-results.json)
- Contracts only: `node scripts/diagnostics/mjpeg_parser_owned_selection.cjs --contracts-only`
- Fixed benchmark and contracts: `node scripts/diagnostics/mjpeg_parser_owned_selection.cjs`

No hashes, Git writes, builds, paid models, network or user-computer actions were performed in this audit. Raw benchmark evidence predates production editing; the final source received the contracts-only qualification, with exact candidate equivalence asserted instead of a redundant timing rerun.
