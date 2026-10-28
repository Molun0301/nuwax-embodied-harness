# C1: preserve the public instance snapshot on SSE

Baseline: `cc89379c839c12e374ad5dd28bafb3b6992040aa` (the affected source also
matches the workstation's earlier `d1e83d32` baseline).

## Defect and fix

MemoryService selects the public instance view, normally up to 500 rows and at
most 1000, and supplies total/limit/truncated. Its agent list view and snapshot
share the row projection. SSE had no separate summary contract, but its generic
recursive JSON budget truncated valid rows and even the source limit metadata.
In a real MemoryService 500-row fixture, only 85 identities survived. The earlier
smaller synthetic row fixture retained 155; these are different payloads.

Only the SSE memory.instances topic now reuses the existing stream JSON encoder.
Regular public rows/fields are not truncated a second time. It does not validate
or authorize a new instance schema and does not change producer selection,
permission handling, subscriptions, queues, sorting or field projection.

SSE selects ASCII JSON escapes so the resulting text is UTF-8 encodable even
when a public name contains an isolated surrogate. JSON-decoded values are
preserved; the old generic path instead replaced that character. Encoding
exceptions on this SSE branch return to the original diagnostic projection.
BaseException is not swallowed. Existing WS calls retain the encoder's default
behavior. Other SSE topics retain their original budgets and encoding path.
No encoding cache, thread, new lock, whole-payload prevalidation or second
serialization implementation is introduced.

## Verified at this source checkpoint

- Final 12 new contract checks passed on Python 3.8.20 and Python 3.12.14
- These compare 500/1000-row output with the actual MemoryService public source
  view, not merely WS against SSE; they preserve exact u64 cursor text
- The final suite includes actual Starlette StreamingResponse ASGI byte emission
  for normal 500 rows, a surrogate name, long legal text, deep JSON and a throwing
  model_dump object. This is in-process ASGI, not a TCP/network test
- Final Python 3.8.20 existing checks: robot state 6, map snapshots 31,
  incremental point-cloud integration 6, encoding/router reuse 21 passed
- Total distinct final Python tests: 76 passed; the second interpreter is not
  counted as another twelve tests. No old load result is relabeled as final timing
- Python 3.8.10 itself was unavailable and is not claimed as tested

The initial unescaped candidate was tested on the workstation with real Uvicorn
and clients, including 16 bounded load/cleanup scenarios and a source-field
comparison. Subsequent edge probes showed that candidate (and the unchanged WS
path) could fail on deep JSON, custom serialization errors and surrogate UTF-8
encoding. Those failures are retained. The final ASCII/exception compatibility
change is covered by the cloud checks above; its additional workstation TCP
qualification is pending at this checkpoint.

## Limits

The source limit bounds row count only, not string length, total bytes, temporary
allocation or synchronous encoding time. A StreamFrame with this topic can carry
arbitrary JSON-compatible content; a topic name is not a structure validator.
Long legal names remain complete, which is the same decoded-value contract as WS.

ASCII escaping can increase non-ASCII payload size (for many Chinese characters,
three UTF-8 bytes become six ASCII bytes). Earlier approximately 250 KB frames,
encoding timings and load measurements belong to the previous unescaped version,
not to this checkpoint. Slow-client backpressure still exists. Previous slow WS
close code 1006 remains an abnormal close, not a successful handshake claim.

The normal fixtures used no real robot, live database, inference model or browser
rendering. This change does not resolve the existing WS edge failures or optimize
per-subscriber repeated encoding.
