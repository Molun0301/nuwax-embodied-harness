# MCP response encoding and shared-loop audit, 2026-10-14

## Decision

No production change. Moving the final MCP success-envelope JSONResponse construction into the existing projection worker did not materially improve shared-loop responsiveness. Large synthetic capacity responses demonstrably delay independent same-loop work, but the proposed small offload does not solve that delay. No live traffic distribution, device CPU result, or whole-system responsiveness claim is established.

## Scope and source findings

- HTTP map catalog/list, memory reads and task routes are synchronous FastAPI routes and already execute service work in the worker pool. Async MCP invocation/projection, configuration, console rendering, agent snapshots and log SQL have existing worker boundaries.
- `gateways/mcp/protocol.py` constructs the final `tools/call` JSONResponse on the event loop after `build_mcp_tool_result` returns from its worker. The complete envelope is encoded exactly once; `tools/list` also constructs its response on-loop but was not changed or capacity-timed here.
- Test-only candidate moves that tools/call JSONResponse construction into the already-existing projection lambda. It does not add a worker hop, encoder, lock, threshold, queue or dependency. Final byte-limit checking and exception/error handling stay in their original locations. Session dictionaries and Request state remain loop-owned. Invocation/admission lifecycle is unchanged.
- CPython's GIL can prevent loop progress even while CPU work runs in a thread. The measurements reject assuming thread ownership alone guarantees responsiveness; they do not individually attribute every delay to a particular redaction or encoder operation.

## Fixture and evidence

Python 3.8.10; real registered POST endpoint from `build_mcp_router`, actual request parsing, public result projection, byte-budget enforcement and JSONResponse. Only tool lookup/invocation use offline fixtures. No ASGI socket transport, runtime/device construction, SQLite or network.

A 1 ms heartbeat and repeated independent real `initialize` requests scheduled approximately every 2 ms run on the same loop. Five alternating A/B pairs per size, warmed worker arms, no production edits. All fixtures are synthetic observation-context text results. The 2,340-byte response is a small plausible-shape control; 1/4/8 MiB cases are explicit capacity fixtures, not observed tool traffic or statements that normal calls approach the 16 MiB cap. Request construction and dictionary construction are outside the main timed handler interval.

Median across five runs of each run's maximum gap/latency:

| Actual response bytes | Arm | Handler ms | Max heartbeat gap ms | Max initialize handler ms | Max initialize scheduled latency ms |
| ---: | --- | ---: | ---: | ---: | ---: |
| 2,340 | Baseline | 1.306 | 1.490 | 0.674 | 1.328 |
| 2,340 | Candidate | 1.162 | 1.509 | 0.560 | 0.851 |
| 1,048,868 | Baseline | 54.985 | 45.753 | 9.383 | 52.464 |
| 1,048,868 | Candidate | 54.862 | 46.405 | 9.248 | 52.836 |
| 4,194,596 | Baseline | 211.483 | 178.805 | 179.207 | 197.813 |
| 4,194,596 | Candidate | 211.611 | 175.869 | 175.809 | 193.946 |
| 8,388,900 | Baseline | 408.391 | 319.133 | 330.351 | 370.054 |
| 8,388,900 | Candidate | 420.091 | 325.256 | 325.189 | 362.276 |

Initialize scheduled latency includes lateness before entering its handler, so a stalled loop is not hidden by starting its timer late. These are scheduling probes, not production SLA or p95 estimates. Final rendering itself measured about 2.530/10.620/21.579 ms for the baseline capacity cases, versus 2.289/12.520/24.202 ms in the candidate; the much larger loop gaps therefore cannot be credited solely to final on-loop rendering.

## Equality, errors and ownership

- Existing nine `McpResponseEncodingTests` pass for both baseline and candidate (18 executions): complete exact encoding, exact/one-byte-over bounds, Unicode/escapes, non-finite and unsupported values, final envelope limit, business failures and matching text/structured channels.
- Every timed response has identical status, raw headers and body across arms. Render tracing confirms baseline final encoding on-loop and candidate off-loop. ContextVar value is preserved.
- Cancellation during a gated worker projection is identical: caller becomes cancelled before projection gate release; underlying projection finishes; context survives. This checks projection ownership only, not a new maintenance-admission cancellation fixture. Maintenance code is not changed and invocation finishes before this projection step.
- Diagnostics are not production correctness claims for other routes. No automated broad integration run is added.

## Serialization count and prior audit overlap

A separate count-only pass uses the unchanged real handler, without concurrent initialize instrumentation. Each control/1 MiB case performs twelve `json.dumps` calls: two tiny empty-envelope/null budget calls on-loop; seven key/scalar size calls in the worker; one public-data-to-text encoding in the worker; one full MCP tool-result-envelope size encoding in the worker; one final JSON-RPC-envelope encoding on-loop.

For the 1 MiB fixture, the worker tool envelope is 1,048,826 bytes and the final RPC response is 1,048,868 bytes. `_json_bytes` returns only length and discards encoded bytes. Thus there is no already-retained exact final RPC byte string that can simply replace JSONResponse. Reusing the transient inner-envelope bytes would require changing the shared builder's return contract and composing the outer RPC envelope while preserving its separate limit and encoding/error behavior.

The prior [serialization audit](../验证证据/observation-state-serialization-audit-20261013.md) already measured and rejected a more complicated size-accounting rewrite that avoided re-traversing structuredContent; it retained conversion/redaction. Retaining/reusing inner-envelope bytes is technically different but is not a demonstrated minimal fix for the dominant delays here. This audit stops rather than widening into a new serializer or cache API.

## Reproduction and retained outputs

From the repository root with the existing Python 3.8.10 project environment:

```sh
PYTHONDONTWRITEBYTECODE=1 python scripts/diagnostics/mcp_response_heartbeat_probe.py results.json
```

[Raw samples, render trace, count trace and cancellation results](mcp-response-heartbeat-20261014-results.json). Original run output and initial standalone probe remain in the workspace `gateway-encoding-audit-20261014` evidence directory. The committed-location probe consolidates the separately executed count pass into the same runnable script and was AST-parsed after consolidation, without re-running timings. CPU lane was exclusive for the retained timing series. No Git writes, hashes or production modifications were performed by this audit.
