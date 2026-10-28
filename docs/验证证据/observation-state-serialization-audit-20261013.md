# Observation, robot-state and HTTP result serialization audit

> 历史验证记录：文中一次性实验及已失效检查入口已于2026-10-22防御性编码优化中退役；下列旧命令仅说明当时证据，不作为当前运行入口。

## Decision

No production changes. Two bounded test-only candidates expose repeated work but do not justify widening the service contract or complicating exact transport-size enforcement. No end-to-end latency benefit is claimed.

## Source findings

- `services/observation_service.py`: robot mode lookup uses the existing O(1) `StateStore.read_latest`. Observation contexts are retained references with separate artifact ownership holds, rather than repeated deep copies. Latest-camera sorting and up-to-default-100 history-pair assembly happen under the condition lock, but no representative cost justifies changing ownership-sensitive paths.
- `services/robot_state_service.py`: each ingest requests adapter health for diagnostics, concrete assembly `get_status`, and metadata `adapter_count`. `drivers/common/assembly.py` builds every status through `adapter.health_check`. This is three sweeps, not evidence of a blocking defect. The base health check creates lightweight models; Slamtec additionally takes its lifecycle lock and checks current scan freshness.
- Reusing an earlier diagnostics/status count shifts the sampling point, so it was rejected without timing. Public `bind_adapter` is not restricted to initialization by contract. The measured candidate instead substitutes `len(self._robot.list_adapters())` at the original metadata sampling point. All existing concrete assemblies inherit matching registry-based listing/health methods. However, `RobotAssemblyProtocol` does not promise `list_adapters`, and adding an optional compatibility branch or a new protocol requirement is not justified by this absolute gain.
- HTTP capability responses call `build_mcp_tool_result(result)["structuredContent"]`. The shared builder creates full public JSON text and serializes the complete result envelope for exact byte enforcement. Discarding MCP text on HTTP cannot simply bypass that existing limit. The measured candidate counts a real placeholder envelope plus the already-created public JSON text length, minus serialized `null`, retaining the exact check. This only avoids the second traversal of `structuredContent`; recursive conversion/redaction remains.

## Measurements

Python 3.8.10, existing cloud checkout, production `StateStore` and builder functions. Offline synthetic workloads, no artificial sleeps, devices, subscribers, hardware loads, or network requests. Alternating implementation order; raw wall and process CPU timings retained. No manual/content hashes were calculated.

Robot-state diagnostic: seven rounds × 200 ingests, real `AdapterBase.health_check` and production `RobotAssemblyStatus` shape. Baseline/candidate results match excluding generated snapshot/diagnostic timestamps. Health sweeps are verified as 3 versus 2; a registry-growth fixture during `get_status` preserves metadata count 2 in both candidates.

| Adapters | Baseline median | Candidate median | Median reduction |
| --- | ---: | ---: | ---: |
| 0 | 79.94 µs | 76.37 µs | 4.47% |
| 4 | 140.26 µs | 125.99 µs | 10.17% |
| 16 | 298.94 µs | 220.93 µs | 26.09% |

The largest absolute saving is about 78 µs per synthetic ingest. This is not a measured robot bottleneck and does not justify an interface/compatibility change.

Full public result builder: five alternating rounds × 10–30 calls; fixtures are escaped/Unicode observation text and nested observation lists. Exact encoded envelope sizes are below 512 KiB.

| Fixture | Bytes | Baseline median | Candidate median | Median reduction |
| --- | ---: | ---: | ---: | ---: |
| Text × 32 | 1,818 | 0.106 ms | 0.110 ms | -3.51% |
| Text × 512 | 25,338 | 0.589 ms | 0.610 ms | -3.70% |
| Text × 8,192 | 401,658 | 9.138 ms | 8.608 ms | 5.80% |
| List × 4 | 1,026 | 0.246 ms | 0.235 ms | 4.35% |
| List × 64 | 14,454 | 3.206 ms | 2.856 ms | 10.93% |
| List × 1,024 | 231,390 | 53.038 ms | 50.807 ms | 4.21% |

The mixed, small improvements do not justify more complex size accounting. The measured candidate remains test-only and unchanged. Its early placeholder-size branch preserves invalid-Unicode sentinel handling; scalar texts shorter than `null` yield envelopes well below the enforced 256-byte minimum. This extra reasoning burden is another reason not to promote it.

## Correctness and reproducibility

`python scripts/check_tool_result_size_reuse.py`: three grouped checks passed, including scalars, dataclasses, mappings/lists, enums, Unicode/escapes, redaction, tool errors, exact boundary −1/at/+1 comparisons, NaN/Inf, cycles, binary values, duplicate keys, unpaired surrogates, and independently owned result trees. This is focused experiment coverage, not a full production test-suite pass.

Both benchmark scripts have main guards to prevent import-triggered measurement. An accidental import reran robot timings once before that guard was added; those repeated `/tmp` results are not used. The original persisted robot evidence was preserved.

Reproduce in the project toolchain environment:

- `python scripts/benchmark_robot_state_health_sampling.py <output.json>`
- `python scripts/check_tool_result_size_reuse.py`
- `python scripts/benchmark_tool_result_size_reuse.py <output.json>`

Raw results: `robot-state-health-sampling-20261013-results.json` and `tool-result-size-reuse-20261013-results.json` alongside this report; original captured files also remain in the workspace `neh-evidence` directory. Future timings need an exclusive CPU lane.
