# Forbidden overlay qualification: CPU passed, browser blocked

## Scope and source

- Baseline: `67c29be27a3800241b3cdf14dc7ec3a11d619206`
- Candidate source: `a56d0cfaad71c1d0ddb0ec8df02364ce3329b753`
- Qualification date: 2026-10-05 UTC
- This is a generic frontend change; no physical robot, backend, platform-specific model or device network was used.

The candidate eliminates repeated geometry/material replacement for unchanged forbidden-zone overlays during incremental pointcloud transactions. Only enabled outside surfaces depend on effective mask bounds. Independent review verified a necessary lightweight Mesh wrapper exception on edits/reorders to preserve Three's immutable-ID transparent tie ordering. Wrappers retain their geometry/material references and do not dispose them.

## Passed evidence

`node scripts/check_forbidden_overlay_reuse.mjs` exercises the actual viewer and bundled Three r170 in Node, tracking 291 resources with exactly-once disposal, real incremental page batches, bounds changes, effective-mask equivalence, edits/deletes, generation/map fences, duplicate IDs, hidden/reveal and actual Three transparent comparator ordering. The unchanged rectangle baseline reproduction incurred 5 geometry and 2 material disposals per identical update; the candidate retains all resources across 20 repeated real page batches.

Also passed: 21 incremental page checks, 12 wire-protocol checks, 10 transport tests, 6 integration tests, 4 native-wire fixture sequences, and the existing viewer lifecycle regression. The integration uses synthetic mmap fixtures, not a native build. System Python lacked `grpc`; the existing project test environment ran those integration checks successfully without installation.

These are CPU/resource/contract checks. Scheduled BufferAttribute writes are not measured driver uploads or GPU timings.

## Browser status: BLOCKED / NOT RUN

The cloud browser could not open the local fixture at `http://127.0.0.1:8765/`: `net::ERR_BLOCKED_BY_CLIENT`. A single retry after the documented shared-network server launch gave the same error. The matching elevated readiness command failed before curl execution:

```
bwrap: Can't mkdir parents for /root/.codex: Not a directory
failed to inspect synthetic bubblewrap mount target /root/.codex: Not a directory (os error 20)
```

An ordinary command's loopback readiness check returned connection refused, while the retained server session remained running, consistent with execution-context isolation. No supported forwarding API was exposed. No alternative browser, user computer, security-setting change, or network bypass was attempted. The owned server was stopped after this diagnosis.

Consequently there is no browser renderer identification, screenshot, actual WebGL upload count, pixel comparison, browser timing or GPU result to report. The fixture's planned assertions must not be described as passed.

## Reproducible offline browser fixture

Run from this checkout in an environment whose browser can reach its local server:

```
python scripts/serve_forbidden_overlay_webgl.py --output /tmp/neh-forbidden-webgl-qualification
```

Open the printed local URL and click **Run comparison**. The server materializes the exact baseline/candidate committed modules via `git show`, not mutable source files. It binds only `127.0.0.1`. No dependency installation is required. Override `--port`, `--baseline` or `--candidate` explicitly when needed.

The two same-origin iframes use actual viewers and Three, with mocked WebSockets and a deterministic 1024-point cloud. They invoke the production render loop synchronously for reproducibility; they do not measure requestAnimationFrame cadence. Instrumentation records actual WebGL2 buffer allocation/deletion/upload API calls and bytes, renderer draw/resource counts and rendered pixel SHA-256 comparisons. Pixel checks require a nonblank rendered scene. Scenarios cover bootstrap, repeated steady updates, effective outside bounds, earlier-zone disable/edit/delete, coincident reorder, map switch, camera movement and hidden/reveal. Captured images and full results are saved to the output folder's `result.json` after both runs.

The fixture records browser and available unmasked renderer information. SwiftShader/other software rendering results, if obtained, must be labeled software-rendered. CPU wall time, shader driver submission time and physical GPU execution time are distinct; this fixture intentionally makes no physical GPU speed claim.

Fixture-only validation passed: both inline JavaScript modules parse; an isolated local-server smoke verified HTTP serving, exact revision materialization and JSON result persistence. This did not exercise a browser or WebGL.

## Integration recommendation

The bounded optimization can integrate on its reviewed CPU/resource correctness evidence with the browser limitation explicitly disclosed. Do not characterize the broader frontend or GPU performance qualification as complete. Run this fixture through a supported reachable browser route before claiming actual rendering equivalence or driver-upload improvements.
