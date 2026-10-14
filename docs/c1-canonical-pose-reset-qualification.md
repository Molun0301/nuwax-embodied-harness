# Canonical pose reset: bounded native component qualification

Final result on 2026-10-05 UTC: **14/14 exact tests passed; zero ignored/skipped.**
Independent source review cleared both tests-only corrections before their runs.
Production source is unchanged from the reviewed source-only WIP checkpoint
`e0496ea1d823e3c22d3bb81c54309023656ed50b`.

## Build and source provenance

- Rust/Cargo 1.88.0; actual `lcm-gen 1.5.2`
- Cargo jobs 2, CMake jobs 1; dev/test debug information disabled
- SDK environment: `/workspace/shared/neh-native-sdk/env.sh`
- Shared target: `/workspace/shared/neh-cargo-target`
- Command: `cargo test --locked --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime --no-run -vv`
- Native executable inventory was checked for each of the 14 unique selectors
  in `c1-canonical-pose-reset-exact14.txt`; each ran individually with
  `--exact --nocapture --test-threads=1`
- A 120-second external per-test watchdog was available; none fired

Only the three reviewed Rust files were temporarily overlaid on the warmed,
clean restored V3 baseline `9334c2aae80651c832b0864c49080af8ffdc9243`, tree
`fa4c24311a5d87bf09cd103ff66d3f4f2edf8269`. All other Rust sources and manifests
match the candidate. The complete final tested overlay tree is
`ba255fd8b304337ae85ef6044bcefd0fad3bfd30`; Python/docs were not compiled.

After every compile, original baseline bytes were unconditionally restored with
fresh mtimes, then Git cleanliness/tree and both native CMake cache bytes/mtimes
were verified. The primary checkout was untouched. Cargo rebuilt only Geometry;
no dependency/native SDK rebuild or fallback download occurred. Compiler warnings
remain; this is not a warning-free build claim. The copied final executable is
labelled with its reviewed input manifest; the shared target must recompile when
next used against restored baseline source.

Final executable SHA-256:
`38ff5c85274b4964dd5cec262073b9334eff2e083fdf41789b45e336d537de80`

## Initial failures and reviewed corrections

1. Initial compile: 74.98 s; 12/14 passed. The replay fixture had replaced its
   cloud writer without binding the actual runtime/frame owner. First odometry
   therefore rejected before the expected TF publication. The successful-cloud
   fixture also returned `success=false`, initially without diagnostic detail.
2. Tests-only correction: bind the injected writer to the actual adapter frame
   and nav producer generation; expose returned outcomes without relaxing
   assertions. Compile: 28.73 s; 13/14 passed. Replay passed; cloud activation
   reported `ACTIVATE_MAP_SEAL_CAS_FAILED` on its legacy contract-0 request.
3. Tests-only correction: use the existing contract-1 request helper with the
   real activation runtime generation, matching the current Python provider
   route. Success/replay/rejection assertions remained intact. Compile: 23.57 s;
   **14/14 passed**.

The contract-0 cloud failure is an unresolved compatibility finding, not repaired
by this stage: publication stores a target receipt, the unversioned server path
passes no target receipt to seal, and the existing seal CAS rejects the mismatch.
The successful cloud-reset fixture now covers contract 1. The caller-cancellation
cloud fixture still covers contract 0 only through its pre-seal reset obligation;
it does not assert successful contract-0 activation.

`c1-canonical-pose-reset-results.json` preserves exact per-case results, source
hashes, tested trees, build restoration evidence, and both initial failed outputs.
Full execution logs remain under
`/workspace/shared/c1-canonical-pose-reset-20261005/qualification-v{1,2,3}/`.

## Scope and remaining limits

The checks cover clearing full canonical pose/TF history, explicit sensor-edge
retention, fresh observation recovery, real injected StartMapping/ActivateMap
handlers, retained reset across caller cancellation, and exact deferred replay
admission before odometry handoff. Registered producers, mapping workers and
separately retained fanout tasks are explicitly settled/joined by fixtures.

Raw native getters during an unfinished transition can still observe predecessor
state until reset commits. These results do not establish atomic cross-API
session visibility, native owner-labelled pose freshness, or later-publication
currentness. Existing startup-replay poisoning can still revoke readiness/close
correction phase A; rejection checks establish pose/TF/registration non-mutation,
not whole-successor isolation. The already-qualified Python composite sampler
complements this boundary by rejecting samples spanning its checked transition.

No real LCM/network, native sidecar process, robot, hardware, deployment, primary
integration, or full-suite acceptance was tested. Default assigned activation
and remaining identity migration are unchanged. Publication must be verified
separately before further implementation.
