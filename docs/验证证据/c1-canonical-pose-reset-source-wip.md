# Canonical pose reset: historical source-only WIP checkpoint

This records the original unqualified checkpoint. Subsequent bounded results are
in `c1-canonical-pose-reset-qualification.md` and `c1-canonical-pose-reset-results.json`.

Status: **UNQUALIFIED — NOT COMPILED OR EXECUTED** (2026-10-05 UTC).
This isolated branch checkpoint preserves reviewed source while the native
validation toolchain is being restored. It is not a primary merge, release,
deployment, or evidence that the tests pass.

## Base and scope

- Local base: `5b6776ab9a759809a25fa26fa934e7a5ab3aaf08`
- Base tree: `0058c4910200f0dc5c8498d667bbd5ce8ef16a5f`
- Corresponding verified remote base: `e318634390d7a8464d10f1234e4872947b67aca4`
- Branch: `fix/v3-canonical-pose-reset-20261005`
- Production files: `lcm_runtime.rs`, `server.rs`, `lio_ingress.rs` under
  `services/geometry/nuwax_geometry_runtime/src/`

The retained new-session transition clears canonical pose and the complete TF
history/mirror while retaining the existing mapping lifecycle lock, before
replay, new-map publication, or successful completion. It does not hold the
shared-state write lock across worker creation or retirement. Replays and
pre-start rejection bypass reset. The planar callsite reinstalls only the
explicit configured sensor descriptor; timestamp zero is not static authority.
Deferred replay checks the captured session, map, session generation, exact
frame generation, and existing stop/mode admission under the lifecycle lock
before canonical odometry handoff.

## Review and planned execution

Independent source review cleared the frozen three-file candidate and the exact
14-case allowlist on 2026-10-05. `git diff --check` passed. **No native build or
test execution has occurred.** Compilation may still reveal type/build errors,
and execution may reveal behavioral failures.

The byte-pinned allowlist is `docs/c1-canonical-pose-reset-exact14.txt`:
4 new pure reset tests, 4 new actual-handler injected tests, 3 new injected
replay tests, and 3 existing pure TF tests. All selectors belong to the
`nuwax-geometry-runtime` main unit-test binary. Each must be checked against the
built binary inventory and run with exact-name selection, without broad prefixes.

The injected test constructor selects its in-process backend before the real
LCM fallback. Server fixtures use the existing task supervisor to join registered
producers, then retire/join mapping workers. Separately retained fanout handles
are aborted and joined before fixture files are removed. Replay fixtures use
in-process publication and the actual callback/ACK path. These are planned
component checks, not network, robot, native sidecar, or hardware qualification.

## Limitations

- Raw native getters during an unfinished transition can observe predecessor
  state until reset commits; this is not atomic cross-API session visibility.
- No native owner-labelled pose or native freshness guarantee is added.
- Existing deferred-startup poisoning remains: a stale pre-handoff error may
  revoke readiness and close correction phase A. Rejection assertions cover
  pose/TF/registration non-mutation, not whole-successor isolation.
- The separately qualified Python composite sampler rejects reads spanning its
  checked Python connection/transition interval; it does not establish native
  pose ownership or currentness through later publication.
- Assigned activation/default gates, genuine planar implementation, and remaining
  application-identity migration are outside this checkpoint.
- Do not broaden execution or stack further implementation before this stage is
  qualified and its exact resulting branch commit is remotely verified.

## Frozen source provenance

- `services/geometry/nuwax_geometry_runtime/src/lcm_runtime.rs`: SHA-256 `7ec60f6145a25cdceea92af493a5d3d450855f2409a4e2093c39509b37ce82de`
- `services/geometry/nuwax_geometry_runtime/src/lio_ingress.rs`: SHA-256 `0f2b5a0cf58939c7099cb53231fbe608e634c0019ffa8bb51854e530f2409784`
- `services/geometry/nuwax_geometry_runtime/src/server.rs`: SHA-256 `2d1a4b101e89f0e1d66eeb21defb1f10e345e8f0f33737d29ff309668398546f`

- Source patch SHA-256: `f60873c910911cf6921f596dd0077047949a1b28ced10519a6838509607b8e55`
- Exact14 allowlist SHA-256: `5ada43d540345cd282b164705ca9e4546799dd40bc5c91ada98e6075468b540e`

These hashes identify reviewed source artifacts; they are not runtime application
identity or test results.
