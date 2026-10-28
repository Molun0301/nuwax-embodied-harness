# Cloud activation sealing and exact retained-publication recovery

Final bounded result, 2026-10-06 UTC: **25/25 exact tests passed, zero ignored/skipped**.
This resolves the contract-0 seal mismatch recorded by the previous canonical-pose
qualification and a separately exposed accepted-input publication-retry mismatch.
It does not enable the default schema-2 save/load workflow.

## Changes and supported callers

The native public classifier explicitly accepts contract 0 as compatibility input.
The current Python provider emits contract 1. Both initially sealing and resuming
publication now pass the actual installed target receipt to the unchanged strict
seal CAS. Public nonmodern projection keeps activation/target digests absent,
LegacyUnbound on initial/resume/query, and the existing SealedReplay disposition
on terminal replay. Relocalization eligibility remains false and immutable target
bodies are released. Contract-1 and assigned-2 proof checks are unchanged.

The failure test used accepted finite coordinates away from voxel centers. Initial
canonical publication emitted ordered voxel centers while the old retained retry
encoded the original request coordinates. The Python loader and native validators
do not require centered coordinates, so this was a real supported-input recovery
inconsistency, not an invalid fixture. The original input and exact byte assertion
were preserved.

Retention now starts with metadata only. After publication failure it can take an
Arc clone only from the writer's already-materialized canonical snapshot cache.
It validates the current metadata, cached metadata and exact cloud receipt, then
encodes the existing retained-frame buffer outside the writer lock. An existing
retained frame is never replaced. Cache absence retries the canonical sequencer;
identity mismatch remains an error with the pending lifecycle fence retained.
The cache accessor is exactly `get().cloned()` and never initializes a snapshot.
Successful/no-demand paths gain no new full-cloud materialization, and assigned
retention does not access the writer. No target is reinstalled to retry publication.

Production files changed: `server.rs`, `correction_coordinator.rs`, `pointcloud.rs`.
No new application hashes, arbitrary runtime limits, or obstacle-check changes.

## Qualification and preserved failures

Independent source/list review preceded every run and the one-token compile fix.
The selected tests are in `c1-legacy-cloud-seal-exact25.txt`: prior 14 qualified
pose/reset cases, two actual injected compatibility handlers, two new pure ledger
cases, four existing modern/assigned ledger cases, and three precise cache/receipt/
assigned-retention cases.

- Round 1: compile passed in 43.69 s; 21/22 passed. Noncenter retry byte equality
  exposed the genuine canonical-versus-request representation mismatch.
- Round 2: compile failed in 37.26 s, before tests, because a renamed `_target`
  binding was still used by the unchanged legacy journal verifier. The reviewed
  one-token rename back preserved that verifier.
- Round 3: compile passed in 35.72 s; **25/25 passed** with the original failing
  noncenter case intact. No watchdog expired.

Each selector was checked against the built test inventory and executed separately
with `--exact --nocapture --test-threads=1`. Tests use actual handlers, protobuf,
codec, ledger, in-process transport and real task/worker joins, without live LCM,
network, hardware, robot or native sidecar execution. This is not a full-suite pass.

Environment: Rust/Cargo 1.88.0, actual LCM generator 1.5.2, Cargo jobs 2, CMake jobs 1,
dev/test debug information disabled, existing SDK and shared target. Only Geometry
Rust targets rebuilt; both native CMake caches were unchanged. Existing compiler
warnings are not claimed resolved.

The clean warmed baseline was `9334c2aae80651c832b0864c49080af8ffdc9243`, tree
`fa4c24311a5d87bf09cd103ff66d3f4f2edf8269`. Five exact Rust files (three current and
two prior qualified files) were temporarily overlaid, with byte-for-byte backup
and unconditional restore using fresh mtimes. Git cleanliness/tree and native
cache bytes/mtimes were verified after every run. Primary was untouched.

Final tested native overlay tree:
`db20d039a8da94e523a9a398e3e537f3eef98c2f`

Final executable SHA-256:
`54ca676f6ac45525f101a9ad65ff7783dcf84e505babd081308fabbc0b640c42`

`c1-legacy-cloud-seal-results.json` preserves exact source/allowlist hashes, per-test
results, initial failed payload comparison and compiler diagnostic. Full logs are
under `/workspace/shared/c1-legacy-cloud-seal-20261006/qualification-v{1,2,3}/`.

## Next priority: the actual default save/load/recovery pair

Current call graph, inspected from this branch's unchanged Python production code:

- Save: gateway command → `MapWorkspaceService.finalize_active_map` →
  `PlatformGeometryRuntime.prepare_map_asset_bundle` → provider checkpoint export
  → native `export_mapping_backend_checkpoint`
- Load: workspace `activate_map` → legacy manifest-keyed journal/recovery → runtime
  `activate_map_asset_bundle` → provider `activate_platform_map` → native
  `ActivateMap` / `activate_map_r1`

Default preparation/export still selects schema 1. Native export rejects schema 1
once AR2 fine state exists. Schema-2 load currently fails at legacy manifest
validation and explicit provider/runtime activation gates. Workspace finalization,
catalog reconciliation, runtime fences and publication still consume manifest
identity. Changing only the export default would create saved maps the normal
loader cannot activate.

Smallest useful next delivery sequence, after this stage's verified publication:

1. Execute and qualify one explicit typed loader using the existing schema-2
   checkpoint reopen/member transfer/request encoder and native assigned activation
   branch. Include actual native export → retained artifact transfer → activation
   receipt checks. Audit exact transition, response-loss and stale-request behavior;
   do not infer safe redispatch or retirement from an unknown result or new endpoint.
   Keep defaults gated until the caller pair exists.
2. Pair ordinary save and load/recovery in one usable cloud workflow: prepare
   schema 2, create one immutable assigned candidate, finalize/reopen through the
   assigned repository API, and use the existing assigned catalog journal/receipt
   operations. Add discriminated assigned branches for manifest validation,
   activation fences, installed snapshots and workspace publication; never use
   empty digests or ID-as-hash adapters. Bind the exact sealed target through the
   existing prepared localization commit. Retain uncertainty across persistence
   failure and compare exact original requests/owners before reconciliation.
3. Qualify the public save → reopen/restart → load → display path with response loss,
   receipt-persistence failure and stale-callback barriers before switching cloud
   defaults. Sealed load initially means localization pending; it does not prove
   an applied alignment or navigation readiness. Pair real assigned relocalization,
   terminal recovery, fresh pose and ordinary navigation in the next evidence-backed
   cut. Planar remains its explicit existing schema-1 route.

Reusable current code includes the assigned repository/finalizer and immutable
asset owners, schema-2 checkpoint member store and request encoder, schema-3
assigned catalog journal APIs, runtime captured target, and localization prepared
commit/publication helpers. Durable journal schema 3 and native receipt schema 2
are different contracts. This is a plan grounded in surviving source, not a claim
that lost historical integration was restored.

Remaining limits from prior work still apply: raw getters can see predecessor
state during unfinished transitions; startup-replay poison can revoke readiness;
no native owner-labelled pose guarantee or full application-hash closure is claimed.
