# C1 revision integration recovery checkpoint

Status: source-only work in progress; NOT build-validated or deployable.
Base: 9688ed0473de0fb7bfdb73b12ca609dc756c2810. Branch: recovery/c1-revision-contracts-20261004.

## Staged scope
- Recovered C1 core: assigned immutable object IDs, explicit opaque legacy aliases, separate block/geometry/publication revisions, frozen pending IDs, immutable no-replace storage, AR1 reader and streaming AR2 writer. Production workspace manifest/lock not replaced.
- Producer reserves the existing cloud revision before sealing with its runtime/frame owner. Same-domain restore continues persisted revision. Retained roots pin exact missing-block requests and survive accumulator transfer; private work does not expose its root until accepted installation. Collection uses writer retention.
- Receiver candidate install follows successful validation and collection, including duplicate-inclusion conflict checks. A cleanup-failure regression is included but unrun.
- NavigationMap3D v3/manifest v2 and missing-block RPC v2 bind complete root and block references; generated LCM C/Python source uses actual lcm-gen 1.5.2 fingerprints.
- Navigation consumers use evidence/geometry/occupancy owner revisions. Ingress retains immutable raw payload ownership for byte-exact same-publication replay conflict detection.
- Recovered explicit checkpoint-v2 native/Python source supports typed checkpoint members and AR2 without a digest adapter.

## Release dependencies and verification still required
- Default finalization and activation remain v1 in recovered checkpoint code; new AR2 fine export explicitly rejects v1. Pair default finalization/activation v2 and preserve explicit legacy-read compatibility before production use. This checkpoint is not an enabled rollout or completed migration.
- Python protobuf bindings need regeneration using pinned grpcio-tools 1.70.0; unified proto descriptor generation succeeded with protoc, but stale checked-in bindings are not sufficient.
- Producer test fixtures still need explicit owner binding and migrated SealedMap/archive APIs. Runtime-wide compile errors may remain. Legacy activation, occupancy authority, goal identities, and other application hashing remain outside the currently staged closure.
- Navigation ignored live-diagnostic RPC fixture and additional focused consumer regressions remain.
- No cargo/native build, runtime test, or integration test has run on this tree. Historical recovered test results do not validate it. Full acceptance requires fresh core/codec/nav/runtime tests, wire/version-skew tests, restart/failure/GC tests, paired Python checkpoint tests, and static application-hash audit.
- No edits were applied to the active build owner's checkout. No deployment, device action, archive snapshot, or remote write is performed by this worker.
