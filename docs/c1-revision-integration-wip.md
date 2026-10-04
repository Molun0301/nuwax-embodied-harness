# C1 revision integration recovery checkpoint

Status: component-qualified work in progress; NOT deployable. Real-LCM integration and default typed activation/finalization remain release gates.
Base: 9688ed0473de0fb7bfdb73b12ca609dc756c2810. Branch: recovery/c1-revision-contracts-20261004.

## Staged scope
- Recovered C1 core: assigned immutable object IDs, explicit opaque legacy aliases, separate block/geometry/publication revisions, frozen pending IDs, immutable no-replace storage, AR1 reader and streaming AR2 writer. Production workspace manifest/lock not replaced.
- Producer reserves the existing cloud revision before sealing with its runtime/frame owner. Same-domain restore continues persisted revision. Retained roots pin exact missing-block requests and survive accumulator transfer; private work does not expose its root until accepted installation. Collection uses writer retention.
- Receiver candidate install follows complete validation, including duplicate-inclusion conflict checks. Collection is post-commit maintenance with an observable error; it cannot return a failed receive after installing a complete root. The cleanup-failure regression passes.
- NavigationMap3D v3/manifest v2 and missing-block RPC v2 bind complete root and block references; generated LCM C/Python source uses actual lcm-gen 1.5.2 fingerprints.
- Navigation consumers use evidence/geometry/occupancy owner revisions. Ingress retains immutable raw payload ownership for byte-exact same-publication replay conflict detection.
- Recovered explicit checkpoint-v2 native/Python source supports typed checkpoint members and AR2 without a digest adapter.

## Release dependencies and verification still required
- Default finalization and activation remain v1 in recovered checkpoint code; new AR2 fine export explicitly rejects v1. Pair default finalization/activation v2 and preserve explicit legacy-read compatibility before production use. This checkpoint is not an enabled rollout or completed migration.
- Unified Python protobuf bindings regenerated using pinned grpcio-tools 1.70.0, grpcio 1.70.0, protobuf 5.29.6; generator --check passes.
- Producer fixtures bind explicit owners and use current SealedMap/archive APIs; retained-root/private-work/same-domain-restore and first-frame lifecycle regressions compile and pass in focused native runs. Full real-LCM server execution remains network-blocked. Legacy activation, occupancy authority, goal identities, and other application hashing remain outside the currently staged closure.
- Focused navigation revision, geometry/evidence, replay, mask and scoped-reply tests pass. Real missing-block process execution remains network-blocked; ignored live recording/device tests have not run.
- Initial scoped validation: map-chunks 33/33; NavigationMap3D codec 10/10 including actual generated C/Python parity; protocol fingerprints 8/8; paired Python checkpoint/preparation tests 33/33. Historical recovered test results are not reused. The later lifecycle qualification below supersedes only the component counts explicitly rerun.
- Final passing logs: /workspace/shared/neh-validation/c1-map-chunks-20261004-r3.log, c1-codec-20261004-r5.log, c1-protocol-fingerprints-20261004.log; /workspace/scratch/71cdb9d17732/c1-python-finalization-final-20261004.log.
- The codec test requires lcm-gen 1.5.2 plus its headers/library. Baseline sysroot lcm-gen 1.3.1 reports unsupported --use-quotes-for-includes; failed attempts are preserved. Verified 1.5.2 environment: /workspace/scratch/71cdb9d17732/nuwax-embodied-harness/.nuwax-test-env; prepend bin to PATH and site-packages/include, site-packages/lib to CPATH/LIBRARY_PATH/LD_LIBRARY_PATH. No SDK/native graph was rebuilt.
- No edits were applied to the active build owner's checkout. No deployment, device action, archive snapshot, or remote write is performed by this worker.

## Default activation release dependency

Explicit Python v2 preparation/reopen is available with assigned checkpoint metadata and bounded canonical-cloud-to-PCD streaming. V2 activation fails before external mutation until a coherent native typed activation protocol exists. Required remaining integration: ActivateMapRequest and activation receipts, target/asset/policy identity, relocalization and terminal-query consumers, outer MapAssetBundle/asset-store validation, and SavedDenseCloudCache identity. Do not fill legacy SHA fields with assigned-ID bytes or flip defaults before these consumers are paired.

The producer retains prior committed root lookup across complete accumulator replacements, and GC remains in private preparation rather than the short accepted-installation lock. Native regression sources cover private candidate invisibility, old-root pinning, rebuild retention, same-domain archive revision-floor continuation, and the next private publication advancing exactly once. These tests await runtime validation.

## Controlled integration validation (before lifecycle follow-up)

- Exact original primary: 5a24498ba716cd9c89577ed0470a5af9fde965e2 / tree cdae9c8072fed2b52385e7bf147591c137ba1e8c.
- First validation merge: 036991cec64dd2c04363e397ddab3610d95bfff2 / tree 813a4bd55feb0043618536fa0d733388585b99e0. Native runtime test binary compiled in 1m57s; no SDK graph rebuild.
- Retained compiled-tree tests: pointcloud 38 passed / 4 ignored; assigned checkpoint assembly 3 passed; artifact store 7 passed; dense cloud 5 passed; focused revision/mask/replay consumers 9 passed.
- Nav chunk suite: 5 passed, dense doorway closed-loop failed in 523.16s. Exact unmodified primary comparison: 2 passed, same doorway failed in 529.87s with identical heading 0 / waypoint 287 / pose / quaternion / ReplanRequired reason. This observed failure is present on primary; no guard or doorway geometry was weakened.
- Real missing-block RPC process test is blocked before RPC by liblcm multicast route failure (Network is unreachable). No networking settings or provider substitution was performed.
- Publication fixture initially bound its frame before ingestion established it; audited six-line correction binds the actual committed frame before seal. Second merge 302bd4518edb6587d18f80c7f461bc9503dee0b1 / tree e0fc36bb4537e4f46745d1fdcc7e6e17a2e3a351: publication 3/4 passed, no-full-cloud-materialization 1/1 passed. Remaining publication assertion compared spatially sorted fine points to dense slot order; same two exact XYZ triples were reversed. Audited correction now compares aligned sorted 12-byte triple vectors and counts, preserving duplicates and bits. That correction still requires a rerun.
- Both controlled windows restored the exact primary branch/head/tree clean before handing it back. The planning owner then resumed independent diagnosis.

## Lifecycle follow-up and initial source checkpoint

The first-anchor transition is now prepared before mutation and carries an opaque MappingAdapter-issued source-owner proof (session, backend, boot nonce, current occupancy lineage, real frame transition, prepared transform). Canonical TF output is validated before changing either owner. Immutable roots can move to the next frame domain without object IO, cloud revision changes, or uncommitted evidence leakage. Dirty and pressure-flushed writer state is retained. In-flight private work receives the rehome only through its exact existing scan ticket and matching source lineage; actual coordinate rebuild/reset invalidates the old ticket.

Publication cancellation/dedup and consumer ingress recognize the distinct RootIdentity at the same cloud revision, while rejecting old-frame replay and same-identity byte mutation. The live odometry path publishes/retries that root without requiring another scan. The older mapping-feed helper is currently called only by tests.

Added source regressions cover first anchor with dirty/in-flight work, unchanged receipts/XYZ/object IDs, old-root readers, repeated-pose no-op, invalid timestamp/base/source rollback, backend reset rejection, actual coordinate rebuild rejection, activation→first real pose→explicit v2 export, and checkpoint flush. At the initial b96e5ce checkpoint these changes had only Rust parsing and whitespace checks. The later qualification below records their actual compile/test results; earlier counts are not reused as proof.

Pre-existing backend reset generation saturation at u64::MAX is separately identified; no broad reset API refactor is included. Default typed activation-v2/outer asset/relocalization identity migration remains the release gate described above.

## Third controlled window: fresh lifecycle qualification

Original primary was verified clean at 85f22475f837a19d266d6dfb37231d421d2d5dc1 / tree 73a7737ef7efe4ac08ae246185a10e787f315757. Temporary validation/c1-frame-lifecycle-20261004 merged exact C1 b96e5ce, producing c8e11e3dd1b0d3346a1dd1de17b7a63f84c1ec31 / tree 740fd1f411e8b37b180d2ce94a7f29eed876cdd0. Native test binary compiled in 1m43s using the warmed original-path target, without an SDK graph rebuild.

Fresh results on that compiled source:
- Map-chunks: 40 passed, including metadata-only rehome without object IO, dirty/future evidence, pressure-flushed membership and frozen retry IDs
- Pointcloud: 39 passed, 4 existing recording/performance/peer tests ignored
- MappingAdapter initial-frame proof, atomicity and rejection tests: 10 passed
- Runtime first-anchor/no-scan rehome, dirty/private-work progress, reset rejection, invalid-output rollback, no-full-cloud regression: 6 passed
- NavigationMap3D publication/receipt/clear/authority/mask regressions: 4 passed; exact-triple order assertion now verified

The nav test compile found two fresh-root diagnostic callers from primary requiring the original four-argument test helper. C1 c12f634 preserves that wrapper and gives the incremental helper its own install_revisioned name; no diagnostic geometry or assertions changed. Merged validation became 6daaff1846b8824548e887b5ce6a80f020bcf777 / tree 7913ca89b60d27b2db6e2fd8ca835e5992047859. Fresh nav tests then passed: 10 revision/mask/replay/domain cases and 5 chunk cases. The stale earlier test binary was not counted as validation of this failed/rebuilt test target.

Two new full server integration tests were attempted and both failed before test-body execution because real liblcm could not create the multicast transport (Network is unreachable). The full missing-block RPC process has the same verified environment blocker. No route/firewall/security settings or provider substitutions were used. A proposed reviewed elevated run was not attempted after inspecting the old test URL formula: PID-dependent isolation can select production-default port 7667, so the required isolation precondition was not established. These are blocked integration checks, not passes and not missing-SDK claims.

Final log set under /workspace/shared/neh-validation:
- c1-frame-core-20261004.log
- c1-frame-native-compile-20261004.log
- c1-frame-pointcloud-20261004.log
- c1-frame-adapter-20261004.log
- c1-frame-runtime-lifecycle-20261004.log
- c1-frame-server-integration-20261004.log
- c1-frame-nav-domain-20261004.log (preserved compatibility compile failure)
- c1-frame-nav-domain-20261004-r2.log
- c1-frame-nav-consumers-20261004.log

Retained native binary c1-runtime-tests-c8e11e3d and nav binary c1-nav-tests-6daaff18 are hardlinks to those exact compiled artifacts, not independent builds. Primary was restored to exact 85f22475/tree73a7737e and verified clean before returning the native lane. No deployment or default-v2 switch occurred.
