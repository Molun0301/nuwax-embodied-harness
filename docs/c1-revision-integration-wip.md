# C1 revision integration recovery checkpoint

Status: partially validated work in progress; NOT deployable.
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
- Producer fixtures now bind explicit owners and use current SealedMap/archive APIs; retained-root/private-work/same-domain-restore regressions are added but native runtime compilation and execution remain pending. Runtime-wide compile errors may remain. Legacy activation, occupancy authority, goal identities, and other application hashing remain outside the currently staged closure.
- Navigation ignored live-diagnostic RPC fixture is migrated and focused consumer regressions are added; execution remains pending.
- Fresh scoped validation: map-chunks 33/33; NavigationMap3D codec 10/10 including actual generated C/Python parity; protocol fingerprints 8/8; paired Python checkpoint/preparation tests 33/33. Historical recovered test results are not reused. Native/runtime and nav integration tests remain pending controlled original-path validation.
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

## Unvalidated lifecycle follow-up in this checkpoint

The first-anchor transition is now prepared before mutation and carries an opaque MappingAdapter-issued source-owner proof (session, backend, boot nonce, current occupancy lineage, real frame transition, prepared transform). Canonical TF output is validated before changing either owner. Immutable roots can move to the next frame domain without object IO, cloud revision changes, or uncommitted evidence leakage. Dirty and pressure-flushed writer state is retained. In-flight private work receives the rehome only through its exact existing scan ticket and matching source lineage; actual coordinate rebuild/reset invalidates the old ticket.

Publication cancellation/dedup and consumer ingress recognize the distinct RootIdentity at the same cloud revision, while rejecting old-frame replay and same-identity byte mutation. The live odometry path publishes/retries that root without requiring another scan. The older mapping-feed helper is currently called only by tests.

Added source regressions cover first anchor with dirty/in-flight work, unchanged receipts/XYZ/object IDs, old-root readers, repeated-pose no-op, invalid timestamp/base/source rollback, backend reset rejection, actual coordinate rebuild rejection, activation→first real pose→explicit v2 export, and checkpoint flush. These new changes have only Rust parsing and whitespace checks so far, not compilation/execution. The earlier passing counts above must not be treated as validation of this new follow-up.

Pre-existing backend reset generation saturation at u64::MAX is separately identified; no broad reset API refactor is included. Default typed activation-v2/outer asset/relocalization identity migration remains the release gate described above.
