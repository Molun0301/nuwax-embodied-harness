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

## Typed activation prerequisite, Stage A (2026-10-04)

This additive stage starts from 1663800662a74849966f76de9b5403505f91e31c. It does not enable activation-v2 or change defaults, and remains nondeployable.

- AssetIdentityV2 represents one immutable finalized asset with one nonzero assigned ID. Asset replacement obtains a new ID; no competing revision clock is introduced. Checkpoint IDs and asset IDs use distinct native types despite sharing the existing encoding.
- MappingBackendCheckpoint now carries explicit backend_state and canonical_occupancy member identities, in addition to the cloud/artifact descriptors. The export assembly owner binds all three to the existing checkpoint ID, paired commit receipts and frame. Missing descriptors, mismatched roles/receipts/frame, and mixed legacy identity are rejected.
- Python persists and reopens those exact descriptors. Pre-Stage-A v2 manifests already serialized backend/occupancy identities in their member inventories: compatibility is confirmed by source comparison and a frozen fixture generated by the unmodified 166380 serializer. The fixture uses deterministic test bodies; it is not a historical user map or a historical runtime test result. Current roundtrip and old-inventory reopening are tested.
- Known wire-generation mismatch is rejected before optional artifact downloads. Old live v2 envelopes missing the required member fields fail explicitly and require a paired client/runtime upgrade. Legacy saved readers remain separate.
- Immutable target admission owns the moved cloud Vec inside a private Arc-backed object. This avoids the full allocation copy incurred by Vec-to-Arc-slice conversion. Clone shares immutable ownership; conflicting retry compares exact descriptors and bytes. Existing size/frame/voxel limits and receipt semantics remain in force.

Independent checks so far: pinned protobuf generation and --check pass; Python checkpoint suite 39/39 passes in an isolated import harness that skips unrelated eager package initializers but runs the actual checkpoint/provider/runtime modules. Log: /workspace/shared/neh-validation/c1-stage-a-python-isolated-20261004-r6.log. Ordinary discovery was separately blocked during eager package import by missing cv2; an initial isolated invocation lacked grpc until the already-installed pinned tools path was supplied. Those blocked invocations are preserved separately. Rust source parsing passes. Native qualification is recorded below; the Python/codegen checks themselves used no primary-checkout build.

Remaining paired chain: activation exact-request admission/journal/receipts; correction candidate/relocalization source and evidence ownership; crash-safe durable terminal v2 with explicit legacy reads; outer finalized asset publication and active-map/goal authority consumers; planar assigned archives; only then the working default-v2 switch and full application-hash audit. IDs identify ownership and are not cryptographic integrity proofs. Real authentication/cryptography and generated protocol fingerprints are preserved.

### Stage A native qualification

The exclusive native owner handed off clean primary 0ea9997753cc0a7e19b4c27ce853dce8b5b10fd9/tree 398019c0c188efb6e23d8a718fe6b8679b5ef72e. In the original checkout, validation/c1-activation-stage-a-20261004 merged frozen Stage A 03717a4523613f2d3fc93c536fc5e9ef4faafee3, producing merge 23268d7cc99d57a831e6e9bad1588a24f0acde72/tree 150ab34d5243e81286c2a6900dfbfe6fa072c2f3. Warmed native --no-run passed in 1m42s without rebuilding the SDK graph.

First execution: checkpoint assembly 4/4 passed; immutable target 1/9 passed and eight failed at shared fixture admission because the supposed 32-digit assigned ID literal contained 34 digits. The strict production parser correctly rejected it. Test-only commit 497f2a8e8911a944bd666199e1d0002b57501afa corrected that one literal, with no production/guard changes. Original failure logs are retained.

Final merge 921808142d458a0e3db78328e425236af2b18ca4/tree 4337cafb7f8752581075457f83150c8e7ea244d5 compiled in 1m19s and passed activation_identity::tests 9/9 plus server::assigned_checkpoint_assembly_tests 4/4. The retained binary is /workspace/shared/neh-validation/c1-runtime-tests-92180814. Tests use in-memory or test-owned fixture data; no real server/multicast or hardware integration is claimed.

Commands used the existing baseline-build-env.sh, warmed geometry-runtime target, jobs 2 and default native feature set:
- cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime --no-run
- cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime activation_identity::tests -- --test-threads=1 --nocapture
- The resulting exact binary with filter server::assigned_checkpoint_assembly_tests --test-threads=1 --nocapture

Logs under /workspace/shared/neh-validation: c1-stage-a-native-compile-20261004.log; c1-stage-a-native-target-tests-20261004.log (original failure); c1-stage-a-native-target-tests-20261004-r2.log; c1-stage-a-native-assembly-tests-20261004.log and -r2.log.

At 14:42 UTC the primary branch recovery/exploration-overlay-20261004 was restored to exact 0ea9997753cc0a7e19b4c27ce853dce8b5b10fd9/tree 398019c0c188efb6e23d8a718fe6b8679b5ef72e and verified clean; the native lane was explicitly returned. Runtime default-off experiments and diagnostic test helpers were preserved. Default typed activation/finalization remains a release gate.

## Typed activation Stage B component checkpoint

This stage starts from bad2ee331e3f632993a7caaae5f92db37bbd7a82. Its initial source-only checkpoint was followed by the native qualification below. It remains nondeployable; the Python/default activation gate is still closed.

- Explicit native contract 2 carries one canonical checkpoint envelope and a separate typed finalized-asset ID. Legacy top-level duplicate cloud/occupancy bodies and digest fields are rejected. Python populates the nested protobuf in place, with one materialization per body and no second intermediate cloud/checkpoint CopyFrom. The isolated Python suite passes 41/41 in c1-stage-b-python-20261004-r3.log.
- The immutable request owner moves backend, occupancy and cloud vectors once. A weak interner compares exact candidate bodies outside short locks, shares equal live ownership, and rejects conflicting candidates. Pinned read-only artifact files remain authoritative across transfer release/expiry; exact-reference retries cannot switch to a recycled transfer-store body. Fifteen new source tests cover ownership, races, replay, expiry and conflicts.
- One existing activation ledger retains the current source owner; tokens, journal keys, adapter/publication fences and permanent terminal records contain small typed metadata/receipts only. Retired tombstones are checked before expensive source re-admission. Existing lifecycle epoch/CAS and retirement behavior remain the authority. Assigned journal variants are step-bound, cloud proofs compare exact installed receipts, and Applied entries cannot be changed or downgraded.
- Assigned backend restore jobs borrow the admitted owner through a small read-only trait shared by binary/library worker code. The mutable installed occupancy grid is decoded from a borrowed source rather than cloning its protobuf first. No new clock or retention cap is introduced.
- Assigned publication fences retain only key/metadata/receipt, not an eagerly encoded full legacy XYZ frame. Retry goes through the same demand-aware canonical publication sequencer after exact receipt validation. Legacy retry behavior remains isolated.
- Existing legacy relocalization and mapping-cloud/goal authority consumers do not receive fabricated v2 digest snapshots. Typed relocalization/evidence, persistent outer assets and planar ownership remain required downstream work.

Important audit limit: common occupancy installation still calls legacy MapRevisionEvent raw/effective hashing, and canonical navigation publication still produces a legacy committed-map payload digest. This stage therefore does not establish hash-free live activation. Those producers and their active-map/goal consumers must migrate together before default activation/finalization is enabled or the application-hash removal is claimed complete. The weak interner also makes no permanent-global-identity claim after every owner is released; persistent finalized-asset ownership and legacy import must supply that later boundary.


### Stage B native qualification

The exclusive owner granted clean primary 326fd0b2bfd996cd961ae1d85740739ae15bb60d/tree 5d3e9e8d251f0b67594c6e9c3d22a63be8a0416b. Temporary branch validation/c1-activation-stage-b-20261004 merged exact 0f58859e35be7c23468828e9ea4bf9bd6acd82c5 into that primary, producing merge d92a3e6c96c1eb909be4a467b49f0d4f7b4be824/tree 273857e892a6584313e8d7c26a97ba9704cf5995. Warmed default-feature native --no-run passed on the first attempt in 1m51s; no SDK rebuild or source fix was needed.

The exact compiled binary passed 43 selected pure tests: 9 immutable target, 15 request/checkpoint ownership, 10 assigned ledger/journal, 3 existing legacy ledger/release, and 6 server metadata/publication-fence/checkpoint-assembly. Tests verify allocation sharing, weak/retired release, artifact expiry/reference ownership, conflicting exact replay, typed proof pairing, journal mutation rejection and phase-A/replay barriers. They do not execute a real-LCM activation transaction or establish hash-free live publication. No doorway/full-server tests ran in this window.

Commands sourced baseline-build-env.sh and used the existing warmed target/jobs2:
- cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime --no-run
- The retained binary c1-runtime-tests-d92a3e6c with --test-threads=1 --nocapture and filters activation_identity::tests, activation_request::tests, correction_coordinator::preflight_only_contract::tests::assigned_, correction_coordinator::preflight_only_contract::tests::activation_, server::assigned_

Evidence: /workspace/shared/neh-validation/c1-stage-b-native-compile-20261004.log; c1-stage-b-selected-tests-20261004.txt; c1-stage-b-pure-tests-20261004.log. Python41/41 and pinned generated-protobuf check also pass on the source stage.

At 15:32 UTC, primary recovery/exploration-overlay-20261004 was restored to exact 326fd0b2bfd996cd961ae1d85740739ae15bb60d/tree 5d3e9e8d251f0b67594c6e9c3d22a63be8a0416b and verified clean, and the native lane was explicitly returned. Common occupancy/committed-map hash identity, active-map/goal consumers, typed relocalization/evidence persistence, planar and outer-asset/default closure remain open.

## Separate real-LCM test fixture isolation repair (source-only)

The test-only helper now selects 239.254.76.67 with TTL=0 and a u16-safe dynamic port in 49152..65535; production default port 7667 and values above 65535 are excluded. Each handle owns a UUID-named temporary directory for sequence state and both mmap files, plus a per-handle test occupancy topic. A pure full-port-cycle regression test has been added but has not run yet.

The missing-block process fixture uses 239.254.76.68, TTL=0, a probed dynamic port, and a random per-test RPC namespace validated in both parent and child before real-LCM construction. It installs the ordinary real RpcClient with that same test name; transport, wire protocol, retries and functional assertions remain unchanged.

The two selected geometry tests are server::lifecycle_tests::activation_first_real_pose_exports_v2_without_another_scan and server::lifecycle_tests::checkpoint_v2_flush_keeps_the_actual_first_pose_frame. Source audit confirms direct service calls with an in-process Voxel backend, no production server/sensor runtime/driver startup, and no actuator publication or motion-guard socket calls. The process RPC test is surface_chunks::rpc_process_tests::real_process_rpc_recovers_late_missing_and_corrupt_blocks_without_partial_install; its child invokes only the exact ignored map_block_server_process test and test-owned temporary repositories.

Limit: geometry /global_map and /nav/map3d topics remain hardcoded, although all selected publications use the explicit test multicast bus. TTL=0 is not a security boundary; a local process deliberately configured for the same endpoint can receive them, finite geometry ports can collide, and the RPC free-port probe has a release/rebind race. Thus this repair establishes default-endpoint exclusion and absence of actuator calls, not universal local isolation. No networking, native build/test, elevated invocation, route/firewall change or alternative transport was used for this source repair. Reviewer/root must decide whether the bounded elevated-run precondition is satisfied before any such invocation.

### Isolated fixture qualification and reviewed network limit

At the next exclusive window, clean primary 36ea5205d8f526500318c21e31dcebc3660a3458/tree 4db5bcda2af2f86fd22b7d8a9a77fe93d4c66e04 merged exact 1c0c9054713a9ccd1e5f23c2e2f07df39ec6342c on validation/c1-isolated-fixtures-20261004. Merge 406535311ee85e5760ba3615eb0ee4f5f561d1b1/tree 423ed08fee7d3235674f7cb23c1d0cf41d79dbbd compiled native in 1m48s and nav-stack --features mls --lib in 28s, using the warmed target/jobs2 without SDK rebuilding.

The pure endpoint range/default-exclusion test passed. Ordinary execution of both exact lifecycle filters and the missing-block process filter remained blocked at real liblcm initialization by Network is unreachable. The approved single reviewed elevated invocation was admitted, but its first exact lifecycle filter encountered the same network blocker. The command stopped immediately; the other two elevated filters were not attempted. No second elevation, alternative transport, route/firewall/VPN change, or hardware runtime was used. Full real-LCM integration remains an environmental gate, not a passing result.

Retained binaries: /workspace/shared/neh-validation/c1-runtime-tests-40653531 and c1-nav-tests-40653531. Logs there: c1-isolated-native-compile-20261004.log; c1-isolated-nav-compile-20261004.log; c1-isolated-native-ordinary-20261004.log; c1-isolated-rpc-ordinary-20261004.log; c1-isolated-reviewed-first-pose-20261004.log.

At 16:08 UTC, original primary was restored to exact 36ea5205d8f526500318c21e31dcebc3660a3458/tree 4db5bcda2af2f86fd22b7d8a9a77fe93d4c66e04 and verified clean, and the native lane was returned. Subsequent composite-provenance source edits in the staging worktree were not part of this merged test tree.

## Stage C: composite live occupancy/cloud provenance (source checkpoint)

This stage removes the common occupancy-event SHA and shared committed-map payload SHA from their live producer/wire/consumer boundary. Prepared occupancy bytes carry no authority until sealed with the existing MappingCommitClock receipt before state installation. Effective, separate raw and fallback views are explicit; the gRPC stream is schema 2 and rejects missing receipts or old live envelopes. The SHM payload layout remains unchanged at version 1. The coalescer compares conflicting immutable replay bodies outside authority/watch locks, then rechecks the small authority snapshot under the existing lifecycle fence before atomically exposing SHM metadata, authority and watch state.

NavigationMap3D version 4 carries current occupancy provenance and the retained cloud's actual occupancy baseline. Mask-only updates preserve cloud source time, root, block evidence and cloud revision; unchanged roots send no included objects. Composite ordering accepts independently progressing occupancy and cloud components, preserves explicit-clear watermarks, and treats opaque boot nonces as equality-only domains. Goal/Path version 7 and PlanarNavigationMap version 3 carry the same typed committed-map receipt. Generated C/Python bindings and fingerprints come from actual lcm-gen 1.5.2; no handwritten fingerprint adapters were used. Mapping-cloud active-map/goal-intent DTOs become version 2 with an explicit protobuf schema/receipt boundary.

The committed-watch publisher carries its exact admitted exposure into the existing canonical sequencer. It releases lifecycle before transport waits; an intervening owner commit cannot be recaptured as a newer unexposed publication. Direct activation/checkpoint callers retain their existing owner-fenced entry and do not await the asynchronous SHM coalescer while holding its lifecycle fence. Source tests cover pre-send recapture, blocked A with concurrent unexposed B, mask-only cancellation, lifecycle progress, and the exact activation publication entry with authority still unexposed. These Rust tests have not yet been compiled or run at this checkpoint. New deterministic component tests use an explicit injected in-process transport before handle construction, with real-LCM availability and actuator readiness false; all existing real-LCM integration constructors remain unchanged. This component coverage is not a substitute for the blocked real-LCM lifecycle tests.

Python preserves the typed receipt through provider, runtime and MappingService instead of hashing cells, synthesizing revision strings, summing independent revisions or incrementing an identity clock. Exact replay admission retains immutable byte ownership; mutable grid/metadata shells are copied separately without expanding packed cells. An unbound unary response cannot overwrite the receipt-bound stream cache. Live views lacking required provenance are explicitly unavailable, while durable legacy reads remain supported. New delivered occupancy still performs full-grid validation/counting and exact physical-change comparison; this is not an incremental occupancy-delta claim.

Source-only qualification: 67 tests pass in the isolated Python namespace harness (15 provider receipt/replay/ownership/race cases, 11 runtime/service receipt cases, 41 checkpoint compatibility/ownership cases). Log: /workspace/shared/neh-validation/c1-stage-c-python-combined-20261004.log. The runtime/service helper also reported 28 existing snapshot/ownership/prepared regressions passing separately; unrelated eager Gateway/cv2 and transport cases were excluded. Pinned protobuf regeneration check and Rust source parsing passed. Authentic generated GoalEnvelope.py/PathEnvelope.py include generator-produced EOF blank lines; those are retained byte-for-byte.

Limit: this source checkpoint is uncompiled on the integrated native tree and not deployable. Separate goal/config/boundary/path payload identities, native PGO/relocalization proof identities, outer asset/default activation/finalization and durable planar identity closure remain release gates. The prior real-LCM network blocker is unchanged; no retry or alternative transport route is claimed by this stage.

### Stage C integrated qualification and exact handback

An exclusive original-path window began from clean primary ec8affe9942e45742e09863fb93b3472191eef4c/tree acc79894f6102e0578aaa8f0c21157b129f990b9. Frozen source 093d07c4d2c75b4c2c38d4bef64ec229e042ec35 merged without conflicts on validation/c1-composite-provenance-20261004 as bc68a6e2d9c52f64f0e558f3c9351d1bd5eecbad/tree cb03c132b1ffe1f5775d16cfe4e201fd4f0f34fa. All source fixes were committed in the separate staging branch, then merged; the primary branch was never rewritten.

Qualification results and preserved failures:
- Codec first run: 91 passed/3 failed because generated C/C++ tests lacked the existing sysroot header search path. Adding C_INCLUDE_PATH and CPLUS_INCLUDE_PATH=/workspace/shared/neh-native-toolchain/sysroot/usr/include produced 93 passed/1 failed. The remaining parity fixture independently incremented producer generation while retaining its fine root, correctly rejected by root/domain validation. Test-only 3654385d71675dc41be4beb613f4f7346aa3ee9a preserves exact fine bytes and retains mutation checks on non-fine cases. Final codec: 94/94, including actual C/C++/Python generator parity, on merge 9714faca3b1e9bf1077cf753c9595049bf285856/tree b5c47d0077d32753fec5f7872fc65d58fa171b6e.
- Nav-stack mls/lib no-run passed. An execution-selection mistake then ran the entire surface_dataflow::tests prefix rather than only pure tests: 16 passed/11 failed/1 ignored. Ten unexpected existing real-LCM cases failed immediately at network initialization; they were not retried, elevated, or replaced with a different transport. The remaining pure rollback fixture had an outdated virtual-mask header and failed codec validation before its intended ordering assertion. Reviewed test-only 6224ff8bc2ca4e02e7f71c578a2d53896c5d721b rebuilds the same clear mask with the deliberately rolled-back header, preserving cells/layout and the rejection assertion.
- Native no-run initially found one remaining legacy PathEnvelope field in an existing real-LCM test fixture. Test-only 19b57ea3567fbb8f755cd768d7574f48cf465330 supplies explicit fixture receipt fields; that real-LCM test was not executed. Native no-run then passed in 1m34s on merge 0e382356c8170d868374c73c30c0ac05cc60bfa8/tree fe56b2e14bfebb256198906a0b3de9ddde2dd8e5.
- On that same merge, rebuilt nav binary passed all 17 source-audited exact pure tests. Their helpers use in-process caches/hubs/planners or test-owned repositories, not the real-LCM TestBus. Exact test inventory and allowlist are retained separately; no broad-prefix execution followed the mistake.
- Four receipt-admission and ten new composite producer component tests initially totaled 12 passed/2 fixture failures. One race test had not primed the mandatory initial clear/live sequence, so its expected-exposure gate correctly rejected the initial clear before reaching the blocked send. The direct activation publication test returned promptly and correctly emitted clear+live, while the fixture expected only one frame. Reviewed test-only 54daf5aedf555f1015351266da1550831f3bb46b primes the intended already-live race and strengthens activation assertions to exactly two ordered clear/live frames plus the exact receipt. Production guards/timeouts were not loosened.
- Final native merge 26d11e85f0059b14ce9d7df11fd82d97df4d729b/tree a720adb6a5b8fd49b33ff60bd82dd7153d05da0b compiled in 1m26s. All 14 audited exact component tests passed. The injected unit transport is explicit and does not construct real LCM or actuator resources. These results do not qualify full server activation or real multicast integration.
- The combined Python 67 tests also passed on final integrated tree a720adb6a5b8fd49b33ff60bd82dd7153d05da0b using the same namespace-only harness and pinned grpc/protobuf path. This harness intentionally skips unrelated eager service startup.

Commands used the existing baseline-build-env.sh target/jobs2, without SDK rebuild/clean/release/fullworkspace work. Commands were cargo test --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-codec --lib; cargo test --manifest-path services/nuwax-workspace/Cargo.toml -p nuwax-nav-stack --features mls --lib --no-run; and cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime --no-run. Final selected tests ran retained binaries with each audited full name and --exact --test-threads=1 --nocapture, after checking the binary's --list inventory.

Evidence under /workspace/shared/neh-validation: c1-stage-c-codec[-r2/-r3]-20261004.log; c1-stage-c-nav-compile[-r2]-20261004.log; c1-stage-c-nav-consumer-20261004.log (unexpected broad execution); c1-stage-c-nav-pure-r2-20261004.log; c1-stage-c-native-compile[-r2/-r3]-20261004.log; c1-stage-c-native-pure[-tail/-tail2/-r2]-20261004.log; c1-stage-c-python-integrated-20261004.log; c1-stage-c-{nav,native}-pure-allowlist.txt; c1-stage-c-pure-allowlist-audit.txt. Retained successful binaries: c1-nav-tests-0e382356 and c1-runtime-tests-26d11e85. Earlier failed binaries/logs remain intact.

At 17:18:28 UTC, original primary recovery/exploration-overlay-20261004 was restored to exact ec8affe9942e45742e09863fb93b3472191eef4c/tree acc79894f6102e0578aaa8f0c21157b129f990b9 and verified clean. The native lane was explicitly returned. Stage C is component-qualified on the recorded merged trees, not deployed and not a claim that the remaining application-identity/default-chain gates are closed.

## D1 explicit assigned outer-asset and durable-record foundation

This Python-only milestone adds an explicit assigned lane without switching default finalization, activation, export or relocalization. MapAsset/MapAssetBundle now distinguish legacy identity from typed AssetIdentityV2; assigned bundles require explicit candidate/finalized state and a caller-owned ID. Neither reads nor model defaults issue IDs. Legacy v1 serializers omit the new fields, and literal pre-D1 bundle/manifest bytes remain unchanged across reopen and repeated finalize.

The explicit finalize_assigned_map_asset_bundle API writes manifest schema2 containing the assigned asset identity, complete path/size inventory and exact checkpoint-v2 member descriptors. It preserves staging/fsync/no-replace publication, compares same-ID retries by complete inventory and streaming exact bodies, and rejects occupied legacy or different-ID namespaces before legacy hashing/body reads. Assigned conflict errors retain typed identities and never place IDs into legacy SHA fields. Managed size/prefix/read operations are separated from legacy hashing. Assigned snapshot capture uses stable descriptor observations, not a content digest. Candidate ownership is frozen once; packed occupancy and immutable cost cells are shared, while callers receive isolated small descriptor views. Equality-only model_copy updates cannot install new mutable aliases.

Staging cleanup pins the expected directory, moves its public name into a fresh private no-replace quarantine, verifies the moved inode before deletion, and traverses only through pinned descriptors. Deterministic pre- and post-quarantine replacement tests preserve unrelated directories/symlinks. Post-publication faults leave current discoverable for restart and metadata reconciliation rather than issuing a new identity or removing the durable owner.

Catalog assigned activation journals/receipts use a distinct durable schema3, because existing schema2 is digest-era. The explicit assigned begin/update/rotate/clear/complete APIs reuse the existing process/flock guard and durable paths. They bind asset ID, checkpoint ID and request ID to the validated nested native receipt, persist/fsync the complete receipt before removing the journal, and reject conflicting same-request replay. contracts/map_activation.py owns only immutable receipt metadata, not payloads; source member roles use canonical_cloud, verified through actual protobuf role3 and the established provider/store translation. The existing legacy journal APIs and records retain their original shape.

Source qualification: 101 explicitly selected Python tests passed in 0.606s: 19 assigned-storage, 15 assigned-catalog/receipt, and all 67 prior C1 tests. Tests include reopen/restart, exact retry, changed body/inventory, concurrent publishers, pre/post rename failures, receipt-write and journal-clear fsync failures, metadata recovery, mutable-alias rejection, cleanup replacement barriers, strict skew/missing identity, actual protobuf member translation and literal legacy compatibility. Hash and whole-payload-copy helpers are patched to fail on assigned-storage paths. Log: /workspace/shared/neh-validation/c1-d1-python-combined-r2-20261004.log. Source review cleared the corrected ownership, cleanup, role-shape and occupied-legacy conflict paths. No native/cargo/codegen/network execution or primary checkout edits were needed; namespace-only services imports still avoid unrelated unavailable cv2 startup.

Limits: D1 supplies explicit library/storage/durable-record operations, not a working default chain. Assigned planar replacement/CAS is rejected until D2 has an actual graph producer/target contract. Default native/Python activation, relocalization request/result/terminal/source proof closure, downstream asset consumers and final paired default switch remain D2/D3. Legacy writers retain their historical behavior, including remaining legacy hash calculations; no full-project hash-free or deployability claim is made here.

## D2a shared sealed-target ownership and exact request descriptors (source checkpoint)

Legacy sealed-target snapshots now retain the existing Arc allocation instead of copying the complete XYZ Vec. The new separately typed assigned snapshot retains InstalledActivationIdentityV2, InstalledTargetReceiptV2, publication proof and CanonicalCheckpointOwnerV2. Both capture only small descriptors/shared owners under the ledger lock and recheck lifecycle/current request/proof/owner pointers after capture. Deterministic tests exercise descriptor/pointer replacement, mapping retirement, Phase A and replacement; acquired snapshots retain only their existing owner until the last reader drops. Assigned relocalization availability remains closed.

The assigned cloud relocalization request descriptor uses the existing request ID, the complete typed installed target receipt and normalized timeout. It rejects all legacy identity fields and missing/unsupported target variants. Policy comparison now has an explicit foundation containing the full small existing canonical parameter records, not their SHA outputs. Existing source/deadline/algorithm versions, parameter encodings and timeout limits remain unchanged. Admitted requests share one canonical policy allocation; repeated request clones share an immutable metadata owner. Source/cloud payloads are not held in this descriptor.

Protobuf adds an explicit assigned-cloud target oneof and policy descriptor. Bindings were regenerated with the pinned tool and --check passed. The server rejects assigned/mixed/unsupported Relocalize contracts before ingress while actor/source/candidate/durable-terminal pairing is incomplete; the old valid schema1 execution path is unchanged. Six pure descriptor tests cover exact protobuf roundtrip, timeout normalization, policy conflicts, missing/mixed/skew and source/installed target binding. Four target-snapshot test cases cover new races and expanded existing owner/replay tests. Native compilation/execution is pending at this source checkpoint.

The 101 explicitly selected Python regressions still pass after codegen (c1-d2a-python-regression-20261004.log), and Rust source parsing/diff checks pass. Bounded independent source review found no blocker. D2a is an ownership/protocol foundation, not an availability/default switch. Shared source/correction proofs, exact actor/terminal restart replay, the genuine planar target/archive branch, client/durable consumers and the final D3 default pairing remain open; no complete hash-free/live-relocation claim is made.

### D2a integrated qualification

Clean primary 5450edfe58185f7ad9227b6ba230434834d7db58/tree afb234797bb65afe54952f6fb28ed2132bdff6a1 granted the exclusive original-path window. Frozen 1134bf2420f9cb518a927cadf53460c679ebab2f merged without conflicts as 18df87ff6e8141033673f6f4d2ee20be1366bf6b/tree 6a272e856b3c50411c84c41107631c11073131de on validation/c1-d2a-ownership-20261004.

Using baseline-build-env.sh/warmed target/jobs2, cargo test --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml --bin nuwax-geometry-runtime --no-run passed first attempt in 1m49s. All ten source-audited exact pure tests passed: six request/policy/target descriptor tests and four coordinator owner/replay/race cases. Each full name was checked against the retained binary's inventory before execution with --exact --test-threads=1 --nocapture. No broad filter, real-LCM test, source fix, SDK rebuild, or mathematical/controller change occurred. The 101 selected Python tests also passed on the integrated tree.

Evidence: /workspace/shared/neh-validation/c1-d2a-native-compile-20261004.log, c1-d2a-native-pure-20261004.log, c1-d2a-native-pure-allowlist.txt, c1-d2a-native-pure-audit.txt, c1-d2a-native-inventory.txt and c1-d2a-python-integrated-20261004.log. Retained binary: c1-runtime-tests-18df87ff.

At 18:18:31 UTC, primary recovery/exploration-overlay-20261004 was restored to exact 5450edfe58185f7ad9227b6ba230434834d7db58/tree afb234797bb65afe54952f6fb28ed2132bdff6a1 and verified clean; native lane was explicitly returned. Qualification is limited to the ownership/protocol foundation. Assigned live relocalization and the final default chain remain gated pending D2/D3.
