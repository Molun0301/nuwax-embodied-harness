# AR1 source under assigned ownership — paired implementation and qualification

## Status

The paired production extension is implemented and qualified. Python capability,
import/reconnect and existing regressions pass 143 tests. The release native
qualification passes 4 synchronous tests, including the actual Python-emitted
AR1 assigned request through sealing, admission, cloud-writer installation,
next real cloud commit and retained-owner replay. A further 8 assigned-store
regressions pass. This remains a bounded synchronous integration slice, not an
end-to-end activation RPC or deployed-map-frequency claim.

## Existing behavior and feasible ownership model

An AR1 archive already has source session/generation, frame generation, producer
generation, root revision and source timestamp. Its block aliases and geometry
aliases do not contain current-format block/geometry revision history.

The truthful representation is an assigned outer artifact/checkpoint owner
retaining **unchanged AR1 bytes** and their original source receipts. Do not
rewrite AR1 to AR2 at the old revision; do not increment a source revision and
manufacture a matching cloud receipt. The archive magic already discriminates
its internal format independently of the assigned outer identity.

Existing native support:

- `ArchiveReader::open` decodes `ArchivedRoot::Legacy`; `read_block` checks actual
  duplicate-alias bytes and per-point evidence; `import` installs exact objects
- `Repository::put_object` uses no-replace installation and exact-byte equality;
  `install_legacy_root` checks original JSON against parsed metadata
- `Writer::restore_legacy` retains aliases and evidence, leaves the block and
  geometry revisions at zero internally, and records the legacy publication
  baseline; that unpublished state is not emitted as a current manifest
- `FineCloud::from_archive` retains the source root identity and observation bound
- `CloudMapShmWriter::commit_accumulated_snapshot` reserves the real publication
  clock, respecting the same-domain baseline floor; `Writer::seal` then stamps
  all baseline block/geometry revisions with that actual publication revision
- The resulting `CloudCommitReceipt` names the same real revision and installed
  occupancy commit. A new domain may begin a new clock; the same domain must
  strictly advance beyond its legacy root

Downstream current roots require positive geometry revisions, ordered as
`block.geometry <= block.revision <= root.revision` and
`block.geometry <= root.geometry`. Subsequent same-domain changes cannot roll
these clocks back. Evidence-only changes advance block revision; membership or
layer-geometry changes also advance geometry revisions. A conservative initial
baseline states when the new owner first published that geometry, not when each
historical point was observed. Original zero/known point evidence remains intact.

The traced AR1 archive/repository/writer path contains no content-hash call.
`nuwax-map-chunks` depends on serde, serde_json, getrandom, libc and tempfile,
not a digest library. The `sha256`/`geometry_sha256` spellings are legacy aliases.
This source inspection is not a test result or a repository-wide hashing claim.

## Deliberate contract restrictions

The obstacles are paired format restrictions, rather than an absent AR1 decoder:

1. Cloud importer eligibility and `assign_legacy_cloud_checkpoint`
2. Python `_validate_checkpoint_artifacts` in the checkpoint store
3. Python provider `_download_graph_artifact` assigned fine-format check
4. Rust `planar_artifacts::Store::validate_archive_binding` previously rejected AR1
   under assigned outer identity before admission

The existing typed owner, sealed-file pinning, original member/receipt checks,
AR1 preparation and real installation path can otherwise represent this source.
Changing these guards is a contract extension requiring paired compatibility
handling and tests; it must not be done by silently relaxing checks.

## Implemented compatibility extension

Add a dedicated, additive field to the current five-field `RuntimeStatus`:

```protobuf
repeated uint32 supported_assigned_fine_archive_versions = 6;
```

New supporting servers advertise `[1, 2, 3]`. An omitted/empty field means the
existing AR2/AR3-only contract; it never grants AR1 permission. Unknown versions
are not interpreted as permission. Do not overload diagnostic text or lcm_backend.
The AR1 magic is already the payload discriminator; no fake digest or new source
receipt is needed in `GraphArtifactRef`.

Approved production changes:

- Proto plus checked-in generated Python protobuf output: additive status field
- `server.rs::get_status`: advertise only formats the server actually admits
- Provider `connect`: capture the capability from its **existing** GetStatus
  response under the committed stub/connection epoch; invalidate it on reconnect
  or failed attach. Expose a read against `capture_assigned_activation_origin`
  and its existing origin guard. No new per-frame or per-RPC status polling
- Runtime/workspace import preparation: obtain the capability with the same
  captured provider origin before opting into AR1 import. The existing
  no-capability paths retain their present behavior; do not publish/adopt an AR1
  assigned catalog identity assuming an older server can consume it
- Store/importer/provider/Rust format guards: opt into unchanged AR1 source bytes
  while preserving all existing size, exact-reference, root-domain, receipt,
  frame, point-evidence, duplicate-alias and immutable-replay checks
- Before dispatch/upload, recheck the provider origin. Connection changes must
  not reuse an old supporting server's capability for a replacement server

An older server receives the unchanged Empty status request and returns no
capability; it already rejects assigned AR1 at seal. An older client ignores the
new status response field and continues using existing formats. A persisted AR1
assigned import opened with a non-supporting server must fail explicitly before
upload/dispatch, retaining recovery/source state; no silent relabel or implicit
content-hashing route is authorized by the new capability.

If protocol policy requires every schema-2 peer to accept exactly AR2/AR3 rather
than capability-extensible payload formats, use a separately versioned import
contract instead. This design does not choose that policy implicitly.

## Required qualification

Passed lower-level test: original AR1 aliases and both known/unknown evidence;
same-domain and new-domain real install; actual reserved cloud receipt/root
parity; first real observation update; membership update; stale occupancy receipt
and forged rollback rejection; old retained root remains readable. Both existing
no-hash sentinels must be active before fixture construction on the same thread.
This uses synthetic occupancy input receipts with the real commit-ID allocator
and the real cloud writer; it is not a full mapping-adapter or activation RPC test.
The synchronous test passed with both native no-hash sentinels active before
construction, alongside the two separate AR2/AR3 typed-admission tests. See
`fine-archive-legacy-import-20261013.md` for exact command and evidence.

Paired qualification requirements:

- Actual Python on-disk AR1 import, unchanged archive bytes and typed source member
- Native assigned transfer sealing/admission/install/replay from that request
- Absent/empty/unknown capability and older-server rejection before adoption
- Supporting connection replaced by old connection during import or upload
- Existing AR2/AR3, dense, planar and current-save formats remain unchanged
- Malformed/truncated AR1, alias/body conflicts, stale source receipts and mixed
  frame/producer generations fail closed without hashes or partial publication

## Current implementation scope and race boundary

The eleven production files are:

1. `services/geometry/nuwax_geometry_runtime/proto/geometry_runtime.proto`
2. `services/geometry/proto_gen/geometry_runtime_pb2.py`
3. `services/geometry/nuwax_geometry_runtime/src/server.rs`
4. `services/geometry/nuwax_geometry_runtime/src/planar_artifacts.rs`
5. `services/geometry/mapping_backend_checkpoint_store.py`
6. `services/mapping/asset_store.py`
7. `services/geometry/geometry_runtime_provider.py`
8. `services/geometry/runtime.py`
9. `skills/map_workspace.py`
10. `skills/assigned_map_save.py`
11. `services/mapping/catalog.py`

The last two provide an optional, single-use adoption context at the existing
catalog `os.replace` linearization point. Catalog serialization, temp-file write
and fsync, directory fsync and state/event callbacks are outside runtime/provider
locks. The existing catalog guard remains outer; runtime state and provider
origin are checked and held only during the rename. A reconnect before rename
rejects and retains the original catalog. A reconnect after rename is later
than a valid adoption, so durability may finish normally; a subsequent AR1 upload
still checks the replacement connection. No new locks or RPC polling are added.
AR2/AR3 callers take the original unguarded catalog-save path.

The capability check briefly releases the provider lock before entering the
origin guard; the re-entry validates the same stub/epoch, and capability is
immutable within that epoch. This avoids recursively entering its non-reentrant
lock while preserving the fence. Runtime provider/attach ownership is retained
through that short operation.

Generated Python protobuf output uses the already available protoc 29.3 and
requires protobuf >=5.29.3. The project's Python 3.8 dependency sets pin 5.29.6;
import and field-6 serialization were verified on that toolchain. Existing RPC
signatures and the generated gRPC stub are unchanged.

## Python evidence

Run `python scripts/diagnostics/verify_legacy_fine_import_nohash.py` with project
dependencies. Sentinels patch hashlib and all relevant payload-digest aliases
before loading test modules or constructing fixtures. The 143-test suite includes
13 focused cases in `tests/test_assigned_ar1_import.py`, plus original AR2/AR3,
checkpoint identity, vector parser, assigned loader and planar repository/catalog/
workspace regressions. Latest result: 143 passed in 3.962 seconds.

`ar1-adoption-race-red.log` records the original missing final-fence test failure;
`ar1-python-regressions.log` records the corrected suite, under the task evidence
directory. Interleavings cover reconnect after import, provider replacement,
reconnect after file fsync immediately before rename, and reconnect immediately
after the guarded rename. Assertions verify both hot locks are absent during
file/directory fsync and present during rename.

Actual AR1 native fixtures are emitted by the checked-in fixture generator using
the real on-disk source, real provider connect capability and workspace import/
catalog adoption APIs. The bridge/status and activation RPCs in Python are test
doubles. Native tests consume those exact emitted protobuf/archive bytes and use
real artifact transfer sealing, typed admission, pinned ownership, cloud-writer
installation and next cloud commit. All four synchronous native tests passed.

This change removes application content hashing from this supported saved-source
migration path. Legacy digest fields remain opaque aliases; no original checksum
integrity is asserted by assigned IDs. Exact bytes, structure, source ownership,
receipts and replay equivalence retain their respective checks. This is a
compatibility/hash-removal change with no measured latency gain or deployed AR1
frequency claim, and no claim that every remaining legacy path is hash-free.

## Native terminal evidence and reproduction

On 2026-10-13 the original incremental release profile completed in 8m32s:

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
export CARGO_BUILD_JOBS=2 CMAKE_BUILD_PARALLEL_LEVEL=1
cargo test --offline --release \
  --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \
  --bin nuwax-geometry-runtime legacy_fine_ -- --test-threads=1
```

Result: **4 passed, 0 failed, 0 ignored**, 1332 filtered out. Both native no-hash
sentinels are enabled before construction and restored by RAII on the same
thread. Coverage includes unchanged AR1/AR2/AR3 source bytes; original frame,
producer and receipt fields; pinned-file ownership after transfer release;
retained replay and changed-source rejection; malformed/truncated archives;
AR1 duplicate-alias/body conflicts; actual AR1 install and next real cloud commit.
The existing lower-baseline test additionally covers same/new owner domains,
known/unknown observation evidence, membership updates and rollback fences.

The same built test binary then ran `assigned_v2_store_ --test-threads=1`:
**8 passed, 0 failed, 0 ignored**, 1328 filtered out. These existing store tests
keep the archive no-hash sentinel active and cover dense, AR2/AR3 and AR1 source
binding, exact retry/release, format limits, corruption and immutable ownership.
No clean or profile/cache change was used.

Task evidence files:

- `ar1-native-release.log` and `ar1-native-release.exit` (exit 0)
- `ar1-native-store-regressions.log` and its `.exit` (exit 0)
- `ar1-python-regressions.log` (143 passes)
- `ar1-python-focused.log` (13 passes, including retained runtime preparation)
- `ar1-adoption-race-red.log` (expected pre-fix failure)

All are under `/workspace/scratch/71cdb9d17732/neh-evidence/`. No application,
file, binary or provenance digest was computed for this qualification.

## Fixture and staging notes

The actual assigned AR1 files are `ar1_voxel.request.pb` (908 bytes) and
`ar1_voxel.fine.archive` (511 bytes) in the existing native
`tests/fixtures/legacy_fine_import/` directory. The literal lower-level
`ar1_source.fine.archive` is unchanged from the earlier qualification. The new
request contains assigned fixture IDs and test-only source names, no credentials.
The generator runs the real guarded capability-connected workspace import;
`python .../legacy_fine_import/generate.py 1` regenerates only AR1 assigned
fixtures. New assigned IDs mean request bytes may legitimately change. Archive
bytes remain the original source bytes.

Some fixture/test paths match repository ignore rules. Explicitly include the
new Python test, updated generator, both new AR1 files and the two updated native
cfg(test) modules when staging. The complete scoped path list is also recorded
in the task evidence file `ar1-stage-files.txt`; it excludes unrelated navigation
and RSS work.
