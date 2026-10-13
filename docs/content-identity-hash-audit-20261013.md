# Content identity hash audit — 2026-10-13

## Scope and evidence

The historical call graph below is a read-only source audit of Geometry
checkpoint, activation, and forbidden-zone content hashing at baseline commit
`9eb9617`. All numbered source references refer to that baseline, not to the
post-cleanup source. The initial audit was documented after HEAD advanced to
`935d732`; no changes to its cited server/runtime/workspace files were observed
between those commits. No hashing functions, tests, or benchmarks were executed
for that initial source audit.

The forbidden-zone cleanup is now implemented and verified on the final source,
as recorded in its implementation section below. The remaining legacy export
and activation routes were not changed. This is not a repository-wide claim
that all application hashing has been removed.

Assigned identity v2 and the older enum variant named `Modern` are different:
`Modern` means activation contract v1 and still computes content hashes.
Legacy hashing remains functional compatibility code. This audit does not
authorize disabling those routes or removing their validation.

## Current assigned save and activation paths

The public native map-save route in `skills/map_workspace.py:1672–1702` calls
`finalize_assigned_cloud_map`. In `skills/assigned_map_save.py:144–145`, that helper
explicitly calls `prepare_map_asset_bundle(..., checkpoint_schema_version=2)`.
`services/geometry/runtime.py:5455–5458` passes this selection to the exporter;
`services/geometry/geometry_runtime_provider.py:5746` sends the protobuf field.

The Rust export entry point in
`services/geometry/nuwax_geometry_runtime/src/server.rs:6247` selects the envelope.
The v2 branch of `assemble_mapping_checkpoint_export` at `4297–4310` assigns typed
checkpoint members and structurally validates them without computing the legacy
backend or canonical-cloud digests. The dense archive export branch at `6591`
likewise inserts an assigned artifact without the legacy archive hashing branch.

Activation classification at `server.rs:1702–1744` selects `Assigned` for contract
version 2. `activate_map_r1:2294–2335` admits typed immutable checkpoint owners,
instead of entering the legacy canonical-hash pipeline. Assigned journal keys,
installed activation identities, effect verifiers, and target receipts use typed
identity and receipt fields. The assigned planar activation branches are in
`server_planar.rs:27–42`, `59–61`, and `220–227`.

The audited assigned save/checkpoint and activation identity paths do not call
the remaining legacy content-hash functions. This does **not** imply that every
operation during an assigned session is hash-free: forbidden-zone parsing below
was independently reachable at the audit baseline.

## Remaining executable legacy hashes

All Rust paths below are relative to
`services/geometry/nuwax_geometry_runtime/src/`.

### Legacy checkpoint export

- `server.rs:4221–4225`: request values 0 and 1 select legacy envelope 1
- `server.rs:4315`: hash backend payload during legacy assembly
- `server.rs:4318`: hash canonical cloud bytes during legacy assembly
- `server.rs:4319` → `validate_cloud_checkpoint_metadata:4012`: verify cloud digest
  with another computation
- `server.rs:4320` → `validate_mapping_backend_checkpoint:4357`: verify backend
  payload digest with another computation
- `server.rs:6596` → `dense_cloud::hash_reader:537`: hash legacy dense archive
- `planar_artifacts.rs:67`: legacy insertion verifies the archive with
  `hash_reader` again; legacy upload completion also verifies at `198`

These are actual computations, not merely retained field names or aliases.
Their cost scales with payload/archive size and includes repeated passes.
No timings were measured.

### Legacy activation, including contract v1 named `Modern`

The non-assigned branch of `activate_map_r1` remains reachable for accepted
contract versions 0 and 1:

- `server.rs:2347` → `canonical_occupancy_sha256:1811` → occupancy bytes at `1807`
  and canonical envelope at `1813`
- `server.rs:2355` → cloud digest verification at `4012`
- `server.rs:2356` → `immutable_target_from_checkpoint:1879` →
  `pointcloud::decode_immutable_activated_target:413`, another cloud-byte hash
- `server.rs:2366` → canonical cloud-envelope hash at `1838`
- `server.rs:2371` → backend payload validation at `4357`
- `server.rs:2386` → `canonical_backend_sha256:1901`: embedded cloud decoding,
  backend payload hashing at `1948`, occupancy/cloud envelope hashing, then
  enclosing backend hash at `1964`; planar variant hashes at `1915`
- `server.rs:2426`, `2469`, `3055/3073`, and `3271`: bundle, request, installed
  activation, and target-receipt identity hashing
- `server.rs:2098/2111`, `2174`, `2784`, and `3127`: legacy-only journal/effect
  verifier hashing; assigned alternatives use typed fields

For planar v1 activation, `server_planar.rs:62–74` computes occupancy/backend,
bundle, and request hashes, while `229` computes the installed activation hash.
The v2 assigned branches avoid those calls.

`correction_coordinator.rs:3594–3610` hashes application policy preimages.
`apply_policy_identity:5180` is called from the `LegacyDigest` branch at `5208`.
The planar policy overrides at `5299–5300` are also in a legacy branch. The
assigned branch at `5213` uses typed policy descriptors instead. Canonical legacy
activation identity construction at `server.rs:2044–2055` also calls these policy
hash helpers.

## Historical forbidden-zone hash path, now removed

`mapping_adapter.rs` invokes the cache during planar preparation (`622`),
candidate preparation (`675`), refresh loading (`1389`), and general loading and
refresh (`2930–2935`, `3044`). Those paths reach
`forbidden_zones::ForbiddenZoneCache::load` and `parse_for_map`.

At the baseline, `forbidden_zones.rs:251` hashed the entire file on every parse,
even when the selected zone set already contained an explicit assigned signature.
For a blank signature, `293` also hashed a serialized zone entry. Cache hits avoided
parsing, but initial reads and changed files still hashed in an assigned session.

The writer already assigns opaque random signatures with `secrets.token_hex(32)`
in `services/navigation/forbidden_zones.py:43–45`. The Rust `signature` is used
as a zone-set update identity and runtime acknowledgment, not a cryptographic
authentication check. `mapping_adapter.rs:3055` skips refresh when it equals the
current signature; Python navigation acknowledgment matching also compares it
(`services/navigation/service.py:103`, `240–251`, `573`).

## Implemented forbidden-zone cleanup and verification

Both content-hash calls in `forbidden_zones.rs::parse_for_map` and its unused
zone-entry serialization have been removed. Signature-less legacy snapshots
and parsed-error outcomes now receive a UUIDv4 observation ID on a parse cache
miss. The existing cache owns and reuses the loaded result and its ID. Valid
stored signatures retain exactly the existing trim/lowercase normalization and
remain authoritative acknowledgment aliases.

This adds no mapping commit clock, mutex, other persistent allocator state,
custom cryptography, or digest-shaped stand-in. UUIDs are observation identities,
not content checksums. An unchanged cache hit allocates no new ID. A changed or
replaced unsigned/parsed-error source receives a new ID even when its generation,
length, mtime, or contents are unchanged. A separately uncached unsigned/error
parse is a new observation; explicit stored aliases remain unchanged.

The existing cache key includes path, map binding, and `FileStamp` device, inode,
length, mtime/nanoseconds, and ctime/nanoseconds. Its before/after metadata check
still excludes unstable reads from the cache; the next read observes the source
again. Stable malformed JSON, unsupported schemas, and invalid zone sets reuse
their cached fail-closed identity. Existing missing/absent/open/read error
sentinels, geometry validation, and fail-closed rasterization are unchanged.
The existing `same_source_version` fence still rejects stale cloned refresh
work. No production mapping-adapter refresh-gate change was needed: a new legacy
observation ID reaches the existing signature-change gate, while cache hits do
not continually recompose or advance the map revision.

Final-source native release verification completed on 2026-10-13 at 18:52 UTC:
**21 tests passed, 0 failed, 0 ignored**. This was the focused forbidden-zone
selection, not the whole runtime suite. Command:

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
cargo test --offline --release \
  --manifest-path services/geometry/nuwax_geometry_runtime/Cargo.toml \
  --bin nuwax-geometry-runtime forbidden_zone -- --test-threads=1
```

The six new regression tests use the existing `map_checksum::FORBID_HASH`
sentinel rather than computing expected content hashes. They cover explicit
alias normalization, blank signatures, cache-hit/clone stability, source
replacement preserving mtime and generation, same-generation in-place edits,
malformed/unsupported/invalid parsed outcomes and recovery, deletion, deterministic
I/O sentinels, torn-read cache exclusion, stale refresh rejection, no-op revision
stability, and changed obstacle rasterization. Existing acknowledgment, cache,
refresh, and conservative geometry tests also passed. `git diff --check` passed.

The final-source test log is retained outside the repository at
`/workspace/scratch/71cdb9d17732/neh-evidence/forbidden-zone-observations-final-test.log`.
The earlier first-pass log is not the final-source verification record. No
initial-support behavior or unrelated legacy hash path was changed.

## Direct/default client inventory and compatibility boundary

- `GeometryRuntime.prepare_map_asset_bundle`, `runtime.py:5396`, defaults to 1
- `GeometryRuntimeProvider.export_mapping_backend_checkpoint`, provider
  `5731–5733`, deliberately defaults to 1
- Raw gRPC export requests with an omitted schema field decode as 0 and select
  legacy 1 in the Rust server
- `_finalize_legacy_active_map` in `skills/map_workspace.py:1706` contains a call
  at `1886` that omits the schema and therefore selects legacy export. Static
  search found its definition but no current non-test call site; do not treat
  this as evidence that an active public save uses it
- The public assigned save explicitly selects 2, as shown above
- Legacy backend activation remains reachable through dynamic dispatch in
  `services/geometry/runtime.py:8713` to
  `GeometryRuntimeProvider.activate_platform_map:6008`. This API defaults the
  expected checkpoint schema to 1 (`6022`) and builds activation contract 1
  (`6350`). Its schema-2 rejection is a guard against using the unpaired legacy
  consumer; assigned activation has a separate request builder (`995`)
- The Rust server accepts explicit legacy activation and artifact transfer
  requests; static source inspection cannot establish which external clients
  still use those interfaces

Legacy content hashing therefore **still remains**. A subsequent migration needs
a compatibility decision and paired producer/consumer changes, not silent
removal of functioning routes during performance work. Keep current legacy
behavior until that decision and implementation are complete.

Retaining stored legacy aliases and historical read models does not itself
require computing new hashes. However, accepting a legacy payload while dropping
its checksum verification must not be presented as equivalent verified behavior.
Any future ID-based import needs a real immutable owner, structural validation,
and session/frame/member/receipt binding. Never fabricate hashes or treat a
legacy digest as proof that a newly assigned owner has been validated.

## Explicit exclusions

Fixed wire/schema protocol fingerprints and ordinary `HashMap`/`HashSet` indexing
are not content-identity hashes to remove. `canonical_hash32` only validates and
converts a 32-byte stored value; it does not hash. Reading or comparing existing
digest aliases is distinct from executing a hash function. Other application
modules and languages require their own audit before any project-wide claim.
