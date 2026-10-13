# Assigned import of legacy-envelope fine archives — 2026-10-13

## Implemented scope

Explicit saved-cloud activation can now import a schema-1 checkpoint containing
an already current-format `NUWAXAR2` or `NUWAXAR3` fine archive through the existing
assigned-identity importer. The existing schema-1 checkpoint store accepts those
archive formats; no deployed inventory was inspected to establish their frequency.

Production changes are limited to:

- `services/mapping/asset_store.py::import_legacy_cloud_bundle`: bounded format
  eligibility and allocation of an outer fine-artifact ID for a new import owner
- `services/geometry/mapping_backend_checkpoint_store.py::assign_legacy_cloud_checkpoint`:
  preserve exact archive bytes; bind the assigned fine member to the actual
  checkpoint, original occupancy/cloud receipts, canonical frame and archive's
  original frame/producer generations
- `services/geometry/fine_map_archive.py::read_fine_archive_root`: shared bounded
  root-JSON parsing extracted from the existing validator; this helper alone is
  explicitly not complete archive admission

The final existing `validate_checkpoint_bundle` remains authoritative for full
archive structure and member/source binding. Assigned publication, source-owner
fences, catalog adoption, retry reuse, transfer and activation request contracts
are reused. No Rust production code or protobuf schema is changed.

## Preserved boundaries and guarantee

AR1 remains on its previous fallback because it needs a root-format migration;
it is not relabeled as AR2. Missing original receipts, ambiguous matching imports,
pending old transactions, and existing startup/ensure fallback behavior are
unchanged. No generations, revisions or commit receipts are invented. Fine data
is never discarded to make an import succeed. An accompanying dense archive is
preserved, including its absence when there was none.

Historical checksum strings are read as aliases. Newly assigned IDs identify the
new immutable import owner; they do not authenticate historical content or prove
that bytes match a former checksum. Structural/source checks and exact-byte replay
checks remain in their existing consumers.

## Python verification

Final focused verification on 2026-10-13 at 21:51 UTC: **87 tests passed, 0 failed**.
The selected modules are `test_legacy_cloud_import` (36),
`test_checkpoint_identity_v2` (43), and `test_fine_archive_vector` (8).

The outer runner disables available `hashlib` constructors and `payload_digest`
before test-module loading and fixture construction. The importer/identity suites
also install their own per-test guards. Fixtures use literal historical aliases
and write real on-disk schema-1 layouts without invoking the hashing legacy
publisher. The new positive test first failed against the previous blanket
fine-archive exclusion, then passed after implementation.

Coverage includes AR2/voxel and AR3/ray inputs, normal v2 request construction,
exact archive bytes and source generations, optional dense data, retry identity,
catalog-publication interruption, source preservation, malformed/mismatched
archives, duplicate object identities with differing bytes, missing/invalid/reused
artifact IDs, retained bytes ownership and same-length source replacement.
The parser extraction is checked against the existing frozen scalar oracle for
AR1/AR2, with AR3 structure/layer-JSON checks and bounded root-only reads.

From the repository root, use the project's Python dependencies and checked-in
guarded runner (the local verification environment sources its toolchain first):

```sh
source /workspace/scratch/71cdb9d17732/neh-toolchain/env.sh
python scripts/diagnostics/verify_legacy_fine_import_nohash.py
```

The final log and exit code are retained outside the repository at
`/workspace/scratch/71cdb9d17732/neh-evidence/fine-import-python-final.log`
and `fine-import-python-final.exit` (0). The red-stage evidence is
`fine-import-red.log`; parse-count evidence is `fine-import-parse-counts.log`.

## Remaining structural work and qualification

Instrumenting the real AR3 import with the no-hash guards active found:

- Fresh import: 5 bounded root-JSON parses and **4 full archive walks**
- Conversion alone: 2 root parses and 1 full archive walk; the fine-archive bytes
  object is the exact source object, not an archive-sized copy
- Published retry: 1 root parse and 1 full archive walk in existing reopen logic

The first implementation performed 5 full walks for a fresh import. Extracting
shared bounded root parsing removed the extra conversion-time walk. The remaining
four include existing read/admission/publication/reopen boundaries; this change
makes no claim that those costs are eliminated. No latency benchmark was run.

These are focused Python results. Native sealed-store/admission coverage is being
prepared separately and is not counted as passed here. This is not end-to-end
activation verification or complete removal of application hashing. Legacy
export, fallback activation, request/bundle/journal/policy hashes and other
application paths remain outside this bounded change. LCM protocol fingerprints
are unaffected.
