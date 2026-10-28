# Incremental display-cloud shared memory v3

The producer, SHM reader and Web stream use producer-assigned page revisions. No coordinate hashing, full point-set differencing, global sorting or full XYZ materialization occurs for ordinary changes. This is a display-cloud transport; native fine-navigation map and explicit checkpoint formats have independent contracts.

## Storage and ownership

- One producer UUID identifies the writer independently of file inode, timestamps, reader reopen and map revision
- Exact u64 `revision` advances on accepted publication. `layout_generation` advances on reset, session change, coarsening, checkpoint installation or PGO/global rewrite
- Dense storage addresses contain 1024 points per page. Insert appends. Delete swap-removes the final point into the vacated address and marks both affected pages. A storage address is not a semantic point ID
- Producer dirty-page state is populated by actual insert/remove operations. A same-count delete+append still updates page revisions
- Immutable committed page byte arrays are shared across private scan work and public state. Unchanged pages keep their byte allocation. Accepted private work installs only its committed dirty pages
- Creating a public writer uses an initialized temporary file and atomic rename. An old reader mapping is never truncated

## Layout (little endian)

| Offset | Type | Meaning |
| --- | --- | --- |
| 0 | u64 | `NUWAXMAP` magic, numeric 0x4E555741584D4150 |
| 8 | u32 | Layout version 3 |
| 12 | u32 | Header size 512 |
| 16 | aligned atomic u64 | Publication sequence: odd while writing, even when committed |
| 24 | u64 | Map revision |
| 32 | u64 | Source timestamp ns |
| 40 | u64 | Layout generation |
| 48 | 16 bytes | Producer UUID bytes |
| 64 | u32 | Current point count |
| 68 | u32 | Point capacity |
| 72 | u32 | Points per page, 1024 |
| 76 | u32 | Capacity page count |
| 80 | f32 | Voxel size |
| 84 | u32 | Reserved |
| 88 | u64 | Mapping session generation |
| 96 | u64 | Page revision table offset, 512 |
| 104 | u64 | XYZ payload offset, next 4096-aligned address after table |
| 112 | u64 | File size |
| 120 | u64 | Reserved |
| 128 | 16 bytes | Commit boot nonce |
| 144 | u64 | Commit sequence |
| 152 | u64 | Correlated occupancy revision |
| 160 | u32 | Mapping session ID UTF-8 length, at most 256 bytes |
| 164 | 256 bytes | Complete mapping session ID, never truncated |
| 420 | 92 bytes | Reserved |

The table stores one aligned little-endian u64 last-changed revision per capacity page. The payload stores packed float32 XYZ. Final partial-page length follows `point_count`; bytes after the current count are not live points. Session ID size is checked at the owned receipt boundary before publication mutation, including multibyte UTF-8 boundaries.

## Publication consistency

1. Producer stores odd publication sequence with SeqCst ordering, then a SeqCst fence
2. Producer overwrites dirty page bytes, their table revisions and frozen commit metadata
3. Producer stores the next even sequence with Release ordering
4. Reader performs an aligned native Acquire load of sequence, then reads the header, page revision table and only changed page bytes into private candidate objects
5. Reader performs a native SeqCst fence and final Acquire sequence load; only equal even sequences commit the candidate snapshot

Python uses the platform `libatomic.so.1` ABI through `ctypes.PyDLL`; tiny native calls retain the GIL. A temporary read-only numpy buffer view pins the mmap only during each native load, and is released before returning or closing. The platform must provide lock-free aligned u64 atomics. Missing capability is reported once for that source lifecycle, not retried per frame. No locks, producer waits, spin waits or payload scans are added. The reader makes at most three immediate attempts; busy writers keep the previously accepted live snapshot and never trigger a saved-map fallback.

These ordering rules have source review, x86_64 native compilation and actual Rust-producer/Python-reader interoperability coverage. The six-phase native fixture verifies append, edit, deletion, reset, exact page bytes and unchanged-page ownership. Actual aarch64 execution remains separate required verification; host tests do not establish hardware-specific behavior.

## Catch-up, bootstrap and Web handoff

A Python snapshot contains immutable current page references. It copies only pages with changed producer revision or changed final-page length. An unchanged poll returns the same snapshot. A socket with the same producer, layout and stream generation selects current pages whose revisions are newer than its applied base. Since every selected page is a complete replacement, arbitrarily old clients need no replay journal. Final point count truncates removed trailing pages.

A new client or epoch/layout mismatch receives a reset/bootstrap as page chunks. A real global rewrite naturally changes all pages and therefore transfers the changed whole map once. Ordinary one-point changes never take this reset path. Stream/Web revisions are exact decimal strings, and a client advances its base only after applying a complete batch. Frozen `mapping_source_epoch` equals `session_id:session_generation`, matching the map snapshot source identity; current runtime metadata must never be grafted onto older cloud bytes.

## Compatibility boundaries

- Python reads legacy v1/v2 SHM with their existing semantics and unchanged-read cache
- `read_stable()` explicitly materializes contiguous bytes for a one-off export. A specified `max_points` explicitly requests legacy sampled output and gathers only those points; normal live streaming has no sampling limit and retains pages
- Native full PointCloud2 is generated only for active person-follow or explicit legacy demand; fine-navigation publication uses its existing chunks without cloning and clearing the display cloud
- Explicit old checkpoint/activation serializers still require full payloads and existing checksum fields. Their hash removal is outside this transport change; this implementation does not claim those contracts are hash-free
