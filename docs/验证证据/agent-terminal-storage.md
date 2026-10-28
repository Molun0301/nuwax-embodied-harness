# Agent call and terminal receipt storage

The call ledger records durable request IDs, canonical arguments, pending/recorded/unknown delivery states, scope owners, and explicitly verified legacy aliases. Terminal receipts have independent source/revision ownership. Neither database executes or resumes a domain action.

## Layouts

- The low-level AgentCallStore API keeps its default: an exclusively new ledger uses a separate sibling `<ledger stem>.terminals.sqlite3`. Its optional `fresh_layout="combined"` applies only to creation.
- Application startup uses the installed launcher's creation policy. A genuinely new managed installation selects split; an upgrade whose previous generation has unknown/missing storage-reader capability selects combined, so an old pinned updater cannot create a new split-layout rollback hazard. Unknown/unmanaged launchers also select combined unless a trusted operator explicitly sets `NUWAX_AGENT_TERMINAL_FRESH_LAYOUT=split`.
- A prior generation with the exact supported integer `agent_terminal_storage.reader_version=1` allows split for newly created ledgers. Boolean, malformed and future/unknown capabilities remain conservative. Each candidate manifest records its own reader capability and chosen fresh policy under the existing backward-readable manifest schema.
- The generated launcher always exports the computed policy, strips stale config values and removes only that key from inherited file-origin tracking. Ambient environment/config cannot silently promote a conservative managed upgrade to split. Broken current pointers/manifests keep their existing errors and are never treated as a fresh installation.
- A recognized existing ledger remains in `combined` layout after additive initialization. Historical rows are not moved automatically.
- `pending` means a specific target/pair is recorded but not activated. The existing combined receipt path remains authoritative. A fresh initializer resumes only its own recorded pair.
- `active` means the exact target is paired and the old terminal table is sealed. Requests and scope authority remain in the original database.

`AgentCallStore.terminal_storage_status(path)` is read-only and reports `ledger_id`, `origin`, `layout_state`, `terminal_path`, `pair_id`, and `legacy_sealed`. It does not initialize or repair a missing database. A pre-existing unrecognized empty file is an incomplete-initialization condition, not evidence of a fresh owner. A normal compatible new installation needs no maintenance action. Existing active pairs always retain their layout even under a combined creation policy; existing combined ledgers are never silently migrated.

The target has no independent scope registry. Assigned owner IDs and opaque aliases are never recomputed or relabeled. A raw scope token is reserved in the intent ledger when its store is initialized; reservation prevents allocator collisions and is **not** a verified owner binding. Maintenance scope listing/binding also recognizes rows present only in the target. Existing capability authorization still runs at each public read.

## Explicit trusted cutover

This is a maintenance API, not a gateway route, automatic device operation, or proof of authorization:

```python
status = AgentCallStore.terminal_storage_status(intent_path)
AgentCallStore.activate_terminal_split(
    intent_path,
    expected_ledger_id=status["ledger_id"],
    terminal_path=approved_target_path,
    writers_quiesced=True,
)
```

Before invoking it, the operator must be authorized for both exact paths, independently verify that `ledger_id` belongs to the intended call ledger, and drain every old writer and in-flight use of that ledger. Include other gateway processes, background receipt owners, direct read/wait persistence, and accepted action work outside SQLite transactions. Use the deployment's supported shutdown/drain procedure. The helper never stops a robot, cancels a task, resets delivery state, or repairs a receipt by executing an action. The boolean is the caller's assertion; absence of an SQLite lock does not establish quiescence.

The updated constructor can add a ledger/layout identifier to a recognized old database without creating a runtime or binding a robot identity. That remains combined storage until explicit activation. Scope binding continues to require its separate verified-owner operation.

Activation first commits a pending exact pair, creates/verifies the terminal file, then installs `BEFORE INSERT`, `BEFORE UPDATE`, and `BEFORE DELETE` abort triggers on the old terminal table and commits `active` in the same intent transaction. Pair IDs are assigned UUIDs, not content checksums or authorization credentials. Same physical files, unrelated existing targets, different pending targets, and wrong pair records are rejected. An already active missing file cannot be recreated through this helper.

This creation-policy compatibility claim addresses only the split-layout regression against the accepted pre-split, already hash-free combined reader. It does not prove rollback to arbitrary historical binaries: older derived-scope/request-fingerprint logic and other changed map/memory formats have separate data-reader compatibility boundaries. A successful software revision probe alone does not prove storage compatibility. Whole-chain historical rollback remains a release-level open item.

There is no cross-file atomic transaction. Interrupted pending initialization may leave an inert target; resume only the same recorded pair. An empty target without a committed identity is not adopted. After activation, missing/unavailable target storage is an explicit receipt outage while healthy intent operations remain usable. Operators must not clear pending/unknown requests or invent replacement request IDs.

## Legacy receipts, revision order and recovery

The old terminal table is retained and immutable. Reads first open the healthy paired target. Only an absent target row permits lookup in sealed legacy storage. A missing target file, I/O failure or pairing conflict never masquerades as row absence and never returns older legacy history as the current receipt.

On first target write, the backend copies an existing legacy row exactly, including its original scope, first gateway identity, JSON, recorded time, source and revision. That seed commits separately before candidate CAS. A crash between seed and CAS leaves a truthful old receipt; a retry updates only persistence. A lost acknowledgement after candidate commit is idempotent.

Candidates from the same source compare revisions, not clocks or writer arrival. Older revisions are stale no-ops; equal revision/content is unchanged; equal revision/different content is explicit conflict. Different sources are noncomparable. Legacy NULL source/revision stays readable and rejects versioned adoption. The target never gains a fresh-scope loophole around a historical conflict. All supported target creation paths establish dominance over immutable legacy; unchanged reads do not repeatedly copy an entire history.

`terminal_versions` reads at most 64 metadata rows per batch, falling back only for target misses. No bulk backfill is required, and no history deletion is included. TaskManager source retention and its cleanup prerequisites remain a separate concern.

## Concurrency and operational limits

Terminal runtime operations do not create/register scopes, initialize the intent schema, or open intent writer transactions. Small read-only authority/legacy lookups finish before any target writer transaction. Snapshot ownership and JSON preparation remain outside SQLite writer transactions. A bounded scope-race retry repeats only persistence authorization, never action execution.

Physical separation removes the demonstrated same-file terminal-writer versus durable-begin writer-lock collision. WAL alone does not allow two writers in the same file. Shared storage bandwidth, CPU, flush latency and brief read/commit contention remain; this is not total resource isolation or a throughput guarantee. Intent begin/finish still need their own successful durable operations. A terminal receipt and a request result were not one atomic transaction before the split and remain independently committed.

The old-table triggers fence mutations, but an old binary can still take a write transaction for a no-op and can execute actions outside it. Running old and new writers together is unsupported. Do not downgrade without an explicit quiesced reverse reconciliation of newer target receipts.

Both files retain SQLite `synchronous=FULL`; new files use owner-only modes, and target creation synchronizes its parent directory before active publication. Process-exit/reopen fixtures test acknowledged states and no replay. Those fixtures do not prove power-loss or media durability on deployment filesystems. Actual deployment filesystem durability remains unverified.

Back up and restore the paired files and applicable SQLite journals at a quiesced consistent boundary. Copying a live database file alone, independently rolling back one file, deleting a missing active target, or restoring an old intent ledger can lose known records. No automatic repair of those operational mistakes is promised. In particular, restoring old request intentions can compromise deduplication and must never be treated as permission to replay actions.
