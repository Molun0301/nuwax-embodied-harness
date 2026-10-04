"""Durable, coalescing desired-vector work; foundation only, not yet integrated.

There is one current row per (library incarnation, source kind, public ID), one
claim slot, fixed aggregate counters, and bounded receipt detail. No source
payload, image, embedding, worker thread, or model is held here. Consumers load
one claimed source and infer OUTSIDE a transaction, then supply a short SQL-only
publication callback. This component does not validate vector dimensions,
artifact bytes, dependency graphs, or actual model versions.

Changes to desired indexing inputs/liveness use ``accept``/``delete`` with an
expected version. Proven input-equivalent metadata writes may keep this indexing
generation, but require their own business-record CAS inside the same owner's
transaction. The integration layer owns that equivalence proof and must retain
current payload/filter validation. Unknown equivalence requires new desired work.
Callbacks use the supplied SQLiteTransaction, never an independent connection or
backend. An injected non-SQLite/different-owner backend needs an explicit adapter
and reconciliation protocol; this module makes no cross-backend atomicity claim.

A claim lease, activation change, stop, or recovery only fences publication. It
cannot stop uncooperative inference. Normal supersession/activation keeps the
occupied slot until the old caller returns. Explicit recovery/expired leases
can free it; the service must separately own/join the actual model worker before
starting another inference on a non-thread-safe model.
"""

from __future__ import annotations

import math
import threading
import time
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, Dict, Optional

from services.memory.sqlite_storage import SQLiteStorage, SQLiteTransaction


class OutboxConflict(RuntimeError):
    """Expected source, library incarnation, or worker ownership is stale."""


@dataclass(frozen=True)
class LibraryIdentity:
    name: str
    incarnation: str


@dataclass(frozen=True)
class WorkKey:
    library_incarnation: str
    source_kind: str
    point_id: str

    def sql(self):
        return (self.library_incarnation, self.source_kind, self.point_id)


@dataclass(frozen=True)
class SourceVersion:
    """Persistent source projection identity/revision, not a wall timestamp.

    A repository may also have an independently advancing business-record CAS
    revision. Proven input-equivalent metadata writes must preserve this indexing
    generation; they must never use it instead of their business-record CAS.
    """

    key: WorkKey
    record_incarnation: str
    revision: int


@dataclass(frozen=True)
class Receipt:
    token: str
    version: SourceVersion


@dataclass(frozen=True)
class WorkClaim:
    receipt: Receipt
    token: str
    worker_token: str
    activation_generation: int
    lease_expires_at: float
    attempt: int


@dataclass(frozen=True)
class CompletionResult:
    published: bool
    reason: str


@dataclass(frozen=True)
class RetryPolicy:
    max_attempts: int = 5
    base_delay: float = 1.0
    max_delay: float = 60.0

    def __post_init__(self) -> None:
        if not isinstance(self.max_attempts, int) or not 1 <= self.max_attempts <= 1000:
            raise ValueError("max_attempts must be an integer in [1, 1000]")
        if not all(math.isfinite(x) for x in (self.base_delay, self.max_delay)):
            raise ValueError("Retry delays must be finite")
        if not 0 <= self.base_delay <= self.max_delay:
            raise ValueError("Retry delays require 0 <= base_delay <= max_delay")


_CALLBACK_CONTEXT = threading.local()

_VERSION_WHERE = "library_incarnation=? AND source_kind=? AND point_id=?"
_TERMINAL = ("completed_fresh", "completed_reused", "failed_terminal")
_COUNTERS = (
    "accepted",
    "completed_fresh",
    "completed_reused",
    "superseded",
    "cancelled_deleted",
    "failed_terminal",
    "attempts_started",
    "attempts_completed",
    "attempts_failed",
    "attempts_superseded",
    "attempts_cancelled",
    "attempts_abandoned",
    "stale_results",
)


class VectorizationOutbox:
    """Single-worker scheduling authority, safe across threads/shared owners.

    SQL source/publication callbacks and methods may nest in storage.transaction.
    Returned receipts/claims remain provisional until that outer commit. The
    optional wake callback is a hint run only after commit. History retention is
    a count of recent accepted versions, not an unlimited per-request audit log.
    Current receipts remain inspectable even when their history row was pruned.
    The 256-character key/name bounds are INTERNAL key contracts;
    integration must map longer legacy public IDs exactly, not reject old data.
    """

    def __init__(
        self,
        storage: SQLiteStorage,
        *,
        retry_policy: Optional[RetryPolicy] = None,
        history_limit: int = 512,
        clock: Callable[[], float] = time.time,
        wake: Optional[Callable[[], None]] = None,
    ) -> None:
        if not isinstance(storage, SQLiteStorage):
            raise TypeError("Durable outbox requires an explicit shared SQLiteStorage owner")
        if not isinstance(history_limit, int) or not 0 <= history_limit <= 100000:
            raise ValueError("history_limit must be in [0, 100000]")
        self.storage = storage
        self.retry_policy = retry_policy or RetryPolicy()
        self.history_limit = history_limit
        self._clock = clock
        self._wake = wake
        self._init_schema()

    @contextmanager
    def _transaction(self):
        if id(self.storage) in getattr(_CALLBACK_CONTEXT, "owners", ()):
            raise OutboxConflict("SQL callbacks cannot reenter scheduler mutations; use an outer transaction")
        with self.storage.transaction() as tx:
            yield tx

    def _sql_callback(self, tx, callback, *args):
        owners = getattr(_CALLBACK_CONTEXT, "owners", None)
        if owners is None:
            owners = _CALLBACK_CONTEXT.owners = set()
        owners.add(id(self.storage))
        try:
            callback(tx, *args)
        finally:
            owners.remove(id(self.storage))

    def _init_schema(self) -> None:
        statements = (
            """CREATE TABLE IF NOT EXISTS memory_vectorization_control (
                singleton INTEGER PRIMARY KEY CHECK (singleton=1),
                activation_generation INTEGER NOT NULL DEFAULT 0,
                active_library TEXT, worker_token TEXT,
                next_ticket INTEGER NOT NULL DEFAULT 0,
                last_now REAL NOT NULL DEFAULT 0)""",
            """CREATE TABLE IF NOT EXISTS memory_vectorization_libraries (
                name TEXT PRIMARY KEY, incarnation TEXT NOT NULL UNIQUE)""",
            """CREATE TABLE IF NOT EXISTS memory_vectorization_state (
                library_incarnation TEXT NOT NULL, source_kind TEXT NOT NULL,
                point_id TEXT NOT NULL, record_incarnation TEXT NOT NULL,
                source_revision INTEGER NOT NULL, receipt TEXT NOT NULL UNIQUE,
                fingerprint TEXT NOT NULL, status TEXT NOT NULL,
                ticket INTEGER NOT NULL, first_dirty_at REAL NOT NULL,
                updated_at REAL NOT NULL, next_attempt_at REAL NOT NULL,
                attempts INTEGER NOT NULL, max_attempts INTEGER NOT NULL,
                base_delay REAL NOT NULL, max_delay REAL NOT NULL,
                error_category TEXT, error_message TEXT,
                PRIMARY KEY (library_incarnation, source_kind, point_id))""",
            """CREATE INDEX IF NOT EXISTS memory_vectorization_ready
                ON memory_vectorization_state (library_incarnation, status, ticket)""",
            """CREATE INDEX IF NOT EXISTS memory_vectorization_backoff
                ON memory_vectorization_state (library_incarnation, status, next_attempt_at, ticket)""",
            """CREATE TABLE IF NOT EXISTS memory_vectorization_claim (
                singleton INTEGER PRIMARY KEY CHECK (singleton=1),
                token TEXT NOT NULL, receipt TEXT NOT NULL, worker_token TEXT NOT NULL,
                activation_generation INTEGER NOT NULL, expires_at REAL NOT NULL,
                disposition TEXT NOT NULL,
                library_incarnation TEXT NOT NULL, source_kind TEXT NOT NULL,
                point_id TEXT NOT NULL, record_incarnation TEXT NOT NULL,
                source_revision INTEGER NOT NULL, fingerprint TEXT NOT NULL,
                attempt INTEGER NOT NULL)""",
            """CREATE TABLE IF NOT EXISTS memory_vectorization_counters (
                name TEXT PRIMARY KEY, value INTEGER NOT NULL DEFAULT 0)""",
            """CREATE TABLE IF NOT EXISTS memory_vectorization_receipts (
                sequence INTEGER PRIMARY KEY AUTOINCREMENT, token TEXT NOT NULL UNIQUE,
                outcome TEXT NOT NULL, accepted_at REAL NOT NULL)""",
        )
        with self._transaction() as tx:
            for statement in statements:
                tx.execute(statement)
            tx.execute("INSERT OR IGNORE INTO memory_vectorization_control(singleton) VALUES (1)")
            tx.executemany(
                "INSERT OR IGNORE INTO memory_vectorization_counters(name) VALUES (?)", [(name,) for name in _COUNTERS]
            )
            self._prune(tx)

    @staticmethod
    def _bounded(value: str, name: str, limit: int = 256) -> str:
        if not isinstance(value, str) or not value or len(value) > limit:
            raise ValueError(f"{name} must be a nonempty string of at most {limit} characters")
        return value

    def _key(self, key: WorkKey) -> None:
        for name, value in zip(("library incarnation", "source kind", "point ID"), key.sql()):
            self._bounded(value, name)

    def _now(self, tx: SQLiteTransaction) -> float:
        value = float(self._clock())
        if not math.isfinite(value) or value < 0:
            raise ValueError("clock must return finite nonnegative epoch seconds")
        last = tx.execute("SELECT last_now FROM memory_vectorization_control WHERE singleton=1").fetchone()[0]
        # Backwards wall-clock jumps must not prematurely make work eligible.
        # Deadlines can be delayed until wall time catches up; no latency promise.
        value = max(value, last)
        tx.execute("UPDATE memory_vectorization_control SET last_now=? WHERE singleton=1", (value,))
        return value

    @staticmethod
    def _control(tx: SQLiteTransaction):
        return tx.execute("SELECT * FROM memory_vectorization_control WHERE singleton=1").fetchone()

    @staticmethod
    def _row(tx: SQLiteTransaction, key: WorkKey):
        return tx.execute("SELECT * FROM memory_vectorization_state WHERE " + _VERSION_WHERE, key.sql()).fetchone()

    @staticmethod
    def _receipt(row) -> Receipt:
        key = WorkKey(row["library_incarnation"], row["source_kind"], row["point_id"])
        version = SourceVersion(key, row["record_incarnation"], row["source_revision"])
        return Receipt(row["receipt"], version)

    @staticmethod
    def _count(tx: SQLiteTransaction, name: str, amount: int = 1) -> None:
        tx.execute("UPDATE memory_vectorization_counters SET value=value+? WHERE name=?", (amount, name))

    @staticmethod
    def _ticket(tx: SQLiteTransaction) -> int:
        tx.execute("UPDATE memory_vectorization_control SET next_ticket=next_ticket+1 WHERE singleton=1")
        return tx.execute("SELECT next_ticket FROM memory_vectorization_control WHERE singleton=1").fetchone()[0]

    def _prune(self, tx: SQLiteTransaction) -> None:
        tx.execute(
            """DELETE FROM memory_vectorization_receipts WHERE sequence <= COALESCE(
            (SELECT sequence FROM memory_vectorization_receipts ORDER BY sequence DESC LIMIT 1 OFFSET ?), -1)""",
            (self.history_limit,),
        )

    def _notify(self, tx: SQLiteTransaction) -> None:
        if self._wake is not None:
            tx.after_commit(self._wake, deduplicate=True)

    @staticmethod
    def _require_library(tx: SQLiteTransaction, incarnation: str) -> None:
        if (
            tx.execute("SELECT 1 FROM memory_vectorization_libraries WHERE incarnation=?", (incarnation,)).fetchone()
            is None
        ):
            raise OutboxConflict("Library incarnation is deleted or unknown")

    def create_library(self, name: str) -> LibraryIdentity:
        """Get/create an identity; recreate after deletion gets a new UUID."""
        self._bounded(name, "library name")
        with self._transaction() as tx:
            row = tx.execute("SELECT incarnation FROM memory_vectorization_libraries WHERE name=?", (name,)).fetchone()
            if row is not None:
                return LibraryIdentity(name, row[0])
            library = LibraryIdentity(name, uuid.uuid4().hex)
            tx.execute("INSERT INTO memory_vectorization_libraries VALUES (?, ?)", (name, library.incarnation))
            return library

    def activate(self, library: Optional[LibraryIdentity]) -> int:
        """Advance persistent activation generation, including A -> A / A -> B -> A.

        Pending work in other libraries is suspended, never dropped. An active
        attempt is logically cancelled; its occupied slot remains until returned.
        """
        with self._transaction() as tx:
            if library is not None:
                row = tx.execute(
                    "SELECT incarnation FROM memory_vectorization_libraries WHERE name=?", (library.name,)
                ).fetchone()
                if row is None or row[0] != library.incarnation:
                    raise OutboxConflict("Stale library identity")
            now = self._now(tx)
            self._invalidate_claim(tx, "cancelled", now)
            tx.execute(
                """UPDATE memory_vectorization_control SET
                activation_generation=activation_generation+1, active_library=? WHERE singleton=1""",
                (library.incarnation if library is not None else None,),
            )
            self._notify(tx)
            return self._control(tx)["activation_generation"]

    def current(self, key: WorkKey) -> Optional[Receipt]:
        with self.storage.read_snapshot() as tx:
            row = self._row(tx, key)
            return self._receipt(row) if row is not None else None

    def inspect(self, key: WorkKey) -> Optional[Dict[str, object]]:
        """Single-key bounded diagnostic projection; never source payload."""
        with self.storage.read_snapshot() as tx:
            row = self._row(tx, key)
            return dict(row) if row is not None else None

    def accept(
        self,
        key: WorkKey,
        *,
        expected: Optional[SourceVersion],
        write_source: Callable[[SQLiteTransaction, SourceVersion], None],
    ) -> Receipt:
        """Atomically write source and accept a new desired version.

        ``expected=None`` means create, and conflicts with an existing row.
        Updates require the exact indexing version from the source snapshot;
        stale input generations cannot acquire a new revision. The source SQL
        must also CAS its business revision if metadata can advance independently.
        This is NOT a duplicate/no-op API: each successful call accepts once.
        The source callback should additionally CAS source SQL during migration
        or when legacy writers exist. Raising rolls back every participant.
        Callbacks perform SQL only; reentering scheduler mutations is rejected.
        To dirty dependencies, call accept for each key AFTER this method returns,
        inside the same outer storage.transaction. All participants commit once.
        """
        self._key(key)
        with self._transaction() as tx:
            self._require_library(tx, key.library_incarnation)
            old = self._row(tx, key)
            if (old is None and expected is not None) or (old is not None and self._receipt(old).version != expected):
                raise OutboxConflict("Source version changed; reload before applying the mutation")
            now = self._now(tx)
            version = SourceVersion(
                key, old["record_incarnation"] if old else uuid.uuid4().hex, old["source_revision"] + 1 if old else 1
            )
            receipt = Receipt(uuid.uuid4().hex, version)
            self._sql_callback(tx, write_source, version)
            self._require_library(tx, key.library_incarnation)
            current = self._row(tx, key)
            def scheduling_state(row):
                return {key: row[key] for key in row.keys() if key != "fingerprint"} if row is not None else None
            if scheduling_state(current) != scheduling_state(old):
                raise OutboxConflict("Source callback changed this key's scheduling state")
            first_dirty = now
            ticket = self._ticket(tx)
            if old is not None:
                first_dirty = now if old["status"].startswith("completed_") else old["first_dirty_at"]
                if old["status"] in ("pending", "backoff"):
                    ticket = old["ticket"]
                if old["status"] not in _TERMINAL:
                    self._settle_receipt(tx, old["receipt"], "superseded")
                self._invalidate_claim(tx, "superseded", now, receipt=old["receipt"], requeue=False)
            policy = self.retry_policy
            tx.execute(
                """INSERT INTO memory_vectorization_state (
                library_incarnation, source_kind, point_id, record_incarnation,
                source_revision, receipt, fingerprint, status, ticket, first_dirty_at,
                updated_at, next_attempt_at, attempts, max_attempts, base_delay, max_delay)
                VALUES (?, ?, ?, ?, ?, ?, ?, 'pending', ?, ?, ?, ?, 0, ?, ?, ?)
                ON CONFLICT(library_incarnation, source_kind, point_id) DO UPDATE SET
                record_incarnation=excluded.record_incarnation, source_revision=excluded.source_revision,
                receipt=excluded.receipt, fingerprint=excluded.fingerprint, status='pending',
                ticket=excluded.ticket, first_dirty_at=excluded.first_dirty_at, updated_at=excluded.updated_at,
                next_attempt_at=excluded.next_attempt_at, attempts=0, max_attempts=excluded.max_attempts,
                base_delay=excluded.base_delay, max_delay=excluded.max_delay, error_category=NULL, error_message=NULL""",
                key.sql()
                + (
                    version.record_incarnation,
                    version.revision,
                    receipt.token,
                    "",  # 旧列仅保留结构兼容，不再参与身份或一致性判断。
                    ticket,
                    first_dirty,
                    now,
                    now,
                    policy.max_attempts,
                    policy.base_delay,
                    policy.max_delay,
                ),
            )
            self._admit_receipt(tx, receipt.token, now)
            self._notify(tx)
            return receipt

    def _admit_receipt(self, tx: SQLiteTransaction, token: str, now: float) -> None:
        self._count(tx, "accepted")
        tx.execute(
            "INSERT INTO memory_vectorization_receipts(token, outcome, accepted_at) VALUES (?, 'pending', ?)",
            (token, now),
        )
        self._prune(tx)

    def _settle_receipt(self, tx: SQLiteTransaction, token: str, outcome: str) -> None:
        self._count(tx, outcome)
        tx.execute("UPDATE memory_vectorization_receipts SET outcome=? WHERE token=?", (outcome, token))

    def delete(
        self,
        key: WorkKey,
        *,
        expected: SourceVersion,
        delete_source: Callable[[SQLiteTransaction, SourceVersion], None],
    ) -> None:
        """Delete source/visibility with work; recreation cannot inherit identity."""
        with self._transaction() as tx:
            row = self._row(tx, key)
            if row is None or self._receipt(row).version != expected:
                raise OutboxConflict("Source version changed or was deleted")
            self._sql_callback(tx, delete_source, expected)
            current = self._row(tx, key)
            if current is None or dict(current) != dict(row):
                raise OutboxConflict("Delete callback changed this key's scheduling state")
            self._require_library(tx, key.library_incarnation)
            now = self._now(tx)
            if row["status"] not in _TERMINAL:
                self._settle_receipt(tx, row["receipt"], "cancelled_deleted")
            self._invalidate_claim(tx, "cancelled", now, receipt=row["receipt"], requeue=False)
            tx.execute("DELETE FROM memory_vectorization_state WHERE " + _VERSION_WHERE, key.sql())
            self._notify(tx)

    def delete_library(
        self,
        library: LibraryIdentity,
        *,
        delete_source: Callable[[SQLiteTransaction, LibraryIdentity], None],
    ) -> None:
        """Delete one incarnation with a bounded-memory set operation."""
        with self._transaction() as tx:
            row = tx.execute(
                "SELECT incarnation FROM memory_vectorization_libraries WHERE name=?", (library.name,)
            ).fetchone()
            if row is None or row[0] != library.incarnation:
                raise OutboxConflict("Library was deleted/recreated")
            self._sql_callback(tx, delete_source, library)
            current = tx.execute(
                "SELECT incarnation FROM memory_vectorization_libraries WHERE name=?", (library.name,)
            ).fetchone()
            if current is None or current[0] != library.incarnation:
                raise OutboxConflict("Delete callback changed this library's lifecycle")
            now = self._now(tx)
            slot = tx.execute(
                """SELECT c.receipt FROM memory_vectorization_claim c
                JOIN memory_vectorization_state s ON s.receipt=c.receipt WHERE s.library_incarnation=?""",
                (library.incarnation,),
            ).fetchone()
            if slot is not None:
                self._invalidate_claim(tx, "cancelled", now, receipt=slot[0], requeue=False)
            predicate = "library_incarnation=? AND status IN ('pending', 'backoff', 'in_flight')"
            count = tx.execute(
                "SELECT count(*) FROM memory_vectorization_state WHERE " + predicate, (library.incarnation,)
            ).fetchone()[0]
            tx.execute(
                """UPDATE memory_vectorization_receipts SET outcome='cancelled_deleted' WHERE token IN (
                SELECT receipt FROM memory_vectorization_state WHERE """
                + predicate
                + ")",
                (library.incarnation,),
            )
            self._count(tx, "cancelled_deleted", count)
            tx.execute("DELETE FROM memory_vectorization_state WHERE library_incarnation=?", (library.incarnation,))
            tx.execute("DELETE FROM memory_vectorization_libraries WHERE incarnation=?", (library.incarnation,))
            tx.execute(
                """UPDATE memory_vectorization_control SET active_library=NULL,
                activation_generation=activation_generation+1 WHERE active_library=?""",
                (library.incarnation,),
            )
            self._notify(tx)

    def start_worker(self, *, recover: bool = False) -> str:
        """Allocate the sole run token. Recovery explicitly fences a prior run.

        ``recover=True`` is for a known dead process or deliberately abandoned
        lease. It does not certify that an old inference thread has stopped.
        Retry attempts and deadlines survive; recovered claims consume budget.
        """
        with self._transaction() as tx:
            control = self._control(tx)
            slot = tx.execute("SELECT token FROM memory_vectorization_claim").fetchone()
            if (control["worker_token"] is not None or slot is not None) and not recover:
                raise OutboxConflict("Worker/claim already owned; finish it or explicitly recover")
            self._invalidate_claim(tx, "abandoned", self._now(tx))
            tx.execute("DELETE FROM memory_vectorization_claim")
            token = uuid.uuid4().hex
            tx.execute("UPDATE memory_vectorization_control SET worker_token=? WHERE singleton=1", (token,))
            self._notify(tx)
            return token

    def stop_worker(self, worker_token: str) -> bool:
        with self._transaction() as tx:
            if self._control(tx)["worker_token"] != worker_token:
                return False
            self._invalidate_claim(tx, "cancelled", self._now(tx))
            tx.execute("UPDATE memory_vectorization_control SET worker_token=NULL WHERE singleton=1")
            return True

    def _return_current(self, tx: SQLiteTransaction, receipt: str, now: float, category: str) -> None:
        row = tx.execute(
            "SELECT * FROM memory_vectorization_state WHERE receipt=? AND status='in_flight'", (receipt,)
        ).fetchone()
        if row is None:
            return
        if row["attempts"] >= row["max_attempts"]:
            self._settle_receipt(tx, receipt, "failed_terminal")
            tx.execute(
                """UPDATE memory_vectorization_state SET status='failed_terminal',
                error_category='retry_exhausted', error_message=?, updated_at=? WHERE receipt=?""",
                (category + ": attempt budget exhausted", now, receipt),
            )
        else:
            tx.execute(
                """UPDATE memory_vectorization_state SET status='pending', ticket=?,
                next_attempt_at=?, updated_at=?, error_category=? WHERE receipt=?""",
                (self._ticket(tx), now, now, category, receipt),
            )

    def _invalidate_claim(
        self,
        tx: SQLiteTransaction,
        disposition: str,
        now: float,
        *,
        receipt: Optional[str] = None,
        requeue: bool = True,
    ) -> None:
        slot = tx.execute("SELECT * FROM memory_vectorization_claim WHERE singleton=1").fetchone()
        if slot is None or slot["disposition"] != "active" or (receipt is not None and slot["receipt"] != receipt):
            return
        self._count(tx, "attempts_" + disposition)
        tx.execute("UPDATE memory_vectorization_claim SET disposition=? WHERE singleton=1", (disposition,))
        if requeue:
            self._return_current(tx, slot["receipt"], now, disposition)

    def _recover_expired(self, tx: SQLiteTransaction, now: float) -> None:
        slot = tx.execute("SELECT * FROM memory_vectorization_claim WHERE singleton=1").fetchone()
        if slot is not None and slot["expires_at"] <= now:
            self._invalidate_claim(tx, "abandoned", now)
            tx.execute("DELETE FROM memory_vectorization_claim WHERE token=?", (slot["token"],))

    def claim_next(self, worker_token: str, *, lease_seconds: float = 300.0) -> Optional[WorkClaim]:
        """Claim at most ONE ready FIFO key; never return a backlog list.

        Each call first promotes at most 64 due backoff keys in deadline order.
        FIFO is exact within the ready set, not across unpromoted due backoff
        rows. A due key has finitely many earlier deadlines/tickets, and is
        promoted after finitely many calls; then its finite ready predecessors
        are serviced. New work cannot starve it if attempts keep finishing.
        Unserviced pending updates retain tickets; serviced retries/superseded
        work move to the tail. There is no completion-latency guarantee.
        """
        if not math.isfinite(lease_seconds) or not 0 < lease_seconds <= 86400:
            raise ValueError("lease_seconds must be finite and in (0, 86400]")
        with self._transaction() as tx:
            control = self._control(tx)
            if control["worker_token"] != worker_token:
                raise OutboxConflict("Worker run token is no longer current")
            now = self._now(tx)
            self._recover_expired(tx, now)
            if tx.execute("SELECT 1 FROM memory_vectorization_claim").fetchone() is not None:
                return None
            # Promote a fixed SQL page, using the deadline index. Ready selection
            # then seeks a FIFO index without sorting/scanning the whole backlog.
            tx.execute(
                """UPDATE memory_vectorization_state SET status='pending' WHERE receipt IN (
                SELECT receipt FROM memory_vectorization_state
                WHERE library_incarnation=? AND status='backoff' AND next_attempt_at<=?
                ORDER BY next_attempt_at, ticket LIMIT 64)""",
                (control["active_library"], now),
            )
            row = tx.execute(
                """SELECT * FROM memory_vectorization_state
                WHERE library_incarnation=? AND status='pending'
                ORDER BY ticket LIMIT 1""",
                (control["active_library"],),
            ).fetchone()
            if row is None:
                return None
            claim = WorkClaim(
                self._receipt(row),
                uuid.uuid4().hex,
                worker_token,
                control["activation_generation"],
                now + lease_seconds,
                row["attempts"] + 1,
            )
            tx.execute(
                """INSERT INTO memory_vectorization_claim VALUES
                (1, ?, ?, ?, ?, ?, 'active', ?, ?, ?, ?, ?, ?, ?)""",
                (claim.token, claim.receipt.token, worker_token, claim.activation_generation, claim.lease_expires_at)
                + claim.receipt.version.key.sql()
                + (
                    claim.receipt.version.record_incarnation,
                    claim.receipt.version.revision,
                    "",
                    claim.attempt,
                ),
            )
            tx.execute(
                "UPDATE memory_vectorization_state SET status='in_flight', attempts=attempts+1 WHERE receipt=?",
                (claim.receipt.token,),
            )
            self._count(tx, "attempts_started")
            return claim

    def next_wakeup_at(self, worker_token: str) -> Optional[float]:
        """Indexed wall-time hint; no backlog scan or inference-stop guarantee.

        A caller may wait on its wake Event until this deadline, then recheck.
        A missing deadline means no scheduled retry is currently due. Callers
        must cap Event waits with a recovery poll, because wake hints may fail or
        be lost; the durable queue, not a notification, remains authoritative.
        """
        with self.storage.read_snapshot() as tx:
            control = self._control(tx)
            if control["worker_token"] != worker_token:
                raise OutboxConflict("Worker run token is no longer current")
            now = float(self._clock())
            if not math.isfinite(now) or now < 0:
                raise ValueError("clock must return finite nonnegative epoch seconds")
            now = max(now, control["last_now"])
            slot = tx.execute("SELECT expires_at FROM memory_vectorization_claim WHERE singleton=1").fetchone()
            if slot is not None:
                return max(now, slot[0])
            ready = tx.execute(
                """SELECT 1 FROM memory_vectorization_state
                WHERE library_incarnation=? AND status='pending' LIMIT 1""",
                (control["active_library"],),
            ).fetchone()
            if ready is not None:
                return now
            row = tx.execute(
                """SELECT next_attempt_at FROM memory_vectorization_state
                WHERE library_incarnation=? AND status='backoff' ORDER BY next_attempt_at, ticket LIMIT 1""",
                (control["active_library"],),
            ).fetchone()
            return max(now, row[0]) if row is not None else None

    def _claim_check(self, tx: SQLiteTransaction, claim: WorkClaim, now: float) -> str:
        slot = tx.execute("SELECT * FROM memory_vectorization_claim WHERE singleton=1").fetchone()
        if slot is None or slot["token"] != claim.token or slot["worker_token"] != claim.worker_token:
            return "stale_claim"
        if (
            self._receipt(slot) != claim.receipt
            or slot["attempt"] != claim.attempt
            or slot["activation_generation"] != claim.activation_generation
        ):
            return "stale_identity"
        if slot["disposition"] != "active":
            return slot["disposition"]
        control = self._control(tx)
        if (
            control["worker_token"] != claim.worker_token
            or control["activation_generation"] != claim.activation_generation
        ):
            return "stale_scope"
        if control["active_library"] != claim.receipt.version.key.library_incarnation:
            return "stale_library"
        row = self._row(tx, claim.receipt.version.key)
        if row is None or row["status"] != "in_flight" or self._receipt(row) != claim.receipt:
            return "stale_source"
        if row["attempts"] != claim.attempt:
            return "stale_attempt"
        if slot["expires_at"] <= now:
            return "lease_expired"
        return "current"

    def _discard(self, tx: SQLiteTransaction, claim: WorkClaim, reason: str, now: float) -> CompletionResult:
        self._count(tx, "stale_results")
        slot = tx.execute("SELECT * FROM memory_vectorization_claim WHERE singleton=1").fetchone()
        # A fabricated/mismatching identity may NOT release even its named slot.
        # Only the exact captured receipt/generation/attempt can release it.
        if (
            slot is not None
            and slot["token"] == claim.token
            and slot["worker_token"] == claim.worker_token
            and self._receipt(slot) == claim.receipt
            and slot["attempt"] == claim.attempt
            and slot["activation_generation"] == claim.activation_generation
            and reason in ("cancelled", "superseded", "abandoned", "lease_expired")
        ):
            if reason == "lease_expired":
                self._invalidate_claim(tx, "abandoned", now)
            tx.execute("DELETE FROM memory_vectorization_claim WHERE token=?", (claim.token,))
        return CompletionResult(False, reason)

    def publish_partial(
        self,
        claim: WorkClaim,
        *,
        publish: Callable[[SQLiteTransaction, WorkClaim], None],
    ) -> CompletionResult:
        """Guard and publish valid named-vector progress without settling work.

        Keeps the receipt in flight, its claim slot, attempts, and completion
        counters unchanged. A later failure/backoff does not undo this committed
        SQL progress. Rejected partial results never release a slot; callers
        must end the attempt via complete/fail or explicitly recover its lease.
        Integration must validate each vector's provenance and modality
        eligibility; this method never declares full/degraded success.
        """
        return self._publish_current(claim, publish, settle=False)

    def complete(
        self,
        claim: WorkClaim,
        *,
        publish: Callable[[SQLiteTransaction, WorkClaim], None],
    ) -> CompletionResult:
        """Guard then atomically publish SQL vectors/provenance and settle receipt.

        A callback exception rolls back vectors, status, counters, and hooks.
        The claim remains available for an explicit fail/retry or lease recovery.
        Callbacks must not run inference or perform nontransactional I/O.
        """
        return self._publish_current(claim, publish, settle=True)

    def _publish_current(self, claim, publish, *, settle):
        with self._transaction() as tx:
            now = self._now(tx)
            reason = self._claim_check(tx, claim, now)
            if reason != "current":
                if settle:
                    return self._discard(tx, claim, reason, now)
                # A partial result is not the end of the physical attempt. Even
                # stale partial notifications must not free its occupied slot.
                self._count(tx, "stale_results")
                return CompletionResult(False, reason)
            self._sql_callback(tx, publish, claim)
            now = self._now(tx)
            if self._claim_check(tx, claim, now) != "current":
                raise OutboxConflict("Publication callback changed its claim/source scope or lease expired")
            if settle:
                self._settle_receipt(tx, claim.receipt.token, "completed_fresh")
                self._count(tx, "attempts_completed")
                tx.execute(
                    """UPDATE memory_vectorization_state SET status='completed_fresh',
                    updated_at=?, error_category=NULL, error_message=NULL WHERE receipt=?""",
                    (now, claim.receipt.token),
                )
                tx.execute("DELETE FROM memory_vectorization_claim WHERE token=?", (claim.token,))
            self._notify(tx)
            return CompletionResult(True, "completed_fresh" if settle else "partial_published")

    def fail(self, claim: WorkClaim, *, category: str, message: str = "", permanent: bool = False) -> CompletionResult:
        """Persist a bounded, non-secret diagnostic and a capped retry deadline.

        All started attempts count against the per-version budget, including
        abandoned/cancelled ones. Explicit retry creates a NEW accepted receipt,
        preserving the prior terminal failure and accounting it only once.
        """
        self._bounded(category, "error category", 64)
        if not isinstance(message, str):
            raise TypeError("error message must be a string")
        with self._transaction() as tx:
            now = self._now(tx)
            reason = self._claim_check(tx, claim, now)
            if reason != "current":
                return self._discard(tx, claim, reason, now)
            row = self._row(tx, claim.receipt.version.key)
            exhausted = row["attempts"] >= row["max_attempts"]
            terminal = permanent or exhausted
            self._count(tx, "attempts_failed")
            if terminal:
                self._settle_receipt(tx, claim.receipt.token, "failed_terminal")
            delay = min(row["max_delay"], row["base_delay"] * (2.0 ** min(row["attempts"] - 1, 1023)))
            tx.execute(
                """UPDATE memory_vectorization_state SET status=?, ticket=?, next_attempt_at=?,
                updated_at=?, error_category=?, error_message=? WHERE receipt=?""",
                (
                    "failed_terminal" if terminal else "backoff",
                    self._ticket(tx),
                    now + delay,
                    now,
                    ("permanent:" if permanent else "retry_exhausted:" if exhausted else "") + category,
                    message[:1024],
                    claim.receipt.token,
                ),
            )
            tx.execute("DELETE FROM memory_vectorization_claim WHERE token=?", (claim.token,))
            self._notify(tx)
            return CompletionResult(False, "failed_terminal" if terminal else "pending_backoff")

    def retry(self, receipt: Receipt) -> Receipt:
        """Explicit repair of a CURRENT failed version; never revive stale work."""
        with self._transaction() as tx:
            row = self._row(tx, receipt.version.key)
            if row is None or self._receipt(row) != receipt or row["status"] != "failed_terminal":
                raise OutboxConflict("Only the current terminal failure can be explicitly retried")
            now = self._now(tx)
            new = Receipt(uuid.uuid4().hex, receipt.version)
            tx.execute(
                """UPDATE memory_vectorization_state SET receipt=?, status='pending', attempts=0,
                ticket=?, next_attempt_at=?, updated_at=?, error_category=NULL, error_message=NULL,
                max_attempts=?, base_delay=?, max_delay=? WHERE receipt=?""",
                (
                    new.token,
                    self._ticket(tx),
                    now,
                    now,
                    self.retry_policy.max_attempts,
                    self.retry_policy.base_delay,
                    self.retry_policy.max_delay,
                    receipt.token,
                ),
            )
            self._admit_receipt(tx, new.token, now)
            self._notify(tx)
            return new

    def renew(self, claim: WorkClaim, *, lease_seconds: float = 300.0) -> bool:
        """Renew a live lease; does not resurrect an already expired claim."""
        if not math.isfinite(lease_seconds) or not 0 < lease_seconds <= 86400:
            raise ValueError("lease_seconds must be finite and in (0, 86400]")
        with self._transaction() as tx:
            now = self._now(tx)
            if self._claim_check(tx, claim, now) != "current":
                return False
            tx.execute(
                "UPDATE memory_vectorization_claim SET expires_at=? WHERE token=?", (now + lease_seconds, claim.token)
            )
            return True

    def receipt_outcome(self, token: str) -> str:
        with self.storage.read_snapshot() as tx:
            row = tx.execute("SELECT status FROM memory_vectorization_state WHERE receipt=?", (token,)).fetchone()
            if row is not None:
                return row[0]
            row = tx.execute("SELECT outcome FROM memory_vectorization_receipts WHERE token=?", (token,)).fetchone()
            # Bounded storage cannot prove whether an arbitrary unknown token was
            # once issued. Never invent completion for pruned/unrecognized tokens.
            return row[0] if row is not None else "detail_expired_or_unknown"

    def snapshot(self) -> Dict[str, object]:
        """Exact conservation snapshot; O(current metadata rows), no payload scan.

        This diagnostic aggregate is not part of the indexed claim/wait hot path.
        """
        with self._transaction() as tx:
            now = self._now(tx)
            control = self._control(tx)
            result = {
                row["name"]: row["value"] for row in tx.execute("SELECT name, value FROM memory_vectorization_counters")
            }
            aggregate = tx.execute(
                """SELECT count(*) AS current_rows,
                COALESCE(sum(status='failed_terminal'), 0) AS blocked_current,
                COALESCE(sum(status='in_flight'), 0) AS in_flight_current,
                COALESCE(sum(status IN ('pending', 'backoff') AND library_incarnation IS NOT ?), 0) AS suspended,
                COALESCE(sum(status IN ('pending', 'backoff') AND library_incarnation=? AND next_attempt_at<=?), 0) AS pending_ready,
                COALESCE(sum(status IN ('pending', 'backoff') AND library_incarnation=? AND next_attempt_at>?), 0) AS pending_backoff,
                min(CASE WHEN status IN ('pending', 'backoff') AND library_incarnation=? AND next_attempt_at<=?
                    THEN first_dirty_at END) AS oldest_eligible_at
                FROM memory_vectorization_state""",
                (
                    control["active_library"],
                    control["active_library"],
                    now,
                    control["active_library"],
                    now,
                    control["active_library"],
                    now,
                ),
            ).fetchone()
            result.update(dict(aggregate))
            slot = tx.execute("SELECT disposition FROM memory_vectorization_claim").fetchone()
            result["occupied_claim_slots"] = int(slot is not None)
            result["attempts_in_flight"] = int(slot is not None and slot[0] == "active")
            result["receipt_details"] = tx.execute("SELECT count(*) FROM memory_vectorization_receipts").fetchone()[0]
            result["activation_generation"] = control["activation_generation"]
            result["active_library"] = control["active_library"]
            result["oldest_eligible_age"] = (
                None if result["oldest_eligible_at"] is None else now - result["oldest_eligible_at"]
            )
            result["desired_accounted"] = sum(
                result[name]
                for name in (
                    "completed_fresh",
                    "completed_reused",
                    "superseded",
                    "cancelled_deleted",
                    "failed_terminal",
                    "pending_ready",
                    "pending_backoff",
                    "in_flight_current",
                    "suspended",
                )
            )
            result["attempts_accounted"] = sum(
                result[name]
                for name in (
                    "attempts_completed",
                    "attempts_failed",
                    "attempts_superseded",
                    "attempts_cancelled",
                    "attempts_abandoned",
                    "attempts_in_flight",
                )
            )
            return result
