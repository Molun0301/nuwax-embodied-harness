"""Service-owned, durable source/input bridge.

Business revisions protect source mutations. Outbox versions protect embedding
inputs. The two deliberately advance independently. Inference and artifact I/O
are never performed by repository transaction hooks.
"""
from __future__ import annotations

import json
import threading
import os
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Optional

from contracts.memory import TaggedLocation, SemanticMemoryEntry
from contracts.spatial_memory import SemanticInstance, ObservationEvent, SceneSpace
from services.memory.vectorization_outbox import VectorizationOutbox, WorkKey, OutboxConflict, SourceVersion, Receipt

SOURCES = {
    'location': ('tagged_locations', 'location_id', TaggedLocation),
    'memory': ('semantic_memories', 'memory_id', SemanticMemoryEntry),
    'instance': ('semantic_instances', 'instance_id', SemanticInstance),
    'event': ('observation_events', 'event_id', ObservationEvent),
    'scene': ('scene_spaces', 'scene_space_id', SceneSpace),
}
TOKEN = '_memory_business_token'
INDEX_META = '_memory_index'


class DerivedCacheCapabilityError(RuntimeError):
    """A legacy cache cannot promise transactional original-row preservation."""



def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(',', ':'), default=str)



@dataclass(frozen=True)
class PreparedSource:
    kind: str
    point_id: str
    library: str
    record: Any
    expected: Any
    projection: dict
    dependencies: tuple
    activation_generation: int
    active_library: Optional[str]


class DurableMemoryIndex:
    """One source owner; derived stores may be shared or separately injected."""

    def __init__(self, service):
        self.service = service
        self.repository = service._repository
        self.storage = self.repository.storage
        self.wake = threading.Event()
        self.outbox = VectorizationOutbox(self.storage, wake=self.wake.set)
        self.closed = False
        self.active_library = None
        self.activation_generation = 0
        self.load_history = True
        self.process_binding = uuid.uuid4().hex
        self._worker_file = None
        self._init_schema()

    def _init_schema(self):
        with self.storage.transaction() as tx:
            tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_aliases (
                alias_kind TEXT NOT NULL, namespace TEXT NOT NULL, external_key TEXT NOT NULL,
                internal_key TEXT NOT NULL UNIQUE,
                PRIMARY KEY(alias_kind,namespace,external_key))''')
            tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_sources (
                source_kind TEXT NOT NULL, external_id TEXT NOT NULL,
                library_name TEXT NOT NULL, library_incarnation TEXT NOT NULL,
                internal_id TEXT NOT NULL UNIQUE, record_incarnation TEXT NOT NULL,
                business_revision INTEGER NOT NULL, source_digest TEXT NOT NULL,
                projection_json TEXT NOT NULL, fingerprint TEXT NOT NULL,
                PRIMARY KEY(source_kind,external_id))''')
            tx.execute('''CREATE INDEX IF NOT EXISTS memory_index_sources_library
                ON memory_index_sources(library_name,source_kind,external_id)''')
            tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_dependencies (
                source_kind TEXT NOT NULL, external_id TEXT NOT NULL,
                parent_kind TEXT NOT NULL, parent_id TEXT NOT NULL, value_digest TEXT NOT NULL,
                PRIMARY KEY(source_kind,external_id,parent_kind,parent_id))''')
            tx.execute('''CREATE INDEX IF NOT EXISTS memory_index_dependencies_parent
                ON memory_index_dependencies(parent_kind,parent_id)''')
            tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_publications (
                internal_id TEXT NOT NULL, vector_name TEXT NOT NULL, publication_id TEXT NOT NULL,
                valid_generation INTEGER NOT NULL DEFAULT 0, fingerprint TEXT NOT NULL DEFAULT '',
                provenance_json TEXT NOT NULL DEFAULT '{}',
                PRIMARY KEY(internal_id,vector_name))''')
            existing = {row[1] for row in tx.execute('PRAGMA table_info(memory_index_publications)')}
            for name, declaration in (('valid_generation', 'INTEGER NOT NULL DEFAULT 0'), ('fingerprint', "TEXT NOT NULL DEFAULT ''"), ('provenance_json', "TEXT NOT NULL DEFAULT '{}'"),
                                      ('vector_revision', 'TEXT'), ('route', 'TEXT'),
                                      ('input_versions_json', "TEXT NOT NULL DEFAULT '{}'"),
                                      ('binding_json', "TEXT NOT NULL DEFAULT '{}'")):
                if name not in existing:
                    tx.execute('ALTER TABLE memory_index_publications ADD COLUMN ' + name + ' ' + declaration)
            tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_external_legacy_archive (
                point_id TEXT NOT NULL, vector_name TEXT NOT NULL, payload_json TEXT NOT NULL,
                model_name TEXT, dimension INTEGER, dtype TEXT, shape_json TEXT, vector_payload BLOB,
                PRIMARY KEY(point_id,vector_name))''')
            tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_cache_cleanup (
                point_id TEXT NOT NULL, record_incarnation TEXT NOT NULL, error_category TEXT,
                PRIMARY KEY(point_id,record_incarnation))''')
            tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_reconciliation (
                library_name TEXT NOT NULL, source_kind TEXT NOT NULL, cursor TEXT NOT NULL,
                completed INTEGER NOT NULL DEFAULT 0,
                PRIMARY KEY(library_name,source_kind))''')

    @staticmethod
    def kind_of(record):
        return next((kind for kind, (_, _, model) in SOURCES.items() if isinstance(record, model)), None)

    def _alias(self, tx, kind, namespace, external):
        row = tx.execute('SELECT internal_key FROM memory_index_aliases WHERE alias_kind=? AND namespace=? AND external_key=?',
                         (kind, namespace, external)).fetchone()
        if row:
            return row[0]
        value = kind + '_' + uuid.uuid4().hex
        tx.execute('INSERT INTO memory_index_aliases VALUES (?,?,?,?)', (kind, namespace, external, value))
        return value

    def library_identity(self, name):
        with self.storage.transaction() as tx:
            alias = self._alias(tx, 'lib', '', name)
            return self.outbox.create_library(alias)

    def activate(self, name, *, load_history=True):
        identity = self.library_identity(name) if name is not None else None
        self.activation_generation = self.outbox.activate(identity)
        self.active_library = name
        self.load_history = load_history
        self.wake.set()
        return identity

    def _state(self, tx, kind, point_id):
        return tx.execute('SELECT * FROM memory_index_sources WHERE source_kind=? AND external_id=?',
                          (kind, point_id)).fetchone()

    @staticmethod
    def key(row):
        return WorkKey(row['library_incarnation'], row['source_kind'], row['internal_id'])

    @staticmethod
    def business_token(row):
        return ('business', row['record_incarnation'], row['business_revision']) if row else None

    @staticmethod
    def raw_token(row):
        return ('raw', (row['payload_json'], row['map_name'], row['library_name']) if row else None)

    def _raw(self, tx, kind, point_id):
        table, column, _ = SOURCES[kind]
        return tx.execute(f'SELECT payload_json, map_name, library_name FROM {table} WHERE {column}=?', (point_id,)).fetchone()

    def load(self, kind, point_id, *, tx=None):
        if tx is None:
            with self.storage.read_snapshot() as snapshot:
                raw = self._raw(snapshot, kind, point_id)
                state = self._state(snapshot, kind, point_id)
        else:
            raw = self._raw(tx, kind, point_id)
            state = self._state(tx, kind, point_id)
        if raw is None:
            return None
        record = self.repository._load_record(SOURCES[kind][2], raw)
        token = self.business_token(state) if state else self.raw_token(raw)
        object.__setattr__(record, TOKEN, token)
        return record

    def _dependency(self, kind, record):
        if kind == 'memory' and record.linked_location_id:
            return ('location', record.linked_location_id)
        if kind == 'location' and record.memory_id:
            return ('memory', record.memory_id)
        return None

    def _dependency_value(self, parent_kind, record, library):
        if record is None or self.repository._resolve_record_library_name(record) != library:
            return None
        return record.name if parent_kind == 'location' else list(record.artifact_ids)

    def prepare(self, kind, record, *, overrides=None):
        """Snapshot dependencies, release reader, then render pure input projection."""
        activation_generation, active_library = self.activation_generation, self.active_library
        point_id = getattr(record, SOURCES[kind][1])
        library = self.repository._resolve_record_library_name(record)
        dep = self._dependency(kind, record)
        with self.storage.read_snapshot() as tx:
            activation_generation = tx.execute('SELECT activation_generation FROM memory_vectorization_control WHERE singleton=1').fetchone()[0]
            scope = self.service._observation_write_scope()
            if scope and 'activation_generation' in scope:
                activation_generation = scope['activation_generation']
            state = self._state(tx, kind, point_id)
            raw = self._raw(tx, kind, point_id)
            expected = getattr(record, TOKEN, None)
            if expected is None:
                expected = self.business_token(state) if state else self.raw_token(raw)
            parent = None
            if dep:
                parent = overrides.get(dep) if overrides and dep in overrides else self.load(*dep, tx=tx)
        value = self._dependency_value(dep[0], parent, library) if dep else None
        projection = self.service._prepare_index_projection(kind, record, dependency_value=value)
        dependencies = ((dep[0], dep[1], value),) if dep else ()
        return PreparedSource(kind, point_id, library, record, expected,
                              projection, dependencies, activation_generation, active_library)

    def _validate(self, tx, prepared, *, validate_dependencies=True):
        if prepared.activation_generation != tx.execute('SELECT activation_generation FROM memory_vectorization_control WHERE singleton=1').fetchone()[0]:
            raise OutboxConflict('Library activation changed during source preparation')
        state = self._state(tx, prepared.kind, prepared.point_id)
        raw = self._raw(tx, prepared.kind, prepared.point_id)
        expected = prepared.expected
        if expected and expected[0] == 'business':
            valid = self.business_token(state) == expected
        elif expected and expected[0] == 'raw':
            valid = state is None and self.raw_token(raw) == expected
        else:
            valid = False
        if not valid:
            raise OutboxConflict('Business source changed; reload before applying the mutation')
        if state and state['library_name'] != prepared.library:
            raise OutboxConflict('Source identity belongs to another library')
        if validate_dependencies:
            for parent_kind, parent_id, expected_value in prepared.dependencies:
                parent = self.load(parent_kind, parent_id, tx=tx)
                if self._dependency_value(parent_kind, parent, prepared.library) != expected_value:
                    raise OutboxConflict('Embedding dependency changed during preparation')
        return state

    def _dependents(self, kind, point_id, new_record):
        with self.storage.read_snapshot() as tx:
            rows = tx.execute('SELECT source_kind,external_id FROM memory_index_dependencies WHERE parent_kind=? AND parent_id=?',
                              (kind, point_id)).fetchall()
            records = [(row[0], self.load(row[0], row[1], tx=tx)) for row in rows]
        return [self.prepare(k, record, overrides={(kind, point_id): new_record})
                for k, record in records if record is not None]

    @contextmanager
    def mutation(self, kind, record, *, origin=None, prepared=None):
        if self.closed:
            raise RuntimeError('Memory service is closing')
        if prepared is None:
            prepared = self.prepare(kind, record)
            dependents = self._dependents(kind, prepared.point_id, record)
        else:
            prepared, dependents = prepared
        with self.storage.transaction() as tx:
            if self.closed:
                raise RuntimeError('Memory service is closing')
            old = self._validate(tx, prepared)
            for dependent in dependents:
                self._validate(tx, dependent, validate_dependencies=False)
            yield tx
            token = self._accept(tx, prepared, old=old, source_mutated=True)
            for dependent in dependents:
                self._validate(tx, dependent)
                self._accept(tx, dependent, old=self._state(tx, dependent.kind, dependent.point_id), source_mutated=False)
            def publish_source():
                object.__setattr__(record, TOKEN, token)
                if origin is not None:
                    object.__setattr__(origin, TOKEN, token)
                    object.__setattr__(origin, '_memory_live_only', False)
                    object.__setattr__(origin, '_memory_source_deleted', False)
                object.__setattr__(record, '_memory_live_only', False)
                object.__setattr__(record, '_memory_source_deleted', False)
                self.service._publish_index_record(kind, record, token)
            tx.after_commit(publish_source)
            tx.after_commit(self.wake.set)

    @staticmethod
    def input_projection(projection):
        return {key: projection[key] for key in ('text', 'artifacts', 'binding', 'eligible')}

    @classmethod
    def same_declared_intent(cls, old, new):
        """A new process cannot renew an unfinished requirement's retry budget.

        Runtime object identity remains a publication fence. It is not evidence
        that the user's requested model/input changed across process recovery.
        """
        old_input, new_input = cls.input_projection(old), cls.input_projection(new)
        old_input['binding'] = old.get('intent_binding', old_input['binding'])
        new_input['binding'] = new.get('intent_binding', new_input['binding'])
        for role in ('text', 'image'):
            before, after = old_input['binding'][role], new_input['binding'][role]
            if any(before.get(key) != after.get(key) for key in ('configured', 'dimension')):
                return False
            # Lazy initialization resolves a runtime provider, not new requested
            # work. Compare actual declarations only when both sides resolved it.
            before_available = before.get('available', any(value is not None for value in before['declarations'].values()))
            after_available = after.get('available', any(value is not None for value in after['declarations'].values()))
            if before_available and after_available and before['declarations'] != after['declarations']:
                return False
        old_input.pop('binding')
        new_input.pop('binding')
        return old_input == new_input

    @staticmethod
    def _role_declarations_equal(before, after):
        return all(before.get(key) == after.get(key) for key in ('configured', 'dimension', 'declarations'))

    @classmethod
    def bindings_compatible(cls, before, after, roles=('text', 'image')):
        if not isinstance(before, dict) or not isinstance(after, dict):
            return False
        for role in roles:
            left, right = before.get(role), after.get(role)
            if not isinstance(left, dict) or not isinstance(right, dict):
                return False
            if before.get('process') == after.get('process'):
                if left != right:
                    return False
            elif not cls._role_declarations_equal(left, right):
                return False
        return True

    @classmethod
    def _provider_generation_same(cls, old, new, role):
        before, after = old['binding'][role], new['binding'][role]
        if old['binding'].get('process') == new['binding'].get('process') and before.get('initialized', True) and after.get('initialized', True):
            return before == after
        # Compare the last resolved intent, retained across lazy restarts.
        # An unresolved placeholder must not erase unavailable -> available work.
        before = old.get('intent_binding', old['binding'])[role]
        after = new.get('intent_binding', new['binding'])[role]
        if (before.get('initialized', True) and after.get('initialized', True)
                and bool(before.get('available', any(value is not None for value in before.get('declarations', {}).values())))
                != bool(after.get('available', any(value is not None for value in after.get('declarations', {}).values())))):
            return False
        if any(before.get(key) != after.get(key) for key in ('configured', 'dimension')):
            return False
        if before.get('available') and after.get('available'):
            return before.get('declarations') == after.get('declarations')
        return True

    @classmethod
    def _next_input_versions(cls, old, new):
        previous = old.get('input_versions', {}) if old else {}
        values = {}
        for name, field in (('text', 'text'), ('artifacts', 'artifacts'), ('eligible', 'eligible')):
            values[name] = int(previous.get(name, 1)) + int(old is not None and old[field] != new[field])
        for role in ('text', 'image'):
            name = role + '_provider'
            values[name] = int(previous.get(name, 1)) + int(old is not None and not cls._provider_generation_same(old, new, role))
        return values

    @staticmethod
    def route_versions(projection, route):
        names = ('artifacts', 'image_provider', 'eligible') if route == 'image_only' else ('text', 'text_provider', 'eligible')
        if route == 'fused_text_image':
            names += ('artifacts',)
        versions = projection.get('input_versions', {})
        return {name: versions.get(name, 1) for name in names}

    @classmethod
    def route_inputs_equal(cls, before, after, route):
        fields = ('artifacts', 'eligible') if route == 'image_only' else ('text', 'eligible')
        if route == 'fused_text_image':
            fields += ('artifacts',)
        return all(before[key] == after[key] for key in fields) and cls.route_binding_status(
            before['binding'], after['binding'], route) != 'incompatible'

    @classmethod
    def route_binding_status(cls, before, after, route):
        if route not in ('text_only', 'text_only_fallback', 'fused_text_image', 'image_only'):
            return 'incompatible'
        role = 'image' if route == 'image_only' else 'text'
        if not cls.bindings_compatible(before, after, (role,)):
            return 'incompatible'
        if before.get('process') == after.get('process'):
            return 'current'
        declarations = before[role].get('declarations', {})
        # 跨进程的同名模型不是持久版本证明；没有明确版本时保留旧证据标记。
        versioned = any(declarations.get(key) not in (None, '', 'undeclared')
                        for key in ('model_revision', 'revision', 'version', 'cache_revision'))
        return 'current' if versioned else 'legacy'

    def _accept(self, tx, prepared, *, old, source_mutated):
        identity = self.library_identity(prepared.library)
        internal_id = old['internal_id'] if old else self._alias(tx, 'rec', identity.incarnation + ':' + prepared.kind, prepared.point_id)
        key = WorkKey(identity.incarnation, prepared.kind, internal_id)
        current = self.outbox.current(key)
        old_projection = json.loads(old['projection_json']) if old else None
        projection = dict(prepared.projection)
        # Keep the last resolved declaration across a lazy restart placeholder.
        # Otherwise a later genuinely revised provider could borrow the old
        # terminal budget because the placeholder erased the comparison proof.
        intent = dict(projection['binding'])
        if old_projection is not None:
            prior_intent = old_projection.get('intent_binding', old_projection['binding'])
            for role in ('text', 'image'):
                current_role, previous_role = intent[role], prior_intent[role]
                if (not current_role.get('initialized', True)
                        and all(current_role.get(key) == previous_role.get(key) for key in ('configured', 'dimension'))):
                    intent[role] = previous_role
        projection['intent_binding'] = intent
        projection['input_versions'] = self._next_input_versions(old_projection, projection)
        projection['format_version'] = 2
        unchanged = old_projection is not None and all(
            projection['input_versions'][name] == old_projection.get('input_versions', {}).get(name, 1)
            for name in projection['input_versions'])
        if current is None or old is None or not unchanged:
            receipt = self.outbox.accept(key, expected=current.version if current else None,
                                         write_source=lambda tx, version: None)
        else:
            receipt = current
        self._retarget_compatible(tx, internal_id, receipt, projection)
        revision = (old['business_revision'] + int(source_mutated)) if old else 1
        tx.execute('''INSERT INTO memory_index_sources VALUES (?,?,?,?,?,?,?,?,?,?)
            ON CONFLICT(source_kind,external_id) DO UPDATE SET
            library_name=excluded.library_name,library_incarnation=excluded.library_incarnation,
            internal_id=excluded.internal_id,record_incarnation=excluded.record_incarnation,
            business_revision=excluded.business_revision,source_digest=excluded.source_digest,
            projection_json=excluded.projection_json,fingerprint=excluded.fingerprint''',
            (prepared.kind, prepared.point_id, prepared.library, identity.incarnation, internal_id,
             receipt.version.record_incarnation, revision, '',
             canonical(projection), ''))
        tx.execute('DELETE FROM memory_index_dependencies WHERE source_kind=? AND external_id=?', (prepared.kind, prepared.point_id))
        tx.executemany('INSERT INTO memory_index_dependencies VALUES (?,?,?,?,?)',
                       [(prepared.kind, prepared.point_id, dep[0], dep[1], '') for dep in prepared.dependencies])
        self._refresh_shared_payload(tx, prepared, revision)
        return ('business', receipt.version.record_incarnation, revision)

    @contextmanager
    def deletion(self, kind, ids, *, expected_tokens=None):
        dependents = [prepared for point_id in ids for prepared in self._dependents(kind, point_id, None)]
        with self.storage.transaction() as tx:
            if self.closed:
                raise RuntimeError('Memory service is closing')
            for dependent in dependents:
                self._validate(tx, dependent, validate_dependencies=False)
            for point_id in ids:
                row = self._state(tx, kind, point_id)
                if expected_tokens is not None and point_id in expected_tokens:
                    token = expected_tokens[point_id]
                    if row is not None and self.business_token(row) != token:
                        raise OutboxConflict('Source changed before conditional deletion')
                if row:
                    current = self.outbox.current(self.key(row))
                    if current:
                        self.outbox.delete(self.key(row), expected=current.version, delete_source=lambda tx, version: None)
                    tx.execute('DELETE FROM memory_index_sources WHERE source_kind=? AND external_id=?', (kind, point_id))
                    tx.execute('DELETE FROM memory_index_aliases WHERE internal_key=?', (row['internal_id'],))
                if row:
                    tx.execute('DELETE FROM memory_index_publications WHERE internal_id=?', (row['internal_id'],))
                tx.execute('DELETE FROM memory_index_dependencies WHERE source_kind=? AND external_id=?', (kind, point_id))
                self._delete_derived(tx, point_id, row['record_incarnation'] if row else '')
                tx.execute('DELETE FROM memory_index_external_legacy_archive WHERE point_id=?', (point_id,))
                tx.after_commit(lambda point_id=point_id: self.service._remove_index_record(kind, point_id))
            yield tx
            for prepared in dependents:
                # A dependent deleted in the same batch no longer needs repair.
                if self._raw(tx, prepared.kind, prepared.point_id):
                    self._accept(tx, prepared, old=self._state(tx, prepared.kind, prepared.point_id), source_mutated=False)
            tx.after_commit(self.wake.set)

    def _delete_derived(self, tx, point_id, incarnation):
        store = self.service._vector_store
        if getattr(store, 'storage', None) is self.storage:
            store.delete_points([point_id])
            if tx.execute("SELECT 1 FROM sqlite_master WHERE name='memory_index_legacy_archive'").fetchone():
                tx.execute('DELETE FROM memory_index_legacy_archive WHERE point_id=?', (point_id,))
        else:
            tx.execute('INSERT OR IGNORE INTO memory_index_cache_cleanup VALUES (?,?,NULL)', (point_id, incarnation))
            tx.after_commit(self.cleanup_cache_page)

    def ensure(self, kind, record):
        """Adopt/reconcile a source without rewriting it or advancing business CAS."""
        record = self.load(kind, getattr(record, SOURCES[kind][1]))
        if record is None:
            return
        prepared = self.prepare(kind, record)
        with self.storage.transaction() as tx:
            if self.closed:
                raise RuntimeError('Memory service is closing')
            old = self._validate(tx, prepared)
            token = self._accept(tx, prepared, old=old, source_mutated=False)
            tx.after_commit(lambda: object.__setattr__(record, TOKEN, token))
        self.wake.set()

    def source_for_claim(self, claim):
        with self.storage.read_snapshot() as tx:
            row = tx.execute('SELECT * FROM memory_index_sources WHERE internal_id=? AND library_incarnation=?',
                             (claim.receipt.version.key.point_id, claim.receipt.version.key.library_incarnation)).fetchone()
            if row is None or self._raw(tx, row['source_kind'], row['external_id']) is None:
                return None
            current = self.outbox.current(self.key(row))
            if current is None or current.version != claim.receipt.version:
                return None
            return dict(row), json.loads(row['projection_json'])

    def reconcile_page(self, *, page_size=16):
        """A fixed old-source quota runs alongside live claims, never only at idle."""
        library = self.active_library
        if not library or not self.load_history or self.closed:
            return 0
        for kind, (table, column, _) in SOURCES.items():
            with self.storage.read_snapshot() as tx:
                progress = tx.execute('SELECT cursor,completed FROM memory_index_reconciliation WHERE library_name=? AND source_kind=?',
                                      (library, kind)).fetchone()
                if progress and progress['completed']:
                    continue
                cursor = progress['cursor'] if progress else ''
                rows = tx.execute(f'SELECT {column} FROM {table} WHERE library_name=? AND {column}>? ORDER BY {column} LIMIT ?',
                                  (library, cursor, page_size)).fetchall()
            processed = 0
            for row in rows:
                if self.active_library != library or not self.load_history:
                    return processed
                record = self.load(kind, row[0])
                if record:
                    try:
                        self.ensure(kind, record)
                    except OutboxConflict:
                        # A current writer already admits its raced source. Do
                        # not let that hot key starve untouched legacy successors.
                        with self.storage.read_snapshot() as tx:
                            state = self._state(tx, kind, row[0])
                            if state is None and self._raw(tx, kind, row[0]) is not None:
                                return processed
                cursor = row[0]
                processed += 1
            with self.storage.transaction() as tx:
                tx.execute('''INSERT INTO memory_index_reconciliation VALUES (?,?,?,?)
                    ON CONFLICT(library_name,source_kind) DO UPDATE SET cursor=excluded.cursor,completed=excluded.completed''',
                    (library, kind, cursor, int(len(rows) < page_size)))
            return processed
        return 0

    def reset_reconciliation(self, library):
        with self.storage.transaction() as tx:
            tx.execute("DELETE FROM memory_index_reconciliation WHERE library_name=? AND source_kind='__rebind__'", (library,))
        self.wake.set()

    def rebind_page(self, page_size=4):
        """Refresh accepted model/cache requirements without resetting legacy progress."""
        library = self.active_library
        if not library or not self.load_history or self.closed:
            return 0
        with self.storage.read_snapshot() as tx:
            progress = tx.execute("SELECT cursor,completed FROM memory_index_reconciliation WHERE library_name=? AND source_kind='__rebind__'", (library,)).fetchone()
            if progress and progress['completed']:
                return 0
            cursor = progress['cursor'] if progress else ''
            rows = tx.execute('SELECT source_kind,external_id,internal_id FROM memory_index_sources WHERE library_name=? AND internal_id>? ORDER BY internal_id LIMIT ?',
                              (library, cursor, page_size)).fetchall()
        for kind, point_id, alias in rows:
            record = self.load(kind, point_id)
            if record:
                try:
                    self.ensure(kind, record)
                except OutboxConflict:
                    pass  # Current writes already own durable admission.
            cursor = alias
        with self.storage.transaction() as tx:
            tx.execute("INSERT INTO memory_index_reconciliation VALUES (?,'__rebind__',?,?) ON CONFLICT(library_name,source_kind) DO UPDATE SET cursor=excluded.cursor,completed=excluded.completed",
                       (library, cursor, int(len(rows) < page_size)))
        return len(rows)

    @staticmethod
    def version_provenance(claim):
        version = claim.receipt.version
        return {'library_incarnation': version.key.library_incarnation, 'source_kind': version.key.source_kind,
                'internal_id': version.key.point_id, 'record_incarnation': version.record_incarnation,
                'input_generation': version.revision, 'proof_version': 2}

    def _archive(self, tx, point_id):
        """Copy original SQLite row representations once, before first replacement."""
        tx.execute('''CREATE TABLE IF NOT EXISTS memory_index_legacy_archive (
            point_id TEXT NOT NULL, vector_name TEXT NOT NULL, model_name TEXT,
            dimension INTEGER, vector_payload BLOB, vector_updated_at TEXT,
            point_payload_json TEXT NOT NULL, point_updated_at TEXT,
            point_row_json TEXT, vector_row_json TEXT,
            PRIMARY KEY(point_id,vector_name))''')
        columns = {r[1] for r in tx.execute('PRAGMA table_info(memory_index_legacy_archive)')}
        for column in ('point_row_json', 'vector_row_json'):
            if column not in columns:
                tx.execute('ALTER TABLE memory_index_legacy_archive ADD COLUMN ' + column + ' TEXT')
        row = tx.execute('SELECT * FROM memory_vector_points WHERE point_id=?', (point_id,)).fetchone()
        if row is None:
            return
        payload = json.loads(row['payload_json'])
        proven = payload.get(INDEX_META, {}).get('vectors', {})
        vectors = tx.execute('SELECT * FROM memory_named_vectors WHERE point_id=?', (point_id,)).fetchall()
        for vector in vectors:
            if vector['vector_name'] not in proven:
                tx.execute('INSERT OR IGNORE INTO memory_index_legacy_archive VALUES (?,?,?,?,?,?,?,?,?,?)',
                           (point_id, vector['vector_name'], vector['model_name'], vector['dimension'],
                            vector['vector_payload'], vector['updated_at'], row['payload_json'], row['updated_at'],
                            canonical(dict(row)), canonical({key: vector[key] for key in vector.keys() if key != 'vector_payload'})))
        if not vectors and INDEX_META not in payload:
            tx.execute('INSERT OR IGNORE INTO memory_index_legacy_archive VALUES (?,?,?,?,?,?,?,?,?,?)',
                       (point_id, '', None, None, None, None, row['payload_json'], row['updated_at'], canonical(dict(row)), None))

    def _write_modality(self, store, tx, row, projection, claim, named, provenance):
        from services.memory.vector_store import _encode_vector
        # SQLite publication stays SQL-only and preserves other named modalities.
        point_id = row['external_id']
        if tx is not None:
            self._archive(tx, point_id)
            stored = tx.execute('SELECT payload_json FROM memory_vector_points WHERE point_id=?', (point_id,)).fetchone()
            old_payload = json.loads(stored[0]) if stored else {}
        else:
            existing = store.get_point(point_id)
            old_payload = existing.payload if existing else {}
        metadata = dict(old_payload.get(INDEX_META, {}))
        old_vectors = dict(metadata.get('vectors', {}))
        legacy_inputs = dict(metadata.get('legacy_inputs', {}))
        if tx is not None:
            old_names = [r[0] for r in tx.execute('SELECT vector_name FROM memory_named_vectors WHERE point_id=?', (point_id,))]
        else:
            old_names = list(existing.vectors) if existing else []
        for old_name in old_names:
            if old_name not in old_vectors and old_name not in legacy_inputs:
                legacy_inputs[old_name] = {key: old_payload[key] for key in ('retrieval_text', 'artifact_ids', 'memory_library_name', 'map_name') if key in old_payload}
                legacy_inputs[old_name]['record_kind'] = old_payload.get('record_kind')
        metadata['legacy_inputs'] = legacy_inputs
        old = old_vectors.get(named.vector_name)
        if old and old.get('library_incarnation') == provenance['library_incarnation'] and old.get('record_incarnation') == provenance['record_incarnation'] and int(old.get('input_generation', 0)) > provenance['input_generation']:
            raise OutboxConflict('Refusing older derived-cache publication')
        old_vectors[named.vector_name] = provenance
        metadata.update({'source_kind': row['source_kind'], 'library_name': row['library_name'],
                         'vectors': old_vectors, 'business_revision': row['business_revision']})
        payload = dict(projection['payload'])
        payload[INDEX_META] = metadata
        if tx is None:
            has_legacy = existing is not None and (INDEX_META not in existing.payload or any(
                name not in existing.payload.get(INDEX_META, {}).get('vectors', {}) for name in existing.vectors))
            archive_upsert = getattr(store, 'archive_and_upsert_point', None)
            if has_legacy and not callable(archive_upsert):
                raise DerivedCacheCapabilityError('legacy_archive_capability_required')
            vectors = dict(existing.vectors) if existing else {}
            vectors[named.vector_name] = named
            writer = archive_upsert if has_legacy else store.upsert_point
            writer(point_id=point_id, collection_name=projection['collection'],
                   record_kind=projection['record_kind'], payload=payload, vectors=vectors)
            return
        # Upsert the payload columns without full-replacement vector semantics.
        columns = ('point_id','collection_name','record_kind','map_name','localization_session_id','linked_location_id',
                   'camera_id','topo_node_id','semantic_region_id','anchor_id','instance_type','movability',
                   'last_seen_ts','timestamp_ts','payload_json','updated_at')
        stamp = str(payload.get('timestamp') or payload.get('last_seen_ts') or '')
        values = (point_id, projection['collection'], projection['record_kind'],
                  *[payload.get(key) for key in columns[3:12]], str(payload.get('last_seen_ts') or ''),
                  stamp, canonical(payload), stamp)
        tx.execute('INSERT INTO memory_vector_points (' + ','.join(columns) + ') VALUES (' + ','.join('?' for _ in columns) +
                   ') ON CONFLICT(point_id) DO UPDATE SET ' + ','.join(f'{key}=excluded.{key}' for key in columns[1:]), values)
        tx.execute('''INSERT INTO memory_named_vectors(point_id,vector_name,model_name,dimension,vector_payload,updated_at,cache_token)
            VALUES (?,?,?,?,?,?,?) ON CONFLICT(point_id,vector_name) DO UPDATE SET
            model_name=excluded.model_name,dimension=excluded.dimension,vector_payload=excluded.vector_payload,
            updated_at=excluded.updated_at,cache_token=excluded.cache_token''',
            (point_id, named.vector_name, named.model_name, named.dimension, _encode_vector(named.vector), stamp, provenance['publication_id']))
        # Cache token selected with the vector row is authoritative. No callback correctness dependency.

    def publish(self, claim, named, *, actual):
        import numpy as np
        vector = np.asarray(named.vector)
        if vector.ndim != 1 or vector.size != named.dimension or not np.isfinite(vector).all() or not vector.size:
            raise ValueError('invalid_vector_output')
        from services.memory.vector_store import NamedVector
        publication_id = uuid.uuid4().hex
        owned = np.frombuffer(np.asarray(named.vector, dtype=np.float32).tobytes(), dtype=np.float32)
        named = NamedVector(named.vector_name, named.model_name, named.dimension, owned, publication_id)
        snapshot = self.source_for_claim(claim)
        if snapshot is None:
            return False
        row, projection = snapshot
        route = actual['route']
        versions = self.route_versions(projection, route)
        provenance = {**self.version_provenance(claim), **actual, 'publication_id': publication_id,
                      'vector_revision': publication_id, 'input_versions': versions,
                      'model_name': named.model_name, 'dimension': named.dimension}
        store = self.service._vector_store
        owner = getattr(store, 'storage', None)
        revision_owned = owner is not None
        def attest(tx, guarded_claim):
            tx.execute('''INSERT INTO memory_index_publications
                (internal_id,vector_name,publication_id,valid_generation,fingerprint,provenance_json,vector_revision,route,input_versions_json,binding_json)
                VALUES (?,?,?,?,?,?,?,?,?,?)
                ON CONFLICT(internal_id,vector_name) DO UPDATE SET publication_id=excluded.publication_id,
                valid_generation=excluded.valid_generation,fingerprint='',provenance_json=excluded.provenance_json,
                vector_revision=excluded.vector_revision,route=excluded.route,input_versions_json=excluded.input_versions_json,binding_json=excluded.binding_json''',
                (claim.receipt.version.key.point_id, named.vector_name, publication_id, claim.receipt.version.revision,
                 '', canonical(provenance), publication_id if revision_owned else None, route, canonical(versions), canonical(provenance['binding'])))
        if owner is self.storage:
            def write(tx, guarded_claim):
                row = self._state(tx, claim.receipt.version.key.source_kind,
                                  self.source_for_claim(guarded_claim)[0]['external_id'])
                projection = json.loads(row['projection_json'])
                self._write_modality(store, tx, row, projection, guarded_claim, named, provenance)
                attest(tx, guarded_claim)
            return self.outbox.publish_partial(claim, publish=write).published
        # Two-phase adapter: cache commit first, authoritative CAS second. Reads
        # validate source/input identity even in the interval between these steps.
        snapshot = self.source_for_claim(claim)
        if snapshot is None:
            return False
        row, projection = snapshot
        if owner is not None:
            with owner.transaction() as tx:
                self._write_modality(store, tx, row, projection, claim, named, provenance)
        else:
            self._write_modality(store, None, row, projection, claim, named, provenance)
        published = self.outbox.publish_partial(claim, publish=attest).published
        if not published and self.source_for_claim(claim) is None:
            with self.storage.transaction() as tx:
                tx.execute('INSERT OR IGNORE INTO memory_index_cache_cleanup VALUES (?,?,NULL)',
                           (row['external_id'], claim.receipt.version.record_incarnation))
                tx.after_commit(self.cleanup_cache_page)
        return published

    @staticmethod
    def _service_point(point_row, vector_rows):
        """Decode each modality independently; corrupt cache bytes cannot hide source text."""
        import numpy as np
        from services.memory.vector_store import VectorPoint, NamedVector, _decode_vector
        invalid_payload = False
        try:
            payload = json.loads(point_row['payload_json'])
            if not isinstance(payload, dict):
                raise ValueError('invalid_point_payload')
        except (ValueError, TypeError):
            payload, invalid_payload = {}, True
        internal = payload.get(INDEX_META, {})
        if not isinstance(internal, dict) or not isinstance(internal.get('vectors', {}), dict) or not isinstance(internal.get('legacy_inputs', {}), dict):
            payload[INDEX_META] = {}
            invalid_payload = True
        if invalid_payload:
            payload['_invalid_cached_payload'] = True
        if payload.get('map_name') is None:
            payload['map_name'] = point_row['map_name']
        vectors = {}
        for row in vector_rows:
            try:
                array = _decode_vector(row['vector_payload'])
            except (ValueError, TypeError, OverflowError):
                array = np.empty(0, dtype=np.float32)
            try:
                dimension = int(row['dimension'])
            except (ValueError, TypeError, OverflowError):
                dimension = 0
            name = str(row['vector_name'])
            vectors[name] = NamedVector(name, str(row['model_name']), dimension, array, row['cache_token'])
        return VectorPoint(point_row['point_id'], point_row['collection_name'], point_row['record_kind'],
                           payload, vectors, point_row['updated_at'])

    @staticmethod
    def _owned_external_point(point):
        if point is None:
            return None
        import copy
        import numpy as np
        from services.memory.vector_store import VectorPoint, NamedVector
        vectors = {}
        for name, named in point.vectors.items():
            owned = np.array(named.vector, copy=True)
            owned.setflags(write=False)
            vectors[name] = NamedVector(name, named.model_name, named.dimension, owned,
                                        getattr(named, 'storage_revision', None))
        return VectorPoint(point.point_id, point.collection_name, point.record_kind,
                           copy.deepcopy(point.payload), vectors, point.updated_at)

    @staticmethod
    def _native_rows(owner, point_ids):
        points, vectors = {}, {}
        with owner.read_snapshot() as tx:
            for offset in range(0, len(point_ids), 250):
                batch = point_ids[offset:offset + 250]
                placeholders = ','.join('?' for _ in batch)
                for row in tx.execute('SELECT point_id,collection_name,record_kind,map_name,payload_json,updated_at FROM memory_vector_points WHERE point_id IN (' + placeholders + ')', batch):
                    points[row['point_id']] = row
                for row in tx.execute('SELECT point_id,vector_name,model_name,dimension,vector_payload,cache_token FROM memory_named_vectors WHERE point_id IN (' + placeholders + ')', batch):
                    vectors.setdefault(row['point_id'], []).append(row)
        return points, vectors

    def current_vectors(self, claim):
        snapshot = self.source_for_claim(claim)
        if snapshot is None:
            return {}
        row, projection = snapshot
        binding = self.service._index_model_binding()
        store = self.service._vector_store
        owner = getattr(store, 'storage', None)
        if owner is not None:
            points, vectors = self._native_rows(owner, [row['external_id']])
            raw = points.get(row['external_id'])
            point = self._service_point(raw, vectors.get(row['external_id'], [])) if raw else None
        else:
            point = self._owned_external_point(store.get_point(row['external_id']))
        if point is None:
            return {}
        metadata = point.payload.get(INDEX_META, {}).get('vectors', {})
        with self.storage.read_snapshot() as tx:
            acknowledged = {row['vector_name']: dict(row) for row in tx.execute('SELECT * FROM memory_index_publications WHERE internal_id=?',
                            (claim.receipt.version.key.point_id,))}
        valid = {}
        for name, named in point.vectors.items():
            proof = metadata.get(name)
            if not isinstance(proof, dict):
                continue
            if (self.route_binding_status(proof.get('binding'), binding, proof.get('route')) == 'current'
                    and self._ack_matches(proof, acknowledged.get(name), claim.receipt, named)):
                valid[name] = named
        return valid

    def scoped_status(self):
        with self.storage.read_snapshot() as tx:
            rows = tx.execute('''SELECT s.status,s.error_category,count(*) AS count
                FROM memory_vectorization_state s JOIN memory_index_sources i ON i.internal_id=s.point_id
                WHERE i.library_name=? GROUP BY s.status,s.error_category''', (self.active_library,)).fetchall()
            return {'work': [dict(row) for row in rows], 'activation_generation': self.activation_generation,
                    'active_library': self.active_library, 'durable': True,
                    'cache_cleanup_pending': tx.execute('SELECT count(*) FROM memory_index_cache_cleanup').fetchone()[0],
                    'reconciliation_pending': tx.execute('SELECT count(*) FROM memory_index_reconciliation WHERE library_name=? AND completed=0',
                                                         (self.active_library,)).fetchone()[0]}

    def query_snapshot(self, visible, *, include_readiness=True):
        """Build points from authoritative source metadata, validating each modality.

        A live-only observation may contribute current lexical metadata. It can
        borrow persisted similarity only if its exact inputs are unchanged.
        The first recall pass may omit per-result readiness objects that it never
        consumes; validation and aggregate readiness counts are always identical.
        """
        from services.memory.vector_store import VectorPoint
        store = self.service._vector_store
        results = []
        counts = {'validated_current': 0, 'legacy_unverified': 0,
                  'known_stale_or_incompatible': 0, 'missing': 0, 'pending': 0, 'failed': 0, 'disabled': 0, 'not_applicable': 0, 'unavailable': 0}
        # Capture bounded SQL rows/tokens together; parsing/models/scoring are
        # outside the owner lock. Common current cache objects need no rehydrate.
        grouped = {}
        for kind, live in visible:
            grouped.setdefault(kind, {})[getattr(live, SOURCES[kind][1])] = live
        source_rows, acknowledgements = {}, {}
        point_ids = list({point_id for group in grouped.values() for point_id in group})
        native_points = native_vectors = None
        with self.storage.read_snapshot() as tx:
            for kind, by_id in grouped.items():
                table, column, _ = SOURCES[kind]
                ids = list(by_id)
                for offset in range(0, len(ids), 250):
                    batch = ids[offset:offset + 250]
                    placeholders = ','.join('?' for _ in batch)
                    rows = tx.execute(f"""SELECT r.{column} AS source_id,r.payload_json AS raw_payload,
                        r.map_name AS source_map_name,r.library_name AS source_library_name,i.*,
                        s.source_revision AS input_revision,s.record_incarnation AS input_incarnation,
                        s.receipt AS desired_receipt,s.status,s.error_category
                        FROM {table} r LEFT JOIN memory_index_sources i ON i.source_kind=? AND i.external_id=r.{column}
                        LEFT JOIN memory_vectorization_state s ON s.point_id=i.internal_id
                        AND s.library_incarnation=i.library_incarnation AND s.source_kind=i.source_kind
                        WHERE r.{column} IN ({placeholders})""", (kind, *batch)).fetchall()
                    source_rows.update(((kind, row['source_id']), row) for row in rows)
            aliases = [row['internal_id'] for row in source_rows.values() if row['internal_id']]
            for offset in range(0, len(aliases), 250):
                batch = aliases[offset:offset + 250]
                # The cache proof is checked against these authoritative fields.
                # Its archived provenance JSON is not consulted by _ack_matches;
                # avoid copying that second full proof while holding the reader.
                rows = tx.execute('SELECT internal_id,vector_name,publication_id,valid_generation,vector_revision,route,input_versions_json,binding_json '
                                  'FROM memory_index_publications WHERE internal_id IN (' + ','.join('?' for _ in batch) + ')', batch).fetchall()
                for row in rows:
                    acknowledgements.setdefault(row['internal_id'], {})[row['vector_name']] = row
            if getattr(store, 'storage', None) is self.storage:
                native_points, native_vectors = self._native_rows(self.storage, point_ids)
        points = None
        cache_errors = {}
        if native_points is None and getattr(store, 'storage', None) is not None:
            native_points, native_vectors = self._native_rows(store.storage, point_ids)
        if native_points is None:
            # Separately injected I/O never holds the authoritative source lock.
            points = {}
            try:
                if hasattr(store, 'get_points_batch'):
                    points = store.get_points_batch(point_ids)
                else:
                    raise NotImplementedError()
            except Exception:
                for point_id in point_ids:
                    try:
                        points[point_id] = store.get_point(point_id)
                    except Exception as exc:
                        points[point_id] = None
                        cache_errors[point_id] = type(exc).__name__
        binding = self.service._index_model_binding()
        repairs = []
        for kind, live in visible:
            point_id = getattr(live, SOURCES[kind][1])
            row = source_rows.pop((kind, point_id), None)
            state = row if row is not None and row['internal_id'] else None
            current = None
            state_status = None
            acknowledged = acknowledgements.get(state['internal_id'], {}) if state else {}
            if state and state['desired_receipt']:
                current = Receipt(state['desired_receipt'], SourceVersion(self.key(state), state['input_incarnation'], state['input_revision']))
                state_status = {'status': state['status'], 'error_category': state['error_category']}
            record = None
            if row is not None:
                token = self.business_token(state) if state else ('raw', (row['raw_payload'], row['source_map_name'], row['source_library_name']))
                record = self.repository._load_record(SOURCES[kind][2],
                    {'payload_json': row['raw_payload'], 'map_name': row['source_map_name'], 'library_name': row['source_library_name']})
                object.__setattr__(record, TOKEN, token)
            if record is None and getattr(live, TOKEN, None) is not None and not getattr(live, '_memory_live_only', False):
                continue
            source = record or live
            ephemeral = getattr(live, '_memory_live_only', False)
            if record is not None and ephemeral and getattr(live, TOKEN, None) == getattr(record, TOKEN, None):
                source = live
            persisted_projection = json.loads(state['projection_json']) if state else None
            live_projection = False
            if state and source is not live or state and not ephemeral:
                projection = persisted_projection
                # Payload normalization is read-only and never rewrites legacy bytes.
                if projection['payload'].get('map_name') is None and source.map_name is not None:
                    projection['payload']['map_name'] = source.map_name
                inputs_current = True
            else:
                dep = self._dependency(kind, source)
                parent = self.load(*dep) if dep else None
                dep_value = self._dependency_value(dep[0], parent, self.active_library) if dep else None
                projection = self.service._prepare_index_projection(kind, source, dependency_value=dep_value, binding=binding)
                live_projection = True
                inputs_current = state is not None
            if self.repository._resolve_record_library_name(source) != self.active_library or not projection['eligible']:
                continue
            if native_points is not None:
                point_row = native_points.pop(point_id, None)
                point = self._service_point(point_row, native_vectors.pop(point_id, [])) if point_row is not None else None
            else:
                point = self._owned_external_point(points.get(point_id))
            if record is None:
                # Live-only observations have no authoritative persisted source;
                # even compatible-looking old cache rows cannot supply scores.
                point = None
            valid, evidence = {}, {}
            unverified = set()
            if point_id in cache_errors:
                counts['cache_read_failures'] = counts.get('cache_read_failures', 0) + 1
                evidence['_cache'] = {'status': 'unavailable', 'reason': 'derived_cache_read_failed', 'error_category': cache_errors[point_id]}
            if point is not None:
                stored_evidence = point.payload.get(INDEX_META, {}).get('vectors', {})
                for name, named in point.vectors.items():
                    reason = None
                    proof = stored_evidence.get(name)
                    invalid_proof = proof is not None and not isinstance(proof, dict)
                    if invalid_proof:
                        proof = None
                    if proof:
                        ack = acknowledged.get(name)
                        publication_current = current is not None and self._publication_matches(proof, ack, current)
                        route = proof.get('route')
                        route_current = inputs_current and (not live_projection or self.route_inputs_equal(persisted_projection, projection, route))
                        binding_status = self.route_binding_status(proof.get('binding', projection['binding']), binding, route)
                        versioned = publication_current and self._ack_matches(proof, ack, current, named)
                        if (not publication_current or not route_current or binding_status == 'incompatible'
                                or (publication_current and not versioned and self._ack_conflicts(proof, ack, named))):
                            reason = 'input_or_incarnation_changed'
                        if reason is None and binding_status == 'legacy':
                            reason = 'provider_version_unverified'
                        disposition = 'validated_current' if versioned and binding_status == 'current' else 'legacy_unverified'
                    else:
                        disposition = 'legacy_unverified'
                        baseline = point.payload.get(INDEX_META, {}).get('legacy_inputs', {}).get(name, point.payload)
                        old_library = baseline.get('memory_library_name')
                        if old_library and old_library != self.active_library:
                            reason = 'legacy_ownership_conflict'
                        elif baseline.get('map_name') and baseline.get('map_name') != projection['payload'].get('map_name'):
                            reason = 'legacy_map_conflict'
                        elif point.record_kind != projection['record_kind']:
                            reason = 'legacy_source_kind_conflict'
                        elif name == 'text_dense' and 'retrieval_text' in baseline and baseline['retrieval_text'] != projection['text']:
                            reason = 'legacy_text_changed'
                        elif name == 'image_dense' and 'artifact_ids' in baseline and baseline['artifact_ids'] != projection['artifacts']:
                            reason = 'legacy_image_references_changed'
                    import numpy as np
                    provider = self.service._text_embedder if name == 'text_dense' else self.service._image_embedder
                    if not projection['eligible']:
                        reason = 'source_ineligible'
                    if provider is None or named.model_name != provider.model_name or named.dimension != provider.dimension:
                        reason = 'model_or_dimension_incompatible'
                    if named.vector.ndim != 1 or named.vector.size != named.dimension or not np.isfinite(named.vector).all():
                        reason = 'invalid_vector_bytes'
                    if invalid_proof or point.payload.get('_invalid_cached_payload'):
                        reason = 'invalid_cached_provenance'
                    if reason == 'provider_version_unverified':
                        counts['legacy_unverified'] += 1
                        unverified.add(name)
                        evidence[name] = {'status': 'legacy_unverified', 'reason': reason}
                    elif reason:
                        counts['known_stale_or_incompatible'] += 1
                        evidence[name] = {'status': 'known_stale_or_incompatible', 'reason': reason}
                    else:
                        counts[disposition] += 1
                        valid[name] = named
                        evidence[name] = {'status': disposition, 'reason': ('legacy_provenance_absent' if proof is None else 'legacy_publication_version') if disposition == 'legacy_unverified' else None,
                                          'model_name': named.model_name, 'dimension': named.dimension,
                                          'route': proof.get('route') if proof else None,
                                          'fallback_reason': proof.get('fallback_reason') if proof else None}
            if not valid:
                counts['missing'] += 1
            # This projection was decoded or built privately for this snapshot.
            payload = projection['payload']
            payload['vector_provenance'] = evidence
            applicability = self.applicable_modalities(projection) if record is not None else set()
            readiness = {} if include_readiness else None
            for name in ('text_dense', 'image_dense'):
                if name in valid:
                    status = evidence[name]['status']
                elif name in unverified:
                    status = 'legacy_unverified'
                elif name not in applicability:
                    if name == 'text_dense' and self.service._text_embedder is None and self.service._text_embedder_ready:
                        status = 'unavailable' if self.service._text_embedding_provider_metadata.get('availability') == 'unavailable' else 'disabled'
                    else:
                        status = 'disabled' if name == 'image_dense' and self.service._configured_image_model.lower() == 'disabled' and self.service._image_embedder is None else 'not_applicable'
                    counts[status] += 1
                else:
                    status = 'failed' if state_status and state_status['status'] == 'failed_terminal' else 'pending'
                    if name == 'image_dense' and self.service._image_embedder is None and status == 'pending':
                        status = 'unavailable'
                    counts[status] += 1
                if readiness is not None:
                    readiness[name] = {'status': status, 'work_status': state_status['status'] if state_status else 'legacy_pending',
                                       'error_category': state_status.get('error_category') if state_status else None}
            if readiness is not None:
                payload['modality_readiness'] = readiness
            if current is not None and state_status['status'].startswith('completed_') and not applicability.issubset(set(valid) | unverified):
                repairs.append((kind, point_id, current.version))
            results.append((kind, source, VectorPoint(point_id, projection['collection'], projection['record_kind'],
                                                     payload, valid, str(payload.get('timestamp') or '')),
                            current.version if current else (None if record is not None else False)))
        for kind, point_id, version in repairs:
            self.repair_if_current(kind, point_id, version)
        return results, counts

    def query_versions_current(self, snapshots):
        with self.storage.read_snapshot() as tx:
            for kind, source, point, version in snapshots:
                if version is False:
                    if self._raw(tx, kind, point.point_id) is not None:
                        return False
                    continue
                row = self._state(tx, kind, point.point_id)
                current = self.outbox.current(self.key(row)) if row else None
                if (current.version if current else None) != version:
                    return False
                if self._raw(tx, kind, point.point_id) is None and (version is not None or getattr(source, TOKEN, None) is not None):
                    return False
        return True

    def _refresh_shared_payload(self, tx, prepared, revision):
        store = self.service._vector_store
        if getattr(store, 'storage', None) is not self.storage:
            return
        row = tx.execute('SELECT payload_json FROM memory_vector_points WHERE point_id=?', (prepared.point_id,)).fetchone()
        if row is None:
            return
        self._archive(tx, prepared.point_id)
        payload = json.loads(row[0])
        metadata = dict(payload.get(INDEX_META, {}))
        legacy = dict(metadata.get('legacy_inputs', {}))
        proofs = metadata.get('vectors', {})
        for name, in tx.execute('SELECT vector_name FROM memory_named_vectors WHERE point_id=?', (prepared.point_id,)):
            if name not in proofs and name not in legacy:
                legacy[name] = {key: payload[key] for key in ('retrieval_text', 'artifact_ids', 'memory_library_name', 'map_name', 'record_kind') if key in payload}
        metadata.update({'source_kind': prepared.kind, 'library_name': prepared.library,
                         'business_revision': revision, 'legacy_inputs': legacy})
        updated = dict(prepared.projection['payload'])
        updated[INDEX_META] = metadata
        columns = ('map_name','localization_session_id','linked_location_id','camera_id','topo_node_id',
                   'semantic_region_id','anchor_id','instance_type','movability','last_seen_ts')
        tx.execute('UPDATE memory_vector_points SET ' + ','.join(key+'=?' for key in columns) +
                   ',timestamp_ts=?,payload_json=? WHERE point_id=?',
                   tuple(updated.get(key) for key in columns) + (str(updated.get('timestamp') or ''), canonical(updated), prepared.point_id))

    def acquire_worker(self):
        """An OS lease proves a crashed prior file-backed worker is no longer live."""
        if self.storage.db_path == ':memory:':
            return
        import fcntl
        handle = open(os.path.realpath(self.storage.db_path) + '.index-worker.lock', 'a+b')
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            handle.close()
            raise OutboxConflict('Another physical memory worker owns this database')
        self._worker_file = handle

    def release_worker(self):
        handle = self._worker_file
        if handle is not None:
            self._worker_file = None
            handle.close()

    def applicable_modalities(self, projection):
        if not projection.get('eligible', False):
            return set()
        names = set()
        if self.service._text_embedder is not None or not self.service._text_embedder_ready:
            names.add('text_dense')
        if projection.get('artifacts') and (self.service._configured_image_model.lower() != 'disabled' or self.service._image_embedder is not None):
            names.add('image_dense')
        return names

    @staticmethod
    def _publication_matches(proof, acknowledgement, receipt):
        if not proof or not acknowledgement or not proof.get('publication_id'):
            return False
        version = receipt.version
        identity = {'library_incarnation': version.key.library_incarnation, 'source_kind': version.key.source_kind,
                    'internal_id': version.key.point_id, 'record_incarnation': version.record_incarnation}
        return (all(proof.get(key) == value for key, value in identity.items())
                and acknowledgement['publication_id'] == proof['publication_id']
                and acknowledgement['valid_generation'] == version.revision)

    @staticmethod
    def _ack_conflicts(proof, acknowledgement, named):
        # 缺失的旧证明只降低证据等级，已有版本的明确冲突仍须逐项处理。
        revision = acknowledgement['vector_revision']
        if revision and (revision != named.storage_revision or revision != proof.get('vector_revision')):
            return True
        if acknowledgement['route'] and proof.get('route') != acknowledgement['route']:
            return True
        for column, key in (('input_versions_json', 'input_versions'), ('binding_json', 'binding')):
            raw = acknowledgement[column]
            if raw not in ('', '{}') and proof.get(key) != json.loads(raw):
                return True
        return proof.get('proof_version') == 2 and (
            proof.get('model_name') != named.model_name or proof.get('dimension') != named.dimension)

    @classmethod
    def _ack_matches(cls, proof, acknowledgement, receipt, named):
        return (cls._publication_matches(proof, acknowledgement, receipt)
                and proof.get('proof_version') == 2 and bool(acknowledgement['vector_revision'])
                and bool(acknowledgement['route']) and acknowledgement['input_versions_json'] not in ('', '{}')
                and acknowledgement['binding_json'] not in ('', '{}')
                and not cls._ack_conflicts(proof, acknowledgement, named))

    def _retarget_compatible(self, tx, internal_id, receipt, projection):
        for row in tx.execute('SELECT * FROM memory_index_publications WHERE internal_id=?', (internal_id,)).fetchall():
            proof = json.loads(row['provenance_json'])
            if not projection['eligible'] or proof.get('proof_version') != 2 or not row['vector_revision'] or row['binding_json'] in ('', '{}'):
                continue
            route = row['route']
            if route not in ('text_only', 'text_only_fallback', 'fused_text_image', 'image_only'):
                continue
            if json.loads(row['input_versions_json']) != self.route_versions(projection, route):
                continue
            if self.route_binding_status(proof['binding'], projection['binding'], route) == 'incompatible':
                continue
            tx.execute('UPDATE memory_index_publications SET valid_generation=? WHERE internal_id=? AND vector_name=?',
                       (receipt.version.revision, internal_id, row['vector_name']))

    def cleanup_cache_page(self, limit=16):
        store = self.service._vector_store
        if getattr(store, 'storage', None) is self.storage:
            return
        with self.storage.read_snapshot() as tx:
            pending = tx.execute('SELECT point_id,record_incarnation FROM memory_index_cache_cleanup LIMIT ?', (limit,)).fetchall()
        for point_id, incarnation in pending:
            try:
                owner = getattr(store, 'storage', None)
                if owner is not None:
                    with owner.transaction() as tx:
                        row = tx.execute('SELECT payload_json FROM memory_vector_points WHERE point_id=?', (point_id,)).fetchone()
                        payload = json.loads(row[0]) if row else {}
                        proofs = payload.get(INDEX_META, {}).get('vectors', {})
                        if not proofs or all(proof.get('record_incarnation') == incarnation for proof in proofs.values()):
                            store.delete_points([point_id])
                        if tx.execute("SELECT 1 FROM sqlite_master WHERE name='memory_index_legacy_archive'").fetchone():
                            tx.execute('DELETE FROM memory_index_legacy_archive WHERE point_id=?', (point_id,))
                else:
                    point = store.get_point(point_id)
                    proofs = point.payload.get(INDEX_META, {}).get('vectors', {}) if point else {}
                    if not proofs or all(proof.get('record_incarnation') == incarnation for proof in proofs.values()):
                        store.delete_points([point_id])
                with self.storage.transaction() as tx:
                    tx.execute('DELETE FROM memory_index_cache_cleanup WHERE point_id=? AND record_incarnation=?', (point_id, incarnation))
            except Exception as exc:
                with self.storage.transaction() as tx:
                    tx.execute('UPDATE memory_index_cache_cleanup SET error_category=? WHERE point_id=? AND record_incarnation=?',
                               (type(exc).__name__, point_id, incarnation))

    def retire_library(self, library):
        """Called in the source-deletion transaction; never reuses incarnation."""
        with self.storage.transaction() as tx:
            alias = tx.execute("SELECT internal_key FROM memory_index_aliases WHERE alias_kind='lib' AND namespace='' AND external_key=?", (library,)).fetchone()
            if alias:
                identity_row = tx.execute('SELECT name,incarnation FROM memory_vectorization_libraries WHERE name=?', (alias[0],)).fetchone()
                if identity_row:
                    from services.memory.vectorization_outbox import LibraryIdentity
                    self.outbox.delete_library(LibraryIdentity(*identity_row), delete_source=lambda tx, identity: None)
                    tx.execute('DELETE FROM memory_record_natural_ids WHERE library_incarnation=?', (identity_row['incarnation'],))
                    from services.memory.scene_identity import retire_library as retire_scene_identities
                    retire_scene_identities(tx, identity_row['incarnation'])
            tx.execute('DELETE FROM memory_index_reconciliation WHERE library_name=?', (library,))
            tx.execute("DELETE FROM memory_index_aliases WHERE alias_kind='lib' AND namespace='' AND external_key=?", (library,))

    def repair_if_current(self, kind, point_id, expected):
        """CAS repair completed work whose acknowledged cache bytes disappeared."""
        with self.storage.transaction() as tx:
            state = self._state(tx, kind, point_id)
            if state is None:
                return False
            current = self.outbox.current(self.key(state))
            status = self.outbox.inspect(self.key(state))
            if current is None or current.version != expected or not status['status'].startswith('completed_'):
                return False
            repaired = self.outbox.accept(self.key(state), expected=expected,
                               write_source=lambda tx, version: None)
            self._retarget_compatible(tx, state['internal_id'], repaired, json.loads(state['projection_json']))
            return True

    def validated_point(self, point_id):
        """The same eligibility rules for arrival verification and other lookups."""
        candidates = []
        with self.service._instances_lock:
            for kind, cache in (('location', self.service._tagged_locations_by_id), ('memory', self.service._semantic_entries_by_id),
                                ('instance', self.service._semantic_instances_by_id), ('event', self.service._observation_events_by_id),
                                ('scene', self.service._scene_spaces_by_id)):
                if point_id in cache:
                    candidates.append((kind, cache[point_id]))
        snapshots, _ = self.query_snapshot(candidates)
        if not self.query_versions_current(snapshots):
            return None
        return snapshots[0][2] if len(snapshots) == 1 else None


class MemoryPayloadMatcher:
    """Stateless filter semantics shared with SQLite, including injected stores."""
    from services.memory.vector_store import SQLiteNamedVectorStore as _Store
    matches = _Store._payload_matches
    _extract_timestamp = _Store._extract_timestamp
    _distance_to_pose = _Store._distance_to_pose
