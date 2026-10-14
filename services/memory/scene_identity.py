"""Transactional natural identities for builder entities and scene-owned samples.

Preparation reads bounded legacy evidence and allocates private provisional IDs.
Only admission inside the source writer transaction can persist an alias.
"""
from __future__ import annotations

import json
import uuid
from typing import Dict, Tuple

from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.naming import build_anchor_id, build_scene_space_id, validate_anchor_id, validate_scene_space_id
from contracts.spatial_memory import SceneSpace

MAX_CANDIDATES = 64
_SCOPE_FIELDS = ("map_name", "source_name", "source_epoch", "frame_id")


class IdentityScopeChanged(RuntimeError):
    """The captured owner or legacy source evidence needs fresh preparation."""


class IdentityPlanChanged(RuntimeError):
    """Concurrent admission selected public identities before this plan."""
    def __init__(self, remap):
        super().__init__("Scene identities changed; remap and reprepare source writes")
        self.remap = dict(remap)


def install_schema(tx):
    tx.execute("CREATE TABLE IF NOT EXISTS memory_scene_identity_control (singleton INTEGER PRIMARY KEY CHECK(singleton=1),reset_epoch TEXT NOT NULL)")
    tx.execute("INSERT OR IGNORE INTO memory_scene_identity_control VALUES(1,?)", (uuid.uuid4().hex,))
    tx.execute("""CREATE TABLE IF NOT EXISTS memory_scene_identity_scopes (
        scope_id TEXT PRIMARY KEY, library_incarnation TEXT NOT NULL, publication_token TEXT NOT NULL DEFAULT '',
        map_name TEXT NOT NULL, source_name TEXT NOT NULL, source_epoch TEXT NOT NULL, frame_id TEXT NOT NULL,
        UNIQUE(library_incarnation,map_name,source_name,source_epoch,frame_id))""")
    if 'publication_token' not in {row[1] for row in tx.execute('PRAGMA table_info(memory_scene_identity_scopes)')}:
        tx.execute("ALTER TABLE memory_scene_identity_scopes ADD COLUMN publication_token TEXT NOT NULL DEFAULT ''")
    tx.execute("""CREATE TABLE IF NOT EXISTS memory_scene_entity_ids (
        scope_id TEXT NOT NULL REFERENCES memory_scene_identity_scopes(scope_id) ON DELETE CASCADE,
        entity_kind TEXT NOT NULL, source_kind TEXT NOT NULL, source_key TEXT NOT NULL, public_id TEXT NOT NULL,
        PRIMARY KEY(scope_id,entity_kind,source_kind,source_key), UNIQUE(scope_id,entity_kind,public_id))""")
    tx.execute("CREATE INDEX IF NOT EXISTS idx_scene_entity_public ON memory_scene_entity_ids(entity_kind,public_id)")
    tx.execute("""CREATE TABLE IF NOT EXISTS memory_scene_identity_publication_scenes (
        scope_id TEXT NOT NULL REFERENCES memory_scene_identity_scopes(scope_id) ON DELETE CASCADE,
        scene_space_id TEXT NOT NULL, PRIMARY KEY(scope_id,scene_space_id))""")
    tx.execute("CREATE INDEX IF NOT EXISTS idx_scene_identity_publication_scene ON memory_scene_identity_publication_scenes(scene_space_id)")
    tx.execute("""CREATE TABLE IF NOT EXISTS memory_scene_sample_ids (
        library_incarnation TEXT NOT NULL, owner_scene_id TEXT NOT NULL,
        source_kind TEXT NOT NULL, source_key TEXT NOT NULL, public_id TEXT NOT NULL,
        PRIMARY KEY(library_incarnation,owner_scene_id,source_kind,source_key),
        UNIQUE(library_incarnation,owner_scene_id,public_id))""")
    tx.execute("DROP TRIGGER IF EXISTS memory_scene_identity_delete_scene")
    tx.execute("""CREATE TRIGGER memory_scene_identity_delete_scene AFTER DELETE ON scene_spaces BEGIN
        UPDATE memory_scene_identity_scopes SET publication_token='' WHERE scope_id IN
            (SELECT scope_id FROM memory_scene_entity_ids WHERE entity_kind='scene' AND public_id=OLD.scene_space_id
             UNION SELECT scope_id FROM memory_scene_identity_publication_scenes WHERE scene_space_id=OLD.scene_space_id);
        DELETE FROM memory_scene_identity_publication_scenes WHERE scene_space_id=OLD.scene_space_id;
        DELETE FROM memory_scene_sample_ids WHERE owner_scene_id=OLD.scene_space_id;
        DELETE FROM memory_scene_entity_ids WHERE entity_kind='scene' AND public_id=OLD.scene_space_id;
        END""")


def retire_library(tx, incarnation):
    tx.execute("DELETE FROM memory_scene_sample_ids WHERE library_incarnation=?", (incarnation,))
    tx.execute("DELETE FROM memory_scene_identity_scopes WHERE library_incarnation=?", (incarnation,))


def clear_all(tx):
    tx.execute("UPDATE memory_scene_identity_control SET reset_epoch=? WHERE singleton=1", (uuid.uuid4().hex,))
    tx.execute("DELETE FROM memory_scene_sample_ids")
    tx.execute("DELETE FROM memory_scene_entity_ids")
    tx.execute("DELETE FROM memory_scene_identity_scopes")


def _library_incarnation(tx, name):
    tables = {row[0] for row in tx.execute("SELECT name FROM sqlite_master WHERE type='table' AND name IN ('memory_index_aliases','memory_vectorization_libraries')")}
    if len(tables) != 2:
        raise IdentityScopeChanged("A durable library incarnation must exist before scene identity preparation")
    row = tx.execute("""SELECT l.incarnation FROM memory_index_aliases a
        JOIN memory_vectorization_libraries l ON l.name=a.internal_key
        WHERE a.alias_kind='lib' AND a.namespace='' AND a.external_key=?""", (name,)).fetchone()
    if row is None:
        raise IdentityScopeChanged("Scene identity library is absent or retired")
    return row[0]


def _sample_key(sample):
    metadata = sample.metadata
    kind = metadata.get("source_record_kind")
    source_instance = str(metadata.get("source_instance_id") or "").strip()
    if kind == "semantic_instance" or (kind is None and source_instance):
        return ("semantic_instance", source_instance) if source_instance else None
    if kind in (None, "observation_event") and sample.observation_event_id:
        return "observation_event", sample.observation_event_id
    return None


def _explicit_scope(payload):
    metadata = payload.get("metadata") or {}
    candidates = []
    for value in (metadata.get("scene_identity_scope"), metadata.get("map_binding"),
                  {**metadata, "map_name": payload.get("map_name") or metadata.get("map_name"),
                   "frame_id": payload.get("frame_id") or metadata.get("frame_id")}):
        if isinstance(value, dict) and all(isinstance(value.get(key), str) and value[key] for key in _SCOPE_FIELDS):
            candidates.append(tuple(value[key] for key in _SCOPE_FIELDS))
    return candidates[0] if candidates and all(value == candidates[0] for value in candidates) else None


class SceneIdentityPlan:
    """One library/map or library/scene identity admission plan; not shared state."""
    def __init__(self, repository, library_name, *, snapshot=None, scene=None):
        if (snapshot is None) == (scene is None):
            raise ValueError("Provide exactly one map snapshot or owning scene")
        self.repository = repository
        self.storage = repository.storage
        self.library_name = str(library_name).strip()
        self.scope_metadata = None
        self.owner_scene_id = scene.scene_space_id if scene is not None else None
        self._scope_id = None
        self._values: Dict[Tuple[str, str, str], str] = {}
        self._diagnostics = []
        self._seeded = {}
        self._origins = {}
        self._evidence = {}
        self._registered = set()
        with self.storage.read_snapshot() as tx:
            self.library_incarnation = _library_incarnation(tx, self.library_name)
            self._reset_epoch = tx.execute("SELECT reset_epoch FROM memory_scene_identity_control WHERE singleton=1").fetchone()[0]
            if snapshot is not None:
                frame = snapshot.frame_id or next((layer.frame_id for layer in (
                    snapshot.semantic_map, snapshot.occupancy_grid, snapshot.cost_map
                ) if layer is not None and layer.frame_id), DEFAULT_MAP_FRAME)
                self.scope_metadata = {key: getattr(snapshot, key) for key in _SCOPE_FIELDS}
                self.scope_metadata['frame_id'] = frame
                if not all(isinstance(value, str) and value for value in self.scope_metadata.values()):
                    raise ValueError("Scene entity identity requires an explicit complete map scope")
                self._scope_id = self._lookup_scope(tx)
            else:
                row = tx.execute("SELECT payload_json,map_name,library_name,graph_source_revision FROM scene_spaces WHERE scene_space_id=?", (self.owner_scene_id,)).fetchone()
                if row is not None and row['library_name'] != self.library_name:
                    raise IdentityScopeChanged("Sample owner belongs to another library")
                self._owner_revision = row['graph_source_revision'] if row is not None else None
                if row is None and repository._resolve_record_library_name(scene) != self.library_name:
                    raise IdentityScopeChanged("New sample owner belongs to another library")
                if (hasattr(scene, '_scene_graph_source_revision')
                        and scene._scene_graph_source_revision != self._owner_revision):
                    raise IdentityScopeChanged("Sample input is an obsolete owner version")
                source_scene = repository._load_record(SceneSpace, row) if row is not None else scene
                self._seed_samples(source_scene)
        # Seed aliases even if the sampler reuses the actual stored sample ID
        # and therefore does not invoke the callback for that source.
        for kind, source_id in self._seeded:
            self.resolve_sample(kind, source_id)

    @property
    def diagnostics(self):
        return tuple(dict(item) for item in self._diagnostics)

    def _diagnose(self, key, status, **details):
        item = {"entity_kind": key[0], "source_kind": key[1], "source_key": key[2], "status": status, **details}
        if item not in self._diagnostics:
            self._diagnostics.append(item)

    def _lookup_scope(self, tx):
        row = tx.execute("""SELECT scope_id FROM memory_scene_identity_scopes
            WHERE library_incarnation=? AND map_name=? AND source_name=? AND source_epoch=? AND frame_id=?""",
            (self.library_incarnation, *(self.scope_metadata[key] for key in _SCOPE_FIELDS))).fetchone()
        return row[0] if row else None

    def _alias(self, tx, key, scope_id=None):
        entity_kind, source_kind, source_key = key
        if entity_kind == "sample":
            row = tx.execute("""SELECT public_id FROM memory_scene_sample_ids WHERE library_incarnation=?
                AND owner_scene_id=? AND source_kind=? AND source_key=?""",
                (self.library_incarnation, self.owner_scene_id, source_kind, source_key)).fetchone()
        else:
            scope_id = scope_id if scope_id is not None else self._lookup_scope(tx)
            row = tx.execute("SELECT public_id FROM memory_scene_entity_ids WHERE scope_id=? AND entity_kind=? AND source_kind=? AND source_key=?",
                             (scope_id, *key)).fetchone() if scope_id is not None else None
        return row[0] if row else None

    def _seed_samples(self, scene):
        candidates, owners = {}, {}
        # The owning scene is already loaded and the sampler traverses this
        # same window. Retain all unique legacy aliases before selection can
        # evict evidence, even when an imported historical window is large.
        for sample in scene.observation_samples:
            key = _sample_key(sample)
            if key is None or not sample.sample_id:
                self._diagnose(("sample", "", sample.sample_id), "unbound")
                continue
            candidates.setdefault(key, set()).add(sample.sample_id)
            owners.setdefault(sample.sample_id, set()).add(key)
        for key, ids in candidates.items():
            if len(ids) == 1 and len(owners[next(iter(ids))]) == 1:
                self._seeded[key] = next(iter(ids))
            else:
                self._diagnose(("sample", *key), "ambiguous", candidate_ids=sorted(ids))

    def resolve(self, entity_kind, source_kind, source_key):
        if self.owner_scene_id is not None or entity_kind not in ("anchor", "scene") or source_kind not in ("region", "topo"):
            raise ValueError("Invalid map entity identity request")
        return self._resolve((entity_kind, source_kind, self._source_key(source_key)))

    def resolve_sample(self, source_kind, source_id):
        if self.owner_scene_id is None or source_kind not in ("observation_event", "semantic_instance"):
            raise ValueError("Invalid sample identity request")
        return self._resolve(("sample", source_kind, self._source_key(source_id)))

    @staticmethod
    def _source_key(value):
        if not isinstance(value, str) or not value:
            raise ValueError("Natural source identity must be a nonempty string")
        return value

    def _resolve(self, key):
        if key in self._values:
            return self._values[key]
        with self.storage.read_snapshot() as tx:
            if _library_incarnation(tx, self.library_name) != self.library_incarnation:
                raise IdentityScopeChanged("Scene identity library was replaced")
            canonical = self._alias(tx, key)
            candidate = self._seeded.get(key[1:]) if key[0] == "sample" else self._legacy_candidate(tx, key)
        if candidate is not None:
            value, origin = candidate, "adopted"
            if canonical is not None and canonical != candidate:
                self._diagnose(key, "ambiguous", candidate_ids=sorted({candidate, canonical}))
                origin = "conflict"
        elif canonical is not None:
            value, origin = canonical, "registered"
        else:
            value, origin = self._new_id(key), "fresh"
        if canonical == value:
            self._registered.add(key)
        if any(existing == value and other != key for other, existing in self._values.items()):
            self._diagnose(key, "ambiguous", candidate_ids=[value])
            value, origin = self._new_id(key), "fresh"
        self._values[key], self._origins[key] = value, origin
        return value

    @staticmethod
    def _new_id(key):
        return ("sample_" + uuid.uuid4().hex if key[0] == "sample" else
                (build_anchor_id if key[0] == "anchor" else build_scene_space_id)(key[1], suffix=uuid.uuid4().hex))

    def _legacy_candidate(self, tx, key):
        entity, kind, source = key
        link_field = "linked_semantic_region_ids" if kind == "region" else "linked_topology_node_ids"
        rows = tx.execute(f"""SELECT scene_space_id,graph_source_revision,
            json_object('scene_space_id',scene_space_id,'map_name',map_name,
                'frame_id',json_extract(payload_json,'$.frame_id'),
                'attributes',json_object('source',json_extract(payload_json,'$.attributes.source')),
                'metadata',json_object('scene_identity_scope',json_extract(payload_json,'$.metadata.scene_identity_scope'),
                    'map_binding',json_extract(payload_json,'$.metadata.map_binding'),
                    'source_name',json_extract(payload_json,'$.metadata.source_name'),
                    'source_epoch',json_extract(payload_json,'$.metadata.source_epoch'),
                    'source',json_extract(payload_json,'$.metadata.source')),
                'linked_anchor_ids',json_extract(payload_json,'$.linked_anchor_ids'),
                'linked_semantic_region_ids',json_extract(payload_json,'$.linked_semantic_region_ids'),
                'linked_topology_node_ids',json_extract(payload_json,'$.linked_topology_node_ids')) AS identity_json
            FROM scene_spaces WHERE library_name=? AND map_name=?
            AND EXISTS(SELECT 1 FROM json_each(payload_json,'$.{link_field}') WHERE value=?)
            ORDER BY scene_space_id LIMIT ?""", (self.library_name, self.scope_metadata['map_name'], source, MAX_CANDIDATES + 1)).fetchall()
        if len(rows) > MAX_CANDIDATES:
            self._diagnose(key, "overflow", candidate_count=len(rows))
            return None
        wanted_scope = tuple(self.scope_metadata[field] for field in _SCOPE_FIELDS)
        candidates = set()
        evidence = {}
        for row in rows:
            before = set(candidates)
            payload = json.loads(row["identity_json"])
            scope = _explicit_scope(payload)
            if scope is None:
                self._diagnose(key, "unbound")
                continue
            if scope != wanted_scope:
                continue
            attributes, metadata = payload.get('attributes') or {}, payload.get('metadata') or {}
            provenance = attributes.get('source') or metadata.get('source')
            if provenance not in ('semantic_region', 'topology_node'):
                self._diagnose(key, 'unbound', reason='missing_builder_source')
                continue
            if entity == 'scene':
                expected_source = 'semantic_region' if kind == 'region' else 'topology_node'
                if provenance == expected_source and payload.get(link_field) == [source]:
                    candidates.add(payload['scene_space_id'])
                    evidence[payload['scene_space_id']] = (row['scene_space_id'], row['graph_source_revision'])
                continue
            anchors = payload.get('linked_anchor_ids') or []
            if kind == 'region' and provenance == 'semantic_region' and payload.get(link_field) == [source]:
                # Builder-generated legacy IDs encode their producer kind; the
                # actual opaque suffix is never reconstructed or interpreted.
                if anchors and anchors[0].startswith('anc_region_'):
                    candidates.add(anchors[0])
            elif kind == 'topo' and payload.get(link_field) == [source]:
                candidates.update(value for value in anchors if value.startswith('anc_topo_'))
            for value in candidates - before:
                evidence[value] = (row['scene_space_id'], row['graph_source_revision'])
        if len(candidates) == 1:
            value = next(iter(candidates))
            try:
                (validate_anchor_id if entity == 'anchor' else validate_scene_space_id)(value)
            except ValueError:
                self._diagnose(key, 'unbound')
                return None
            self._evidence[key] = evidence[value]
            return value
        if candidates:
            self._diagnose(key, 'ambiguous', candidate_ids=sorted(candidates))
        return None

    def admit(self, tx):
        """Validate first, then insert aliases in the caller's source transaction.

        Map admission returns a scope-local publication token. Sample admission
        returns None. The caller owns source writes and rollback of both.
        """
        if tx.owner is not self.storage or self.storage.current_transaction is not tx:
            raise ValueError("Identity admission requires this repository's current writer transaction")
        if (_library_incarnation(tx, self.library_name) != self.library_incarnation
                or tx.execute("SELECT reset_epoch FROM memory_scene_identity_control WHERE singleton=1").fetchone()[0] != self._reset_epoch):
            raise IdentityScopeChanged("Scene identity library was replaced or reset")
        if self.owner_scene_id is not None:
            owner = tx.execute("SELECT library_name,graph_source_revision FROM scene_spaces WHERE scene_space_id=?", (self.owner_scene_id,)).fetchone()
            if ((owner['graph_source_revision'] if owner is not None else None) != self._owner_revision
                    or owner is not None and owner['library_name'] != self.library_name):
                raise IdentityScopeChanged("Sample owner changed during identity preparation")
        scope_id = self._lookup_scope(tx) if self.scope_metadata is not None else None
        winners = {key: self._alias(tx, key, scope_id) for key in self._values}
        remap = {}
        missing = []
        for key, value in self._values.items():
            winner = winners[key]
            if winner is not None and winner != value:
                if self._origins[key] == 'fresh':
                    remap[value] = winner
                else:
                    self._diagnose(key, 'ambiguous', candidate_ids=sorted({winner, value}))
                    self._origins[key] = 'conflict'
            elif winner is None:
                if key in self._registered:
                    raise IdentityScopeChanged("A previously registered scene identity was retired")
                missing.append(key)
        if remap:
            raise IdentityPlanChanged(remap)
        # Only actual adoption witnesses are fenced. Unrelated source writes
        # cannot invalidate new key allocation or existing canonical aliases.
        for key in self._values:
            if key in self._evidence and winners[key] != self._values[key]:
                scene_id, revision = self._evidence[key]
                current = tx.execute("SELECT graph_source_revision,library_name FROM scene_spaces WHERE scene_space_id=?", (scene_id,)).fetchone()
                if (current is None or current['library_name'] != self.library_name
                        or current['graph_source_revision'] != revision
                        or self._legacy_candidate(tx, key) != self._values[key]):
                    raise IdentityScopeChanged("A specific legacy identity witness changed; prepare again")
        admitted = []
        for key in missing:
            if key[0] == 'sample':
                owner = tx.execute("SELECT source_kind,source_key FROM memory_scene_sample_ids WHERE library_incarnation=? AND owner_scene_id=? AND public_id=?",
                    (self.library_incarnation, self.owner_scene_id, self._values[key])).fetchone()
            else:
                owner = tx.execute("SELECT source_kind,source_key FROM memory_scene_entity_ids WHERE scope_id=? AND entity_kind=? AND public_id=?",
                    (scope_id, key[0], self._values[key])).fetchone() if scope_id is not None else None
            if owner is not None and tuple(owner) != key[1:]:
                if self._origins[key] == 'fresh':
                    raise IdentityScopeChanged("A provisional public identity collided; prepare a fresh ID")
                self._diagnose(key, 'ambiguous', candidate_ids=[self._values[key]], reason='public_id_has_other_source')
                self._origins[key] = 'conflict'
            else:
                admitted.append(key)
        if self.scope_metadata is not None and scope_id is None:
            scope_id = uuid.uuid4().hex
            tx.execute("INSERT INTO memory_scene_identity_scopes(scope_id,library_incarnation,publication_token,map_name,source_name,source_epoch,frame_id) VALUES(?,?,?,?,?,?,?)",
                       (scope_id, self.library_incarnation, '', *(self.scope_metadata[field] for field in _SCOPE_FIELDS)))
        for key in admitted:
            if key[0] == 'sample':
                tx.execute("INSERT INTO memory_scene_sample_ids VALUES(?,?,?,?,?)",
                           (self.library_incarnation, self.owner_scene_id, key[1], key[2], self._values[key]))
            else:
                tx.execute("INSERT INTO memory_scene_entity_ids VALUES(?,?,?,?,?)", (scope_id, *key, self._values[key]))
        if self.scope_metadata is None:
            return None
        # A preserved legacy conflict can be returned in a context without
        # replacing its canonical alias. Track that actual scene dependency so
        # deleting it still fences the published context's warm cache.
        tx.execute("DELETE FROM memory_scene_identity_publication_scenes WHERE scope_id=?", (scope_id,))
        tx.executemany("INSERT INTO memory_scene_identity_publication_scenes VALUES(?,?)",
                       [(scope_id, value) for key, value in self._values.items() if key[0] == 'scene'])
        token = uuid.uuid4().hex
        tx.execute("UPDATE memory_scene_identity_scopes SET publication_token=? WHERE scope_id=?", (token, scope_id))
        self._scope_id = scope_id
        return token

    def is_current(self, token):
        """Read-only scope-local map publication fence; never checks a payload."""
        if self.scope_metadata is None or self._scope_id is None or not token:
            return False
        with self.storage.read_snapshot() as tx:
            row = tx.execute("""SELECT 1 FROM memory_scene_identity_scopes s
                JOIN memory_vectorization_libraries l ON l.incarnation=s.library_incarnation
                JOIN memory_index_aliases a ON a.internal_key=l.name
                WHERE s.scope_id=? AND s.publication_token=? AND l.incarnation=?
                    AND a.alias_kind='lib' AND a.namespace='' AND a.external_key=?""",
                (self._scope_id, token, self.library_incarnation, self.library_name)).fetchone()
            return row is not None

    def refresh(self, remap):
        """Accept only provisional-ID winners outside the source writer."""
        if self.storage.in_transaction:
            raise ValueError("Refresh source identities outside the writer transaction")
        for key, value in list(self._values.items()):
            if value in remap:
                if self._origins[key] != 'fresh':
                    raise ValueError("Stored legacy identities cannot be automatically remapped")
                self._values[key] = remap[value]
                self._origins[key] = 'registered'
                self._registered.add(key)
        return self
