"""Opaque retained checkpoint sources for the explicit assigned saved-map lane.

Only the runtime installation owner mints handles. Metadata comparisons never
compare the checkpoint, prepared activation, or immutable whole-map bodies.
"""
import json
from dataclasses import dataclass, field

from services.geometry.checkpoint_payload import FrozenPayload


@dataclass(frozen=True, init=False)
class AssignedSavedCloudHandle:
    """A saved source pinned to one exact process-local installation."""

    asset_id: str
    checkpoint_id: str
    installation_generation: int
    frame_id: str
    map_name: str
    point_count: int
    revision_ordinal: int
    source_timestamp_ns: int
    installed_mapping_source_epoch: str
    _owner: object = field(repr=False, compare=False)
    _snapshot: object = field(repr=False, compare=False)
    _prepared: object = field(repr=False, compare=False)
    canonical_cloud_xyz: object = field(repr=False, compare=False)
    dense_archive: object = field(repr=False, compare=False)
    dense_reference: object = field(repr=False, compare=False)

    def __init__(self, *args, **kwargs):
        raise TypeError("assigned saved sources must be captured from the runtime owner")

    @property
    def assigned_source(self):
        return {"identity_schema_version": 2, "asset_identity": {"asset_id": self.asset_id},
                "checkpoint_id": self.checkpoint_id,
                "installation_generation": str(self.installation_generation)}

    @property
    def source_epoch(self):
        return f"assigned:{self.asset_id}:{self.checkpoint_id}:{self.installation_generation}"

    @property
    def owner_key(self):
        return (self.asset_id, self.checkpoint_id, self.installation_generation, self.frame_id,
                id(self._owner), id(self._snapshot), id(self._prepared))

    def require_current(self):
        self._owner.require_assigned_saved_cloud_source(self)

    def same_installation_as(self, other):
        return (isinstance(other, AssignedSavedCloudHandle)
                and self._owner is other._owner and self._snapshot is other._snapshot
                and self._prepared is other._prepared)


def _capture_assigned_saved_cloud_handle(owner, snapshot, prepared):
    """Mint from already-admitted objects using bounded metadata checks only."""
    bundle, stored = snapshot.bundle, prepared.stored_checkpoint
    checkpoint, cloud = stored.checkpoint, stored.cloud_asset
    geometry = bundle.geometry_asset
    reference = geometry.metadata.get("mapping_backend_checkpoint") if geometry is not None else None
    if (bundle.asset_state != "finalized" or bundle.manifest_sha256 is not None
            or checkpoint.schema_version != 2 or not isinstance(reference, dict)
            or reference.get("schema_version") != 2
            or reference.get("checkpoint_id") != checkpoint.checkpoint_id
            or reference.get("backend_kind") != checkpoint.backend_kind
            or cloud is None or checkpoint.cloud_identity is None
            or checkpoint.cloud_identity.checkpoint_id != checkpoint.checkpoint_id
            or prepared.asset_identity != bundle.asset_identity
            or prepared.map_name != bundle.map_name):
        raise ValueError("assigned saved cloud differs from its admitted installation")
    frame_id = geometry.frame_id or bundle.frame_id
    payload = checkpoint.canonical_cloud_xyz
    count = cloud["point_count"]
    if (frame_id != cloud["frame_id"] or frame_id != checkpoint.cloud_identity.frame_id
            or not isinstance(payload, (bytes, FrozenPayload))
            or type(count) is not int or count <= 0 or len(payload) != count * 12
            or len(payload) != cloud["xyz_size"]):
        raise ValueError("assigned saved canonical cloud metadata is unavailable or inconsistent")
    activation = json.loads(snapshot.activation_result_json)
    session_id, session_generation = activation.get("session_id"), activation.get("session_generation")
    if not isinstance(session_id, str) or not session_id or type(session_generation) is not int or session_generation <= 0:
        raise ValueError("assigned saved cloud lacks installed mapping session metadata")
    result = object.__new__(AssignedSavedCloudHandle)
    for name, value in {
        "asset_id": bundle.asset_identity.asset_id, "checkpoint_id": checkpoint.checkpoint_id,
        "installation_generation": snapshot.generation, "frame_id": frame_id,
        "map_name": bundle.map_name, "point_count": count,
        "revision_ordinal": cloud["source_cloud_revision"],
        "source_timestamp_ns": checkpoint.cloud_receipt["source_timestamp_ns"],
        "installed_mapping_source_epoch": f"{session_id}:{session_generation}",
        "_owner": owner, "_snapshot": snapshot, "_prepared": prepared,
        "canonical_cloud_xyz": payload, "dense_archive": checkpoint.dense_archive,
        "dense_reference": checkpoint.dense_reference,
    }.items():
        object.__setattr__(result, name, value)
    return result
