"""GeometryRuntimeProvider — 通过 gRPC 调用 Rust nuwax-geometry-runtime 子进程。

Rust 运行时是主要的平台几何交互层（替代旧 nuwax-ros-bridge），本模块封装其 gRPC 接口，
提供与原始 Python Provider 相同的调用接口。
"""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from services.navigation.motion_guard_client import StopAllMotionConfirmation

from services.geometry.planar_archive import read_planar_structure
from services.geometry.checkpoint_payload import FrozenPayload, payload_chunks, MAX_PLANAR_ARCHIVE_BYTES
from core.config_values import runtime_value
from core.source_time import acquired_unix_ns, acquired_times, SourceTimeError

from contextlib import contextmanager
from collections import deque
import copy
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import logging
import math
import os
from pathlib import Path
import struct
import threading
import time
import uuid
from types import MappingProxyType
from typing import Any, Callable, Dict, List, Mapping, Optional, Set, Tuple, Union

import grpc

from contracts.frame_semantics import infer_frame_role
from contracts.geometry import FrameTree, Pose, Quaternion, Transform, Vector3
from contracts.geometry_runtime import DEFAULT_MAP_FRAME
from contracts.maps import OccupancyGrid
from contracts.navigation import NavigationGoal, NavigationState, NavigationStatus, navigation_reverse_wire_fields
from contracts.map_activation import (
    AssignedActivationReceipt,
    _request as _admit_assigned_activation_request,
)
from contracts.map_relocalization import (
    AssignedRelocalizationRequest,
    AssignedRelocalizationTerminal,
    AssignedRelocalizationQuery,
    _project_proto as _project_assigned_proto,
    _provider_fields as _assigned_proto_fields,
)
from services.geometry.proto_gen import geometry_runtime_pb2 as pb2
from services.geometry.proto_gen import geometry_runtime_pb2_grpc as pb2_grpc
from services.geometry.geometry_runtime_client import GeometryRuntimeSubprocess
from services.geometry.dense_cloud import MAX_DENSE_ARCHIVE_BYTES, DENSE_CHUNK_POINTS, dense_chunk_payload, validate_dense_archive
from services.geometry.fine_map_archive import MAX_FINE_ARCHIVE_BYTES, validate_fine_archive
from services.geometry.mapping_backend_checkpoint_store import (
    MappingBackendCheckpoint,
    CheckpointMemberIdentity,
    CheckpointArtifactReference,
    validate_artifact_reference,
    MappingBackendCheckpointStore,
    StoredMappingBackendCheckpoint,
    _identity_document,
)

LOGGER = logging.getLogger("nuwax_robot_bridge.geometry.geometry_runtime_provider")

_DEFAULT_GRPC_MESSAGE_LIMIT_BYTES = 128 * 1024 * 1024
_OCCUPANCY_SCHEMA_VERSION = 2
_ROLLOVER_RECOVERY_DEADLINE_SEC = 4.0

_MappingIdentity = Tuple[str, str, int]
_NavigationStreamToken = Tuple[int, int, int]
_NavigationLease = Tuple[str, int, str, int, int]
_NavigationCommandToken = Tuple[str, _NavigationLease, int, int]
_NavigationRequestIdentity = Tuple[str, int, str]
_NavigationSetPrecommitKey = Tuple[
    int,
    _NavigationStreamToken,
    _NavigationCommandToken,
    _NavigationRequestIdentity,
    _NavigationLease,
]
_NavigationDeliveryToken = Tuple[
    int,
    Optional[_NavigationStreamToken],
    int,
]
_PUBLIC_NAV_OWNER_KIND = int(pb2.NAVIGATION_OWNER_KIND_PUBLIC_NAV)
_EXPLORATION_CHILD_NAV_OWNER_KIND = int(
    pb2.NAVIGATION_OWNER_KIND_EXPLORATION_CHILD
)
_SUPPORTED_MAPPING_BACKEND_KINDS = frozenset({"raytracing", "voxel", "slam_toolbox"})
_RELOCALIZE_SUCCESS_DISPOSITIONS = frozenset(
    {
        int(pb2.RELOCALIZE_DISPOSITION_APPLIED),
        int(pb2.RELOCALIZE_DISPOSITION_ALREADY_APPLIED),
    }
)


def _decode_motion_admission_proof(value: object) -> Tuple[bytes, int]:
    """仅转换传输表示；凭据是否有效由守卫申请租约时判定。"""
    boot_nonce = bytes.fromhex(value["boot_nonce_hex"])
    if len(boot_nonce) != 16:
        raise ValueError("motion_admission.boot_nonce_hex must encode exactly 16 bytes")
    return boot_nonce, value["motion_stop_generation"]


def _ordinary_global_stop_confirmation_proto(confirmation: StopAllMotionConfirmation):
    """把同一次普通全局停车回执无损传给原生拥有方。"""
    return pb2.OrdinaryGlobalStopConfirmation(
        operation_id=confirmation.operation_id,
        zero_ack_id=confirmation.zero.zero_ack_id,
        applied_monotonic_ns=confirmation.zero.applied_monotonic_ns,
        stationary_confirmed=confirmation.zero.stationary_confirmed,
    )


class GeometryRuntimeRpcError(RuntimeError):
    """Typed unary RPC failure with stable diagnostic fields."""

    def __init__(
        self,
        *,
        rpc_name: str,
        grpc_code: object,
        grpc_details: str,
    ) -> None:
        self.rpc_name = str(rpc_name or "")
        self.grpc_code = grpc_code
        self.grpc_details = str(grpc_details or "")
        code_name = str(getattr(grpc_code, "name", "") or "UNKNOWN")
        super().__init__(
            "%s failed (%s): %s"
            % (self.rpc_name, code_name, self.grpc_details)
        )

    @classmethod
    def from_rpc_error(
        cls,
        rpc_name: str,
        exc: grpc.RpcError,
    ) -> "GeometryRuntimeRpcError":
        try:
            code_reader = getattr(exc, "code", None)
            grpc_code = code_reader() if callable(code_reader) else None
        except Exception:
            grpc_code = None
        if grpc_code is None:
            grpc_code = grpc.StatusCode.UNKNOWN

        try:
            details_reader = getattr(exc, "details", None)
            details = details_reader() if callable(details_reader) else None
        except Exception:
            details = None
        if not details:
            try:
                details = str(exc)
            except Exception:
                details = ""
        return cls(
            rpc_name=rpc_name,
            grpc_code=grpc_code,
            grpc_details=str(details or "gRPC call failed without details"),
        )


class GeometryRuntimeMapActivationOutcomeUnknown(RuntimeError):
    """ActivateMap crossed the RPC dispatch boundary without a trusted ACK."""

    backend_activation_outcome_unknown = True


class GeometryRuntimeMapActivationAppliedError(RuntimeError):
    """An exact native ACK survived, but local adoption could not complete."""

    backend_activation_applied = True
    backend_switch_applied = True
    backend_activation_disposition = "applied"


@dataclass(frozen=True)
class _AssignedActivationAttempt:
    """同连接内的未决 dispatch 证据；只保留已接纳的不可变载荷引用。"""
    request_key: tuple
    stored_checkpoint: StoredMappingBackendCheckpoint
    stub: object
    connection_epoch: int

    def matches(self, request_key, stored_checkpoint, origin):
        if (self.request_key != request_key or self.stub is not origin[0]
                or self.connection_epoch != origin[1]):
            return False
        original = self.stored_checkpoint
        if original.checkpoint.backend_kind == "slam_toolbox":
            # Pending planar replay stays bound to the retained immutable owner.
            # A matching ID or a newly reopened body is not that owner.
            return (original.checkpoint is stored_checkpoint.checkpoint
                and original.occupancy_asset == stored_checkpoint.occupancy_asset
                and stored_checkpoint.cloud_asset is None)
        # 重开文件只比较原生请求的实际成员；分块精确比较在状态锁外完成，不引入摘要。
        return original is stored_checkpoint or (
            original.checkpoint == stored_checkpoint.checkpoint
            and original.occupancy_asset == stored_checkpoint.occupancy_asset
            and original.cloud_asset == stored_checkpoint.cloud_asset
            and all(struct.pack(fmt, original.occupancy_asset[name])
                    == struct.pack(fmt, stored_checkpoint.occupancy_asset[name])
                    for name, fmt in (("resolution", ">f"), ("origin_x", ">d"), ("origin_y", ">d")))
            and struct.pack(">f", original.cloud_asset["voxel_size"])
                == struct.pack(">f", stored_checkpoint.cloud_asset["voxel_size"]))


@dataclass(frozen=True)
class _AssignedNativeLocalizationSample:
    """提供器解码一次的原生观察，位姿和坐标树按只读契约共享。"""

    _provider: object
    _origin: tuple
    _delivery_token: tuple
    terminal: AssignedRelocalizationTerminal
    source_timestamp_ns: int
    correction_revision: int
    map_frame_generation: int
    current_pose: Pose
    frame_tree: FrameTree



def _admit_assigned_localization_sample(response, *, provider, origin, delivery_token):
    """在协议边界构造一次样本，保留每条边和位姿的真实来源时间。"""
    if type(response) is not pb2.AssignedLocalizationSampleV2:
        raise ValueError("assigned localization sample has an unsupported protobuf type")
    if response.schema_version != 2 or any(not response.HasField(name) for name in (
            "terminal", "pose", "map_from_odom", "odom_from_body", "correction_owner")):
        raise ValueError("assigned localization sample is missing its native owners or geometry")
    terminal = AssignedRelocalizationTerminal.from_proto(response.terminal)
    for geometry, position, orientation in ((response.pose, "position", "orientation"),
                                            (response.map_from_odom, "translation", "rotation"),
                                            (response.odom_from_body, "translation", "rotation")):
        _require_proto_message_field(geometry, position)
        _require_proto_message_field(geometry, orientation)
    pose = _build_pose(response.pose)
    if pose is None:
        raise ValueError("assigned localization sample pose cannot be represented")
    transforms = []
    for edge in (response.map_from_odom, response.odom_from_body):
        timestamp = datetime.fromtimestamp(edge.timestamp_ns // 1_000_000_000, tz=timezone.utc).replace(
            microsecond=(edge.timestamp_ns % 1_000_000_000) // 1_000)
        transforms.append(Transform(
            timestamp=timestamp, parent_frame_id=edge.parent_frame, child_frame_id=edge.child_frame,
            translation=Vector3(x=edge.translation.x, y=edge.translation.y, z=edge.translation.z),
            rotation=Quaternion(x=edge.rotation.x, y=edge.rotation.y, z=edge.rotation.z, w=edge.rotation.w),
            authority="geometry_runtime_assigned",
        ))
    tree = FrameTree(timestamp=pose.timestamp, root_frame_id=pose.frame_id, transforms=transforms)
    return _AssignedNativeLocalizationSample(
        provider, origin, delivery_token, terminal, response.pose.timestamp_ns,
        response.correction_revision, response.map_frame_generation, pose, tree,
    )



class GeometryRuntimeMapActivationStaleRequest(RuntimeError):
    """A terminal mapping session ID cannot be reused for another activation."""

    backend_activation_stale_request = True
    backend_switch_applied = False
    backend_activation_disposition = "stale_request"

    def __init__(
        self,
        *,
        activation_request_id: str,
        map_name: str,
        current_phase: str,
        current_map_name: str,
    ) -> None:
        self.activation_request_id = str(activation_request_id)
        self.map_name = str(map_name)
        self.current_phase = str(current_phase)
        self.current_map_name = str(current_map_name)
        super().__init__(
            "mapping activation request ID is stale: request_id=%s "
            "target_map=%s current_phase=%s current_map=%s"
            % (
                self.activation_request_id,
                self.map_name,
                self.current_phase or "<empty>",
                self.current_map_name or "<empty>",
            )
        )


def _freeze_checkpoint_value(value: object) -> object:
    if isinstance(value, Mapping):
        return MappingProxyType(
            {
                str(key): _freeze_checkpoint_value(item)
                for key, item in value.items()
            }
        )
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_checkpoint_value(item) for item in value)
    return value


@dataclass(frozen=True)
class MappingBackendCheckpointExportBundle:
    """Immutable backend state plus the correlated canonical map assets."""

    checkpoint: MappingBackendCheckpoint
    occupancy_asset: Mapping[str, object]
    cloud_asset: Optional[Mapping[str, object]]

    @property
    def backend_kind(self) -> str:
        return self.checkpoint.backend_kind

    def canonical_occupancy_checkpoint(self):
        return _canonical_occupancy_from_stored(self.checkpoint, self.occupancy_asset)

    def canonical_cloud_checkpoint(self):
        if self.checkpoint.backend_kind == "slam_toolbox":
            return None
        return _canonical_cloud_from_stored(self.checkpoint, self.cloud_asset)

    def publish_to(
        self,
        map_bundle_root: Path,
    ) -> StoredMappingBackendCheckpoint:
        """Persist backend payload and metadata under one finalized map bundle."""

        return MappingBackendCheckpointStore(map_bundle_root).publish(
            self.checkpoint,
            occupancy_asset=self.occupancy_asset,
            cloud_asset=self.cloud_asset,
        )





def _require_proto_message_field(message, field_name: str):
    try:
        present = bool(message.HasField(field_name))
    except (AttributeError, ValueError) as exc:
        raise ValueError(
            "mapping backend checkpoint protobuf lacks field %s" % field_name
        ) from exc
    if not present:
        raise ValueError(
            "mapping backend checkpoint requires field %s" % field_name
        )
    return getattr(message, field_name)


def _mapping_commit_identity_dict(message) -> Dict[str, object]:
    return {
        "boot_nonce": bytes(message.boot_nonce),
        "sequence": int(message.sequence),
    }


def _occupancy_receipt_dict(message) -> Dict[str, object]:
    commit_id = _require_proto_message_field(message, "commit_id")
    return {
        "commit_id": _mapping_commit_identity_dict(commit_id),
        "session_id": str(message.session_id),
        "session_generation": int(
            message.session_generation
        ),
        "occupancy_revision": int(
            message.occupancy_revision
        ),
        "source_timestamp_ns": int(
            message.source_timestamp_ns
        ),
        "backend_kind": str(message.backend_kind),
        "backend_generation": int(
            message.backend_generation
        ),
    }


def _cloud_receipt_dict(message) -> Dict[str, object]:
    commit_id = _require_proto_message_field(message, "commit_id")
    return {
        "commit_id": _mapping_commit_identity_dict(commit_id),
        "session_generation": int(
            message.session_generation
        ),
        "occupancy_revision": int(
            message.occupancy_revision
        ),
        "cloud_revision": int(message.cloud_revision),
        "source_timestamp_ns": int(
            message.source_timestamp_ns
        ),
        "point_count": int(message.point_count),
    }


def _validate_checkpoint_identity_envelope(response):
    if response.schema_version != 2:
        raise ValueError("检查点传输只接收schema 2")
    if response.backend_kind == "slam_toolbox":
        _require_proto_message_field(response, "occupancy_identity")
        reference = _artifact_reference_from_proto(_require_proto_message_field(response, "graph_artifact"))
        if (response.payload or response.backend_payload_schema_version != 2
                or response.target_kind != "planar_graph" or reference.identity_schema_version != 2
                or reference.member_identity.role != "planar_graph"
                or reference.member_identity.checkpoint_id != response.checkpoint_id
                or any(response.HasField(name) for name in ("backend_identity", "cloud_receipt", "canonical_cloud_checkpoint", "fine_map_artifact", "dense_cloud_artifact"))):
            raise ValueError("assigned planar checkpoint envelope mismatch")
        return
    _require_proto_message_field(response, "backend_identity")
    _require_proto_message_field(response, "occupancy_identity")
    cloud = _require_proto_message_field(response, "canonical_cloud_checkpoint")
    _require_proto_message_field(cloud, "member_identity")
    return


def _checkpoint_export_bundle_from_proto(response, payload=None, fine_payload=None, dense_payload=None):
    _validate_checkpoint_identity_envelope(response)
    return _checkpoint_export_bundle_v2(response, payload, fine_payload, dense_payload)


def _mapping_commit_identity_proto(value: Mapping[str, object]):
    return pb2.MappingCommitIdentity(
        boot_nonce=bytes(value.get("boot_nonce") or b""),
        sequence=int(value.get("sequence") or 0),
    )




_CHECKPOINT_MEMBER_ROLE_NAMES = {
    1: "backend_state", 2: "canonical_occupancy", 3: "canonical_cloud",
    4: "fine_map", 5: "dense_cloud", 6: "planar_graph",
}


def _member_identity_from_proto(message):
    role = _CHECKPOINT_MEMBER_ROLE_NAMES.get(int(message.role))
    if role is None:
        raise ValueError("checkpoint member role is unspecified or unsupported")
    return CheckpointMemberIdentity(str(message.checkpoint_id), role,
        _occupancy_receipt_dict(_require_proto_message_field(message, "occupancy_receipt")),
        _cloud_receipt_dict(message.cloud_receipt) if message.HasField("cloud_receipt") else None,
        str(message.frame_id), int(message.frame_generation), int(message.producer_generation))


def _artifact_reference_from_proto(reference, maximum_bytes=MAX_PLANAR_ARCHIVE_BYTES):
    if reference.identity_schema_version != 2:
        raise ValueError("制品传输只接收分配式身份")
    descriptor = _member_identity_from_proto(reference.member_identity) if reference.HasField("member_identity") else None
    return validate_artifact_reference(CheckpointArtifactReference(str(reference.id), int(reference.size),
        int(reference.identity_schema_version), descriptor), maximum_bytes=maximum_bytes)


def _occupancy_receipt_proto(receipt):
    return pb2.CanonicalOccupancyCommitReceipt(commit_id=_mapping_commit_identity_proto(receipt["commit_id"]),
        **{key: value for key, value in receipt.items() if key != "commit_id"})


def _cloud_receipt_proto(receipt):
    return pb2.CanonicalCloudCommitReceipt(commit_id=_mapping_commit_identity_proto(receipt["commit_id"]),
        **{key: value for key, value in receipt.items() if key != "commit_id"})


def _member_identity_proto(identity):
    return pb2.CheckpointMemberIdentity(checkpoint_id=identity.checkpoint_id,
        role=next(number for number, name in _CHECKPOINT_MEMBER_ROLE_NAMES.items() if name == identity.role),
        occupancy_receipt=_occupancy_receipt_proto(identity.occupancy_receipt),
        cloud_receipt=_cloud_receipt_proto(identity.cloud_receipt) if identity.cloud_receipt is not None else None,
        frame_id=identity.frame_id, frame_generation=identity.frame_generation, producer_generation=identity.producer_generation)


def _artifact_reference_proto(reference):
    return pb2.GraphArtifactRef(id=reference.id, size=reference.size, identity_schema_version=reference.identity_schema_version,
        member_identity=_member_identity_proto(reference.member_identity) if reference.member_identity is not None else None)


def _canonical_occupancy_from_stored(checkpoint, asset):
    return pb2.OccupancyGridData(meta=pb2.OccupancyGridMeta(width=asset["width"], height=asset["height"],
        resolution=asset["resolution"], origin_x=asset["origin_x"], origin_y=asset["origin_y"], frame_id=asset["frame_id"],
        revision=asset["source_occupancy_revision"], timestamp_ns=asset["timestamp_ns"]),
        data=_protobuf_payload_bytes(checkpoint.canonical_occupancy_data))


def _canonical_cloud_from_stored(checkpoint, asset):
    return pb2.CloudMapCheckpoint(frame_id=asset["frame_id"], source_revision=asset["source_cloud_revision"],
        point_count=asset["point_count"], voxel_size=asset["voxel_size"], payload_schema_version=2,
        xyz_f32_le=_protobuf_payload_bytes(checkpoint.canonical_cloud_xyz), member_identity=_member_identity_proto(checkpoint.cloud_identity))


def _protobuf_payload_bytes(value):
    # Materialize only when a caller requests a protobuf bytes field.
    return value if isinstance(value, bytes) else value[:]


def _checkpoint_export_bundle_v2(response, payload, fine_payload, dense_payload):
    if response.backend_kind == "slam_toolbox":
        return _planar_checkpoint_export_bundle_v2(response, payload, fine_payload, dense_payload)
    if response.HasField("graph_artifact") or payload is not None:
        raise ValueError("cloud checkpoint cannot carry graph artifact")
    if response.target_kind != "":
        raise ValueError("v2 cloud checkpoint target kind mismatch")
    if response.backend_payload_schema_version != 1:
        raise ValueError("v2 checkpoint has mixed identity or unsupported backend bytes")
    occupancy = _require_proto_message_field(response, "canonical_occupancy_checkpoint")
    meta = _require_proto_message_field(occupancy, "meta")
    cloud = _require_proto_message_field(response, "canonical_cloud_checkpoint")
    if cloud.payload_schema_version != 2:
        raise ValueError("v2 cloud checkpoint requires assigned identity")
    fine_reference = _artifact_reference_from_proto(response.fine_map_artifact, MAX_FINE_ARCHIVE_BYTES) if response.HasField("fine_map_artifact") else None
    dense_reference = _artifact_reference_from_proto(response.dense_cloud_artifact, MAX_DENSE_ARCHIVE_BYTES) if response.HasField("dense_cloud_artifact") else None
    checkpoint = MappingBackendCheckpoint(schema_version=2, backend_kind=response.backend_kind,
        source_session_id=response.source_session_id, source_session_generation=response.source_session_generation,
        content_sha256=b"", payload_size=response.payload_size, created_at_ns=response.created_at_ns,
        payload=bytes(response.payload), occupancy_receipt=_occupancy_receipt_dict(_require_proto_message_field(response, "occupancy_receipt")),
        cloud_receipt=_cloud_receipt_dict(_require_proto_message_field(response, "cloud_receipt")),
        fine_archive=fine_payload, dense_archive=dense_payload, checkpoint_id=response.checkpoint_id, target_kind=response.target_kind,
        backend_payload_schema_version=response.backend_payload_schema_version, canonical_occupancy_data=bytes(occupancy.data),
        canonical_cloud_xyz=bytes(cloud.xyz_f32_le), fine_reference=fine_reference, dense_reference=dense_reference,
        cloud_identity=_member_identity_from_proto(_require_proto_message_field(cloud, "member_identity")),
        backend_identity=_member_identity_from_proto(_require_proto_message_field(response, "backend_identity")),
        occupancy_identity=_member_identity_from_proto(_require_proto_message_field(response, "occupancy_identity")))
    occupancy_asset = dict(raw_data_size=len(checkpoint.canonical_occupancy_data), width=int(meta.width), height=int(meta.height),
        resolution=float(meta.resolution), origin_x=float(meta.origin_x), origin_y=float(meta.origin_y), frame_id=str(meta.frame_id),
        source_occupancy_revision=int(meta.revision), timestamp_ns=int(meta.timestamp_ns))
    cloud_asset = dict(xyz_size=len(checkpoint.canonical_cloud_xyz), frame_id=str(cloud.frame_id), source_cloud_revision=int(cloud.source_revision),
        point_count=int(cloud.point_count), voxel_size=float(cloud.voxel_size), payload_schema_version=2)
    checkpoint, occupancy_asset, cloud_asset = MappingBackendCheckpointStore.validate_checkpoint_bundle(checkpoint,
        occupancy_asset=occupancy_asset, cloud_asset=cloud_asset)
    return MappingBackendCheckpointExportBundle(checkpoint, occupancy_asset, cloud_asset)


def _planar_checkpoint_export_bundle_v2(response, payload, fine_payload=None, dense_payload=None):
    if fine_payload is not None or dense_payload is not None or not isinstance(payload, FrozenPayload):
        raise ValueError("planar checkpoint requires one retained graph FrozenPayload")
    occupancy = _require_proto_message_field(response, "canonical_occupancy_checkpoint")
    meta = _require_proto_message_field(occupancy, "meta")
    checkpoint = MappingBackendCheckpoint(schema_version=2, backend_kind="slam_toolbox",
        source_session_id=response.source_session_id, source_session_generation=response.source_session_generation,
        content_sha256=b"", payload_size=response.payload_size, created_at_ns=response.created_at_ns,
        payload=payload, occupancy_receipt=_occupancy_receipt_dict(_require_proto_message_field(response, "occupancy_receipt")),
        cloud_receipt=None, checkpoint_id=response.checkpoint_id, target_kind="planar_graph",
        backend_payload_schema_version=2, canonical_occupancy_data=bytes(occupancy.data),
        graph_reference=_artifact_reference_from_proto(response.graph_artifact),
        occupancy_identity=_member_identity_from_proto(response.occupancy_identity))
    occupancy_asset = dict(raw_data_size=len(checkpoint.canonical_occupancy_data), width=int(meta.width), height=int(meta.height),
        resolution=float(meta.resolution), origin_x=float(meta.origin_x), origin_y=float(meta.origin_y), frame_id=str(meta.frame_id),
        source_occupancy_revision=int(meta.revision), timestamp_ns=int(meta.timestamp_ns))
    checkpoint, occupancy_asset, cloud_asset = MappingBackendCheckpointStore.validate_checkpoint_bundle(checkpoint,
        occupancy_asset=occupancy_asset, cloud_asset=None)
    return MappingBackendCheckpointExportBundle(checkpoint, occupancy_asset, cloud_asset)



def reopen_mapping_backend_checkpoint(
    map_bundle_root, backend_kind, *, schema_version=2, expected_checkpoint_id=None,
):
    """Reopen exact v2 export bytes; legacy canonical assets were never persisted."""
    if schema_version != 2:
        raise ValueError("exact canonical reopen requires checkpoint schema 2")
    stored = MappingBackendCheckpointStore(map_bundle_root).load(
        backend_kind, schema_version=2, expected_checkpoint_id=expected_checkpoint_id,
    )
    return MappingBackendCheckpointExportBundle(stored.checkpoint, stored.occupancy_asset, stored.cloud_asset)


def _mapping_backend_checkpoint_v2_proto(stored, *, graph_artifact=None, fine_map_artifact=None, dense_cloud_artifact=None):
    checkpoint = stored.checkpoint
    if graph_artifact is not None and checkpoint.backend_kind != "slam_toolbox":
        raise ValueError("cloud checkpoint cannot carry planar graph artifact")
    for supplied, retained, maximum in ((graph_artifact, checkpoint.graph_reference, MAX_PLANAR_ARCHIVE_BYTES),
                                       (fine_map_artifact, checkpoint.fine_reference, MAX_FINE_ARCHIVE_BYTES),
                                       (dense_cloud_artifact, checkpoint.dense_reference, MAX_DENSE_ARCHIVE_BYTES)):
        if supplied is not None and _artifact_reference_from_proto(supplied, maximum) != retained:
            raise ValueError("reupload must preserve the retained artifact identity")
    message = pb2.MappingBackendCheckpoint()
    _populate_mapping_backend_checkpoint_v2(message, stored)
    return message


def _populate_mapping_backend_checkpoint_v2(message, stored):
    """Fill a caller-owned protobuf submessage without a second full-body CopyFrom."""
    checkpoint = stored.checkpoint
    message.schema_version = 2
    for field in ("backend_kind", "source_session_id", "source_session_generation", "checkpoint_id",
                  "backend_payload_schema_version", "payload_size", "created_at_ns", "target_kind"):
        setattr(message, field, getattr(checkpoint, field))
    planar = checkpoint.backend_kind == "slam_toolbox"
    if planar:
        message.graph_artifact.CopyFrom(_artifact_reference_proto(checkpoint.graph_reference))
    else:
        message.payload = _protobuf_payload_bytes(checkpoint.payload)
        message.cloud_receipt.CopyFrom(_cloud_receipt_proto(checkpoint.cloud_receipt))
        message.backend_identity.CopyFrom(_member_identity_proto(checkpoint.backend_identity))
    message.occupancy_receipt.CopyFrom(_occupancy_receipt_proto(checkpoint.occupancy_receipt))
    message.occupancy_identity.CopyFrom(_member_identity_proto(checkpoint.occupancy_identity))
    occupancy = stored.occupancy_asset
    meta = message.canonical_occupancy_checkpoint.meta
    for field in ("width", "height", "resolution", "origin_x", "origin_y", "frame_id", "timestamp_ns"):
        setattr(meta, field, occupancy[field])
    meta.revision = occupancy["source_occupancy_revision"]
    message.canonical_occupancy_checkpoint.data = _protobuf_payload_bytes(checkpoint.canonical_occupancy_data)
    if planar:
        return
    cloud = stored.cloud_asset
    target = message.canonical_cloud_checkpoint
    for field in ("frame_id", "point_count", "voxel_size"):
        setattr(target, field, cloud[field])
    target.source_revision = cloud["source_cloud_revision"]
    target.payload_schema_version = 2
    target.member_identity.CopyFrom(_member_identity_proto(checkpoint.cloud_identity))
    target.xyz_f32_le = _protobuf_payload_bytes(checkpoint.canonical_cloud_xyz)
    if checkpoint.fine_reference is not None:
        message.fine_map_artifact.CopyFrom(_artifact_reference_proto(checkpoint.fine_reference))
    if checkpoint.dense_reference is not None:
        message.dense_cloud_artifact.CopyFrom(_artifact_reference_proto(checkpoint.dense_reference))


def _assigned_activation_request_proto(stored, *, asset_identity, session_id, map_name,
                                       activation_request_id, expected_runtime_generation):
    """Construct explicit v2 wire ownership without enabling default callers."""
    from contracts.map_identity import AssetIdentityV2
    if not isinstance(asset_identity, AssetIdentityV2):
        raise ValueError("activation requires an admitted typed asset identity")
    asset_identity = AssetIdentityV2(asset_id=asset_identity.asset_id)
    if (type(expected_runtime_generation) is not int
            or not 1 <= expected_runtime_generation <= (1 << 64) - 1):
        raise ValueError("activation runtime generation must be a positive u64")
    alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._:-"
    for field, value in (("session_id", session_id), ("activation_request_id", activation_request_id)):
        if not isinstance(value, str) or not 1 <= len(value) <= 128 or any(c not in alphabet for c in value):
            raise ValueError(f"{field} must be canonical ASCII [A-Za-z0-9._:-]")
    if (not isinstance(map_name, str) or not map_name or map_name != map_name.strip()
            or len(map_name.encode("utf-8")) > 256
            or any(ord(character) < 32 or 127 <= ord(character) <= 159 for character in map_name)):
        raise ValueError("map_name must be canonical nonempty text within 256 bytes")
    if stored.checkpoint.schema_version != 2:
        raise ValueError("activation requires admitted assigned checkpoint ownership")
    message = pb2.ActivateMapRequest(session_id=session_id, map_name=map_name,
        activation_request_id=activation_request_id, expected_runtime_generation=expected_runtime_generation,
        activation_contract_version=2, asset_identity=pb2.AssetIdentityV2(asset_id=asset_identity.asset_id))
    _populate_mapping_backend_checkpoint_v2(message.mapping_backend_checkpoint, stored)
    return message


def _assigned_activation_ack(payload, *, activation_request):
    """使用已解码回执确认本次请求的原生激活结果。"""
    if not payload["success"] or payload["phase"] != "ACTIVE":
        raise ValueError("ActivateMap did not acknowledge a sealed assigned installation")
    receipt = payload["activation_receipt_v2"]
    if not isinstance(receipt, AssignedActivationReceipt):
        raise ValueError("ActivateMap omitted its complete assigned target receipt")
    if receipt.to_dict()["request"] != activation_request:
        raise ValueError("ActivateMap receipt differs from the retained request")
    return payload



_ASSIGNED_PREDECESSOR_FIELDS = (
    "session_id", "map_name", "session_generation", "runtime_generation",
    "mapping_backend_kind",
)


def _assigned_mapping_predecessor(value):
    """Retain only the stable native owner descriptor, never mutable revisions."""
    if not isinstance(value, Mapping):
        raise ValueError("assigned predecessor requires mapping state")
    result = {name: value.get(name) for name in _ASSIGNED_PREDECESSOR_FIELDS}
    if (any(not isinstance(result[name], str) or not result[name]
            for name in ("session_id", "map_name"))
            or any(type(result[name]) is not int or result[name] <= 0
                   for name in ("session_generation", "runtime_generation"))
            or result["mapping_backend_kind"] not in _SUPPORTED_MAPPING_BACKEND_KINDS):
        raise ValueError("assigned predecessor descriptor is incomplete")
    return result























def _canonical_relocalization_request_id(
    value: Optional[str], *, generate_if_missing: bool
) -> str:
    if value is None:
        if not generate_if_missing:
            raise ValueError("request_id is required")
        resolved = "relocalize:%s" % uuid.uuid4().hex
    elif isinstance(value, str):
        resolved = value
    else:
        raise ValueError("request_id must be canonical ASCII [A-Za-z0-9._:-]")
    if (
        not resolved
        or len(resolved) > 128
        or any(
            not (character.isascii() and (character.isalnum() or character in "._:-"))
            for character in resolved
        )
    ):
        raise ValueError("request_id must be canonical ASCII [A-Za-z0-9._:-]")
    return resolved



def _planar_activation_target_payload(message):
    """Project only the planar activation branch; cloud relocalization stays separate."""
    if message.DESCRIPTOR.full_name != "nuwax.geometry.InstalledTargetReceiptV2":
        raise ValueError("assigned target protobuf type differs")
    source_field = message.DESCRIPTOR.fields_by_name.get("source_graph_member")
    if source_field is None or source_field.number != 5 or source_field.type != 11:
        raise ValueError("planar target source field differs from paired protocol")
    source = _member_identity_from_proto(_require_proto_message_field(message, "source_graph_member"))
    if (source.role != "planar_graph" or source.cloud_receipt is not None
            or message.HasField("source_cloud_member") or message.HasField("cloud_receipt")):
        raise ValueError("planar activation target source differs from its backend")
    return {
        "activation": _project_assigned_proto(_require_proto_message_field(message, "activation"), "activation"),
        "source_graph_member": _identity_document(source),
        "occupancy_receipt": _project_assigned_proto(_require_proto_message_field(message, "occupancy_receipt"), "occupancy"),
    }


def _assigned_activation_receipt_payload(message):
    """解码完整回执一次；未完成状态只保留实际存在的字段。"""
    if message.DESCRIPTOR.full_name != "nuwax.geometry.ActivationReceiptV2":
        raise ValueError("assigned activation receipt protobuf type differs")
    request = _project_assigned_proto(_require_proto_message_field(message, "request"), "activation_request")
    activation = (_project_assigned_proto(message.activation, "activation")
                  if message.HasField("activation") else None)
    if message.HasField("target"):
        if activation is None:
            raise ValueError("assigned target omitted its installed activation")
        target = (_planar_activation_target_payload(message.target)
                  if activation["backend_kind"] == "slam_toolbox"
                  else _project_assigned_proto(message.target, "target"))
        return AssignedActivationReceipt.admit({
            "request": request, "activation": activation, "target": target,
        })
    return {"request": request, "activation": activation, "target": None}



def _assigned_activation_receipt_proto(receipt: AssignedActivationReceipt):
    """Project the admitted original metadata without constructing new identities."""
    payload = receipt.to_dict()
    target = payload["target"]
    if payload["activation"]["backend_kind"] == "slam_toolbox":
        source = target["source_graph_member"]
        source["occupancy_receipt"] = _assigned_proto_fields(source["occupancy_receipt"], "occupancy")
        target_fields = pb2.InstalledTargetReceiptV2(
            activation=_assigned_proto_fields(target["activation"], "activation"),
            source_graph_member=_member_identity_proto(CheckpointMemberIdentity(**source)),
            occupancy_receipt=_assigned_proto_fields(target["occupancy_receipt"], "occupancy"),
        )
    else:
        target_fields = _assigned_proto_fields(target, "target")
    return pb2.ActivationReceiptV2(
        request=_assigned_proto_fields(payload["request"], "activation_request"),
        activation=_assigned_proto_fields(payload["activation"], "activation"),
        target=target_fields,
    )




def _unactivated_mapping_state_payload(response, disposition: int, disposition_name: str) -> Dict[str, Any]:
    """No activation record is not an idle mapping session or an installed target."""
    if response.DESCRIPTOR.full_name != "nuwax.geometry.MappingSessionState":
        raise ValueError("unactivated mapping state protobuf type differs")
    if (disposition not in {pb2.ACTIVATION_RECEIPT_DISPOSITION_UNSPECIFIED, pb2.ACTIVATION_RECEIPT_DISPOSITION_RETIRED}
            or response.HasField("activation_receipt_v2")
            or response.relocalization_target_available or response.activation_request_id
            or response.backend_generation or response.target_kind):
        raise ValueError("schema-0 mapping state contains activation/target claims")
    policy = None
    if response.HasField("relocalization_policy_v2"):
        policy = _project_assigned_proto(response.relocalization_policy_v2, "policy")
    fields = (
        "success", "phase", "message", "session_id", "map_name", "session_generation",
        "revision", "active", "idempotent_replay",
        "mapping_backend_kind", "runtime_generation", "planar_mode",
    )
    payload = {name: getattr(response, name) for name in fields}
    payload.update(activation_receipt_schema_version=0,
        activation_receipt_disposition=disposition,
        activation_receipt_disposition_name=disposition_name,
        activation_receipt_v2=None, relocalization_policy_v2=policy,
        relocalization_target_available=False, backend_generation=0, activation_request_id="")
    return payload


def _assigned_mapping_state_payload(response, disposition: int, disposition_name: str) -> Dict[str, Any]:
    if response.DESCRIPTOR.full_name != "nuwax.geometry.MappingSessionState":
        raise ValueError("assigned mapping state protobuf type differs")
    receipt = _assigned_activation_receipt_payload(
        _require_proto_message_field(response, "activation_receipt_v2"))
    if disposition in {2, 3} and not isinstance(receipt, AssignedActivationReceipt):
        raise ValueError("sealed assigned activation omitted its complete target receipt")
    policy = (_project_assigned_proto(response.relocalization_policy_v2, "policy")
              if response.HasField("relocalization_policy_v2") else None)
    fields = (
        "success", "phase", "message", "session_id", "map_name", "session_generation",
        "revision", "active", "idempotent_replay",
        "mapping_backend_kind", "runtime_generation", "backend_generation", "activation_request_id",
        "relocalization_target_available",
    )
    result = {name: getattr(response, name) for name in fields}
    result.update(activation_receipt_schema_version=2,
        activation_receipt_disposition=disposition,
        activation_receipt_disposition_name=disposition_name,
        activation_receipt_v2=receipt, relocalization_policy_v2=policy)
    if response.mapping_backend_kind == "slam_toolbox":
        result["planar_mode"] = response.planar_mode
        result["target_kind"] = response.target_kind
    return result



def _mapping_state_document(value: Mapping[str, object]) -> Dict[str, object]:
    """在公开元数据或持久写入处投影一次普通字典。"""
    result = dict(value)
    receipt = result.get("activation_receipt_v2")
    if isinstance(receipt, AssignedActivationReceipt):
        result["activation_receipt_v2"] = receipt.to_dict()
    return result


def _assigned_request_for_rpc(
    *, activation_state: Mapping[str, object], assigned_request_v2,
    request_id: Optional[str], logical_timeout_ms: int, query_only: bool, planar_seed=None,
) -> AssignedRelocalizationRequest:
    """查询必须使用持久化精确请求；不从旧 SHA 或缺失别名重建身份。"""
    if not query_only and activation_state.get("activation_receipt_schema_version") != 2:
        raise ValueError("unsupported activation schema for assigned relocalization")
    if assigned_request_v2 is not None:
        request = (
            assigned_request_v2 if isinstance(assigned_request_v2, AssignedRelocalizationRequest)
            else AssignedRelocalizationRequest.from_dict(assigned_request_v2)
        )
        if request_id is not None and request_id != request.request_id:
            raise ValueError("assigned request alias differs from durable request")
        if type(logical_timeout_ms) is not int or logical_timeout_ms != request.logical_timeout_ms:
            raise ValueError("assigned timeout differs from normalized durable request")
    elif query_only:
        raise ValueError("assigned terminal query requires the whole normalized durable request")
    else:
        receipt = activation_state["activation_receipt_v2"]
        policy = activation_state.get("relocalization_policy_v2")
        if policy is None:
            raise ValueError("assigned activation omitted relocalization_policy_v2")
        planar = receipt.to_dict()["activation"]["backend_kind"] == "slam_toolbox"
        body_key = "assigned_planar_v2" if planar else "assigned_cloud_v2"
        body = {"target": receipt.to_dict()["target"], "policy": policy,
                "logical_timeout_ms": logical_timeout_ms}
        if planar:
            if not isinstance(planar_seed, (list, tuple)) or len(planar_seed) != 3:
                raise ValueError("assigned planar request requires its three-value seed")
            body["seed"] = dict(zip(("x", "y", "yaw"), planar_seed))
        request = AssignedRelocalizationRequest.admit({"schema_version": 2,
            "request_id": _canonical_relocalization_request_id(request_id, generate_if_missing=True),
            body_key: body})
    planar = request.backend_kind == "slam_toolbox"
    if planar_seed is not None:
        if not planar or not isinstance(planar_seed, (list, tuple)) or len(planar_seed) != 3:
            raise ValueError("planar seed differs from the assigned backend")
        replay = request.to_dict()
        replay[request.body_key]["seed"] = dict(zip(("x", "y", "yaw"), planar_seed))
        if not request.exact_replay_of(AssignedRelocalizationRequest.admit_retained(replay)):
            raise ValueError("assigned planar retry changed its seed")
    if not query_only:
        receipt = activation_state["activation_receipt_v2"]
        if not request.activation_receipt.exact_replay_of(receipt):
            raise ValueError("assigned request target differs from activation receipt")
        policy = activation_state.get("relocalization_policy_v2")
        if policy is None:
            raise ValueError("assigned activation omitted relocalization_policy_v2")
        # 持久化身份可读不代表旧策略可重新执行；只比较已有策略元数据。
        if policy != request.to_dict()[request.body_key]["policy"]:
            raise ValueError("assigned request policy differs from activation state")
        if activation_state.get("activation_receipt_disposition") not in {2, 3}:
            raise ValueError("assigned relocalization requires a sealed activation")
        if activation_state.get("relocalization_target_available") is not True:
            raise RuntimeError("当前激活地图没有可用的重定位目标回执。")
    return request


def _assigned_terminal_payload(terminal: AssignedRelocalizationTerminal) -> Dict[str, Any]:
    payload = terminal.to_dict()
    body = payload[terminal.request.body_key]
    disposition_name = pb2.RelocalizeDisposition.Name(terminal.disposition)
    prefix = "RELOCALIZE_DISPOSITION_"
    if disposition_name.startswith(prefix):
        disposition_name = disposition_name[len(prefix):]
    payload.update(
        success=terminal.disposition in _RELOCALIZE_SUCCESS_DISPOSITIONS,
        disposition=terminal.disposition,
        disposition_name=disposition_name.lower(),
        correction_revision=terminal.correction_revision,
        detail=body["detail"],
    )
    return payload
















































class _NavigationNotificationTicket:
    """One immutable navigation event plus its commit-time delivery fence."""

    __slots__ = (
        "ticket_id",
        "state",
        "delivery_token",
        "subscribers",
        "cancelled",
    )

    def __init__(
        self,
        *,
        ticket_id: int,
        state: NavigationState,
        delivery_token: _NavigationDeliveryToken,
        subscribers: Tuple[
            Tuple[int, Callable[[NavigationState], None]],
            ...,
        ],
    ) -> None:
        self.ticket_id = int(ticket_id)
        self.state = state.model_copy(deep=True)
        self.delivery_token = delivery_token
        self.subscribers = subscribers
        self.cancelled = False


def _cancel_stream_call(call, *, context: str) -> bool:
    cancel = getattr(call, "cancel", None)
    if not callable(cancel):
        return True
    try:
        cancel()
        return True
    except Exception:
        LOGGER.exception("取消 %s stream call 失败。", context)
        return False


def _latest_pose_updates(call, stop_event):
    """持续排空远程位姿，只向回调线程保留最新一项；订阅拥有并回收读线程。"""
    condition = threading.Condition()
    latest = None
    finished = False
    failure = None

    def receive():
        nonlocal latest, finished, failure
        try:
            for update in call:
                with condition:
                    if stop_event.is_set():
                        break
                    latest = update
                    condition.notify()
        except Exception as exc:
            with condition:
                failure = exc
        finally:
            with condition:
                finished = True
                condition.notify()

    reader = threading.Thread(
        target=receive, name="geometry_runtime_pose_receive", daemon=True
    )
    reader.start()
    try:
        while not stop_event.is_set():
            with condition:
                condition.wait_for(
                    lambda: latest is not None or finished or stop_event.is_set(),
                    timeout=0.1,
                )
                if stop_event.is_set():
                    return
                update, latest = latest, None
                if update is None:
                    if failure is not None:
                        raise failure
                    if finished:
                        return
                    continue
            yield update
    finally:
        # 回调线程退出前先停止真实读取；未回收时原订阅句柄仍保持存活。
        _cancel_stream_call(call, context="位姿接收")
        reader.join()


def _coalesced_navigation_updates(call):
    """接收不等待订阅回调；合并同一目标的连续进度，保留终态与身份边界。"""
    condition = threading.Condition()
    pending = deque()
    stopping = threading.Event()
    finished = False
    failure = None

    def merge_key(update):
        status = str(update.status).strip().lower()
        return (
            _navigation_lease_from_message(update),
            int(update.planner_runtime_generation),
            int(update.runtime_generation),
            status if status in _NAV_TERMINAL_STATUS or status not in _NAV_STREAM_STATUS else "progress",
            str(update.cancel_phase),
            int(update.cancel_generation),
            bool(update.actuator_zeroed),
            str(update.zero_ack_id),
            int(update.zero_applied_monotonic_ns),
        )

    def receive():
        nonlocal finished, failure
        try:
            for update in call:
                key = merge_key(update)
                with condition:
                    if stopping.is_set():
                        return
                    if pending and pending[-1][0] == key:
                        previous = pending[-1][1]
                        if (update.timestamp_ns >= previous.timestamp_ns
                                and update.path_revision >= previous.path_revision):
                            pending[-1] = (key, update)
                            condition.notify()
                            continue
                    # 不丢弃其它目标或停车证据。异常高频身份变化只施加有界背压。
                    condition.wait_for(lambda: len(pending) < 32 or stopping.is_set())
                    if stopping.is_set():
                        return
                    pending.append((key, update))
                    condition.notify()
        except Exception as exc:
            with condition:
                failure = exc
        finally:
            with condition:
                finished = True
                condition.notify()

    reader = threading.Thread(target=receive, name="geometry_runtime_navigation_receive", daemon=True)
    reader.start()
    try:
        while True:
            with condition:
                condition.wait_for(lambda: pending or finished)
                if pending:
                    _, update = pending.popleft()
                    condition.notify()
                elif failure is not None:
                    raise failure
                else:
                    return
            yield update
    finally:
        stopping.set()
        with condition:
            condition.notify_all()
        _cancel_stream_call(call, context="导航状态接收")
        reader.join()


class _CancellableStreamHandle:
    """Own one reconnecting streaming worker and its current gRPC call."""

    def __init__(
        self,
        *,
        name: str,
        worker: Callable[["_CancellableStreamHandle"], None],
        on_finish: Callable[["_CancellableStreamHandle"], None],
    ) -> None:
        self.stop_event = threading.Event()
        self._call_lock = threading.Lock()
        self._current_call = None
        self._worker = worker
        self._on_finish = on_finish
        self._thread = threading.Thread(
            target=self._run,
            name=name,
            daemon=True,
        )

    def start(self) -> "_CancellableStreamHandle":
        self._thread.start()
        return self

    def _run(self) -> None:
        try:
            self._worker(self)
        except Exception:
            LOGGER.exception("geometry stream worker 异常退出: %s", self._thread.name)
        finally:
            with self._call_lock:
                call = self._current_call
            if call is not None:
                _cancel_stream_call(call, context=self._thread.name)
            with self._call_lock:
                if self._current_call is call:
                    self._current_call = None
            self._on_finish(self)

    def bind_call(self, call) -> bool:
        with self._call_lock:
            if not self.stop_event.is_set():
                self._current_call = call
                return True
        _cancel_stream_call(call, context="尚未绑定的 geometry")
        return False

    def clear_call(self, call, *, cancel: bool = False) -> None:
        if cancel:
            _cancel_stream_call(call, context=self._thread.name)
        with self._call_lock:
            if self._current_call is call:
                self._current_call = None

    def cancel(self) -> bool:
        self.stop_event.set()
        with self._call_lock:
            call = self._current_call
        if call is not None:
            _cancel_stream_call(call, context="geometry")
        if self._thread is threading.current_thread():
            return True
        if self._thread.is_alive():
            self._thread.join(timeout=_read_deadline_sec() + 0.5)
        if self._thread.is_alive():
            LOGGER.warning(
                "geometry stream worker 未在取消 deadline 内退出: %s",
                self._thread.name,
            )
            return False
        return True

    def join(self, timeout: Optional[float] = None) -> None:
        self._thread.join(timeout=timeout)

    def is_alive(self) -> bool:
        return self._thread.is_alive()


class _MapUpdatePayload(dict):
    """Dict-compatible payload carrying a non-serialized delivery fence."""

    def __init__(self, *args, delivery_token, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.delivery_token = delivery_token


class _MapDeliveryTicket:
    """One claimed map callback, drained by mapping/connection transitions."""

    __slots__ = (
        "ticket_id",
        "delivery_token",
        "owner_thread_id",
        "invalidated",
    )

    def __init__(
        self,
        *,
        ticket_id: int,
        delivery_token,
        owner_thread_id: int,
    ) -> None:
        self.ticket_id = int(ticket_id)
        self.delivery_token = delivery_token
        self.owner_thread_id = int(owner_thread_id)
        self.invalidated = False


_STATIC_EDGE_SKIP_LOG_INTERVAL_SEC = 30.0
_last_static_edge_skip_log_at = 0.0
_last_static_edge_skip_signature: Optional[str] = None


def _warn_static_edge_skip(message: str, *, signature: str) -> None:
    """对重复的静态外参跳过告警做节流，避免 get_frame_tree 高频刷屏。"""
    global _last_static_edge_skip_log_at, _last_static_edge_skip_signature

    import time

    now = time.monotonic()
    if signature == _last_static_edge_skip_signature and now - _last_static_edge_skip_log_at < _STATIC_EDGE_SKIP_LOG_INTERVAL_SEC:
        return
    _last_static_edge_skip_log_at = now
    _last_static_edge_skip_signature = signature
    LOGGER.warning(message)


def _read_deadline_sec() -> float:
    """unary 轻量读类 gRPC 调用 deadline（秒）。

    下游 Rust runtime 慢或未就绪时，无 deadline 的 unary 调用会无界阻塞，
    单点挂死整条 MCP 请求（参见超时根因 D1）。轻量读类（GetStatus / GetOccupancyGrid 元数据 /
    GetNavigationReadiness / CancelNavigation / SetNavigationGoal / PublishOdom）统一施加该 deadline。

    经 NUWAX_GEOMETRY_GRPC_READ_DEADLINE_SEC 可配；默认 2.0s，下限 0.1s 避免误配为 0。
    """
    try:
        return max(0.1, float(runtime_value("NUWAX_GEOMETRY_GRPC_READ_DEADLINE_SEC", "2.0")))
    except (TypeError, ValueError):
        return 2.0


def _pose_deadline_sec() -> float:
    """unary 位姿/坐标树读类 gRPC 调用 deadline（秒）。

    GetCurrentPose / GetFrameTree 在首帧 map→odom→base_link tf2 链合成偏慢时可能 >2s
    （见 rust-bridge-pose-localizing-bug），用轻量读 deadline 会把"合法但偏慢"的首帧误降级成
    None（伪"无定位"）。故单独给位姿类一档更宽的 deadline，仍有界（不退回无界阻塞）。

    经 NUWAX_GEOMETRY_GRPC_POSE_DEADLINE_SEC 可配；默认 5.0s，下限 0.1s。
    默认值需按实机 p95 复核。
    """
    try:
        return max(0.1, float(runtime_value("NUWAX_GEOMETRY_GRPC_POSE_DEADLINE_SEC", "5.0")))
    except (TypeError, ValueError):
        return 5.0


def _heavy_deadline_sec() -> float:
    """unary 重操作 gRPC 调用 deadline（秒）。

    用于可能传输较大载荷的 unary 调用（如 GetOccupancyGridData 全量栅格字节）。
    经 NUWAX_GEOMETRY_GRPC_HEAVY_DEADLINE_SEC 可配；默认 10.0s，下限 0.5s。
    """
    try:
        return max(0.5, float(runtime_value("NUWAX_GEOMETRY_GRPC_HEAVY_DEADLINE_SEC", "10.0")))
    except (TypeError, ValueError):
        return 10.0


def _grpc_message_limit_bytes() -> int:
    """返回大地图 RPC 的收发上限，至少覆盖 Rust 8192x8192 栅格。"""

    try:
        configured = int(
            runtime_value(
                "NUWAX_GEOMETRY_GRPC_MAX_MESSAGE_BYTES",
                str(_DEFAULT_GRPC_MESSAGE_LIMIT_BYTES),
            )
        )
    except (TypeError, ValueError):
        configured = _DEFAULT_GRPC_MESSAGE_LIMIT_BYTES
    return max(_DEFAULT_GRPC_MESSAGE_LIMIT_BYTES, configured)


def _grpc_stream_retry_sec() -> float:
    try:
        return max(
            0.01,
            float(runtime_value("NUWAX_GEOMETRY_GRPC_STREAM_RETRY_SEC", "1.0")),
        )
    except (TypeError, ValueError):
        return 1.0


def _grpc_stream_retry_max_sec() -> float:
    try:
        return max(
            _grpc_stream_retry_sec(),
            float(
                runtime_value(
                    "NUWAX_GEOMETRY_GRPC_STREAM_RETRY_MAX_SEC",
                    "8.0",
                )
            ),
        )
    except (TypeError, ValueError):
        return 8.0


def _exploration_poll_sec() -> float:
    try:
        return max(
            0.01,
            float(runtime_value("NUWAX_GEOMETRY_EXPLORATION_POLL_SEC", "0.5")),
        )
    except (TypeError, ValueError):
        return 0.5


def _datetime_to_timestamp_ns(value: object) -> int:
    timestamp = getattr(value, "timestamp", None)
    if callable(timestamp):
        try:
            return max(0, int(timestamp() * 1_000_000_000))
        except (OverflowError, OSError, TypeError, ValueError):
            pass
    return time.time_ns()


def _log_rpc_error(rpc_name: str, exc: grpc.RpcError) -> None:
    """统一记录 unary gRPC 失败；区分 DEADLINE_EXCEEDED 以便排障检索。

    DEADLINE_EXCEEDED 表示下游未在 deadline 内回包（本应触发降级而非挂死），
    单列 warning 便于在运行时日志中检索"被 deadline 截断"的调用。
    """
    code = exc.code() if callable(getattr(exc, "code", None)) else None
    if code == grpc.StatusCode.DEADLINE_EXCEEDED:
        LOGGER.warning("gRPC %s 超过 deadline 被截断（DEADLINE_EXCEEDED），按降级返回", rpc_name)
    else:
        LOGGER.debug("gRPC %s 调用失败: %s", rpc_name, exc)


def _exploration_rpc_error(
    rpc_name: str,
    exc: grpc.RpcError,
) -> GeometryRuntimeRpcError:
    error = GeometryRuntimeRpcError.from_rpc_error(rpc_name, exc)
    try:
        _log_rpc_error(rpc_name, exc)
    except Exception:
        LOGGER.debug(
            "gRPC %s 调用失败，且读取原始错误元数据失败: %s",
            rpc_name,
            error.grpc_details,
            exc_info=True,
        )
    return error


def _missing_exploration_stub_error(rpc_name: str) -> GeometryRuntimeRpcError:
    details = "geometry runtime gRPC stub is unavailable"
    LOGGER.debug("gRPC %s 调用失败: %s", rpc_name, details)
    return GeometryRuntimeRpcError(
        rpc_name=rpc_name,
        grpc_code=grpc.StatusCode.UNAVAILABLE,
        grpc_details=details,
    )


def _exploration_rpc_timeout(
    deadline_monotonic: Optional[float],
    rpc_name: str,
) -> float:
    # 续建会恢复完整图，使用重操作预算；任务显式截止时间仍取更早者。
    timeout = _heavy_deadline_sec() if rpc_name == "StartMapping" else _read_deadline_sec()
    if deadline_monotonic is None:
        return timeout
    remaining = float(deadline_monotonic) - time.monotonic()
    if remaining <= 0:
        error = GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="cooperative exploration deadline exhausted",
        )
        error.exploration_rpc_dispatched = False
        raise error
    return min(timeout, remaining)


@contextmanager
def _cooperative_deadline_lock(lock, deadline_monotonic, rpc_name):
    if deadline_monotonic is None:
        with lock:
            yield
        return
    timeout = _exploration_rpc_timeout(deadline_monotonic, rpc_name)
    if not lock.acquire(timeout=timeout):
        error = GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="cooperative lock deadline exhausted",
        )
        error.exploration_rpc_dispatched = False
        raise error
    try:
        yield
    finally:
        lock.release()


@contextmanager
def _cooperative_post_rpc_lock(lock, deadline_monotonic, rpc_name):
    if deadline_monotonic is None:
        with lock:
            yield
        return
    remaining = float(deadline_monotonic) - time.monotonic()
    acquired = (
        lock.acquire(blocking=False)
        if remaining <= 0
        else lock.acquire(timeout=remaining)
    )
    if not acquired:
        raise GeometryRuntimeRpcError(
            rpc_name=rpc_name,
            grpc_code=grpc.StatusCode.DEADLINE_EXCEEDED,
            grpc_details="post-RPC cooperative lock deadline exhausted",
        )
    try:
        yield
    finally:
        lock.release()


# --- NavigationState.status 字符串语义映射（round-4 HIGH 3 同步） ---
# Rust proto SetNavigationGoal / CancelNavigation / SubscribeNavigationState
# 全部返回/推送 NavigationState，没有 Rust 端 NavigationAck.accepted / CancelResult.success
# 之类的布尔字段；accepted / success 语义由 status 字符串反推。
_NAV_ACCEPTED_STATUS = frozenset({"accepted", "pending", "planning", "running"})
_NAV_SUCCEEDED_STATUS = frozenset({
    "succeeded",
    "success",
    "completed",
    "reached",
})
_NAV_CANCELLED_STATUS = frozenset({"cancelled"})
_NAV_CANCEL_SUCCESS_STATUS = frozenset({"pending", "cancelled"})
_NAV_STREAM_STATUS = frozenset({
    "idle",
    "unconfirmed",
    "accepted",
    "pending",
    "planning",
    "running",
    "succeeded",
    "failed",
    "cancelled",
})
_NAV_TERMINAL_STATUS = frozenset({"succeeded", "failed", "cancelled"})
_NAV_STATUS_PROGRESS = {
    "idle": 0,
    "unconfirmed": 1,
    "accepted": 1,
    "pending": 1,
    "planning": 2,
    "running": 3,
}


def _status_means_accepted(status: str) -> bool:
    """SetNavigationGoal 返回时：服务端是否接受请求（非失败类）。"""
    return str(status or "").strip().lower() in _NAV_ACCEPTED_STATUS


def _status_means_succeeded(status: str) -> bool:
    """navigation 是否完成。"""
    return str(status or "").strip().lower() in _NAV_SUCCEEDED_STATUS


def _status_means_cancelled(status: str) -> bool:
    """CancelNavigation 返回时：是否被服务端视为已取消。"""
    return str(status or "").strip().lower() in _NAV_CANCELLED_STATUS


def _status_means_cancel_success(status: str) -> bool:
    """Accept only the two statuses emitted by Rust CancelNavigation."""

    return str(status or "").strip().lower() in _NAV_CANCEL_SUCCESS_STATUS


def _navigation_status_can_follow(
    current: str, incoming: str, *, current_timestamp_ns: int = 0,
    incoming_timestamp_ns: int = 0, current_path_revision: int = 0,
    incoming_path_revision: int = 0,
) -> bool:
    """同一租约按来源时间和路径修订推进；运行与重规划可以往返。"""

    current_status = str(current or "").strip().lower()
    incoming_status = str(incoming or "").strip().lower()
    if current_status in _NAV_TERMINAL_STATUS:
        return False
    if incoming_status in _NAV_TERMINAL_STATUS:
        return True
    if (current_path_revision > 0 and incoming_path_revision > 0
            and current_timestamp_ns > 0 and incoming_timestamp_ns > 0
            and incoming_timestamp_ns < current_timestamp_ns):
        return False
    if current_path_revision > 0 and incoming_path_revision < current_path_revision:
        return False
    if (current_status in {"running", "planning"}
            and incoming_status in {"pending", "planning"}
            and incoming_timestamp_ns > current_timestamp_ns > 0
            and incoming_path_revision >= current_path_revision > 0):
        return True
    return _NAV_STATUS_PROGRESS.get(incoming_status, -1) >= (
        _NAV_STATUS_PROGRESS.get(current_status, -1)
    )


def _navigation_status_is_ahead(current: str, incoming: str) -> bool:
    """Return whether ``incoming`` carries more progress than ``current``."""

    current_status = str(current or "").strip().lower()
    incoming_status = str(incoming or "").strip().lower()
    if incoming_status in _NAV_TERMINAL_STATUS:
        return current_status not in _NAV_TERMINAL_STATUS
    if current_status in _NAV_TERMINAL_STATUS:
        return False
    return _NAV_STATUS_PROGRESS.get(incoming_status, -1) > (
        _NAV_STATUS_PROGRESS.get(current_status, -1)
    )


def _navigation_update_is_admission_rejection(update) -> bool:
    """原生准入拒绝描述候选请求，不能把其候选租约当作已接纳的目标。"""
    return (str(update.status).strip().lower() == "failed"
            and str(update.message).startswith("SET_GOAL_REJECTED["))


_NAV_DISPATCH_UNCONFIRMED_REASONS = frozenset({
    "GOAL_PUBLISH_UNCONFIRMED", "GOAL_TERMINAL_PROOF_PENDING",
})
_NAV_LEASE_FIELDS = (
    "goal_id", "owner_kind", "parent_request_id", "goal_generation",
    "producer_runtime_generation",
)


def _navigation_identity_document(lease: _NavigationLease) -> Dict[str, Any]:
    return dict(zip(_NAV_LEASE_FIELDS, lease))


def _navigation_wire_document(update) -> Dict[str, Any]:
    """Preserve the full existing NavigationState wire body as JSON values."""
    metadata = _navigation_state_typed_metadata(update)
    metadata.pop("raw_status", None)
    metadata.pop("source_timestamp_ns", None)
    metadata.pop("observed_goal_id", None)
    return {
        **metadata,
        "status": str(update.status),
        "message": str(update.message),
        "goal_id": str(update.goal_id),
        "distance_remaining": float(update.distance_remaining),
        "timestamp_ns": int(update.timestamp_ns),
    }


def _navigation_submission_result(goal_id: str, *, accepted=False, message="",
                                  disposition="rejected", reason_code="",
                                  ownership_retained=False, authority_current=False,
                                  navigation=None) -> Dict[str, Any]:
    return {
        "goal_id": goal_id, "accepted": bool(accepted), "message": str(message or ""),
        "disposition": disposition, "reason_code": reason_code,
        "ownership_retained": bool(ownership_retained),
        "authority_current": bool(authority_current), "navigation": navigation,
    }


def _is_shutdown_guard_enabled() -> bool:
    """NUWAX_GEOMETRY_SHUTDOWN_GUARD=0 时关闭（紧急回滚）。"""
    value = os.environ.get("NUWAX_GEOMETRY_SHUTDOWN_GUARD", "1").strip().lower()
    return value not in {"0", "false", "no", "off"}


# Legacy readiness 子项仍可从 `reason` 尾部 JSON 读取；冻结 proto 的 typed
# reason code、blockers、generation 与 frame 字段在 RPC 方法内直接消费。
_NAV_READINESS_FIELDS = (
    "lcm_backend",
    "tf",
    "costmap",
    "planner",
    "cmd_vel_sink",
    "profile",
    "module_node_count",
    "p1_native_profiles",
    "backend",
    "tf_count",
    "costmap_revision",
    "cmd_vel_sink_ready",
    "active_profile",
)


def _navigation_state_typed_metadata(update) -> Dict[str, Any]:
    """Project typed NavigationState fields without parsing message text."""

    return {
        "local_path": [{"x": float(p.x), "y": float(p.y), "z": float(p.z)}
                       for p in update.local_path],
        "planned_path": [{"x": float(p.x), "y": float(p.y), "z": float(p.z)}
                         for p in update.planned_path],
        "path_revision": int(update.path_revision),
        "raw_status": str(update.status),
        "source_timestamp_ns": int(update.timestamp_ns),
        "observed_goal_id": str(update.goal_id),
        "owner_kind": int(update.owner_kind),
        "parent_request_id": str(
            update.parent_request_id
        ),
        "goal_generation": int(
            update.goal_generation
        ),
        "producer_runtime_generation": int(
            update.producer_runtime_generation
        ),
        "planner_runtime_generation": int(
            update.planner_runtime_generation
        ),
        "cancel_phase": str(update.cancel_phase),
        "cancel_generation": int(update.cancel_generation),
        "actuator_zeroed": bool(update.actuator_zeroed),
        "reason_code": str(update.reason_code),
        "runtime_generation": int(update.runtime_generation),
        "sink_ready": bool(update.sink_ready),
        "planner_map_frame": str(update.planner_map_frame),
        "planner_pose_frame": str(update.planner_pose_frame),
        "zero_ack_id": str(update.zero_ack_id),
        "zero_applied_monotonic_ns": int(
            update.zero_applied_monotonic_ns
        ),
    }


def _navigation_lease_from_message(update) -> _NavigationLease:
    return (
        str(update.goal_id).strip(),
        int(update.owner_kind),
        str(update.parent_request_id).strip(),
        int(update.goal_generation),
        int(update.producer_runtime_generation),
    )


def _is_canonical_public_navigation_lease(
    lease: _NavigationLease,
) -> bool:
    (
        goal_id,
        owner_kind,
        parent_request_id,
        goal_generation,
        producer_runtime_generation,
    ) = lease
    return bool(
        goal_id
        and owner_kind == _PUBLIC_NAV_OWNER_KIND
        and parent_request_id == goal_id
        and goal_generation > 0
        and producer_runtime_generation > 0
    )


def _is_canonical_exploration_child_navigation_lease(
    lease: _NavigationLease,
) -> bool:
    (
        goal_id,
        owner_kind,
        parent_request_id,
        goal_generation,
        producer_runtime_generation,
    ) = lease
    prefix = "%s.nav." % parent_request_id
    return bool(
        owner_kind == _EXPLORATION_CHILD_NAV_OWNER_KIND
        and parent_request_id
        and goal_id.startswith(prefix)
        and goal_id[len(prefix) :]
        and goal_generation > 0
        and producer_runtime_generation > 0
    )


def _parse_navigation_readiness_reason(ready: bool, reason: str) -> Dict[str, Any]:
    """从 gRPC `NavigationReadiness.reason` 提取细分字段。

    Rust 端格式:``"{top_reason}; details={...json...}"``。
    解析失败或字段缺失时,top-level reason 仍保留,子项用 `None` 占位,
    避免上层 `readiness["planner"]` 抛 `KeyError`。
    """
    result: Dict[str, Any] = {
        "ready": ready,
        "message": reason,
        "lcm_backend": None,
        "tf": None,
        "costmap": None,
        "planner": None,
        "cmd_vel_sink": None,
        "profile": None,
        "module_node_count": None,
        "p1_native_profiles": None,
    }
    marker = "; details="
    idx = reason.find(marker)
    if idx < 0:
        return result
    top_reason = reason[:idx].strip()
    json_str = reason[idx + len(marker) :].strip()
    result["message"] = top_reason
    try:
        details = json.loads(json_str)
    except (ValueError, TypeError):
        return result
    if not isinstance(details, dict):
        return result
    for field in _NAV_READINESS_FIELDS:
        if field in details:
            result[field] = details[field]
    return result


def _nav_readiness_disconnected_dict(message: str = "未连接") -> Dict[str, Any]:
    """``_stub`` 未就绪 / gRPC 失败时返回的最小 dict。"""
    return {
        "ready": False,
        "message": message,
        "lcm_backend": None,
        "tf": None,
        "costmap": None,
        "planner": None,
        "cmd_vel_sink": None,
        "profile": None,
        "module_node_count": None,
        "p1_native_profiles": None,
        # Phase 7 增量字段
        "backend": None,
        "tf_count": None,
        "costmap_revision": None,
        "cmd_vel_sink_ready": None,
        "active_profile": None,
        # Frozen typed NavigationReadiness fields 3..9.
        "reason_code": None,
        "blockers": [],
        "tf_generation": None,
        "planner_generation": None,
        "motion_sink_generation": None,
        "map_frame": None,
        "pose_frame": None,
        "forbidden_zone_map_name": None,
        "forbidden_zone_generation": None,
        "forbidden_zone_signature": None,
        "forbidden_zone_applied_revision": None,
    }


def _parse_readiness_message(message: str) -> Dict[str, Any]:
    """把 Phase 7 gRPC `MappingReadiness.message` / `ExplorationReadiness.message`
    中的 JSON 字符串解析为 dict。失败时返回空 dict(调用方负责补默认值)。

    该 helper 与 `_parse_navigation_readiness_reason` 形状相似但 payload 不同:
    - mapping: {lcm_backend, adapter_present, odom_count, pointcloud_count,
                last_occupancy_revision, accumulated_point_count, last_error}
    - exploration: {backend_available, controller_active, source_count,
                    frontier_source_ready, costmap_input_ready,
                    costmap_input_revision, active_source, arbitration}
    """
    try:
        data = json.loads(message or "{}")
    except (ValueError, TypeError):
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def _normalize_frame(frame_id: str) -> str:
    """去掉 ROS 风格前导斜杠并去空白，便于契约校验。"""
    return str(frame_id or "").strip().lstrip("/")




def _exploration_child_lease_to_dict(lease: Any) -> Dict[str, Any]:
    return {
        "session_id": str(lease.session_id or ""),
        "owner_kind": int(lease.owner_kind),
        "parent_request_id": str(lease.parent_request_id or ""),
        "goal_generation": int(lease.goal_generation),
        "producer_runtime_generation": int(lease.producer_runtime_generation),
    }






def _exploration_state_to_dict(resp: Any) -> Dict[str, Any]:
    """把 proto `ExplorationState` 转成 dict(给 `GeometryRuntimeProvider` 对外契约用)。

    状态字符串由 native scheduler 直接给出：IDLE / ACCEPTED / REJECTED /
    PENDING / STOPPING / ACTIVE / SUCCEEDED / CANCELLED / FAILED。失败时上游
    RPC 层已经处理，这里只做字段拷贝，不二次推进状态。
    """
    child_lease = None
    if resp.HasField("child_lease"):
        child_lease = _exploration_child_lease_to_dict(resp.child_lease)
    return {
        "mission_progress": json.loads(resp.mission_progress_json) if resp.mission_progress_json else None,
        "status": resp.status,
        "message": resp.message,
        "request_id": resp.request_id,
        "selected_source": resp.selected_source,
        "last_stop_reason": resp.last_stop_reason,
        "frontier_count": int(resp.frontier_count),
        "revision": int(resp.revision),
        "timestamp_ns": int(resp.timestamp_ns),
        "child_task_id": str(resp.child_task_id),
        "exploration_generation": int(resp.exploration_generation),
        "child_lease": child_lease,
        "parking_pending": bool(resp.parking_pending),
    }


def _build_pose(proto_pose: Any) -> Optional[Pose]:
    """把 gRPC Pose 转成契约 Pose；frame_id 为空（无定位）时返回 None。"""
    frame_id = _normalize_frame(proto_pose.frame_id)
    if not frame_id:
        return None
    pos = proto_pose.position
    ori = proto_pose.orientation
    try:
        source_timestamp_ns = int(proto_pose.timestamp_ns)
        if source_timestamp_ns < 0:
            return None
        # 保留源采样时间；零值仍为未知时间哨兵，不能以处理时刻伪造新鲜度。
        source_timestamp = datetime.fromtimestamp(
            source_timestamp_ns // 1_000_000_000,
            tz=timezone.utc,
        ).replace(microsecond=(source_timestamp_ns % 1_000_000_000) // 1_000)
        return Pose(
            timestamp=source_timestamp,
            frame_id=frame_id,
            position=Vector3(x=pos.x, y=pos.y, z=pos.z),
            orientation=Quaternion(x=ori.x, y=ori.y, z=ori.z, w=ori.w),
        )
    except Exception:  # pragma: no cover - 非法 frame_id 等
        return None


def _build_frame_tree(transforms: Any) -> Optional[FrameTree]:
    """把 gRPC Transform 列表转成契约 FrameTree；空树返回 None。"""
    contract_transforms: List[Transform] = []
    children: set = set()
    parents: List[str] = []
    for t in transforms:
        parent = _normalize_frame(t.parent_frame)
        child = _normalize_frame(t.child_frame)
        if not parent or not child:
            continue
        try:
            source_timestamp_ns = int(t.timestamp_ns)
            # 与 Pose 一致：保留每条边的源时间，零值表示未知，而非处理时刻。
            source_timestamp = datetime.fromtimestamp(
                source_timestamp_ns // 1_000_000_000,
                tz=timezone.utc,
            ).replace(microsecond=(source_timestamp_ns % 1_000_000_000) // 1_000)
            contract_transforms.append(
                Transform(
                    timestamp=source_timestamp,
                    parent_frame_id=parent,
                    child_frame_id=child,
                    translation=Vector3(x=t.translation.x, y=t.translation.y, z=t.translation.z),
                    rotation=Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w),
                )
            )
        except Exception:  # pragma: no cover - 非法 frame_id
            continue
        children.add(child)
        if parent not in parents:
            parents.append(parent)
    if not contract_transforms:
        return None
    # 根坐标系：从不作为子坐标系出现的父坐标系；缺省回退 map / 第一个父系。
    root = next((p for p in parents if p not in children), None) or (
        "map" if any(p == "map" for p in parents) else parents[0]
    )
    try:
        return FrameTree(root_frame_id=root, transforms=contract_transforms)
    except Exception:  # pragma: no cover
        return None


def _merge_static_edges(frame_tree: Optional[FrameTree], static_edges: List[Transform]) -> Optional[FrameTree]:
    """把装配期缓存的静态外参边合并进每帧重建的 frame_tree。

    设计约束（见评测报告 §4.5 不变量）：
    1. frame_tree 每帧由 gRPC 全量重建，本函数在重建后合并，不在 _build_frame_tree
       纯函数里做一次性注入（那样会被下一帧覆盖）。
    2. 静态边只新增标定自有的 child（如相机光学系），**禁止覆盖** map/odom/base
       等动态定位链；child 已存在或角色属动态链时跳过。
    3. 静态边声明的 parent 是端侧 driver 的 base 命名（如 world/go2/base），可能与
       实机 tf 实际发布的 base 串（base_footprint/base_link）不同。这里按语义角色把
       parent 重锚定到"当前帧真实存在的 base 节点串"，否则静态边会成为孤岛、
       camera→map 路径不可达。找不到 base 节点则跳过该边（不兜底、不投错链）。
    """
    if not static_edges or frame_tree is None:
        return frame_tree
    existing = list(frame_tree.transforms)
    present_children = {t.child_frame_id for t in existing}
    present_frames = set()
    for t in existing:
        present_frames.add(t.parent_frame_id)
        present_frames.add(t.child_frame_id)
    # 当前帧真实存在的 base 节点串（按语义角色匹配，取第一个角色为 base 的 frame）。
    live_base_frame = next((f for f in present_frames if infer_frame_role(f) == "base"), None)
    if live_base_frame is None:
        configured_base = _normalize_frame(runtime_value("NUWAX_BASE_FRAME", ""))
        if not configured_base:
            configured_base = _normalize_frame(
                os.environ.get("NUWAX_GEOMETRY_BASE_FRAME", "")
            )
        if configured_base and configured_base in present_frames:
            live_base_frame = configured_base

    merged = list(existing)
    appended = 0
    for edge in static_edges:
        # 守护：child 已存在 → 跳过，绝不覆盖既有边（含动态链）。
        if edge.child_frame_id in present_children:
            continue
        # 守护：静态边的 child 不得是 map/odom/base 等动态语义角色。
        if infer_frame_role(edge.child_frame_id) in {"map", "odom", "world", "base"}:
            LOGGER.warning("拒绝合并静态外参边：child=%s 属动态定位链角色", edge.child_frame_id)
            continue
        # 重锚定 parent 到当前帧真实 base 节点串。
        parent_frame = edge.parent_frame_id
        if parent_frame not in present_frames:
            if live_base_frame is None:
                _warn_static_edge_skip(
                    "静态外参边 parent=%s 在当前 frame_tree 不存在且无 base 节点可锚定，跳过 child=%s"
                    % (parent_frame, edge.child_frame_id),
                    signature="missing_base:%s:%s" % (parent_frame, edge.child_frame_id),
                )
                continue
            if infer_frame_role(parent_frame) == "base":
                parent_frame = live_base_frame
            else:
                LOGGER.warning(
                    "静态外参边 parent=%s 既不在当前 frame_tree 也非 base 角色，跳过 child=%s",
                    parent_frame, edge.child_frame_id,
                )
                continue
        try:
            merged.append(
                Transform(
                    parent_frame_id=parent_frame,
                    child_frame_id=edge.child_frame_id,
                    translation=edge.translation,
                    rotation=edge.rotation,
                    authority=edge.authority,
                )
            )
            appended += 1
        except Exception:  # pragma: no cover - 非法 frame_id
            continue
    if appended == 0:
        return frame_tree
    try:
        return FrameTree(root_frame_id=frame_tree.root_frame_id, transforms=merged)
    except Exception:  # pragma: no cover
        return frame_tree


def resolve_direct_static_calibration_edge(
    frame_tree: FrameTree,
    *,
    parent_frame_id: str,
    child_frame_id: str,
) -> Transform:
    """Resolve one direct, finite, unit-quaternion static calibration edge."""
    matches = [
        edge
        for edge in frame_tree.transforms
        if edge.parent_frame_id == parent_frame_id
        and edge.child_frame_id == child_frame_id
    ]
    if len(matches) != 1:
        raise ValueError(
            "expected exactly one direct calibration edge %s->%s; found %d"
            % (parent_frame_id, child_frame_id, len(matches))
        )

    edge = matches[0]
    if edge.authority != "calibration_static":
        raise ValueError(
            "direct calibration edge %s->%s must have authority=calibration_static"
            % (parent_frame_id, child_frame_id)
        )

    translation = edge.translation
    rotation = edge.rotation
    values = (
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(
            "direct calibration edge %s->%s must contain only finite values"
            % (parent_frame_id, child_frame_id)
        )

    quaternion_norm_squared = sum(
        float(value) * float(value)
        for value in (rotation.x, rotation.y, rotation.z, rotation.w)
    )
    if not math.isclose(quaternion_norm_squared, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(
            "direct calibration edge %s->%s must use a unit quaternion"
            % (parent_frame_id, child_frame_id)
        )
    return edge


class GeometryRuntimeProvider:
    """统一的 GeometryRuntimeProvider，封装所有 gRPC 调用。"""

    def __init__(self, bridge: GeometryRuntimeSubprocess) -> None:
        self._bridge = bridge
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[pb2_grpc.GeometryRuntimeStub] = None
        self._connected = False
        # shutdown 期守卫标志：disconnect()/清状态时置 True，所有 RPC 入口 guard。
        # 防止 in-flight task 持陈旧 _stub 引用继续打 RPC 全 UNAVAILABLE。
        self._stopping = False
        # 装配期一次性缓存的静态外参边（相机光学系等），每帧 get_frame_tree 合并。
        self._static_extrinsic_edges: List[Transform] = []
        # NavigationProvider 协议 shim 的本地状态缓存：SubscribeNavigationState 流最近一帧
        # + 上一次 set_goal/cancel_goal 触发的目标 id；get_navigation_state() 据此合成
        # contracts.NavigationState。lock 防后台订阅线程与 caller 并发改写。
        self._nav_connection_transition_lock = threading.Lock()
        self._nav_delivery_lock = threading.RLock()
        self._nav_notification_condition = threading.Condition()
        self._nav_connection_transition_inflight = False
        self._nav_pending_notification_tickets: Dict[
            int,
            _NavigationNotificationTicket,
        ] = {}
        self._next_navigation_notification_ticket_id = 1
        self._nav_active_notification_count = 0
        self._nav_notification_threads: Dict[int, int] = {}
        self._nav_active_notification_sequences: Dict[int, int] = {}
        self._nav_notification_sequence_threads: Dict[
            Tuple[int, int],
            int,
        ] = {}
        self._nav_active_notification_subscriptions: Dict[int, int] = {}
        self._nav_notification_subscription_threads: Dict[
            Tuple[int, int],
            int,
        ] = {}
        self._nav_state_lock = threading.Lock()
        self._nav_command_lock = threading.Lock()
        # Serialize complete SetGoal transactions without blocking Cancel or
        # holding delivery/command/state locks across the unary RPC.
        self._nav_set_transaction_lock = threading.Lock()
        self._nav_set_transaction_context = threading.local()
        self._nav_command_generation = 0
        self._nav_inflight_command_token: Optional[
            _NavigationCommandToken
        ] = None
        self._nav_completed_cancel_command_token: Optional[
            _NavigationCommandToken
        ] = None
        self._nav_set_precommit_candidates: Dict[
            _NavigationSetPrecommitKey,
            Any,
        ] = {}
        self._nav_connection_epoch = 0
        self._nav_stream_epoch = 0
        self._nav_active_stream_token: Optional[_NavigationStreamToken] = None
        self._nav_stream_call = None
        self._last_nav_goal_id: Optional[str] = None
        self._last_nav_owner_kind = 0
        self._last_nav_parent_request_id = ""
        self._last_nav_goal_generation = 0
        self._last_nav_producer_runtime_generation = 0
        self._last_nav_goal_command_generation = 0
        self._last_nav_status: str = ""  # 空串表示尚未收到流更新；视为 IDLE
        self._last_nav_message: Optional[str] = None
        self._last_nav_distance_remaining: Optional[float] = None
        self._last_nav_current_pose: Optional[Pose] = None
        self._last_nav_timestamp_ns: int = 0
        self._last_nav_metadata: Dict[str, Any] = {}
        self._nav_exploration_overlay: Optional[pb2.NavigationState] = None
        self._nav_exploration_overlay_terminal = False
        self._nav_stream_started = False
        self._nav_stream_thread: Optional[threading.Thread] = None
        self._navigation_update_subscribers: Dict[int, Callable[[NavigationState], None]] = {}
        self._next_navigation_subscription_id = 1
        self._exploration_state_lock = threading.RLock()
        self._exploration_update_subscribers: Dict[
            int,
            Callable[[Dict[str, Any]], None],
        ] = {}
        self._next_exploration_subscription_id = 1
        self._exploration_poll_thread: Optional[threading.Thread] = None
        self._exploration_poll_stop_event: Optional[threading.Event] = None
        self._last_exploration_signature = None
        self._map_delivery_lock = threading.RLock()
        self._map_state_lock = threading.Lock()
        self._planar_scan_receipts = {}
        self._planar_unix_offset_ns = 0
        self._map_delivery_condition = threading.Condition()
        self._map_delivery_tickets: Dict[int, _MapDeliveryTicket] = {}
        self._next_map_delivery_ticket_id = 1
        self._last_occupancy_grid: Optional[OccupancyGrid] = None
        self._last_map_update_evidence = None
        self._assigned_fine_archive_versions = frozenset()
        self._native_input_failure_reason: Optional[str] = None
        self._last_occupancy_grid_identity: Optional[_MappingIdentity] = None
        self._last_occupancy_grid_revision = 0
        self._last_map_stream_observation = None
        self._mapping_identity: Optional[_MappingIdentity] = None
        self._planar_input_active = False
        self._map_connection_epoch = 0
        self._mapping_transition_epoch = 0
        self._mapping_transition_inflight = False
        self._assigned_activation_attempt: Optional[_AssignedActivationAttempt] = None
        self._map_delivery_owner_thread_id: Optional[int] = None
        self._map_delivery_in_progress = False
        self._stream_handle_lock = threading.Lock()
        self._pose_stream_handles: Set[_CancellableStreamHandle] = set()
        self._map_stream_handles: Set[_CancellableStreamHandle] = set()

    def set_static_extrinsics(self, edges: List[Transform]) -> None:
        """装配期由平台几何运行时注入静态外参边（来自 CalibrationProvider）。

        外参为静态值，此处缓存一次；get_frame_tree 每帧重建后零 I/O 合并这些边，
        既不被全量重建覆盖，又不引入运行时读盘/高频写。重复调用以最新一次为准。
        """
        self._static_extrinsic_edges = list(edges or [])
        LOGGER.info("已缓存 %d 条静态外参边用于 frame_tree 合并", len(self._static_extrinsic_edges))

    def _current_navigation_lease_locked(self) -> _NavigationLease:
        """Return the cached lease while `_nav_state_lock` is held."""

        return (
            str(self._last_nav_goal_id or "").strip(),
            int(self._last_nav_owner_kind),
            str(self._last_nav_parent_request_id or "").strip(),
            int(self._last_nav_goal_generation),
            int(self._last_nav_producer_runtime_generation),
        )

    def _navigation_obligation_locked(self, lease=None):
        """Read only this cache's current-epoch obligation; caller holds state lock."""
        lease = self._current_navigation_lease_locked() if lease is None else lease
        obligation = self._last_nav_metadata.get("dispatch_obligation")
        if (isinstance(obligation, dict)
                and self._last_nav_metadata.get("ownership_retained") is True
                and self._last_nav_metadata.get("authority_current") is True
                and obligation.get("connection_epoch") == self._nav_connection_epoch
                and obligation.get("identity") == _navigation_identity_document(lease)
                and _is_canonical_public_navigation_lease(lease)):
            return obligation
        return None

    def _navigation_has_unsettled_owner_locked(self) -> bool:
        """仅当前连接的真实未决租约阻止另一地点目标。"""
        return bool(
            self._navigation_obligation_locked() is not None
            or (_is_canonical_public_navigation_lease(self._current_navigation_lease_locked())
                and self._last_nav_status.strip().lower()
                    in {"accepted", "pending", "planning", "running", "unconfirmed"})
        )

    def _navigation_cache_wire_locked(self) -> Dict[str, Any]:
        metadata = copy.deepcopy(self._last_nav_metadata)
        # Contract-only diagnostics are additive; the original wire field values remain intact.
        return {
            **metadata,
            "goal_id": self._last_nav_goal_id or "",
            "status": self._last_nav_status,
            "message": self._last_nav_message or "",
            "distance_remaining": self._last_nav_distance_remaining,
            "timestamp_ns": self._last_nav_timestamp_ns,
        }

    def _navigation_project_update_locked(
        self, update, *, trusted_origin_reason="", command_projection_without_source=False,
    ):
        """Rebuild metadata, inheriting only this exact lease/epoch's obligation.

        Only the current Set/cloud unary commit supplies trusted_origin_reason.
        Stream and Cancel callers can preserve responsibility but cannot establish it.
        """
        lease = _navigation_lease_from_message(update)
        metadata = _navigation_state_typed_metadata(update)
        raw_status = str(update.status).strip().lower()
        inherited = self._navigation_obligation_locked(lease)
        metadata["authority_current"] = _is_canonical_public_navigation_lease(lease)
        metadata["ownership_retained"] = False
        # 原生拥有方已裁决终态，停车字段只作为原始信息保留。
        if raw_status in _NAV_TERMINAL_STATUS:
            return raw_status, metadata
        obligation = copy.deepcopy(inherited) if inherited is not None else None
        if trusted_origin_reason in _NAV_DISPATCH_UNCONFIRMED_REASONS:
            if obligation is None:
                obligation = {
                    "origin_reason_code": trusted_origin_reason,
                    "connection_epoch": self._nav_connection_epoch,
                    "identity": _navigation_identity_document(lease),
                    "terminal_observed": False,
                }
            if trusted_origin_reason == "GOAL_TERMINAL_PROOF_PENDING":
                obligation["terminal_observed"] = True
        if obligation is not None:
            if obligation.get("terminal_observed"):
                # No invented FAILED/SUCCEEDED when the typed unary only says terminal seen.
                metadata["reason_code"] = "GOAL_TERMINAL_PROOF_PENDING"
                raw_status = "unconfirmed"
            metadata["dispatch_obligation"] = obligation
            metadata["ownership_retained"] = True
        if command_projection_without_source:
            metadata["command_projection_without_source"] = True
        return raw_status, metadata

    def _navigation_owned_update_can_follow_locked(self, update) -> bool:
        """Apply terminal observation and source/path ordering before public projection."""
        obligation = self._navigation_obligation_locked()
        if obligation is None:
            return True
        if str(update.status).strip().lower() in _NAV_TERMINAL_STATUS:
            return True
        path = int(update.path_revision)
        previous_path = int(self._last_nav_metadata.get("path_revision") or 0)
        stamp = int(update.timestamp_ns)
        if path < previous_path:
            return False
        if (self._last_nav_timestamp_ns > 0 and stamp > 0
                and stamp < self._last_nav_timestamp_ns):
            return False
        if (path == previous_path > 0
                and int(update.planner_runtime_generation)
                    != int(self._last_nav_metadata.get("planner_runtime_generation") or 0)):
            return False
        return True

    def _register_navigation_set_locked(self, request_lease, *, reject_any_owner=False):
        """Check owner and register atomically under delivery + command locks."""
        if (self._nav_connection_transition_inflight or not self._connected or self._stub is None):
            return None, None
        with self._nav_state_lock:
            busy = (self._navigation_has_unsettled_owner_locked() if reject_any_owner
                    else self._last_nav_metadata.get("ownership_retained") is True)
            if busy:
                self._nav_set_transaction_context.result = _navigation_submission_result(
                    request_lease[0], message="已有导航责任尚未结算，禁止提交新目标。",
                    reason_code="BUSY",
                )
                return None, None
        self._clear_navigation_set_precommit_locked()
        self._nav_command_generation += 1
        token = ("set", request_lease, self._nav_command_generation, self._nav_connection_epoch)
        self._nav_inflight_command_token = token
        self._nav_completed_cancel_command_token = None
        return token, self._stub

    def _finish_navigation_set_without_receipt(self, command_token, command_stub) -> None:
        """No response body cannot establish installation or unknown provenance."""
        with self._nav_delivery_lock:
            with self._nav_command_lock:
                self._clear_navigation_set_precommit_locked(command_token)
                if (command_token == self._nav_inflight_command_token
                        and command_token[2] == self._nav_command_generation
                        and command_token[3] == self._nav_connection_epoch
                        and command_stub is self._stub):
                    self._nav_inflight_command_token = None
                    with self._nav_state_lock:
                        self._last_nav_goal_command_generation = command_token[2]

    def _commit_navigation_submission(self, resp, *, command_token, command_stub):
        """Return the command result; rejected path materialization stays outside cache locks."""
        result = self._try_commit_navigation_submission(
            resp, command_token=command_token, command_stub=command_stub,
        )
        if result is not None:
            return result
        rejected = _navigation_submission_result(
            command_token[1][0], message=str(resp.message),
            reason_code=str(resp.reason_code),
            navigation=_navigation_wire_document(resp),
        )
        self._nav_set_transaction_context.result = rejected
        return rejected

    def _try_commit_navigation_submission(self, resp, *, command_token, command_stub):
        """Commit under the original lock order; None means this response was not admitted."""
        request_goal_id = command_token[1][0]
        with self._nav_delivery_lock:
            with self._nav_command_lock:
                if not (command_token == self._nav_inflight_command_token
                        and command_token[2] == self._nav_command_generation
                        and command_token[3] == self._nav_connection_epoch
                        and command_stub is self._stub and self._connected
                        and not self._nav_connection_transition_inflight):
                    self._clear_navigation_set_precommit_locked(command_token)
                    return None
                lease = _navigation_lease_from_message(resp)
                canonical = (_is_canonical_public_navigation_lease(lease)
                             and lease[:3] == command_token[1][:3])
                raw_status = str(resp.status).strip().lower()
                reason = str(resp.reason_code)
                # Freeze provenance before selecting a stronger real precommit body.
                origin = reason if (canonical and reason in _NAV_DISPATCH_UNCONFIRMED_REASONS
                                    and raw_status in {"unconfirmed", "accepted", "pending", "planning", "running"}) else ""
                terminal = raw_status in _NAV_TERMINAL_STATUS
                may_commit = (canonical and not _navigation_update_is_admission_rejection(resp)
                              and (_status_means_accepted(raw_status) or bool(origin) or terminal))
                staged = self._take_navigation_set_precommit_locked(command_token=command_token, ack_lease=lease)
                self._nav_inflight_command_token = None
                with self._nav_state_lock:
                    self._last_nav_goal_command_generation = command_token[2]
                    if not may_commit:
                        return None
                    committed = resp
                    from_stream = False
                    if staged is not None and not terminal:
                        staged_status = str(staged.status).strip().lower()
                        if (staged_status in _NAV_TERMINAL_STATUS
                                or _navigation_status_is_ahead(raw_status, staged_status)
                                or (bool(origin) and staged_status == raw_status)):
                            committed, from_stream = staged, True
                    without_source = bool(origin and not from_stream
                                          and int(resp.timestamp_ns) == 0)
                    status, metadata = self._navigation_project_update_locked(
                        committed, trusted_origin_reason=origin,
                        command_projection_without_source=without_source,
                    )
                    # This matching public unary is authoritative over a stale display
                    # overlay; arbitrary later foreign streams cannot replace its duty.
                    self._nav_exploration_overlay = None
                    self._nav_exploration_overlay_terminal = False
                    self._last_nav_goal_id = lease[0]
                    self._last_nav_owner_kind = lease[1]
                    self._last_nav_parent_request_id = lease[2]
                    self._last_nav_goal_generation = lease[3]
                    self._last_nav_producer_runtime_generation = lease[4]
                    self._last_nav_status = status
                    self._last_nav_message = str(committed.message) or None
                    self._last_nav_distance_remaining = float(committed.distance_remaining)
                    self._last_nav_current_pose = None
                    self._last_nav_timestamp_ns = int(committed.timestamp_ns)
                    self._last_nav_metadata = metadata
                    resolved = status in _NAV_TERMINAL_STATUS
                    accepted = _status_means_accepted(status)
                    owned = metadata.get("ownership_retained") is True
                    navigation = _navigation_wire_document(committed)
                    # Full receipts preserve all original wire values. Extra diagnostic keys
                    # never replace a receipt's reason, time, identity or proof.
                    navigation.update(
                        ownership_retained=owned, authority_current=True,
                        command_projection_without_source=bool(metadata.get("command_projection_without_source")),
                    )
                    if "dispatch_obligation" in metadata:
                        navigation["dispatch_obligation"] = copy.deepcopy(metadata["dispatch_obligation"])
                    if not resolved:
                        navigation["status"] = status
                        navigation["reason_code"] = metadata.get("reason_code", "")
                    result = _navigation_submission_result(
                        request_goal_id, accepted=accepted, message=navigation["message"],
                        disposition=("resolved" if resolved else "confirmed" if accepted else "owned_unconfirmed"),
                        reason_code=(navigation["reason_code"] if resolved or status == "unconfirmed" else origin or navigation["reason_code"]),
                        ownership_retained=owned, authority_current=True, navigation=navigation,
                    )
                    self._nav_set_transaction_context.result = result
                    return result

    def _clear_navigation_set_precommit_locked(
        self,
        command_token: Optional[_NavigationCommandToken] = None,
    ) -> None:
        """Discard staged SetGoal frames while `_nav_command_lock` is held."""

        if command_token is None:
            self._nav_set_precommit_candidates.clear()
            return
        stale_keys = tuple(
            key
            for key in self._nav_set_precommit_candidates
            if key[2] == command_token
        )
        for key in stale_keys:
            self._nav_set_precommit_candidates.pop(key, None)

    def _stage_navigation_set_precommit_locked(
        self,
        update,
        *,
        incoming_lease: _NavigationLease,
        stream_token: _NavigationStreamToken,
    ) -> bool:
        """Stage a current SetGoal frame without publishing it.

        The full key freezes every authority boundary needed at unary commit:
        connection, stream, command, canonical request identity and exact lease.
        """

        command_token = self._nav_inflight_command_token
        if (
            command_token is None
            or command_token[0] != "set"
            or command_token[2] != self._nav_command_generation
            or command_token[3] != self._nav_connection_epoch
        ):
            return False
        request_lease = command_token[1]
        request_identity = (
            request_lease[0],
            request_lease[1],
            request_lease[2],
        )
        incoming_identity = (
            incoming_lease[0],
            incoming_lease[1],
            incoming_lease[2],
        )
        if (
            incoming_identity != request_identity
            or not _is_canonical_public_navigation_lease(incoming_lease)
        ):
            return False
        key = (
            self._nav_connection_epoch,
            stream_token,
            command_token,
            request_identity,
            incoming_lease,
        )
        previous = self._nav_set_precommit_candidates.get(key)
        if (previous is not None
                and not _navigation_status_can_follow(
            previous.status,
            update.status,
            current_timestamp_ns=int(previous.timestamp_ns),
            incoming_timestamp_ns=int(update.timestamp_ns),
            current_path_revision=int(previous.path_revision),
            incoming_path_revision=int(update.path_revision),
        )):
            LOGGER.info(
                "丢弃回退的 SetGoal precommit frame: goal_id=%s generation=%s status=%s->%s",
                incoming_lease[0],
                incoming_lease[3],
                previous.status,
                update.status,
            )
            return True
        staged = pb2.NavigationState()
        staged.CopyFrom(update)
        self._nav_set_precommit_candidates[key] = staged
        return True

    def _take_navigation_set_precommit_locked(
        self,
        *,
        command_token: _NavigationCommandToken,
        ack_lease: _NavigationLease,
    ):
        """Take the active stream's exact ack candidate, then clear the command."""

        stream_token = self._nav_active_stream_token
        request_lease = command_token[1]
        request_identity = (
            request_lease[0],
            request_lease[1],
            request_lease[2],
        )
        candidate = None
        if stream_token is not None:
            key = (
                self._nav_connection_epoch,
                stream_token,
                command_token,
                request_identity,
                ack_lease,
            )
            candidate = self._nav_set_precommit_candidates.get(key)
        self._clear_navigation_set_precommit_locked(command_token)
        return candidate

    def _cancel_pending_navigation_notifications_locked(self) -> None:
        """Cancel tickets that have not started all snapshot callbacks."""

        with self._nav_notification_condition:
            tickets = tuple(self._nav_pending_notification_tickets.values())
            self._nav_pending_notification_tickets.clear()
            for ticket in tickets:
                ticket.cancelled = True
            self._nav_notification_condition.notify_all()

    def _begin_navigation_notification(
        self,
        state: NavigationState,
        *,
        stream_token: Optional[_NavigationStreamToken] = None,
    ) -> Optional[_NavigationNotificationTicket]:
        """Freeze tokens and subscribers while `_nav_delivery_lock` is held."""

        with self._nav_command_lock:
            if (
                self._nav_connection_transition_inflight
                or not self._connected
                or self._stopping
                or (
                    stream_token is not None
                    and stream_token != self._nav_active_stream_token
                )
            ):
                return None
            delivery_token = (
                self._nav_connection_epoch,
                stream_token,
                self._nav_command_generation,
            )
            with self._nav_state_lock:
                subscribers = tuple(
                    self._navigation_update_subscribers.items()
                )
        if not subscribers:
            return None
        with self._nav_notification_condition:
            ticket_id = self._next_navigation_notification_ticket_id
            self._next_navigation_notification_ticket_id += 1
            ticket = _NavigationNotificationTicket(
                ticket_id=ticket_id,
                state=state,
                delivery_token=delivery_token,
                subscribers=subscribers,
            )
            self._nav_pending_notification_tickets[ticket_id] = ticket
            return ticket

    def _begin_navigation_notification_callback(
        self,
        ticket: _NavigationNotificationTicket,
        subscription_id: int,
        callback: Callable[[NavigationState], None],
    ) -> bool:
        """Linearize one callback start against transition and unsubscribe."""

        thread_id = threading.get_ident()
        with self._nav_notification_condition:
            while True:
                if (
                    ticket.cancelled
                    or self._nav_pending_notification_tickets.get(
                        ticket.ticket_id
                    )
                    is not ticket
                ):
                    return False
                older_active_count = sum(
                    count
                    for sequence, count in (
                        self._nav_active_notification_sequences.items()
                    )
                    if sequence < ticket.ticket_id
                )
                older_owned_by_thread = sum(
                    count
                    for (owner_thread_id, sequence), count in (
                        self._nav_notification_sequence_threads.items()
                    )
                    if owner_thread_id == thread_id
                    and sequence < ticket.ticket_id
                )
                if older_active_count <= older_owned_by_thread:
                    break
                self._nav_notification_condition.wait()
            if older_owned_by_thread > 0:
                LOGGER.warning(
                    "导航 callback 重入产生更新，丢弃无法单调投递的 ticket=%s",
                    ticket.ticket_id,
                )
                return False
        with self._nav_delivery_lock:
            with self._nav_command_lock:
                connection_epoch, stream_token, command_generation = (
                    ticket.delivery_token
                )
                if (
                    self._nav_connection_transition_inflight
                    or not self._connected
                    or self._stopping
                    or connection_epoch != self._nav_connection_epoch
                    or command_generation != self._nav_command_generation
                    or (
                        stream_token is not None
                        and stream_token != self._nav_active_stream_token
                    )
                ):
                    return False
                with self._nav_state_lock:
                    if (
                        self._navigation_update_subscribers.get(subscription_id)
                        is not callback
                    ):
                        return False
                with self._nav_notification_condition:
                    if (
                        ticket.cancelled
                        or self._nav_pending_notification_tickets.get(
                            ticket.ticket_id
                        )
                        is not ticket
                    ):
                        return False
                    self._nav_active_notification_count += 1
                    self._nav_notification_threads[thread_id] = (
                        self._nav_notification_threads.get(thread_id, 0) + 1
                    )
                    self._nav_active_notification_sequences[
                        ticket.ticket_id
                    ] = (
                        self._nav_active_notification_sequences.get(
                            ticket.ticket_id,
                            0,
                        )
                        + 1
                    )
                    sequence_thread_key = (thread_id, ticket.ticket_id)
                    self._nav_notification_sequence_threads[
                        sequence_thread_key
                    ] = (
                        self._nav_notification_sequence_threads.get(
                            sequence_thread_key,
                            0,
                        )
                        + 1
                    )
                    self._nav_active_notification_subscriptions[
                        subscription_id
                    ] = (
                        self._nav_active_notification_subscriptions.get(
                            subscription_id,
                            0,
                        )
                        + 1
                    )
                    subscription_thread_key = (thread_id, subscription_id)
                    self._nav_notification_subscription_threads[
                        subscription_thread_key
                    ] = (
                        self._nav_notification_subscription_threads.get(
                            subscription_thread_key,
                            0,
                        )
                        + 1
                    )
                    return True

    def _finish_navigation_notification(
        self,
        ticket: _NavigationNotificationTicket,
        subscription_id: int,
    ) -> None:
        thread_id = threading.get_ident()
        with self._nav_notification_condition:
            count = self._nav_notification_threads.get(thread_id, 0)
            if count <= 1:
                self._nav_notification_threads.pop(thread_id, None)
            else:
                self._nav_notification_threads[thread_id] = count - 1
            self._nav_active_notification_count = max(
                0,
                self._nav_active_notification_count - 1,
            )
            sequence_count = self._nav_active_notification_sequences.get(
                ticket.ticket_id,
                0,
            )
            if sequence_count <= 1:
                self._nav_active_notification_sequences.pop(
                    ticket.ticket_id,
                    None,
                )
            else:
                self._nav_active_notification_sequences[
                    ticket.ticket_id
                ] = sequence_count - 1
            sequence_thread_key = (thread_id, ticket.ticket_id)
            sequence_thread_count = (
                self._nav_notification_sequence_threads.get(
                    sequence_thread_key,
                    0,
                )
            )
            if sequence_thread_count <= 1:
                self._nav_notification_sequence_threads.pop(
                    sequence_thread_key,
                    None,
                )
            else:
                self._nav_notification_sequence_threads[
                    sequence_thread_key
                ] = sequence_thread_count - 1
            subscription_count = self._nav_active_notification_subscriptions.get(
                subscription_id,
                0,
            )
            if subscription_count <= 1:
                self._nav_active_notification_subscriptions.pop(
                    subscription_id,
                    None,
                )
            else:
                self._nav_active_notification_subscriptions[
                    subscription_id
                ] = subscription_count - 1
            subscription_thread_key = (thread_id, subscription_id)
            subscription_thread_count = (
                self._nav_notification_subscription_threads.get(
                    subscription_thread_key,
                    0,
                )
            )
            if subscription_thread_count <= 1:
                self._nav_notification_subscription_threads.pop(
                    subscription_thread_key,
                    None,
                )
            else:
                self._nav_notification_subscription_threads[
                    subscription_thread_key
                ] = subscription_thread_count - 1
            self._nav_notification_condition.notify_all()

    def _retire_navigation_notification(
        self,
        ticket: _NavigationNotificationTicket,
    ) -> None:
        with self._nav_notification_condition:
            if (
                self._nav_pending_notification_tickets.get(ticket.ticket_id)
                is ticket
            ):
                self._nav_pending_notification_tickets.pop(ticket.ticket_id, None)
            self._nav_notification_condition.notify_all()

    def _wait_for_navigation_notifications(self) -> None:
        with self._nav_notification_condition:
            while self._nav_active_notification_count > 0:
                self._nav_notification_condition.wait()

    def _wait_for_navigation_subscription_callbacks(
        self,
        subscription_id: int,
    ) -> None:
        thread_id = threading.get_ident()
        subscription_thread_key = (thread_id, subscription_id)
        with self._nav_notification_condition:
            while self._nav_active_notification_subscriptions.get(
                subscription_id,
                0,
            ) > self._nav_notification_subscription_threads.get(
                subscription_thread_key,
                0,
            ):
                self._nav_notification_condition.wait()

    def _raise_if_navigation_notification_reentry(
        self,
        operation: str,
    ) -> None:
        thread_id = threading.get_ident()
        with self._nav_notification_condition:
            if self._nav_notification_threads.get(thread_id, 0) > 0:
                raise RuntimeError(
                    "%s is not allowed from a navigation callback"
                    % str(operation or "navigation connection transition")
                )

    def _transition_navigation_connection(
        self,
        *,
        connected: bool,
        message: Optional[str],
    ) -> None:
        """Gate delivery, drain started callbacks, then invalidate atomically."""

        self._raise_if_navigation_notification_reentry(
            "navigation connection transition"
        )
        with self._nav_connection_transition_lock:
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    self._nav_connection_transition_inflight = True
                self._cancel_pending_navigation_notifications_locked()

            self._wait_for_navigation_notifications()

            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    self._nav_connection_epoch += 1
                    self._nav_stream_epoch += 1
                    self._nav_command_generation += 1
                    self._nav_inflight_command_token = None
                    self._nav_completed_cancel_command_token = None
                    self._clear_navigation_set_precommit_locked()
                    self._nav_active_stream_token = None
                    stream_call = self._nav_stream_call
                    self._connected = bool(connected)
                    with self._nav_state_lock:
                        self._nav_stream_started = False
                        self._nav_stream_thread = None
                        unresolved_metadata = (copy.deepcopy(self._last_nav_metadata)
                            if self._last_nav_metadata.get("ownership_retained") is True else None)
                        historical_navigation = (self._last_nav_metadata.get("historical_navigation")
                            or self._navigation_cache_wire_locked()) if unresolved_metadata else None
                        historical_identity = (self._last_nav_metadata.get("historical_navigation_identity")
                            or _navigation_identity_document(self._current_navigation_lease_locked())) if unresolved_metadata else None
                        self._last_nav_goal_id = None
                        self._last_nav_owner_kind = 0
                        self._last_nav_parent_request_id = ""
                        self._last_nav_goal_generation = 0
                        self._last_nav_producer_runtime_generation = 0
                        # 新连接的空状态属于本次命令代次，允许其合法空闲心跳。
                        self._last_nav_goal_command_generation = self._nav_command_generation
                        self._last_nav_status = ""
                        self._last_nav_message = message
                        self._last_nav_distance_remaining = None
                        self._last_nav_current_pose = None
                        self._last_nav_timestamp_ns = 0
                        self._last_nav_metadata = {}
                        if unresolved_metadata is not None:
                            self._last_nav_goal_id = str(historical_identity["goal_id"])
                            self._last_nav_status = "unconfirmed"
                            self._last_nav_message = "原导航目标尚未结算；连接已更换，当前 exact 取消资格不可用。"
                            self._last_nav_timestamp_ns = int(historical_navigation.get("timestamp_ns") or 0)
                            self._last_nav_metadata = {
                                **unresolved_metadata,
                                "raw_status": "unconfirmed", "observed_goal_id": self._last_nav_goal_id,
                                "owner_kind": 0, "parent_request_id": "", "goal_generation": 0,
                                "producer_runtime_generation": 0,
                                "ownership_retained": True, "authority_current": False,
                                "historical_navigation_identity": copy.deepcopy(historical_identity),
                                "historical_navigation": copy.deepcopy(historical_navigation),
                                "connection_diagnostic": "NAVIGATION_CONNECTION_REPLACED",
                            }
                        self._nav_exploration_overlay = None
                        self._nav_exploration_overlay_terminal = False
                    self._nav_connection_transition_inflight = False
            if stream_call is not None:
                _cancel_stream_call(
                    stream_call,
                    context="navigation connection transition",
                )
                with self._nav_command_lock:
                    if self._nav_stream_call is stream_call:
                        self._nav_stream_call = None

    def connect(self) -> bool:
        """连接到 Rust 子进程的 gRPC server。

        失败时必须清空 channel/stub 并调 `bridge.disconnect()`，让下一次
        attach 重新执行端口与 gRPC 双门验活。attach 到失联进程时
        `bridge.is_running` 在 attach 状态短暂为 True（端口刚关但状态未清），
        RPC 失败后必须 reset bridge 状态，否则会留下永久 `_started=True`
        假活锁（参见 grpc-attach-zombie-bridge-2026-06-27）。
        """
        self._raise_if_navigation_notification_reentry("connect")
        # shutdown 后禁止重连——防止留下半连接 ghost channel。
        if self._stopping and _is_shutdown_guard_enabled():
            LOGGER.debug("shutdown 中，拒绝重新连接 gRPC")
            return False
        if not self._bridge.is_running:
            LOGGER.error("子进程未启动，无法连接")
            return False
        staged_channel = None
        try:
            self._raise_if_map_delivery_reentry_locked("connect")
            message_limit = _grpc_message_limit_bytes()
            staged_channel = grpc.insecure_channel(
                self._bridge.grpc_target,
                options=(
                    ("grpc.max_receive_message_length", message_limit),
                    ("grpc.max_send_message_length", message_limit),
                ),
            )
            staged_stub = pb2_grpc.GeometryRuntimeStub(staged_channel)
            resp = staged_stub.GetStatus(pb2.Empty(), timeout=_read_deadline_sec())
            LOGGER.info("gRPC 连接成功: phase=%s, rss=%dKB", resp.phase, resp.rss_kb)
            with self._map_delivery_lock:
                with self._map_state_lock:
                    self._channel = staged_channel
                    self._stub = staged_stub
                    retired_attempt = self._invalidate_map_connection_locked()
                    self._assigned_fine_archive_versions = frozenset(
                        resp.supported_assigned_fine_archive_versions)
                self._invalidate_map_delivery_tickets_locked()
            del retired_attempt
            self._wait_for_map_deliveries()
            self._transition_navigation_connection(
                connected=True,
                message=None,
            )
            return True
        except Exception as exc:
            LOGGER.error("gRPC 连接失败: %s", exc)
            if staged_channel is not None and staged_channel is not self._channel:
                try:
                    staged_channel.close()
                except Exception:
                    LOGGER.exception("关闭未提交的 gRPC channel 异常。")
            try:
                self._cleanup_after_failed_connect()
            except Exception:
                LOGGER.exception("gRPC connect cleanup 异常，执行强制清理。")
                channel = self._channel
                if channel is not None:
                    try:
                        channel.close()
                    except Exception:
                        LOGGER.exception("强制关闭失败的 gRPC channel 异常。")
                with self._map_state_lock:
                    self._channel = None
                    self._stub = None
                    self._connected = False
                    self._stopping = True
                try:
                    self._bridge.disconnect()
                except Exception:
                    LOGGER.exception("强制 reset geometry bridge 异常。")
            return False

    def _cleanup_after_failed_connect(self) -> None:
        """connect 失败时释放已建 channel/stub 并强制 bridge 进入未启动状态。"""
        self._raise_if_map_delivery_reentry_locked("failed-connect cleanup")
        with self._map_delivery_lock:
            with self._map_state_lock:
                self._stopping = True
                channel = self._channel
                self._channel = None
                self._stub = None
                retired_attempt = self._invalidate_map_connection_locked()
            self._invalidate_map_delivery_tickets_locked()
        del retired_attempt
        if channel is not None:
            try:
                channel.close()
            except Exception:  # pragma: no cover - 关 channel 失败不影响 reset 目标
                pass
        self._cancel_data_stream_handles()
        self._wait_for_map_deliveries()
        self._transition_navigation_connection(
            connected=False,
            message="Geometry runtime 连接失败。",
        )
        # 关键：让 attach handle 回到未连接状态；进程仍由 canonical launcher 管理。
        try:
            self._bridge.disconnect()
        except Exception as exc:  # pragma: no cover - disconnect 失败不影响主流程
            LOGGER.warning("bridge.disconnect 失败（不影响主流程）: %s", exc)

    def disconnect(self) -> None:
        # RLock 同线程可重入，先做 owner 检查，避免 callback 内调用 disconnect
        # 在触碰订阅或连接状态后才失败。
        self._raise_if_navigation_notification_reentry("disconnect")
        self._raise_if_map_delivery_reentry_locked("disconnect")
        with self._exploration_state_lock:
            exploration_stop_event = self._exploration_poll_stop_event
            exploration_thread = self._exploration_poll_thread
            self._exploration_update_subscribers.clear()
            if exploration_stop_event is not None:
                exploration_stop_event.set()
        with self._map_delivery_lock:
            with self._map_state_lock:
                self._stopping = True
                channel = self._channel
                self._channel = None
                self._stub = None
                retired_attempt = self._invalidate_map_connection_locked()
            self._invalidate_map_delivery_tickets_locked()
        del retired_attempt
        if channel is not None:
            channel.close()
        self._cancel_data_stream_handles()
        self._wait_for_map_deliveries()
        self._transition_navigation_connection(
            connected=False,
            message="Geometry runtime 已断开。",
        )
        if (
            exploration_thread is not None
            and exploration_thread.is_alive()
            and exploration_thread is not threading.current_thread()
        ):
            exploration_thread.join(timeout=_read_deadline_sec() + 0.5)

    def _discard_pose_stream_handle(
        self,
        handle: _CancellableStreamHandle,
    ) -> None:
        with self._stream_handle_lock:
            self._pose_stream_handles.discard(handle)

    def _discard_map_stream_handle(
        self,
        handle: _CancellableStreamHandle,
    ) -> None:
        with self._stream_handle_lock:
            self._map_stream_handles.discard(handle)

    def _cancel_data_stream_handles(self) -> None:
        with self._stream_handle_lock:
            handles = tuple(
                self._pose_stream_handles | self._map_stream_handles
            )
        for handle in handles:
            handle.cancel()

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def connection_generation(self) -> int:
        """Return the generation used to reject stale in-flight RPC results."""

        with self._map_state_lock:
            return int(self._map_connection_epoch)

    def _raise_if_stopping(self, rpc_name: str) -> None:
        """shutdown 守卫：_stopping=True 且 guard 启用时，所有 RPC 入口抛 shutting_down。

        防止 shutdown 期间 in-flight task 持陈旧 _stub 引用继续打 RPC 全 UNAVAILABLE。
        调用方（task runner 等）应 catch RuntimeError 优雅退出（AC4 期望 DEBUG 日志）。
        通过 NUWAX_GEOMETRY_SHUTDOWN_GUARD=0 可紧急回滚禁用此守卫。
        """
        if self._stopping and _is_shutdown_guard_enabled():
            LOGGER.debug("GeometryRuntimeProvider.%s 在 shutdown 期被阻断", rpc_name)
            raise RuntimeError("shutting_down")

    def is_available(self) -> bool:
        self._raise_if_stopping("is_available")
        if not self._connected or not self._stub:
            return False
        try:
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_pose_deadline_sec())
            return len(resp.transforms) > 0
        except grpc.RpcError as e:
            _log_rpc_error("GetFrameTree(is_available)", e)
            return False

    def is_localization_available(self) -> bool:
        return self.is_available()

    # === Phase 3: mapping / lcm backend diagnostics ===
    @staticmethod
    def _mapping_state_payload(response) -> Dict[str, Any]:
        activation_disposition = int(
            response.activation_receipt_disposition
        )
        try:
            activation_disposition_name = pb2.ActivationReceiptDisposition.Name(
                activation_disposition
            )
            activation_disposition_prefix = "ACTIVATION_RECEIPT_DISPOSITION_"
            if activation_disposition_name.startswith(activation_disposition_prefix):
                activation_disposition_name = activation_disposition_name[
                    len(activation_disposition_prefix) :
                ]
            activation_disposition_name = activation_disposition_name.lower()
        except ValueError:
            activation_disposition_name = "unknown"
        if response.activation_receipt_schema_version == 0:
            return _unactivated_mapping_state_payload(response, activation_disposition, activation_disposition_name)
        if response.activation_receipt_schema_version == 2:
            return _assigned_mapping_state_payload(response, activation_disposition, activation_disposition_name)
        raise ValueError("unsupported activation receipt schema")

    @staticmethod
    def _relocalize_result_payload(response) -> Dict[str, Any]:
        return _assigned_terminal_payload(AssignedRelocalizationTerminal.from_proto(response))

    @staticmethod
    def _mapping_identity_from_payload(payload: Dict[str, Any]) -> _MappingIdentity:
        identity = (
            str(payload.get("session_id") or "").strip(),
            str(payload.get("map_name") or "").strip(),
            int(payload.get("session_generation") or 0),
        )
        if not identity[0] or not identity[1] or identity[2] <= 0:
            raise ValueError("mapping identity is incomplete")
        return identity

    def _begin_mapping_transition(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> int:
        _exploration_rpc_timeout(deadline_monotonic, "StartMapping")
        with _cooperative_deadline_lock(
            self._map_delivery_lock,
            deadline_monotonic,
            "StartMapping",
        ):
            self._raise_if_map_delivery_reentry_locked("StartMapping")
            with _cooperative_deadline_lock(
                self._map_state_lock,
                deadline_monotonic,
                "StartMapping",
            ):
                if self._stopping:
                    raise RuntimeError("provider is disconnecting")
                if self._mapping_transition_inflight:
                    raise RuntimeError("another StartMapping transition is in flight")
                self._mapping_transition_epoch += 1
                self._mapping_transition_inflight = True
                transition_epoch = self._mapping_transition_epoch
            self._invalidate_map_delivery_tickets_locked()
        try:
            self._wait_for_map_deliveries(
                deadline_monotonic=deadline_monotonic,
            )
        except BaseException:
            try:
                self._finish_mapping_transition(
                    transition_epoch, deadline_monotonic=deadline_monotonic,
                )
            except BaseException:
                LOGGER.exception("Mapping transition release failed after interruption.")
            raise
        return transition_epoch

    def _finish_mapping_transition(
        self,
        transition_epoch: int,
        *,
        identity: Optional[_MappingIdentity] = None,
        input_active: Optional[bool] = None,
        expected_origin=None,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        with _cooperative_post_rpc_lock(
            self._map_delivery_lock,
            deadline_monotonic,
            "StartMapping",
        ):
            with _cooperative_post_rpc_lock(
                self._map_state_lock,
                deadline_monotonic,
                "StartMapping",
            ):
                if (
                    self._stopping
                    or transition_epoch != self._mapping_transition_epoch
                    or not self._mapping_transition_inflight
                    or (expected_origin is not None and (
                        self._stub is not expected_origin[0]
                        or self._map_connection_epoch != expected_origin[1]
                        or not self._connected
                    ))
                ):
                    return False
                if identity is not None:
                    if self._mapping_identity != identity:
                        self._last_occupancy_grid = None
                        self._last_occupancy_grid_identity = None
                    self._mapping_identity = identity
                if input_active is not None:
                    self._planar_input_active = input_active
                self._mapping_transition_inflight = False
                return True

    def _invalidate_map_connection_locked(self) -> Optional[_AssignedActivationAttempt]:
        # 调用方在两个锁外释放旧载荷；断连撤销重试资格，UNKNOWN 不因此变成未执行。
        retired_attempt = self._assigned_activation_attempt
        self._assigned_activation_attempt = None
        self._map_connection_epoch += 1
        self._assigned_fine_archive_versions = frozenset()
        self._mapping_transition_epoch += 1
        self._mapping_transition_inflight = False
        self._mapping_identity = None
        self._planar_input_active = False
        self._last_occupancy_grid = None
        self._last_occupancy_grid_identity = None
        return retired_attempt

    def _raise_if_map_delivery_reentry_locked(self, operation: str) -> None:
        thread_id = threading.get_ident()
        with self._map_delivery_condition:
            if any(
                ticket.owner_thread_id == thread_id
                for ticket in self._map_delivery_tickets.values()
            ):
                raise RuntimeError(
                    "%s is not allowed from the active map callback thread"
                    % str(operation or "mapping identity transition")
                )

    def _claim_map_delivery(
        self,
        payload: Dict[str, Any],
    ) -> Optional[_MapDeliveryTicket]:
        with self._map_delivery_lock:
            with self._map_state_lock:
                if not self._map_payload_is_current_locked(payload):
                    return None
                delivery_token = self._map_delivery_token_locked()
            with self._map_delivery_condition:
                ticket_id = self._next_map_delivery_ticket_id
                self._next_map_delivery_ticket_id += 1
                ticket = _MapDeliveryTicket(
                    ticket_id=ticket_id,
                    delivery_token=delivery_token,
                    owner_thread_id=threading.get_ident(),
                )
                self._map_delivery_tickets[ticket_id] = ticket
                self._map_delivery_in_progress = True
                self._map_delivery_owner_thread_id = ticket.owner_thread_id
                return ticket

    def _finish_map_delivery(self, ticket: _MapDeliveryTicket) -> None:
        with self._map_delivery_condition:
            if self._map_delivery_tickets.get(ticket.ticket_id) is ticket:
                self._map_delivery_tickets.pop(ticket.ticket_id, None)
            remaining = tuple(self._map_delivery_tickets.values())
            self._map_delivery_in_progress = bool(remaining)
            self._map_delivery_owner_thread_id = (
                remaining[0].owner_thread_id if len(remaining) == 1 else None
            )
            self._map_delivery_condition.notify_all()

    def _invalidate_map_delivery_tickets_locked(self) -> None:
        with self._map_delivery_condition:
            for ticket in self._map_delivery_tickets.values():
                ticket.invalidated = True
            self._map_delivery_condition.notify_all()

    def _wait_for_map_deliveries(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> None:
        thread_id = threading.get_ident()
        with self._map_delivery_condition:
            while any(
                ticket.owner_thread_id != thread_id
                for ticket in self._map_delivery_tickets.values()
            ):
                if deadline_monotonic is None:
                    self._map_delivery_condition.wait()
                else:
                    timeout = _exploration_rpc_timeout(
                        deadline_monotonic,
                        "StartMapping",
                    )
                    self._map_delivery_condition.wait(timeout=timeout)

    def _map_delivery_token_locked(self):
        return (
            self._map_connection_epoch,
            self._mapping_transition_epoch,
            self._mapping_identity,
        )

    def _map_payload_is_current_locked(self, payload: Dict[str, Any]) -> bool:
        return bool(
            not self._stopping
            and not self._mapping_transition_inflight
            and isinstance(payload, _MapUpdatePayload)
            and payload.delivery_token == self._map_delivery_token_locked()
        )

    def start_mapping(
        self,
        *,
        session_id: str,
        map_name: str,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        self._raise_if_stopping("start_mapping")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        stub = self._stub
        _exploration_rpc_timeout(deadline_monotonic, "StartMapping")
        transition_epoch = self._begin_mapping_transition(
            deadline_monotonic=deadline_monotonic,
        )
        try:
            rpc_timeout = _exploration_rpc_timeout(
                deadline_monotonic,
                "StartMapping",
            )
            response = stub.StartMapping(
                pb2.StartMappingRequest(
                    session_id=str(session_id or "").strip(),
                    map_name=str(map_name or "").strip(),
                ),
                timeout=rpc_timeout,
            )
            payload = self._mapping_state_payload(response)
            if (
                not payload["success"]
                or payload["phase"] != "ACTIVE"
                or not payload["active"]
                or payload["session_id"] != str(session_id or "").strip()
                or payload["map_name"] != str(map_name or "").strip()
                or payload["session_generation"] <= 0
            ):
                raise RuntimeError("StartMapping 未返回匹配的 ACTIVE 会话。")
            identity = self._mapping_identity_from_payload(payload)
        except Exception:
            self._finish_mapping_transition(
                transition_epoch,
                deadline_monotonic=deadline_monotonic,
            )
            raise
        if not self._finish_mapping_transition(
            transition_epoch,
            identity=identity,
            input_active=True,
            deadline_monotonic=deadline_monotonic,
        ):
            raise RuntimeError(
                "StartMapping 完成时 mapping transition 已被断连或新代际取代。"
            )
        return payload

    def _graph_transfer(self, action, reference, *, offset=0, data=b"", stub=None):
        request = pb2.GraphArtifactTransferRequest(action=action, artifact=reference, offset=offset, data=data)
        origin_stub = self._stub if stub is None else stub
        for attempt in range(3):
            try:
                response = origin_stub.TransferGraphArtifact(
                    request, timeout=_heavy_deadline_sec())
                break
            except grpc.RpcError as error:
                if attempt == 2 or error.code() not in {grpc.StatusCode.DEADLINE_EXCEEDED, grpc.StatusCode.UNAVAILABLE}:
                    raise
                # 相同制品、偏移和字节重放，不能换身份重写未决分块。
        if not response.HasField("artifact") or response.artifact != reference:
            raise RuntimeError("二维图传输回执身份冲突")
        return response

    def _download_graph_artifact(self, reference, maximum_bytes=MAX_PLANAR_ARCHIVE_BYTES, *, validate_structure=True,
                                 _stub_override=None, _release_on_completion=True):
        retained = _artifact_reference_from_proto(reference, maximum_bytes)
        if retained.member_identity.role not in ("planar_graph", "fine_map", "dense_cloud"):
            raise ValueError("unpaired assigned artifact role")
        transfer_options = {"stub": _stub_override} if _stub_override is not None else {}
        def chunks():
            offset = 0
            while offset < reference.size:
                response = self._graph_transfer(4, reference, offset=offset, **transfer_options)
                data = bytes(response.data)
                if response.offset != reference.size or not response.sealed or not data or len(data) > min(1024 * 1024, reference.size - offset):
                    raise RuntimeError("二维图下载分块非法")
                offset += len(data)
                yield data
        try:
            payload = FrozenPayload(chunks(), maximum_bytes=maximum_bytes)
            if len(payload) != reference.size:
                raise ValueError("artifact downloaded size mismatch")
            if validate_structure:
                identity = retained.member_identity
                if identity.role == "planar_graph":
                    read_planar_structure(payload, checkpoint_id=identity.checkpoint_id,
                        session_id=identity.occupancy_receipt["session_id"],
                        generation=identity.occupancy_receipt["session_generation"],
                        timestamp_ns=identity.occupancy_receipt["source_timestamp_ns"])
                    return payload
                keywords = dict(session=identity.occupancy_receipt["session_id"],
                    generation=identity.occupancy_receipt["session_generation"], cloud_revision=identity.cloud_receipt["cloud_revision"],
                    source_timestamp_ns=identity.cloud_receipt["source_timestamp_ns"])
                if identity.role == "fine_map":
                    if payload[:8] in (b"NUWAXAR1", b"NUWAXAR4", b"NUWAXAR5"):
                        version = int(payload[7:8])
                        origin = self.capture_assigned_activation_origin()
                        if _stub_override is not None and origin[0] is not _stub_override:
                            raise RuntimeError("assigned fine download connection changed")
                        if not self.supports_assigned_fine_archive_version(version, expected_origin=origin):
                            raise RuntimeError("runtime does not support assigned AR%d source archives" % version)
                    validate_fine_archive(payload, **keywords, frame_generation=identity.frame_generation,
                        producer_generation=identity.producer_generation)
                elif identity.role == "dense_cloud":
                    validate_dense_archive(payload, **keywords, frame_id=identity.frame_id)
                else:
                    raise ValueError("unpaired assigned artifact role")
            return payload
        finally:
            if _release_on_completion:
                self._graph_transfer(5, reference, **transfer_options)

    def _upload_graph_artifact(self, checkpoint, *, payload=None, reference=None, transfer=None,
                               expected_origin=None):
        transfer = self._graph_transfer if transfer is None else transfer
        payload = checkpoint.payload if payload is None else payload
        if reference is None:
            reference = (checkpoint.graph_reference if checkpoint.backend_kind == "slam_toolbox" and payload is checkpoint.payload
                else checkpoint.fine_reference if payload is checkpoint.fine_archive
                else checkpoint.dense_reference if payload is checkpoint.dense_archive else None)
        if reference is None:
            raise ValueError("v2 upload requires the retained artifact reference")
        if not isinstance(reference, CheckpointArtifactReference):
            reference = _artifact_reference_from_proto(reference, max(MAX_DENSE_ARCHIVE_BYTES, len(payload)))
        validate_artifact_reference(reference, checkpoint=checkpoint, maximum_bytes=MAX_DENSE_ARCHIVE_BYTES)
        if reference == checkpoint.graph_reference:
            retained_payload = checkpoint.payload
            if not isinstance(payload, FrozenPayload) or payload is not retained_payload:
                raise ValueError("planar upload requires its retained immutable payload owner")
        elif reference == checkpoint.fine_reference:
            retained_payload = checkpoint.fine_archive
            if retained_payload[:8] in (b"NUWAXAR1", b"NUWAXAR4", b"NUWAXAR5"):
                self.require_assigned_fine_archive_support(checkpoint,
                    expected_origin=expected_origin or self.capture_assigned_activation_origin())
        elif reference == checkpoint.dense_reference:
            retained_payload = checkpoint.dense_archive
        else:
            raise ValueError("upload reference is not a retained checkpoint artifact")
        if (not isinstance(payload, (bytes, FrozenPayload)) or len(payload) != reference.size
                or not isinstance(retained_payload, (bytes, FrozenPayload)) or len(retained_payload) != reference.size):
            raise ValueError("artifact upload or retained owner type/size mismatch")
        if payload is not retained_payload and any(left != right for left, right in zip(
                payload_chunks(payload), payload_chunks(retained_payload))):
            raise ValueError("upload bytes differ from the retained immutable artifact")
        reference = _artifact_reference_proto(reference)
        try:
            admitted = transfer(1, reference)
            frontier = int(admitted.offset)
            sealed = bool(admitted.sealed)
            if not 0 <= frontier <= reference.size or (sealed and frontier != reference.size):
                raise RuntimeError("artifact initialization frontier is invalid")
            offset = 0
            for data in payload_chunks(payload):
                response = transfer(2, reference, offset=offset, data=data)
                offset += len(data)
                # Retained frontier can lead the current replayed chunk.
                if (not max(frontier, offset) <= response.offset <= reference.size
                        or (response.sealed and response.offset != reference.size)
                        or (sealed and not response.sealed)):
                    raise RuntimeError("二维图上传位置回执不一致")
                frontier = int(response.offset)
                sealed = bool(response.sealed)
            complete = transfer(3, reference)
            if not complete.sealed or complete.offset != reference.size:
                raise RuntimeError("二维图上传未封存")
            return reference
        except BaseException:
            try:
                transfer(5, reference)
            except BaseException:
                LOGGER.exception("Graph artifact release failed after upload failure.")
            raise

    def export_mapping_backend_checkpoint(
        self,
    ) -> MappingBackendCheckpointExportBundle:
        """导出当前分配式检查点，并将制品读取固定在原连接。"""

        self._raise_if_stopping("export_mapping_backend_checkpoint")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        origin = self.capture_assigned_activation_origin()
        origin_stub = origin[0]
        rpc = origin_stub.ExportMappingBackendCheckpoint
        try:
            response = rpc(pb2.ExportMappingBackendCheckpointRequest(), timeout=_heavy_deadline_sec())
        except grpc.RpcError as exc:
            raise GeometryRuntimeRpcError.from_rpc_error(
                "ExportMappingBackendCheckpoint",
                exc,
            ) from exc
        if response.schema_version != 2:
            raise ValueError("server returned a different checkpoint schema; downgrade is forbidden")
        if response.backend_kind not in {"voxel", "raytracing", "slam_toolbox"}:
            raise ValueError("assigned checkpoint export requires a paired backend")
        _validate_checkpoint_identity_envelope(response)
        # The export owns all returned leases. Pin reads and cleanup to its
        # origin even if this provider is replaced before the first read.
        references = {}
        completed = False
        cleanup_error = None
        try:
            for field_name, role, maximum in (
                ("graph_artifact", "planar_graph", MAX_PLANAR_ARCHIVE_BYTES),
                ("fine_map_artifact", "fine_map", MAX_FINE_ARCHIVE_BYTES),
                ("dense_cloud_artifact", "dense_cloud", MAX_DENSE_ARCHIVE_BYTES),
            ):
                if response.HasField(field_name):
                    reference = getattr(response, field_name)
                    retained = _artifact_reference_from_proto(reference, maximum)
                    if (retained.identity_schema_version != 2
                            or retained.member_identity.checkpoint_id != response.checkpoint_id
                            or retained.member_identity.role != role):
                        raise ValueError("assigned export artifact differs from its checkpoint role")
                    if any(reference.id == previous.id for previous in references.values()):
                        raise ValueError("distinct checkpoint members cannot reuse one artifact ID")
                    references[field_name] = reference
            self.require_assigned_activation_origin(origin)
            fine_payload = self._download_graph_artifact(
                references["fine_map_artifact"], MAX_FINE_ARCHIVE_BYTES,
                validate_structure=False, _stub_override=origin_stub, _release_on_completion=False,
            ) if "fine_map_artifact" in references else None
            self.require_assigned_activation_origin(origin)
            dense_payload = self._download_graph_artifact(
                references["dense_cloud_artifact"], MAX_DENSE_ARCHIVE_BYTES,
                validate_structure=False, _stub_override=origin_stub, _release_on_completion=False,
            ) if "dense_cloud_artifact" in references else None
            self.require_assigned_activation_origin(origin)
            graph_payload = self._download_graph_artifact(
                references["graph_artifact"], MAX_PLANAR_ARCHIVE_BYTES,
                validate_structure=False, _stub_override=origin_stub, _release_on_completion=False,
            ) if "graph_artifact" in references else None
            self.require_assigned_activation_origin(origin)
            exported = _checkpoint_export_bundle_from_proto(response, graph_payload, fine_payload, dense_payload)
            self.require_assigned_activation_origin(origin)
            completed = True
        finally:
            for reference in references.values():
                try:
                    self._graph_transfer(5, reference, stub=origin_stub)
                except BaseException as error:
                    if cleanup_error is None:
                        cleanup_error = error
                    LOGGER.exception("Assigned export artifact release failed.")
            if completed and cleanup_error is not None:
                raise cleanup_error
        self.require_assigned_activation_origin(origin)
        return exported

    def get_dense_cloud_chunk(self, *, expected_epoch="", offset_points=0, allow_delta=False):
        """读取当前规范提交中的完整扫描前缀，不触发启动或建图。"""
        self._raise_if_stopping("get_dense_cloud_chunk")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry运行时未连接")
        with self._map_delivery_condition:
            if self._mapping_transition_inflight:
                raise RuntimeError("地图正在切换，完整点云暂不可用")
            token = self._map_delivery_token_locked()
        try:
            response = self._stub.GetDenseCloudChunk(pb2.DenseCloudChunkRequest(
                expected_epoch=expected_epoch, offset_points=offset_points, max_points=DENSE_CHUNK_POINTS, allow_delta=allow_delta,
            ), timeout=5.0)
        except grpc.RpcError as error:
            raise GeometryRuntimeRpcError.from_rpc_error("GetDenseCloudChunk", error) from error
        with self._map_delivery_condition:
            if token != self._map_delivery_token_locked() or self._mapping_transition_inflight:
                raise RuntimeError("DENSE_CLOUD_SOURCE_CHANGED")
        return dense_chunk_payload(response, expected_epoch=expected_epoch, offset_points=offset_points, allow_delta=allow_delta)

    def _selected_mapping_backend_state(self) -> Dict[str, Any]:
        state = self.get_mapping_state()
        backend_kind = str(state.get("mapping_backend_kind") or "").strip()
        if backend_kind not in _SUPPORTED_MAPPING_BACKEND_KINDS:
            raise RuntimeError(
                "Geometry runtime returned unsupported mapping_backend_kind: %s"
                % (backend_kind or "<empty>")
            )
        return state

    def activate_assigned_platform_map(
        self, *, activation_request, stored_checkpoint: StoredMappingBackendCheckpoint,
        expected_origin=None,
    ) -> Dict[str, Any]:
        """显式加载 schema-2；调用方保留原六字段请求，不改写代际或自动重执行。

        仅接纳已停止/空闲后端或同会话精确重放；切换活跃地图必须由调用方先显式停止。
        """
        self._raise_if_stopping("activate_assigned_platform_map")
        retained = _admit_assigned_activation_request(activation_request)
        if not isinstance(stored_checkpoint, StoredMappingBackendCheckpoint):
            raise TypeError("assigned activation requires a stored checkpoint owner")
        checkpoint = stored_checkpoint.checkpoint
        request_key = tuple(retained[name] for name in (
            "activation_request_id", "session_id", "map_name", "checkpoint_id", "expected_runtime_generation")) + (
                retained["asset_identity"]["asset_id"],)
        if checkpoint.checkpoint_id != retained["checkpoint_id"]:
            raise ValueError("assigned activation checkpoint differs from the retained request")
        from contracts.map_identity import AssetIdentityV2
        request = _assigned_activation_request_proto(
            stored_checkpoint, asset_identity=AssetIdentityV2(**retained["asset_identity"]),
            **{name: retained[name] for name in ("session_id", "map_name", "activation_request_id", "expected_runtime_generation")},
        )
        with self._map_state_lock:
            if not self._connected or self._stub is None or self._stopping:
                raise RuntimeError("Geometry runtime 未连接。")
            origin = (self._stub, self._map_connection_epoch)
            if expected_origin is not None and (
                    origin[0] is not expected_origin[0] or origin[1] != expected_origin[1]):
                raise RuntimeError("assigned activation connection changed before dispatch")
            pending = self._assigned_activation_attempt
        stub = origin[0]
        self.require_assigned_fine_archive_support(checkpoint, expected_origin=origin)
        transition = self._begin_mapping_transition()
        dispatched = False
        acknowledged = None
        attempted_artifacts = []

        def require_origin():
            with self._map_state_lock:
                if (self._stub is not stub or self._map_connection_epoch != origin[1]
                        or not self._connected or self._stopping
                        or self._mapping_transition_epoch != transition
                        or not self._mapping_transition_inflight
                        or self._assigned_activation_attempt is not pending):
                    raise RuntimeError("assigned activation connection or transition changed")

        def transfer(action, reference, **kwargs):
            require_origin()
            return self._graph_transfer(action, reference, stub=stub, **kwargs)

        def release_artifacts():
            # 保持 transition 预约直到租用清理结束，避免删除后继重放刚上传的同名制品。
            while attempted_artifacts:
                reference = attempted_artifacts.pop()
                try:
                    self._graph_transfer(5, reference, stub=stub)
                except BaseException:
                    LOGGER.exception("Assigned activation artifact release failed.")

        def invalidate_identity():
            with self._map_state_lock:
                if (self._mapping_transition_epoch == transition
                        and self._stub is stub and self._map_connection_epoch == origin[1]):
                    self._mapping_identity = None
                    self._last_occupancy_grid = None
                    self._last_occupancy_grid_identity = None
                    self._planar_input_active = False

        try:
            require_origin()
            if pending is not None:
                if not pending.matches(request_key, stored_checkpoint, origin):
                    raise ValueError("unresolved assigned activation requires the exact retained request/checkpoint and connection")
                # 只有本连接已 dispatch 的原请求允许略过状态读；原生未决 fence 可能占有状态读锁。
                # 这是调用方本次显式重试，不是丢失回复后的自动重执行。
            else:
                state = self._mapping_state_payload(stub.GetMappingState(pb2.Empty(), timeout=_read_deadline_sec()))
                require_origin()
                if (not state["success"] or state["phase"] not in {"IDLE", "ACTIVE", "STOPPED"}
                        or state["active"] != (state["phase"] == "ACTIVE")
                        or state["mapping_backend_kind"] != checkpoint.backend_kind
                        or state["runtime_generation"] != retained["expected_runtime_generation"]):
                    raise ValueError("assigned activation preflight differs from the retained runtime/backend")
                if state["session_id"] == retained["session_id"] and (
                        state["phase"] == "STOPPED" or state["map_name"] != retained["map_name"]):
                    raise GeometryRuntimeMapActivationStaleRequest(
                        activation_request_id=retained["activation_request_id"], map_name=retained["map_name"],
                        current_phase=state["phase"], current_map_name=state["map_name"],
                    )
                if state["active"]:
                    self._mapping_identity_from_payload(state)
                    if state["session_id"] != retained["session_id"]:
                        raise RuntimeError("assigned activation requires an explicit predecessor stop")
                    if state["activation_receipt_schema_version"] != 2:
                        raise ValueError("active assigned replay requires its retained request owner")
                    current_receipt = state["activation_receipt_v2"]
                    current = (current_receipt.to_dict() if isinstance(current_receipt, AssignedActivationReceipt)
                               else current_receipt)
                    planar = checkpoint.backend_kind == "slam_toolbox"
                    source_key = "source_graph_member" if planar else "source_cloud_member"
                    source_identity = checkpoint.graph_reference.member_identity if planar else checkpoint.cloud_identity
                    if (current["request"] != retained or (current["target"] is not None
                            and current["target"][source_key] != _identity_document(source_identity))):
                        raise ValueError("active assigned replay differs from its retained request/source owner")
                    # STAGED 没有完整 target 时也只发送调用方原请求，由原生账本裁定显式重放。
            require_origin()
            # 只重放原始制品身份及不可变字节；不生成摘要或替代 ID。
            for archive, reference in ((checkpoint.payload, checkpoint.graph_reference),
                                       (checkpoint.fine_archive, checkpoint.fine_reference),
                                       (checkpoint.dense_archive, checkpoint.dense_reference)):
                if reference is not None:
                    attempted_artifacts.append(_artifact_reference_proto(reference))
                    self._upload_graph_artifact(checkpoint, payload=archive, reference=reference, transfer=transfer,
                        expected_origin=origin)
            require_origin()
            dispatched = True
            response = stub.ActivateMap(request, timeout=_heavy_deadline_sec())
            acknowledged = _assigned_activation_ack(
                self._mapping_state_payload(response), activation_request=retained,
            )
            with self._map_state_lock:
                if (self._stub is stub and self._map_connection_epoch == origin[1]
                        and self._mapping_transition_epoch == transition
                        and self._assigned_activation_attempt is pending):
                    self._assigned_activation_attempt = None
            release_artifacts()
            if not self._finish_mapping_transition(
                    transition, identity=self._mapping_identity_from_payload(acknowledged),
                    input_active=False, expected_origin=origin):
                raise RuntimeError("assigned activation acknowledgement outlived its local connection")
            return {**acknowledged, "backend_switch_applied": True,
                    "backend_activation_disposition": "applied"}
        except BaseException as exc:
            if dispatched:
                invalidate_identity()
                if acknowledged is not None:
                    error = GeometryRuntimeMapActivationAppliedError(
                        "Assigned activation was acknowledged, but local adoption failed: %s" % exc)
                    error.activation_receipt_v2 = acknowledged["activation_receipt_v2"]
                    error.activation_evidence = acknowledged
                else:
                    with self._map_state_lock:
                        if (self._stub is stub and self._map_connection_epoch == origin[1]
                                and self._mapping_transition_epoch == transition
                                and self._assigned_activation_attempt is pending):
                            self._assigned_activation_attempt = _AssignedActivationAttempt(
                                request_key, stored_checkpoint, stub, origin[1])
                    error = GeometryRuntimeMapActivationOutcomeUnknown(
                        "Assigned activation dispatch or acknowledgement is unresolved: %s" % exc)
                    error.backend_activation_disposition = "unknown"
                error.activation_request = retained
                error.activation_request_id = retained["activation_request_id"]
                error.map_name = retained["map_name"]
                error.backend_changed = True
                error.backend_stop_confirmed = False
                raise error from exc
            raise
        finally:
            # 清理始终发往原连接；清理失败不能覆盖 ACK、UNKNOWN 或原始异常。
            release_artifacts()
            try:
                self._finish_mapping_transition(transition)
            except BaseException:
                LOGGER.exception("Assigned activation transition release failed.")



    def relocalize(
        self,
        *,
        activation_state: Mapping[str, object],
        request_id: Optional[str] = None,
        logical_timeout_ms: int = 10_000,
        planar_seed: Optional[Tuple[float, float, float]] = None,
        assigned_request_v2: Optional[Union[Mapping[str, object], AssignedRelocalizationRequest]] = None,
    ) -> Dict[str, Any]:
        self._raise_if_stopping("relocalize")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        rpc = self._stub.Relocalize

        state = dict(activation_state)
        assigned = _assigned_request_for_rpc(
            activation_state=state, assigned_request_v2=assigned_request_v2,
            request_id=request_id, logical_timeout_ms=logical_timeout_ms, query_only=False, planar_seed=planar_seed,
        )
        request = assigned.to_proto(pb2.RelocalizeRequest)
        try:
            response = rpc(request, timeout=max(_heavy_deadline_sec(), assigned.logical_timeout_ms / 1000.0 + 1.0))
        except grpc.RpcError as exc:
            raise GeometryRuntimeRpcError.from_rpc_error("Relocalize", exc) from exc
        terminal = AssignedRelocalizationTerminal.from_proto(response)
        if not assigned.exact_replay_of(terminal.request):
            raise ValueError("assigned terminal belongs to a different exact request")
        return _assigned_terminal_payload(terminal)

    def get_relocalization_terminal(
        self,
        *,
        activation_state: Mapping[str, object],
        request_id: str,
        logical_timeout_ms: int,
        planar_seed: Optional[Tuple[float, float, float]] = None,
        assigned_request_v2: Optional[Union[Mapping[str, object], AssignedRelocalizationRequest]] = None,
    ) -> Dict[str, Any]:
        """只读查询已存在 request 的权威终态，不创建或推进请求。"""

        self._raise_if_stopping("get_relocalization_terminal")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        rpc = self._stub.GetRelocalizationTerminal
        state = dict(activation_state)
        assigned = _assigned_request_for_rpc(
            activation_state=state, assigned_request_v2=assigned_request_v2,
            request_id=request_id, logical_timeout_ms=logical_timeout_ms, query_only=True, planar_seed=planar_seed,
        )
        query = AssignedRelocalizationQuery.from_dict({
            "schema_version": 2, "request_id": assigned.request_id,
            "assigned_request_v2": assigned.to_dict(),
        })
        try:
            response = rpc(query.to_proto(pb2.GetRelocalizationTerminalRequest), timeout=_read_deadline_sec())
        except grpc.RpcError as exc:
            raise GeometryRuntimeRpcError.from_rpc_error("GetRelocalizationTerminal", exc) from exc
        terminal = query.validate_proto_response(response)
        observation = None
        if response.HasField("ledger_observation"):
            candidate = _project_assigned_proto(response.ledger_observation, "observation")
            if (candidate["observation_schema_version"] == 1 and candidate["observation_epoch"] != 0
                    and candidate["terminal_record_count"] + candidate["pending_record_count"]
                    <= candidate["admitted_command_count"]):
                observation = candidate
        return {
            "schema_version": 2, "request_id": assigned.request_id,
            "status": {1: "terminal", 2: "pending", 3: "not_found", 4: "identity_conflict"}[response.status],
            "assigned_request_v2": assigned.to_dict(),
            "terminal": _assigned_terminal_payload(terminal) if terminal is not None else None,
            "ledger_observation": observation,
        }

    def capture_assigned_activation_origin(self):
        with self._map_state_lock:
            if not self._connected or self._stub is None or self._stopping:
                raise RuntimeError("assigned activation provider is disconnected")
            return self._stub, self._map_connection_epoch

    def supports_assigned_fine_archive_version(self, version, *, expected_origin):
        with self.assigned_activation_origin_guard(expected_origin):
            return version in (2, 3) or (version in (1, 4, 5)
                and version in self._assigned_fine_archive_versions)

    def require_assigned_fine_archive_support(self, checkpoint, *, expected_origin):
        if checkpoint.fine_archive is not None and checkpoint.fine_archive[:8] in (b"NUWAXAR1", b"NUWAXAR4", b"NUWAXAR5"):
            version = int(checkpoint.fine_archive[7:8])
            if not self.supports_assigned_fine_archive_version(version, expected_origin=expected_origin):
                raise RuntimeError("runtime does not support assigned AR%d source archives" % version)

    def require_assigned_activation_origin(self, expected_origin) -> None:
        with self.assigned_activation_origin_guard(expected_origin):
            pass

    @contextmanager
    def assigned_activation_origin_guard(self, expected_origin):
        """Fence connection metadata for a short callback-free local commit.

        Runtime uses R→provider-state→localization S→publication P. No RPC,
        delivery drain, or callback may execute while this existing lock is held.
        """
        with self._map_state_lock:
            if (not self._connected or self._stopping
                    or self._stub is not expected_origin[0]
                    or self._map_connection_epoch != expected_origin[1]):
                raise RuntimeError("assigned activation provider connection changed")
            yield

    def capture_assigned_mapping_state(self, *, expected_origin) -> Dict[str, Any]:
        with self.assigned_activation_origin_guard(expected_origin):
            if self._mapping_transition_inflight:
                raise RuntimeError("assigned mapping state is unavailable during a transition")
            pending = self._assigned_activation_attempt
            transition = self._mapping_transition_epoch
        response = expected_origin[0].GetMappingState(pb2.Empty(), timeout=_read_deadline_sec())
        payload = self._mapping_state_payload(response)
        settled = False
        if pending is not None and (pending.stub is expected_origin[0]
                                    and pending.connection_epoch == expected_origin[1]):
            retained_request = dict(zip(("activation_request_id", "session_id", "map_name",
                                        "checkpoint_id", "expected_runtime_generation"), pending.request_key[:5]))
            retained_request["asset_identity"] = {"asset_id": pending.request_key[5]}
            try:
                # Only this actual pinned RPC response can settle the pending
                # owner. Journal dictionaries and caller receipt claims cannot.
                _assigned_activation_ack(payload, activation_request=retained_request)
                settled = True
            except ValueError:
                # Partial, nonmatching, or nonsealed state remains diagnostic;
                # it never turns an unresolved dispatch into a settled owner.
                pass
        with self.assigned_activation_origin_guard(expected_origin):
            if (self._mapping_transition_inflight or self._mapping_transition_epoch != transition
                    or self._assigned_activation_attempt is not pending):
                raise RuntimeError("assigned mapping state ownership changed during capture")
            if settled:
                self._assigned_activation_attempt = None
        # `pending` owns the retired payload references until after lock exit.
        return payload

    def stop_assigned_mapping_predecessor(self, *, predecessor, expected_origin) -> Dict[str, Any]:
        """Stop only the journaled owner; every ambiguous dispatch remains UNKNOWN.

        The existing transition reservation drains deliveries and prevents a
        concurrent local StartMapping. Native StopMapping retains its own
        exploration safety ownership checks. No lock spans either RPC.
        """
        expected = _assigned_mapping_predecessor(predecessor)
        self.require_assigned_activation_origin(expected_origin)
        transition = self._begin_mapping_transition()
        dispatched = False

        def require_origin():
            with self._map_state_lock:
                if (not self._connected or self._stopping
                        or self._stub is not expected_origin[0]
                        or self._map_connection_epoch != expected_origin[1]
                        or self._mapping_transition_epoch != transition
                        or not self._mapping_transition_inflight):
                    raise RuntimeError("assigned predecessor stop ownership changed")

        try:
            require_origin()
            state = self._mapping_state_payload(expected_origin[0].GetMappingState(
                pb2.Empty(), timeout=_read_deadline_sec()))
            require_origin()
            if (state.get("success") is not True
                    or _assigned_mapping_predecessor(state) != expected
                    or state.get("phase") not in {"ACTIVE", "STOPPED"}
                    or state.get("active") is not (state.get("phase") == "ACTIVE")):
                raise ValueError("assigned predecessor differs from the captured owner")
            if state["phase"] == "ACTIVE":
                dispatched = True
                state = self._mapping_state_payload(expected_origin[0].StopMapping(
                    pb2.StopMappingRequest(session_id=expected["session_id"],
                                           reason="assigned map activation predecessor"),
                    timeout=_read_deadline_sec()))
            require_origin()
            if (state.get("success") is not True or state.get("phase") != "STOPPED"
                    or state.get("active") is not False
                    or _assigned_mapping_predecessor(state) != expected):
                raise ValueError("assigned predecessor stop lacks exact STOPPED evidence")
            if not self._finish_mapping_transition(
                    transition, identity=self._mapping_identity_from_payload(state),
                    input_active=False, expected_origin=expected_origin):
                raise RuntimeError("assigned predecessor stop outlived its connection")
            return state
        except BaseException as exc:
            if dispatched:
                error = GeometryRuntimeMapActivationOutcomeUnknown(
                    "Assigned predecessor StopMapping outcome is unresolved: %s" % exc)
                error.backend_activation_disposition = "unknown"
                error.backend_stop_confirmed = False
                error.backend_changed = True
                error.predecessor = expected
                raise error from exc
            raise
        finally:
            try:
                self._finish_mapping_transition(transition)
            except BaseException:
                LOGGER.exception("Assigned predecessor transition release failed.")

    def stop_mapping(self, *, session_id: str, reason: str = "") -> Dict[str, Any]:
        self._raise_if_stopping("stop_mapping")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        with self._map_delivery_lock:
            self._raise_if_map_delivery_reentry_locked("StopMapping")
        self._wait_for_map_deliveries()
        response = self._stub.StopMapping(
            pb2.StopMappingRequest(
                session_id=str(session_id or "").strip(),
                reason=str(reason or "").strip(),
            ),
            timeout=_read_deadline_sec(),
        )
        payload = self._mapping_state_payload(response)
        if (
            not payload["success"]
            or payload["phase"] != "STOPPED"
            or payload["active"]
            or payload["session_id"] != str(session_id or "").strip()
        ):
            raise RuntimeError("StopMapping 未返回匹配的 STOPPED 会话。")
        stopped_identity = self._mapping_identity_from_payload(payload)
        with self._map_delivery_lock:
            with self._map_state_lock:
                if self._stopping or self._mapping_transition_inflight:
                    raise RuntimeError(
                        "StopMapping 完成时 provider 正在断连或切换建图会话。"
                    )
                if (
                    self._mapping_identity is not None
                    and stopped_identity != self._mapping_identity
                ):
                    raise RuntimeError("StopMapping 返回了不同建图会话的终态。")
                if self._mapping_identity != stopped_identity:
                    self._mapping_transition_epoch += 1
                # STOPPED 后保留同一身份的最终栅格，供 save-map 收尾读取。
                self._mapping_identity = stopped_identity
                self._planar_input_active = False
        return payload

    def get_mapping_state(self) -> Dict[str, Any]:
        self._raise_if_stopping("get_mapping_state")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        response = self._stub.GetMappingState(
            pb2.Empty(), timeout=_read_deadline_sec()
        )
        return self._mapping_state_payload(response)

    def is_mapping_ready(self) -> bool:
        """Phase 3: mapping 板块 readiness。

        走 gRPC `GetMappingReadiness`，不把进程级 `HealthCheck.healthy`
        误当作 mapping capability ready。该方法与
        `runtime_diagnostics.rs::mapping_readiness` 对账。

        返回 True 表示所选二维或三维建图拥有方已有有效规范地图。
        未连接、输入未就绪、会话停止或后端失败时返回 False。
        """
        diagnostics = self.get_mapping_diagnostics()
        return bool(diagnostics and diagnostics.get("ready", False))

    def get_lcm_backend_name(self) -> str:
        """返回真实 LCM 后端名称或不可用状态。"""
        if not self._connected or not self._stub:
            return "<disconnected>"
        try:
            resp = self._stub.HealthCheck(
                pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
            )
            return str(resp.lcm_backend)
        except grpc.RpcError as e:
            _log_rpc_error("HealthCheck(get_lcm_backend_name)", e)
            return "<unknown>"

    # === Phase 6: mapping / nav / explore diagnostics facade ===
    # Python gateway/service 只感知 provider 协议。provider 对外暴露三套
    # diagnostics dict:
    #   - get_mapping_diagnostics()    : 建图板块
    #   - get_navigation_diagnostics() : 导航板块
    #   - get_exploration_diagnostics(): 探索板块(已在 Phase 5 实装,本阶段保留入口)
    # 任一方法失败都返回 None,绝不假成功。
    def get_mapping_diagnostics(self) -> Optional[Dict[str, Any]]:
        """返回 mapping 板块 diagnostics dict(Phase 7:走 `GetMappingReadiness`)。

        Phase 7:不再只用 HealthCheck + GetOccupancyGrid 拼凑,直接走新增的
        `GetMappingReadiness` RPC,`message` 字段为 JSON dict,涵盖
        odom_count / pointcloud_count / last_occupancy_revision /
        accumulated_point_count / last_error / adapter_present。

        字段:
          - ``ready`` (bool): 综合 ready(bool)
          - ``lcm_backend`` (str): canonical LCM backend 名(从 message JSON 解析)
          - ``adapter_present`` (bool): mapping adapter 是否已构造
          - ``odom_count`` (int): 累计 odom 帧数
          - ``pointcloud_count`` (int): 累计 pointcloud 帧数
          - ``last_occupancy_revision`` (int): 最近 occupancy revision
          - ``accumulated_point_count`` (int): accumulator 累计点数
          - ``last_error`` (str|None): 失败原因
          - ``error`` (str|None): RPC 层失败原因

        返回 None —— 子进程未连接。
        """
        if not self._connected or not self._stub:
            return None
        # Phase 7:主路径走新增 GetMappingReadiness。
        try:
            resp = self._stub.GetMappingReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            payload = _parse_readiness_message(resp.message)
            payload["ready"] = bool(resp.ready)
            payload["error"] = None
            # fallback 字段名兼容旧调用方。
            payload.setdefault("has_occupancy_grid", payload.get("last_occupancy_revision", 0) > 0)
            payload.setdefault("occupancy_revision", payload.get("last_occupancy_revision"))
            return payload
        except grpc.RpcError as e:
            _log_rpc_error("GetMappingReadiness", e)
            # 兜底:旧 HealthCheck + GetOccupancyGrid 路径,保持向后兼容。
            try:
                health = self._stub.HealthCheck(
                    pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
                )
                lcm_backend = str(health.lcm_backend)
            except grpc.RpcError as e2:
                _log_rpc_error("HealthCheck(get_mapping_diagnostics fallback)", e2)
                return {
                    "ready": False,
                    "lcm_backend": "<unknown>",
                    "odom_count": None,
                    "pointcloud_count": None,
                    "last_occupancy_revision": None,
                    "accumulated_point_count": None,
                    "adapter_present": None,
                    "last_error": None,
                    "has_occupancy_grid": False,
                    "occupancy_revision": None,
                    "error": "GetMappingReadiness+HealthCheck 失败: %s / %s" % (e, e2),
                }
            return {
                "ready": False,
                "lcm_backend": lcm_backend,
                "odom_count": None,
                "pointcloud_count": None,
                "last_occupancy_revision": None,
                "accumulated_point_count": None,
                "adapter_present": None,
                "last_error": "GetMappingReadiness 不可用: %s" % e,
                "has_occupancy_grid": False,
                "occupancy_revision": None,
                "error": "GetMappingReadiness 失败: %s" % e,
            }

    def get_navigation_diagnostics(self) -> Optional[Dict[str, Any]]:
        """返回 navigation 板块 diagnostics dict(走 `GetNavigationReadiness`)。

        字段:
          - ``ready`` (bool): nuwax nav_runtime + lcm backend 都健康
          - ``lcm_backend`` (str): canonical LCM backend 名
          - ``readiness`` (dict): `GetNavigationReadiness` 解析后字段
            (ready/message/lcm_backend/tf/costmap/planner/cmd_vel_sink/profile/
             module_node_count/p1_native_profiles)
          - ``profile`` / ``active_profile`` (str|None): 当前 nav profile
            (Phase 7 兼容旧 `profile` + 新 `active_profile`)
          - ``backend`` (str|None): "RealLcm" 或 "Unknown"（未启动时）
          - ``tf_count`` (int|None): 累计 tf 帧数
          - ``costmap_revision`` (int|None): 当前 occupancy revision
          - ``cmd_vel_sink_ready`` (bool|None): cmd_vel consumer + watchdog 状态
          - ``module_node_count`` / ``p1_native_profiles``: 兼容旧字段
          - ``error`` (str|None): 失败原因

        返回 None —— 子进程未连接。
        """
        if not self._connected or not self._stub:
            return None
        lcm_backend = "<unknown>"
        try:
            health = self._stub.HealthCheck(
                pb2.HealthCheckRequest(), timeout=_pose_deadline_sec()
            )
            lcm_backend = str(health.lcm_backend)
        except grpc.RpcError as e:
            _log_rpc_error("HealthCheck(get_navigation_diagnostics)", e)
            return {
                "ready": False,
                "lcm_backend": "<unknown>",
                "readiness": None,
                "profile": None,
                "active_profile": None,
                "backend": None,
                "tf_count": None,
                "costmap_revision": None,
                "cmd_vel_sink_ready": None,
                "module_node_count": None,
                "p1_native_profiles": None,
                "error": "HealthCheck 失败: %s" % e,
            }
        readiness = self.get_navigation_readiness()
        profile = readiness.get("profile") if isinstance(readiness, dict) else None
        active_profile = (
            readiness.get("active_profile") if isinstance(readiness, dict) else None
        ) or profile
        module_node_count = (
            readiness.get("module_node_count") if isinstance(readiness, dict) else None
        )
        p1_native_profiles = (
            readiness.get("p1_native_profiles") if isinstance(readiness, dict) else None
        )
        backend = readiness.get("backend") if isinstance(readiness, dict) else None
        tf_count = readiness.get("tf_count") if isinstance(readiness, dict) else None
        costmap_revision = (
            readiness.get("costmap_revision") if isinstance(readiness, dict) else None
        )
        cmd_vel_sink_ready = (
            readiness.get("cmd_vel_sink_ready") if isinstance(readiness, dict) else None
        )
        return {
            "ready": bool(health.healthy) and bool(
                isinstance(readiness, dict) and readiness.get("ready")
            ),
            "lcm_backend": lcm_backend,
            "readiness": readiness,
            "profile": profile,
            "active_profile": active_profile,
            "backend": backend,
            "tf_count": tf_count,
            "costmap_revision": costmap_revision,
            "cmd_vel_sink_ready": cmd_vel_sink_ready,
            "module_node_count": module_node_count,
            "p1_native_profiles": p1_native_profiles,
            "error": None,
        }

    # === Localization ===
    def capture_assigned_localization_sample(
        self, *, assigned_request_v2, expected_terminal, expected_origin,
    ) -> _AssignedNativeLocalizationSample:
        """Read one exact-owner native pose/TF sample through a pinned stub.

        No state lock spans the RPC and no alternate connection is retried.
        Missing/unavailable samples remain failures, never raw-pose fallbacks.
        """
        request = (assigned_request_v2 if type(assigned_request_v2) is AssignedRelocalizationRequest
                   else AssignedRelocalizationRequest.from_dict(assigned_request_v2))
        if (type(expected_terminal) is not AssignedRelocalizationTerminal
                or not request.exact_replay_of(expected_terminal.request)
                or expected_terminal.disposition not in _RELOCALIZE_SUCCESS_DISPOSITIONS):
            raise ValueError("assigned sample capture requires the retained applied terminal")
        with self.assigned_activation_origin_guard(expected_origin):
            if self._mapping_transition_inflight:
                raise RuntimeError("assigned localization sample is unavailable during a transition")
            token = self._map_delivery_token_locked()
        rpc = getattr(expected_origin[0], "GetAssignedLocalizationSample", None)
        if not callable(rpc):
            raise RuntimeError("Geometry runtime does not support assigned localization samples")
        try:
            response = rpc(request.to_proto(pb2.RelocalizeRequest), timeout=_pose_deadline_sec())
        except grpc.RpcError as exc:
            raise GeometryRuntimeRpcError.from_rpc_error("GetAssignedLocalizationSample", exc) from exc
        sample = _admit_assigned_localization_sample(
            response, provider=self, origin=expected_origin, delivery_token=token,
        )
        with self.assigned_localization_sample_guard(sample):
            return sample

    @contextmanager
    def assigned_localization_sample_guard(self, sample):
        """Existing provider-state fence; callers use R → provider → S → P."""
        if type(sample) is not _AssignedNativeLocalizationSample or sample._provider is not self:
            raise ValueError("assigned localization sample belongs to another provider")
        with self.assigned_activation_origin_guard(sample._origin):
            if self._mapping_transition_inflight or self._map_delivery_token_locked() != sample._delivery_token:
                raise RuntimeError("assigned localization sample mapping ownership changed")
            yield

    def capture_localization_sample(self):
        """Read pose/TF through one checked Python connection interval.

        This is not an atomic native snapshot or evidence of native activation
        ownership. Never hold map locks across RPCs or retry on a new stub.
        """
        with self._map_state_lock:
            stub = self._stub
            if (self._stopping or not self._connected or stub is None
                    or self._mapping_transition_inflight):
                return None, None
            token = self._map_delivery_token_locked()
            static_edges = tuple(self._static_extrinsic_edges)
        tree_reader = getattr(stub, "GetFrameTree", None)
        pose_reader = getattr(stub, "GetCurrentPose", None)
        if not callable(tree_reader) or not callable(pose_reader):
            return None, None
        try:
            tree_response = tree_reader(pb2.Empty(), timeout=_pose_deadline_sec())
            pose_response = pose_reader(pb2.PoseRequest(), timeout=_pose_deadline_sec())
            frame_tree = _merge_static_edges(
                _build_frame_tree(tree_response.transforms), static_edges)
            pose = _build_pose(pose_response)
        except grpc.RpcError as error:
            _log_rpc_error("capture_localization_sample", error)
            return None, None
        with self._map_state_lock:
            if (self._stopping or not self._connected
                    or self._mapping_transition_inflight or self._stub is not stub
                    or self._map_delivery_token_locked() != token):
                return None, None
        return pose, frame_tree

    def get_current_pose(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> Optional[Pose]:
        self._raise_if_stopping("get_current_pose")
        if not self._stub:
            return None
        reader = getattr(self._stub, "GetCurrentPose", None)
        if not callable(reader):
            return None
        timeout = _pose_deadline_sec()
        if deadline_monotonic is not None:
            remaining = float(deadline_monotonic) - time.monotonic()
            if remaining <= 0:
                return None
            timeout = min(timeout, remaining)
        try:
            resp = reader(pb2.PoseRequest(), timeout=timeout)
            return _build_pose(resp)
        except grpc.RpcError as e:
            _log_rpc_error("GetCurrentPose", e)
            return None

    def get_frame_tree(self) -> Optional[FrameTree]:
        self._raise_if_stopping("get_frame_tree")
        if not self._stub:
            return None
        try:
            resp = self._stub.GetFrameTree(pb2.Empty(), timeout=_pose_deadline_sec())
            frame_tree = _build_frame_tree(resp.transforms)
            return _merge_static_edges(frame_tree, self._static_extrinsic_edges)
        except grpc.RpcError as e:
            _log_rpc_error("GetFrameTree", e)
            return None

    # === Mapping ===
    def get_occupancy_grid_meta(self) -> Optional[Dict[str, Any]]:
        self._raise_if_stopping("get_occupancy_grid_meta")
        if not self._stub:
            return None
        try:
            resp = self._stub.GetOccupancyGrid(pb2.Empty(), timeout=_read_deadline_sec())
            return {
                "width": resp.width, "height": resp.height,
                "resolution": resp.resolution,
                "origin_x": resp.origin_x, "origin_y": resp.origin_y,
                "frame_id": resp.frame_id, "revision": resp.revision,
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetOccupancyGrid", e)
            return None

    def get_occupancy_grid_data(self) -> Optional[Dict[str, Any]]:
        """获取完整 OccupancyGrid 数据（元数据 + 栅格字节）。"""
        self._raise_if_stopping("get_occupancy_grid_data")
        if not self._stub:
            return None
        try:
            # 全量栅格字节可能较大，用重操作 deadline 而非读类 deadline。
            resp = self._stub.GetOccupancyGridData(pb2.Empty(), timeout=_heavy_deadline_sec())
            meta = resp.meta
            return {
                "width": meta.width, "height": meta.height,
                "resolution": meta.resolution,
                "origin_x": meta.origin_x, "origin_y": meta.origin_y,
                "frame_id": meta.frame_id, "revision": meta.revision,
                "timestamp_ns": meta.timestamp_ns,
                "data": bytes(resp.data),
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetOccupancyGridData", e)
            return None

    def get_raw_occupancy_grid_data(self) -> Optional[Dict[str, Any]]:
        """获取未叠加虚拟禁行区的物理 OccupancyGrid 数据。"""

        self._raise_if_stopping("get_raw_occupancy_grid_data")
        if not self._stub:
            return None
        rpc = getattr(self._stub, "GetRawOccupancyGridData", None)
        if not callable(rpc):
            return None
        try:
            resp = rpc(pb2.Empty(), timeout=_heavy_deadline_sec())
            meta = resp.meta
            return {
                "width": meta.width,
                "height": meta.height,
                "resolution": meta.resolution,
                "origin_x": meta.origin_x,
                "origin_y": meta.origin_y,
                "frame_id": meta.frame_id,
                "revision": meta.revision,
                "timestamp_ns": meta.timestamp_ns,
                "data": bytes(resp.data),
            }
        except grpc.RpcError as e:
            _log_rpc_error("GetRawOccupancyGridData", e)
            return None

    @staticmethod
    def _occupancy_grid_from_components(
        *,
        map_name: str,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        frame_id: str,
        timestamp_ns: int,
        data: bytes,
    ) -> OccupancyGrid:
        width = int(width)
        height = int(height)
        resolution = float(resolution)
        timestamp_ns = int(timestamp_ns)
        if width <= 0 or height <= 0:
            raise ValueError("occupancy width/height must be positive")
        if resolution <= 0.0:
            raise ValueError("occupancy resolution must be positive")
        if timestamp_ns < 0:
            raise ValueError("occupancy timestamp_ns must be non-negative")
        expected_size = width * height
        if len(data) != expected_size:
            raise ValueError(
                "occupancy data size mismatch: expected=%d actual=%d"
                % (expected_size, len(data))
            )
        # 原生只读字节在进入合同前检查一次；列表仅在公共序列化或显式深复制时展开。
        from contracts.occupancy_cells import PackedOccupancyCells
        decoded = PackedOccupancyCells(data)
        frame_id = str(frame_id or "map").strip().lstrip("/") or "map"
        map_name = str(map_name or "").strip()
        if not map_name:
            raise ValueError("occupancy map_name must be non-empty")
        timestamp = datetime.fromtimestamp(
            timestamp_ns / 1_000_000_000.0,
            tz=timezone.utc,
        )
        return OccupancyGrid(
            timestamp=timestamp,
            map_id=map_name,
            frame_id=frame_id,
            width=width,
            height=height,
            resolution_m=resolution,
            origin=Pose(
                timestamp=timestamp,
                frame_id=frame_id,
                position=Vector3(x=float(origin_x), y=float(origin_y), z=0.0),
                orientation=Quaternion(),
            ),
            data=decoded,
        )

    def get_occupancy_grid(self) -> Optional[OccupancyGrid]:
        """Return the receipt-bound stream owner, or an explicitly unbound unary view."""
        with self._map_state_lock:
            if self._stopping or self._mapping_transition_inflight:
                return None
            request_identity = self._mapping_identity
            request_token = self._map_delivery_token_locked()
            cached = self._last_occupancy_grid
            evidence = self._last_map_update_evidence
            bound = (cached is not None and evidence is not None and evidence[0] == request_token
                     and self._last_occupancy_grid_identity == request_identity)
        if request_identity is None:
            return None
        if bound:
            # Packed cells own immutable bytes. Copy only editable descriptor shells,
            # never expand the whole grid merely to return an existing owner.
            result = cached.model_copy(update={"origin": cached.origin.model_copy(deep=True),
                                               "metadata": copy.deepcopy(cached.metadata)})
            with self._map_state_lock:
                if (self._stopping or self._mapping_transition_inflight
                        or self._map_delivery_token_locked() != request_token
                        or self._last_occupancy_grid is not cached):
                    return None
            return result
        payload = self.get_occupancy_grid_data()
        if payload is None:
            return None
        try:
            grid = self._occupancy_grid_from_components(
                map_name=request_identity[1], width=int(payload["width"]), height=int(payload["height"]),
                resolution=float(payload["resolution"]), origin_x=float(payload["origin_x"]),
                origin_y=float(payload["origin_y"]), frame_id=str(payload["frame_id"]),
                timestamp_ns=int(payload.get("timestamp_ns") or 0), data=bytes(payload["data"]))
        except (KeyError, TypeError, ValueError):
            LOGGER.exception("GetOccupancyGridData returned an invalid unbound grid")
            return None
        grid.metadata = {"source_name": "geometry_runtime", "identity_status": "unbound_unary"}
        with self._map_state_lock:
            if (self._stopping or self._mapping_transition_inflight
                    or self._map_delivery_token_locked() != request_token):
                return None
        # The unary wire lacks a receipt: it cannot replace the committed stream cache.
        return grid

    def get_occupancy_grid_identity(
        self,
        occupancy_grid: Optional[OccupancyGrid] = None,
    ) -> Optional[_MappingIdentity]:
        """读取仍拥有此栅格的会话，与修订描述共用同一来源。"""

        descriptor = self.get_occupancy_grid_descriptor(occupancy_grid)
        return descriptor["mapping_identity"] if descriptor is not None else None

    def get_occupancy_grid_descriptor(
        self,
        occupancy_grid: Optional[OccupancyGrid] = None,
    ) -> Optional[Dict[str, Any]]:
        """捕获同一来源的栅格与修订，整图比较在状态锁外完成。"""

        with self._map_state_lock:
            identity = self._mapping_identity
            cached = self._last_occupancy_grid
            if (
                self._stopping
                or self._mapping_transition_inflight
                or identity is None
                or self._last_occupancy_grid_identity != identity
                or cached is None
            ):
                return None
            token = self._map_delivery_token_locked()
            revision = int(self._last_occupancy_grid_revision)
        if occupancy_grid is not None and occupancy_grid != cached:
            return None
        with self._map_state_lock:
            if (self._stopping or self._mapping_transition_inflight
                    or self._map_delivery_token_locked() != token
                    or self._last_occupancy_grid is not cached
                    or self._last_occupancy_grid_revision != revision):
                return None
            return {
                "mapping_identity": identity,
                "revision": revision,
                "content_identity": copy.deepcopy(cached.metadata.get("content_identity")),
                "map_provenance": copy.deepcopy(cached.metadata.get("map_provenance")),
            }

    def get_mapping_stream_status(self) -> Dict[str, Any]:
        """只读返回当前来源实际收到的地图观测，不把查询时间变成输入时间。"""

        with self._map_state_lock:
            observation = self._last_map_stream_observation
            if (
                not self._connected
                or self._stopping
                or self._mapping_transition_inflight
                or observation is None
                or observation[0] != self._map_delivery_token_locked()
            ):
                return {}
            status = dict(observation[1])
            status["occupancy_metrics"] = dict(status["occupancy_metrics"])
            return status

    def get_raw_occupancy_grid(self) -> Optional[OccupancyGrid]:
        """返回物理栅格，供地图资产持久化避免烘焙虚拟禁行区。"""

        with self._map_state_lock:
            if self._stopping or self._mapping_transition_inflight:
                return None
            request_identity = self._mapping_identity
            request_token = self._map_delivery_token_locked()
        if request_identity is None:
            return None
        payload = self.get_raw_occupancy_grid_data()
        if payload is None:
            return None
        try:
            grid = self._occupancy_grid_from_components(
                map_name=request_identity[1],
                width=int(payload["width"]),
                height=int(payload["height"]),
                resolution=float(payload["resolution"]),
                origin_x=float(payload["origin_x"]),
                origin_y=float(payload["origin_y"]),
                frame_id=str(payload["frame_id"]),
                timestamp_ns=int(payload.get("timestamp_ns") or 0),
                data=bytes(payload["data"]),
            )
        except (KeyError, TypeError, ValueError):
            LOGGER.exception("GetRawOccupancyGridData 返回了无效物理栅格。")
            return None
        with self._map_state_lock:
            if (
                self._stopping
                or self._mapping_transition_inflight
                or self._map_delivery_token_locked() != request_token
                or self._mapping_identity != request_identity
            ):
                return None
        return grid

    def get_cost_map(self):
        """Rust facade 当前没有独立 cost-map RPC。"""

        return None

    def get_semantic_map(self):
        """Rust facade 当前没有独立 semantic-map RPC。"""

        return None

    def _map_update_to_payload(self, update) -> Optional[Dict[str, Any]]:
        """Admit a complete owned map update without content-derived identity."""
        import numpy as np
        if int(update.schema_version) != _OCCUPANCY_SCHEMA_VERSION:
            raise ValueError("unsupported map schema_version=%d" % int(update.schema_version))
        meta = _require_proto_message_field(update, "meta")
        receipt = _occupancy_receipt_dict(_require_proto_message_field(update, "occupancy_receipt"))
        commit = receipt["commit_id"]
        session_id, map_name = str(update.session_id), str(update.map_name)
        generation, revision, timestamp = int(update.session_generation), int(meta.revision), int(meta.timestamp_ns)
        if (not session_id or session_id != session_id.strip() or not map_name or map_name != map_name.strip()
                or generation <= 0 or revision <= 0 or receipt["session_id"] != session_id
                or receipt["session_generation"] != generation or receipt["occupancy_revision"] != revision
                or receipt["source_timestamp_ns"] != timestamp or receipt["backend_generation"] <= 0
                or receipt["backend_kind"] not in _SUPPORTED_MAPPING_BACKEND_KINDS
                or len(commit["boot_nonce"]) != 16 or not any(commit["boot_nonce"]) or commit["sequence"] <= 0):
            raise ValueError("map update differs from its actual occupancy receipt")
        if (not meta.frame_id or meta.frame_id != meta.frame_id.strip()
                or not all(math.isfinite(value) for value in (meta.resolution, meta.origin_x, meta.origin_y))
                or meta.resolution <= 0 or meta.width <= 0 or meta.height <= 0):
            raise ValueError("map update geometry is invalid")
        from contracts.naming import validate_frame_id
        validate_frame_id(str(meta.frame_id))
        data, raw = bytes(update.data), bytes(update.raw_data)
        size, representation = int(update.size), int(update.raw_representation)
        if size != len(data) or size != int(meta.width) * int(meta.height):
            raise ValueError("map update size differs from its complete snapshot")
        if representation == int(pb2.RAW_MAP_REPRESENTATION_SEPARATE_RAW):
            if len(raw) != size:
                raise ValueError("separate raw map must carry a complete same-layout body")
            display_body = raw
        elif representation in (int(pb2.RAW_MAP_REPRESENTATION_SAME_AS_EFFECTIVE), int(pb2.RAW_MAP_REPRESENTATION_EFFECTIVE_FALLBACK)):
            if raw:
                raise ValueError("shared/fallback raw representation must not carry a second body")
            display_body = data if representation == int(pb2.RAW_MAP_REPRESENTATION_SAME_AS_EFFECTIVE) else None
        else:
            raise ValueError("map update raw representation is missing or unsupported")
        identity = (session_id, map_name, generation)
        provenance = {"schema_version": 2, "commit_id": {"boot_nonce": commit["boot_nonce"].hex(), "sequence": commit["sequence"]},
            "session_id": session_id, "session_generation": generation, "occupancy_revision": revision,
            "source_timestamp_ns": timestamp, "backend_kind": receipt["backend_kind"],
            "backend_generation": receipt["backend_generation"], "frame_id": str(meta.frame_id)}
        receipt_key = (commit["boot_nonce"], commit["sequence"], session_id, generation,
                       receipt["backend_kind"], receipt["backend_generation"], revision, timestamp, str(meta.frame_id), map_name)
        layout = (int(meta.width), int(meta.height), struct.pack("<fdd", meta.resolution, meta.origin_x, meta.origin_y), str(meta.frame_id))
        body = (layout, representation, data, raw, int(update.last_seen_ms))
        with self._map_state_lock:
            admission_token = self._map_delivery_token_locked()
        # Exact duplicate admission is outside the short state lock and occurs
        # before grid/count materialization. No opaque nonce is numerically ordered.
        previous = None
        while True:
            with self._map_state_lock:
                if self._stopping or self._mapping_transition_inflight or self._map_delivery_token_locked() != admission_token:
                    raise ValueError("map update crossed provider connection/mapping transition")
                if self._mapping_identity is not None and self._mapping_identity != identity:
                    raise ValueError("map update mapping identity mismatch")
                observed = self._last_map_update_evidence
                previous = observed if observed is not None and observed[0] == admission_token else None
            if previous is not None:
                prior = previous[1]
                if (receipt_key[0] != prior[0] or receipt_key[2:6] != prior[2:6] or receipt_key[8:] != prior[8:]):
                    raise ValueError("map owner changed without an admitted provider transition")
                if revision < prior[6] or commit["sequence"] < prior[1]:
                    raise ValueError("stale map occupancy receipt")
                if revision == prior[6] or commit["sequence"] == prior[1]:
                    if receipt_key != prior or body != previous[2]:
                        raise ValueError("same map receipt has conflicting immutable content")
                    with self._map_state_lock:
                        if self._last_map_update_evidence is observed and self._map_delivery_token_locked() == admission_token:
                            return None
                    continue
            break
        grid = self._occupancy_grid_from_components(map_name=map_name, width=meta.width, height=meta.height,
            resolution=meta.resolution, origin_x=meta.origin_x, origin_y=meta.origin_y,
            frame_id=meta.frame_id, timestamp_ns=timestamp, data=data)
        cells = np.frombuffer(data, dtype=np.int8)
        known = int(np.count_nonzero(cells >= 0))
        occupied = int(np.count_nonzero(cells >= 50))
        display_grid = grid
        if display_body is not None and representation == int(pb2.RAW_MAP_REPRESENTATION_SEPARATE_RAW):
            display_grid = self._occupancy_grid_from_components(map_name=map_name, width=meta.width, height=meta.height,
                resolution=meta.resolution, origin_x=meta.origin_x, origin_y=meta.origin_y,
                frame_id=meta.frame_id, timestamp_ns=timestamp, data=display_body)
            display_cells = np.frombuffer(display_body, dtype=np.int8)
            display_known = int(np.count_nonzero(display_cells >= 0))
            display_occupied = int(np.count_nonzero(display_cells >= 50))
        else:
            display_known, display_occupied = known, occupied
        content_identity = {"kind": "committed_occupancy", "representation": "effective", "provenance": provenance}
        display_identity = {"kind": "committed_occupancy", "representation": "raw_physical" if display_body is not None else "effective_fallback", "provenance": provenance}
        # Recheck an immutable evidence snapshot after concurrent admissions;
        # comparisons never hold the lock, and a newer complete receipt wins.
        while True:
            with self._map_state_lock:
                if self._stopping or self._mapping_transition_inflight or self._map_delivery_token_locked() != admission_token:
                    raise ValueError("map update crossed provider connection/mapping transition")
                observed = self._last_map_update_evidence
                previous = observed if observed is not None and observed[0] == admission_token else None
            if previous is not None:
                prior = previous[1]
                if receipt_key[0] != prior[0] or receipt_key[2:6] != prior[2:6] or receipt_key[8:] != prior[8:]:
                    raise ValueError("map owner changed without an admitted provider transition")
                if revision < prior[6] or commit["sequence"] < prior[1]:
                    raise ValueError("stale map occupancy receipt")
                if revision == prior[6] or commit["sequence"] == prior[1]:
                    if receipt_key != prior or body != previous[2]:
                        raise ValueError("same map receipt has conflicting immutable content")
                    with self._map_state_lock:
                        if self._last_map_update_evidence is observed and self._map_delivery_token_locked() == admission_token:
                            return None
                    continue
            content_changed = previous is None or previous[2][0] != layout or previous[2][2] != data
            raw_changed = (previous is None or previous[2][0] != layout
                or previous[2][1] != representation
                or (previous[3] if previous[3] is not None else previous[2][2])
                    != (display_body if display_body is not None else data))
            # Prepare immutable delivery metadata before exposing the cached grid.
            count = 1 if previous is None else previous[4] + 1
            effective_metrics = {"content_identity": content_identity, "content_revision": revision,
                "content_changed": content_changed, "receipt_progress": True, "known_cell_count": known, "occupied_cell_count": occupied}
            display_metrics = {"content_identity": display_identity, "content_revision": revision,
                "content_changed": raw_changed, "receipt_progress": True, "known_cell_count": display_known, "occupied_cell_count": display_occupied}
            metadata = {"event_type": "content_update", "source_kind": "platform_live_map_provider", "source_name": "geometry_runtime",
                "map_snapshot_kind": "live_streaming", "live_streaming_ready": True, "mapping_live_streaming_ready": True,
                "source_alive": True, "streaming": True, "frame_id": grid.frame_id, "revision": revision,
                "update_sequence": revision, "occupancy_sequence": revision, "occupancy_message_count": count,
                "occupancy_content_revision": revision, "content_revision": revision, "content_identity": content_identity,
                "map_provenance": provenance, "raw_representation": representation, "schema_version": 2,
                "size": size, "last_seen_ms": int(update.last_seen_ms), "timestamp_ns": timestamp,
                "mapping_session_id": session_id, "session_id": session_id, "map_name": map_name,
                "source_epoch": "%s:%s:%d" % (commit["boot_nonce"].hex(), session_id, generation), "session_generation": generation,
                "known_cell_count": known, "occupied_cell_count": occupied, "occupancy_metrics": effective_metrics}
            display_metadata = {"content_identity": display_identity, "map_provenance": provenance,
                "occupancy_content_revision": revision, "content_revision": revision,
                "known_cell_count": display_known, "occupied_cell_count": display_occupied, "occupancy_metrics": display_metrics}
            grid.metadata = dict(metadata)
            if display_grid is not grid:
                display_grid.metadata = dict(metadata, **display_metadata)
            cached_grid = grid.model_copy(update={"origin": grid.origin.model_copy(deep=True),
                                                  "metadata": copy.deepcopy(grid.metadata)})
            with self._map_state_lock:
                if self._last_map_update_evidence is not observed:
                    continue
                if self._stopping or self._mapping_transition_inflight or self._map_delivery_token_locked() != admission_token:
                    raise ValueError("map update crossed provider connection/mapping transition")
                if self._mapping_identity is None:
                    self._mapping_identity = identity
                    self._mapping_transition_epoch += 1
                elif self._mapping_identity != identity:
                    raise ValueError("map update mapping identity mismatch")
                delivery_token = self._map_delivery_token_locked()
                # This is an observed delivery count only, never an identity clock.
                self._last_map_update_evidence = (delivery_token, receipt_key, body, display_body, count)
                self._last_occupancy_grid = cached_grid
                self._last_occupancy_grid_identity = identity
                self._last_occupancy_grid_revision = revision
                break
        with self._map_state_lock:
            if self._map_delivery_token_locked() == delivery_token and self._last_map_update_evidence[1] == receipt_key:
                self._last_map_stream_observation = (delivery_token, {
                    "mapping_session_id": session_id, "map_name": map_name, "session_generation": generation,
                    "occupancy_grid_ready": True, "occupancy_has_publisher": True, "occupancy_revision": revision,
                    "occupancy_sequence": revision, "occupancy_message_count": count,
                    "latest_occupancy_timestamp": grid.timestamp.isoformat() if timestamp > 0 else None,
                    "latest_occupancy_received_at": datetime.now(timezone.utc).isoformat(), "occupancy_metrics": effective_metrics})
        return _MapUpdatePayload({"source_name": "geometry_runtime", "source_epoch": metadata["source_epoch"],
            "occupancy_grid": grid, "display_occupancy_grid": display_grid,
            "display_occupancy_source": "raw_physical" if display_body is not None else "effective_fallback",
            "display_occupancy_fallback_reason": None if display_body is not None else "raw_grid_unavailable",
            "display_occupancy_metadata": display_metadata, "cost_map": None, "semantic_map": None,
            "width": grid.width, "height": grid.height, "resolution": grid.resolution_m, "frame_id": grid.frame_id,
            "revision": revision, "session_id": session_id, "map_name": map_name, "session_generation": generation,
            "metadata": metadata}, delivery_token=delivery_token)

    # === Navigation ===
    # Rust proto 字段映射（round-4 HIGH 3 同步）：
    # - NavigationGoal.target_pose/behavior_tree → pose/frame_id/goal_id
    # - SetNavigationGoal 返回 NavigationState（非 NavigationAck）：
    #   accepted/goal_id/message → status/goal_id/message
    # - CancelNavigation 接受 CancelNavigationRequest（带 goal_id），返回 NavigationState
    # - NavigationReadiness 仅提供 ready + reason
    def get_mapping_cloud_authority(self) -> Dict[str, Any]:
        """读取 Geometry 服务端当前 finalized-map 与 committed-map 组合权威。"""

        self._raise_if_stopping("get_mapping_cloud_authority")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        response = self._stub.GetMappingCloudAuthority(
            pb2.Empty(), timeout=_read_deadline_sec()
        )
        self._raise_if_stopping("get_mapping_cloud_authority")
        if response.schema_version != 3:
            raise ValueError("unsupported MappingCloudAuthority schema_version")
        receipt = _assigned_activation_receipt_payload(
            _require_proto_message_field(response, "activation_receipt_v2"))
        if not isinstance(receipt, AssignedActivationReceipt):
            raise ValueError("MappingCloudAuthority requires a complete assigned receipt")
        commit = _mapping_commit_identity_dict(
            _require_proto_message_field(response, "committed_occupancy_commit_id"))
        authority = {
            "schema_version": 3,
            "activation_receipt_v2": receipt,
            "map_name": str(response.map_name),
            "map_frame_id": str(response.map_frame_id),
            "runtime_generation": int(response.runtime_generation),
            "lifecycle_epoch": int(response.lifecycle_epoch),
            "mapping_session_id": str(response.mapping_session_id),
            "mapping_session_generation": int(response.mapping_session_generation),
            "backend_kind": str(response.backend_kind),
            "backend_generation": int(response.backend_generation),
            "committed_occupancy_revision": int(response.committed_occupancy_revision),
            "committed_occupancy_commit_id": {"boot_nonce": commit["boot_nonce"].hex(), "sequence": commit["sequence"]},
            "committed_occupancy_source_timestamp_ns": int(response.committed_occupancy_source_timestamp_ns),
            "owner_epoch": int(response.owner_epoch),
        }
        return authority

    def submit_mapping_cloud_goal(
        self,
        *,
        query_request_id: str,
        query_text: str,
        authority: Dict[str, Any],
        georeference_json: bytes,
        motion_admission: Dict[str, Any],
        caller_timeout_ms: int = 15000,
        provider_timeout_ms: int = 10000,
    ) -> Dict[str, Any]:
        """提交地点文本查询并等待唯一 GoalLease 准入终态。"""

        self._raise_if_stopping("submit_mapping_cloud_goal")
        if not self._connected or not self._stub:
            raise RuntimeError("Geometry runtime 未连接。")
        receipt = authority["activation_receipt_v2"]
        commit = authority["committed_occupancy_commit_id"]
        request = pb2.MappingCloudGoalRequest(
            schema_version=3,
            activation_receipt_v2=_assigned_activation_receipt_proto(receipt),
            query_request_id=str(query_request_id),
            query_text=str(query_text),
            caller_timeout_ms=int(caller_timeout_ms),
            provider_timeout_ms=int(provider_timeout_ms),
            map_name=authority["map_name"],
            map_frame_id=authority["map_frame_id"],
            georeference_json=bytes(georeference_json),
            runtime_generation=authority["runtime_generation"],
            mapping_session_id=authority["mapping_session_id"],
            mapping_session_generation=authority["mapping_session_generation"],
            committed_occupancy_revision=authority["committed_occupancy_revision"],
            committed_occupancy_commit_id=pb2.MappingCommitIdentity(
                boot_nonce=bytes.fromhex(commit["boot_nonce"]), sequence=commit["sequence"]),
            committed_occupancy_source_timestamp_ns=authority["committed_occupancy_source_timestamp_ns"],
            motion_guard_boot_nonce=bytes(motion_admission["guard_boot_nonce"]),
            motion_stop_generation=int(motion_admission["motion_stop_generation"]),
        )
        context = self._nav_set_transaction_context
        if getattr(context, "active", False):
            raise RuntimeError("reentrant_set_goal")
        request_goal_id = str(query_request_id).strip()
        context.active = True
        context.result = _navigation_submission_result(request_goal_id)
        context.notification = None
        token = None
        stub = None
        try:
            with self._nav_set_transaction_lock:
                with self._nav_delivery_lock:
                    with self._nav_command_lock:
                        token, stub = self._register_navigation_set_locked(
                            (request_goal_id, _PUBLIC_NAV_OWNER_KIND, request_goal_id, 0, 0),
                            reject_any_owner=True,
                        )
                        if token is None:
                            result = dict(context.result)
                            return {**result, "status": "BUSY" if result["reason_code"] == "BUSY" else "ADMISSION_REJECTED",
                                    "query_request_id": request_goal_id, "retryable": False}
                    self._cancel_pending_navigation_notifications_locked()
                self._raise_if_stopping("submit_mapping_cloud_goal")
                response = stub.SubmitMappingCloudGoal(
                    request, timeout=max(float(caller_timeout_ms) / 1000.0 + 1.0, 2.0),
                )
                self._raise_if_stopping("submit_mapping_cloud_goal")
                if (str(response.query_request_id) != request_goal_id
                        or not response.HasField("navigation")):
                    self._finish_navigation_set_without_receipt(token, stub)
                    return {
                        **_navigation_submission_result(request_goal_id, message=str(response.message)),
                        "status": (str(response.status) if str(response.status) not in {"SUCCEEDED", "ADMISSION_UNCONFIRMED"}
                                   else "ADMISSION_REJECTED"),
                        "query_request_id": request_goal_id, "retryable": False,
                    }
                result = self._commit_navigation_submission(
                    response.navigation, command_token=token, command_stub=stub,
                )
                disposition = result["disposition"]
                if disposition == "owned_unconfirmed":
                    status = "ADMISSION_UNCONFIRMED"
                elif disposition == "confirmed":
                    status = "SUCCEEDED"
                elif disposition == "resolved":
                    status = str(result["navigation"]["status"]).upper()
                else:
                    status = "ADMISSION_REJECTED"
            if result["accepted"] or result["ownership_retained"]:
                self._ensure_nav_state_stream()
            return {**result, "status": status, "query_request_id": request_goal_id, "retryable": False}
        except Exception:
            if token is not None:
                self._finish_navigation_set_without_receipt(token, stub)
            raise
        finally:
            # Cloud returns its result directly; never retain a second mutable
            # reference for a later direct-Set take_goal_submission_result call.
            context.result = None
            context.active = False
            context.notification = None

    def set_navigation_goal(
        self,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        frame_id: str = "map",
        goal_id: str = "",
        motion_admission: Optional[Dict[str, Any]] = None,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Compatibility facade routed through the canonical lease transaction."""

        request_goal_id = str(goal_id or "").strip() or (
            "debug-%s" % uuid.uuid4().hex
        )
        goal = NavigationGoal(
            goal_id=request_goal_id,
            target_pose=Pose(
                frame_id=str(frame_id or "map"),
                position=Vector3(x=float(x), y=float(y), z=float(z)),
                orientation=Quaternion(
                    x=float(qx),
                    y=float(qy),
                    z=float(qz),
                    w=float(qw),
                ),
            ),
            metadata={"motion_admission": motion_admission},
        )
        accepted = self.set_goal(
            goal,
            deadline_monotonic=deadline_monotonic,
        )
        result = self.take_goal_submission_result(request_goal_id)
        if result is not None:
            return result
        return _navigation_submission_result(request_goal_id, accepted=accepted,
            disposition="confirmed" if accepted else "rejected",
            message="SetNavigationGoal 已接纳" if accepted else "SetNavigationGoal 未被接纳")

    def cancel_navigation(
        self,
        goal_id: str = "",
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        """兼容 facade；复用正式 provider 的 active exact-goal 栅栏。"""

        return self.cancel_goal(
            goal_id,
            deadline_monotonic=deadline_monotonic,
        )

    def get_navigation_readiness(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        """从 gRPC `GetNavigationReadiness` 返回结构化就绪度。

        冻结字段 3..9 直接从 protobuf 读取；`reason` 内嵌 JSON 只作为旧服务端
        兼容投影，不参与 reason_code、blockers、generation 或 frame 的判定。

        返回 dict 字段:
        - ``ready`` (bool): 整体 ready
        - ``message`` (str): 顶层 reason
        - ``lcm_backend`` / ``tf`` / ``costmap`` / ``planner`` /
          ``cmd_vel_sink`` (bool): 子项
        - ``profile`` (str): 当前 NUWAX_NAV_PROFILE
        - ``module_node_count`` (int): ModuleGraph 节点数
        - ``p1_native_profiles`` (list[str]): 本期未启用的 native profile
        """
        self._raise_if_stopping("get_navigation_readiness")
        if not self._stub:
            return _nav_readiness_disconnected_dict()
        try:
            rpc_timeout = _exploration_rpc_timeout(
                deadline_monotonic,
                "GetNavigationReadiness",
            )
            resp = self._stub.GetNavigationReadiness(
                pb2.Empty(), timeout=rpc_timeout
            )
            readiness = _parse_navigation_readiness_reason(
                bool(resp.ready), str(resp.reason)
            )
            readiness.update(
                {
                    "reason_code": str(resp.reason_code),
                    "blockers": [
                        str(item) for item in resp.blockers
                    ],
                    "tf_generation": int(resp.tf_generation),
                    "planner_generation": int(
                        resp.planner_generation
                    ),
                    "motion_sink_generation": int(
                        resp.motion_sink_generation
                    ),
                    "map_frame": str(resp.map_frame),
                    "pose_frame": str(resp.pose_frame),
                    "forbidden_zone_map_name": str(
                        resp.forbidden_zone_map_name
                    ),
                    "forbidden_zone_generation": int(
                        resp.forbidden_zone_generation
                    ),
                    "forbidden_zone_signature": str(
                        resp.forbidden_zone_signature
                    ),
                    "forbidden_zone_applied_revision": int(
                        resp.forbidden_zone_applied_revision
                    ),
                }
            )
            planner_costmap_revision = int(
                resp.planner_costmap_revision
            )
            if planner_costmap_revision > 0:
                readiness["costmap_revision"] = planner_costmap_revision
            return readiness
        except grpc.RpcError as e:
            _log_rpc_error("GetNavigationReadiness", e)
            if deadline_monotonic is not None:
                raise _exploration_rpc_error(
                    "GetNavigationReadiness",
                    e,
                ) from e
            return _nav_readiness_disconnected_dict(message="gRPC 错误")

    # === NavigationProvider 协议 shim ===
    # provider bundle / runtime.py 使用 NavigationProvider 统一方法：
    # - provider bundle / runtime.py 直调 backend.set_goal/cancel_goal/get_navigation_state，
    #   而 GeometryRuntimeProvider gRPC facade 的真实方法名是 set_navigation_goal / cancel_navigation
    #   / (subscribe-driven 状态缓存)。
    # - 这里在 provider 侧补齐旧 NavigationProvider 协议方法名，全部委托到 gRPC facade；
    #   不假装成功——gRPC 失败或 backend 未就绪时，返回值如实反映（accepted=False / success=False / IDLE）。
    def _ensure_nav_state_stream(self) -> None:
        """懒启动可重连的 SubscribeNavigationState 后台线程。"""
        with self._nav_command_lock:
            with self._nav_state_lock:
                if self._nav_stream_started:
                    return
                stream_stub = self._stub
                if (
                    stream_stub is None
                    or self._stopping
                    or not self._connected
                    or self._nav_connection_transition_inflight
                ):
                    return
                self._nav_stream_epoch += 1
                stream_token = (
                    self._nav_connection_epoch,
                    self._nav_stream_epoch,
                    id(stream_stub),
                )
                thread = threading.Thread(
                    target=self._nav_state_stream_worker,
                    args=(stream_token, stream_stub),
                    name="geometry_runtime_nav_state_cache",
                    daemon=True,
                )
                self._nav_active_stream_token = stream_token
                self._nav_stream_started = True
                self._nav_stream_thread = thread
        try:
            thread.start()
        except Exception:
            with self._nav_command_lock:
                if self._nav_active_stream_token == stream_token:
                    self._nav_active_stream_token = None
                    with self._nav_state_lock:
                        self._nav_stream_started = False
                        self._nav_stream_thread = None
            raise

    def _navigation_stream_is_current_locked(
        self,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> bool:
        """Validate a stream token while `_nav_command_lock` is held."""

        return bool(
            self._nav_active_stream_token == stream_token
            and stream_token[0] == self._nav_connection_epoch
            and stream_token[2] == id(stream_stub)
            and self._stub is stream_stub
            and self._connected
            and not self._stopping
            and not self._nav_connection_transition_inflight
        )

    def _nav_state_stream_worker(
        self,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> None:
        """持续拉取导航状态；断流复位权威缓存并自动重连。"""

        try:
            while True:
                with self._nav_command_lock:
                    if not self._navigation_stream_is_current_locked(
                        stream_token,
                        stream_stub,
                    ):
                        return
                call = None
                updates = None
                try:
                    call = stream_stub.SubscribeNavigationState(pb2.Empty())
                    with self._nav_command_lock:
                        stream_is_current = (
                            self._navigation_stream_is_current_locked(
                                stream_token,
                                stream_stub,
                            )
                        )
                        if stream_is_current:
                            self._nav_stream_call = call
                    if not stream_is_current:
                        _cancel_stream_call(
                            call,
                            context="stale navigation",
                        )
                        return
                    updates = _coalesced_navigation_updates(call)
                    for update in updates:
                        notification = None
                        with self._nav_delivery_lock:
                            committed = self._cache_navigation_update(
                                update,
                                stream_token=stream_token,
                                stream_stub=stream_stub,
                            )
                            if committed:
                                self._cancel_pending_navigation_notifications_locked()
                                state = (
                                    self._build_navigation_state_snapshot(
                                        include_readiness=False
                                    )
                                )
                                notification = (
                                    self._begin_navigation_notification(
                                        state,
                                        stream_token=stream_token,
                                    )
                                )
                        if notification is not None:
                            self._notify_navigation_subscribers(notification)
                        with self._nav_command_lock:
                            if not self._navigation_stream_is_current_locked(
                                stream_token,
                                stream_stub,
                            ):
                                return
                except grpc.RpcError as exc:
                    LOGGER.warning(
                        "SubscribeNavigationState 流断开，准备重连: %s",
                        exc,
                    )
                    notification = None
                    with self._nav_delivery_lock:
                        disconnected = self._mark_navigation_stream_disconnected(
                            str(exc),
                            stream_token=stream_token,
                            stream_stub=stream_stub,
                        )
                        if disconnected:
                            self._cancel_pending_navigation_notifications_locked()
                            state = (
                                self._build_navigation_state_snapshot(
                                    include_readiness=False
                                )
                            )
                            notification = self._begin_navigation_notification(
                                state,
                                stream_token=stream_token,
                            )
                    if notification is not None:
                        self._notify_navigation_subscribers(notification)
                except Exception as exc:
                    LOGGER.exception(
                        "SubscribeNavigationState worker 异常，准备重连。"
                    )
                    notification = None
                    with self._nav_delivery_lock:
                        disconnected = self._mark_navigation_stream_disconnected(
                            str(exc),
                            stream_token=stream_token,
                            stream_stub=stream_stub,
                        )
                        if disconnected:
                            self._cancel_pending_navigation_notifications_locked()
                            state = (
                                self._build_navigation_state_snapshot(
                                    include_readiness=False
                                )
                            )
                            notification = self._begin_navigation_notification(
                                state,
                                stream_token=stream_token,
                            )
                    if notification is not None:
                        self._notify_navigation_subscribers(notification)
                finally:
                    if updates is not None:
                        updates.close()
                    if call is not None:
                        _cancel_stream_call(
                            call,
                            context="navigation",
                        )
                        with self._nav_command_lock:
                            if self._nav_stream_call is call:
                                self._nav_stream_call = None
                with self._nav_command_lock:
                    if not self._navigation_stream_is_current_locked(
                        stream_token,
                        stream_stub,
                    ):
                        return
                time.sleep(_grpc_stream_retry_sec())
        finally:
            with self._nav_command_lock:
                call = (
                    self._nav_stream_call
                    if self._nav_active_stream_token == stream_token
                    else None
                )
            if call is not None:
                _cancel_stream_call(
                    call,
                    context="navigation worker exit",
                )
            with self._nav_command_lock:
                if self._nav_active_stream_token == stream_token:
                    if self._nav_stream_call is call:
                        self._nav_stream_call = None
                    self._nav_active_stream_token = None
                    with self._nav_state_lock:
                        self._nav_stream_started = False
                        self._nav_stream_thread = None

    def _cache_navigation_update(
        self,
        update,
        *,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> bool:
        incoming_lease = _navigation_lease_from_message(update)
        status = update.status
        normalized_status = str(status).strip().lower()
        if normalized_status not in _NAV_STREAM_STATUS:
            LOGGER.warning(
                "丢弃未知 navigation stream status: %r",
                status,
            )
            return False
        if _navigation_update_is_admission_rejection(update):
            return False
        message = update.message
        distance = update.distance_remaining
        ts_ns = int(update.timestamp_ns)
        with self._nav_command_lock:
            if not self._navigation_stream_is_current_locked(
                stream_token,
                stream_stub,
            ) or self._nav_connection_transition_inflight:
                return False
            staged = self._stage_navigation_set_precommit_locked(
                update, incoming_lease=incoming_lease, stream_token=stream_token)
            with self._nav_state_lock:
                # 后继请求尚未接纳时，原目标仍拥有自己的真实状态；拒绝回包不能回滚它。
                inflight = self._nav_inflight_command_token
                previous_goal_update = bool(
                    inflight is not None and inflight[0] == "set"
                    and inflight[2] == self._nav_command_generation
                    and inflight[3] == self._nav_connection_epoch
                    and incoming_lease == self._current_navigation_lease_locked()
                    and _is_canonical_public_navigation_lease(incoming_lease))
                if staged and not previous_goal_update:
                    return False
                if (self._navigation_obligation_locked() is not None
                        and incoming_lease != self._current_navigation_lease_locked()):
                    # Includes foreign exploration overlays and their empty-IDLE clear.
                    return False
                if _is_canonical_exploration_child_navigation_lease(
                    incoming_lease
                ):
                    current_overlay = self._nav_exploration_overlay
                    if current_overlay is not None:
                        current_overlay_lease = _navigation_lease_from_message(
                            current_overlay
                        )
                        current_order = (
                            current_overlay_lease[4],
                            current_overlay_lease[3],
                        )
                        incoming_order = (incoming_lease[4], incoming_lease[3])
                        if incoming_lease == current_overlay_lease:
                            overlay_disconnected_placeholder = bool(
                                current_overlay.reason_code
                                == "NAVIGATION_STREAM_DISCONNECTED"
                            )
                            if (
                                not overlay_disconnected_placeholder
                                and not _navigation_status_can_follow(
                                    current_overlay.status,
                                    status,
                                    current_timestamp_ns=int(current_overlay.timestamp_ns),
                                    incoming_timestamp_ns=ts_ns,
                                    current_path_revision=int(current_overlay.path_revision),
                                    incoming_path_revision=int(update.path_revision),
                                )
                            ):
                                return False
                        elif incoming_order <= current_order:
                            return False
                    overlay = pb2.NavigationState()
                    overlay.CopyFrom(update)
                    self._nav_exploration_overlay = overlay
                    self._nav_exploration_overlay_terminal = (
                        normalized_status in _NAV_TERMINAL_STATUS
                    )
                    return True

                if incoming_lease == ("", 0, "", 0, 0) and normalized_status == "idle":
                    if self._nav_exploration_overlay is not None:
                        if not self._nav_exploration_overlay_terminal:
                            return False
                        self._nav_exploration_overlay = None
                        self._nav_exploration_overlay_terminal = False
                        self._last_nav_goal_id = None
                        self._last_nav_owner_kind = 0
                        self._last_nav_parent_request_id = ""
                        self._last_nav_goal_generation = 0
                        self._last_nav_producer_runtime_generation = 0
                        self._last_nav_goal_command_generation = (
                            self._nav_command_generation
                        )
                        self._last_nav_status = status
                        self._last_nav_message = message
                        self._last_nav_distance_remaining = (
                            float(distance) if distance is not None else None
                        )
                        self._last_nav_current_pose = None
                        self._last_nav_timestamp_ns = ts_ns
                        self._last_nav_metadata = (
                            _navigation_state_typed_metadata(update)
                        )
                        return True

                current_lease = self._current_navigation_lease_locked()
                current_goal_id = current_lease[0] or None
                # 当前流令牌已核验；无在途命令、无当前租约时采纳后端真实初始身份。
                unbound_current_connection = bool(
                    not _is_canonical_public_navigation_lease(current_lease)
                    and self._nav_inflight_command_token is None
                    and self._last_nav_goal_command_generation == self._nav_command_generation
                    and (_is_canonical_public_navigation_lease(incoming_lease)
                         or (incoming_lease == ("", 0, "", 0, 0) and normalized_status == "idle"))
                )
                lease_is_exact = bool(unbound_current_connection or (
                    incoming_lease == current_lease
                    and (
                        _is_canonical_public_navigation_lease(current_lease)
                        or (
                            current_lease == ("", 0, "", 0, 0)
                            and normalized_status == "idle"
                            and self._nav_inflight_command_token is None
                        )
                    )
                ))
                current_status = self._nav_status_to_enum(
                    self._last_nav_status
                )
                incoming_status = self._nav_status_to_enum(status)
                terminal_statuses = {
                    NavigationStatus.SUCCEEDED,
                    NavigationStatus.FAILED,
                    NavigationStatus.CANCELLED,
                }
                stream_disconnected_placeholder = bool(
                    self._last_nav_metadata.get("reason_code")
                    == "NAVIGATION_STREAM_DISCONNECTED"
                )
                # 已结束目标之后的无主空闲心跳是正常现象，不能每帧写日志再挤占状态交付。
                if (current_status in terminal_statuses and not stream_disconnected_placeholder
                        and incoming_lease == ("", 0, "", 0, 0) and normalized_status == "idle"):
                    return False
                inflight_token = self._nav_inflight_command_token
                inflight_cancel_terminal = bool(
                    inflight_token is not None
                    and inflight_token[0] == "cancel"
                    and inflight_token[1] == current_lease
                    and inflight_token[1] == incoming_lease
                    and inflight_token[2] == self._nav_command_generation
                    and inflight_token[3] == self._nav_connection_epoch
                    # 同一目标的原生终态结束在途取消；后继目标仍由事务身份隔离。
                    and incoming_status in terminal_statuses
                )
                command_generation_is_current = bool(
                    self._last_nav_goal_command_generation
                    == self._nav_command_generation
                )
                status_is_monotonic = bool(
                    unbound_current_connection or stream_disconnected_placeholder
                    or _navigation_status_can_follow(
                        self._last_nav_status,
                        status,
                        current_timestamp_ns=int(self._last_nav_timestamp_ns or 0),
                        incoming_timestamp_ns=ts_ns,
                        current_path_revision=int(self._last_nav_metadata.get("path_revision") or 0),
                        incoming_path_revision=int(update.path_revision),
                    )
                )
                if (
                    not lease_is_exact
                    or not self._navigation_owned_update_can_follow_locked(update)
                    or not (
                        command_generation_is_current
                        or inflight_cancel_terminal
                        or previous_goal_update
                    )
                    or not status_is_monotonic
                    or (
                        current_status in terminal_statuses
                        and not stream_disconnected_placeholder
                    )
                ):
                    LOGGER.info(
                        "丢弃 stale navigation stream frame: incoming_goal=%s/%s current_goal=%s/%s command=%s/%s status=%s->%s",
                        incoming_lease[0] or None,
                        incoming_lease[3],
                        current_goal_id,
                        current_lease[3],
                        self._last_nav_goal_command_generation,
                        self._nav_command_generation,
                        current_status.value,
                        incoming_status.value,
                    )
                    return False
                projected_status, projected_metadata = self._navigation_project_update_locked(update)
                self._last_nav_goal_id = incoming_lease[0] or None
                self._last_nav_owner_kind = incoming_lease[1]
                self._last_nav_parent_request_id = incoming_lease[2]
                self._last_nav_goal_generation = incoming_lease[3]
                self._last_nav_producer_runtime_generation = incoming_lease[4]
                self._last_nav_status = projected_status
                self._last_nav_message = message
                self._last_nav_distance_remaining = (
                    float(distance) if distance is not None else None
                )
                self._last_nav_current_pose = None
                self._last_nav_timestamp_ns = ts_ns
                self._last_nav_metadata = projected_metadata
                if inflight_cancel_terminal:
                    self._last_nav_goal_command_generation = inflight_token[2]
                    self._nav_inflight_command_token = None
                    self._nav_completed_cancel_command_token = inflight_token
                return True

    def _mark_navigation_stream_disconnected(
        self,
        error: str,
        *,
        stream_token: _NavigationStreamToken,
        stream_stub,
    ) -> bool:
        with self._nav_command_lock:
            if not self._navigation_stream_is_current_locked(
                stream_token,
                stream_stub,
            ):
                return False
            with self._nav_state_lock:
                current_overlay = self._nav_exploration_overlay
                if current_overlay is not None:
                    if self._nav_exploration_overlay_terminal:
                        return False
                    disconnected_overlay = pb2.NavigationState()
                    disconnected_overlay.CopyFrom(current_overlay)
                    disconnected_overlay.status = "failed"
                    disconnected_overlay.message = (
                        "导航状态流断开，正在重连：%s"
                        % (error or "unknown")
                    )
                    disconnected_overlay.timestamp_ns = time.time_ns()
                    disconnected_overlay.reason_code = (
                        "NAVIGATION_STREAM_DISCONNECTED"
                    )
                    self._nav_exploration_overlay = disconnected_overlay
                    self._nav_exploration_overlay_terminal = False
                    return True

                current_status = self._nav_status_to_enum(
                    self._last_nav_status
                )
                terminal_statuses = {
                    NavigationStatus.SUCCEEDED,
                    NavigationStatus.FAILED,
                    NavigationStatus.CANCELLED,
                }
                current_lease = self._current_navigation_lease_locked()
                if (
                    current_status in terminal_statuses
                    or self._last_nav_goal_command_generation
                    != self._nav_command_generation
                    or (
                        current_lease[0]
                        and not _is_canonical_public_navigation_lease(
                            current_lease
                        )
                    )
                ):
                    return False
                if self._navigation_obligation_locked(current_lease) is not None:
                    self._last_nav_metadata = {
                        **copy.deepcopy(self._last_nav_metadata),
                        "connection_diagnostic": "NAVIGATION_STREAM_DISCONNECTED",
                        "connection_error": str(error or "unknown"),
                    }
                    return True
                has_goal = bool(self._last_nav_goal_id)
                self._last_nav_status = "failed" if has_goal else "idle"
                self._last_nav_message = (
                    "导航状态流断开，正在重连：%s"
                    % (error or "unknown")
                )
                self._last_nav_distance_remaining = None
                self._last_nav_timestamp_ns = time.time_ns()
                self._last_nav_metadata = {
                    "raw_status": self._last_nav_status,
                    "reason_code": "NAVIGATION_STREAM_DISCONNECTED",
                    "observed_goal_id": self._last_nav_goal_id or "",
                    "owner_kind": self._last_nav_owner_kind,
                    "parent_request_id": (
                        self._last_nav_parent_request_id
                    ),
                    "goal_generation": self._last_nav_goal_generation,
                    "producer_runtime_generation": (
                        self._last_nav_producer_runtime_generation
                    ),
                }
                return True

    @staticmethod
    def _nav_status_to_enum(raw_status: str) -> NavigationStatus:
        """把 gRPC 流推送的 status 字符串映射成 NavigationStatus 枚举。

        未识别字符串映射为 IDLE（保守），绝不向上伪造 SUCCEEDED。
        """
        normalized = (raw_status or "").strip().lower()
        mapping = {
            "idle": NavigationStatus.IDLE,
            "unconfirmed": NavigationStatus.UNCONFIRMED,
            "accepted": NavigationStatus.ACCEPTED,
            "pending": NavigationStatus.ACCEPTED,
            "queued": NavigationStatus.ACCEPTED,
            "planning": NavigationStatus.PLANNING,
            "running": NavigationStatus.RUNNING,
            "moving": NavigationStatus.RUNNING,
            "succeeded": NavigationStatus.SUCCEEDED,
            "success": NavigationStatus.SUCCEEDED,
            "completed": NavigationStatus.SUCCEEDED,
            "arrived": NavigationStatus.SUCCEEDED,
            "reached": NavigationStatus.SUCCEEDED,
            "failed": NavigationStatus.FAILED,
            "failure": NavigationStatus.FAILED,
            "cancelled": NavigationStatus.CANCELLED,
            "canceled": NavigationStatus.CANCELLED,
        }
        return mapping.get(normalized, NavigationStatus.IDLE)

    def _build_navigation_state_snapshot(
        self,
        *,
        include_readiness: bool = True,
        include_exploration_overlay: bool = True,
        deadline_monotonic: Optional[float] = None,
    ) -> NavigationState:
        """合成 NavigationState 契约（按本地缓存 + readiness，不假装 ready）。

        - 未收到任何 SubscribeNavigationState 帧：返回 IDLE；
        - readiness 不就绪：state.message 携带 readiness 提示；
        - 缓存的 current_goal_id 与最近一次 set_goal 的 goal_id 不一致：以缓存为准
          （流是 Rust 侧权威来源）。
        """
        with _cooperative_deadline_lock(
            self._nav_state_lock,
            deadline_monotonic,
            "GetNavigationState",
        ):
            overlay = (
                self._nav_exploration_overlay
                if include_exploration_overlay
                else None
            )
            if overlay is not None:
                raw_status = str(overlay.status)
                goal_id = str(overlay.goal_id) or None
                message = str(overlay.message) or None
                distance = float(
                    overlay.distance_remaining
                )
                pose = None
                timestamp_ns = int(
                    overlay.timestamp_ns
                )
                typed_metadata = _navigation_state_typed_metadata(overlay)
            else:
                raw_status = self._last_nav_status
                goal_id = self._last_nav_goal_id
                message = self._last_nav_message
                distance = self._last_nav_distance_remaining
                pose = self._last_nav_current_pose
                timestamp_ns = self._last_nav_timestamp_ns
                typed_metadata = dict(self._last_nav_metadata)

        timestamp = (
            datetime.fromtimestamp(timestamp_ns / 1_000_000_000.0, tz=timezone.utc)
            if timestamp_ns > 0
            else datetime.now(timezone.utc)
        )

        status_enum = self._nav_status_to_enum(raw_status)
        # 没有任何流帧 → IDLE，goal_id 留空，绝不假装有 current goal
        if not raw_status:
            return NavigationState(
                timestamp=timestamp,
                updated_at=timestamp,
                status=NavigationStatus.IDLE,
                current_goal_id=None,
                goal_reached=False,
                message="GeometryRuntimeProvider 尚未收到 SubscribeNavigationState 流更新。",
                metadata={"provider": "geometry_runtime", "shim": True},
            )

        # readiness 兜底：未就绪时在 message 上补 waiting_for 提示（不伪造状态）
        metadata: Dict[str, Any] = {
            "provider": "geometry_runtime",
            "shim": True,
            **typed_metadata,
        }
        metadata.setdefault("raw_status", raw_status)
        if pose is not None:
            source_ns = int(pose.timestamp.timestamp() * 1_000_000_000)
            timing = {"source_timestamp_ns": source_ns}
            try:
                acquired_ns, acquired_mono_ns = acquired_times(source_ns)
                age_ns = (time.monotonic_ns() - acquired_mono_ns if acquired_mono_ns is not None
                          else time.time_ns() - acquired_ns)
                timing.update(acquired_unix_ns=acquired_ns, age_ms=age_ns / 1_000_000,
                              source_clock="simulation_logical" if acquired_mono_ns is not None else "unix")
            except SourceTimeError as exc:
                timing.update(acquired_unix_ns=None, age_ms=None, unavailable_reason=str(exc))
            metadata["current_pose_timing"] = timing
        if include_readiness:
            try:
                if deadline_monotonic is None:
                    readiness = self.get_navigation_readiness()
                else:
                    readiness = self.get_navigation_readiness(
                        deadline_monotonic=deadline_monotonic,
                    )
                if readiness:
                    metadata["readiness"] = dict(readiness)
                    if not bool(readiness.get("ready")):
                        hint = readiness.get("message") or readiness.get("waiting_for") or "导航后端未就绪"
                        if message:
                            message = "%s | %s" % (message, hint)
                        else:
                            message = str(hint)
            except GeometryRuntimeRpcError:
                if deadline_monotonic is not None:
                    raise
            except Exception:  # pragma: no cover - readiness 失败不影响主流程
                pass

        goal_reached = status_enum == NavigationStatus.SUCCEEDED

        return NavigationState(
            timestamp=timestamp,
            updated_at=timestamp,
            status=status_enum,
            current_goal_id=goal_id,
            current_pose=pose,
            remaining_distance_m=distance,
            remaining_yaw_rad=None,
            goal_reached=goal_reached,
            message=message,
            metadata=metadata,
        )

    def set_goal(
        self,
        goal,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        """NavigationProvider 协议 shim：下发导航目标到 nuwax-nav-stack。

        参数：
            goal: contracts.navigation.NavigationGoal 契约实例（runtime.py 的
                  _dispatch_navigate / platform_exploration_runtime 实际传入的对象）。

        返回：
            bool: 由 NavigationState.status 经 _status_means_accepted 反推（accepted/
            planning/running → True；failed/cancelled → False）；调用失败 / stub 不可用
            返回 False，绝不假装成功。
        """
        transaction_context = self._nav_set_transaction_context
        if getattr(transaction_context, "active", False):
            raise RuntimeError("reentrant_set_goal")
        request_goal_id = str(getattr(goal, "goal_id", "") or "").strip()
        transaction_context.result = _navigation_submission_result(request_goal_id)
        transaction_context.notification = None
        transaction_context.active = True
        try:
            with _cooperative_deadline_lock(
                self._nav_set_transaction_lock,
                deadline_monotonic,
                "SetNavigationGoal",
            ):
                # 命令事务自己推进代次并退休旧通知，显示回调不参与目标准入。
                accepted = self._set_goal_transaction(
                    goal,
                    deadline_monotonic=deadline_monotonic,
                )
                notification = transaction_context.notification

            if notification is not None:
                # 回执已写入状态缓存，真实状态流负责通知；命令返回不等待页面消费者。
                self._retire_navigation_notification(notification)
            if not accepted:
                submission = getattr(transaction_context, "result", None)
                if isinstance(submission, dict) and submission.get("goal_id") == request_goal_id:
                    LOGGER.warning("导航目标提交结果 goal_id=%s disposition=%s reason=%s",
                                   request_goal_id, submission.get("disposition"), submission.get("reason_code"))
            submission = getattr(transaction_context, "result", {}) or {}
            if ((accepted and deadline_monotonic is None) or submission.get("ownership_retained")):
                try:
                    self._ensure_nav_state_stream()
                except Exception:
                    LOGGER.exception(
                        "SetNavigationGoal 已提交，但导航状态流启动异常。"
                    )
            return accepted
        finally:
            transaction_context.notification = None
            transaction_context.active = False

    def take_goal_submission_result(self, goal_id: str):
        """领取本线程本次提交的结果；不读取可能已被另一目标覆盖的全局状态。"""
        context = self._nav_set_transaction_context
        result = getattr(context, "result", None)
        context.result = None
        if isinstance(result, dict) and result.get("goal_id") == goal_id:
            # Commit materializes cache paths and result paths independently and
            # copies the obligation. Clearing TLS transfers this detached result.
            return result
        return None

    def _set_goal_transaction(
        self,
        goal,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> bool:
        """Execute one SetGoal transaction while the Set-only gate is held."""

        self._raise_if_stopping("set_goal")
        if not self._stub:
            LOGGER.warning("GeometryRuntimeProvider.set_goal stub 不可用，返回 False。")
            return False
        target_pose = getattr(goal, "target_pose", None)
        if target_pose is None:
            LOGGER.warning(
                "GeometryRuntimeProvider.set_goal goal=%s 缺少 target_pose，返回 False。",
                getattr(goal, "goal_id", None),
            )
            return False
        command_generation = None
        command_token = None
        command_stub = None
        try:
            (
                motion_guard_boot_nonce,
                motion_stop_generation,
            ) = _decode_motion_admission_proof(
                dict(getattr(goal, "metadata", {}) or {}).get(
                    "motion_admission"
                )
            )
            proto_pose = pb2.Pose(
                position=pb2.Vec3(
                    x=float(target_pose.position.x),
                    y=float(target_pose.position.y),
                    z=float(target_pose.position.z),
                ),
                orientation=pb2.Quat(
                    x=float(target_pose.orientation.x),
                    y=float(target_pose.orientation.y),
                    z=float(target_pose.orientation.z),
                    w=float(target_pose.orientation.w),
                ),
                frame_id=str(getattr(target_pose, "frame_id", "") or ""),
                timestamp_ns=_datetime_to_timestamp_ns(
                    getattr(target_pose, "timestamp", None)
                ),
            )
            request_goal_id = str(
                getattr(goal, "goal_id", "") or ""
            ).strip()
            if not request_goal_id:
                LOGGER.warning("GeometryRuntimeProvider 拒绝空 goal_id。")
                return False
            canonical_parent_request_id = request_goal_id
            target_frame_id = str(getattr(target_pose, "frame_id", "") or "") or "map"
            proto_goal = pb2.NavigationGoal(
                pose=proto_pose,
                frame_id=target_frame_id,
                **navigation_reverse_wire_fields(getattr(goal, "allow_reverse", None)),
                position_tolerance_m=float(getattr(goal, "tolerance_position_m", 0.35)),
                yaw_tolerance_rad=(float(getattr(goal, "tolerance_yaw_rad", 0.3))
                    if getattr(goal, "require_goal_orientation", True) else math.pi),
                goal_id=request_goal_id,
                owner_kind=_PUBLIC_NAV_OWNER_KIND,
                parent_request_id=canonical_parent_request_id,
                goal_generation=0,
                motion_guard_boot_nonce=motion_guard_boot_nonce,
                motion_stop_generation=motion_stop_generation,
            )
            request_lease = (
                request_goal_id,
                _PUBLIC_NAV_OWNER_KIND,
                canonical_parent_request_id,
                0,
                0,
            )
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    command_token, command_stub = self._register_navigation_set_locked(request_lease)
                    if command_token is None:
                        return False
                    command_generation = command_token[2]
                self._cancel_pending_navigation_notifications_locked()
            rpc_timeout = _exploration_rpc_timeout(
                deadline_monotonic,
                "SetNavigationGoal",
            )
            resp = command_stub.SetNavigationGoal(
                proto_goal,
                timeout=rpc_timeout,
            )
            result = self._commit_navigation_submission(
                resp, command_token=command_token, command_stub=command_stub,
            )
            return bool(result["accepted"])
        except grpc.RpcError as exc:
            if command_generation is not None:
                with self._nav_delivery_lock:
                    with self._nav_command_lock:
                        if command_token is not None:
                            self._clear_navigation_set_precommit_locked(
                                command_token
                            )
                        if (
                            command_token is not None
                            and command_token
                            == self._nav_inflight_command_token
                            and command_generation
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._stub is command_stub
                        ):
                            self._nav_inflight_command_token = None
                            with self._nav_state_lock:
                                self._last_nav_goal_command_generation = (
                                    command_generation
                                )
            _log_rpc_error("SetNavigationGoal", exc)
            if deadline_monotonic is not None:
                raise _exploration_rpc_error("SetNavigationGoal", exc) from exc
            return False
        except Exception as exc:  # pragma: no cover - 防御性兜底
            if command_generation is not None:
                with self._nav_delivery_lock:
                    with self._nav_command_lock:
                        if command_token is not None:
                            self._clear_navigation_set_precommit_locked(
                                command_token
                            )
                        if (
                            command_token is not None
                            and command_token
                            == self._nav_inflight_command_token
                            and command_generation
                            == self._nav_command_generation
                            and command_token[3]
                            == self._nav_connection_epoch
                            and self._stub is command_stub
                        ):
                            self._nav_inflight_command_token = None
                            with self._nav_state_lock:
                                self._last_nav_goal_command_generation = (
                                    command_generation
                                )
            LOGGER.exception("GeometryRuntimeProvider.set_goal 异常。")
            if isinstance(exc, GeometryRuntimeRpcError):
                raise
            return False

    def cancel_goal(
        self,
        request_id: Optional[str] = None,
        *,
        deadline_monotonic: Optional[float] = None,
        global_stop_confirmation: Optional[StopAllMotionConfirmation] = None,
    ) -> bool:
        """转交定向取消；空身份由原生拥有方绑定当时的当前目标。"""
        self._raise_if_stopping("cancel_goal")
        exact_goal_id = str(request_id or "").strip()
        with _cooperative_deadline_lock(self._nav_delivery_lock, deadline_monotonic, "CancelNavigation"):
            with _cooperative_deadline_lock(self._nav_command_lock, deadline_monotonic, "CancelNavigation"):
                if (self._nav_connection_transition_inflight or not self._connected or self._stub is None):
                    return False
                inflight = self._nav_inflight_command_token
                if (inflight is not None and inflight[0] == "set"
                        and inflight[2] == self._nav_command_generation
                        and inflight[3] == self._nav_connection_epoch):
                    return False
                command_stub = self._stub
                with self._nav_state_lock:
                    active_lease = self._current_navigation_lease_locked()
                    active_status = self._nav_status_to_enum(self._last_nav_status)
                if exact_goal_id and (
                        active_lease[0] != exact_goal_id
                        or not _is_canonical_public_navigation_lease(active_lease)
                        or (global_stop_confirmation is None and active_status in {
                            NavigationStatus.IDLE, NavigationStatus.SUCCEEDED,
                            NavigationStatus.FAILED, NavigationStatus.CANCELLED})):
                    return False
                self._clear_navigation_set_precommit_locked()
                self._nav_command_generation += 1
                command_token = ("cancel", active_lease, self._nav_command_generation, self._nav_connection_epoch)
                self._nav_inflight_command_token = command_token
                self._nav_completed_cancel_command_token = None
            self._cancel_pending_navigation_notifications_locked()
        try:
            # 空请求不从显示缓存推导租约；原生在取消事务锁内选择当前目标。
            req = pb2.CancelNavigationRequest()
            if exact_goal_id:
                req.goal_id, req.owner_kind, req.parent_request_id = active_lease[:3]
                req.goal_generation, req.producer_runtime_generation = active_lease[3:]
            if global_stop_confirmation is not None:
                req.global_stop_confirmation.CopyFrom(
                    _ordinary_global_stop_confirmation_proto(global_stop_confirmation))
            resp = command_stub.CancelNavigation(
                req, timeout=_exploration_rpc_timeout(deadline_monotonic, "CancelNavigation"))
            raw_status = str(resp.status).strip().lower()
            response_lease = _navigation_lease_from_message(resp)
            success = (_status_means_cancel_success(raw_status) or raw_status in _NAV_TERMINAL_STATUS)
            notification = None
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    with self._nav_state_lock:
                        current_lease = self._current_navigation_lease_locked()
                        terminal_already_committed = bool(
                            command_token == self._nav_completed_cancel_command_token
                            and command_token[2] == self._nav_command_generation
                            and command_token[3] == self._nav_connection_epoch
                            and current_lease == active_lease
                            and str(self._last_nav_status).strip().lower() in _NAV_TERMINAL_STATUS)
                        command_current = bool(
                            command_token == self._nav_inflight_command_token
                            and command_token[2] == self._nav_command_generation
                            and command_token[3] == self._nav_connection_epoch
                            and command_stub is self._stub and self._connected
                            and not self._nav_connection_transition_inflight)
                        response_current = bool(command_current and current_lease == active_lease
                            and (not exact_goal_id or response_lease == active_lease)
                            and (_is_canonical_public_navigation_lease(response_lease)
                                 or (not exact_goal_id and _is_canonical_exploration_child_navigation_lease(response_lease))))
                        if terminal_already_committed:
                            self._nav_completed_cancel_command_token = None
                            return True
                        if not response_current:
                            return False
                        self._nav_inflight_command_token = None
                        self._last_nav_goal_command_generation = command_token[2]
                        if _is_canonical_exploration_child_navigation_lease(response_lease):
                            overlay = pb2.NavigationState()
                            overlay.CopyFrom(resp)
                            self._nav_exploration_overlay = overlay
                            self._nav_exploration_overlay_terminal = raw_status in _NAV_TERMINAL_STATUS
                        elif raw_status == "pending" and success and response_lease == active_lease:
                            self._last_nav_metadata = {
                                **self._last_nav_metadata,
                                "cancel_phase": str(resp.cancel_phase),
                                "cancel_generation": int(resp.cancel_generation),
                                "cancel_message": resp.message,
                            }
                        elif success:
                            status, metadata = self._navigation_project_update_locked(resp)
                            self._last_nav_goal_id = response_lease[0]
                            self._last_nav_owner_kind = response_lease[1]
                            self._last_nav_parent_request_id = response_lease[2]
                            self._last_nav_goal_generation = response_lease[3]
                            self._last_nav_producer_runtime_generation = response_lease[4]
                            self._last_nav_status = status
                            self._last_nav_metadata = metadata
                            self._last_nav_timestamp_ns = int(resp.timestamp_ns)
                            self._last_nav_distance_remaining = float(resp.distance_remaining)
                            self._last_nav_message = resp.message
                if success:
                    state = self._build_navigation_state_snapshot(
                        include_readiness=False, include_exploration_overlay=False)
                    notification = self._begin_navigation_notification(state)
            if notification is not None:
                self._notify_navigation_subscribers(notification)
            return success
        except grpc.RpcError as exc:
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    with self._nav_state_lock:
                        terminal_already_committed = bool(
                            command_token == self._nav_completed_cancel_command_token
                            and command_token[2] == self._nav_command_generation
                            and command_token[3] == self._nav_connection_epoch
                            and self._current_navigation_lease_locked() == active_lease
                            and str(self._last_nav_status).strip().lower() in _NAV_TERMINAL_STATUS)
                        if terminal_already_committed:
                            self._nav_completed_cancel_command_token = None
            _log_rpc_error("CancelNavigation", exc)
            if terminal_already_committed:
                return True
            if deadline_monotonic is not None:
                raise _exploration_rpc_error("CancelNavigation", exc) from exc
            return False
        finally:
            with self._nav_delivery_lock:
                with self._nav_command_lock:
                    if (command_token == self._nav_inflight_command_token
                            and command_token[2] == self._nav_command_generation
                            and command_token[3] == self._nav_connection_epoch
                            and command_stub is self._stub):
                        self._nav_inflight_command_token = None
                        with self._nav_state_lock:
                            self._last_nav_goal_command_generation = command_token[2]

    def is_goal_reached(self) -> bool:
        """NavigationProvider 协议 shim：当前目标是否已到达。

        基于本地缓存（SubscribeNavigationState 最近一帧的 status）。
        """
        with self._nav_state_lock:
            return _status_means_succeeded(self._last_nav_status)

    def get_navigation_control_state(
        self, *, deadline_monotonic: Optional[float] = None,
    ) -> NavigationState:
        """只读已核验导航流，停车对账不附带就绪或定位远程查询。"""
        return self._build_navigation_state_snapshot(
            include_readiness=False, deadline_monotonic=deadline_monotonic,
        )

    def get_navigation_state(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> NavigationState:
        """NavigationProvider 协议 shim：返回 NavigationState 契约实例。

        第一次调用前若无 set_goal/get_navigation_state 触发，状态为 IDLE；订阅流启动后
        Rust 端推送会持续覆盖本地缓存。
        """
        _exploration_rpc_timeout(deadline_monotonic, "GetNavigationState")
        if deadline_monotonic is None:
            self._ensure_nav_state_stream()
        state = self._build_navigation_state_snapshot(
            deadline_monotonic=deadline_monotonic,
        )
        pose = self.get_current_pose(deadline_monotonic=deadline_monotonic)
        return state.model_copy(update={"current_pose": pose})

    def subscribe_navigation_updates(
        self,
        callback: Callable[[NavigationState], None],
    ) -> Optional[int]:
        if not callable(callback):
            raise ValueError("navigation update callback must be callable")
        with self._nav_delivery_lock:
            with self._nav_command_lock:
                if (
                    self._stub is None
                    or self._stopping
                    or not self._connected
                    or self._nav_connection_transition_inflight
                ):
                    return None
                with self._nav_state_lock:
                    subscription_id = self._next_navigation_subscription_id
                    self._next_navigation_subscription_id += 1
                    self._navigation_update_subscribers[
                        subscription_id
                    ] = callback
        self._ensure_nav_state_stream()
        return subscription_id

    def unsubscribe_navigation_updates(self, subscription_id: int) -> bool:
        subscription_id = int(subscription_id)
        with self._nav_delivery_lock:
            with self._nav_state_lock:
                removed = self._navigation_update_subscribers.pop(
                    subscription_id,
                    None,
                )
        if removed is None:
            return False
        self._wait_for_navigation_subscription_callbacks(subscription_id)
        return True

    def _notify_navigation_subscribers(
        self,
        ticket: _NavigationNotificationTicket,
    ) -> None:
        try:
            for subscription_id, callback in ticket.subscribers:
                if not self._begin_navigation_notification_callback(
                    ticket,
                    subscription_id,
                    callback,
                ):
                    continue
                try:
                    callback(ticket.state.model_copy(deep=True))
                except Exception:
                    LOGGER.exception("导航状态回调失败，其他订阅者继续。")
                finally:
                    self._finish_navigation_notification(
                        ticket,
                        subscription_id,
                    )
        finally:
            self._retire_navigation_notification(ticket)

    def navigate(self, goal, timeout_sec: Optional[int] = None) -> str:
        del timeout_sec
        return str(getattr(goal, "goal_id", "") or "") if self.set_goal(goal) else ""

    def cancel(self, request_id: Optional[str] = None) -> bool:
        return self.cancel_goal(request_id=request_id)

    def get_state(self, request_id: Optional[str] = None) -> NavigationState:
        state = self.get_navigation_state()
        if request_id and state.current_goal_id not in {None, request_id}:
            metadata = dict(state.metadata)
            metadata["requested_goal_id"] = request_id
            metadata["goal_id_mismatch"] = True
            return state.model_copy(update={"metadata": metadata}, deep=True)
        return state

    # === 探索 RPC（Phase 5 additive, 走 Rust explore_runtime） ===
    def start_exploration(
        self,
        *,
        request_id: str = "",
        strategy: str = "wavefront",
        arbitration: str = "prefer_wavefront",
        boundary_line_segments=None,
        stop_conditions: Optional[Dict[str, Any]] = None,
        max_navigation_retries: int = 0,
        blacklist_radius_m: float = 0.0,
        resume_progress_json: Optional[str] = None,
        return_policy: Optional[Dict[str, Any]] = None,
        motion_admission: Optional[Dict[str, Any]] = None,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        """启动探索(走 Rust `nuwax_exploration::ExplorationController`)。

        完整编码顺序边界、typed stop conditions、重试与 blacklist 参数。
        stub 未就绪或 RPC 异常时抛出带 gRPC 细节的 typed error。
        """
        self._raise_if_stopping("start_exploration")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error("StartExploration")
        try:
            (
                motion_guard_boot_nonce,
                motion_stop_generation,
            ) = _decode_motion_admission_proof(motion_admission)
            proto_segments = []
            for start_pose, end_pose in boundary_line_segments or ():
                for pose in (start_pose, end_pose):
                    frame_id = str(pose.frame_id or "").strip().lstrip("/")
                    if frame_id != DEFAULT_MAP_FRAME:
                        raise ValueError(
                            "exploration boundary frame must be map, received %s"
                            % (frame_id or "<empty>")
                        )
                proto_segments.append(
                    pb2.ExplorationBoundarySegment(
                        start_x=float(start_pose.position.x),
                        start_y=float(start_pose.position.y),
                        end_x=float(end_pose.position.x),
                        end_y=float(end_pose.position.y),
                    )
                )

            req = pb2.ExploreRequest(
                strategy=strategy,
                arbitration=arbitration,
                request_id=request_id,
                boundary_line_segments=proto_segments,
                boundary_frame_id=DEFAULT_MAP_FRAME if proto_segments else "",
                max_navigation_retries=max(0, int(max_navigation_retries)),
                blacklist_radius_m=max(0.0, float(blacklist_radius_m)),
                motion_guard_boot_nonce=motion_guard_boot_nonce,
                motion_stop_generation=motion_stop_generation,
            )
            if resume_progress_json:
                if len(resume_progress_json.encode("utf-8")) > 2_097_152:
                    raise ValueError("探索恢复载荷超过两兆字节")
                req.resume_progress_json = resume_progress_json
            if return_policy is not None:
                from contracts.exploration_mission import ExplorationReturnPolicy
                policy = ExplorationReturnPolicy.model_validate(return_policy)
                req.return_policy.CopyFrom(pb2.ExplorationReturnPolicy(**policy.model_dump(exclude_none=True)))
            raw_stop_conditions = dict(stop_conditions or {})
            recognized_stop_keys = {
                "max_coverage_pct",
                "no_frontier_timeout_ms",
                "max_duration_ms",
                "max_goals",
            }
            if any(
                key in raw_stop_conditions and raw_stop_conditions[key] is not None
                for key in recognized_stop_keys
            ):
                proto_stop_conditions = pb2.ExplorationStopConditions()
                if raw_stop_conditions.get("max_coverage_pct") is not None:
                    proto_stop_conditions.max_coverage_pct = float(
                        raw_stop_conditions["max_coverage_pct"]
                    )
                if raw_stop_conditions.get("no_frontier_timeout_ms") is not None:
                    proto_stop_conditions.no_frontier_timeout_ms = max(
                        0, int(raw_stop_conditions["no_frontier_timeout_ms"])
                    )
                if raw_stop_conditions.get("max_duration_ms") is not None:
                    proto_stop_conditions.max_duration_ms = max(
                        0, int(raw_stop_conditions["max_duration_ms"])
                    )
                if raw_stop_conditions.get("max_goals") is not None:
                    proto_stop_conditions.max_goals = max(
                        0, int(raw_stop_conditions["max_goals"])
                    )
                req.stop_conditions.CopyFrom(proto_stop_conditions)
            rpc_timeout = _exploration_rpc_timeout(
                deadline_monotonic,
                "StartExploration",
            )
            resp = stub.StartExploration(req, timeout=rpc_timeout)
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            raise _exploration_rpc_error("StartExploration", e) from e

    def get_exploration_progress(self, *, request_id: str) -> str:
        """读取已停止原任务的进度；原生端拒绝仍持有子导航的任务。"""
        self._raise_if_stopping("get_exploration_progress")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error("GetExplorationProgress")
        try:
            return stub.GetExplorationProgress(pb2.ExplorationProgressRequest(request_id=request_id), timeout=5).checkpoint_json
        except grpc.RpcError as error:
            raise _exploration_rpc_error("GetExplorationProgress", error) from error

    def stop_exploration(
        self,
        *,
        request_id: str = "",
        deadline_monotonic: Optional[float] = None,
        global_stop_confirmation: Optional[StopAllMotionConfirmation] = None,
        expected_exploration_generation: Optional[int] = None,
    ) -> Dict[str, Any]:
        """停止探索。

        返回 ExplorationState 字典；失败时抛出带 gRPC 细节的 typed error。
        """
        self._raise_if_stopping("stop_exploration")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error("StopExploration")
        try:
            req = pb2.StopExplorationRequest(request_id=request_id)
            if global_stop_confirmation is not None:
                req.global_stop_confirmation.CopyFrom(
                    _ordinary_global_stop_confirmation_proto(global_stop_confirmation))
            if expected_exploration_generation is not None:
                req.expected_exploration_generation = expected_exploration_generation
            rpc_timeout = _exploration_rpc_timeout(
                deadline_monotonic,
                "StopExploration",
            )
            resp = stub.StopExploration(req, timeout=rpc_timeout)
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            raise _exploration_rpc_error("StopExploration", e) from e




    def get_exploration_state(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        """读取当前 exploration scheduler 的只读快照。

        返回 ExplorationState 字典；失败时抛出带 gRPC 细节的 typed error。
        """
        self._raise_if_stopping("get_exploration_state")
        stub = self._stub
        if stub is None:
            raise _missing_exploration_stub_error("GetExplorationState")
        try:
            rpc_timeout = _exploration_rpc_timeout(
                deadline_monotonic,
                "GetExplorationState",
            )
            resp = stub.GetExplorationState(pb2.Empty(), timeout=rpc_timeout)
            return _exploration_state_to_dict(resp)
        except grpc.RpcError as e:
            raise _exploration_rpc_error("GetExplorationState", e) from e

    @staticmethod
    def _exploration_state_signature(state: Dict[str, Any]):
        """返回不含采样时间的状态身份，用于抑制轮询重复帧。"""

        return (
            state.get("status"),
            state.get("message"),
            state.get("request_id"),
            state.get("selected_source"),
            state.get("last_stop_reason"),
            state.get("frontier_count"),
            state.get("revision"),
            state.get("child_task_id"),
            state.get("exploration_generation"),
            state.get("child_lease"),
            state.get("parking_pending"),
        )

    def _ensure_exploration_poll_worker(self) -> None:
        """首个订阅者到来时启动唯一的 exploration 状态轮询线程。"""

        with self._exploration_state_lock:
            if (
                not self._exploration_update_subscribers
                or not self._stub
                or self._stopping
            ):
                return
            current = self._exploration_poll_thread
            if current is not None and current.is_alive():
                return
            stop_event = threading.Event()
            thread = threading.Thread(
                target=self._exploration_poll_worker,
                args=(stop_event,),
                name="geometry_runtime_exploration_state_poll",
                daemon=True,
            )
            self._exploration_poll_stop_event = stop_event
            self._exploration_poll_thread = thread
            self._last_exploration_signature = None
            thread.start()

    def _exploration_poll_worker(self, stop_event: threading.Event) -> None:
        """轮询 frozen unary RPC；失败指数退避，成功后按状态身份去重。"""

        retry_base = _grpc_stream_retry_sec()
        retry_max = _grpc_stream_retry_max_sec()
        retry_delay = retry_base
        try:
            while not stop_event.is_set():
                with self._exploration_state_lock:
                    if not self._exploration_update_subscribers:
                        return
                    stub = self._stub
                if stub is None or self._stopping:
                    return
                try:
                    response = stub.GetExplorationState(
                        pb2.Empty(),
                        timeout=_read_deadline_sec(),
                    )
                    state = _exploration_state_to_dict(response)
                except grpc.RpcError as exc:
                    if stop_event.is_set() or self._stopping:
                        return
                    LOGGER.warning(
                        "GetExplorationState 后台轮询失败，%.2fs 后重试: %s",
                        retry_delay,
                        exc,
                    )
                    if stop_event.wait(retry_delay):
                        return
                    retry_delay = min(retry_max, retry_delay * 2.0)
                    continue
                except Exception as exc:  # pragma: no cover - 防御第三方 stub
                    if stop_event.is_set() or self._stopping:
                        return
                    LOGGER.warning(
                        "GetExplorationState 后台轮询异常，%.2fs 后重试: %s",
                        retry_delay,
                        exc,
                    )
                    if stop_event.wait(retry_delay):
                        return
                    retry_delay = min(retry_max, retry_delay * 2.0)
                    continue

                retry_delay = retry_base
                signature = self._exploration_state_signature(state)
                with self._exploration_state_lock:
                    if signature == self._last_exploration_signature:
                        subscribers = ()
                    else:
                        subscribers = tuple(
                            self._exploration_update_subscribers.values()
                        )
                accepted_delivery = False
                for callback in subscribers:
                    try:
                        # ``None`` is the long-standing callback success value;
                        # only an explicit ``False`` rejects this snapshot.
                        if callback(dict(state)) is not False:
                            accepted_delivery = True
                    except Exception:
                        LOGGER.exception("探索状态回调失败，其他订阅者继续。")
                if accepted_delivery:
                    with self._exploration_state_lock:
                        self._last_exploration_signature = signature
                if stop_event.wait(_exploration_poll_sec()):
                    return
        finally:
            with self._exploration_state_lock:
                if self._exploration_poll_thread is threading.current_thread():
                    self._exploration_poll_thread = None
                if self._exploration_poll_stop_event is stop_event:
                    self._exploration_poll_stop_event = None
                should_restart = bool(
                    self._exploration_update_subscribers
                    and self._stub is not None
                    and not self._stopping
                )
            # 覆盖“最后一个订阅刚取消、另一线程立刻新订阅”的交错：旧 worker
            # 已收到 stop，但新订阅不得因此永久失去后台状态源。
            if should_restart:
                self._ensure_exploration_poll_worker()

    def subscribe_exploration_updates(
        self,
        callback: Callable[[Dict[str, Any]], None],
    ) -> Optional[int]:
        if not callable(callback):
            raise ValueError("exploration update callback must be callable")
        if not self._stub or self._stopping:
            return None
        with self._exploration_state_lock:
            subscription_id = self._next_exploration_subscription_id
            self._next_exploration_subscription_id += 1
            self._exploration_update_subscribers[subscription_id] = callback
        self._ensure_exploration_poll_worker()
        return subscription_id

    def unsubscribe_exploration_updates(self, subscription_id: int) -> bool:
        with self._exploration_state_lock:
            removed = self._exploration_update_subscribers.pop(
                int(subscription_id),
                None,
            )
            if removed is None:
                return False
            if self._exploration_update_subscribers:
                return True
            stop_event = self._exploration_poll_stop_event
            thread = self._exploration_poll_thread
            self._last_exploration_signature = None
            if stop_event is not None:
                stop_event.set()
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=_read_deadline_sec() + 0.5)
        return True

    def get_exploration_diagnostics(self) -> Optional[Dict[str, Any]]:
        """读取探索 diagnostics(Phase 7:主路径走新增的 `GetExplorationReadiness`)。

        返回 dict 字段:
          - 旧字段(active / arbitration / selected_source / last_stop_reason /
            sources(list) / frontier_count / revision / timestamp_ns)
          - Phase 7 readiness 字段(ready / backend_available / controller_active /
            source_count / frontier_source_ready / costmap_input_ready /
            costmap_input_revision / active_source / message)

        失败时返回 None。
        """
        self._raise_if_stopping("get_exploration_diagnostics")
        if not self._stub:
            return None
        # Phase 7:主路径走 GetExplorationReadiness(JSON message)。
        try:
            resp = self._stub.GetExplorationReadiness(
                pb2.Empty(), timeout=_read_deadline_sec()
            )
            payload = _parse_readiness_message(resp.message)
            payload["ready"] = bool(resp.ready)
            # 兼容旧字段(snake_case → 已有 Python 调用方)
            payload.setdefault("backend_available", payload.get("backend_available"))
            payload.setdefault("controller_active", payload.get("controller_active"))
            payload.setdefault("active_source", payload.get("active_source"))
            payload.setdefault("sources", [])
            payload.setdefault("frontier_count", 0)
            payload.setdefault("selected_source", payload.get("active_source", ""))
            payload.setdefault("last_stop_reason", "")
            payload.setdefault("revision", payload.get("costmap_input_revision"))
            payload.setdefault("timestamp_ns", 0)
            payload.setdefault("arbitration", payload.get("arbitration"))
            payload["error"] = None
            return payload
        except grpc.RpcError as e:
            _log_rpc_error("GetExplorationReadiness", e)
            # 兜底:旧 GetExplorationDiagnostics。
            try:
                resp = self._stub.GetExplorationDiagnostics(
                    pb2.Empty(), timeout=_read_deadline_sec()
                )
                sources = [
                    {
                        "source_name": s.source_name,
                        "is_active": bool(s.is_active),
                        "last_goal_id": s.last_goal_id,
                        "last_error": s.last_error,
                    }
                    for s in resp.sources
                ]
                return {
                    "ready": bool(resp.active),
                    "active": bool(resp.active),
                    "backend_available": None,
                    "controller_active": bool(resp.active),
                    "source_count": len(sources),
                    "frontier_source_ready": bool(resp.active) and len(sources) > 0,
                    "costmap_input_ready": False,
                    "costmap_input_revision": 0,
                    "active_source": resp.selected_source,
                    "arbitration": resp.arbitration,
                    "selected_source": resp.selected_source,
                    "last_stop_reason": resp.last_stop_reason,
                    "sources": sources,
                    "frontier_count": int(resp.frontier_count),
                    "revision": int(resp.revision),
                    "timestamp_ns": int(resp.timestamp_ns),
                    "message": "Phase 6 fallback: GetExplorationReadiness 不可用",
                    "error": "GetExplorationReadiness 失败: %s" % e,
                }
            except grpc.RpcError as e2:
                _log_rpc_error("GetExplorationDiagnostics", e2)
                return None

    # === 输入桥 ===
    # Rust proto PublishOdom 接受 PublishOdomRequest（仅 pose 字段；速度由 nuwax-nav-stack
    # 内部从 pose 增量计算，不再走 gRPC facade）。速度参数保留以兼容旧调用方签名，丢弃。
    def get_geometry_pipeline(self) -> Dict[str, Any]:
        """读取实际进程的执行链和运行代际，用于输入桥和网页能力绑定。"""
        self._raise_if_stopping("get_geometry_pipeline")
        if not self._stub:
            raise RuntimeError("Geometry尚未连接")
        response = self._stub.HealthCheck(pb2.HealthCheckRequest(), timeout=_read_deadline_sec())
        self._planar_unix_offset_ns = int(response.planar_unix_offset_ns)
        return {
            "healthy": bool(response.healthy),
            "geometry_pipeline": str(response.geometry_pipeline or "lio_3d"),
            "runtime_generation": int(response.runtime_generation),
            "planar_input_config": json.loads(response.planar_input_config) if response.planar_input_config else None,
            "planar_unix_offset_ns": int(response.planar_unix_offset_ns),
        }

    def capture_planar_input_context(self, runtime_generation: int):
        """回调捕获当前会话，排队数据始终携带此身份，发送时不能改绑。"""
        with self._map_state_lock:
            if self._stopping or self._mapping_transition_inflight or self._mapping_identity is None or not self._planar_input_active:
                return None
            identity = self._mapping_identity
            return {
                "session_id": identity[0],
                "session_generation": identity[2],
                "runtime_generation": runtime_generation,
            }

    def publish_laser_scan(self, scan, context: Dict[str, Any]) -> bool:
        """发送规范二维扫描；成功表示通信接纳，算法就绪由运行状态表达。"""
        from services.geometry.planar_wire import encode_planar_scan
        self._raise_if_stopping("publish_laser_scan")
        if not self._stub:
            return False
        if scan.scan_received_unix_ns is not None:
            stamp = scan.timestamp_ns + (self._planar_unix_offset_ns if scan.clock_domain == "monotonic_ns" else 0)
            with self._map_state_lock:
                identity = self._mapping_identity
                if identity is not None and identity[0] == context["session_id"] and identity[2] == context["session_generation"]:
                    self._planar_scan_receipts[stamp] = (identity, scan.scan_received_unix_ns)
                    if len(self._planar_scan_receipts) > 256:
                        self._planar_scan_receipts.pop(next(iter(self._planar_scan_receipts)))
        payload = encode_planar_scan(scan, int(context["runtime_generation"]))
        response = self._stub.PublishLaserScan(pb2.PublishLaserScanRequest(
            payload=payload, session_id=context["session_id"],
            session_generation=context["session_generation"],
        ), timeout=_read_deadline_sec())
        return bool(response.success)

    def publish_planar_odometry(self, pose, timestamp_ns: int, clock_domain: str,
                                clock_id: str, context: Dict[str, Any]) -> bool:
        """设备里程计保留源时间和时钟身份，禁止使用回调接收时间补齐。"""
        self._raise_if_stopping("publish_planar_odometry")
        if not self._stub:
            return False
        response = self._stub.PublishOdom(pb2.PublishOdomRequest(
            pose=pb2.Pose(
                position=pb2.Vec3(x=pose.position.x, y=pose.position.y, z=pose.position.z),
                orientation=pb2.Quat(x=pose.orientation.x, y=pose.orientation.y,
                                     z=pose.orientation.z, w=pose.orientation.w),
                frame_id=pose.frame_id, timestamp_ns=timestamp_ns,
            ), clock_domain=clock_domain, clock_id=clock_id, **context,
        ), timeout=_read_deadline_sec())
        return bool(response.success)

    def publish_odom(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float,
                     lvx: float = 0, lvy: float = 0, lvz: float = 0,
                     avx: float = 0, avy: float = 0, avz: float = 0,
                     frame_id: str = "odom", timestamp_ns: int = 0) -> bool:
        del lvx, lvy, lvz, avx, avy, avz  # round-4 HIGH 3：Rust 端不再接收 velocity 字段
        self._raise_if_stopping("publish_odom")
        if not self._stub:
            return False
        try:
            msg = pb2.PublishOdomRequest(
                pose=pb2.Pose(
                    position=pb2.Vec3(x=x, y=y, z=z),
                    orientation=pb2.Quat(x=qx, y=qy, z=qz, w=qw),
                    frame_id=frame_id,
                    timestamp_ns=max(0, int(timestamp_ns)),
                ),
            )
            resp = self._stub.PublishOdom(msg, timeout=_read_deadline_sec())
            return bool(resp.success)
        except grpc.RpcError as e:
            _log_rpc_error("PublishOdom", e)
            return False

    def report_native_input_failure(self, reason: str, first_missing_source_timestamp_ns: int) -> bool:
        """将生产者连续性故障锁定到当前原生代际，不能靠新建输入桥绕过。"""
        self._native_input_failure_reason = str(reason)
        self._raise_if_stopping("report_native_input_failure")
        if self._stub is None:
            return False
        response = self._stub.ReportNativeInputFailure(
            pb2.NativeInputFailureRequest(
                reason=str(reason).encode("utf-8")[:1000].decode("utf-8", errors="ignore"),
                first_missing_source_timestamp_ns=max(0, int(first_missing_source_timestamp_ns)),
            ), timeout=3.0,
        )
        return bool(response.success)

    def publish_point_cloud(
        self,
        payload: bytes,
        point_step: int,
        *,
        frame_id: str = "",
        timestamp_ns: int = 0,
        fields: Optional[List[Dict[str, Any]]] = None,
        point_count: int = 0,
        is_dense: bool = True,
        imu_samples: Optional[List[Any]] = None,
        mapping_only: bool = False,
        sensor_origin_xyz: Optional[List[float]] = None,
        sensor_id: str = "",
        source_sequence: int = 0,
        clock_id: str = "",
        calibration_ref: str = "",
    ) -> bool:
        """Publish normalized PointCloud2 data through the Rust gRPC facade.

        ``payload`` and ``point_step`` remain positional for compatibility with the
        original bridge call shape. Point-cloud metadata is keyword-only, and
        ``frame_id`` is semantically required: missing or whitespace-padded values
        are rejected before any RPC instead of being repaired to a legacy frame.
        """
        self._raise_if_stopping("publish_point_cloud")
        if imu_samples and self._native_input_failure_reason:
            raise RuntimeError("原生输入连续性已失败，需重启拥有方：%s" % self._native_input_failure_reason)
        raw_frame_id = str(frame_id or "")
        normalized_frame_id = raw_frame_id.strip()
        if not normalized_frame_id or normalized_frame_id != raw_frame_id:
            return False
        if not self._stub:
            return False

        proto_fields = []
        for point_field in fields or []:
            proto_fields.append(
                pb2.PointCloudField(
                    name=str(point_field.get("name", "")),
                    offset=max(0, int(point_field.get("offset", 0))),
                    datatype=max(0, int(point_field.get("datatype", 0))),
                    count=max(0, int(point_field.get("count", 0))),
                )
            )

        proto_imu_samples = []
        for sample in imu_samples or []:
            accel = tuple(getattr(sample, "accel_mps2"))
            gyro = tuple(getattr(sample, "gyro_rps"))
            if len(accel) != 3 or len(gyro) != 3:
                return False
            proto_imu_samples.append(
                pb2.LioImuSample(
                    timestamp_ns=max(0, int(getattr(sample, "timestamp_ns"))),
                    frame_id=str(getattr(sample, "frame_id")),
                    accel_mps2_x=float(accel[0]),
                    accel_mps2_y=float(accel[1]),
                    accel_mps2_z=float(accel[2]),
                    gyro_rps_x=float(gyro[0]),
                    gyro_rps_y=float(gyro[1]),
                    gyro_rps_z=float(gyro[2]),
                )
            )

        try:
            request = pb2.PublishPointCloudRequest(
                points=bytes(payload),
                point_step=max(0, int(point_step)),
                frame_id=normalized_frame_id,
                timestamp_ns=max(0, int(timestamp_ns)),
                fields=proto_fields,
                point_count=max(0, int(point_count)),
                is_dense=bool(is_dense),
                imu_samples=proto_imu_samples,
                mapping_only=mapping_only,
                sensor_origin_xyz=sensor_origin_xyz if sensor_origin_xyz is not None else [],
                sensor_id=sensor_id,
                source_sequence=source_sequence,
                clock_id=clock_id,
                calibration_ref=calibration_ref,
            )
            response = self._stub.PublishPointCloud(
                request,
                timeout=_heavy_deadline_sec(),
            )
            if imu_samples and not bool(response.success):
                raise RuntimeError("原生输入未确认：%s" % response.message)
            return bool(response.success)
        except grpc.RpcError as e:
            _log_rpc_error("PublishPointCloud", e)
            if imu_samples:
                raise RuntimeError("原生输入RPC失败：%s %s" % (e.code(), e.details())) from e
            return False

    def get_status(
        self,
        *,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        self._raise_if_stopping("get_status")
        if not self._stub:
            if deadline_monotonic is not None:
                raise GeometryRuntimeRpcError(
                    rpc_name="GetStatus",
                    grpc_code=grpc.StatusCode.UNAVAILABLE,
                    grpc_details="geometry runtime gRPC stub is unavailable",
                )
            return {"phase": "disconnected", "rss_kb": 0}
        try:
            rpc_timeout = _exploration_rpc_timeout(
                deadline_monotonic,
                "GetStatus",
            )
            resp = self._stub.GetStatus(pb2.Empty(), timeout=rpc_timeout)
            return {"phase": resp.phase, "message": resp.message, "rss_kb": resp.rss_kb}
        except grpc.RpcError as e:
            _log_rpc_error("GetStatus", e)
            if deadline_monotonic is not None:
                raise GeometryRuntimeRpcError.from_rpc_error("GetStatus", e) from e
            return {"phase": "error", "rss_kb": 0}

    def notify_cmd_vel_sink_ready(
        self,
        ready: bool,
        generation: Optional[int] = None,
    ) -> bool:
        """把 cmd_vel sink ready 状态跨进程写回到 Rust handle（deprecated）。

        `ready=true` 只有携带正数真实 generation 才允许发出；清除 ready 时
        wire 仍携带非零 sentinel，服务端会忽略该 generation。

        Returns:
            True 当 RPC 调用成功；False 当 stub 未连 / RPC 失败。
        """
        self._raise_if_stopping("notify_cmd_vel_sink_ready")
        if not self._stub:
            return False
        generation_value = int(generation or 0)
        if ready and generation_value <= 0:
            LOGGER.warning(
                "拒绝 SetCmdVelSinkReady(ready=true)：缺少真实 motion sink generation。"
            )
            return False
        if generation_value <= 0:
            # Rust ignores generation while clearing ready, but the deprecated
            # wire request still carries a nonzero fail-closed sentinel.
            generation_value = 1
        try:
            req = pb2.CmdVelSinkReadyRequest(
                ready=bool(ready),
                generation=generation_value,
            )
            resp = self._stub.SetCmdVelSinkReady(req, timeout=_read_deadline_sec())
            return bool(resp.success)
        except grpc.RpcError as e:
            _log_rpc_error("SetCmdVelSinkReady", e)
            return False

    # === 流式订阅 ===
    def subscribe_pose(
        self,
        callback: Callable[
            [Pose, Optional[FrameTree], Dict[str, object]],
            None,
        ],
    ) -> Optional[_CancellableStreamHandle]:
        """订阅位姿更新流。按 LocalizationProvider 协议回调 (Pose, Optional[FrameTree], metadata)。

        frame_id 为空（尚未定位）的更新会被跳过，避免下游收到非法 Pose。
        """
        if not self._stub:
            return None
        # 坐标树独立刷新，位姿只领取已完成结果，不等待远程查询。
        _FRAME_TREE_TTL_S = 0.2

        def _stream_worker(handle: _CancellableStreamHandle) -> None:
            while self._stub and not handle.stop_event.is_set():
                # 重连后的位姿不能复用上一次流调用的坐标树。
                cached_tree: Optional[FrameTree] = None
                next_tree_refresh_at = 0.0
                tree_call = None
                updates = None
                call = None
                call_failed = False
                try:
                    stub = self._stub
                    if stub is None:
                        return
                    call = stub.SubscribePose(pb2.PoseRequest())
                    if not handle.bind_call(call):
                        return
                    updates = _latest_pose_updates(call, handle.stop_event)
                    for update in updates:
                        if handle.stop_event.is_set():
                            return
                        pose = _build_pose(update.pose)
                        if pose is None:
                            continue
                        now = time.monotonic()
                        try:
                            if tree_call is None and now >= next_tree_refresh_at:
                                # 动态变换已过刷新期限，不能把旧树配给持续到达的新位姿。
                                cached_tree = None
                                tree_call = stub.GetFrameTree.future(
                                    pb2.Empty(), timeout=_pose_deadline_sec()
                                )
                            if tree_call is not None and tree_call.done():
                                response = tree_call.result()
                                cached_tree = _merge_static_edges(
                                    _build_frame_tree(response.transforms),
                                    self._static_extrinsic_edges,
                                )
                                tree_call = None
                                next_tree_refresh_at = time.monotonic() + _FRAME_TREE_TTL_S
                        except grpc.RpcError as exc:
                            cached_tree = None
                            tree_call = None
                            next_tree_refresh_at = time.monotonic() + _FRAME_TREE_TTL_S
                            _log_rpc_error("GetFrameTree(pose stream)", exc)
                        if handle.stop_event.is_set():
                            return
                        try:
                            metadata = {"sequence": update.sequence,
                                        "source_timestamp_ns": int(update.pose.timestamp_ns)}
                            try:
                                metadata["source_acquired_unix_ns"] = acquired_unix_ns(metadata["source_timestamp_ns"])
                            except SourceTimeError:
                                metadata["source_acquired_unix_ns"] = None
                            with self._map_state_lock:
                                receipt = self._planar_scan_receipts.get(metadata["source_timestamp_ns"])
                                if receipt is not None and receipt[0] == self._mapping_identity:
                                    metadata["scan_received_unix_ns"] = receipt[1]
                            callback(pose, cached_tree, metadata)
                        except Exception:
                            LOGGER.exception(
                                "位姿流消费回调失败，当前订阅继续。"
                            )
                except grpc.RpcError as exc:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.warning(
                            "SubscribePose 流断开，1 秒后重连: %s",
                            exc,
                        )
                except Exception:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.exception(
                            "SubscribePose 非 gRPC 异常，准备重连。"
                        )
                finally:
                    if updates is not None:
                        updates.close()
                    if tree_call is not None:
                        _cancel_stream_call(tree_call, context="位姿订阅坐标树")
                    if call is not None:
                        handle.clear_call(call, cancel=call_failed)
                if not self._stub or handle.stop_event.is_set():
                    break
                handle.stop_event.wait(_grpc_stream_retry_sec())

        handle = _CancellableStreamHandle(
            name="geometry_runtime_pose_stream",
            worker=_stream_worker,
            on_finish=self._discard_pose_stream_handle,
        )
        with self._stream_handle_lock:
            self._pose_stream_handles.add(handle)
        return handle.start()

    def subscribe_map_updates(
        self,
        callback: Callable[[Dict[str, Any]], None],
    ) -> Optional[_CancellableStreamHandle]:
        """订阅完整地图流；单帧解析或回调失败不会终止订阅线程。"""
        if not self._stub:
            return None

        def _stream_worker(handle: _CancellableStreamHandle) -> None:
            while self._stub and not handle.stop_event.is_set():
                call = None
                call_failed = False
                try:
                    stub = self._stub
                    if stub is None:
                        return
                    call = stub.SubscribeMapUpdates(pb2.Empty())
                    if not handle.bind_call(call):
                        return
                    for update in call:
                        if handle.stop_event.is_set():
                            return
                        try:
                            payload = self._map_update_to_payload(update)
                            if payload is None:
                                continue
                        except Exception:
                            LOGGER.exception("丢弃无效 MapUpdateNotification，订阅继续。")
                            continue
                        try:
                            ticket = self._claim_map_delivery(payload)
                            if ticket is None:
                                LOGGER.info(
                                    "丢弃跨连接或跨建图 transition 的迟到地图 payload。"
                                )
                                continue
                            try:
                                callback(payload)
                            finally:
                                self._finish_map_delivery(ticket)
                        except Exception:
                            LOGGER.exception("地图流消费回调失败，订阅继续。")
                except grpc.RpcError as exc:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.warning(
                            "SubscribeMapUpdates 流断开，1 秒后重连: %s",
                            exc,
                        )
                except Exception:
                    call_failed = True
                    if not handle.stop_event.is_set():
                        LOGGER.exception(
                            "SubscribeMapUpdates 非 gRPC 异常，准备重连。"
                        )
                finally:
                    if call is not None:
                        handle.clear_call(call, cancel=call_failed)
                if not self._stub or handle.stop_event.is_set():
                    break
                handle.stop_event.wait(_grpc_stream_retry_sec())

        handle = _CancellableStreamHandle(
            name="geometry_runtime_map_stream",
            worker=_stream_worker,
            on_finish=self._discard_map_stream_handle,
        )
        with self._stream_handle_lock:
            self._map_stream_handles.add(handle)
        return handle.start()

    def subscribe_navigation_state(self, callback: Callable[[Dict[str, Any]], None]) -> Optional[threading.Thread]:
        """订阅导航状态流，每次更新调用 callback(nav_state_dict)。"""
        if not self._stub:
            return None
        import time
        def _stream_worker():
            while self._stub:
                try:
                    for update in self._stub.SubscribeNavigationState(pb2.Empty()):
                        callback({
                            "status": update.status,
                            "message": update.message,
                            "goal_id": update.goal_id,
                            "distance_remaining": update.distance_remaining,
                        })
                except grpc.RpcError:
                    pass
                if not self._stub:
                    break
                time.sleep(1.0)
        t = threading.Thread(target=_stream_worker, name="geometry_runtime_nav_stream", daemon=True)
        t.start()
        return t
