from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Mapping, Optional

from contracts.map_activation import AssignedActivationReceipt
from contracts.map_workspace import MapGeoreferenceV1
from services.mapping.asset_store import (
    MapAssetBundleRepository,
)


class FinalizedAssetAuthorityError(RuntimeError):
    """The finalized asset and native assigned authority do not form one snapshot."""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class FinalizedMapRuntimeAuthority:
    """Bounded metadata captured by one atomic Geometry authority read."""

    activation_receipt: AssignedActivationReceipt
    owner_epoch: int
    committed_revision: int
    committed_boot_nonce: str
    committed_sequence: int
    committed_source_timestamp_ns: int

    @classmethod
    def admit(cls, value: Mapping[str, object]) -> FinalizedMapRuntimeAuthority:
        """消费提供器已解码的原生权威，不重复核对内部字段。"""
        commit = value["committed_occupancy_commit_id"]
        return cls(value["activation_receipt_v2"], value["owner_epoch"],
                   value["committed_occupancy_revision"], commit["boot_nonce"], commit["sequence"],
                   value["committed_occupancy_source_timestamp_ns"])

    def to_dict(self) -> Dict[str, object]:
        receipt = self.activation_receipt.to_dict()
        activation = receipt["activation"]
        source_key = "source_graph_member" if activation["backend_kind"] == "slam_toolbox" else "source_cloud_member"
        return {
            "schema_version": 3,
            "activation_receipt_v2": self.activation_receipt,
            "map_name": self.activation_receipt.map_name,
            "map_frame_id": receipt["target"][source_key]["frame_id"],
            "runtime_generation": activation["runtime_generation"],
            "lifecycle_epoch": activation["lifecycle_epoch"],
            "mapping_session_id": self.activation_receipt.session_id,
            "mapping_session_generation": activation["session_generation"],
            "backend_kind": activation["backend_kind"],
            "backend_generation": activation["backend_generation"],
            "committed_occupancy_revision": self.committed_revision,
            "committed_occupancy_commit_id": {"boot_nonce": self.committed_boot_nonce, "sequence": self.committed_sequence},
            "committed_occupancy_source_timestamp_ns": self.committed_source_timestamp_ns,
            "owner_epoch": self.owner_epoch,
        }


@dataclass(frozen=True)
class ResolvedFinalizedAssetAuthority:
    """Original validated georeference bytes and their existing assigned owner."""

    authority: FinalizedMapRuntimeAuthority
    georeference_bytes: bytes
    georeference: MapGeoreferenceV1


class FinalizedAssetAuthorityResolver:
    """Join the stable asset snapshot, durable activation and native authority."""

    def __init__(
        self,
        *,
        asset_repository: MapAssetBundleRepository,
        runtime_authority_provider: Callable[[], Optional[Mapping[str, object]]],
    ) -> None:
        self._asset_repository = asset_repository
        self._runtime_authority_provider = runtime_authority_provider

    def _read_authority(self) -> FinalizedMapRuntimeAuthority:
        value = self._runtime_authority_provider()
        if value is None:
            self._fail("runtime_authority_unavailable", "Geometry 当前没有 sealed 地图权威。")
        return FinalizedMapRuntimeAuthority.admit(value)

    def resolve(self) -> ResolvedFinalizedAssetAuthority:
        initial = self._read_authority()
        receipt = initial.activation_receipt
        snapshot = self._asset_repository.get_validated_map_asset_bundle_snapshot(
            receipt.map_name, expected_asset_identity=receipt.asset_identity,
        )
        if snapshot is None:
            self._fail("finalized_asset_missing", "finalized 地图 current 资产不存在。")
        bundle = snapshot.bundle
        if bundle.georeference is None:
            self._fail("map_georeference_required",
                "当前地图没有地理锚点，无法将经纬度转换为地图目标；本地地图坐标仍可使用。")
        georeference = bundle.georeference.contract
        return ResolvedFinalizedAssetAuthority(initial, snapshot.georeference_bytes, georeference)

    @staticmethod
    def _fail(code: str, message: str) -> None:
        raise FinalizedAssetAuthorityError(code, message)
