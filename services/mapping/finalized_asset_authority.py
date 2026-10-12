from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Mapping, Optional

from contracts.map_activation import AssignedActivationReceipt, _commit, _integer
from contracts.map_workspace import MapGeoreferenceV1
from services.geometry.mapping_backend_checkpoint_store import _identity_document
from services.mapping.asset_store import (
    MapAssetBundleRepository,
    MapAssetBundleValidationError,
)
from services.mapping.catalog import MapCatalogRepository


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
        try:
            if not isinstance(value, Mapping) or type(value.get("schema_version")) is not int or value["schema_version"] != 3:
                raise ValueError("mapping-cloud authority requires schema 3")
            receipt = AssignedActivationReceipt.admit(value.get("activation_receipt_v2"))
            commit = _commit(value.get("committed_occupancy_commit_id"))
            authority = cls(receipt, _integer(value.get("owner_epoch"), "owner epoch"),
                _integer(value.get("committed_occupancy_revision"), "committed occupancy revision"),
                commit["boot_nonce"], commit["sequence"],
                _integer(value.get("committed_occupancy_source_timestamp_ns"), "committed occupancy source time"))
            expected = authority.to_dict()
            for name in ("runtime_generation", "lifecycle_epoch", "mapping_session_generation", "backend_generation"):
                _integer(value.get(name), name)
            for name in ("map_name", "map_frame_id", "runtime_generation", "lifecycle_epoch",
                         "mapping_session_id", "mapping_session_generation", "backend_kind", "backend_generation"):
                if value.get(name) != expected[name]:
                    raise ValueError("mapping-cloud %s differs from assigned receipt" % name)
            # Current committed occupancy can advance after target installation.
            # Its tuple is fenced independently of the original source and
            # installed target receipts; none may substitute for another.
            return authority
        except (KeyError, TypeError, ValueError) as exc:
            raise FinalizedAssetAuthorityError("invalid_runtime_authority", str(exc)) from exc

    def to_dict(self) -> Dict[str, object]:
        receipt = self.activation_receipt.to_dict()
        activation = receipt["activation"]
        source_key = "source_graph_member" if activation["backend_kind"] == "slam_toolbox" else "source_cloud_member"
        return {
            "schema_version": 3,
            "activation_receipt_v2": receipt,
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
        catalog_repository: MapCatalogRepository,
        runtime_authority_provider: Callable[[], Optional[Mapping[str, object]]],
    ) -> None:
        self._asset_repository = asset_repository
        self._catalog_repository = catalog_repository
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
        payload = receipt.to_dict()
        source_key = "source_graph_member" if payload["activation"]["backend_kind"] == "slam_toolbox" else "source_cloud_member"
        source_member = payload["target"][source_key]
        frame_id = source_member["frame_id"]
        if (bundle.identity_schema_version != 2 or bundle.asset_identity != receipt.asset_identity
                or bundle.map_name != receipt.map_name or bundle.frame_id != frame_id):
            self._fail("repository_identity_mismatch", "finalized 资产 assigned/map/frame 与 sealed 权威不一致。")
        source = snapshot.assigned_source_member
        if source is None or _identity_document(source) != source_member:
            self._fail("repository_source_mismatch", "finalized 资产原 checkpoint 成员与 sealed 权威不一致。")
        if bundle.georeference is None:
            self._fail("map_georeference_required",
                "当前地图没有地理锚点，无法将经纬度转换为地图目标；本地地图坐标仍可使用。")
        georeference = bundle.georeference.contract
        if georeference.map_name != receipt.map_name or georeference.map_frame_id != frame_id:
            self._fail("georeference_identity_mismatch", "地理参考 map 或 frame 身份不一致。")

        catalog = self._catalog_repository.get_last_successful_map_activation_receipt()
        if (not isinstance(catalog, Mapping) or catalog.get("trusted_identity") is not True
                or type(catalog.get("schema_version")) is not int or catalog["schema_version"] != 3):
            self._fail("trusted_receipt_unavailable", "缺少可信的 assigned sealed activation receipt。")
        try:
            durable_receipt = AssignedActivationReceipt.admit(catalog.get("activation_receipt_v2"))
        except (TypeError, ValueError) as exc:
            raise FinalizedAssetAuthorityError("activation_receipt_mismatch", "activation receipt 非法。") from exc
        if not receipt.exact_replay_of(durable_receipt):
            self._fail("activation_receipt_mismatch", "activation receipt 与 sealed 权威不一致。")

        # owner_epoch belongs to each complete native snapshot. A separate RPC
        # for it would mix observations instead of making this join more atomic.
        final = self._read_authority()
        if final.owner_epoch != initial.owner_epoch:
            self._fail("owner_epoch_changed", "解析期间 S5 owner epoch 已变化。")
        if final != initial:
            self._fail("runtime_authority_changed", "解析期间 Geometry 地图权威已变化。")
        try:
            self._asset_repository.validate_map_asset_bundle_snapshot_identity(receipt.map_name, snapshot)
        except MapAssetBundleValidationError as exc:
            raise FinalizedAssetAuthorityError("repository_snapshot_changed",
                "解析期间 finalized repository snapshot 已变化。") from exc
        return ResolvedFinalizedAssetAuthority(initial, snapshot.georeference_bytes, georeference)

    @staticmethod
    def _fail(code: str, message: str) -> None:
        raise FinalizedAssetAuthorityError(code, message)
