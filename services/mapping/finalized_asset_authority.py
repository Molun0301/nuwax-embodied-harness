from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Callable, Mapping, Optional

from contracts.map_workspace import MapGeoreferenceV1
from services.mapping.asset_store import (
    MapAssetBundleRepository,
    MapAssetBundleValidationError,
)
from services.mapping.catalog import MapCatalogRepository

_COMPANION_DOMAIN = b"nuwax.mapping.finalized-asset-authority.v1\0"


class FinalizedAssetAuthorityError(RuntimeError):
    """finalized 资产权威无法形成同一份可信快照。"""

    def __init__(self, code: str, message: str) -> None:
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class SealedFinalizedAssetIdentity:
    """由 Batch03 sealed snapshot 投影的服务端内部身份。"""

    map_name: str
    map_frame_id: str
    asset_manifest_sha256: str
    asset_bundle_sha256: str
    activation_request_id: str
    activation_request_identity_sha256: str
    activation_identity_sha256: str
    target_receipt_sha256: str
    runtime_generation: int
    lifecycle_epoch: int
    session_id: str
    session_generation: int
    backend_kind: str
    backend_generation: int


@dataclass(frozen=True)
class CommittedMapAuthority:
    """由 Geometry committed-map handle 投影的只读身份。"""

    session_id: str
    session_generation: int
    revision: int
    payload_sha256: str
    frame_id: str


@dataclass(frozen=True)
class FinalizedMapRuntimeAuthority:
    """一次 Geometry authority provider 原子读取的组合结果。"""

    sealed: SealedFinalizedAssetIdentity
    committed_map: CommittedMapAuthority


@dataclass(frozen=True)
class ResolvedFinalizedAssetAuthorityV1:
    """S5 内部消费的 finalized-asset authority companion。"""

    map_name: str
    map_frame_id: str
    asset_manifest_sha256: str
    asset_bundle_sha256: str
    georeference_bytes: bytes
    georeference_sha256: str
    georeference: MapGeoreferenceV1
    activation_request_id: str
    activation_request_identity_sha256: str
    activation_identity_sha256: str
    target_receipt_sha256: str
    runtime_generation: int
    lifecycle_epoch: int
    session_id: str
    session_generation: int
    backend_kind: str
    backend_generation: int
    committed_revision: int
    committed_payload_sha256: str
    owner_epoch: int
    companion_sha256: str


class FinalizedAssetAuthorityResolver:
    """只读 join finalized repository、activation receipt 与 Geometry authority。"""

    def __init__(
        self,
        *,
        asset_repository: MapAssetBundleRepository,
        catalog_repository: MapCatalogRepository,
        runtime_authority_provider: Callable[[], Optional[FinalizedMapRuntimeAuthority]],
        owner_epoch_provider: Callable[[], int],
    ) -> None:
        self._asset_repository = asset_repository
        self._catalog_repository = catalog_repository
        self._runtime_authority_provider = runtime_authority_provider
        self._owner_epoch_provider = owner_epoch_provider

    def resolve(self) -> ResolvedFinalizedAssetAuthorityV1:
        """fresh 读取全部服务端 authority；任一缺失、变化或冲突均 fail closed。"""

        initial = self._runtime_authority_provider()
        if initial is None:
            self._fail("runtime_authority_unavailable", "Geometry 当前没有 sealed 地图权威。")
        assert initial is not None
        self._validate_runtime_authority(initial)
        sealed = initial.sealed
        committed = initial.committed_map

        snapshot = self._asset_repository.get_validated_map_asset_bundle_snapshot(
            sealed.map_name,
            expected_manifest_sha256=sealed.asset_manifest_sha256,
        )
        if snapshot is None:
            self._fail("finalized_asset_missing", "finalized 地图 current 资产不存在。")
        assert snapshot is not None
        bundle = snapshot.bundle
        if bundle.map_name != sealed.map_name or bundle.frame_id != sealed.map_frame_id:
            self._fail("repository_identity_mismatch", "finalized 资产 map/frame 与 sealed 权威不一致。")
        if bundle.manifest_sha256 != sealed.asset_manifest_sha256:
            self._fail("manifest_mismatch", "finalized 资产清单摘要与 sealed 权威不一致。")
        if bundle.georeference is None:
            self._fail(
                "map_georeference_required",
                "当前地图没有地理锚点，无法将经纬度转换为地图目标；本地地图坐标仍可使用。",
            )
        assert bundle.georeference is not None
        georeference = bundle.georeference.contract
        georeference_sha256 = hashlib.sha256(snapshot.georeference_bytes).hexdigest()
        if (
            bundle.georeference.raw_sha256 != georeference_sha256
            or georeference.map_name != sealed.map_name
            or georeference.map_frame_id != sealed.map_frame_id
        ):
            self._fail("georeference_identity_mismatch", "地理参考原文、map 或 frame 身份不一致。")

        receipt = self._catalog_repository.get_last_successful_map_activation_receipt()
        if receipt is None or receipt.get("trusted_identity") is not True:
            self._fail("trusted_receipt_unavailable", "缺少可信的 sealed activation receipt。")
        assert receipt is not None
        self._validate_receipt(receipt, sealed)

        owner_epoch = self._require_positive_int(
            self._owner_epoch_provider(),
            "owner_epoch",
        )
        final = self._runtime_authority_provider()
        final_owner_epoch = self._require_positive_int(
            self._owner_epoch_provider(),
            "owner_epoch",
        )
        if final is None or final != initial:
            self._fail("runtime_authority_changed", "解析期间 Geometry 地图权威已变化。")
        if final_owner_epoch != owner_epoch:
            self._fail("owner_epoch_changed", "解析期间 S5 owner epoch 已变化。")
        try:
            self._asset_repository.validate_map_asset_bundle_snapshot_identity(
                sealed.map_name,
                snapshot,
            )
        except MapAssetBundleValidationError as exc:
            raise FinalizedAssetAuthorityError(
                "repository_snapshot_changed",
                "解析期间 finalized repository snapshot 已变化。",
            ) from exc

        digest_fields = {
            "activation_identity_sha256": sealed.activation_identity_sha256,
            "activation_request_id": sealed.activation_request_id,
            "activation_request_identity_sha256": sealed.activation_request_identity_sha256,
            "asset_bundle_sha256": sealed.asset_bundle_sha256,
            "asset_manifest_sha256": sealed.asset_manifest_sha256,
            "backend_generation": sealed.backend_generation,
            "backend_kind": sealed.backend_kind,
            "committed_payload_sha256": committed.payload_sha256,
            "committed_revision": committed.revision,
            "georeference_sha256": georeference_sha256,
            "lifecycle_epoch": sealed.lifecycle_epoch,
            "map_frame_id": sealed.map_frame_id,
            "map_name": sealed.map_name,
            "owner_epoch": owner_epoch,
            "runtime_generation": sealed.runtime_generation,
            "session_generation": sealed.session_generation,
            "session_id": sealed.session_id,
            "target_receipt_sha256": sealed.target_receipt_sha256,
        }
        companion_sha256 = hashlib.sha256(
            _COMPANION_DOMAIN
            + json.dumps(
                digest_fields,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        return ResolvedFinalizedAssetAuthorityV1(
            map_name=sealed.map_name,
            map_frame_id=sealed.map_frame_id,
            asset_manifest_sha256=sealed.asset_manifest_sha256,
            asset_bundle_sha256=sealed.asset_bundle_sha256,
            georeference_bytes=snapshot.georeference_bytes,
            georeference_sha256=georeference_sha256,
            georeference=georeference,
            activation_request_id=sealed.activation_request_id,
            activation_request_identity_sha256=sealed.activation_request_identity_sha256,
            activation_identity_sha256=sealed.activation_identity_sha256,
            target_receipt_sha256=sealed.target_receipt_sha256,
            runtime_generation=sealed.runtime_generation,
            lifecycle_epoch=sealed.lifecycle_epoch,
            session_id=sealed.session_id,
            session_generation=sealed.session_generation,
            backend_kind=sealed.backend_kind,
            backend_generation=sealed.backend_generation,
            committed_revision=committed.revision,
            committed_payload_sha256=committed.payload_sha256,
            owner_epoch=owner_epoch,
            companion_sha256=companion_sha256,
        )

    def _validate_runtime_authority(self, authority: FinalizedMapRuntimeAuthority) -> None:
        sealed = authority.sealed
        committed = authority.committed_map
        required_text = {
            "map_name": sealed.map_name,
            "map_frame_id": sealed.map_frame_id,
            "activation_request_id": sealed.activation_request_id,
            "session_id": sealed.session_id,
            "backend_kind": sealed.backend_kind,
        }
        for field_name, value in required_text.items():
            if not isinstance(value, str) or not value or value.strip() != value:
                self._fail("invalid_runtime_authority", "%s 非法。" % field_name)
        for field_name in (
            "asset_manifest_sha256",
            "asset_bundle_sha256",
            "activation_request_identity_sha256",
            "activation_identity_sha256",
            "target_receipt_sha256",
        ):
            self._require_sha256(getattr(sealed, field_name), field_name)
        for field_name in (
            "runtime_generation",
            "lifecycle_epoch",
            "session_generation",
            "backend_generation",
        ):
            self._require_positive_int(getattr(sealed, field_name), field_name)
        committed_required_text = {
            "committed session_id": committed.session_id,
            "committed frame_id": committed.frame_id,
        }
        for field_name, value in committed_required_text.items():
            if not isinstance(value, str) or not value or value.strip() != value:
                self._fail("invalid_runtime_authority", "%s 非法。" % field_name)
        self._require_positive_int(committed.session_generation, "committed session_generation")
        self._require_positive_int(committed.revision, "committed revision")
        self._require_sha256(committed.payload_sha256, "committed payload_sha256")
        if (
            committed.session_id != sealed.session_id
            or committed.session_generation != sealed.session_generation
            or committed.frame_id != sealed.map_frame_id
        ):
            self._fail("committed_map_mismatch", "committed map 与 sealed session/frame 不一致。")

    def _validate_receipt(
        self,
        receipt: Mapping[str, object],
        sealed: SealedFinalizedAssetIdentity,
    ) -> None:
        expected = {
            "target_map_name": sealed.map_name,
            "target_manifest_sha256": sealed.asset_manifest_sha256,
            "asset_bundle_sha256": sealed.asset_bundle_sha256,
            "activation_request_id": sealed.activation_request_id,
            "activation_request_identity_sha256": sealed.activation_request_identity_sha256,
            "activation_identity_sha256": sealed.activation_identity_sha256,
            "target_receipt_sha256": sealed.target_receipt_sha256,
            "runtime_generation": sealed.runtime_generation,
            "session_id": sealed.session_id,
            "session_generation": sealed.session_generation,
            "mapping_backend_kind": sealed.backend_kind,
            "backend_generation": sealed.backend_generation,
        }
        for field_name, expected_value in expected.items():
            if receipt.get(field_name) != expected_value:
                self._fail(
                    "activation_receipt_mismatch",
                    "activation receipt 字段 %s 与 sealed 权威不一致。" % field_name,
                )

    @staticmethod
    def _require_positive_int(value: object, field_name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise FinalizedAssetAuthorityError(
                "invalid_runtime_authority",
                "%s 必须为正整数。" % field_name,
            )
        return value

    @staticmethod
    def _require_sha256(value: object, field_name: str) -> str:
        if (
            not isinstance(value, str)
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
        ):
            raise FinalizedAssetAuthorityError(
                "invalid_runtime_authority",
                "%s 必须为小写 SHA-256。" % field_name,
            )
        return value

    @staticmethod
    def _fail(code: str, message: str) -> None:
        raise FinalizedAssetAuthorityError(code, message)
