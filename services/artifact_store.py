"""临时制品文件存储；记忆证据由记忆库自己的 photos 目录保存。"""
from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timedelta
import os
from pathlib import Path
import re
import threading
from typing import Iterable, Optional

from contracts.artifacts import ArtifactCleanupResult, ArtifactKind, ArtifactRef, ArtifactRetentionPolicy, ArtifactStorageSummary
from contracts.base import utc_now
from contracts.naming import build_artifact_id, validate_artifact_id
from core.errors import ArtifactNotFoundError, GatewayError


_ARTIFACT_FILENAME = re.compile(r"^(art_[a-z0-9_]+_[0-9]{8}T[0-9]{6}Z_[a-f0-9]{8})")


@dataclass(frozen=True)
class _ArtifactEntry:
    ref: ArtifactRef
    meta_path: Path
    file_path: Optional[Path]


class LocalArtifactStore:
    def __init__(self, base_dir: str, public_base_url: str) -> None:
        self._base_dir = Path(base_dir)
        self._base_dir.mkdir(parents=True, exist_ok=True)
        self._public_base_url = public_base_url.rstrip("/")
        self._lock = threading.RLock()
        self._status_lock = threading.Lock()
        self._maintenance_status = {"status": "not_run"}

    @property
    def base_dir(self) -> Path:
        return self._base_dir

    def save_bytes(self, *, kind: ArtifactKind, mime_type: str, data: bytes,
                   metadata: Optional[dict] = None, extension: Optional[str] = None) -> ArtifactRef:
        artifact_id = build_artifact_id(kind.value)
        suffix = extension or self._guess_extension(mime_type)
        directory = self._base_dir / kind.value
        file_path, meta_path = directory / (artifact_id + suffix), directory / (artifact_id + ".meta.json")
        temporary_data, temporary_meta = directory / ("." + artifact_id + ".data.tmp"), directory / ("." + artifact_id + ".meta.tmp")
        ref = ArtifactRef(artifact_id=artifact_id, kind=kind, mime_type=mime_type, size_bytes=len(data),
                          uri=f"{self._public_base_url}/artifacts/{artifact_id}", metadata=dict(metadata or {}))
        with self._lock:
            directory.mkdir(parents=True, exist_ok=True)
            if file_path.exists() or meta_path.exists():
                raise GatewayError("制品标识已存在。", error_code="artifact_id_collision", http_status=409)
            published = False
            try:
                temporary_data.write_bytes(data)
                temporary_meta.write_text(ref.model_dump_json(indent=2), encoding="utf-8")
                os.replace(temporary_data, file_path)
                published = True
                os.replace(temporary_meta, meta_path)
            except BaseException:
                for path in (temporary_data, temporary_meta) + ((file_path,) if published else ()):
                    path.unlink(missing_ok=True)
                raise
        return ref

    def _find_meta_path(self, artifact_id: str) -> Path:
        validate_artifact_id(artifact_id)
        path = next(self._base_dir.glob(f"**/{artifact_id}.meta.json"), None)
        if path is None:
            raise ArtifactNotFoundError(artifact_id)
        return path

    def _find_file_path(self, artifact_id: str) -> Path:
        for path in self._base_dir.glob(f"**/{artifact_id}*"):
            if path.is_file() and not path.name.endswith(".meta.json"):
                return path
        raise ArtifactNotFoundError(artifact_id)

    @staticmethod
    def _parse_metadata(path: Path) -> ArtifactRef:
        ref = ArtifactRef.model_validate_json(path.read_text(encoding="utf-8"))
        if path.name != ref.artifact_id + ".meta.json":
            raise GatewayError("制品元数据标识与文件名不一致。", error_code="artifact_inventory_invalid", http_status=409)
        return ref

    def get_ref(self, artifact_id: str) -> ArtifactRef:
        with self._lock:
            return self._parse_metadata(self._find_meta_path(artifact_id))

    def get_file_path(self, artifact_id: str) -> Path:
        validate_artifact_id(artifact_id)
        with self._lock:
            return self._find_file_path(artifact_id)

    @contextmanager
    def read_artifact(self, artifact_id: str):
        # 在同一文件锁内读取侧车并打开文件，打开后的流按普通文件资源关闭。
        with self._lock:
            ref = self._parse_metadata(self._find_meta_path(artifact_id))
            stream = self._find_file_path(artifact_id).open("rb")
        with stream:
            yield ref, stream

    def read_bytes(self, artifact_id: str, *, max_bytes: Optional[int] = None) -> bytes:
        with self.read_artifact(artifact_id) as (_ref, stream):
            data = stream.read() if max_bytes is None else stream.read(max_bytes + 1)
        if max_bytes is not None and len(data) > max_bytes:
            raise GatewayError("制品超过读取大小上限。", error_code="artifact_too_large", http_status=413)
        return data

    def _scan_entries(self) -> tuple[_ArtifactEntry, ...]:
        entries, seen = [], set()
        def scan_error(error):
            raise error
        for directory, dirs, files in os.walk(self._base_dir, onerror=scan_error):
            dirs.sort(); files.sort()
            parent, payloads = Path(directory), {}
            for name in files:
                match = _ARTIFACT_FILENAME.match(name)
                if match is not None and not name.endswith(".meta.json"):
                    artifact_id = match.group(1)
                    if artifact_id in payloads:
                        raise GatewayError("同一制品存在多个数据文件。", error_code="artifact_inventory_invalid", http_status=409)
                    payloads[artifact_id] = parent / name
            for name in files:
                if name.endswith(".meta.json"):
                    path = parent / name
                    ref = self._parse_metadata(path)
                    if ref.artifact_id in seen:
                        raise GatewayError("存在重复制品标识。", error_code="artifact_inventory_invalid", http_status=409)
                    seen.add(ref.artifact_id)
                    entries.append(_ArtifactEntry(ref, path, payloads.get(ref.artifact_id)))
        return tuple(sorted(entries, key=lambda item: (item.ref.timestamp, str(item.meta_path))))

    def list_refs(self, *, kind: Optional[ArtifactKind] = None) -> tuple[ArtifactRef, ...]:
        with self._lock:
            return tuple(item.ref for item in self._scan_entries() if kind is None or item.ref.kind == kind)

    @staticmethod
    def _summary(refs: Iterable[ArtifactRef]) -> ArtifactStorageSummary:
        items = sorted(refs, key=lambda ref: ref.timestamp)
        by_kind = {}
        for ref in items:
            by_kind[ref.kind.value] = by_kind.get(ref.kind.value, 0) + 1
        return ArtifactStorageSummary(artifact_count=len(items), total_size_bytes=sum(ref.size_bytes or 0 for ref in items),
            by_kind=by_kind, oldest_artifact_id=items[0].artifact_id if items else None,
            newest_artifact_id=items[-1].artifact_id if items else None)

    def build_summary(self, refs: Optional[Iterable[ArtifactRef]] = None) -> ArtifactStorageSummary:
        with self._lock:
            return self._summary(refs if refs is not None else (item.ref for item in self._scan_entries()))

    @staticmethod
    def _delete_entry(entry: _ArtifactEntry) -> ArtifactRef:
        if entry.file_path is None:
            raise ArtifactNotFoundError(entry.ref.artifact_id)
        entry.file_path.unlink()
        entry.meta_path.unlink()
        return entry.ref

    def delete_artifact(self, artifact_id: str) -> ArtifactRef:
        with self._lock:
            meta_path = self._find_meta_path(artifact_id)
            return self._delete_entry(_ArtifactEntry(self._parse_metadata(meta_path), meta_path, self._find_file_path(artifact_id)))

    def cleanup(self, policy: ArtifactRetentionPolicy, *, as_of: Optional[datetime] = None) -> ArtifactCleanupResult:
        with self._lock:
            try:
                entries = self._scan_entries()
                expire_before = (as_of or utc_now()) - timedelta(days=policy.retention_days) if policy.retention_days else None
                retained_count, retained_bytes = len(entries), sum(item.ref.size_bytes or 0 for item in entries)
                removed = []
                for entry in entries:
                    expired = expire_before is not None and entry.ref.timestamp < expire_before
                    over_count = policy.max_count is not None and retained_count > policy.max_count
                    over_bytes = policy.max_total_bytes is not None and retained_bytes > policy.max_total_bytes
                    if len(removed) >= policy.cleanup_batch_size:
                        break
                    if expired or over_count or over_bytes:
                        removed.append(self._delete_entry(entry))
                        retained_count -= 1
                        retained_bytes -= entry.ref.size_bytes or 0
                removed_ids = {ref.artifact_id for ref in removed}
                retained = [item.ref for item in entries if item.ref.artifact_id not in removed_ids]
                result = ArtifactCleanupResult(removed_count=len(removed), freed_bytes=sum(ref.size_bytes or 0 for ref in removed),
                    removed_artifact_ids=[ref.artifact_id for ref in removed], summary=self._summary(retained))
                pending_count = max(0, retained_count - policy.max_count) if policy.max_count is not None else 0
                pending_bytes = max(0, retained_bytes - policy.max_total_bytes) if policy.max_total_bytes is not None else 0
                pending_expired = sum(ref.timestamp < expire_before for ref in retained) if expire_before else 0
                status = {"status": "deferred" if pending_count or pending_bytes or pending_expired else "completed",
                          "pending_count": pending_count, "pending_bytes": pending_bytes, "pending_expired": pending_expired,
                          "result": result.model_dump(mode="json")}
                with self._status_lock:
                    self._maintenance_status = status
                return result
            except Exception as error:
                with self._status_lock:
                    self._maintenance_status = {"status": "failed", "error": str(error)}
                raise

    def get_maintenance_status(self) -> dict:
        with self._status_lock:
            return deepcopy(self._maintenance_status)

    def clear_all(self) -> ArtifactCleanupResult:
        with self._lock:
            removed = [self._delete_entry(item) for item in self._scan_entries()]
            return ArtifactCleanupResult(removed_count=len(removed), freed_bytes=sum(ref.size_bytes or 0 for ref in removed),
                removed_artifact_ids=[ref.artifact_id for ref in removed], summary=self._summary(()))

    @staticmethod
    def _guess_extension(mime_type: str) -> str:
        return {"image/jpeg": ".jpg", "image/png": ".png", "application/json": ".json", "audio/wav": ".wav"}.get(mime_type.lower().strip(), ".bin")
