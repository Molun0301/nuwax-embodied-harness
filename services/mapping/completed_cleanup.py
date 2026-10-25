"""Post-mapping cleanup: compute a private preview, then save an explicit new map.

The original finalized bundle is only read. The native worker uses complete raw
scans and the committed final poses when available; older maps remain supported
for floating-geometry cleanup without claiming full human-replay coverage.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import threading
import uuid

from core.errors import GatewayError


def _error(message):
    return GatewayError(message, error_code="map_cleanup_failed", http_status=409)


def _sha(path):
    with Path(path).open("rb") as source:
        digest = hashlib.sha256()
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
        return digest.hexdigest()


def copy_raw_scan_history(checkpoint, destination):
    """Capture only complete frames up to this exact saved source timestamp."""
    root = Path(os.environ.get("NUWAX_RAW_SCAN_DIRECTORY", "runtime_data/scan_logs"))
    session = {"session_id": checkpoint.source_session_id, "generation": checkpoint.source_session_generation}
    stem = hashlib.sha256(f"{session['session_id']}:{session['generation']}".encode()).hexdigest()
    source = root / (stem + ".scan")
    if not source.is_file():
        return {"complete": False, "note": "原始扫描日志缺失，不能完成全量人体离线重判"}
    timestamp = checkpoint.cloud_receipt["source_timestamp_ns"]
    target = Path(destination) / "raw-scans.scan"
    frames = 0
    last = 0
    def record(stream):
        size = stream.read(4)
        if not size:
            return None
        if len(size) != 4:
            raise ValueError("原始扫描记录头不完整")
        length = struct.unpack("<I", size)[0]
        if length > 65536:
            raise ValueError("原始扫描记录头无效")
        data = stream.read(length)
        if len(data) != length:
            raise ValueError("原始扫描记录头不完整")
        return size + data, json.loads(data)
    with source.open("rb") as inp, target.open("xb") as out:
        magic = inp.read(8)
        if magic != b"NEHSCAN1":
            raise ValueError("原始扫描日志格式无效")
        header, identity = record(inp)
        if identity != session:
            raise ValueError("原始扫描日志不属于保存会话")
        out.write(magic + header)
        while last < timestamp:
            entry = record(inp)
            if entry is None:
                break
            header, frame = entry
            if frame["source_timestamp_ns"] > timestamp:
                break
            count = frame["point_count"]
            if type(count) is not int or not 0 <= count <= 2_000_000:
                raise ValueError("原始扫描点数无效")
            remaining = count * 25
            out.write(header)
            while remaining:
                data = inp.read(min(remaining, 1024 * 1024))
                if not data:
                    raise ValueError("保存快照以内的原始扫描不完整")
                out.write(data)
                remaining -= len(data)
            frames += 1
            last = frame["source_timestamp_ns"]
        out.flush()
        os.fsync(out.fileno())
    result = {"path": target.name, "frames": frames, "last_timestamp_ns": last, "complete": last == timestamp}
    revisions = []
    for path in root.glob(stem + "-poses-*.json"):
        try:
            revision = int(path.stem.rsplit("-", 1)[-1])
        except ValueError:
            continue
        if revision <= checkpoint.cloud_receipt["cloud_revision"]:
            revisions.append((revision, path))
    if revisions:
        _, source = max(revisions)
        poses = json.loads(source.read_text())
        if poses["session"] != session:
            raise ValueError("最终位姿不属于保存会话")
        shutil.copyfile(source, Path(destination) / "final-poses.json")
        result["final_poses"] = "final-poses.json"
    return result


class CompletedMapCleanup:
    def __init__(self, owner):
        self.owner = owner
        self.root = owner._asset_repository._root_dir / ".cleanup-previews"
        self.root.mkdir(parents=True, exist_ok=True)
        self.jobs = {}
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="map-cleanup")

    def start(self, *, map_name, strength_percent=50):
        if type(strength_percent) is not int or not 0 <= strength_percent <= 100:
            raise _error("清理力度必须是 0–100 的整数百分比")
        source = self.owner.get_map_asset_bundle(map_name=map_name)
        if source is None or source.geometry_asset is None:
            raise _error("请先完成并保存地图，再预览清理")
        context = self.owner._geometry_runtime.get_mapping_session_context()
        if isinstance(context, dict) and context.get("active") and context.get("map_name") == map_name:
            raise _error("请先停止建图。浮空清理只处理已经完成的保存地图")
        reference = source.geometry_asset.metadata.get("mapping_backend_checkpoint") or {}
        if reference.get("backend_kind") not in {"voxel", "raytracing"}:
            raise _error("当前地图不包含三维点云")
        from services.geometry.mapping_backend_checkpoint_store import MappingBackendCheckpointStore
        source_root = self.owner._asset_repository.current_directory(map_name)
        stored = MappingBackendCheckpointStore(source_root).load(reference["backend_kind"],
            schema_version=reference["schema_version"],
            **({"expected_checkpoint_id": reference["checkpoint_id"]} if reference["schema_version"] == 2 else {}))
        if stored.checkpoint.fine_archive is None or stored.checkpoint.dense_archive is None:
            raise _error("地图缺少细点云或累计扫描资产，不能给出可靠的清理预览")
        job_id = uuid.uuid4().hex
        folder = self.root / job_id
        folder.mkdir()
        for role, payload in (("fine", stored.checkpoint.fine_archive), ("dense", stored.checkpoint.dense_archive),
                              ("backend", stored.checkpoint.payload), ("occupancy", stored.checkpoint.canonical_occupancy_data)):
            if payload is None:
                raise _error("旧地图缺少完整后端资产，请先完成地图导入")
            with (folder / (role + ".source")).open("xb") as target:
                for offset in range(0, len(payload), 1024 * 1024):
                    target.write(payload[offset:offset + 1024 * 1024])
        history = source.metadata.get("raw_scan_history") or {}
        def saved_file(key):
            relative = history.get(key)
            if not relative:
                return None
            path = (source_root / relative).resolve()
            try:
                path.relative_to(source_root.resolve())
            except ValueError:
                raise _error("保存的原始扫描资产路径无效")
            if not path.is_file():
                raise _error("保存的原始扫描资产路径无效")
            return str(path)
        request = {"fine": str(folder / "fine.source"), "dense": str(folder / "dense.source"),
            "backend": str(folder / "backend.source"), "occupancy": str(folder / "occupancy.source"),
            "occupancy_descriptor": dict(stored.occupancy_asset), "output": str(folder / "result"),
            "strength_percent": strength_percent, "source_timestamp_ns": stored.checkpoint.cloud_receipt["source_timestamp_ns"],
            "scan_log": saved_file("path"), "final_poses": saved_file("final_poses"), "cancel_file": str(folder / "cancel")}
        (folder / "request.json").write_text(json.dumps(request), encoding="utf-8")
        job = dict(id=job_id, state="queued", map_name=map_name, strength_percent=strength_percent,
                   source=source, source_root=source_root, stored=stored, folder=folder, report=None, error=None, result_map=None)
        with self.lock:
            self.jobs[job_id] = job
        self.executor.submit(self._compute, job)
        return self.status(job_id)

    def _compute(self, job):
        try:
            with self.lock:
                if job["state"] == "cancelled":
                    return
                job["state"] = "running"
            runtime = self.owner._geometry_runtime
            binary = Path(str(getattr(runtime.config, "runtime_binary", "") or runtime.config.external.binary_path))
            if not binary.is_file():
                raise _error("清理运行程序尚未构建，请更新几何运行程序")
            with (job["folder"] / "worker.log").open("wb") as log:
                process = subprocess.run([str(binary), "--cleanup-completed-cloud", str(job["folder"] / "request.json")],
                                         stdout=log, stderr=subprocess.STDOUT, check=False)
            if (job["folder"] / "cancel").exists():
                state, report = "cancelled", None
            elif process.returncode:
                raise _error((job["folder"] / "worker.log").read_text(errors="replace")[-2000:] or "清理计算失败")
            else:
                report = json.loads((job["folder"] / "result/preview.json").read_text())
                state = "ready"
            with self.lock:
                job.update(state=state, report=report)
        except Exception as error:
            with self.lock:
                job.update(state="failed", error=str(error))

    def _job(self, job_id):
        try:
            return self.jobs[job_id]
        except KeyError:
            raise _error("清理预览已不存在，请重新预览") from None

    def status(self, job_id):
        with self.lock:
            job = self._job(job_id)
            report = dict(job["report"]) if job["report"] else None
            if report:
                # Exact deletion keys stay in the native artifact; UI counts are
                # actual counts, not the percentage of the whole map to delete.
                report["preview"] = {k: v for k, v in report["preview"].items() if k != "removed_keys"}
            return dict({key: job[key] for key in ("id", "state", "map_name", "strength_percent", "error", "result_map")}, report=report)

    def cancel(self, job_id):
        with self.lock:
            job = self._job(job_id)
            if job["state"] in {"applying", "applied"}:
                raise _error("地图已进入保存阶段，不能取消已提交的另存操作")
            (job["folder"] / "cancel").touch()
            if job["state"] in {"queued", "ready"}:
                job["state"] = "cancelled"
        return self.status(job_id)

    def points(self, job_id, kind):
        with self.lock:
            job = self._job(job_id)
            if job["state"] not in {"ready", "applied"}:
                raise _error("清理预览尚未完成")
            names = {"source": "source.xyzf32", "candidates": "candidates.xyzf32", "result": "cloud.xyzf32"}
            if kind not in names:
                raise _error("未知预览图层")
            return job["folder"] / "result" / names[kind]

    def apply(self, job_id, *, new_map_name, confirmed=False):
        if confirmed is not True:
            raise _error("请确认预览，再另存清理后的新地图")
        with self.lock:
            job = self._job(job_id)
            if job["state"] == "applied" and job["result_map"] == new_map_name:
                return self.status(job_id)
            if job["state"] != "ready":
                raise _error("请先完成最新力度的清理预览")
            name = self.owner._require_map_name(new_map_name)
            if name == job["map_name"] or self.owner._repository.get_map_asset(name) is not None:
                raise _error("请输入尚未使用的新地图名称，原图不会被覆盖")
            current = self.owner.get_map_asset_bundle(map_name=job["map_name"])
            source = job["source"]
            if current is None or (current.asset_identity, current.manifest_sha256) != (source.asset_identity, source.manifest_sha256):
                raise _error("原地图已改变，请重新预览")
            for role in ("fine", "dense"):
                if _sha(job["folder"] / (role + ".source")) != job["report"][f"source_{role}_sha256"]:
                    raise _error("预览输入发生变化，请重新预览")
            job["state"] = "applying"
        try:
            self._save_copy(job, name)
        except Exception as error:
            with self.lock:
                job.update(state="failed", error=str(error))
            raise
        with self.lock:
            job.update(state="applied", result_map=name)
        return self.status(job_id)

    def _save_copy(self, job, name):
        from contracts.map_workspace import MapAsset, MapAssetBundle, MapAssetStatus, AssetIdentityV2
        from services.geometry.mapping_backend_checkpoint_store import (MappingBackendCheckpointStore,
            CheckpointMemberIdentity, CheckpointArtifactReference)
        from services.geometry.geometry_runtime_provider import MappingBackendCheckpointExportBundle
        from services.geometry.fine_map_archive import read_fine_archive_root
        source, previous = job["source"], job["stored"].checkpoint
        folder = job["folder"] / "result"
        from services.geometry.checkpoint_payload import FrozenPayload
        def frozen(path):
            with path.open("rb") as stream:
                return FrozenPayload(iter(lambda: stream.read(1024 * 1024), b""), maximum_bytes=path.stat().st_size)
        fine, dense = frozen(folder / "fine.bin"), frozen(folder / "dense.bin")
        backend, xyz = frozen(folder / "backend.bin"), frozen(folder / "cloud.xyzf32")
        occupancy = (folder / "occupancy.bin").read_bytes()
        checkpoint_id = uuid.uuid4().hex
        # The output is a new offline revision of the same measured source,
        # never a new sensor observation or a fabricated later timestamp.
        occ_receipt = dict(previous.occupancy_receipt)
        cloud_receipt = dict(previous.cloud_receipt)
        cloud_receipt["cloud_revision"] += 1
        cloud_receipt["point_count"] = len(xyz) // 12
        occ_asset = {k: v for k, v in job["stored"].occupancy_asset.items() if k != "raw_data_sha256"}
        cloud_asset = {k: v for k, v in job["stored"].cloud_asset.items() if k != "xyz_sha256"}
        point_count = len(xyz) // 12
        _, root, _ = read_fine_archive_root(fine)
        cloud_asset.update(point_count=point_count, xyz_size=len(xyz), source_cloud_revision=cloud_receipt["cloud_revision"],
                           voxel_size=root["voxel_size_m"], payload_schema_version=2)
        def identity(role, **kwargs):
            return CheckpointMemberIdentity(checkpoint_id, role, occ_receipt, cloud_receipt, source.frame_id, **kwargs)
        checkpoint = replace(previous, schema_version=2, checkpoint_id=checkpoint_id, content_sha256=b"",
            payload=backend, payload_size=len(backend), occupancy_receipt=occ_receipt, cloud_receipt=cloud_receipt,
            canonical_occupancy_data=occupancy, canonical_cloud_xyz=xyz, fine_archive=fine, dense_archive=dense,
            backend_identity=identity("backend_state"), occupancy_identity=identity("canonical_occupancy"), cloud_identity=identity("canonical_cloud"),
            fine_reference=CheckpointArtifactReference(uuid.uuid4().hex, len(fine), 2, identity("fine_map",
                frame_generation=root["domain"]["frame_generation"], producer_generation=root["domain"]["producer_generation"])),
            dense_reference=CheckpointArtifactReference(uuid.uuid4().hex, len(dense), 2, identity("dense_cloud")))
        checkpoint, occ_asset, cloud_asset = MappingBackendCheckpointStore.validate_checkpoint_bundle(checkpoint,
            occupancy_asset=occ_asset, cloud_asset=cloud_asset)
        export = MappingBackendCheckpointExportBundle(checkpoint, occ_asset, cloud_asset)
        # Create a separate catalog entry, without activating it or changing the
        # source map's memory library, navigation state, or live obstacle stream.
        asset = MapAsset(map_name=name, display_name=name, bound_memory_library_name=name, status=MapAssetStatus.CREATED)
        staging = self.owner._asset_repository.create_staging_directory(name)
        runtime = self.owner._geometry_runtime
        try:
            snapshot = source.snapshot.model_copy(update={"map_name": name, "occupancy_grid": None, "cost_map": None, "semantic_map": None, "metadata": {**dict(source.snapshot.metadata), "map_name": name}})
            draft = MapAssetBundle(map_name=name, source_name=source.source_name, frame_id=source.frame_id, snapshot=snapshot,
                metadata={"cleanup_source_map": source.map_name, "cleanup_strength_percent": job["strength_percent"],
                          "cleanup_preview": self.status(job["id"])["report"]})
            draft = runtime._replace_bundle_occupancy_from_checkpoint(draft, export)
            stored = export.publish_to(staging)
            geometry = source.geometry_asset.model_copy(deep=True)
            geometry = geometry.model_copy(update={"map_name": name, "metadata": {**dict(geometry.metadata),
                "mapping_backend_checkpoint": {"present": True, "schema_version": 2, "backend_kind": checkpoint.backend_kind,
                    "checkpoint_id": checkpoint_id, "manifest_path": stored.manifest_path.relative_to(staging).as_posix()}}})
            pcd = staging / geometry.pcd_path
            pcd.parent.mkdir(parents=True, exist_ok=True)
            with pcd.open("xb") as stream:
                stream.write(("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                    "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (point_count, point_count)).encode())
                for chunk in xyz.chunks():
                    stream.write(chunk)
                stream.flush()
                os.fsync(stream.fileno())
            geometry = geometry.model_copy(update={"quality_metrics": {**dict(geometry.quality_metrics),
                "pcd": {"present": True, "point_count": point_count, "source_kind": "completed_map_cleanup"}},
                "metadata": {**dict(geometry.metadata), "pcd": {"pcd_present": True, "point_count": point_count,
                    "cloud_revision": cloud_receipt["cloud_revision"], "pcd_target_path": geometry.pcd_path}}})
            runtime._materialize_native_bundle_database(staging / geometry.mapping_database_path, bundle=draft,
                checkpoint_bundle=export, mapping_context={"session_id": checkpoint.source_session_id, "session_generation": checkpoint.source_session_generation})
            runtime._materialize_nav2_map_assets(draft, bundle_root=staging)
            source_root = job["source_root"]
            history = source.metadata.get("raw_scan_history") or {}
            for key in ("path", "final_poses"):
                if history.get(key):
                    target = staging / history[key]
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source_root / history[key], target)
            if source.georeference:
                contract = source.georeference.contract.model_copy(update={"map_name": name})
                georeference = source.georeference.model_copy(update={"contract": contract})
            else:
                georeference = None
            identity = AssetIdentityV2(asset_id=uuid.uuid4().hex)
            candidate = MapAssetBundle(**{**{key: getattr(draft, key) for key in type(draft).model_fields},
                "geometry_asset": geometry, "georeference": georeference, "identity_schema_version": 2,
                "asset_identity": identity, "asset_state": "candidate", "metadata": {**dict(draft.metadata), "raw_scan_history": history}})
            geometry_path = staging / geometry.metadata.get("geometry_asset_path", "geometry/geometry_asset.json")
            geometry_path.parent.mkdir(parents=True, exist_ok=True)
            geometry_path.write_text(geometry.model_dump_json(indent=2), encoding="utf-8")
            self.owner._asset_repository.finalize_assigned_map_asset_bundle(candidate, staging_dir=staging, asset_identity=identity)
            finalized = self.owner._asset_repository.get_map_asset_bundle(name, migrate_legacy=False, expected_asset_identity=identity)
            if finalized is None:
                raise _error("清理后新图保存重载失败")
            # Publish the catalog entry only after the complete new bundle can
            # be reopened. A failed calculation never leaves a fake saved map.
            self.owner._repository.save_map_asset(asset.model_copy(update={
                "identity_schema_version": 2, "asset_identity": identity, "finalized": True,
                "finalized_at": finalized.timestamp, "frame_id": finalized.frame_id,
                "status": MapAssetStatus.READY, "metadata": {"cleanup_source_map": source.map_name}}))
        except Exception:
            if staging.exists():
                self.owner._asset_repository.discard_staging_directory(name, staging)
            raise
