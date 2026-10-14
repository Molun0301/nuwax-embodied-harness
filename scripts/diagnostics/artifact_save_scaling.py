#!/usr/bin/env python3
"""制品保存/清理真实实现的离线扩展性诊断，仅使用自动清理的临时合成目录。

不启动机器人、模型、服务或网络，不读写已有制品目录。真实存储方法保持原样；
计数子类仅记录调用次数。保存耗时单独使用未覆盖方法的 LocalArtifactStore。

    .nuwax-test-env/bin/python scripts/diagnostics/artifact_save_scaling.py \
        --counts 100 1000 10000 --samples 7 --output /tmp/neh-artifact-before.json
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import platform
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from contracts.artifacts import ArtifactKind, ArtifactRetentionPolicy
from contracts.image import ImageEncoding, ImageFrame
from services.artifact_service import ArtifactService
from services.artifact_store import LocalArtifactStore

DATA = b"synthetic-storage-diagnostic:" + b"x" * 997


def save(store):
    return store.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=DATA)


def frame():
    return ImageFrame(camera_id="diagnostic", frame_id="camera", width_px=1,
                      height_px=1, encoding=ImageEncoding.JPEG, data=DATA)


def measure(call):
    started = time.perf_counter_ns()
    result = call()
    return (time.perf_counter_ns() - started) / 1_000_000, result


def summary(values):
    return {"samples_ms": values, "median_ms": statistics.median(values),
            "min_ms": min(values), "max_ms": max(values)}


class CountingStore(LocalArtifactStore):
    """不修改语义，仅累计实际调用和列表返回的元数据条数。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.counts = {"list_refs": 0, "sidecars_parsed": 0,
                       "reference_index_builds": 0, "delete_artifact": 0,
                       "find_meta_path": 0, "find_file_path": 0, "inventory_scans": 0, "delete_entry": 0}

    def list_refs(self, **kwargs):
        refs = super().list_refs(**kwargs)
        self.counts["list_refs"] += 1
        if not hasattr(LocalArtifactStore, "_scan_entries"):
            self.counts["sidecars_parsed"] += len(refs)
        return refs

    def _scan_entries(self):
        self.counts["inventory_scans"] += 1
        entries = super()._scan_entries()
        if not hasattr(LocalArtifactStore, "_parse_metadata"):
            self.counts["sidecars_parsed"] += len(entries)
        return entries

    def _parse_metadata(self, *args, **kwargs):
        self.counts["sidecars_parsed"] += 1
        return super()._parse_metadata(*args, **kwargs)

    def _delete_entry(self, *args, **kwargs):
        self.counts["delete_entry"] += 1
        return super()._delete_entry(*args, **kwargs)

    def _build_reference_index(self, *args, **kwargs):
        self.counts["reference_index_builds"] += 1
        return super()._build_reference_index(*args, **kwargs)

    def delete_artifact(self, *args, **kwargs):
        self.counts["delete_artifact"] += 1
        return super().delete_artifact(*args, **kwargs)

    def _find_meta_path(self, *args, **kwargs):
        self.counts["find_meta_path"] += 1
        return super()._find_meta_path(*args, **kwargs)

    def _find_file_path(self, *args, **kwargs):
        self.counts["find_file_path"] += 1
        return super()._find_file_path(*args, **kwargs)


def scaling(root, count, samples):
    directory = root / f"scaling-{count}"
    seed = LocalArtifactStore(str(directory), "http://diagnostic.invalid")
    for _ in range(count):
        save(seed)
    # 新实例模拟重启后的冷进程缓存；OS 页缓存仍由刚刚准备的 fixture 加热。
    store = LocalArtifactStore(str(directory), "http://diagnostic.invalid")
    policy = ArtifactRetentionPolicy(retention_days=7, max_count=count + 100,
                                     max_total_bytes=1024 ** 3, cleanup_batch_size=200)
    service = ArtifactService(store, retention_policy=policy)
    image = frame()
    cold_ms, _ = measure(lambda: service.save_image_frame(image))
    raw_samples, service_samples = [], []
    for _ in range(samples):
        raw_ms, _ = measure(lambda: save(store))
        service_ms, _ = measure(lambda: service.save_image_frame(image))
        raw_samples.append(raw_ms)
        service_samples.append(service_ms)
    counted = CountingStore(str(directory), "http://diagnostic.invalid")
    counted_service = ArtifactService(counted, retention_policy=policy)
    # 先以一次生产扫描加热计数实例的元数据缓存，再只计数下一次保存。
    if hasattr(counted, "_scan_entries"):
        counted._scan_entries()
        counted.counts = {key: 0 for key in counted.counts}
    counted_service.save_image_frame(image)
    expected_final = count + 2 * samples + 2
    assert len(tuple(directory.glob("**/*.meta.json"))) == expected_final
    return {"starting_artifacts": count, "ending_artifacts": expected_final,
            "payload_bytes": len(DATA), "cold_python_cache_service_save_ms": cold_ms,
            "raw_save": summary(raw_samples), "service_save": summary(service_samples),
            "counted_save_calls": counted.counts}


def edge_cases(root):
    results = {}
    store = CountingStore(str(root / "eviction"), "http://diagnostic.invalid")
    artifacts = [save(store) for _ in range(50)]
    calls = [0]

    def provider():
        calls[0] += 1
        return {artifacts[0].artifact_id: [{"source": "synthetic"}]}

    store.set_reference_provider(provider)
    elapsed, result = measure(lambda: store.cleanup(
        ArtifactRetentionPolicy(max_count=5, cleanup_batch_size=40)))
    assert result.removed_count == 40
    assert artifacts[0].artifact_id not in result.removed_artifact_ids
    results["eviction"] = {"elapsed_ms": elapsed, "provider_calls": calls[0],
                           "operations": store.counts, "result": result.model_dump(mode="json")}

    store = LocalArtifactStore(str(root / "failed-provider"), "http://diagnostic.invalid")
    old, _ = save(store), save(store)

    def failing_provider():
        raise RuntimeError("synthetic reference source unavailable")

    store.set_reference_provider(failing_provider)
    result = store.cleanup(ArtifactRetentionPolicy(max_count=1))
    results["provider_failure"] = {
        "cleanup_returned_success": True,
        "candidate_deleted_despite_unknown_reference_state": old.artifact_id in result.removed_artifact_ids,
        "maintenance_status": store.get_maintenance_status() if hasattr(store, "get_maintenance_status") else None}

    store = LocalArtifactStore(str(root / "changed-reference"), "http://diagnostic.invalid")
    old, _ = save(store), save(store)
    calls = [0]

    def changed_provider():
        calls[0] += 1
        return {} if calls[0] == 1 else {old.artifact_id: [{"source": "new-synthetic-reference"}]}

    store.set_reference_provider(changed_provider)
    result = store.cleanup(ArtifactRetentionPolicy(max_count=1))
    results["reference_added_after_snapshot"] = {
        "provider_calls": calls[0],
        "candidate_deleted_despite_later_reference": old.artifact_id in result.removed_artifact_ids}

    store = LocalArtifactStore(str(root / "oversized-new-image"), "http://diagnostic.invalid")
    service = ArtifactService(store, retention_policy=ArtifactRetentionPolicy(max_total_bytes=1))
    artifact = service.save_image_frame(frame())
    results["new_image_exceeds_byte_cap"] = {
        "save_returned_artifact": True,
        "returned_artifact_sidecar_exists": bool(store._find_meta_path(artifact.artifact_id)),
        "returned_artifact_data_exists": bool(store._find_file_path(artifact.artifact_id)),
        "maintenance_status": store.get_maintenance_status() if hasattr(store, "get_maintenance_status") else None}
    return results


def async_scaling(root, count, samples):
    """真实后台扫描并发测量；不插入人为扫描等待，不把窗口声称为实验室隔离。"""
    from core import StateStore
    from services.memory.artifact_references import MemoryArtifactReferences
    from services.memory.repository import MemoryRepository
    from services.observation_service import ObservationService

    directory = root / f"async-{count}"
    seed = LocalArtifactStore(str(directory), "http://diagnostic.invalid")
    for _ in range(count):
        save(seed)
    store = LocalArtifactStore(str(directory), "http://diagnostic.invalid")
    repo = MemoryRepository(":memory:")
    bridge = MemoryArtifactReferences(repo, store)
    policy = ArtifactRetentionPolicy(retention_days=7, max_count=count + samples * 3 + 200,
                                     max_total_bytes=1024 ** 3, cleanup_batch_size=200)
    service = ArtifactService(store, retention_policy=policy)
    image = frame()
    provider = SimpleNamespace(provider_name="synthetic", provider_version="1", is_available=lambda: True,
                               capture_image=lambda _camera: image, get_camera_info=lambda _camera: None)
    observations = ObservationService(provider_owner=SimpleNamespace(providers=provider), artifact_service=service,
                                      state_store=StateStore(), history_limit=2)
    scan_entered, scan_active = threading.Event(), threading.Event()
    scans = []
    original_scan = store._scan_entries
    def observed_scan():
        started = time.perf_counter_ns()
        scan_active.set()
        scan_entered.set()
        try:
            return original_scan()
        finally:
            scans.append({"duration_ms": (time.perf_counter_ns() - started) / 1_000_000,
                          "thread": threading.current_thread().name})
            scan_active.clear()
    store._scan_entries = observed_scan
    try:
        startup_ms, started = measure(service.start_maintenance)
        assert started
        worker = service._maintenance_worker
        cold_save_ms, _ = measure(lambda: service.save_image_frame(image))
        cold_wait_ms, idle = measure(lambda: worker.wait_idle(30))
        assert idle
        cold_scans = list(scans)
        scans.clear()
        raw_times, service_times, capture_times, health_times, drain_times, overlap = [], [], [], [], [], []
        for _ in range(samples):
            raw_ms, _ = measure(lambda: save(store))
            raw_times.append(raw_ms)
            scan_entered.clear()
            assert worker.request() is not None
            assert scan_entered.wait(10)
            active_at_start = scan_active.is_set()
            save_ms, _ = measure(lambda: service.save_image_frame(image))
            capture_ms, _ = measure(observations.capture_observation)
            active_at_end = scan_active.is_set()
            before_health_scans = len(scans)
            health_ms, _ = measure(lambda: (service.get_cached_summary(), service.get_maintenance_status()))
            service_times.append(save_ms)
            capture_times.append(capture_ms)
            health_times.append(health_ms)
            overlap.append({"scan_active_before_save": active_at_start, "scan_active_after_capture": active_at_end,
                            "completed_scans_during_status_call": len(scans) - before_health_scans})
            drain_ms, idle = measure(lambda: worker.wait_idle(30))
            drain_times.append(drain_ms)
            assert idle
        status = service.get_maintenance_status()
        operations = {"maintenance_scans": len(scans), "scan_threads": sorted({row["thread"] for row in scans}),
                      "worker_counts": status["worker"]["counts"], "active_requests": status["worker"]["active_requests"],
                      "pending_requests": status["worker"]["pending_requests"]}
        assert operations["scan_threads"] == ["nuwax_artifact_maintenance"]
        expected_final = count + 1 + 3 * samples
        actual_final = len(tuple(directory.glob("**/*.meta.json")))
        assert actual_final == expected_final
        return {"starting_artifacts": count, "ending_artifacts": actual_final, "payload_bytes": len(DATA),
                "worker_startup_ms": startup_ms, "cold_python_cache_service_save_ms": cold_save_ms,
                "cold_maintenance_wait_ms": cold_wait_ms, "cold_scans": cold_scans,
                "raw_save": summary(raw_times), "service_save": summary(service_times),
                "observation_capture": summary(capture_times), "cached_status": summary(health_times),
                "maintenance_drain_after_capture": summary(drain_times), "maintenance_scans": scans,
                "overlap": overlap, "operations": operations, "maintenance_status": status}
    finally:
        assert service.close(timeout=30)
        assert observations.close(timeout=3)
        assert bridge.close()
        repo.close()


def continuous_save_burst(root):
    from services.memory.artifact_references import MemoryArtifactReferences
    from services.memory.repository import MemoryRepository
    store = LocalArtifactStore(str(root / "continuous"), "http://diagnostic.invalid")
    for _ in range(1000):
        save(store)
    repo = MemoryRepository(":memory:")
    bridge = MemoryArtifactReferences(repo, store)
    service = ArtifactService(store, retention_policy=ArtifactRetentionPolicy(max_count=100000))
    try:
        assert service.start_maintenance()
        worker = service._maintenance_worker
        assert worker.request() is not None
        maximum_active = maximum_pending = 0
        values = []
        for _ in range(100):
            elapsed, _artifact = measure(lambda: service.save_image_frame(frame()))
            values.append(elapsed)
            state = worker.status(compact=True)
            maximum_active = max(maximum_active, state["active_requests"])
            maximum_pending = max(maximum_pending, state["pending_requests"])
        assert worker.wait_idle(30)
        actual_count = len(tuple(store.base_dir.glob("**/*.meta.json")))
        assert actual_count == 1100
        assert maximum_active <= 1 and maximum_pending <= 1
        return {"starting_artifacts": 1000, "continuous_saves": 100, "ending_artifacts": actual_count,
                "service_save": summary(values), "maximum_observed_active": maximum_active,
                "maximum_observed_pending": maximum_pending, "status": service.get_maintenance_status()}
    finally:
        assert service.close(timeout=30)
        assert bridge.close()
        repo.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--counts", type=int, nargs="+", default=[100, 1000, 10000])
    parser.add_argument("--samples", type=int, default=7)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--async-maintenance", action="store_true")
    args = parser.parse_args()
    if args.samples < 1 or any(count < 1 or count > 100000 for count in args.counts):
        parser.error("samples >= 1, artifact counts 1..100000")
    source_names = ["services/artifact_service.py", "services/artifact_store.py", "contracts/artifacts.py"]
    if args.async_maintenance:
        source_names += ["services/artifact_maintenance.py", "services/observation_service.py",
                         "services/memory/artifact_references.py", "services/memory/repository.py", "services/memory/sqlite_storage.py"]
    result = {"recorded_at_utc": datetime.now(timezone.utc).isoformat(),
              "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "source_files": source_names, "python": sys.version,
              "pydantic": importlib.metadata.version("pydantic"), "platform": platform.platform(),
              "cpu_affinity_count": len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
              "loadavg_start": os.getloadavg(), "async_maintenance": args.async_maintenance,
              "scope": "production storage methods, synthetic temporary files, warm OS page cache; shared host, not laboratory isolation"}
    with tempfile.TemporaryDirectory(prefix="neh-artifact-scaling-") as temporary:
        root = Path(temporary)
        result["scaling"] = []
        for count in args.counts:
            result["scaling"].append((async_scaling if args.async_maintenance else scaling)(root, count, args.samples))
            print(json.dumps(result["scaling"][-1]), flush=True)
        if not args.async_maintenance:
            result["edge_cases"] = edge_cases(root)
        else:
            result["continuous_save_burst"] = continuous_save_burst(root)
    result["loadavg_end"] = os.getloadavg()
    payload = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)


if __name__ == "__main__":
    main()
