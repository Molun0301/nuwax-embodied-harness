"""离线地图清理作业；地图内部格式完全交给 Geometry 工具。"""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
from typing import Dict, List, Optional
from uuid import uuid4

from core.errors import GatewayError
from core.tasks import Cancelled, TaskContext
from services.geometry.maps import MapStore


def _failure(message: str) -> GatewayError:
    return GatewayError(message, error_code="map_cleanup_failed", http_status=409)


@dataclass
class _Job:
    id: str
    map_name: str
    strength_percent: int
    directory: Path
    state: str = "queued"
    report: Optional[dict] = None
    error: Optional[str] = None
    result_map: Optional[str] = None
    process: Optional[subprocess.Popen] = None
    future: Optional[Future] = None
    cancelled: threading.Event = field(default_factory=threading.Event)
    completed: threading.Event = field(default_factory=threading.Event)


class MapCleanupService:
    def __init__(self, config, maps: MapStore) -> None:
        self._binary = str(config.geometry.external.binary_path)
        self._cwd = str(config.base_dir)
        self._maps = maps
        self._map_root = maps.root.resolve()
        self._root = self._map_root / ".cleanup"
        if self._root.exists():
            shutil.rmtree(self._root)
        self._lock = threading.RLock()
        self._jobs: Dict[str, _Job] = {}
        self._closed = False
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="map-cleanup")

    def _job(self, preview_id: str) -> _Job:
        try:
            return self._jobs[preview_id]
        except KeyError:
            raise GatewayError("清理预览不存在，请重新预览。", error_code="map_cleanup_not_found", http_status=404) from None

    def _check_open(self) -> None:
        if self._closed:
            raise _failure("地图清理服务已关闭。")

    def preview(self, map_name: str, strength_percent: int = 50) -> dict:
        with self._lock:
            self._check_open()
            preview_id = uuid4().hex
            directory = self._root / preview_id
            directory.mkdir(parents=True)
            job = _Job(preview_id, map_name, strength_percent, directory)
            self._jobs[preview_id] = job
            job.future = self._executor.submit(self._preview, job)
        return self.status(preview_id)

    @staticmethod
    def _check_cancelled(job: _Job, ctx: Optional[TaskContext]) -> None:
        if ctx is not None:
            ctx.check()
        if job.cancelled.is_set():
            raise Cancelled()

    def _tool(self, job: _Job, arguments: List[str], ctx: Optional[TaskContext] = None) -> None:
        self._check_cancelled(job, ctx)
        environment = dict(os.environ, NUWAX_MAP_CATALOG_ROOT=str(self._map_root))
        process = None
        try:
            # 错误输出写入临时文件，避免管道写满阻塞工具；退出后完整读取真实原因。
            with tempfile.TemporaryFile() as error_output:
                with self._lock:
                    if job.cancelled.is_set():
                        raise Cancelled()
                    process = subprocess.Popen(
                        [self._binary, "--cleanup-completed-cloud", *arguments],
                        cwd=self._cwd, env=environment, stdin=subprocess.DEVNULL,
                        stdout=subprocess.DEVNULL, stderr=error_output,
                    )
                    job.process = process
                try:
                    while process.poll() is None:
                        self._check_cancelled(job, ctx)
                        if ctx is None:
                            job.cancelled.wait(0.05)
                        else:
                            ctx.wait(0.05)
                    process.wait()
                except BaseException:
                    if process.poll() is None and not job.cancelled.is_set():
                        process.terminate()
                    process.wait()
                    raise
                if process.returncode:
                    self._check_cancelled(job, ctx)
                    error_output.seek(0)
                    reason = error_output.read().decode("utf-8", errors="replace").strip()
                    raise _failure(reason or f"地图清理工具退出，退出码 {process.returncode}。")
        except OSError as error:
            raise _failure(str(error)) from error
        finally:
            with self._lock:
                if job.process is process:
                    job.process = None

    def _preview(self, job: _Job) -> None:
        state, report, error_message = "ready", None, None
        try:
            with self._lock:
                if job.cancelled.is_set():
                    raise Cancelled()
                job.state = "running"
            self._tool(job, ["preview", "--map", job.map_name, "--strength", str(job.strength_percent),
                             "--out", str(job.directory)])
            self._check_cancelled(job, None)
            with (job.directory / "preview.json").open(encoding="utf-8") as stream:
                report = json.load(stream)
            if not isinstance(report, dict):
                raise ValueError("清理工具的 preview.json 必须是统计摘要对象。")
        except Cancelled:
            state = "cancelled"
        except Exception as error:
            state, error_message = "failed", str(error)
        finally:
            with self._lock:
                job.state = "cancelled" if job.cancelled.is_set() else state
                job.report, job.error = report, error_message
                job.completed.set()

    def status(self, preview_id: str) -> dict:
        with self._lock:
            job = self._job(preview_id)
            return {"id": job.id, "state": job.state, "map_name": job.map_name,
                    "strength_percent": job.strength_percent, "report": deepcopy(job.report),
                    "error": job.error, "result_map": job.result_map}

    @staticmethod
    def _remove_directory(job: _Job) -> None:
        try:
            shutil.rmtree(job.directory)
        except FileNotFoundError:
            pass

    def _cancel(self, job: _Job) -> None:
        with self._lock:
            job.cancelled.set()
            process = job.process
            if job.future is not None and job.future.cancel():
                job.completed.set()
        if process is not None:
            process.terminate()
        job.completed.wait()
        self._remove_directory(job)
        with self._lock:
            if job.state != "applied":
                job.state = "cancelled"

    def cancel(self, preview_id: str) -> dict:
        with self._lock:
            job = self._job(preview_id)
            if job.state in {"applying", "applied"}:
                raise _failure("预览已进入另存阶段；正在执行的另存请通过任务取消。")
            job.state = "cancelling"
            job.cancelled.set()
        try:
            self._cancel(job)
        except OSError as error:
            raise _failure(str(error)) from error
        return self.status(preview_id)

    def points(self, preview_id: str, kind: str) -> Path:
        filenames = {"source": "source.xyzf32", "candidates": "candidates.xyzf32", "result": "result.xyzf32"}
        try:
            filename = filenames[kind]
        except KeyError:
            raise GatewayError("未知清理预览图层。", error_code="invalid_params", http_status=400) from None
        with self._lock:
            job = self._job(preview_id)
            if job.state not in {"ready", "applying"}:
                raise _failure("清理预览尚未完成或已取消。")
            return job.directory / filename

    def _write_meta(self, job: _Job, new_map_name: str) -> None:
        try:
            meta = self._maps.read_meta(job.map_name)
            now = datetime.now(timezone.utc).isoformat()
            meta.update(display_name=new_map_name, created_at=now, updated_at=now,
                        cleanup_source_map=job.map_name, cleanup_strength_percent=job.strength_percent,
                        cleanup_preview=deepcopy(job.report))
            if isinstance(meta.get("georeference"), dict):
                meta["georeference"] = {**meta["georeference"], "map_name": new_map_name}
            self._maps.write_meta(new_map_name, meta)
        except (OSError, ValueError) as error:
            raise _failure(str(error)) from error

    def apply(self, preview_id: str, new_map_name: str, ctx: TaskContext) -> dict:
        ctx.check()
        with self._lock:
            self._check_open()
            job = self._job(preview_id)
            if job.state != "ready":
                raise _failure("请先完成清理预览，再另存新地图。")
            job.state = "applying"
            job.error = None
            job.completed.clear()
        try:
            ctx.progress(0.1, "applying", "正在另存清理后的新地图。")
            self._tool(job, ["apply", "--work", str(job.directory), "--map", new_map_name], ctx)
            # 工具成功退出即已安装目标地图，随后完成网关拥有的显示信息。
            with self._lock:
                job.result_map = new_map_name
            self._write_meta(job, new_map_name)
            with self._lock:
                job.state = "applied"
            self._remove_directory(job)
            return self.status(preview_id)
        except Cancelled:
            with self._lock:
                job.cancelled.set()
                job.state = "cancelled"
            self._remove_directory(job)
            raise
        except Exception as error:
            with self._lock:
                job.state = "ready" if job.result_map is None and (self._map_root / new_map_name).exists() else "failed"
                job.error = str(error)
            raise
        finally:
            job.completed.set()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            jobs = tuple(self._jobs.values())
        errors = []
        for job in jobs:
            try:
                self._cancel(job)
            except Exception as error:
                errors.append(error)
        self._executor.shutdown(wait=True)
        if errors:
            raise errors[0]
