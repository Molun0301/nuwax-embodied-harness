"""系统性能指标采集器（平台侧纯逻辑，基于 psutil）。

设计要点（对齐 SP12 性能监控方案）：
- 通用性优先：只用 psutil 读 /proc、/sys、syscall，与设备/机器人无关，
  在 Jetson / 树莓派 / 香橙派 / x86 上行为一致；不引入任何设备专属依赖。
- 非阻塞采样：所有 cpu_percent 调用均为 interval=None（非阻塞），由调用方
  以固定周期轮询，周期本身即采样间隔；禁止在此处使用阻塞式 interval>0。
- 首样预热：cpu_percent(interval=None) 首次返回无意义 0.0，须 warmup() 丢弃。
- Process 对象缓存复用：保留 dict[pid, Process] 才能让 cpu_percent 有“上次调用”基准。
- 优雅降级：psutil 缺失或单进程异常都不抛出，返回带状态的快照。
"""

from __future__ import annotations

import logging
import os
import time
from typing import Dict, List, Optional, Sequence

from contracts.debug_views import (
    CpuMetric,
    DebugDataStatus,
    MemoryMetric,
    ProcessMetric,
    SystemMetricsSnapshot,
)

try:  # psutil 为可选运行依赖，缺失时整体降级而非崩溃。
    import psutil
except Exception:  # pragma: no cover - 仅在未安装 psutil 的环境触发
    psutil = None  # type: ignore[assignment]

TELEMETRY_LOGGER = logging.getLogger("nuwax_robot_bridge.telemetry")

# 默认通用进程关键字集：覆盖网关与常见 ROS/导航/建图组件，按 cmdline 子串匹配。
# 不绑定任何机器人型号；“python” 因过宽（会误纳无关解释器进程）被刻意排除。
_DEFAULT_PROCESS_KEYWORDS = ("gateway", "rtabmap", "nav2", "robot_bridge", "ros2")


class SystemMetricsCollector:
    """采集系统 CPU / 内存 / 本项目进程指标。

    线程模型：实例由单一采集线程顺序调用（warmup 一次后周期 sample），
    Process 缓存因此被独占访问，无需加锁。
    """

    def __init__(
        self,
        *,
        keywords: Optional[Sequence[str]] = None,
        include_process_tree: bool = True,
        root_pid: Optional[int] = None,
        sample_interval_sec: float = 2.0,
    ) -> None:
        # 关键字统一小写，便于 cmdline/name 子串大小写不敏感匹配。
        raw_keywords = tuple(keywords) if keywords is not None else _DEFAULT_PROCESS_KEYWORDS
        self._keywords = tuple(k.strip().lower() for k in raw_keywords if k and k.strip())
        self._include_process_tree = include_process_tree
        self._root_pid = root_pid if root_pid is not None else os.getpid()
        self._sample_interval_sec = max(0.1, float(sample_interval_sec))
        # pid -> psutil.Process 缓存，复用以维持 cpu_percent 基准。
        self._proc_cache: Dict[int, "psutil.Process"] = {}
        self._warmed_up = False

    @property
    def available(self) -> bool:
        """psutil 是否可用。"""

        return psutil is not None

    @property
    def sample_interval_sec(self) -> float:
        return self._sample_interval_sec

    @property
    def monitor_scope(self) -> str:
        """当前监控范围的可读说明。"""

        parts: List[str] = []
        if self._include_process_tree:
            parts.append(f"进程树(root_pid={self._root_pid})")
        if self._keywords:
            parts.append("关键字[" + ",".join(self._keywords) + "]")
        return " + ".join(parts) if parts else "无"

    def warmup(self) -> None:
        """预热：丢弃 cpu_percent 首次无意义的 0.0（系统级 + 每进程各一次）。"""

        if psutil is None:
            return
        try:
            psutil.cpu_percent(interval=None)
        except Exception:
            TELEMETRY_LOGGER.debug("系统 CPU 预热失败。", exc_info=True)
        for proc in self._discover_processes().values():
            self._safe_prime_proc_cpu(proc)
        self._warmed_up = True

    @staticmethod
    def _safe_prime_proc_cpu(proc: "psutil.Process") -> None:
        """对单个进程预热 cpu_percent，吞掉进程态异常。"""

        try:
            proc.cpu_percent(interval=None)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
        except Exception:
            TELEMETRY_LOGGER.debug("进程 CPU 预热失败。", exc_info=True)

    def _matches(self, name: str, cmdline: str) -> Optional[str]:
        """按 cmdline 子串匹配关键字，name 作为补充。返回命中的关键字或 None。"""

        haystack_cmd = cmdline.lower()
        haystack_name = name.lower()
        for keyword in self._keywords:
            if keyword in haystack_cmd or keyword in haystack_name:
                return keyword
        return None

    def _discover_processes(self) -> Dict[int, "psutil.Process"]:
        """每周期重扫匹配进程：进程树成员 ∪ 关键字命中。

        维护 Process 缓存：新进程纳入并预热一次，退出进程清出缓存，
        存活进程复用对象以保 cpu_percent 基准。
        """

        if psutil is None:
            return {}

        matched: Dict[int, "psutil.Process"] = {}
        tree_pids = self._collect_process_tree_pids()

        for proc in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
            try:
                pid = proc.info.get("pid")
                if pid is None:
                    continue
                name = proc.info.get("name") or ""
                cmdline_list = proc.info.get("cmdline") or []
                cmdline = " ".join(cmdline_list) if isinstance(cmdline_list, (list, tuple)) else str(cmdline_list)
                matched_keyword: Optional[str] = None
                if self._include_process_tree and pid in tree_pids:
                    matched_keyword = "process_tree"
                if matched_keyword is None:
                    matched_keyword = self._matches(name, cmdline)
                if matched_keyword is None:
                    continue
                cached = self._proc_cache.get(pid)
                if cached is None:
                    cached = proc
                    self._proc_cache[pid] = cached
                    self._safe_prime_proc_cpu(cached)
                matched[pid] = cached
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
            except Exception:
                TELEMETRY_LOGGER.debug("遍历进程时异常，跳过该进程。", exc_info=True)
                continue

        # 清出已退出/不再匹配的进程缓存，避免无界增长。
        for stale_pid in [pid for pid in self._proc_cache if pid not in matched]:
            self._proc_cache.pop(stale_pid, None)
        return matched

    def _collect_process_tree_pids(self) -> set:
        """收集 root_pid 自身 + 全部子孙进程的 pid 集合。"""

        pids: set = set()
        if psutil is None or not self._include_process_tree:
            return pids
        try:
            root = psutil.Process(self._root_pid)
            pids.add(root.pid)
            for child in root.children(recursive=True):
                pids.add(child.pid)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
        except Exception:
            TELEMETRY_LOGGER.debug("收集进程树失败。", exc_info=True)
        return pids

    def _unavailable_snapshot(self) -> SystemMetricsSnapshot:
        """psutil 不可用时的降级快照。"""

        return SystemMetricsSnapshot(
            status=DebugDataStatus.UNSUPPORTED,
            psutil_available=False,
            sample_interval_sec=self._sample_interval_sec,
            monitor_scope=self.monitor_scope,
            sampled_at=time.time(),
        )

    def sample(self) -> SystemMetricsSnapshot:
        """一次完整非阻塞采样：系统 CPU/内存 + 被监控进程列表。"""

        if psutil is None:
            return self._unavailable_snapshot()
        if not self._warmed_up:
            self.warmup()

        cpu = self._sample_cpu()
        memory = self._sample_memory()
        processes = self._sample_processes()
        status = DebugDataStatus.AVAILABLE if processes else DebugDataStatus.MISSING
        return SystemMetricsSnapshot(
            status=status,
            psutil_available=True,
            sample_interval_sec=self._sample_interval_sec,
            monitor_scope=self.monitor_scope,
            cpu=cpu,
            memory=memory,
            processes=processes,
            sampled_at=time.time(),
            source="psutil",
        )

    def _sample_cpu(self) -> CpuMetric:
        percent = 0.0
        per_core: Optional[List[float]] = None
        loadavg: Optional[List[float]] = None
        try:
            percent = float(psutil.cpu_percent(interval=None))
            per_core = [float(v) for v in psutil.cpu_percent(interval=None, percpu=True)]
        except Exception:
            TELEMETRY_LOGGER.debug("采集系统 CPU 失败。", exc_info=True)
        try:
            if hasattr(psutil, "getloadavg"):
                loadavg = [float(v) for v in psutil.getloadavg()]
        except Exception:
            loadavg = None
        return CpuMetric(percent=percent, per_core=per_core, loadavg=loadavg)

    def _sample_memory(self) -> MemoryMetric:
        try:
            vm = psutil.virtual_memory()
            swap_used: Optional[int] = None
            swap_total: Optional[int] = None
            try:
                sm = psutil.swap_memory()
                swap_used, swap_total = int(sm.used), int(sm.total)
            except Exception:
                swap_used = swap_total = None
            return MemoryMetric(
                total_bytes=int(vm.total),
                used_bytes=int(vm.used),
                available_bytes=int(vm.available),
                percent=float(vm.percent),
                swap_used_bytes=swap_used,
                swap_total_bytes=swap_total,
            )
        except Exception:
            TELEMETRY_LOGGER.debug("采集系统内存失败。", exc_info=True)
            return MemoryMetric()

    def _sample_processes(self) -> List[ProcessMetric]:
        results: List[ProcessMetric] = []
        tree_pids = self._collect_process_tree_pids()
        for pid, proc in self._discover_processes().items():
            try:
                with proc.oneshot():
                    name = proc.name()
                    cpu_percent = float(proc.cpu_percent(interval=None))
                    rss = int(proc.memory_info().rss)
                cmdline_list = []
                try:
                    cmdline_list = proc.cmdline()
                except Exception:
                    cmdline_list = []
                cmdline = " ".join(cmdline_list) if cmdline_list else ""
                if self._include_process_tree and pid in tree_pids:
                    matched_keyword = "process_tree"
                else:
                    matched_keyword = self._matches(name, cmdline)
                results.append(
                    ProcessMetric(
                        pid=pid,
                        name=name,
                        matched_keyword=matched_keyword,
                        rss_bytes=rss,
                        cpu_percent=cpu_percent,
                    )
                )
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                self._proc_cache.pop(pid, None)
                continue
            except Exception:
                TELEMETRY_LOGGER.debug("采集进程 %s 指标失败，跳过。", pid, exc_info=True)
                continue
        # 按 CPU 占用降序，便于面板优先展示重载进程。
        results.sort(key=lambda p: p.cpu_percent, reverse=True)
        return results



