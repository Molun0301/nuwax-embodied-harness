"""性能监控采集服务导出（平台侧，基于 psutil）。"""

from services.telemetry.collector import SystemMetricsCollector

__all__ = ["SystemMetricsCollector"]
