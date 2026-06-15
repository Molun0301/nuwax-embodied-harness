from __future__ import annotations

from abc import ABC, abstractmethod
from datetime import datetime
from contracts.compat import StrEnum
from typing import Dict, Optional, Any, Generic, TypeVar

from pydantic import Field

from contracts.base import ContractModel, MetadataDict, utc_now

InputT = TypeVar("InputT")
OutputT = TypeVar("OutputT")


class AdapterCategory(StrEnum):
    """适配器类别。"""

    MAPPING = "mapping"
    LOCALIZATION = "localization"
    NAVIGATION = "navigation"
    PERCEPTION = "perception"
    STREAMS = "streams"


class AdapterLifecycleState(StrEnum):
    """适配器生命周期状态。"""

    CREATED = "created"
    INITIALIZED = "initialized"
    DEGRADED = "degraded"
    ERROR = "error"
    STOPPED = "stopped"


class AdapterConfig(ContractModel):
    """适配器配置。"""

    name: str = Field(description="适配器名称。")
    category: AdapterCategory = Field(description="适配器类别。")
    source_kind: str = Field(description="外部来源类型，例如 ros2、nav2、webrtc。")
    contract_type: str = Field(description="输出契约类型。")
    enabled: bool = Field(default=True, description="是否启用。")
    source_ref: Optional[str] = Field(default=None, description="外部来源引用，例如 topic、流地址或服务名。")
    healthcheck_timeout_sec: float = Field(default=1.0, gt=0.0, description="健康检查超时时间。")
    recoverable: bool = Field(default=True, description="是否允许恢复。")
    settings: MetadataDict = Field(default_factory=dict, description="适配器私有配置。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class AdapterHealthStatus(ContractModel):
    """适配器健康状态。"""

    name: str = Field(description="适配器名称。")
    category: AdapterCategory = Field(description="适配器类别。")
    source_kind: str = Field(description="外部来源类型。")
    contract_type: str = Field(description="输出契约类型。")
    lifecycle_state: AdapterLifecycleState = Field(description="生命周期状态。")
    is_healthy: bool = Field(description="当前是否健康。")
    message: Optional[str] = Field(default=None, description="健康说明。")
    last_error: Optional[str] = Field(default=None, description="最近一次错误。")
    initialized_at: Optional[datetime] = Field(default=None, description="初始化时间。")
    last_success_at: Optional[datetime] = Field(default=None, description="最近一次成功转换时间。")
    metadata: MetadataDict = Field(default_factory=dict, description="附加元数据。")


class AdapterBase(ABC, Generic[InputT, OutputT]):
    """平台适配器基类。"""

    def __init__(self, config: AdapterConfig) -> None:
        self.config = config
        self.lifecycle_state = AdapterLifecycleState.CREATED
        self.initialized_at: Optional[datetime] = None
        self.last_success_at: Optional[datetime] = None
        self.last_error: Optional[str] = None
        self.last_message: Optional[str] = "等待初始化。"

    @property
    def adapter_name(self) -> str:
        """返回适配器名称。"""

        return self.config.name

    def initialize(self) -> None:
        """初始化适配器。"""

        if not self.config.enabled:
            self.lifecycle_state = AdapterLifecycleState.STOPPED
            self.last_message = "适配器已禁用。"
            return

        try:
            self.on_initialize()
        except Exception as exc:
            self._mark_error(exc, "初始化失败。")
            raise

        self.initialized_at = utc_now()
        self.last_error = None
        self.last_message = "初始化完成。"
        self.lifecycle_state = AdapterLifecycleState.INITIALIZED

    def adapt(self, payload: InputT) -> OutputT:
        """执行标准化转换。"""

        if not self.config.enabled:
            raise RuntimeError(f"适配器 {self.adapter_name} 已禁用。")

        if self.lifecycle_state in {
            AdapterLifecycleState.CREATED,
            AdapterLifecycleState.ERROR,
            AdapterLifecycleState.STOPPED,
        }:
            self.initialize()

        try:
            result = self.convert_payload(payload)
        except Exception as exc:
            self._mark_error(exc, "转换失败。")
            raise

        self.last_success_at = utc_now()
        if self.lifecycle_state != AdapterLifecycleState.DEGRADED:
            self.lifecycle_state = AdapterLifecycleState.INITIALIZED
        self.last_message = "转换成功。"
        return result

    def mark_degraded(self, message: str) -> None:
        """标记为降级状态。"""

        self.lifecycle_state = AdapterLifecycleState.DEGRADED
        self.last_message = message

    def recover(self) -> None:
        """尝试恢复适配器。"""

        if not self.config.recoverable:
            raise RuntimeError(f"适配器 {self.adapter_name} 不允许恢复。")

        try:
            self.on_recover()
        except Exception as exc:
            self._mark_error(exc, "恢复失败。")
            raise

        self.last_error = None
        self.last_message = "恢复完成。"
        self.lifecycle_state = AdapterLifecycleState.INITIALIZED

    def stop(self) -> None:
        """停止适配器。"""

        try:
            self.on_stop()
        except Exception as exc:
            self._mark_error(exc, "停机失败。")
            raise

        self.lifecycle_state = AdapterLifecycleState.STOPPED
        self.last_message = "已停止。"

    def health_check(self) -> AdapterHealthStatus:
        """返回当前健康状态。"""

        is_healthy = self.config.enabled and self.lifecycle_state in {
            AdapterLifecycleState.INITIALIZED,
            AdapterLifecycleState.DEGRADED,
        }
        metadata = dict(self.config.metadata)
        metadata["source_ref"] = self.config.source_ref
        metadata["recoverable"] = self.config.recoverable
        return AdapterHealthStatus(
            name=self.adapter_name,
            category=self.config.category,
            source_kind=self.config.source_kind,
            contract_type=self.config.contract_type,
            lifecycle_state=self.lifecycle_state,
            is_healthy=is_healthy,
            message=self.last_message,
            last_error=self.last_error,
            initialized_at=self.initialized_at,
            last_success_at=self.last_success_at,
            metadata=metadata,
        )

    def describe(self) -> Dict[str, Any]:
        """返回适配器绑定摘要。"""

        return {
            "name": self.adapter_name,
            "category": self.config.category.value,
            "source_kind": self.config.source_kind,
            "contract_type": self.config.contract_type,
            "source_ref": self.config.source_ref,
            "enabled": self.config.enabled,
        }

    def on_initialize(self) -> None:
        """子类可选初始化钩子。"""

    def on_recover(self) -> None:
        """子类可选恢复钩子。"""

        self.on_initialize()

    def on_stop(self) -> None:
        """子类可选停机钩子。"""

    @abstractmethod
    def convert_payload(self, payload: InputT) -> OutputT:
        """把外部输入转换为平台契约。"""

    def _mark_error(self, exc: Exception, message: str) -> None:
        self.lifecycle_state = AdapterLifecycleState.ERROR
        self.last_error = str(exc)
        self.last_message = message
