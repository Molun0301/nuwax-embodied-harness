from __future__ import annotations

from dataclasses import dataclass
from contracts.compat import StrEnum
from typing import Tuple


class TcpLegacyStage(StrEnum):
    """旧 TCP 通道阶段。"""

    DEPRECATED = "deprecated"
    REMOVED = "removed"


@dataclass(frozen=True)
class TcpLegacyPolicy:
    """旧 TCP 控制面的兼容策略。"""

    stage: TcpLegacyStage
    entrypoint: str
    allows_new_feature_work: bool
    deprecation_gate: Tuple[str, ...]
    exit_criteria: Tuple[str, ...]


def build_tcp_legacy_policy() -> TcpLegacyPolicy:
    """返回当前 TCP 兼容策略。"""

    return TcpLegacyPolicy(
        stage=TcpLegacyStage.REMOVED,
        entrypoint="removed",
        allows_new_feature_work=False,
        deprecation_gate=(
            "旧 TCP 私有控制面已移除，所有能力统一走 MCP/HTTP/事件流。",
            "不再接受任何对 go2_proxy_server.py 的兼容性修复请求。",
        ),
        exit_criteria=(
            "保留历史策略说明，供运维面返回当前迁移状态。",
        ),
    )
