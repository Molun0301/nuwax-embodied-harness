"""本地制品存储过渡再导出。

LocalArtifactStore 已下沉至 `services.artifact_store`（解开 services ⇄ gateways
包级循环）。本模块保留为再导出 shim，供尚未迁移的网关内部引用过渡使用。新代码请直接
从 `services.artifact_store` 导入。
"""

from __future__ import annotations

from services.artifact_store import LocalArtifactStore

__all__ = ["LocalArtifactStore"]
