"""skills 公共脚手架（P2-4 批0）。

导出参数校验纯函数模块 params 与依赖容器 SkillContext，供后续各批 handler 下放共用。
"""

from __future__ import annotations

from skills.common import params
from skills.common.context import SkillContext
from skills.common.providers import get_provider

__all__ = ["params", "SkillContext", "get_provider"]
