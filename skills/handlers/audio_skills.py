"""音频域 handler（P2-4 批 R1）。

从 gateways/runtime.py 下放的音频控制技能。handler 为模块级纯函数，首参 ctx:
SkillContext，函数体内原 self.audio_service→ctx.audio_service，原 self._require_float
改用 skills.common.params。行为与抛错条件与迁移前完全一致。

本批下放（分析文档 §1 音频域，2 个全 A 类）：handle_get_volume、handle_set_volume。
留网关（B 类，任务化）：speak_text（依赖 _resolve_async_timeout/_build_task_request_metadata，R3）。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from skills.common import params
from skills.common.context import SkillContext


def handle_get_volume(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """返回当前音量状态。

    迁移自 GatewayRuntime._handle_get_volume；self.audio_service→ctx.audio_service。无参数。
    """

    del payload, requested_by
    return {"volume_state": ctx.audio_service.get_volume_state()}


def handle_set_volume(
    ctx: SkillContext,
    payload: Dict[str, Any],
    *,
    requested_by: Optional[str] = None,
) -> Dict[str, Any]:
    """设置音量。

    迁移自 GatewayRuntime._handle_set_volume；self.audio_service→ctx.audio_service，
    self._require_float→params.require_float。
    """

    del requested_by
    volume = params.require_float(payload, "volume")
    return {"volume_state": ctx.audio_service.set_volume(volume)}
