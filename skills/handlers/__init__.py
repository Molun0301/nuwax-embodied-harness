"""skills.handlers — 网关 handler 下放后的域级模块包（P2-4）。

每个域一个模块（system_skills/perception_skills/...），handler 为模块级纯函数，
首参 ctx: SkillContext，由网关侧 functools.partial(fn, ctx) 绑定后注册进 handlers dict。
本包仅依赖 core/contracts/services/skills.common，禁止 import gateways（守 R4b）。
"""

from __future__ import annotations
