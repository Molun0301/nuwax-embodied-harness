#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""install_nav2_action_proxy phase：阶段 8 — 注册 nav2_action_proxy.service。

设计要点（openspec/changes/navigation-and-exploration-boundary/design.md 决策 D1 + R5）：

* 模板来源：``deploy/systemd/nav2_action_proxy.service``（task 3.4 已落地）。
* 执行步骤：
  1. ``cp deploy/systemd/nav2_action_proxy.service /etc/systemd/system/``
  2. ``systemctl daemon-reload``
  3. ``systemctl enable nav2_action_proxy.service``
  4. ``systemctl start nav2_action_proxy.service``
* 与 installer-redesign 共享 ``runtime_data/.gateway.lock``（memory
  start-gateway-shared-lock-path.md：禁止 .installer.lock 二锁方案）。
* phase 顺序：在 ``install_geometry_system_dependencies`` 之后；本 phase order = 8，
  原 systemd phase (7) 仍负责 nuwax-host-gateway 渲染。
* 跨机器人通用化：robot_profile.forbidden_phases 命中本阶段时直接 SKIP。

【G7】``_is_forbidden_by_robot_profile`` 守门 —— 其它平台（如 G1 / 树莓派 2D 雷达
无 Nav2 需求）可声明 ``forbidden_phases=("install_nav2_action_proxy",)`` 跳过。
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from typing import List

from scripts.installer._lock import acquire_lock, release_lock
from scripts.installer.phases.base import Phase


SERVICE_NAME = "nav2_action_proxy.service"
SERVICE_TEMPLATE = Path("deploy/systemd") / SERVICE_NAME
SYSTEMD_DEST = Path("/etc/systemd/system") / SERVICE_NAME
# 与 installer-redesign / start_gateway.sh 共用锁路径（memory start-gateway-shared-lock-path.md）
SHARED_LOCK_PATH = Path("runtime_data/.gateway.lock")


def _run(cmd: List[str], *, check: bool = True) -> subprocess.CompletedProcess:
    """subprocess 封装；失败时按 check 决定抛错或返回。"""
    return subprocess.run(cmd, check=check, capture_output=True, text=True)


class InstallNav2ActionProxyPhase(Phase):
    """阶段 8 — 注册 nav2_action_proxy.service（plan 4.1 L121 阶段化扩展）。"""

    name = "install_nav2_action_proxy"
    order = 8
    requires = ("systemd",)
    retryable = True
    dry_run_capable = True

    def run(self, planner, ctx) -> int:
        if self._is_forbidden_by_robot_profile(planner):
            ctx.info(self._robot_profile_skip_message())
            return 0
        if not SERVICE_TEMPLATE.exists():
            ctx.info(
                "[phase: %s] 模板不存在 path=%s，跳过注册（gateway 仍独立运行）",
                self.name,
                SERVICE_TEMPLATE,
            )
            return 0

        # 共享 runtime_data/.gateway.lock（与 installer-redesign / start_gateway.sh 互斥）
        try:
            acquire_lock(timeout=0.0)
        except RuntimeError as exc:
            ctx.info(
                "[phase: %s] gateway lock 已被占用（%s），跳过",
                self.name,
                exc,
            )
            return 0
        try:
            ctx.info(f"[phase: {self.name}] 注册 {SERVICE_NAME}")
            try:
                shutil.copyfile(SERVICE_TEMPLATE, SYSTEMD_DEST)
            except PermissionError as exc:
                ctx.info(
                    "[phase: %s] 复制 service 模板失败（非 root?）path=%s error=%s",
                    self.name,
                    SYSTEMD_DEST,
                    exc,
                )
                return 0
            try:
                _run(["systemctl", "daemon-reload"])
            except (FileNotFoundError, subprocess.CalledProcessError) as exc:
                ctx.info(f"[phase: {self.name}] systemctl daemon-reload 失败：{exc}")
                return 0
            try:
                _run(["systemctl", "enable", SERVICE_NAME])
            except (FileNotFoundError, subprocess.CalledProcessError) as exc:
                ctx.info(f"[phase: {self.name}] systemctl enable 失败：{exc}")
                return 0
            try:
                _run(["systemctl", "start", SERVICE_NAME], check=False)
            except FileNotFoundError:
                # 非 systemd 环境（开发机 / 容器）—— daemon-reload 已成功即可
                ctx.info(f"[phase: {self.name}] systemctl 不可用，跳过 start")
            ctx.success(f"[phase: {self.name}] {SERVICE_NAME} 已注册并启动")
            return 0
        finally:
            release_lock()

    def dry_run(self, planner, ctx) -> List[str]:
        if self._is_forbidden_by_robot_profile(planner):
            return [f"[SKIP] {self.name}: robot_profile.forbidden_phases 命中"]
        if not SERVICE_TEMPLATE.exists():
            return [f"[SKIP] {self.name}: 模板缺失 path={SERVICE_TEMPLATE}"]
        return [
            f"+ cp {SERVICE_TEMPLATE} {SYSTEMD_DEST}",
            "+ systemctl daemon-reload",
            f"+ systemctl enable {SERVICE_NAME}",
            f"+ systemctl start {SERVICE_NAME}",
            f"+ lock 共享: {SHARED_LOCK_PATH}",
        ]


from scripts.installer.phases import default_registry  # noqa: E402

default_registry.register(name=InstallNav2ActionProxyPhase.name, cls=InstallNav2ActionProxyPhase)