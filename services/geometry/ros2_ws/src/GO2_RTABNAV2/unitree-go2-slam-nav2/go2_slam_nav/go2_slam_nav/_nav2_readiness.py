"""Nav2 lifecycle 就绪检查（Foxy）。"""

from __future__ import annotations

import os
import re
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Optional, Tuple

_ACTIVE_ID = 3
_CACHE_TTL_SEC = 2.0
_PER_NODE_TIMEOUT_SEC = 2.5

_DEFAULT_NODES = (
    'controller_server',
    'planner_server',
    'recoveries_server',
    'bt_navigator',
)

_lifecycle_cache = {
    'at': 0.0,
    'ready': False,
    'issues': [],  # type: List[str]
}


def _lifecycle_get_subprocess(node_name: str, timeout_sec: float = _PER_NODE_TIMEOUT_SEC) -> Tuple[Optional[int], str]:
    """通过 ros2 CLI 查询 lifecycle，避免在 timer 回调内 spin 死锁。"""
    cmd = ['ros2', 'lifecycle', 'get', f'/{node_name}']
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=max(1.0, float(timeout_sec)),
            env=os.environ,
        )
        output = ((proc.stdout or '') + (proc.stderr or '')).strip()
        match = re.search(r'\[(\d+)\]', output)
        if match:
            return int(match.group(1)), output
        if proc.returncode != 0:
            return None, output or f'exit {proc.returncode}'
        return None, output or 'no state parsed'
    except subprocess.TimeoutExpired:
        return None, 'lifecycle 查询超时'
    except FileNotFoundError:
        return None, 'ros2 CLI 不可用'
    except Exception as exc:
        return None, str(exc)


def check_nav2_lifecycle_ready(
    node=None,
    *,
    node_names: Tuple[str, ...] = _DEFAULT_NODES,
    timeout_sec: float = 12.0,
    use_cache: bool = True,
) -> Tuple[bool, List[str]]:
    """返回 (是否全部 active, 未就绪节点列表)。node 参数保留兼容，不再使用。"""
    del node
    now = time.monotonic()
    if use_cache and (now - _lifecycle_cache['at']) < _CACHE_TTL_SEC:
        return _lifecycle_cache['ready'], list(_lifecycle_cache['issues'])

    per_node_timeout = max(_PER_NODE_TIMEOUT_SEC, float(timeout_sec) / max(1, len(node_names)))
    issues: List[str] = []

    with ThreadPoolExecutor(max_workers=len(node_names)) as pool:
        futures = {
            pool.submit(_lifecycle_get_subprocess, name, per_node_timeout): name
            for name in node_names
        }
        for future in as_completed(futures):
            name = futures[future]
            try:
                state_id, detail = future.result()
            except Exception as exc:
                issues.append(f'{name} lifecycle 查询失败 ({exc})')
                continue
            if state_id is None:
                issues.append(f'{name} {detail}')
            elif state_id != _ACTIVE_ID:
                issues.append(f'{name} 非 active (state={state_id})')

    ready = len(issues) == 0
    _lifecycle_cache['at'] = now
    _lifecycle_cache['ready'] = ready
    _lifecycle_cache['issues'] = list(issues)
    return ready, issues


def nav2_action_ready(
    node,
    action_client,
    *,
    stabilize_sec: float = 3.0,
    ready_since_attr: str = '_go2_nav2_ready_since',
) -> Tuple[bool, str]:
    """action server + lifecycle active + 稳定等待。"""
    if action_client is None or not action_client.server_is_ready():
        setattr(node, ready_since_attr, None)
        return False, 'navigate_to_pose action server 未就绪'
    ready, issues = check_nav2_lifecycle_ready()
    if not ready:
        setattr(node, ready_since_attr, None)
        return False, '；'.join(issues)
    now = time.monotonic()
    ready_since = getattr(node, ready_since_attr, None)
    if ready_since is None:
        setattr(node, ready_since_attr, now)
        ready_since = now
    remain = float(stabilize_sec) - (now - float(ready_since))
    if remain > 0.0:
        return False, f'Nav2 lifecycle 刚就绪，请再等 {remain:.0f}s'
    return True, ''