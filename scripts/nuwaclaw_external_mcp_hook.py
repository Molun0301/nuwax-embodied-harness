#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""为 NuwaClaw/NuwaxBot 自动安装 MCP 配置注入。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
from pathlib import Path
import stat
import subprocess
import sys
from typing import Any, Optional, Sequence

try:
    from scripts.nuwaclaw_mcp_config_sync import ConfigSyncError, apply_json_config, read_json_setting
except ModuleNotFoundError:  # pragma: no cover - direct script execution fallback
    from nuwaclaw_mcp_config_sync import ConfigSyncError, apply_json_config, read_json_setting


DEFAULT_NUWAXBOT_HOME = Path("/var/lib/docker/volumes/nuwax_home/_data/.nuwaxbot")
DEFAULT_MODERN_HOME = Path("/var/lib/docker/volumes/nuwax_home/_data/.nuwaclaw")
DEFAULT_EXTERNAL_CONFIG = Path("/var/lib/docker/volumes/nuwax_home/_data/.nuwaxbot/mcp/external_servers.json")
DEFAULT_PROXY_SCRIPT = Path(
    "/var/lib/docker/volumes/nuwax_home/_data/.nuwaxbot/node_modules/nuwax-mcp-stdio-proxy/dist/index.js"
)
DEFAULT_RUNTIME_EXTERNAL_CONFIG = Path("/home/user/.nuwaxbot/mcp/external_servers.json")
DEFAULT_MODERN_PROXY_IN_CONTAINER = Path("/opt/NuwaClaw/resources/nuwax-mcp-stdio-proxy/dist/index.js")
DEFAULT_CONTAINER_CANDIDATES = ("nuwax-client", "nuwaxbot", "nuwaclaw")
LEGACY_CONFIG_DIR_NAME = ".nuwaxbot"
MODERN_CONFIG_DIR_NAME = ".nuwaclaw"
LEGACY_DB_NAME = "nuwaxbot.db"
MODERN_DB_NAME = "nuwaclaw.db"
MODE_AUTO = "auto"
MODE_LEGACY_HOOK = "legacy-hook"
MODE_MODERN_DB = "modern-db"
BACKUP_SCRIPT_NAME = "index.codex-real.js"
WRAPPER_MARKER = "NUWAX_EXTERNAL_MCP_WRAPPER"


class HookInstallError(RuntimeError):
    """安装外部 MCP hook 时的错误。"""


@dataclass(frozen=True)
class NuwaClawInstallTarget:
    """NuwaClaw/NuwaxBot 容器内 MCP 注入目标。"""

    injection_mode: str
    container_name: Optional[str]
    host_home_path: Path
    runtime_home_path: Path
    config_home_path: Path
    runtime_config_home_path: Path
    proxy_script_path: Path
    external_config_path: Path
    runtime_external_config_path: Path
    db_path: Optional[Path]


def _load_json_file(path: Path) -> dict:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise HookInstallError("外部 MCP 配置文件不存在: %s" % path) from exc
    except json.JSONDecodeError as exc:
        raise HookInstallError("外部 MCP 配置文件不是合法 JSON: %s" % exc) from exc

    if not isinstance(payload, dict):
        raise HookInstallError("外部 MCP 配置文件根节点必须是对象。")
    servers = payload.get("mcpServers")
    if not isinstance(servers, dict):
        raise HookInstallError("外部 MCP 配置文件必须包含对象类型的 mcpServers。")
    return payload


def _normalize_json(payload: dict) -> str:
    return json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"


def _build_wrapper_script(runtime_external_config_path: Path, backup_script_name: str) -> str:
    external_config_literal = json.dumps(str(runtime_external_config_path))
    backup_script_literal = json.dumps(backup_script_name)
    return """#!/usr/bin/env node
// %s
import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import process from "node:process";
import { spawn } from "node:child_process";
import { fileURLToPath } from "node:url";

const WRAPPER_MARKER = %s;
const REAL_SCRIPT_NAME = %s;
const DEFAULT_EXTERNAL_CONFIG = %s;
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const REAL_SCRIPT_PATH = path.join(__dirname, REAL_SCRIPT_NAME);

function readJsonFile(filePath) {
  return JSON.parse(fs.readFileSync(filePath, "utf-8"));
}

function writeMergedConfig(baseConfig, externalConfigPath) {
  if (!fs.existsSync(externalConfigPath)) {
    return null;
  }

  const externalConfig = readJsonFile(externalConfigPath);
  if (!externalConfig || typeof externalConfig !== "object" || typeof externalConfig.mcpServers !== "object") {
    throw new Error("外部 MCP 配置文件缺少 mcpServers。");
  }

  const mergedConfig = {
    ...baseConfig,
    mcpServers: {
      ...(baseConfig.mcpServers || {}),
      ...(externalConfig.mcpServers || {}),
    },
  };

  const configDir = path.join(os.tmpdir(), "nuwax-mcp-configs");
  fs.mkdirSync(configDir, { recursive: true });
  const mergedPath = path.join(configDir, "mcp-config-external-merged.json");
  fs.writeFileSync(mergedPath, JSON.stringify(mergedConfig, null, 2), "utf-8");
  return mergedPath;
}

function buildChildArgs(originalArgs) {
  const args = originalArgs.slice();
  const externalConfigPath = process.env.NUWAX_EXTERNAL_MCP_CONFIG || DEFAULT_EXTERNAL_CONFIG;

  let configIndex = -1;
  let configFileIndex = -1;
  for (let i = 0; i < args.length; i++) {
    if (args[i] === "--config" && i + 1 < args.length) {
      configIndex = i;
    }
    if (args[i] === "--config-file" && i + 1 < args.length) {
      configFileIndex = i;
    }
  }

  if (configIndex === -1 && configFileIndex === -1) {
    return args;
  }

  let baseConfig;
  if (configFileIndex !== -1) {
    baseConfig = readJsonFile(args[configFileIndex + 1]);
  } else {
    baseConfig = JSON.parse(args[configIndex + 1]);
  }

  const mergedPath = writeMergedConfig(baseConfig, externalConfigPath);
  if (!mergedPath) {
    return args;
  }

  if (configFileIndex !== -1) {
    args[configFileIndex + 1] = mergedPath;
    return args;
  }

  args.splice(configIndex, 2, "--config-file", mergedPath);
  return args;
}

function main() {
  let childArgs = process.argv.slice(2);
  try {
    childArgs = buildChildArgs(childArgs);
  } catch (error) {
    console.error("[nuwax-external-mcp] 合并外部 MCP 配置失败，已回退到原始参数: %%s", error && error.stack ? error.stack : String(error));
  }
  const child = spawn(process.execPath, [REAL_SCRIPT_PATH, ...childArgs], {
    stdio: "inherit",
    env: process.env,
  });

  child.on("exit", (code, signal) => {
    if (signal) {
      process.kill(process.pid, signal);
      return;
    }
    process.exit(code === null ? 1 : code);
  });
}

main();
""" % (WRAPPER_MARKER, json.dumps(WRAPPER_MARKER), backup_script_literal, external_config_literal)


def _run_text_command(command: Sequence[str]) -> str:
    try:
        completed = subprocess.run(
            list(command),
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except FileNotFoundError as exc:
        raise HookInstallError("找不到命令: %s" % command[0]) from exc
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or exc.stdout or "").strip()
        raise HookInstallError("命令执行失败: %s%s" % (" ".join(command), "\n%s" % detail if detail else "")) from exc
    return completed.stdout


def _docker_inspect(container_name: str) -> dict[str, Any]:
    output = _run_text_command(("docker", "inspect", container_name))
    try:
        payload = json.loads(output)
    except json.JSONDecodeError as exc:
        raise HookInstallError("docker inspect 输出不是合法 JSON。") from exc
    if not isinstance(payload, list) or not payload or not isinstance(payload[0], dict):
        raise HookInstallError("docker inspect 未返回容器信息: %s" % container_name)
    return payload[0]


def _list_running_container_names() -> list[str]:
    output = _run_text_command(("docker", "ps", "--format", "{{.Names}}"))
    return [line.strip() for line in output.splitlines() if line.strip()]


def _candidate_container_names(explicit_container_name: Optional[str]) -> list[str]:
    if explicit_container_name:
        return [explicit_container_name]

    running_names = _list_running_container_names()
    candidates: list[str] = []
    for name in DEFAULT_CONTAINER_CANDIDATES:
        if name in running_names and name not in candidates:
            candidates.append(name)
    for name in running_names:
        lowered = name.lower()
        if "nuwax" in lowered and name not in candidates:
            candidates.append(name)
    return candidates


def _find_home_mount(inspect_payload: dict[str, Any]) -> tuple[Path, Path]:
    mounts = inspect_payload.get("Mounts")
    if not isinstance(mounts, list):
        raise HookInstallError("docker inspect 缺少 Mounts 信息。")

    fallback: Optional[tuple[Path, Path]] = None
    for mount in mounts:
        if not isinstance(mount, dict):
            continue
        source = mount.get("Source")
        destination = mount.get("Destination")
        if not isinstance(source, str) or not isinstance(destination, str):
            continue
        pair = (Path(source), Path(destination))
        if destination == "/home/user":
            return pair
        if "nuwax" in source.lower() and fallback is None:
            fallback = pair

    if fallback is not None:
        return fallback
    raise HookInstallError("未找到 NuwaClaw/NuwaxBot 的 /home/user Docker 挂载。")


def _merged_dir(inspect_payload: dict[str, Any]) -> Optional[Path]:
    graph_driver = inspect_payload.get("GraphDriver")
    if not isinstance(graph_driver, dict):
        return None
    data = graph_driver.get("Data")
    if not isinstance(data, dict):
        return None
    merged = data.get("MergedDir")
    if not isinstance(merged, str) or not merged:
        return None
    return Path(merged)


def _modern_proxy_script_path(inspect_payload: dict[str, Any]) -> Path:
    merged = _merged_dir(inspect_payload)
    if merged is None:
        return DEFAULT_MODERN_PROXY_IN_CONTAINER
    return merged / str(DEFAULT_MODERN_PROXY_IN_CONTAINER).lstrip("/")


def _build_modern_target(
    container_name: str,
    host_home_path: Path,
    runtime_home_path: Path,
    inspect_payload: dict[str, Any],
) -> NuwaClawInstallTarget:
    config_home_path = host_home_path / MODERN_CONFIG_DIR_NAME
    runtime_config_home_path = runtime_home_path / MODERN_CONFIG_DIR_NAME
    return NuwaClawInstallTarget(
        injection_mode=MODE_MODERN_DB,
        container_name=container_name,
        host_home_path=host_home_path,
        runtime_home_path=runtime_home_path,
        config_home_path=config_home_path,
        runtime_config_home_path=runtime_config_home_path,
        proxy_script_path=_modern_proxy_script_path(inspect_payload),
        external_config_path=config_home_path / "mcp" / "external_servers.json",
        runtime_external_config_path=runtime_config_home_path / "mcp" / "external_servers.json",
        db_path=config_home_path / MODERN_DB_NAME,
    )


def _build_legacy_target(
    container_name: Optional[str],
    host_home_path: Path,
    runtime_home_path: Path,
) -> NuwaClawInstallTarget:
    config_home_path = host_home_path / LEGACY_CONFIG_DIR_NAME
    runtime_config_home_path = runtime_home_path / LEGACY_CONFIG_DIR_NAME
    return NuwaClawInstallTarget(
        injection_mode=MODE_LEGACY_HOOK,
        container_name=container_name,
        host_home_path=host_home_path,
        runtime_home_path=runtime_home_path,
        config_home_path=config_home_path,
        runtime_config_home_path=runtime_config_home_path,
        proxy_script_path=config_home_path / "node_modules" / "nuwax-mcp-stdio-proxy" / "dist" / "index.js",
        external_config_path=config_home_path / "mcp" / "external_servers.json",
        runtime_external_config_path=runtime_config_home_path / "mcp" / "external_servers.json",
        db_path=config_home_path / LEGACY_DB_NAME,
    )


def _resolve_install_from_inspect(container_name: str, inspect_payload: dict[str, Any]) -> NuwaClawInstallTarget:
    host_home_path, runtime_home_path = _find_home_mount(inspect_payload)
    modern_config_home = host_home_path / MODERN_CONFIG_DIR_NAME
    legacy_config_home = host_home_path / LEGACY_CONFIG_DIR_NAME

    if (modern_config_home / MODERN_DB_NAME).exists() or modern_config_home.exists():
        return _build_modern_target(container_name, host_home_path, runtime_home_path, inspect_payload)

    if (legacy_config_home / LEGACY_DB_NAME).exists() or legacy_config_home.exists():
        return _build_legacy_target(container_name, host_home_path, runtime_home_path)

    raise HookInstallError(
        "未在挂载 %s 下找到 %s 或 %s 配置目录。"
        % (host_home_path, MODERN_CONFIG_DIR_NAME, LEGACY_CONFIG_DIR_NAME)
    )


def detect_nuwaclaw_install(container_name: Optional[str] = None) -> NuwaClawInstallTarget:
    errors: list[str] = []
    candidates = _candidate_container_names(container_name)
    if not candidates:
        raise HookInstallError("未找到运行中的 nuwaxclaw/nuwaxbot 容器。")

    for candidate in candidates:
        try:
            return _resolve_install_from_inspect(candidate, _docker_inspect(candidate))
        except HookInstallError as exc:
            errors.append("%s: %s" % (candidate, exc))

    raise HookInstallError("无法自动识别 NuwaClaw/NuwaxBot 容器路径: %s" % "; ".join(errors))


def _manual_legacy_target(args: argparse.Namespace) -> NuwaClawInstallTarget:
    config_home_path = Path(args.nuwaxbot_home or DEFAULT_NUWAXBOT_HOME).expanduser()
    runtime_home_path = DEFAULT_RUNTIME_EXTERNAL_CONFIG.parents[2]
    return NuwaClawInstallTarget(
        injection_mode=MODE_LEGACY_HOOK,
        container_name=None,
        host_home_path=config_home_path.parent,
        runtime_home_path=runtime_home_path,
        config_home_path=config_home_path,
        runtime_config_home_path=DEFAULT_RUNTIME_EXTERNAL_CONFIG.parents[1],
        proxy_script_path=config_home_path / "node_modules" / "nuwax-mcp-stdio-proxy" / "dist" / "index.js",
        external_config_path=config_home_path / "mcp" / "external_servers.json",
        runtime_external_config_path=DEFAULT_RUNTIME_EXTERNAL_CONFIG,
        db_path=config_home_path / LEGACY_DB_NAME,
    )


def _manual_modern_target(args: argparse.Namespace) -> NuwaClawInstallTarget:
    config_home_path = Path(args.nuwaxbot_home or DEFAULT_MODERN_HOME).expanduser()
    runtime_home_path = Path("/home/user")
    runtime_config_home_path = runtime_home_path / MODERN_CONFIG_DIR_NAME
    return NuwaClawInstallTarget(
        injection_mode=MODE_MODERN_DB,
        container_name=None,
        host_home_path=config_home_path.parent,
        runtime_home_path=runtime_home_path,
        config_home_path=config_home_path,
        runtime_config_home_path=runtime_config_home_path,
        proxy_script_path=DEFAULT_MODERN_PROXY_IN_CONTAINER,
        external_config_path=config_home_path / "mcp" / "external_servers.json",
        runtime_external_config_path=runtime_config_home_path / "mcp" / "external_servers.json",
        db_path=config_home_path / MODERN_DB_NAME,
    )


def _path_arg(value: Optional[str]) -> Optional[Path]:
    return None if value is None else Path(value).expanduser()


def resolve_install_target(args: argparse.Namespace) -> NuwaClawInstallTarget:
    detected: Optional[NuwaClawInstallTarget] = None
    if not getattr(args, "no_auto_detect", False):
        try:
            detected = detect_nuwaclaw_install(getattr(args, "container_name", None))
        except HookInstallError:
            if not any(
                (
                    getattr(args, "nuwaxbot_home", None),
                    getattr(args, "proxy_script", None),
                    getattr(args, "external_config", None),
                    getattr(args, "runtime_external_config", None),
                )
            ):
                raise

    forced_mode = getattr(args, "injection_mode", MODE_AUTO)
    if detected is not None:
        target = detected
    elif forced_mode == MODE_MODERN_DB:
        target = _manual_modern_target(args)
    else:
        target = _manual_legacy_target(args)
    if forced_mode != MODE_AUTO:
        target = replace(target, injection_mode=forced_mode)

    proxy_script = _path_arg(getattr(args, "proxy_script", None))
    external_config = _path_arg(getattr(args, "external_config", None))
    runtime_external_config = _path_arg(getattr(args, "runtime_external_config", None))
    nuwaxbot_home = _path_arg(getattr(args, "nuwaxbot_home", None))
    db_path = _path_arg(getattr(args, "db_path", None))

    if nuwaxbot_home is not None:
        if target.injection_mode == MODE_MODERN_DB:
            target = replace(
                target,
                config_home_path=nuwaxbot_home,
                host_home_path=nuwaxbot_home.parent,
                external_config_path=nuwaxbot_home / "mcp" / "external_servers.json",
                db_path=nuwaxbot_home / MODERN_DB_NAME,
            )
        else:
            target = replace(
                target,
                config_home_path=nuwaxbot_home,
                host_home_path=nuwaxbot_home.parent,
                proxy_script_path=nuwaxbot_home / "node_modules" / "nuwax-mcp-stdio-proxy" / "dist" / "index.js",
                external_config_path=nuwaxbot_home / "mcp" / "external_servers.json",
                db_path=nuwaxbot_home / LEGACY_DB_NAME,
            )
    if proxy_script is not None:
        target = replace(target, proxy_script_path=proxy_script)
    if external_config is not None:
        target = replace(target, external_config_path=external_config)
    if runtime_external_config is not None:
        target = replace(target, runtime_external_config_path=runtime_external_config)
    if db_path is not None:
        target = replace(target, db_path=db_path)
    return target


def install_hook(
    proxy_script_path: Path,
    external_config_path: Path,
    runtime_external_config_path: Path,
    *,
    force: bool = False,
) -> None:
    if not proxy_script_path.exists():
        raise HookInstallError("nuwax-mcp-stdio-proxy 入口文件不存在: %s" % proxy_script_path)

    _load_json_file(external_config_path)

    backup_path = proxy_script_path.with_name(BACKUP_SCRIPT_NAME)
    wrapper_content = _build_wrapper_script(runtime_external_config_path, BACKUP_SCRIPT_NAME)

    current_content = proxy_script_path.read_text(encoding="utf-8")
    if WRAPPER_MARKER in current_content and not force:
        return

    if not backup_path.exists():
        backup_path.write_text(current_content, encoding="utf-8")

    proxy_script_path.write_text(wrapper_content, encoding="utf-8")
    current_mode = proxy_script_path.stat().st_mode
    proxy_script_path.chmod(current_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def restore_hook(proxy_script_path: Path) -> None:
    backup_path = proxy_script_path.with_name(BACKUP_SCRIPT_NAME)
    if not backup_path.exists():
        raise HookInstallError("找不到备份文件，无法恢复: %s" % backup_path)
    proxy_script_path.write_text(backup_path.read_text(encoding="utf-8"), encoding="utf-8")
    current_mode = backup_path.stat().st_mode
    proxy_script_path.chmod(current_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


def write_external_config(config_path: Path, payload: dict, *, overwrite: bool = False) -> None:
    config_path.parent.mkdir(parents=True, exist_ok=True)
    if config_path.exists() and not overwrite:
        return
    config_path.write_text(_normalize_json(payload), encoding="utf-8")


def _build_default_payload() -> dict:
    return {
        "mcpServers": {
            "nuwax_robot_bridge": {
                "url": "http://host.docker.internal:8766/mcp",
                "headers": {
                    "Authorization": "Bearer agent-robot-bridge-token",
                },
            }
        }
    }


def apply_modern_db_injection(
    db_path: Path,
    config_path: Path,
    *,
    backup_path: Optional[Path] = None,
    create_backup: bool = True,
    merge: bool = True,
) -> bool:
    return apply_json_config(
        db_path,
        config_path,
        backup_path=backup_path,
        create_backup=create_backup,
        merge=merge,
    )


def _restart_container(container_name: str) -> None:
    _run_text_command(("docker", "restart", container_name))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="自动安装 NuwaClaw/NuwaxBot MCP 配置注入。")
    parser.add_argument(
        "--container-name",
        help="NuwaClaw/NuwaxBot 容器名；缺省自动查找 nuwax-client/nuwaxbot/nuwaclaw 等容器。",
    )
    parser.add_argument(
        "--no-auto-detect",
        action="store_true",
        help="禁用 Docker 自动探测，完全使用显式路径或旧版默认路径。",
    )
    parser.add_argument(
        "--injection-mode",
        choices=(MODE_AUTO, MODE_LEGACY_HOOK, MODE_MODERN_DB),
        default=MODE_AUTO,
        help="注入方式。auto 会按配置目录路径区分：.nuwaxbot=legacy-hook，.nuwaclaw=modern-db。",
    )
    parser.add_argument(
        "--nuwaxbot-home",
        default=None,
        help="NuwaClaw 数据目录，一般是 ~/.nuwaxbot 对应的宿主机路径。",
    )
    parser.add_argument(
        "--proxy-script",
        default=None,
        help="nuwax-mcp-stdio-proxy 的入口脚本路径。",
    )
    parser.add_argument(
        "--external-config",
        default=None,
        help="MCP JSON 配置文件路径。legacy-hook 中为宿主机持久化外部配置；modern-db 中为写入 SQLite 的声明式配置。",
    )
    parser.add_argument(
        "--runtime-external-config",
        default=None,
        help="容器内运行时读取的外部 MCP JSON 文件路径。",
    )
    parser.add_argument(
        "--db-path",
        default=None,
        help="modern-db 注入方式使用的 NuwaClaw SQLite 数据库路径。",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    install_parser = subparsers.add_parser("install", help="安装配置注入 hook。")
    install_parser.add_argument("--force", action="store_true", help="即使已经安装过也覆盖写入包装层。")
    install_parser.add_argument(
        "--init-config",
        action="store_true",
        help="如果外部配置文件不存在，则写入默认示例配置。",
    )
    install_parser.add_argument(
        "--overwrite-config",
        action="store_true",
        help="与 --init-config 一起使用时，覆盖已有外部配置文件。",
    )
    install_parser.add_argument("--backup-path", help="modern-db 写入前的数据库备份路径。")
    install_parser.add_argument("--no-backup", action="store_true", help="modern-db 写入前不备份数据库。")
    install_parser.add_argument(
        "--replace",
        action="store_true",
        help="modern-db 默认合并已有 mcpServers；传入该参数时全量替换 mcp_proxy_config。",
    )
    install_parser.add_argument(
        "--restart-container",
        action="store_true",
        help="写入后重启自动探测到的 NuwaClaw/NuwaxBot 容器，使 modern-db 配置立即加载。",
    )

    subparsers.add_parser("status", help="检查 hook 是否已安装。")
    subparsers.add_parser("restore", help="恢复原始 nuwax-mcp-stdio-proxy 入口脚本。")

    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        target = resolve_install_target(args)

        if args.command == "install":
            if args.init_config:
                write_external_config(
                    target.external_config_path,
                    _build_default_payload(),
                    overwrite=args.overwrite_config,
                )

            if target.injection_mode == MODE_MODERN_DB:
                if target.db_path is None:
                    raise HookInstallError("modern-db 注入缺少 --db-path。")
                backup_path = Path(args.backup_path).expanduser() if args.backup_path else None
                changed = apply_modern_db_injection(
                    target.db_path,
                    target.external_config_path,
                    backup_path=backup_path,
                    create_backup=not args.no_backup,
                    merge=not args.replace,
                )
                print("已同步 modern-db MCP 配置。" if changed else "modern-db MCP 配置未变化。")
                print("db=%s" % target.db_path)
                if args.restart_container:
                    if not target.container_name:
                        raise HookInstallError("无法重启容器：没有自动探测到 container_name。")
                    _restart_container(target.container_name)
                    print("已重启容器: %s" % target.container_name)
            else:
                install_hook(
                    target.proxy_script_path,
                    target.external_config_path,
                    target.runtime_external_config_path,
                    force=args.force,
                )
                print("已安装 legacy hook。")

            print("mode=%s" % target.injection_mode)
            print("container=%s" % (target.container_name or ""))
            print("proxy=%s" % target.proxy_script_path)
            print("external_config=%s" % target.external_config_path)
            print("runtime_external_config=%s" % target.runtime_external_config_path)
            return 0

        if args.command == "status":
            if target.injection_mode == MODE_MODERN_DB:
                if target.db_path is None:
                    raise HookInstallError("modern-db 注入缺少 --db-path。")
                payload = read_json_setting(target.db_path)
                servers = payload.get("mcpServers", {})
                installed = isinstance(servers, dict) and "nuwax_robot_bridge" in servers
                server_names = sorted(servers.keys()) if isinstance(servers, dict) else []
                print("installed=%s" % ("true" if installed else "false"))
                print("mode=%s" % target.injection_mode)
                print("container=%s" % (target.container_name or ""))
                print("db=%s" % target.db_path)
                print("mcp_servers=%s" % ",".join(server_names))
            else:
                if not target.proxy_script_path.exists():
                    raise HookInstallError("proxy 入口脚本不存在: %s" % target.proxy_script_path)
                content = target.proxy_script_path.read_text(encoding="utf-8")
                installed = WRAPPER_MARKER in content
                print("installed=%s" % ("true" if installed else "false"))
                print("mode=%s" % target.injection_mode)
                print("container=%s" % (target.container_name or ""))
                print("proxy=%s" % target.proxy_script_path)
                print("runtime_external_config=%s" % target.runtime_external_config_path)
            print("external_config=%s" % target.external_config_path)
            if target.external_config_path.exists():
                print("external_config_exists=true")
            else:
                print("external_config_exists=false")
            return 0 if installed else 1

        if args.command == "restore":
            if target.injection_mode == MODE_MODERN_DB:
                raise HookInstallError("modern-db 注入不支持自动 restore；请用数据库备份文件恢复。")
            restore_hook(target.proxy_script_path)
            print("已恢复原始 proxy 入口脚本。")
            return 0

        parser.error("未知命令。")
        return 2
    except (HookInstallError, ConfigSyncError) as exc:
        print("错误: %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
