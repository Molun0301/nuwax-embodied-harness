#!/usr/bin/env python3
"""修复 NuwaClaw 容器云端离线：重建 nuwax-client 并注入正确 DNS/hosts。"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts import bootstrap as boot  # noqa: E402


def _load_saved_key(explicit: str) -> str:
    if explicit.strip():
        return explicit.strip()
    env_file = REPO_ROOT / ".env"
    if env_file.is_file():
        for line in env_file.read_text(encoding="utf-8").splitlines():
            if line.startswith("NUWAX_SAVED_KEY="):
                return line.split("=", 1)[1].strip()
    return str(os.environ.get("NUWAX_SAVED_KEY", "") or "").strip()


def main() -> int:
    parser = argparse.ArgumentParser(description="重建 NuwaClaw 容器并修复云端 DNS")
    parser.add_argument("--saved-key", default="", help="NUWAX_SAVED_KEY，默认读 .env")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    saved_key = _load_saved_key(args.saved_key)
    if not saved_key:
        print("未找到 NUWAX_SAVED_KEY，请写入 .env 或通过 --saved-key 传入。", file=sys.stderr)
        return 1

    command = [
        "docker",
        "run",
        "-d",
        "--name",
        boot.DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME,
        "--restart",
        "always",
        "--add-host",
        "host.docker.internal:host-gateway",
        *boot._nuwaclaw_docker_network_args(),
        "-p",
        f"{boot.DEFAULT_NUWACLAW_WEB_PORT}:{boot.DEFAULT_NUWACLAW_WEB_PORT}",
        "-p",
        f"{boot.DEFAULT_NUWACLAW_FILE_SERVER_PORT}:{boot.DEFAULT_NUWACLAW_FILE_SERVER_PORT}",
        "-p",
        f"{boot.DEFAULT_NUWACLAW_AGENT_PORT}:{boot.DEFAULT_NUWACLAW_AGENT_PORT}",
        "-v",
        f"{boot.DEFAULT_NUWACLAW_CLIENT_VOLUME_NAME}:/home/user",
        "-e",
        "TZ=Asia/Shanghai",
        "-e",
        f"NUWAX_SERVER_HOST={boot.DEFAULT_NUWACLAW_SERVER_HOST}",
        "-e",
        f"NUWAX_AGENT_PORT={boot.DEFAULT_NUWACLAW_AGENT_PORT}",
        "-e",
        f"NUWAX_FILE_SERVER_PORT={boot.DEFAULT_NUWACLAW_FILE_SERVER_PORT}",
        "-e",
        f"NUWAX_WORKSPACE_DIR={boot.DEFAULT_NUWACLAW_WORKSPACE_DIR}",
        "-e",
        f"NUWAX_SAVED_KEY={saved_key}",
        boot.DEFAULT_NUWACLAW_CLIENT_IMAGE,
    ]
    redacted = boot._build_redacted_nuwaclaw_docker_command(command)

    print("将重建容器并写入云端 hosts/DNS 修复：")
    print(redacted)

    if args.dry_run:
        return 0

    subprocess.run(
        ["docker", "rm", "-f", boot.DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME],
        check=False,
    )
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
        print("容器重建失败。", file=sys.stderr)
        return result.returncode

    print("容器已重建。等待 20s 后检查 lanproxy 连接...")
    subprocess.run(["sleep", "20"], check=False)
    check = subprocess.run(
        [
            "docker",
            "exec",
            boot.DEFAULT_NUWACLAW_CLIENT_CONTAINER_NAME,
            "getent",
            "hosts",
            "nac01.nuwax.com",
            "agent.nuwax.com",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    print(check.stdout.strip() or check.stderr.strip())
    if "198.18." in (check.stdout + check.stderr):
        print("警告: 容器仍解析到 198.18.x 假地址，请检查 NUWACLAW_DOCKER_DNS。", file=sys.stderr)
        return 2
    print("DNS 修复完成，请在云端刷新设备在线状态。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())