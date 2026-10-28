#!/usr/bin/env python3
"""在检查与两平台预编译成功后发布匿名main通道附件。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.neh_source import remote_revision, source_version
from scripts.neh_prebuilt import PROGRAMS, PLATFORMS, RELEASE, fetch_index
from scripts.device_manager.common import timestamp


def main():
    repository, revision = os.environ["GITHUB_REPOSITORY"], os.environ["GITHUB_SHA"]
    if os.environ.get("GITHUB_REF") != "refs/heads/main":
        raise RuntimeError("只有main分支成功的产品总工作流可以发布main通道")
    if remote_revision("https://github.com/" + repository + ".git", "main")["revision"] != revision:
        print("main已前进，本轮索引保留，通道交给新提交发布。")
        return
    index = fetch_index(repository, revision)
    for platform in PLATFORMS:
        if set(index["platforms"].get(platform, {}).get("programs", {})) != set(PROGRAMS):
            raise RuntimeError("通道不能引用缺少平台或程序的索引")
    policy_path = Path("channels/main-policy.json")
    policy = json.loads(policy_path.read_text()) if policy_path.exists() else {}
    confirmation = policy.get("requires_confirmation", False)
    if type(confirmation) is not bool or set(policy) - {"requires_confirmation"}:
        raise ValueError("通道策略仅接受requires_confirmation布尔值")
    value = {"schema": 1, "product": "neh", "channel": "main", "revision": revision,
             "version": "main@" + revision[:8], "published_at": timestamp(),
             "requires_confirmation": confirmation, "minimum_manager_version": source_version(revision)}
    with tempfile.TemporaryDirectory(prefix="neh-channel-publish-") as raw:
        path = Path(raw) / "channels-main.json"
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        subprocess.run(["gh", "release", "upload", RELEASE, str(path), "--repo", repository, "--clobber"], check=True)
    print("已发布channels/main.json逻辑通道：Release附件channels-main.json，提交" + revision)


if __name__ == "__main__":
    main()
