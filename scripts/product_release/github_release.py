"""仅在GitHub Actions中使用内置令牌准备历史目录和原子公开版本。"""
from __future__ import annotations

import json
import os
import re
import subprocess
from pathlib import Path

from .validation import ReleaseError, digest, read_json


class GitHubRelease:
    def __init__(self, repository: str):
        if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repository):
            raise ReleaseError("GitHub仓库身份无效")
        self.repository = repository

    def api(self, endpoint: str, *, method: str = "GET", payload: dict = None, missing_ok: bool = False):
        command = ["gh", "api", endpoint, "--method", method, "-H", "Accept: application/vnd.github+json", "-H", "X-GitHub-Api-Version: 2022-11-28"]
        if payload is not None:
            command += ["--input", "-"]
        result = subprocess.run(command, input=json.dumps(payload).encode() if payload is not None else None, capture_output=True)
        if result.returncode:
            if missing_ok and b"HTTP 404" in result.stderr:
                return None
            raise ReleaseError("GitHub发布接口失败：" + result.stderr.decode("utf-8", errors="replace").strip())
        return json.loads(result.stdout) if result.stdout.strip() else None

    def release(self, tag: str):
        if not re.fullmatch(r"v[A-Za-z0-9._+-]+", tag):
            raise ReleaseError("版本标签无效")
        return self.api("repos/%s/releases/tags/%s" % (self.repository, tag), missing_ok=True)

    def download_asset(self, asset: dict, destination: Path) -> None:
        endpoint = "repos/%s/releases/assets/%s" % (self.repository, int(asset["id"]))
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open("wb") as output:
            result = subprocess.run(["gh", "api", endpoint, "-H", "Accept: application/octet-stream", "-H", "X-GitHub-Api-Version: 2022-11-28"], stdout=output, stderr=subprocess.PIPE)
        if result.returncode:
            destination.unlink(missing_ok=True)
            raise ReleaseError("下载既有发布资产失败：" + result.stderr.decode("utf-8", errors="replace"))

    def previous_catalog(self, destination: Path, *, current_tag: str) -> Path:
        releases = self.api("repos/%s/releases?per_page=100" % self.repository)
        if not isinstance(releases, list):
            raise ReleaseError("GitHub版本列表返回了无效结构")
        for release in sorted(releases, key=lambda item: item.get("published_at") or "", reverse=True):
            if release.get("draft") or release.get("tag_name") == current_tag:
                continue
            asset = next((item for item in release.get("assets", []) if item.get("name") == "catalog.json"), None)
            if asset is None:
                continue
            if asset.get("size", 0) > 16 * 1024 ** 2:
                raise ReleaseError("既有版本目录超过大小限制")
            self.download_asset(asset, destination)
            catalog = read_json(destination.read_bytes())
            if catalog.get("schema_version") != 1 or catalog.get("product") != "neh" or not isinstance(catalog.get("releases"), list):
                raise ReleaseError("既有版本目录无效，拒绝用新目录覆盖历史")
            return destination
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text('{"schema_version":1,"product":"neh","releases":[]}\n', encoding="utf-8")
        return destination

    def check_published(self, version: str, revision: str, targets: list) -> bool:
        release = self.release("v" + version)
        if release is None:
            return False
        if release.get("draft"):
            if release.get("target_commitish") != revision:
                raise ReleaseError("该版本草稿属于其它源码修订，请核对草稿或递增版本")
            return False
        # 已公开版本绝不覆盖。只有同一提交且目标集合相同的重复运行可以直接成功结束。
        if release.get("target_commitish") != revision:
            raise ReleaseError("版本已公开且源码修订不同，请递增发布版本：" + version)
        plan_asset = next((item for item in release.get("assets", []) if item.get("name") == "release-plan.json"), None)
        if plan_asset is None:
            raise ReleaseError("版本已存在但没有可核对的发布计划，不能覆盖")
        temporary = Path(os.environ.get("RUNNER_TEMP", ".")) / "neh-existing-plan.json"
        self.download_asset(plan_asset, temporary)
        existing = read_json(temporary.read_bytes())
        temporary.unlink()
        if set(existing.get("requests", [])) != set(targets):
            raise ReleaseError("已公开版本的目标集合不同，请递增发布版本")
        return True

    def publish(self, directory: Path, *, version: str, revision: str, channel: str, notes: str) -> str:
        tag = "v" + version
        index = read_json((directory / "release-index.json").read_bytes())
        if index.get("version") != version or index.get("revision") != revision:
            raise ReleaseError("发布文件索引与当前提交/版本不符")
        expected = {item["path"]: item for item in index["files"]}
        files = {path.name: path for path in directory.iterdir() if path.is_file()}
        if set(files) != set(expected) | {"release-index.json"}:
            raise ReleaseError("发布目录包含索引之外的文件或缺项")
        for name, item in expected.items():
            path = files[name]
            if path.is_symlink() or path.stat().st_size != item["size"] or digest(path) != item["sha256"]:
                raise ReleaseError("发布前文件核验失败：" + name)
        if len(files) > 1000 or any(path.stat().st_size >= 2 * 1024 ** 3 for path in files.values()):
            raise ReleaseError("发布附件超过GitHub限制")
        release = self.release(tag)
        if release is not None and not release.get("draft"):
            raise ReleaseError("该版本已公开，不能替换其附件")
        if release is None:
            release = self.api("repos/%s/releases" % self.repository, method="POST", payload={"tag_name": tag, "target_commitish": revision, "name": "NEH " + version, "body": notes, "draft": True, "prerelease": channel == "preview"})
        elif release.get("target_commitish") != revision:
            raise ReleaseError("既有草稿属于另一源码修订，请维护者核对后采用新版本")
        # 草稿可安全恢复上传；公开版本不进入此分支。索引最后上传，完成后才公开。
        current_assets = {item["name"]: item for item in release.get("assets", [])}
        for name in sorted(set(files) - {"release-index.json"}) + ["release-index.json"]:
            path = files[name]
            existing = current_assets.get(name)
            checksum = digest(path)
            if existing and existing.get("size") == path.stat().st_size and existing.get("digest") == "sha256:" + checksum:
                continue
            if existing:
                self.api("repos/%s/releases/assets/%s" % (self.repository, existing["id"]), method="DELETE")
            result = subprocess.run(["gh", "release", "upload", tag, str(path), "--repo", self.repository], capture_output=True, text=True)
            if result.returncode:
                raise ReleaseError("上传草稿附件失败（草稿未公开）：" + result.stderr.strip())
        confirmed = self.api("repos/%s/releases/%s" % (self.repository, release["id"]))
        actual = {item["name"]: item for item in confirmed.get("assets", [])}
        if set(actual) != set(files) or any(actual[name].get("size") != files[name].stat().st_size for name in files):
            raise ReleaseError("远端草稿附件数量或大小不一致，保留草稿并停止公开")
        for name, asset in actual.items():
            expected_digest = digest(files[name])
            if asset.get("digest"):
                if asset["digest"] != "sha256:" + expected_digest:
                    raise ReleaseError("远端草稿附件摘要不符，停止公开：" + name)
            else:
                temporary = directory.parent / (".verify-release-asset-" + str(asset["id"]))
                try:
                    self.download_asset(asset, temporary)
                    if digest(temporary) != expected_digest:
                        raise ReleaseError("远端附件读取核验失败：" + name)
                finally:
                    temporary.unlink(missing_ok=True)
        result = self.api("repos/%s/releases/%s" % (self.repository, release["id"]), method="PATCH", payload={"draft": False, "prerelease": channel == "preview", "make_latest": "false" if channel == "preview" else "true"})
        return result["html_url"]
