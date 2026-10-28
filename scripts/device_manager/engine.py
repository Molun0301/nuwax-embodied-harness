"""网页和命令行共用的持久安装事务；交互端退出不会撤销任务。"""
from __future__ import annotations

import base64
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import uuid
from pathlib import Path

from scripts.module_installer.catalog import MODULES, ModuleError, module_system_packages, resolve_modules
from scripts.module_installer.hardware import detect_hardware, recommend_modules
from scripts.module_installer.manager import existing_config
from scripts.module_installer.runtime import read_settings, _VALUES

from . import MANAGER_DEVELOPMENT, MANAGER_REVISION, MANAGER_VERSION, PACKAGE_ROOT
from .config_client import ConfigurationClient
from .common import (ManagerError, compare_versions, atomic_json, atomic_write, digest_file, file_lock,
                     fsync_directory, identifier, process_alive, process_identity, read_json, timestamp)
from .releases import download, github_catalog_url, load_catalog, platform_issues, unpack, validate_url
from .runtime import Runtime, clean_environment, probe, run_command, system_helper

TERMINAL = {"succeeded", "failed", "cancelled", "interrupted"}
ACTIONS = {"install", "update", "modules", "rollback", "start", "stop", "restart", "migrate", "update-manager"}


class Cancelled(ManagerError):
    def __init__(self):
        super().__init__("cancelled", "任务已在安全边界取消。", 409)


class Engine:
    def __init__(self, home):
        from .locations import installation_home, manager_home
        self.manager_home = manager_home(home)
        self.home = installation_home(self.manager_home)
        for root, directories in ((self.manager_home, ("config", "jobs", "plans", "logs")),
                                  (self.home, ("releases", "data", "config", "logs", "cache"))):
            for directory in directories:
                (root / directory).mkdir(parents=True, exist_ok=True, mode=0o700)
        self.lock_path = self.manager_home / "config/transaction.lock"
        self.runtime = Runtime(self.home, self.manager_home)
        identity = self.manager_home / "config/instance-id"
        try:
            descriptor = os.open(str(identity), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(descriptor, "w") as handle:
                handle.write(uuid.uuid4().hex)
                handle.flush()
                os.fsync(handle.fileno())
        except FileExistsError:
            pass
        self.instance_id = identity.read_text().strip()
        self.configuration = ConfigurationClient(self.home, self.instance_id)
        # 公开源迁移只删除本管理根的旧凭据，不读取或转移其内容。
        (self.manager_home / "config/credentials.json").unlink(missing_ok=True)

    def selected(self):
        """Return a fixed-root snapshot; never change paths under another request."""
        from .locations import installation_home
        return self if self.home == installation_home(self.manager_home) else Engine(self.manager_home)

    def _assert_location(self):
        from .locations import installation_home
        if self.home != installation_home(self.manager_home):
            raise ManagerError("install_location_changed", "安装位置已变化，请刷新页面并重新生成计划。", 409)

    def install_location(self):
        reason = ""
        if self.current():
            reason = "已安装业务版本，不能在网页中迁移安装目录。"
        elif self.active_job():
            reason = "管理任务正在执行，请等待任务结束后再选择目录。"
        else:
            active = read_json(self.manager_home / "config/active-job.json", {})
            if active and self.job(active["id"]).get("error", {}).get("code") == "recovery_failed":
                reason = "上次任务尚未完成安全恢复，请先处理恢复问题。"
        return {"install_dir": str(self.home), "manager_dir": str(self.manager_home), "parent_dir": str(self.home.parent),
                "user_home": str(Path.home()), "editable": not reason, "reason": reason}

    def preview_install_location(self, parent):
        from .locations import selected_directory
        target = selected_directory(parent)
        return {"install_dir": str(target), "parent_dir": str(target.parent), "user_home": str(Path.home())}

    def save_install_location(self, values):
        from .locations import prepare_directory, selected_directory
        if not isinstance(values, dict) or set(values) not in ({"install_dir"}, {"parent_dir"}):
            raise ManagerError("invalid_install_dir", "请提交parent_dir父目录或install_dir完整安装目录。")
        with file_lock(self.lock_path):
            self._assert_idle()
            location = self.install_location()
            if not location["editable"]:
                raise ManagerError("install_location_locked", location["reason"], 409)
            value = str(selected_directory(values["parent_dir"])) if "parent_dir" in values else values["install_dir"]
            target = prepare_directory(self, value)
            atomic_json(self.manager_home / "config/install-location.json", {"install_dir": str(target)})
        result = self.selected().install_location()
        if target != self.home:
            result["previous_install_dir"] = str(self.home)
        return result

    def install_directories(self, path=None):
        from .locations import browse_directories
        return browse_directories(self, path)

    def settings(self):
        from .sources import normalized_settings
        settings = normalized_settings(read_json(self.manager_home / "config/settings.json", {}))
        settings["password_enabled"] = bool(read_json(self.manager_home / "config/password.json", {}).get("digest"))
        return {key: value for key, value in settings.items() if not key.startswith("_")}

    def save_settings(self, values, *, allow_local_source=False):
        from .sources import is_project_default
        from scripts.neh_source import SourceError, validate_repository
        values = dict(values)
        if set(values) - {"feed_url", "source_repository", "source_branch", "source_mode", "channel", "auto_start", "gateway_port",
                          "auto_update", "check_interval_minutes", "maintenance_window"}:
            raise ManagerError("invalid_settings", "存在未知管理设置。")
        if "feed_url" in values and values["feed_url"]:
            validate_url(values["feed_url"])
        if "channel" in values and values["channel"] not in ("stable", "preview"):
            from scripts.neh_source import _branch
            channel = values["channel"]
            if channel == "main":
                branch = "main"
            elif isinstance(channel, str) and channel.startswith("branch:"):
                try:
                    branch = _branch(channel[7:])
                except SourceError as exc:
                    raise ManagerError("invalid_settings", str(exc)) from exc
            else:
                raise ManagerError("invalid_settings", "当前通道支持main或branch:分支名称，stable尚未开放。")
            if values.get("source_branch", branch) != branch:
                raise ManagerError("invalid_settings", "通道与源码分支不一致。")
            values.update(source_branch=branch, source_mode="git")
        elif "source_branch" in values and "channel" not in values:
            values["channel"] = "main" if values["source_branch"] == "main" else "branch:" + str(values["source_branch"])
        if "auto_update" in values and values["auto_update"] not in ("off", "notify", "idle"):
            raise ManagerError("invalid_settings", "更新档位支持off、notify、idle。")
        if "check_interval_minutes" in values and (type(values["check_interval_minutes"]) is not int or not 15 <= values["check_interval_minutes"] <= 1440):
            raise ManagerError("invalid_settings", "检查间隔必须为15—1440分钟。")
        if "maintenance_window" in values:
            from .updates import validate_window
            if not isinstance(values["maintenance_window"], str):
                raise ManagerError("invalid_settings", "维护窗口必须是字符串。")
            validate_window(values["maintenance_window"])
        if "source_mode" in values and values["source_mode"] not in ("git", "catalog"):
            raise ManagerError("invalid_settings", "源码模式只支持git或catalog。")
        if "source_branch" in values and (not isinstance(values["source_branch"], str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]{0,199}", values["source_branch"]) or ".." in values["source_branch"]):
            raise ManagerError("invalid_settings", "源码分支名称无效。")
        if "source_repository" in values:
            try:
                values["source_repository"] = validate_repository(values["source_repository"], allow_local=allow_local_source)
            except SourceError as exc:
                raise ManagerError("invalid_settings", str(exc)) from exc
            values["_allow_local_source"] = bool(allow_local_source)
            if values.get("source_mode") != "catalog":
                values.update(source_mode="git", feed_url=values["source_repository"])
        elif "feed_url" in values:
            if is_project_default(values["feed_url"]) and values.get("source_mode") != "catalog":
                from scripts.neh_source import DEFAULT_REPOSITORY
                values.update(source_mode="git", source_repository=DEFAULT_REPOSITORY, feed_url=DEFAULT_REPOSITORY, _allow_local_source=False)
            else:
                values["source_mode"] = "catalog"
        elif "source_branch" in values and "source_mode" not in values:
            values["source_mode"] = "git"
        if "auto_start" in values and not isinstance(values["auto_start"], bool):
            raise ManagerError("invalid_settings", "auto_start必须是布尔值。")
        if "gateway_port" in values:
            if type(values["gateway_port"]) is not int or not 1024 <= values["gateway_port"] <= 65535:
                raise ManagerError("invalid_settings", "业务端口必须是1024到65535之间的整数。")
            if self.current():
                raise ManagerError("managed_setting", "业务端口须在首次安装前设置。")
        with file_lock(self.lock_path):
            self._assert_idle()
            result = self.settings()
            result.update({key: value for key, value in read_json(self.manager_home / "config/settings.json", {}).items() if key.startswith("_")})
            result.update(values)
            result.pop("password_enabled", None)
            atomic_json(self.manager_home / "config/settings.json", result)
            if set(values) & {"channel", "auto_update", "source_branch", "source_repository", "check_interval_minutes"}:
                (self.manager_home / "config/update-check.json").unlink(missing_ok=True)
        return self.settings()

    def _source_options(self, catalog=None):
        return {"allow_file": bool(catalog)}

    def current(self):
        link = self.home / "current"
        if not link.is_symlink():
            if link.exists():
                raise ManagerError("state_corrupt", "current不是受管版本链接，拒绝覆盖。", 500)
            return {}
        path = link.resolve()
        if path.parent != self.home / "releases":
            raise ManagerError("state_corrupt", "current指向受管发布目录以外。", 500)
        record = read_json(path / "installed.json", {})
        if not record.get("complete"):
            raise ManagerError("state_corrupt", "当前版本缺少完整安装记录。", 500)
        return dict(record, prefix=str(path))

    def _existing(self, current):
        if not current:
            return {}
        return _VALUES.configuration_values(root=Path(current["prefix"]) / "source", environment={"NUWAX_MANAGED_HOME": str(self.home), "NUWAX_ROBOT_PROFILE_MODE": "1"})

    def hardware(self):
        hardware = detect_hardware(self.home)
        current = self.current()
        existing = self._existing(current)
        recommendation = recommend_modules(hardware, existing, current.get("modules"))
        issues = []
        if platform.system() != "Linux":
            issues.append("当前管理器安装入口需要Linux。")
        if not (3, 8) <= sys.version_info[:2] < (3, 13):
            issues.append("管理器需要Python 3.8—3.12，业务依赖按设备实际解释器准备。")
        if hardware.architecture not in ("x86_64", "aarch64"):
            issues.append("当前处理器架构没有正式安装支持。")
        return {"hardware": hardware.to_dict(), "recommendation": recommendation.to_dict(), "supported": not issues, "issues": issues}

    def modules(self):
        current = self.current()
        selected = current.get("modules", [])
        return {"modules": [dict(key=item.key, title=item.title, description=item.description,
                                 requires=list(item.requires), conflicts=list(item.conflicts), installed=item.key in selected)
                            for item in MODULES.values()], "selected": selected, "robot": current.get("robot", "customrobot"), "robots": ["customrobot", "go2"], "robot_labels": {"customrobot": "标准机器人接入", "go2": "宇树Go2"}}

    def catalog(self, refresh=False, local=None):
        settings = self.settings()
        if not local and settings["source_mode"] == "git":
            from .sources import discover
            source = discover(self, refresh=refresh)
            return {"schema_version": 1, "product": "neh", "source": source, "releases": [],
                    "manager": {"version": source["version"], "revision": source["revision"]}}
        url = str(Path(local).resolve()) if local else settings["feed_url"]
        if not url:
            return {"schema_version": 1, "product": "neh", "releases": []}
        cache = read_json(self.home / "cache/catalog.json", {})
        if local or refresh or cache.get("url") != url or cache.get("channel") != settings["channel"]:
            options = self._source_options(local)
            resolved, github = (url, False) if local else github_catalog_url(url, settings["channel"])
            catalog = load_catalog(resolved, **options)
            catalog["_github_channel"] = github
            if not local:
                atomic_json(self.home / "cache/catalog.json", {"url": url, "channel": settings["channel"], "catalog": catalog})
            return catalog
        return cache.get("catalog", {"releases": []})

    @staticmethod
    def _ordered_releases(catalog):
        from functools import cmp_to_key
        def compare(first, second):
            return compare_versions(first["version"], second["version"], first.get("published_at"), second.get("published_at")) or 0
        return sorted(catalog["releases"], key=cmp_to_key(compare), reverse=True)

    def releases(self, refresh=False, local=None):
        settings = self.settings()
        result = {"configured": bool(local or settings["feed_url"]), "channel": settings["channel"], "feed_url": settings["feed_url"], "releases": []}
        try:
            hardware = self.hardware()["hardware"]
            catalog = self.catalog(refresh=refresh, local=local)
            if catalog.get("source"):
                source = catalog["source"]
                result.update(source_mode="git", source_repository=source["repository"], source_branch=source["branch"], source_revision=source["revision"])
                result["releases"] = [{"version": source["version"], "revision": source["revision"], "channel": settings["channel"],
                    "notes": "设备下载固定源码并按程序指纹复用、下载或构建", "published_at": source.get("channel_record", {}).get("published_at", ""), "compatible": self.hardware()["supported"],
                    "display_version": source.get("channel_record", {}).get("version", source["branch"] + "@" + source["revision"][:8]),
                    "reason": "", "modules": list(MODULES)}]
                return result
            for release in self._ordered_releases(catalog):
                if release["channel"] != settings["channel"] and not (catalog.get("_github_channel") and settings["channel"] == "preview"):
                    continue
                artifacts = [a for a in release["artifacts"] if a.get("kind") == "runtime"]
                compatible = [a for a in artifacts if not platform_issues(a["platform"], hardware)]
                result["releases"].append({key: release.get(key, "") for key in ("version", "channel", "notes", "published_at")})
                result["releases"][-1].update(compatible=bool(compatible), reason="" if compatible else "没有与当前系统匹配的运行包",
                                              modules=sorted({m for a in compatible for m in a["modules"]}))
        except ManagerError as exc:
            if exc.code == "channel_pending":
                result.update(pending=True, message=exc.message)
            else:
                result["error"] = exc.message
        return result

    def _job_path(self, job_id):
        return self.manager_home / "jobs" / (identifier(job_id) + ".json")

    def job(self, job_id):
        job = read_json(self._job_path(job_id))
        if job is None:
            raise ManagerError("not_found", "任务不存在。", 404)
        return {key: value for key, value in job.items() if not key.startswith("_")}

    def jobs(self):
        result = [self.job(path.stem) for path in (self.manager_home / "jobs").glob("*.json")]
        return sorted(result, key=lambda item: item["created_at"], reverse=True)[:200]

    def active_job(self):
        active = read_json(self.manager_home / "config/active-job.json", {})
        if not active:
            return None
        job = self.job(active["id"])
        return job if job["status"] not in TERMINAL else None

    def status(self):
        self.recover_if_needed()
        current = self.current()
        history = read_json(self.home / "config/history.json", {})
        releases = self.releases()
        available = next((item for item in releases["releases"] if item["compatible"]), None)
        current_published = current.get("published_at") or next((item.get("published_at") for item in releases["releases"] if item["version"] == current.get("version")), None)
        relation = compare_versions(available["version"], current.get("version"), available.get("published_at"), current_published) if available and current else None
        source_mode = self.settings()["source_mode"] == "git"
        relation_name = {1: "newer", 0: "same", -1: "older"}.get(relation, "unknown")
        if source_mode and available and current:
            from .versioning import revision_relation, source_tree_unchanged
            try:
                relation_name = revision_relation(self, self.catalog()["source"], current.get("revision"))
                if relation_name == "newer" and source_tree_unchanged(self, self.catalog()["source"], current.get("revision")):
                    relation_name = "same"
            except (OSError, RuntimeError) as exc:
                relation_name = "unknown"
                releases["error"] = "无法核对完整提交历史：" + str(exc)
            relation = {"newer": 1, "same": 0, "older": -1}.get(relation_name)
        installed_manager = MANAGER_VERSION
        try:
            latest_manager = self.catalog().get("manager", {}).get("version")
        except ManagerError:
            latest_manager = None
        manager_relation = None if MANAGER_DEVELOPMENT else compare_versions(latest_manager, installed_manager)
        if source_mode and available:
            from .versioning import revision_relation, source_tree_unchanged
            try:
                manager_relation_name = revision_relation(self, self.catalog()["source"], MANAGER_REVISION)
                if manager_relation_name == "newer" and source_tree_unchanged(self, self.catalog()["source"], MANAGER_REVISION):
                    manager_relation_name = "same"
            except (OSError, RuntimeError):
                manager_relation_name = "unknown"
            manager_relation = {"newer": 1, "same": 0, "older": -1}.get(manager_relation_name)
        else:
            manager_relation_name = {1: "newer", 0: "same", -1: "older"}.get(manager_relation, "unknown")
        failed = read_json(self.manager_home / "config/failed-updates.json", {}).get(available.get("revision") if available else "", {})
        channel_record = self.catalog().get("source", {}).get("channel_record", {}) if source_mode and available else {}
        installation = {"installed": bool(current), "install_dir": str(self.home), "version": current.get("version"), "modules": current.get("modules", []),
                        "robot": current.get("robot", "customrobot"), "previous_version": history.get("previous_version"), "source": current.get("source"), "revision": current.get("revision", "")}
        location = self.install_location()
        installation.update(manager_dir=location["manager_dir"], parent_dir=location["parent_dir"], user_home=location["user_home"],
                            location_editable=location["editable"], location_reason=location["reason"])
        return {"product": "NEH", "manager_version": MANAGER_VERSION, "manager_revision": MANAGER_REVISION, "manager_development": MANAGER_DEVELOPMENT, "instance_id": self.instance_id, "installation": installation,
                "runtime": self.runtime.status(), "active_job": self.active_job(),
                "update": {"available": bool(available and current and relation == 1 and not failed), "failed": failed or None,
                           "pending": bool(releases.get("pending")), "message": releases.get("message"),
                           "requires_confirmation": channel_record.get("requires_confirmation", False), "display_version": available.get("display_version") if available else None,
                           "check": read_json(self.manager_home / "config/update-check.json", {}),
                           "version": available["version"] if available else None, "revision": available.get("revision") if available else None, "error": releases.get("error"), "manager_available": bool(latest_manager and manager_relation == 1 and not failed), "manager_version": latest_manager, "manager_switch_available": bool(latest_manager and manager_relation == 1 and not failed), "manager_relation": manager_relation_name, "relation": relation_name},
                "password_enabled": self.settings()["password_enabled"], "platform_supported": self.hardware()["supported"]}

    def _plan_source(self, plan, request, local_request):
        from .sources import discover
        source = discover(self, refresh=True, request=request, local_request=local_request)
        if request.get("version") and request["version"] != source["version"]:
            plan["blockers"].append("源码分支提交已变化，请刷新版本或明确指定完整source_revision。")
        plan.update(version=source["version"], source_revision=source["revision"], source_repository=source["repository"],
                    source_branch=source["branch"], _source=source, download_bytes=None,
                    download_breakdown={"application": None, "dependencies": None, "models": None})
        current = self.current()
        channel_record = source.get("channel_record", {})
        if request.get("automatic") and channel_record.get("requires_confirmation"):
            plan["blockers"].append("此版本需要人工确认，自动更新只提醒。")
        if (channel_record.get("minimum_manager_version") and plan["action"] != "update-manager"
                and not request.get("one_step") and MANAGER_VERSION != channel_record["minimum_manager_version"]):
            plan["blockers"].append("此版本需要先更新管理程序，请执行neh update或先应用管理程序更新计划。")
        if plan["action"] in ("update", "update-manager") and not request.get("source_revision"):
            from .versioning import revision_relation
            installed = MANAGER_REVISION if plan["action"] == "update-manager" else current.get("revision")
            if installed:
                try:
                    relation = revision_relation(self, source, installed)
                except (OSError, RuntimeError) as exc:
                    relation = "unknown"
                    plan["warnings"].append("无法核对完整提交历史：" + str(exc))
                if relation not in ("newer", "same"):
                    plan["blockers"].append({"older": "当前版本领先通道", "diverged": "当前版本与通道已分叉"}.get(relation, "无法确认更新方向") + "；如需切换，请显式指定完整source_revision。")
            if read_json(self.manager_home / "config/failed-updates.json", {}).get(source["revision"]) and not request.get("retry"):
                plan["blockers"].append("此版本此前更新失败；请等待新版本或使用--retry明确重试。")
        from .versioning import source_tree_unchanged
        try:
            same_tree = source_tree_unchanged(self, source, current.get("revision")) if current else False
        except (OSError, RuntimeError):
            same_tree = False
        unchanged = same_tree and set(current.get("modules", [])) == set(plan["modules"]) and current.get("robot") == plan["robot"] and not request.get("configuration")
        plan["_source_noop"] = MANAGER_REVISION == source["revision"] if plan["action"] == "update-manager" else (plan["action"] in ("update", "modules") and unchanged)
        if plan["_source_noop"]:
            plan["requires_stop"] = False
            plan["steps"].append({"title": "所选源码提交与模块已经生效，无需重新构建"})
            return
        plan["warnings"].append("源码传输大小由Git增量协商决定，暂无法预报；依赖和原生构建缓存按实际输入复用。")
        plan["steps"].append({"title": "获取计划固定的源码提交"})
        if plan["action"] == "update-manager":
            plan["steps"].extend([{"title": "在设备本地组装管理程序"}, {"title": "切换管理程序并重启网页进程"}])
        else:
            # 与源码构建的metadata使用同一模块配方；仅查询本机包状态，不安装或提权。
            from scripts.neh_system_helper import dependency_plan
            try:
                packages = list(module_system_packages(plan["modules"], plan["robot"]))
                if set(plan["modules"]) & {"geometry", "slam2d", "fastlio2", "pointlio", "navigation", "follow"}:
                    packages.append("patchelf")
                dependencies = dependency_plan({"packages": list(dict.fromkeys(packages))})
                plan["system_dependencies"] = dependencies
                if dependencies["missing"] and not Path("/usr/local/libexec/neh-system-helper").is_file():
                    plan["blockers"].append("缺少系统依赖（" + "、".join(dependencies["missing"]) +
                        "），且系统助手尚未安装。请由设备管理员完成引导器的系统集成步骤，或按本机包管理器安装这些依赖后重新生成计划。")
            except (ValueError, RuntimeError, OSError, subprocess.SubprocessError) as exc:
                plan["blockers"].append("系统依赖无法预检：" + str(exc))
            plan["steps"].extend([{"title": "准备并复用所选模块依赖"}, {"title": "增量构建并验证独立候选版本"}, {"title": "保存配置并切换版本"}])

    def _fingerprint(self):
        current = self.current()
        values = {"install_dir": str(self.home), "current": current.get("prefix"), "running": self.runtime.status()["running"], "settings": self.settings()}
        active = self.home / "data/settings/active.env"
        if current and active.is_file():
            values["config"] = self.configuration.request(Path(current["prefix"]), "revision")
        # 绑定共同拥有方的当前快照令牌及明确状态，不散列整个配置文件。
        return values

    def plan(self, request, *, local_catalog=None, local_request=False):
        self.recover_if_needed()
        if not isinstance(request, dict) or request.get("action") not in ACTIONS:
            raise ManagerError("invalid_action", "请选择受支持的管理操作。")
        action = request["action"]
        if request.get("source_id") and action == "migrate" and not local_request:
            sources = self.migration_sources()["sources"]
            source = next((item for item in sources if item["id"] == request["source_id"]), None)
            if not source:
                raise ManagerError("source_missing", "未发现该受认可安装源，请刷新迁入检测。", 409)
            request = dict(request, source_path=source["source"])
            local_request = True
        if (request.get("source_path") or local_catalog) and not local_request:
            raise ManagerError("local_only", "本机路径和迁入操作只能通过设备命令行提交。", 403)
        if action == "migrate" and not local_request:
            raise ManagerError("local_only", "现有部署迁入需要在设备本地命令行执行。", 403)
        current, settings = self.current(), self.settings()
        hardware = self.hardware()
        try:
            modules = list(resolve_modules(request.get("modules", current.get("modules", hardware["recommendation"]["modules"]))))
        except (ModuleError, TypeError) as exc:
            raise ManagerError("invalid_modules", str(exc)) from exc
        robot = request.get("robot", current.get("robot", hardware["recommendation"]["robot"]))
        if robot not in ("customrobot", "go2"):
            raise ManagerError("robot_unsupported", "当前完整启动器支持通用机器人和Go2；其它驱动仍待适配。")
        blockers, warnings, steps = [], [], []
        if action in ("install", "migrate"):
            configuration = request.get("configuration", {})
            if not isinstance(configuration, dict):
                raise ManagerError("invalid_config", "预配置必须是对象。")
            if not configuration.get("NUWAX_GATEWAY_PUBLIC_BASE_URL"):
                server = read_json(self.manager_home / "config/server.json", {})
                if process_alive(server.get("process")) and server.get("public_url"):
                    request = dict(request, configuration=dict(configuration, NUWAX_GATEWAY_PUBLIC_BASE_URL=server["public_url"]))
                warnings.append("命令行首次安装请确认公开制品地址；可通过configuration.NUWAX_GATEWAY_PUBLIC_BASE_URL指定远端可达的管理台地址。")
        if action in ("install", "migrate") and current:
            blockers.append("已有安装，请使用更新或模块调整。")
        if action in ("update", "modules", "rollback", "start", "stop", "restart") and not current:
            blockers.append("尚未安装业务版本。")
        if self.active_job():
            blockers.append("已有管理任务尚未结束。")
        running = self.runtime.status()["running"]
        requires_stop = running and action not in ("start", "update-manager")
        plan = {"id": uuid.uuid4().hex, "action": action, "install_dir": str(self.home), "version": request.get("version"), "modules": modules, "robot": robot,
                "steps": steps, "warnings": warnings, "blockers": blockers, "download_bytes": 0, "requires_stop": requires_stop,
                "_fingerprint": self._fingerprint(), "_created": time.time(), "_request": request,
                "_local_catalog": str(Path(local_catalog).resolve()) if local_catalog else None, "download_breakdown": {"application": 0, "dependencies": None, "models": None}}
        if action == "update-manager" and not local_catalog and settings["source_mode"] == "git":
            self._plan_source(plan, request, local_request)
        elif action == "update-manager":
            from .self_update import manager_metadata
            metadata = manager_metadata(self, self.catalog(refresh=True, local=local_catalog), local_catalog)
            if request.get("version") and request["version"] != metadata["version"]:
                blockers.append("版本目录当前没有请求的管理程序版本。")
            if MANAGER_DEVELOPMENT:
                warnings.append("当前运行源码开发入口，此操作将切换到选定的管理发布版本。")
            elif compare_versions(metadata["version"], MANAGER_VERSION) == -1:
                warnings.append("选定管理版本早于当前版本，将执行降级；业务版本保持不变。")
            plan.update(version=metadata["version"], download_bytes=metadata["size"], _manager=metadata, download_breakdown={"application": metadata["size"], "dependencies": 0, "models": 0})
            steps.extend([{"title": "下载并校验独立管理程序"}, {"title": "切换管理程序并重启网页进程"}])
        elif action in ("install", "update", "modules", "migrate"):
            blockers.extend(hardware["issues"])
            if action == "migrate":
                plan["_migration"] = self._migration_info(request.get("source_path", ""))
                if "robot" not in request:
                    robot = plan["_migration"].get("robot", robot)
                    plan["robot"] = robot
                if "modules" not in request and plan["_migration"].get("modules"):
                    modules = list(resolve_modules(plan["_migration"]["modules"]))
                    plan["modules"] = modules
                if robot not in ("customrobot", "go2"):
                    blockers.append("迁入源的机器人类型没有完整产品启动支持。")
                warnings.append("迁入复制现有数据和配置；源安装保留，旧自动启动入口停用。")
                if plan["_migration"]["running"] and not request.get("stop_running"):
                    blockers.append("迁入源正在运行，请明确允许停止原拥有方。")
                plan["requires_stop"] = plan["requires_stop"] or plan["_migration"]["running"]
            if not local_catalog and settings["source_mode"] == "git":
                self._plan_source(plan, request, local_request)
            else:
                catalog = self.catalog(refresh=True, local=local_catalog)
                selected = None
                mismatches = []
                for release in self._ordered_releases(catalog):
                    if (release["channel"] != settings["channel"] and not (catalog.get("_github_channel") and settings["channel"] == "preview")) or (request.get("version") and release["version"] != request["version"]):
                        continue
                    for artifact in release["artifacts"]:
                        if artifact.get("kind") != "runtime" or artifact.get("robot") != robot or set(artifact.get("modules", [])) != set(modules):
                            continue
                        issues = platform_issues(artifact.get("platform", {}), hardware["hardware"])
                        if issues:
                            mismatches.extend(issues)
                            continue
                        selected = dict(artifact, version=release["version"], revision=release.get("revision", ""), published_at=release.get("published_at"))
                        break
                    if selected:
                        break
                if not selected:
                    blockers.append("没有与版本、模块集合、机器人及目标环境匹配的发布包。" + "；".join(dict.fromkeys(mismatches)))
                else:
                    if current and compare_versions(selected["version"], current["version"], selected.get("published_at"), current.get("published_at")) == -1:
                        warnings.append("选定业务版本早于当前版本，将执行降级；持久数据与用户配置保持。")
                    plan.update(version=selected["version"], download_bytes=selected["size"], _artifact=selected)
                    plan["download_breakdown"] = {"application": selected["size"], "dependencies": selected.get("dependency_bytes"), "models": selected.get("model_bytes")}
                    if selected.get("dependency_bytes") is not None and selected.get("model_bytes") is not None:
                        plan["download_bytes"] = selected["size"] + selected["dependency_bytes"] + selected["model_bytes"]
                        warnings.append("下载量是首次安装的上限；已校验的依赖缓存与模型可复用。")
                    else:
                        plan["download_bytes"] = None
                        warnings.append("依赖与模型下载量将在校验轻量业务包后确定；应用包大小不代表总下载量。")
                    if hardware["hardware"]["free_disk_bytes"] < selected["size"] * 3 + 512 * 1024 ** 2:
                        blockers.append("可用磁盘空间不足以安全准备候选版本。")
                steps.extend([{"title": "下载并验证发布包"}, {"title": "按固定配方下载依赖与模型并准备运行环境"}, {"title": "保存配置并切换版本"}])
            if robot == "go2":
                warnings.append("Go2首次安装按官方流程编译Python接口适配并缓存；DDS原生库直接使用宇树官方预编译文件，后续安装复用已校验缓存。")
        elif action == "rollback":
            history = read_json(self.home / "config/history.json", {})
            previous = history.get("previous")
            if not previous or not (self.home / "releases" / identifier(previous) / "installed.json").is_file():
                blockers.append("没有可恢复的上一版本。")
            else:
                old = read_json(self.home / "releases" / previous / "installed.json")
                plan.update(version=old["version"], modules=old["modules"], robot=old["robot"], _previous=previous)
            steps.append({"title": "恢复完整上一版本"})
        else:
            plan["version"] = current.get("version")
        if plan["requires_stop"] and not request.get("stop_running") and action not in ("stop", "restart", "update"):
            blockers.append("业务正在运行；确认停止当前任务后再应用（stop_running=true）。")
        if plan["requires_stop"] or action in ("stop", "restart"):
            steps.append({"title": "通过原拥有方停止业务并等待退出"})
        if not plan.get("_source_noop") and action not in ("stop", "update-manager") and (settings["auto_start"] or running or action in ("start", "restart")):
            steps.append({"title": "调用原启动器并验证网关健康接口"})
            warnings.append("启动会按机器人配置连接设备；网关健康不代表建图或运动功能已经就绪。")
        atomic_json(self.manager_home / "plans" / (plan["id"] + ".json"), plan)
        return self._public_plan(plan)

    @staticmethod
    def _public_plan(plan):
        return {key: value for key, value in plan.items() if not key.startswith("_")}

    def _assert_idle(self):
        self._assert_location()
        if self.active_job():
            raise ManagerError("busy", "已有管理任务正在执行。", 409)

    def submit(self, plan_id):
        with file_lock(self.lock_path):
            self._assert_idle()
            plan = read_json(self.manager_home / "plans" / (identifier(plan_id) + ".json"))
            if not plan:
                raise ManagerError("plan_missing", "安装计划不存在，请重新生成。", 404)
            if plan["blockers"]:
                raise ManagerError("plan_blocked", "计划尚有未解决条件。", 409, plan["blockers"])
            if time.time() - plan["_created"] > 3600 or plan["_fingerprint"] != self._fingerprint():
                raise ManagerError("plan_stale", "设备状态或配置已变化，请重新生成计划。", 409)
            job_id = uuid.uuid4().hex
            job = {"id": job_id, "action": plan["action"], "status": "queued", "stage": "queued", "progress": 0,
                   "message": "任务已持久保存", "created_at": timestamp(), "updated_at": timestamp(), "log": [],
                   "_plan": plan, "_queued_at": time.time()}
            atomic_json(self._job_path(job_id), job)
            atomic_json(self.manager_home / "config/active-job.json", {"id": job_id})
            package_root = PACKAGE_ROOT
            environment = clean_environment()
            environment["PYTHONPATH"] = str(package_root)
            try:
                descriptor = os.open(str(self.manager_home / "logs" / (job_id + ".log")), os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o600)
                with os.fdopen(descriptor, "ab") as log:
                    process = subprocess.Popen([sys.executable, "-m", "scripts.device_manager", "--home", str(self.manager_home), "_worker", job_id],
                                               cwd=package_root, env=environment, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                               start_new_session=True)
                job["_process"] = process_identity(process.pid)
                atomic_json(self._job_path(job_id), job)
            except OSError as exc:
                job.update(status="failed", error={"code": "worker_start", "message": "后台工作进程无法启动。"})
                atomic_json(self._job_path(job_id), job)
                raise ManagerError("worker_start", "后台工作进程无法启动。", 500) from exc
        return self.job(job_id)

    def cancel(self, job_id):
        job = self.job(job_id)
        if job["status"] in TERMINAL:
            return job
        atomic_write(self.manager_home / "jobs" / (identifier(job_id) + ".cancel"), b"cancel\n")
        return dict(job, message="已请求取消，后台正在等待安全恢复边界。")

    def _cancelled(self, job_id):
        if (self.manager_home / "jobs" / (job_id + ".cancel")).exists():
            raise Cancelled()

    def _progress(self, job, stage, progress, message):
        job.update(stage=stage, progress=progress, message=message, updated_at=timestamp())
        if not job["log"] or job["log"][-1]["message"] != message:
            job["log"].append({"time": timestamp(), "stage": stage, "message": message})
            job["log"] = job["log"][-300:]
        atomic_json(self._job_path(job["id"]), job)

    def run_job(self, job_id):
        job_id = identifier(job_id)
        saved = {key: os.environ.get(key) for key in ("NEH_MANAGER_WORKER_HOME", "NEH_MANAGER_WORKER_JOB")}
        os.environ["NEH_MANAGER_WORKER_HOME"] = str(self.manager_home)
        os.environ["NEH_MANAGER_WORKER_JOB"] = job_id
        try:
            return self._run_job_locked(job_id)
        finally:
            for key, value in saved.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

    def _run_job_locked(self, job_id):
        with file_lock(self.lock_path, blocking=True):
            job = read_json(self._job_path(job_id))
            if not job or job["status"] in TERMINAL:
                return
            job.update(status="running", _process=process_identity(os.getpid()))
            if job["_plan"].get("_request", {}).get("automatic"):
                os.nice(10)
                if shutil.which("ionice"):
                    subprocess.run(["ionice", "-c", "3", "-p", str(os.getpid())], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._progress(job, "preflight", 2, "正在复核持久计划")
            try:
                self._cancelled(job_id)
                self._assert_location()
                if job.get("_manager_adopted"):
                    from .sources import normalized_settings
                    expected = job["_plan"]["_fingerprint"]
                    migrated = dict(expected, settings=normalized_settings(expected["settings"]))
                    # 只迁移新管理程序定义的默认值；真实配置、拥有方或版本变化仍须拒绝。
                    if migrated == self._fingerprint():
                        job["_plan"]["_fingerprint"] = migrated
                        atomic_json(self._job_path(job_id), job)
                if job["_plan"]["_fingerprint"] != self._fingerprint():
                    raise ManagerError("plan_stale", "任务执行前设备状态已变化，请重新生成计划。", 409)
                result = self._execute(job)
                job.update(status="succeeded", result=result)
                self._progress(job, "complete", 100, "管理操作完成")
                if job["action"] == "update-manager" and result.get("restart_required", True):
                    atomic_json(self.manager_home / "config/restart-manager.json", {"instance_id": self.instance_id, "job_id": job["id"], "version": result["manager_version"]})
            except BaseException as exc:
                if not isinstance(exc, ManagerError):
                    import traceback
                    atomic_write(self.manager_home / "logs" / (job_id + ".traceback.log"), traceback.format_exc().encode())
                error = exc if isinstance(exc, ManagerError) else ManagerError("operation_failed", "管理操作失败：" + type(exc).__name__, 500)
                try:
                    self._restore(job)
                except BaseException as recovery:
                    error = ManagerError("recovery_failed", error.message + "；自动恢复未完成，需要在设备本地检查原拥有方。", 500,
                                         {"recovery": getattr(recovery, "code", type(recovery).__name__)})
                job.update(status="cancelled" if isinstance(exc, Cancelled) and error.code != "recovery_failed" else "failed", error=error.as_dict())
                revision = job["_plan"].get("source_revision")
                if revision and job["action"] == "update" and not isinstance(exc, Cancelled):
                    failures = read_json(self.manager_home / "config/failed-updates.json", {})
                    failures[revision] = {"time": timestamp(), "error": error.as_dict(), "job_id": job_id}
                    atomic_json(self.manager_home / "config/failed-updates.json", failures)
                self._progress(job, "failed", job["progress"], error.message)
            if (job["status"] == "succeeded" and job["action"] in {"install", "update", "modules", "migrate", "rollback", "update-manager"}
                    and not job["result"].get("unchanged")):
                # 安装已经提交；存储保留失败不能触发安装回滚。
                try:
                    from .storage import prune_storage
                    job["result"]["storage"] = prune_storage(self.manager_home)
                except Exception as exc:
                    job["result"]["storage"] = {"removed": [], "warnings": [{"error": type(exc).__name__}]}
                atomic_json(self._job_path(job_id), job)

    def recover_if_needed(self):
        selected = self.selected()
        if selected is not self:
            return selected.recover_if_needed()
        active = read_json(self.manager_home / "config/active-job.json", {})
        if not active:
            return
        job = read_json(self._job_path(active["id"]), {})
        if not job or job.get("status") in TERMINAL or process_alive(job.get("_process")):
            return
        if job.get("status") == "queued" and time.time() - job.get("_queued_at", 0) < 15:
            return
        try:
            with file_lock(self.lock_path):
                self._assert_location()
                job = read_json(self._job_path(active["id"]), {})
                if job.get("status") in TERMINAL or process_alive(job.get("_process")):
                    return
                try:
                    job["status"] = "running"
                    self._progress(job, "recovering", job.get("progress", 0), "工作进程中断，正在恢复上一版本状态")
                    self._restore(job)
                    error = ManagerError("interrupted", "后台进程中断；已恢复切换前状态，请重新提交计划。", 500)
                except BaseException as exc:
                    error = ManagerError("recovery_failed", "中断后恢复未完成，请检查设备原拥有方。", 500, {"recovery": getattr(exc, "code", type(exc).__name__)})
                job.update(status="interrupted", error=error.as_dict())
                self._progress(job, "interrupted", job.get("progress", 0), error.message)
        except ManagerError as exc:
            if exc.code != "busy":
                raise

    def _journal(self, job, **values):
        job.setdefault("_journal", {}).update(values)
        atomic_json(self._job_path(job["id"]), job)

    def _switch(self, prefix):
        self.configuration.close()
        link = self.home / "current"
        temporary = self.home / (".current-" + uuid.uuid4().hex)
        try:
            if prefix:
                temporary.symlink_to(os.path.relpath(prefix, self.home))
                os.replace(str(temporary), str(link))
            else:
                link.unlink(missing_ok=True)
            fsync_directory(self.home)
        finally:
            temporary.unlink(missing_ok=True)

    def _restore(self, job):
        self._assert_location()
        if job.get("_plan", {}).get("install_dir", str(self.home)) != str(self.home):
            raise ManagerError("install_location_changed", "任务的安装位置与当前选择不一致，拒绝跨目录恢复。", 409)
        command_state = self.manager_home / "jobs" / (identifier(job["id"]) + ".command")
        command = read_json(command_state)
        if process_alive(command):
            import signal
            try:
                os.killpg(command["pid"], signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 10
            while process_alive(command) and time.monotonic() < deadline:
                time.sleep(0.1)
            if process_alive(command):
                try:
                    os.killpg(command["pid"], signal.SIGKILL)
                except ProcessLookupError:
                    pass
        command_state.unlink(missing_ok=True)
        journal = job.get("_journal", {})
        if not journal or journal.get("committed"):
            return
        if journal.get("manager_switch"):
            from scripts.neh_bootstrap import switch_link
            link = self.manager_home / "manager/current"
            if journal.get("manager_previous"):
                switch_link(link, Path(journal["manager_previous"]))
            else:
                link.unlink(missing_ok=True)
            self._journal(job, restored=True, committed=True)
            return
        previous = Path(journal["previous"]) if journal.get("previous") else None
        candidate = Path(journal["candidate"]) if journal.get("candidate") else None
        if journal.get("switch_intent"):
            # 兼容中断的旧start日志：未停止过的既有拥有方不属于本次启动事务。
            kept_owner = job.get("action") == "start" and candidate == previous and journal.get("was_running") and not journal.get("stop_intent")
            if candidate and self.runtime.status()["running"]:
                runtime_record = read_json(self.home / "config/runtime.json", {})
                owner = Path(runtime_record["prefix"]) if runtime_record.get("prefix") else candidate
                if owner == candidate:
                    if not kept_owner:
                        self.runtime.stop(candidate)
                elif owner != previous:
                    raise ManagerError("owner_identity", "恢复时发现不属于本事务的业务拥有方，拒绝停止。", 409)
            self._restore_profiles(journal)
            self._switch(previous)
        for relative in reversed(journal.get("migration_paths", [])):
            original = self.home / relative
            retained = self.home / "cache" / ("migration-" + job["id"]) / relative
            if original.exists() or original.is_symlink():
                retained.parent.mkdir(parents=True, exist_ok=True)
                os.replace(str(original), str(retained))
        if journal.get("legacy_stopped"):
            if journal.get("legacy_systemd"):
                system_helper("restore-legacy")
            elif journal.get("legacy_running"):
                self._start_legacy(journal["legacy_launcher"], journal["legacy_source"])
        if journal.get("stop_intent") and journal.get("was_running") and previous and not self.runtime.status()["running"]:
            self.runtime.start(previous)
        if candidate and candidate != previous and candidate.name.endswith("-" + job["id"][:8]) and candidate.parent == self.home / "releases" and candidate.exists():
            shutil.rmtree(candidate)
        self._journal(job, restored=True, committed=True)

    def _restore_profiles(self, journal):
        for item in journal.get("profiles", []):
            path = self.home / item["path"]
            if item["kind"] == "missing":
                path.unlink(missing_ok=True)
            elif item["kind"] == "link":
                path.unlink(missing_ok=True)
                path.symlink_to(item["value"])
            else:
                atomic_write(path, base64.b64decode(item["value"]))

    def _profile_name(self, robot):
        selector = self.home / "config/profiles" / robot / ".active"
        active = self.home / "data/settings/active.env"
        if active.is_symlink():
            target = active.resolve()
            if target.parent == self.home / "config/profiles" / robot and target.suffix == ".env":
                return target.stem
        name = selector.read_text().strip() if selector.is_file() else "default"
        if not re.fullmatch(r"[\w\-\u4e00-\u9fff][\w\- \u4e00-\u9fff]{0,63}", name) or name in (".", ".."):
            raise ManagerError("invalid_profile", "当前配置方案名称不合法。")
        return name

    def _snapshot_profiles(self, robot):
        result = []
        name = self._profile_name(robot)
        paths = [self.home / "data/settings/active.env", self.home / "config/profiles" / robot / ".active",
                 self.home / "config/profiles" / robot / (name + ".env")]
        for path in paths:
            if path.is_symlink():
                kind, value = "link", os.readlink(str(path))
            elif path.is_file():
                kind, value = "file", base64.b64encode(path.read_bytes()).decode()
            else:
                kind, value = "missing", ""
            result.append({"path": str(path.relative_to(self.home)), "kind": kind, "value": value})
        return result

    def _activate_profile(self, candidate, robot):
        directory = self.home / "config/profiles" / robot
        directory.mkdir(parents=True, exist_ok=True)
        name = self._profile_name(robot)
        values = read_settings(candidate / "settings.env")
        rendered = "\n".join(_VALUES.env_assignment(key, str(value)) for key, value in sorted(values.items()) if str(value).strip()) + "\n"
        atomic_write(directory / (name + ".env"), rendered.encode())
        atomic_write(directory / ".active", (name + "\n").encode())
        settings = self.home / "data/settings"
        settings.mkdir(parents=True, exist_ok=True)
        active = settings / "active.env"
        temporary = settings / (".active-" + uuid.uuid4().hex)
        temporary.symlink_to(directory / (name + ".env"))
        os.replace(str(temporary), str(active))
        fsync_directory(settings)

    def _execute(self, job):
        plan, action = job["_plan"], job["action"]
        if action == "update-manager":
            from .self_update import apply_manager
            return apply_manager(self, job)
        current = self.current()
        if action == "update" and plan.get("_request", {}).get("one_step") and not job.get("_manager_adopted"):
            from .update_flow import adopt_manager
            adopt_manager(self, job)
        if plan.get("_source_noop"):
            self._journal(job, committed=True)
            return {"version": current.get("version"), "revision": current.get("revision"), "unchanged": True, "runtime": self.runtime.status()}
        previous = Path(current["prefix"]) if current else None
        runtime_state = self.runtime.status()
        was_running = runtime_state["running"]
        self._journal(job, previous=str(previous) if previous else None, was_running=was_running)
        if action in ("start", "stop", "restart"):
            if action == "start" and was_running:
                if runtime_state["healthy"]:
                    self._journal(job, committed=True)
                    return {"version": current.get("version"), "unchanged": True, "runtime": runtime_state}
                raise ManagerError("already_running", "业务拥有方已运行但健康接口尚未响应，请检查原服务；不会重复启动或停止。", 409)
            if action in ("stop", "restart"):
                self._journal(job, stop_intent=True)
                self._progress(job, "stopping", 25, "通过原拥有方停止业务")
                self.runtime.stop(previous)
            self._cancelled(job["id"])
            if action in ("start", "restart"):
                self._journal(job, candidate=str(previous), switch_intent=True)
                self._progress(job, "starting", 70, "通过原拥有方启动业务并读取健康接口")
                self.runtime.start(previous)
            self._cancelled(job["id"])
            self._journal(job, committed=True)
            return {"version": current.get("version"), "runtime": self.runtime.status()}
        if action == "rollback":
            candidate = self.home / "releases" / plan["_previous"]
            self._configure_candidate(candidate, plan, self._existing(current))
        else:
            candidate = self._prepare(job)
        self._cancelled(job["id"])
        if plan.get("_fingerprint") and plan["_fingerprint"] != self._fingerprint():
            raise ManagerError("plan_stale", "准备期间设备配置发生变化，保留当前版本，请重新生成计划。", 409)
        profiles = self._snapshot_profiles(plan["robot"])
        self._journal(job, candidate=str(candidate), profiles=profiles)
        if action == "migrate":
            self._stop_migration(job)
            self._copy_migration(job)
        if action == "update":
            from .update_flow import wait_until_idle
            wait_until_idle(self, job)
            if plan.get("_fingerprint") and plan["_fingerprint"] != self._fingerprint():
                raise ManagerError("plan_stale", "等待空闲期间配置发生变化，请重新生成更新计划。", 409)
        if was_running:
            self._journal(job, stop_intent=True)
            self._progress(job, "stopping", 75, "等待原拥有方退出后切换")
            self.runtime.stop(previous)
        self._cancelled(job["id"])
        self._journal(job, switch_intent=True)
        self._progress(job, "switching", 82, "正在切换完整版本和持久配置")
        self._activate_profile(candidate, plan["robot"])
        self._switch(candidate)
        self._cancelled(job["id"])
        should_start = was_running or self.settings()["auto_start"]
        if should_start:
            values = read_settings(candidate / "settings.env")
            if str(values.get("NUWAX_REAL_LCM_REQUIRED", "")).lower() in ("true", "1", "yes"):
                system_helper("prepare-network", {"url": values.get("NUWAX_LCM_URL", "udpm://239.255.76.67:7667?ttl=0")})
            self._progress(job, "starting", 90, "启动原业务拥有方并验证网关接口")
            self.runtime.start(candidate)
        self._cancelled(job["id"])
        if action == "migrate" and job["_journal"].get("legacy_systemd"):
            system_helper("commit-legacy")
        atomic_json(self.home / "config/history.json", {"previous": previous.name if previous else None, "previous_version": current.get("version")})
        self._journal(job, committed=True)
        return {"version": plan["version"], "modules": plan["modules"], "robot": plan["robot"], "runtime": self.runtime.status()}

    def _prepare(self, job):
        if job["_plan"].get("_source"):
            return self._prepare_source(job)
        plan, job_id = job["_plan"], job["id"]
        artifact = plan["_artifact"]
        archive = self.home / "cache" / (artifact["sha256"] + ".tar.gz")
        self._progress(job, "downloading", 5, "下载已固定摘要的发布包")
        if not archive.is_file() or archive.stat().st_size != artifact["size"] or digest_file(archive) != artifact["sha256"].lower():
            download(artifact, archive, progress=lambda count, size: self._progress(job, "downloading", 5 + int(25 * count / size), "正在下载发布包"),
                     cancelled=lambda: self._cancelled(job_id), **self._source_options(plan.get("_local_catalog")))
        self._cancelled(job_id)
        if platform_issues(artifact["platform"], self.hardware()["hardware"]):
            raise ManagerError("platform_mismatch", "执行时目标环境与发布包不匹配。")
        candidate = self.home / "releases" / (identifier(plan["version"]) + "-" + artifact["sha256"][:12] + "-" + job_id[:8])
        self._journal(job, candidate=str(candidate))
        self._progress(job, "verifying", 32, "校验归档、清单和每个文件")
        manifest = unpack(archive, candidate, artifact, cancelled=lambda: self._cancelled(job_id))
        source = candidate / "source"
        for required in (source / "scripts/start_gateway.sh", source / "scripts/module_installer/runtime.py", candidate / "requirements.txt"):
            if not required.exists():
                raise ManagerError("package_incomplete", "发布包缺少运行所需文件：" + str(required.relative_to(candidate)))
        current = self.current()
        existing = self._existing(current)
        if plan.get("_migration"):
            existing = self._migration_configuration(Path(plan["_migration"]["source"]))
        from .downloads import MODEL_SETTINGS, prepare_dependencies, uses_default_model
        configuration = plan["_request"].get("configuration", {})
        if not isinstance(configuration, dict):
            raise ManagerError("invalid_config", "预配置必须是键值对象。")
        model_choices = dict(existing)
        for kind in ("TEXT", "IMAGE"):
            key = "NUWAX_MEMORY_" + kind + "_EMBEDDING_MODEL"
            if uses_default_model(model_choices.get(key, "")):
                remembered = existing.get("_NUWAX_INSTALLER_SAVED_" + key, "")
                if remembered:
                    model_choices[key] = remembered
        model_choices.update({key: str(value) for key, value in configuration.items() if key in MODEL_SETTINGS})
        for name in ("bin", "lib", "lib64"):
            payload = candidate / "payload" / name
            if payload.exists():
                shutil.copytree(payload, candidate / name)
        wheelhouse, model_overrides = prepare_dependencies(self, candidate, manifest, artifact, model_choices, job)
        from .downloads import apply_model_overrides
        existing = apply_model_overrides(existing, model_overrides)
        packages = manifest.get("system_packages", [])
        if not isinstance(packages, list) or any(not isinstance(item, str) for item in packages):
            raise ManagerError("manifest_invalid", "系统依赖声明不合法。")
        from scripts.neh_system_helper import dependency_plan
        try:
            dependencies = dependency_plan({"packages": packages})
        except (ValueError, RuntimeError) as exc:
            raise ManagerError("system_dependencies_invalid", str(exc), 409) from exc
        missing_packages = dependencies["missing"]
        if missing_packages:
            self._progress(job, "system_dependencies", 63, "通过固定系统助手安装运行依赖")
            system_helper("install-dependencies", {"packages": packages})
        python_version = artifact["platform"]["python"]
        python = sys.executable if python_version == "{}.{}".format(*sys.version_info[:2]) else shutil.which("python" + python_version)
        self._progress(job, "python_environment", 66, "创建本版本Python环境并安装刚下载且已校验的依赖")
        environment = clean_environment(candidate)
        run_command([python, "-m", "venv", str(candidate / "venv")], environment=environment, cancelled=lambda: self._cancelled(job_id))
        interpreter = str(candidate / "venv/bin/python")
        # Ubuntu20自带pip不认识部分现代轮子标签，先安装配方已校验的通用pip轮子。
        for record in manifest["install"]["python"]:
            if record["name"] == "pip":
                run_command([interpreter, "-m", "pip", "--isolated", "install", "--no-index", "--no-deps", str(wheelhouse / record["filename"])],
                            environment=environment, timeout=600, cancelled=lambda: self._cancelled(job_id))
        run_command([interpreter, "-m", "pip", "--isolated", "install", "--no-index", "--only-binary=:all:", "--find-links", str(wheelhouse), "-r", str(candidate / "requirements.txt")],
                    environment=environment, timeout=1800, cancelled=lambda: self._cancelled(job_id))
        if "go2" in manifest["install"]:
            from scripts.go2_dependencies import install
            from .downloads import cached_download
            # 官方归档已计入下载量；共享安装器再次核验缓存，不重复累计下载统计。
            install(candidate, self.home / "cache/go2", manifest["install"]["go2"],
                    fetch=lambda record: cached_download(self, record, filename=record["filename"], progress=lambda *_: None,
                                                         cancelled=lambda: self._cancelled(job_id))[0],
                    run=run_command, environment=environment, cancelled=lambda: self._cancelled(job_id),
                    progress=lambda message: self._progress(job, "go2_dependencies", 68, message))
        run_command([interpreter, "-m", "pip", "check"], environment=environment, cancelled=lambda: self._cancelled(job_id))
        self._progress(job, "runtime_files", 70, "核对原生产物并生成统一启动入口")
        if (source / "runtime_data").exists():
            raise ManagerError("package_private_data", "发布源码不得包含用户运行数据。")
        (source / "runtime_data").symlink_to(self.home / "data", target_is_directory=True)
        for driver in (source / "drivers").iterdir():
            if driver.is_dir():
                configs = driver / "configs"
                if configs.exists():
                    raise ManagerError("package_private_data", "发布源码不得包含机器人私有配置。")
                target = self.home / "config/profiles" / driver.name
                target.mkdir(parents=True, exist_ok=True)
                configs.symlink_to(target, target_is_directory=True)
        self._configure_candidate(candidate, plan, existing)
        from scripts.module_installer.catalog import module_imports
        imports = module_imports(plan["modules"], plan["robot"])
        code = "import importlib,json; [importlib.import_module(x) for x in json.loads(input())]"
        run_command([interpreter, "-c", code], cwd=source, environment=environment, payload=json.dumps(imports), timeout=180,
                    cancelled=lambda: self._cancelled(job_id))
        for module, binary in (("geometry", "nuwax-geometry-runtime"), ("navigation", "nuwax-nav-stack"), ("follow", "nuwax-person-follow"),
                               ("fastlio2", "fastlio2_native"), ("pointlio", "pointlio_native"), ("slam2d", "nuwax-slam-2d-worker")):
            path = candidate / "bin" / binary
            if module in plan["modules"] and (not path.is_file() or not os.access(str(path), os.X_OK)):
                raise ManagerError("native_missing", "发布包缺少所选模块的原生产物：" + binary)
        native_paths = [path for directory in (candidate / "bin", candidate / "lib", candidate / "lib64") if directory.exists() for path in directory.rglob("*") if path.is_file()]
        for binary in native_paths:
            with binary.open("rb") as handle:
                elf = handle.read(4) == b"\x7fELF"
            if not elf:
                continue
            checked = subprocess.run(["ldd", str(binary)], env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=30)
            if "not found" in checked.stdout or (checked.returncode and "not a dynamic executable" not in checked.stdout and "statically linked" not in checked.stdout):
                from .runtime import safe_diagnostic
                raise ManagerError("native_dependency_missing", "原生产物的系统依赖检查失败：" + binary.name, 500, {"diagnostic": safe_diagnostic(checked.stdout, self.home)})
        atomic_json(candidate / "installed.json", {"complete": True, "version": plan["version"], "modules": plan["modules"], "robot": plan["robot"],
                                                 "data_schema": 1, "source": "release", "artifact_sha256": artifact["sha256"], "published_at": artifact.get("published_at")})
        return candidate

    def _prepare_source(self, job):
        from scripts.neh_source import source_checkout, export_source
        from .sources import bridge
        from scripts.neh_system_helper import dependency_plan
        plan = job["_plan"]
        selected = plan["_source"]
        candidate = self.home / "releases" / (identifier(selected["version"]) + "-" + job["id"][:8])
        self._journal(job, candidate=str(candidate))
        self._progress(job, "source_download", 5, "获取计划固定的源码提交")
        with source_checkout(self.home, selected["repository"], selected["branch"], selected["revision"],
                             run=run_command, allow_local=selected.get("allow_local", False),
                             cancelled=lambda: self._cancelled(job["id"]),
                             progress=lambda message: self._progress(job, "source_download", 15, message)) as source:
            from .update_flow import wait_prebuilt
            source["prebuilt_index"] = wait_prebuilt(self, selected, job)
            candidate.mkdir()
            export_source(source["root"], source["revision"], candidate / "source")
            self._cancelled(job["id"])
            metadata = bridge(self, source, candidate, "metadata", job)
            system = dependency_plan({"packages": metadata["system_packages"]})
            if system["missing"]:
                self._progress(job, "system_dependencies", 25, "通过固定系统助手准备构建与运行依赖")
                system_helper("install-dependencies", {"packages": metadata["system_packages"]})
            self._progress(job, "python_environment", 35, "准备或复用已封存的Python依赖环境")
            dependencies = bridge(self, source, candidate, "dependencies", job)
            self._progress(job, "native_build", 50, "在设备稳定缓存中增量构建所选模块")
            built = bridge(self, source, candidate, "build", job)
            mutable_source = candidate / "source"
            (mutable_source / "runtime_data").symlink_to(self.home / "data", target_is_directory=True)
            for driver in (mutable_source / "drivers").iterdir():
                if driver.is_dir():
                    target = self.home / "config/profiles" / driver.name
                    target.mkdir(parents=True, exist_ok=True)
                    (driver / "configs").symlink_to(target, target_is_directory=True)
            existing = self._migration_configuration(Path(plan["_migration"]["source"])) if plan.get("_migration") else self._existing(self.current())
            self._configure_candidate(candidate, plan, existing, allow_pending_models=True)
            self._progress(job, "models", 65, "准备所选模型并复用已有模型文件")
            bridge(self, source, candidate, "models", job)
            self._progress(job, "source_verify", 70, "验证候选依赖、程序和启动入口")
            bridge(self, source, candidate, "verify", job)
            record = {"complete": True, "version": selected["version"], "revision": selected["revision"],
                      "modules": plan["modules"], "robot": plan["robot"], "data_schema": 1, "source": "git",
                      "source_repository": selected["repository"], "source_branch": selected["branch"],
                      "source_changes": source["changed_files"], "previous_source_revision": source["previous_revision"],
                      **dependencies, **built}
            atomic_json(candidate / "installed.json", record)
        return candidate

    def _configure_candidate(self, candidate, plan, existing, *, allow_pending_models=False):
        if plan.get("action") == "rollback":
            existing = probe(candidate, "validate_existing", values=existing)
        configuration = plan.get("_request", {}).get("configuration", {})
        if not isinstance(configuration, dict):
            raise ManagerError("invalid_config", "预配置必须是键值对象。")
        if configuration:
            existing.update(probe(candidate, "validate", values=configuration))
        defaults = read_settings(candidate / "source/.env.example")
        persistent = {"NUWAX_GATEWAY_ARTIFACT_DIR", "NUWAX_MAP_CATALOG_ROOT", "NUWAX_MAPPING_DATABASE_ROOT", "NUWAX_MAPPING_PCD_ROOT", "NUWAX_MEMORY_DB_PATH", "NUWAX_RUNTIME_DATA_DIR", "NUWAX_LOG_DIR", "NUWAX_LOG_FILE", "NUWAX_NUWAX_WORKSPACE_DIR", "NUWAX_FORBIDDEN_ZONES_PATH"}
        for key in persistent:
            value = existing.get(key, defaults.get(key, "")).strip()
            if not value:
                continue
            path = Path(value)
            if not path.is_absolute():
                relative = Path(*path.parts[1:]) if path.parts and path.parts[0] == "runtime_data" else path
                existing[key] = str(self.home / "data" / relative)
            elif self.home / "releases" in path.parents:
                parts = path.relative_to(self.home / "releases").parts
                if len(parts) >= 3 and parts[1:3] == ("source", "runtime_data"):
                    existing[key] = str(self.home / "data" / Path(*parts[3:]))
        model_settings = []
        if "perception" in plan["modules"]:
            model_settings.append(("NUWAX_PERCEPTION_YOLO_WORLD_MODEL", "yolov8s-worldv2.pt"))
        if "memory" in plan["modules"]:
            model_settings.extend((("NUWAX_MEMORY_TEXT_EMBEDDING_MODEL", "BAAI__bge-m3"), ("NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL", "openai__clip-vit-base-patch32")))
        from .downloads import apply_model_overrides, uses_default_model
        model_overrides = {}
        for key, name in model_settings:
            value = existing.get(key, "")
            if allow_pending_models and uses_default_model(value):
                value = existing.get("_NUWAX_INSTALLER_SAVED_" + key, "")
                if uses_default_model(value):
                    value = str(self.home / "data/models" / name)
            selected = Path(value) if value else None
            if not selected or uses_default_model(value):
                raise ManagerError("model_missing", "所选模块缺少已下载或明确自定义的模型入口：" + key)
            if not selected.is_absolute():
                selected = candidate / "source" / selected
            if not allow_pending_models and (not selected.exists() or (selected.is_dir() and not any(item.is_file() for item in selected.rglob("*")))):
                raise ManagerError("model_missing", "所选模块缺少本地模型，不会在启动时隐式下载：" + name)
            model_overrides[key] = str(selected.resolve())
        existing = apply_model_overrides(existing, model_overrides)
        existing.update(NUWAX_GATEWAY_HOST="127.0.0.1", NUWAX_GATEWAY_PORT=str(self.runtime.port()))
        probe(candidate, "runtime", install_prefix=str(candidate), modules=plan["modules"], robot=plan["robot"], values=existing)
        normalized = probe(candidate, "normalize_profile", values=read_settings(candidate / "settings.env"))
        rendered = "\n".join(_VALUES.env_assignment(key, str(value)) for key, value in sorted(normalized.items())) + "\n"
        atomic_write(candidate / "settings.env", rendered.encode())
        import shlex
        launcher = candidate / "start.sh"
        text = launcher.read_text()
        text = text.replace("set -euo pipefail", "set -euo pipefail\nexport NUWAX_MANAGED_HOME=" + shlex.quote(str(self.home)))
        text = text.replace('export PYTHONNOUSERSITE=1', 'export PYTHONNOUSERSITE=1\nexport LD_LIBRARY_PATH=' + shlex.quote(str(candidate / "lib") + ":" + str(candidate / "lib64")))
        atomic_write(launcher, text.encode(), mode=0o755)

    def migration_sources(self):
        import hashlib
        paths = {Path("/opt/nuwax_robot_bridge"), Path("/opt/nuwax/current"), Path.home() / "nuwax_robot_bridge"}
        try:
            state = subprocess.run(["systemctl", "show", "nuwax-gateway.service", "--property=WorkingDirectory", "--value"], capture_output=True, text=True, timeout=5)
            value = state.stdout.strip()
            if value.startswith("/"):
                paths.add(Path(value))
        except (OSError, subprocess.SubprocessError):
            pass
        sources = []
        for path in paths:
            if not (path / "scripts/start_gateway.sh").is_file():
                continue
            try:
                info = self._migration_info(str(path))
                info["id"] = hashlib.sha256(info["source"].encode()).hexdigest()[:24]
                sources.append({key: value for key, value in info.items() if not key.startswith("_")})
            except (ManagerError, OSError, ValueError):
                continue
        return {"sources": sources, "message": "网页只迁入已识别的原安装；其它路径可在设备本地使用migrate明确指定。"}

    @staticmethod
    def _migration_configuration(source):
        prefix = source / "runtime_data/installation/current"
        current = {"prefix": str(prefix)} if (prefix / "settings.env").is_file() else {}
        return existing_config(source, current)

    def _migration_info(self, source_path):
        if not source_path:
            raise ManagerError("source_missing", "迁入需要明确指定原项目目录。")
        source = Path(source_path).expanduser().resolve()
        if source == self.home or self.home in source.parents or source in self.home.parents:
            raise ManagerError("source_invalid", "迁入源与管理安装目录不能互相包含。")
        if not (source / "scripts/start_gateway.sh").is_file():
            raise ManagerError("source_invalid", "迁入源缺少原统一启动器。")
        values = self._migration_configuration(source)
        systemd = any(str(values.get(key, "")).lower() in ("true", "1") for key in ("NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD", "NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD"))
        try:
            result = subprocess.run(["systemctl", "show", "nuwax-gateway.service", "--property=WorkingDirectory,ExecStart,ActiveState"],
                                    capture_output=True, text=True, timeout=5)
            if str(source) in result.stdout:
                systemd = True
        except (OSError, subprocess.SubprocessError):
            pass
        launcher = source / "runtime_data/installation/current/start.sh"
        if not launcher.is_file():
            launcher = source / "scripts/start_gateway.sh"
        try:
            pid = int((source / "runtime_data/gateway/gateway.pid").read_text().strip())
        except (OSError, ValueError):
            pid = 0
        legacy = read_json(source / "runtime_data/installation/current/manifest.json", {})
        return {"source": str(source), "launcher": str(launcher), "systemd": systemd,
                "robot": values.get("NUWAX_ACTIVE_ROBOT", legacy.get("robot", "customrobot")), "modules": legacy.get("modules", []), "_configuration": values,
                "running": bool(process_identity(pid)) or (systemd and "ActiveState=active" in (result.stdout if "result" in locals() else ""))}

    def _stop_migration(self, job):
        info = job["_plan"]["_migration"]
        fresh = self._migration_info(info["source"])
        if any(fresh.get(key) != info.get(key) for key in ("running", "systemd", "launcher", "_configuration")):
            raise ManagerError("plan_stale", "准备期间迁入源的拥有方或配置发生变化，请重新生成计划。", 409)
        self._journal(job, legacy_stopped=False, legacy_systemd=info["systemd"], legacy_running=info["running"], legacy_launcher=info["launcher"], legacy_source=info["source"])
        self._progress(job, "migration_stop", 68, "停止迁入源的原拥有方")
        if info["systemd"]:
            self._journal(job, legacy_stopped=True)
            system_helper("stop-legacy")
        else:
            source = Path(info["source"])
            if self.runtime._foreign_owners((source,)):
                raise ManagerError("foreign_owner", "检测到迁入源以外的业务拥有方，拒绝原脚本的全局清理。", 409)
            try:
                pid = int((source / "runtime_data/gateway/gateway.pid").read_text().strip())
            except (OSError, ValueError):
                pid = 0
            identity = process_identity(pid)
            if identity:
                if Path("/proc/{}/cwd".format(pid)).resolve() != source:
                    raise ManagerError("owner_identity", "迁入源的进程身份不符，拒绝停止未知拥有方。", 409)
                import signal
                self._journal(job, legacy_stopped=True)
                os.kill(pid, signal.SIGTERM)
                deadline = time.monotonic() + 180
                while process_alive(identity) and time.monotonic() < deadline:
                    time.sleep(0.2)
                if process_alive(identity):
                    raise ManagerError("stop_incomplete", "迁入源拥有方尚未退出。", 409)
            self._journal(job, legacy_stopped=True)
            run_command(["bash", info["launcher"], "stop"], cwd=source, environment=clean_environment(), timeout=180)

    def _copy_migration(self, job):
        root = Path(job["_plan"]["_migration"]["source"])
        self._progress(job, "migration_data", 72, "迁入持久数据和全部配置方案，保留源安装")
        candidates = []
        source = root / "runtime_data"
        if source.is_dir():
            candidates.extend((child, Path("data") / child.name) for child in source.iterdir() if child.name not in ("installation", "gateway", "settings", "logs"))
        for driver in (root / "drivers").iterdir():
            configs = driver / "configs"
            if configs.is_dir():
                candidates.extend((child, Path("config/profiles") / driver.name / child.name) for child in configs.iterdir() if child.is_file() and (child.name == ".active" or child.suffix == ".env"))
        if (root / "artifacts").is_dir():
            candidates.append((root / "artifacts", Path("data/artifacts")))
        expanded = []
        for child, relative in candidates:
            if child.is_dir():
                if child.is_symlink() or any(item.is_symlink() for item in child.rglob("*")):
                    raise ManagerError("migration_external", "迁入源包含符号链接数据，请先在设备本地明确迁移方式：" + child.name)
                expanded.extend((item, relative / item.relative_to(child)) for item in child.rglob("*") if item.is_file())
            else:
                expanded.append((child, relative))
        candidates = expanded
        staging = self.home / "cache" / ("migration-" + job["id"])
        for child, relative in candidates:
            self._cancelled(job["id"])
            target = self.home / relative
            if target.exists() or target.is_symlink():
                raise ManagerError("migration_conflict", "迁入内容与目标已有文件冲突：" + str(relative))
            if child.is_symlink() or (child.is_dir() and any(item.is_symlink() for item in child.rglob("*"))):
                raise ManagerError("migration_external", "迁入源包含外部或符号链接数据，请先在设备本地明确迁移方式：" + child.name)
            temporary = staging / relative
            temporary.parent.mkdir(parents=True, exist_ok=True)
            def copy_file(src, dst):
                self._cancelled(job["id"])
                return shutil.copy2(src, dst)
            if child.is_dir():
                shutil.copytree(child, temporary, copy_function=copy_file)
            else:
                copy_file(child, temporary)
            paths = list(job["_journal"].get("migration_paths", [])) + [str(relative)]
            self._journal(job, migration_paths=paths)
            target.parent.mkdir(parents=True, exist_ok=True)
            os.replace(str(temporary), str(target))
        # 原拥有方已停止后再次读取最终方案，避免准备期间用户的最后一次保存丢失。
        candidate = Path(job["_journal"]["candidate"])
        values = self._migration_configuration(root)
        from .downloads import MODEL_SETTINGS
        prepared = read_settings(candidate / "settings.env")
        for key in MODEL_SETTINGS:
            if key in prepared:
                values[key] = prepared[key]
        for key, value in list(values.items()):
            path = Path(value)
            if path.is_absolute() and (root / "runtime_data" == path or root / "runtime_data" in path.parents):
                values[key] = str(self.home / "data" / path.relative_to(root / "runtime_data"))
            elif path.is_absolute() and (root / "artifacts" == path or root / "artifacts" in path.parents):
                values[key] = str(self.home / "data/artifacts" / path.relative_to(root / "artifacts"))
        self._configure_candidate(candidate, job["_plan"], values)

    def _start_legacy(self, launcher, source):
        try:
            pid = int((Path(source) / "runtime_data/gateway/gateway.pid").read_text().strip())
        except (OSError, ValueError):
            pid = 0
        if process_identity(pid):
            raise ManagerError("legacy_restore_pending", "迁入源拥有方仍在退出，保留其关闭流程，恢复尚未完成。", 500)
        log = os.open(str(self.home / "logs/migration-restore.log"), os.O_CREAT | os.O_APPEND | os.O_WRONLY, 0o600)
        with os.fdopen(log, "ab") as output:
            process = subprocess.Popen(["bash", launcher, "start"], cwd=source, stdin=subprocess.DEVNULL, stdout=output, stderr=output,
                                       env=clean_environment(), start_new_session=True)
        values = self._migration_configuration(Path(source))
        port = int(values.get("NUWAX_GATEWAY_PORT", "8766") or "8766")
        import urllib.request
        deadline = time.monotonic() + 90
        while process.poll() is None and time.monotonic() < deadline:
            try:
                with urllib.request.build_opener(urllib.request.ProxyHandler({})).open("http://127.0.0.1:{}/api/health".format(port), timeout=2) as response:
                    if response.status == 200:
                        return
            except OSError:
                pass
            time.sleep(0.5)
        raise ManagerError("legacy_restore_failed", "迁入失败后原拥有方未恢复健康，请检查原服务。", 500)

    def config(self):
        current = self.current()
        if not current:
            return {"available": False, "groups": [], "fields": [], "message": "安装完成后可在业务离线时编辑机器人配置；首次安装支持预配置文件。"}
        schema = self.configuration.request(Path(current["prefix"]), "get")
        fields, groups = [], []
        for section in schema["sections"]:
            groups.append({"key": section["section_id"], "title": section["section_title"]})
            for field in section["fields"]:
                fields.append(dict(field, label=field["zh_name"], secret=field["sensitive"], group=section["section_id"], restart_required=True))
        return {"available": True, "groups": groups, "fields": fields, "revision": (schema.get("profile_context") or {}).get("revision"),
                "message": "已保存配置供下次启动读取；在线热加载请使用原控制台设置。"}

    def save_config(self, values, revision=None):
        if not isinstance(values, dict):
            raise ManagerError("invalid_config", "配置values必须是对象。")
        if not isinstance(revision, str) or not revision:
            raise ManagerError("config_revision_required", "保存配置需要编辑版本，请先读取当前配置。", 409)
        protected = {"NUWAX_GATEWAY_HOST": "127.0.0.1", "NUWAX_GATEWAY_PORT": str(self.runtime.port())}
        if any(key in values and str(values[key]) != expected for key, expected in protected.items()):
            raise ManagerError("managed_setting", "管理模式固定业务网关回环地址与受管端口。")
        with file_lock(self.lock_path):
            self._assert_idle()
            current = self.current()
            if not current:
                raise ManagerError("not_installed", "请先安装业务版本。", 409)
            result = self.configuration.request(Path(current["prefix"]), "save", values=values, revision=revision)
            if result["rejected"]:
                raise ManagerError("config_rejected", "部分配置没有通过原校验。", 400, result)
            return {"saved": [key for keys in result["written"].values() for key in keys], "restart_required": bool(result["written"]),
                    "message": "配置已保存，下次业务启动生效。", "skipped": result["skipped"]}
