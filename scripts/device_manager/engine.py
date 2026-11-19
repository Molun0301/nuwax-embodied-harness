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

from . import MANAGER_DEVELOPMENT, MANAGER_REVISION, MANAGER_VERSION, PACKAGE_ROOT
from .common import (ManagerError, atomic_json, atomic_write, file_lock,
                     fsync_directory, identifier, process_alive, process_identity, read_json, timestamp)
from .runtime import (Runtime, append_job_output, clean_environment, probe, run_command,
                      set_task_secrets, system_helper, task_redaction)

from .storage import TERMINAL
ACTIONS = {"install", "update", "modules", "rollback", "start", "stop", "restart", "update-manager"}


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
        return {key: value for key, value in self._settings_snapshot().items() if not key.startswith("_")}

    def _settings_snapshot(self):
        from .sources import normalized_settings
        stored = read_json(self.manager_home / "config/settings.json", {})
        settings = normalized_settings(stored)
        settings["_allow_local_source"] = bool(stored.get("_allow_local_source"))
        settings["password_enabled"] = bool(read_json(self.manager_home / "config/password.json", {}).get("digest"))
        return settings

    def save_settings(self, values, *, allow_local_source=False):
        from scripts.neh_source import SourceError, validate_repository
        values = dict(values)
        if set(values) - {"source_repository", "source_branch", "channel", "auto_start", "gateway_port",
                          "auto_update", "check_interval_minutes", "maintenance_window"}:
            raise ManagerError("invalid_settings", "存在未知管理设置。")
        if "channel" in values:
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
            values.update(source_branch=branch)
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
        if "source_branch" in values and (not isinstance(values["source_branch"], str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]{0,199}", values["source_branch"]) or ".." in values["source_branch"]):
            raise ManagerError("invalid_settings", "源码分支名称无效。")
        if "source_repository" in values:
            try:
                values["source_repository"] = validate_repository(values["source_repository"], allow_local=allow_local_source)
            except SourceError as exc:
                raise ManagerError("invalid_settings", str(exc)) from exc
            values["_allow_local_source"] = bool(allow_local_source)
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

    def _existing(self, candidate, job):
        result = probe(candidate, "existing")
        self._progress(job, "configuration", 55, "旧格式方案导入结果：" + json.dumps(result["import"], ensure_ascii=False))
        return result["values"]

    def hardware(self):
        hardware = detect_hardware(self.home)
        current = self.current()
        recommendation = recommend_modules(hardware, current.get("modules"), installed_robot=current.get("robot"))
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
                            for item in MODULES.values()], "selected": selected, "robot": current.get("robot", "customrobot"), "robots": ["customrobot", "go2", "unitree_sim"], "robot_labels": {"customrobot": "标准机器人接入", "go2": "宇树Go2", "unitree_sim": "unitree-sim/1 设备"}}

    def catalog(self, refresh=False, *, settings=None):
        from .sources import discover
        source = discover(self, refresh=refresh, settings=settings)
        return {"schema_version": 1, "product": "neh", "source": source, "releases": [],
                "manager": {"version": source["version"], "revision": source["revision"]}}

    def releases(self, refresh=False):
        _, result = self._release_snapshot(refresh=refresh, settings=self._settings_snapshot(), hardware=self.hardware())
        return result

    def _release_snapshot(self, *, refresh=False, settings, hardware):
        catalog = None
        result = {"configured": bool(settings["source_repository"]), "channel": settings["channel"], "releases": []}
        try:
            catalog = self.catalog(refresh=refresh, settings=settings)
            source = catalog["source"]
            result.update(source_repository=source["repository"], source_branch=source["branch"], source_revision=source["revision"])
            result["releases"] = [{"version": source["version"], "revision": source["revision"], "channel": settings["channel"],
                "notes": "设备下载固定源码并按程序指纹复用、下载或构建", "published_at": source.get("channel_record", {}).get("published_at", ""), "compatible": hardware["supported"],
                "display_version": source.get("channel_record", {}).get("version", source["branch"] + "@" + source["revision"][:8]),
                "reason": "", "modules": list(MODULES)}]
            return catalog, result
        except ManagerError as exc:
            if exc.code == "channel_pending":
                result.update(pending=True, message=exc.message)
            else:
                result["error"] = exc.message
        return catalog, result

    def _job_path(self, job_id):
        return self.manager_home / "jobs" / (identifier(job_id) + ".json")

    def job(self, job_id):
        job = read_json(self._job_path(job_id))
        if job is None:
            raise ManagerError("not_found", "任务不存在。", 404)
        return {key: value for key, value in job.items() if not key.startswith("_")}

    def job_output(self, job_id, offset=0, tail=None):
        """按字节偏移读取任务详细日志；tail给出时从末尾该字节数之后的完整行开始。"""
        self.job(job_id)
        path = self.manager_home / "logs" / (identifier(job_id) + ".output.log")
        try:
            size = path.stat().st_size
        except FileNotFoundError:
            return {"offset": 0, "size": 0, "text": ""}
        start = max(0, size - tail) if tail is not None else min(max(0, offset), size)
        with path.open("rb") as handle:
            handle.seek(start)
            data = handle.read(512 * 1024)
        if tail is not None and start > 0:
            skipped = data.find(b"\n") + 1
            data, start = data[skipped:], start + skipped
        if start + len(data) < size and b"\n" in data:
            # 只返回完整行，下次从断开处继续，避免切断多字节字符。
            data = data[:data.rfind(b"\n") + 1]
        return {"offset": start + len(data), "size": size, "text": data.decode("utf-8", errors="replace")}

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
        settings = self._settings_snapshot()
        hardware = self.hardware()
        catalog, releases = self._release_snapshot(settings=settings, hardware=hardware)
        available = next((item for item in releases["releases"] if item["compatible"]), None)
        failure = read_json(self.manager_home / "config/failed-updates.json", {}).get(available.get("revision")) if available else None
        relation = None
        relation_name = "unknown"
        if available and current:
            from .versioning import revision_relation, source_tree_unchanged
            try:
                relation_name = revision_relation(self, catalog["source"], current.get("revision"))
                if relation_name == "newer" and source_tree_unchanged(self, catalog["source"], current.get("revision")):
                    relation_name = "same"
            except (OSError, RuntimeError) as exc:
                relation_name = "unknown"
                releases["error"] = "无法核对完整提交历史：" + str(exc)
            relation = {"newer": 1, "same": 0, "older": -1}.get(relation_name)
        latest_manager = None
        manager_relation = None
        manager_relation_name = "unknown"
        if available:
            latest_manager = catalog.get("manager", {}).get("version")
            from .versioning import revision_relation, source_tree_unchanged
            try:
                manager_relation_name = revision_relation(self, catalog["source"], MANAGER_REVISION)
                if manager_relation_name == "newer" and source_tree_unchanged(self, catalog["source"], MANAGER_REVISION):
                    manager_relation_name = "same"
            except (OSError, RuntimeError):
                manager_relation_name = "unknown"
            manager_relation = {"newer": 1, "same": 0, "older": -1}.get(manager_relation_name)
        channel_record = catalog.get("source", {}).get("channel_record", {}) if available else {}
        installation = {"installed": bool(current), "install_dir": str(self.home), "version": current.get("version"), "modules": current.get("modules", []),
                        "robot": current.get("robot", "customrobot"), "previous_version": history.get("previous_version"), "source": current.get("source"), "revision": current.get("revision", "")}
        location = self.install_location()
        installation.update(manager_dir=location["manager_dir"], parent_dir=location["parent_dir"], user_home=location["user_home"],
                            location_editable=location["editable"], location_reason=location["reason"])
        return {"product": "NEH", "manager_version": MANAGER_VERSION, "manager_revision": MANAGER_REVISION, "manager_development": MANAGER_DEVELOPMENT, "instance_id": self.instance_id, "installation": installation,
                "runtime": self.runtime.status(), "active_job": self.active_job(),
                "update": {"available": bool(available and current and relation == 1),
                           "pending": bool(releases.get("pending")), "message": releases.get("message"),
                           "requires_confirmation": channel_record.get("requires_confirmation", False), "display_version": available.get("display_version") if available else None,
                           "check": read_json(self.manager_home / "config/update-check.json", {}),
                           "last_failure": {"time": failure["time"], "message": failure["error"]["message"]} if failure else None,
                           "version": available["version"] if available else None, "revision": available.get("revision") if available else None, "error": releases.get("error"), "manager_available": bool(latest_manager and manager_relation == 1), "manager_version": latest_manager, "manager_switch_available": bool(latest_manager and manager_relation == 1), "manager_relation": manager_relation_name, "relation": relation_name},
                "password_enabled": settings["password_enabled"], "platform_supported": hardware["supported"]}

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
        if plan["action"] == "update" or (plan["action"] == "update-manager" and not request.get("source_revision")):
            from .versioning import revision_relation
            installed = MANAGER_REVISION if plan["action"] == "update-manager" else current.get("revision")
            if installed:
                try:
                    relation = revision_relation(self, source, installed)
                except (OSError, RuntimeError) as exc:
                    relation = "unknown"
                    plan["warnings"].append("无法核对完整提交历史：" + str(exc))
                if ((request.get("automatic") and relation != "newer")
                        or (plan["action"] == "update-manager" and relation not in ("newer", "same"))):
                    plan["blockers"].append({"older": "当前版本领先通道", "diverged": "当前版本与通道已分叉"}.get(relation, "无法确认更新方向") + "；如需切换，请显式指定完整source_revision。")
                elif relation in ("older", "diverged"):
                    plan["warnings"].append("将切换到通道提交，它" + {"older": "旧于", "diverged": "分叉于"}[relation] + "当前版本")
        from .versioning import source_tree_unchanged
        try:
            same_tree = source_tree_unchanged(self, source, current.get("revision")) if current else False
        except (OSError, RuntimeError):
            same_tree = False
        unchanged = same_tree and set(current.get("modules", [])) == set(plan["modules"]) and current.get("robot") == plan["robot"] and not request.get("configuration")
        if plan["action"] == "update" and current.get("revision") != source["revision"]:
            unchanged = False
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
        return values

    def plan(self, request, *, local_request=False):
        self.recover_if_needed()
        if not isinstance(request, dict) or request.get("action") not in ACTIONS:
            raise ManagerError("invalid_action", "请选择受支持的管理操作。")
        action = request["action"]
        current, settings = self.current(), self.settings()
        hardware = self.hardware()
        try:
            modules = list(resolve_modules(request.get("modules", current.get("modules", hardware["recommendation"]["modules"]))))
        except (ModuleError, TypeError) as exc:
            raise ManagerError("invalid_modules", str(exc)) from exc
        robot = request.get("robot", current.get("robot", hardware["recommendation"]["robot"]))
        if robot not in ("customrobot", "go2", "unitree_sim"):
            raise ManagerError("robot_unsupported", "当前启动器支持通用机器人、Go2 和 unitree_sim。")
        blockers, warnings, steps = [], [], []
        if action == "install":
            configuration = request.get("configuration", {})
            if not isinstance(configuration, dict):
                raise ManagerError("invalid_config", "预配置必须是对象。")
            if not configuration.get("gateway.public_base_url"):
                server = read_json(self.manager_home / "config/server.json", {})
                if process_alive(server.get("process")) and server.get("public_url"):
                    request = dict(request, configuration=dict(configuration, **{"gateway.public_base_url": server["public_url"]}))
                warnings.append("命令行首次安装请确认公开制品地址；可通过configuration.gateway.public_base_url指定远端可达的管理台地址。")
        if action == "install" and current:
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
                "download_breakdown": {"application": 0, "dependencies": None, "models": None}}
        if action == "update-manager":
            self._plan_source(plan, request, local_request)
        elif action in ("install", "update", "modules"):
            blockers.extend(hardware["issues"])
            if hardware["hardware"].get("is_jetson") and set(plan["modules"]) & {"perception", "memory"}:
                # 与源码构建前置检查使用同一探测，生成计划时就给出原因。
                from scripts.module_installer.manager import InstallError, jetson_vendor_packages
                try:
                    jetson_vendor_packages(sys.executable)
                except InstallError as exc:
                    blockers.append(str(exc))
            self._plan_source(plan, request, local_request)
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
            # 详细日志里用阶段标记分隔各步骤的命令输出。
            append_job_output("==> " + message, home=self.manager_home, job_id=job["id"])
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
        with file_lock(self.lock_path, blocking=True), task_redaction() as redaction:
            job = read_json(self._job_path(job_id))
            if not job or job["status"] in TERMINAL:
                return
            redaction["current"] = job["action"] in ("start", "stop", "restart")
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
            if (job["status"] == "succeeded" and job["action"] in {"install", "update", "modules", "rollback", "update-manager"}
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
        selector = self.home / "config/active_profile"
        return selector.read_text().strip() if selector.exists() else "default"

    def _snapshot_profiles(self, robot):
        result = []
        name = self._profile_name(robot)
        paths = [self.home / "config/active_profile", self.home / "config/profiles" / (name + ".toml"),
                 self.home / "run/config.toml"]
        for path in paths:
            if path.is_symlink():
                kind, value = "link", os.readlink(str(path))
            elif path.is_file():
                kind, value = "file", base64.b64encode(path.read_bytes()).decode()
            else:
                kind, value = "missing", ""
            result.append({"path": str(path.relative_to(self.home)), "kind": kind, "value": value})
        return result

    def _activate_profile(self, candidate, robot, values):
        probe(candidate, "install", name=self._profile_name(robot), values=values)

    def _execute(self, job):
        plan, action = job["_plan"], job["action"]
        if action == "update-manager":
            from .self_update import apply_manager
            return apply_manager(self, job)
        current = self.current()
        if action == "update" and not job.get("_manager_adopted"):
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
            plan["_configuration"] = self._configure_candidate(candidate, plan, self._existing(candidate, job))
        else:
            candidate = self._prepare_source(job)
        self._cancelled(job["id"])
        if plan.get("_fingerprint") and plan["_fingerprint"] != self._fingerprint():
            raise ManagerError("plan_stale", "准备期间设备配置发生变化，保留当前版本，请重新生成计划。", 409)
        profiles = self._snapshot_profiles(plan["robot"])
        self._journal(job, candidate=str(candidate), profiles=profiles)
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
        self._activate_profile(candidate, plan["robot"], plan["_configuration"])
        self._switch(candidate)
        self._cancelled(job["id"])
        should_start = was_running or self.settings()["auto_start"]
        if should_start:
            self._progress(job, "starting", 90, "启动原业务拥有方并验证网关接口")
            self.runtime.start(candidate)
        self._cancelled(job["id"])
        atomic_json(self.home / "config/history.json", {"previous": previous.name if previous else None, "previous_version": current.get("version")})
        self._journal(job, committed=True)
        return {"version": plan["version"], "modules": plan["modules"], "robot": plan["robot"], "runtime": self.runtime.status()}


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
            existing = self._existing(candidate, job)
            plan["_configuration"] = self._configure_candidate(candidate, plan, existing, allow_pending_models=True)
            self._progress(job, "models", 65, "准备所选模型并复用已有模型文件")
            bridge(self, source, candidate, "models", job, configuration=plan["_configuration"])
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
        configuration = plan.get("_request", {}).get("configuration", {})
        if not isinstance(configuration, dict):
            raise ManagerError("invalid_config", "预配置必须是键值对象。")
        existing = dict(existing, **configuration)
        existing = probe(candidate, "prepare", values=existing)
        model_settings = []
        if "perception" in plan["modules"]:
            model_settings.append(("perception.yolo_world_model", "yolov8s-worldv2.pt"))
        if "memory" in plan["modules"]:
            model_settings.extend((("memory.text_embedding_model", "BAAI__bge-small-zh-v1.5"), ("memory.image_embedding_model", "openai__clip-vit-base-patch32")))
        from .downloads import apply_model_overrides, uses_default_model
        model_overrides = {}
        for key, name in model_settings:
            value = existing.get(key, "")
            if allow_pending_models and uses_default_model(value):
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
        existing["gateway.host"] = "127.0.0.1"
        existing.pop("gateway.port", None)
        values = probe(candidate, "runtime", install_prefix=str(candidate), modules=plan["modules"], robot=plan["robot"], values=existing)
        set_task_secrets(probe(candidate, "secrets", values=values))
        return values

    def _configuration(self, operation, **parameters):
        current = self.current()
        if not current:
            raise ManagerError("not_installed", "请先安装业务版本。", 409)
        return probe(Path(current["prefix"]), operation, **parameters)

    def config(self, profile=None):
        if not self.current():
            return {"available": False, "groups": [], "fields": [], "message": "安装完成后可编辑机器人配置。"}
        result = self._configuration("schema", profile=profile)
        result.update(available=True, groups=[{"key": section["section_id"], "title": section["section_title"]} for section in result["sections"]],
                      fields=[dict(field, group=section["section_id"], secret=field["sensitive"], restart_required=field["apply"] == "restart")
                              for section in result["sections"] for field in section["fields"]],
                      application={"restart_available": True}, message="热加载项保存后即时生效，其余参数需要重启业务。")
        return result

    def save_config(self, values, profile=None, *, text_values=False):
        with file_lock(self.lock_path):
            self._assert_idle()
            result = self._configuration("save", values=values, profile=profile, text_values=text_values)
            result.update(hot_applied=[], apply_errors={})
            if result["is_active"] and result["hot_keys"] and self.runtime.has_owner():
                import urllib.request
                import urllib.error
                import json
                request = urllib.request.Request("http://127.0.0.1:" + str(self.runtime.port()) + "/api/config/reload", data=b"{}", method="POST", headers={"Content-Type": "application/json"})
                try:
                    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=30) as response:
                        result.update(json.load(response))
                except (OSError, ValueError) as error:
                    result["apply_errors"] = dict.fromkeys(result["hot_keys"], str(error))
            result["message"] = ("方案已保存，切换后生效。" if not result["is_active"] else
                                 "已保存；需要重启的参数请重启业务。" if result["restart_required"] else "已保存热加载设置。")
            return result

    def manage_profile(self, operation, *, name=None, source=None):
        with file_lock(self.lock_path):
            self._assert_idle()
            result = self._configuration(operation, name=name, source=source)
            if operation in ("create", "delete"):
                return result
            current = Path(self.current()["prefix"])
            self.runtime.stop(current)
            self.runtime.start(current)
            return result

    def restart_business(self):
        plan = self.plan({"action": "restart"}, local_request=True)
        return self.submit(plan["id"])
