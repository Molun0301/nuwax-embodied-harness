"""机器人配置方案的持久化；读写和切换共用目录锁与版本检查。"""
from __future__ import annotations

from collections import OrderedDict
from contextlib import contextmanager
import tempfile
import threading
import os
import uuid
from pathlib import Path

from core.config_values import configuration_location, configuration_sources, configuration_file_values, parse_env_line, env_assignment

_PATH_KEYS = frozenset({
    "NUWAX_BOOTSTRAP_ENV_FILE", "GO2_DATA_PLANE_ENV_FILE", "NUWAX_USER_CONFIG_ENV_FILE",
    "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE", "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE",
    "NUWAX_GEOMETRY_EXTRA_ENV_FILES", "NUWAX_SETTINGS_LAUNCHER",
})


class ConfigurationConflict(ValueError):
    """页面读取之后文件已变化，保留客户端编辑以供刷新合并。"""


def _conflict():
    return ConfigurationConflict("配置版本已变化或编辑版本已失效，请刷新后重新保存；本页输入仍保留。")


def _scope(location, root):
    return (str(Path(root).resolve()), location["robot"], location["profile"])


class CapturedProfile:
    """私有源字节快照；小输入留在RAM，大输入滚入匿名临时文件。"""

    def __init__(self, location, root):
        self.scope = _scope(location, root)
        self._lock = threading.RLock()
        self._references = 1
        self._stream = None
        self.sources = ()
        self.total_bytes = 0
        self.profile_path = Path(location["path"])
        # 只读配置不要求项目目录可写；溢出交给0600的私有临时文件设施。
        stream = tempfile.SpooledTemporaryFile(max_size=256 * 1024, mode="w+b")
        self._stream = stream
        try:
            sources = []
            for path in configuration_sources(root=root, robot=location["robot"], profile=location["profile"]):
                path = Path(path)
                start = stream.tell()
                present = path.is_file()
                if present:
                    with path.open("rb") as source:
                        before = os.fstat(source.fileno())
                        remaining = before.st_size
                        while remaining:
                            data = source.read(min(64 * 1024, remaining))
                            if not data:
                                raise _conflict()
                            stream.write(data)
                            remaining -= len(data)
                        after = os.fstat(source.fileno())
                    if (before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (after.st_size, after.st_mtime_ns, after.st_ctime_ns):
                        raise _conflict()
                sources.append((path, present, start, stream.tell() - start))
            self.sources = tuple(sources)
            self.total_bytes = stream.tell()
            stream.flush()
            self.profile_present = any(path == self.profile_path and present for path, present, _offset, _size in self.sources)
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass
            raise

    def retain(self):
        with self._lock:
            if self._stream is None:
                raise _conflict()
            self._references += 1
        return self

    def close(self):
        with self._lock:
            if self._references <= 0:
                return
            self._references -= 1
            if not self._references and self._stream is not None:
                stream = self._stream
                self._stream = None
                stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def equivalent(self, other):
        if self is other:
            return True
        if self.scope != other.scope or self.sources != other.sources or self.total_bytes != other.total_bytes:
            return False
        first, second = sorted((self, other), key=id)
        with first._lock, second._lock:
            if self._stream is None or other._stream is None:
                raise _conflict()
            self._stream.seek(0)
            other._stream.seek(0)
            remaining = self.total_bytes
            while remaining:
                count = min(64 * 1024, remaining)
                if self._stream.read(count) != other._stream.read(count):
                    return False
                remaining -= count
        return True

    def raw(self, path):
        path = Path(path)
        for source_path, present, offset, size in self.sources:
            if source_path == path:
                if not present:
                    return None
                with self._lock:
                    if self._stream is None:
                        raise _conflict()
                    self._stream.seek(offset)
                    return self._stream.read(size)
        return None

    def read_values(self, path):
        raw = self.raw(path)
        values = {}
        for line in (raw.decode("utf-8").splitlines() if raw is not None else ()):
            parsed = parse_env_line(line)
            if parsed is not None:
                values[parsed[0]] = parsed[1]
        return values

    def values(self):
        values = {}
        for path, _present, _offset, _size in self.sources:
            values.update(configuration_file_values(path, reader=self.read_values))
        values["NUWAX_ACTIVE_ROBOT"] = self.scope[1]
        return values


class ProfileRevisionOwner:
    """一个设置拥有方的有限当前版本；不建立每客户端历史或持久注册表。"""

    def __init__(self, *, max_profiles=16):
        self._lock = threading.RLock()
        self._pid = os.getpid()
        self._epoch = uuid.uuid4().hex
        self._sequence = 0
        self._max_profiles = max(1, int(max_profiles))
        self._current = OrderedDict()
        self._closed = False

    def issue(self, snapshot):
        if self._pid != os.getpid():
            raise _conflict()
        while True:
            with self._lock:
                if self._closed:
                    raise _conflict()
                previous = self._current.get(snapshot.scope)
                lease = previous[1].retain() if previous else None
            try:
                same = lease is not None and lease.equivalent(snapshot)
                released = []
                with self._lock:
                    if self._closed:
                        raise _conflict()
                    if self._current.get(snapshot.scope) is not previous:
                        continue
                    if same:
                        token = previous[0]
                    else:
                        self._sequence += 1
                        token = "config:" + self._epoch + ":" + str(self._sequence)
                        self._current[snapshot.scope] = (token, snapshot.retain())
                        if previous:
                            released.append(previous[1])
                    self._current.move_to_end(snapshot.scope)
                    while len(self._current) > self._max_profiles:
                        released.append(self._current.popitem(last=False)[1][1])
                for old in released:
                    old.close()
                return token
            finally:
                if lease is not None:
                    lease.close()

    def check(self, snapshot, expected):
        if self._pid != os.getpid():
            raise _conflict()
        with self._lock:
            current = self._current.get(snapshot.scope)
            if self._closed or current is None or current[0] != expected:
                raise _conflict()
            baseline = current[1].retain()
        try:
            if not baseline.equivalent(snapshot):
                raise _conflict()
        finally:
            baseline.close()

    def close(self):
        # fork复制的拥有方不能访问父进程的快照游标或继承的线程锁。
        if self._pid != os.getpid():
            return
        with self._lock:
            self._closed = True
            snapshots = [item[1] for item in self._current.values()]
            self._current.clear()
        failure = None
        for snapshot in snapshots:
            try:
                snapshot.close()
            except BaseException as error:
                if failure is None:
                    failure = error
        if failure is not None:
            raise failure

    @property
    def closed(self):
        if self._pid != os.getpid():
            return True
        with self._lock:
            return self._closed

    def __del__(self):
        if hasattr(self, "_current"):
            self.close()


def read_profile(location, *, root, snapshot=None):
    if snapshot is not None:
        if snapshot.scope != _scope(location, root):
            raise _conflict()
        return snapshot.values()
    with CapturedProfile(location, root) as captured:
        return captured.values()


def revision(location, *, root, revision_owner, snapshot=None):
    if snapshot is None:
        with CapturedProfile(location, root) as captured:
            return revision(location, root=root, revision_owner=revision_owner, snapshot=captured)
    if snapshot.scope != _scope(location, root):
        raise _conflict()
    return revision_owner.issue(snapshot)


@contextmanager
def checked_profile(location, expected, *, root, revision_owner=None):
    with CapturedProfile(location, root) as captured:
        if expected is not None:
            if revision_owner is None:
                raise _conflict()
            revision_owner.check(captured, expected)
        yield captured


def check_revision(location, expected, *, root, revision_owner=None):
    if expected is None:
        return
    from core.config_io import _target_lock
    with _target_lock(Path(root) / "runtime_data" / "settings" / "transaction"), checked_profile(
            location, expected, root=root, revision_owner=revision_owner):
        pass


def render_values(values):
    lines = ["# 机器人独立配置；通过控制台保存、另存或选择方案。"]
    lines.extend(env_assignment(key, str(value)) for key, value in sorted(values.items())
                 if key not in _PATH_KEYS and not key.startswith("_NUWAX_"))
    return "\n".join(lines) + "\n"


def ensure_profile(location, *, root, snapshot=None):
    from core.config_io import _atomic_write

    if snapshot is not None:
        current = location["path"].read_bytes() if location["path"].is_file() else None
        if current != snapshot.raw(location["path"]):
            raise _conflict()
    if not location["path"].exists():
        _atomic_write(location["path"], render_values(read_profile(location, root=root, snapshot=snapshot)))


def profile_catalog(*, root, robot=None, profile=None, revision_owner=None, snapshot=None):
    location = configuration_location(root=root, robot=robot, profile=profile)
    if location is None:
        return None
    directory = root / "drivers" / location["robot"]
    names = {path.stem for path in (directory / "configs").glob("*.env") if path.is_file() and not path.is_symlink()}
    names.add(location["profile"])
    selected = configuration_location(root=root, robot=location["robot"])
    robots = sorted(path.name for path in (root / "drivers").iterdir()
                    if path.is_dir() and not path.is_symlink() and (path / "manifest.py").is_file()
                    and (path / "assembly.py").is_file())
    active = configuration_location(root=root)
    return {"robot": location["robot"], "profile": location["profile"],
            "profiles": sorted(names), "robots": robots, "selected_profile": selected["profile"],
            "selected_robot": active["robot"] if active else "",
            "path": str(location["path"]), "revision": revision(location, root=root, revision_owner=revision_owner, snapshot=snapshot) if revision_owner else None,
            "migration_required": not snapshot.profile_present if snapshot is not None else not location["path"].exists()}


def create_profile(*, root, robot, source, name, expected_revision=None, revision_owner=None):
    from core.config_io import _atomic_write, _target_lock

    location = configuration_location(root=root, robot=robot, profile=source)
    target = configuration_location(root=root, robot=robot, profile=name)
    with _target_lock(root / "runtime_data" / "settings" / "transaction"), checked_profile(
            location, expected_revision, root=root, revision_owner=revision_owner) as snapshot:
        if target["path"].exists() or target["profile"] == source:
            raise ValueError("同名配置已经存在，请使用其它名称")
        _atomic_write(target["path"], render_values(read_profile(location, root=root, snapshot=snapshot)))
    return profile_catalog(root=root, robot=robot, profile=name, revision_owner=revision_owner)


def activate_profile(*, root, robot, profile, expected_revision=None, revision_owner=None):
    from core import config_io as io

    location = configuration_location(root=root, robot=robot, profile=profile)
    with io._target_lock(root / "runtime_data" / "settings" / "transaction"), checked_profile(
            location, expected_revision, root=root, revision_owner=revision_owner) as snapshot:
        values = read_profile(location, root=root, snapshot=snapshot)
        # 手工编辑的方案仍须经过同一类型与算法契约校验；失效时不改活动选择。
        for key, value in values.items():
            # 共同取值器将空值解释为默认值，安装模板中的空数值也应能激活。
            if not value.strip():
                continue
            field = io.R.get_field(key)
            if field is not None and (io._validate(key, value, field.type) is None
                                     or (field.choices and value.strip() not in field.choices)):
                raise ValueError("方案中的配置无效：" + key)
        if values.get("NUWAX_GEOMETRY_PIPELINE", "").strip():
            from contracts.navigation_pipeline import resolve_navigation_profile
            expected = resolve_navigation_profile(values["NUWAX_GEOMETRY_PIPELINE"])
            if (values.get("NUWAX_NAV_PROFILE", "").strip() or expected) != expected:
                raise ValueError("配置方案中的导航与建图执行链不一致")
        ensure_profile(location, root=root, snapshot=snapshot)
        io._atomic_write(location["selector"], profile + "\n")
        active = root / "runtime_data" / "settings" / "active.env"
        active.parent.mkdir(parents=True, exist_ok=True)
        temporary = active.with_name(".active-" + uuid.uuid4().hex)
        try:
            # 受管发布的runtime_data和configs可指向持久目录，链接按物理父目录计算。
            temporary.symlink_to(os.path.relpath(location["path"].resolve(), active.parent.resolve()))
            os.replace(temporary, active)
        finally:
            if temporary.is_symlink():
                temporary.unlink()
    return profile_catalog(root=root, robot=robot, profile=profile, revision_owner=revision_owner)
