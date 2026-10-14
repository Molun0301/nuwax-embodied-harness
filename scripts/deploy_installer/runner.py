from __future__ import annotations

import os
import pwd
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Mapping, Optional, Sequence


class CommandError(RuntimeError):
    pass


def resolve_invoking_account():
    if os.geteuid() != 0:
        return pwd.getpwuid(os.geteuid())
    raw_uid = os.environ.get("SUDO_UID", "0").strip()
    if not raw_uid.isdigit() or int(raw_uid) == 0:
        return pwd.getpwuid(0)
    try:
        account = pwd.getpwuid(int(raw_uid))
    except KeyError as exc:
        raise CommandError(f"cannot resolve invoking uid {raw_uid}") from exc
    sudo_user = os.environ.get("SUDO_USER", "").strip()
    if sudo_user and sudo_user != account.pw_name:
        raise CommandError(f"SUDO_USER={sudo_user} does not match SUDO_UID={raw_uid}")
    sudo_gid = os.environ.get("SUDO_GID", "").strip()
    if sudo_gid and (not sudo_gid.isdigit() or int(sudo_gid) != account.pw_gid):
        raise CommandError(f"SUDO_GID={sudo_gid} does not match the invoking account")
    return account


def source_user_command(argv: Sequence[str]) -> List[str]:
    command = [str(item) for item in argv]
    account = resolve_invoking_account()
    if os.geteuid() == 0 and account.pw_uid != 0:
        return [
            "/usr/sbin/runuser",
            "--preserve-environment",
            "-u",
            account.pw_name,
            "--",
            *command,
        ]
    return command


def source_user_environment() -> Mapping[str, str]:
    account = resolve_invoking_account()
    return {
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_OPTIONAL_LOCKS": "0",
        "HOME": account.pw_dir,
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "LOGNAME": account.pw_name,
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "USER": account.pw_name,
    }


def resolve_build_account():
    if os.geteuid() != 0:
        return pwd.getpwuid(os.geteuid())
    user = os.environ.get("SUDO_USER", "").strip()
    if user and user != "root":
        try:
            return pwd.getpwnam(user)
        except KeyError as exc:
            raise CommandError(f"cannot resolve SUDO_USER={user}") from exc
    try:
        return pwd.getpwnam("nobody")
    except KeyError as exc:
        raise CommandError("direct-root deployment requires the system nobody account") from exc


class Runner:
    def run(
        self,
        argv: Sequence[str],
        *,
        cwd: Optional[Path] = None,
        env: Optional[Mapping[str, str]] = None,
        as_build_user: bool = False,
        network_isolated: bool = False,
        timeout: Optional[int] = None,
    ) -> None:
        raise NotImplementedError


class SubprocessRunner(Runner):
    def _build_user_prefix(self) -> List[str]:
        if os.geteuid() != 0:
            return []
        account = resolve_build_account()
        return ["runuser", "--preserve-environment", "-u", account.pw_name, "--"]

    def _build_user_environment(self) -> Mapping[str, str]:
        account = resolve_build_account()
        return {
            "HOME": account.pw_dir,
            "USER": account.pw_name,
            "LOGNAME": account.pw_name,
            "LANG": "C.UTF-8",
            "LC_ALL": "C.UTF-8",
            "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        }

    def _runtime_environment(self) -> Mapping[str, str]:
        account = pwd.getpwuid(os.geteuid())
        return {
            "HOME": account.pw_dir,
            "USER": account.pw_name,
            "LOGNAME": account.pw_name,
            "LANG": "C.UTF-8",
            "LC_ALL": "C.UTF-8",
            "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        }

    def run(
        self,
        argv: Sequence[str],
        *,
        cwd: Optional[Path] = None,
        env: Optional[Mapping[str, str]] = None,
        as_build_user: bool = False,
        network_isolated: bool = False,
        timeout: Optional[int] = None,
    ) -> None:
        command = list(argv)
        if as_build_user:
            command = self._build_user_prefix() + command
        if network_isolated:
            if os.geteuid() != 0:
                raise CommandError("network-isolated commands require root")
            command = ["unshare", "--net", "--", *command]
        merged_env = dict(self._build_user_environment()) if as_build_user else dict(self._runtime_environment())
        if env:
            merged_env.update({str(key): str(value) for key, value in env.items()})
        completed = subprocess.run(
            command,
            cwd=str(cwd) if cwd else None,
            env=merged_env,
            check=False,
            timeout=timeout,
        )
        if completed.returncode != 0:
            raise CommandError(f"command failed with exit {completed.returncode}: " + " ".join(command))


@dataclass
class RecordingRunner(Runner):
    executed: List[List[str]] = field(default_factory=list)

    def run(
        self,
        argv: Sequence[str],
        *,
        cwd: Optional[Path] = None,
        env: Optional[Mapping[str, str]] = None,
        as_build_user: bool = False,
        network_isolated: bool = False,
        timeout: Optional[int] = None,
    ) -> None:
        del cwd, env, as_build_user, network_isolated, timeout
        self.executed.append(list(argv))
