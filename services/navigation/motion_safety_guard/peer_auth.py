from __future__ import annotations

from dataclasses import dataclass
import pwd
import socket
import struct
from typing import Mapping


class AuthorizationError(PermissionError):
    pass


@dataclass(frozen=True)
class PeerCredentials:
    pid: int
    uid: int
    gid: int


SOURCE_ALLOWLIST_BY_USER: Mapping[str, frozenset[str]] = {
    "nuwax-motion-guard": frozenset(),
    "nuwax-motion-admin": frozenset({"ADMIN"}),
    "nuwax-geometry": frozenset({"NAV"}),
    "nuwax-gateway": frozenset(
        {
            "GATEWAY_MANUAL",
            "GATEWAY_FOLLOW",
            "GATEWAY_RELATIVE",
            "GATEWAY_SPORT",
        }
    ),
}


def get_peer_credentials(sock: socket.socket) -> PeerCredentials:
    if not hasattr(socket, "SO_PEERCRED"):
        raise AuthorizationError("SO_PEERCRED is unavailable on this platform")
    raw = sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
    pid, uid, gid = struct.unpack("3i", raw)
    return PeerCredentials(pid=pid, uid=uid, gid=gid)


def system_uid_source_allowlist() -> dict[int, frozenset[str]]:
    result: dict[int, frozenset[str]] = {}
    missing: list[str] = []
    for username, sources in SOURCE_ALLOWLIST_BY_USER.items():
        try:
            uid = pwd.getpwnam(username).pw_uid
        except KeyError:
            missing.append(username)
            continue
        result[uid] = sources
    if missing:
        raise AuthorizationError(
            "required service identities are not provisioned: " + ", ".join(missing)
        )
    return result


class PeerAuthorizer:
    def __init__(self, source_allowlist_by_uid: Mapping[int, frozenset[str]]) -> None:
        self._source_allowlist_by_uid = {
            int(uid): frozenset(sources)
            for uid, sources in source_allowlist_by_uid.items()
        }

    @classmethod
    def from_system_identities(cls) -> "PeerAuthorizer":
        return cls(system_uid_source_allowlist())

    def authorize(self, uid: int, source: str) -> None:
        allowed_sources = self._source_allowlist_by_uid.get(int(uid))
        if allowed_sources is None:
            raise AuthorizationError(f"peer UID {uid} is not authorized")
        normalized = str(source).removeprefix("GUARD_SOURCE_")
        if not normalized or normalized in {"UNSPECIFIED", "UNKNOWN"}:
            raise AuthorizationError("claimed source is unspecified")
        if normalized not in allowed_sources:
            raise AuthorizationError(
                f"peer UID {uid} is not authorized for source {normalized}"
            )
