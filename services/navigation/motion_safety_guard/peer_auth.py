from __future__ import annotations

from dataclasses import dataclass
import pwd
import socket
import struct
from typing import Dict, FrozenSet, List, Mapping, Optional


class AuthorizationError(PermissionError):
    pass


@dataclass(frozen=True)
class PeerCredentials:
    pid: int
    uid: int
    gid: int


SOURCE_ALLOWLIST_BY_USER: Mapping[str, FrozenSet[str]] = {
    "nuwax-motion-guard": frozenset(),
    "nuwax-navigation": frozenset({"NAV"}),
    "nuwax-gateway": frozenset(
        {
            "GATEWAY_MANUAL",
            "GATEWAY_FOLLOW",
            "GATEWAY_RELATIVE",
            "GATEWAY_SPORT",
        }
    ),
}
SERVICE_IDENTITY_USERS = (
    "nuwax-motion-guard",
    "nuwax-navigation",
    "nuwax-geometry",
    "nuwax-gateway",
)


def get_peer_credentials(sock: socket.socket) -> PeerCredentials:
    if not hasattr(socket, "SO_PEERCRED"):
        raise AuthorizationError("SO_PEERCRED is unavailable on this platform")
    raw = sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
    pid, uid, gid = struct.unpack("3i", raw)
    return PeerCredentials(pid=pid, uid=uid, gid=gid)


def validate_service_identity_uids(service_uid_by_user: Mapping[str, int]) -> None:
    missing = [username for username in SERVICE_IDENTITY_USERS if username not in service_uid_by_user]
    if missing:
        raise AuthorizationError("required service identities are not provisioned: " + ", ".join(missing))
    users_by_uid: Dict[int, List[str]] = {}
    for username in SERVICE_IDENTITY_USERS:
        users_by_uid.setdefault(int(service_uid_by_user[username]), []).append(username)
    collisions = [names for names in users_by_uid.values() if len(names) > 1]
    if collisions:
        detail = "; ".join(", ".join(names) for names in collisions)
        raise AuthorizationError(f"service identity UID collision: {detail}")


def system_service_identity_uids() -> Dict[str, int]:
    result: Dict[str, int] = {}
    missing: List[str] = []
    for username in SERVICE_IDENTITY_USERS:
        try:
            uid = pwd.getpwnam(username).pw_uid
        except KeyError:
            missing.append(username)
            continue
        result[username] = int(uid)
    if missing:
        raise AuthorizationError("required service identities are not provisioned: " + ", ".join(missing))
    validate_service_identity_uids(result)
    return result


def system_uid_source_allowlist() -> Dict[int, FrozenSet[str]]:
    service_uid_by_user = system_service_identity_uids()
    return {
        service_uid_by_user[username]: sources
        for username, sources in SOURCE_ALLOWLIST_BY_USER.items()
    }


class PeerAuthorizer:
    def __init__(
        self,
        source_allowlist_by_uid: Mapping[int, FrozenSet[str]],
        *,
        service_uid_by_user: Optional[Mapping[str, int]] = None,
    ) -> None:
        if service_uid_by_user is not None:
            validate_service_identity_uids(service_uid_by_user)
        self._source_allowlist_by_uid = {
            int(uid): frozenset(sources) for uid, sources in source_allowlist_by_uid.items()
        }

    @classmethod
    def from_system_identities(cls) -> "PeerAuthorizer":
        service_uid_by_user = system_service_identity_uids()
        return cls(
            {
                service_uid_by_user[username]: sources
                for username, sources in SOURCE_ALLOWLIST_BY_USER.items()
            },
            service_uid_by_user=service_uid_by_user,
        )

    def authorize(self, uid: int, source: str) -> None:
        allowed_sources = self._source_allowlist_by_uid.get(int(uid))
        if allowed_sources is None:
            raise AuthorizationError(f"peer UID {uid} is not authorized")
        source_name = str(source)
        prefix = "GUARD_SOURCE_"
        normalized = source_name[len(prefix) :] if source_name.startswith(prefix) else source_name
        if not normalized or normalized in {"UNSPECIFIED", "UNKNOWN"}:
            raise AuthorizationError("claimed source is unspecified")
        if normalized not in allowed_sources:
            raise AuthorizationError(f"peer UID {uid} is not authorized for source {normalized}")
