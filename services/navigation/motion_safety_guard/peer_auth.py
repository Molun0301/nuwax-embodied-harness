from __future__ import annotations

from dataclasses import dataclass
import socket
import struct


class AuthorizationError(PermissionError):
    pass


@dataclass(frozen=True)
class PeerCredentials:
    """SO_PEERCRED metadata retained for transport diagnostics only."""

    pid: int
    uid: int
    gid: int


def get_peer_credentials(sock: socket.socket) -> PeerCredentials:
    if not hasattr(socket, "SO_PEERCRED"):
        raise AuthorizationError("SO_PEERCRED is unavailable on this platform")
    raw = sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
    pid, uid, gid = struct.unpack("3i", raw)
    return PeerCredentials(pid=pid, uid=uid, gid=gid)
