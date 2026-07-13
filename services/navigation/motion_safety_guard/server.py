from __future__ import annotations

from pathlib import Path
import os
import socket
import socketserver
import stat
import threading

from google.protobuf.message import DecodeError

from services.navigation.proto_gen import motion_guard_pb2 as pb

from .actor import GuardActor
from .constants import MAX_FRAME_BYTES
from .framing import FramingError, recv_frame, send_frame
from .peer_auth import get_peer_credentials


class _ThreadingUnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    block_on_close = True
    allow_reuse_address = False

    def __init__(self, *args, max_connections: int, **kwargs) -> None:
        self._connection_slots = threading.BoundedSemaphore(max_connections)
        super().__init__(*args, **kwargs)

    def process_request(self, request, client_address) -> None:
        if not self._connection_slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self._connection_slots.release()
            raise

    def process_request_thread(self, request, client_address) -> None:
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._connection_slots.release()


class _MotionGuardHandler(socketserver.BaseRequestHandler):
    server: "_BoundMotionGuardServer"

    def handle(self) -> None:
        connection: socket.socket = self.request
        connection.settimeout(None)
        peer = get_peer_credentials(connection)
        while True:
            try:
                payload = recv_frame(
                    connection,
                    max_frame_bytes=self.server.max_frame_bytes,
                )
                request = pb.MotionGuardRequest()
                request.ParseFromString(payload)
                response = self.server.actor.call(
                    request,
                    peer,
                    timeout_seconds=self.server.actor_timeout_seconds,
                )
                send_frame(
                    connection,
                    response.SerializeToString(deterministic=True),
                    max_frame_bytes=self.server.max_frame_bytes,
                )
            except (
                DecodeError,
                FramingError,
                OSError,
                RuntimeError,
                TimeoutError,
                ValueError,
            ):
                return


class _BoundMotionGuardServer(_ThreadingUnixServer):
    def __init__(
        self,
        socket_path: Path,
        actor: GuardActor,
        *,
        max_frame_bytes: int,
        connection_timeout_seconds: float,
        actor_timeout_seconds: float,
        max_connections: int,
    ) -> None:
        self.actor = actor
        self.max_frame_bytes = max_frame_bytes
        self.connection_timeout_seconds = connection_timeout_seconds
        self.actor_timeout_seconds = actor_timeout_seconds
        super().__init__(
            str(socket_path),
            _MotionGuardHandler,
            bind_and_activate=True,
            max_connections=max_connections,
        )


class MotionGuardUnixServer:
    def __init__(
        self,
        *,
        socket_path: Path,
        actor: GuardActor,
        socket_gid: int,
        max_frame_bytes: int = MAX_FRAME_BYTES,
        connection_timeout_seconds: float = 1.0,
        actor_timeout_seconds: float = 2.0,
        max_connections: int = 32,
    ) -> None:
        if max_connections <= 0:
            raise ValueError("max_connections must be positive")
        self.socket_path = Path(socket_path)
        self._actor = actor
        self._socket_gid = socket_gid
        self._max_frame_bytes = max_frame_bytes
        self._connection_timeout_seconds = connection_timeout_seconds
        self._actor_timeout_seconds = actor_timeout_seconds
        self._max_connections = max_connections
        self._server: _BoundMotionGuardServer | None = None
        self._thread: threading.Thread | None = None
        self._bound_identity: tuple[int, int] | None = None

    def start(self) -> None:
        if self._server is not None:
            return
        if not self.socket_path.parent.is_dir():
            raise RuntimeError(
                f"motion guard runtime directory is missing: {self.socket_path.parent}"
            )
        if self.socket_path.exists() or self.socket_path.is_symlink():
            raise RuntimeError(
                f"refusing to replace existing socket path: {self.socket_path}"
            )
        self._actor.start()
        try:
            server = _BoundMotionGuardServer(
                self.socket_path,
                self._actor,
                max_frame_bytes=self._max_frame_bytes,
                connection_timeout_seconds=self._connection_timeout_seconds,
                actor_timeout_seconds=self._actor_timeout_seconds,
                max_connections=self._max_connections,
            )
            os.chown(self.socket_path, os.geteuid(), self._socket_gid)
            os.chmod(self.socket_path, 0o660)
            metadata = os.lstat(self.socket_path)
            if not stat.S_ISSOCK(metadata.st_mode):
                raise RuntimeError("bound motion guard path is not a Unix socket")
            if metadata.st_uid != os.geteuid() or metadata.st_gid != self._socket_gid:
                raise RuntimeError("motion guard socket owner/group drift")
            if stat.S_IMODE(metadata.st_mode) != 0o660:
                raise RuntimeError("motion guard socket mode drift")
            self._bound_identity = (metadata.st_dev, metadata.st_ino)
            self._server = server
            thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.05},
                name="nuwax-motion-guard-uds",
                daemon=True,
            )
            self._thread = thread
            thread.start()
        except BaseException:
            self._safe_unlink_bound_socket()
            self._actor.stop()
            raise

    def stop(self) -> None:
        server, self._server = self._server, None
        thread, self._thread = self._thread, None
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2.0)
            if thread.is_alive():
                raise RuntimeError("motion guard UDS server did not stop")
        fence_error: BaseException | None = None
        if server is not None:
            try:
                self._actor.fence()
            except BaseException as exc:
                fence_error = exc
        try:
            self._actor.stop()
        finally:
            self._safe_unlink_bound_socket()
        if fence_error is not None:
            raise fence_error

    def _safe_unlink_bound_socket(self) -> None:
        identity, self._bound_identity = self._bound_identity, None
        if identity is None:
            return
        try:
            metadata = os.lstat(self.socket_path)
        except FileNotFoundError:
            return
        if (
            stat.S_ISSOCK(metadata.st_mode)
            and (metadata.st_dev, metadata.st_ino) == identity
            and metadata.st_uid == os.geteuid()
        ):
            self.socket_path.unlink()
