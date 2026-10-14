from __future__ import annotations

import socket
import struct
from typing import List

from .constants import MAX_FRAME_BYTES


class FramingError(RuntimeError):
    pass


class FrameTooLarge(FramingError):
    pass


class TruncatedFrame(FramingError):
    pass


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks: List[bytes] = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise TruncatedFrame(
                f"stream ended with {remaining} of {size} bytes missing"
            )
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv_frame(
    sock: socket.socket,
    *,
    max_frame_bytes: int = MAX_FRAME_BYTES,
) -> bytes:
    prefix = _recv_exact(sock, 4)
    (length,) = struct.unpack(">I", prefix)
    if length == 0:
        raise TruncatedFrame("zero-length protobuf frame is invalid")
    if length > max_frame_bytes:
        raise FrameTooLarge(
            f"frame length {length} exceeds maximum {max_frame_bytes}"
        )
    return _recv_exact(sock, length)


def send_frame(
    sock: socket.socket,
    payload: bytes,
    *,
    max_frame_bytes: int = MAX_FRAME_BYTES,
) -> None:
    length = len(payload)
    if length == 0:
        raise TruncatedFrame("zero-length protobuf frame is invalid")
    if length > max_frame_bytes:
        raise FrameTooLarge(
            f"frame length {length} exceeds maximum {max_frame_bytes}"
        )
    sock.sendall(struct.pack(">I", length) + payload)
