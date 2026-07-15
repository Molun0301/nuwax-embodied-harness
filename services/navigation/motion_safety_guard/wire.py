from __future__ import annotations

import hashlib
import hmac
from typing import Optional

from google.protobuf.message import Message

from services.navigation.proto_gen import motion_guard_pb2 as pb


class WireContractError(ValueError):
    pass


def operation_message(request: pb.MotionGuardRequest) -> Message:
    operation = request.WhichOneof("operation")
    if operation is None:
        raise WireContractError("request operation is missing")
    return getattr(request, operation)


def operation_envelope(request: pb.MotionGuardRequest) -> Optional[pb.RequestEnvelope]:
    message = operation_message(request)
    if "envelope" not in message.DESCRIPTOR.fields_by_name:
        return None
    return message.envelope


def canonical_payload(request: pb.MotionGuardRequest) -> bytes:
    message = operation_message(request)
    if "envelope" not in message.DESCRIPTOR.fields_by_name:
        return b""
    payload = message.__class__()
    payload.CopyFrom(message)
    payload.ClearField("envelope")
    return payload.SerializeToString(deterministic=True)


def payload_sha256(request: pb.MotionGuardRequest) -> bytes:
    return hashlib.sha256(canonical_payload(request)).digest()


def seal_request(request: pb.MotionGuardRequest) -> pb.MotionGuardRequest:
    envelope = operation_envelope(request)
    if envelope is None:
        return request
    envelope.payload_sha256 = payload_sha256(request)
    return request


def verify_payload_sha256(request: pb.MotionGuardRequest) -> None:
    envelope = operation_envelope(request)
    if envelope is None:
        return
    actual = bytes(envelope.payload_sha256)
    if len(actual) != 32:
        raise WireContractError("payload_sha256 must be exactly 32 bytes")
    if not hmac.compare_digest(actual, payload_sha256(request)):
        raise WireContractError("payload_sha256 does not match canonical typed payload")


def source_name(source: int) -> str:
    try:
        name = pb.GuardSource.Name(source)
    except ValueError as exc:
        raise WireContractError(f"unknown GuardSource value {source}") from exc
    prefix = "GUARD_SOURCE_"
    return name[len(prefix) :] if name.startswith(prefix) else name


def request_source_name(request: pb.MotionGuardRequest) -> str:
    envelope = operation_envelope(request)
    if envelope is not None:
        return source_name(envelope.source)
    message = operation_message(request)
    return source_name(message.source)
