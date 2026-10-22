from __future__ import annotations

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


def operation_envelope(request: pb.MotionGuardRequest) -> Optional[Message]:
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
