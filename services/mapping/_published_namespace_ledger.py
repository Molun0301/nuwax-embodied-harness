from __future__ import annotations

import hashlib
import json
import struct
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

_MAGIC = b"NWPLED1\0"
_PROTOCOL = "nuwax.mapping.published-namespace-ledger.v1"
_SCHEMA_VERSION = 1
_MAX_BODY_BYTES = 65536
_MAX_JSON_DEPTH = 32
_I64_MAX = (1 << 63) - 1
_ZERO_HASH = "0" * 64
_RECORD_DOMAIN = b"nuwax.mapping.published-namespace-ledger.record.v1\0"
_PAYLOAD_DOMAIN = b"nuwax.mapping.published-namespace-ledger.payload.v1\0"
_ROOT_DOMAIN = b"nuwax.mapping.published-namespace-ledger.root.v1\0"
_FRAME_FIXED_BYTES = 52
_HEX_DIGITS = frozenset("0123456789abcdef")
_RECORD_KEYS = frozenset(
    {
        "checkpoint",
        "ledger",
        "payload",
        "payload_sha256",
        "prev_record_hash",
        "protocol",
        "record_type",
        "root_id",
        "schema_version",
        "sequence",
    }
)
_IDENTITY_KEYS = frozenset({"device", "gid", "inode", "mode", "nlink", "uid"})
_LOCK_IDENTITY_KEYS = frozenset({"device", "gid", "inode", "mode", "nlink", "size", "uid"})
_EXACT_BINDING_KEYS = frozenset(
    {"canonical_sha256", "device", "gid", "inode", "mode", "nlink", "size", "uid"}
)
_GENESIS_PAYLOAD_KEYS = frozenset(
    {"lock_binding", "marker_binding", "root_binding", "state_binding"}
)
_CHECKPOINT_KEYS = frozenset({"committed_prefix_sha256", "committed_size"})
_CHECKPOINT_PAYLOAD_KEYS = frozenset({"committed_record_hash", "committed_sequence"})
_ENABLED_RECORD_TYPES = frozenset({"CHECKPOINT", "GENESIS"})


class PublishedNamespaceLedgerError(ValueError):
    """ledger wire 数据不满足冻结协议。"""

    def __init__(self, code: str, message: str, *, offset: Optional[int] = None) -> None:
        self.code = code
        self.offset = offset
        super().__init__(message)


@dataclass(frozen=True)
class DecodedLedgerRecord:
    body: Mapping[str, Any]
    body_bytes: bytes
    frame_bytes: bytes
    offset: int
    record_hash: str

    @property
    def sequence(self) -> int:
        return int(self.body["sequence"])

    @property
    def record_type(self) -> str:
        return str(self.body["record_type"])


@dataclass(frozen=True)
class LedgerReplay:
    records: Tuple[DecodedLedgerRecord, ...]
    committed_size: int
    committed_prefix_sha256: str
    head_record_hash: str
    head_sequence: int


def canonical_json_bytes(value: Any) -> bytes:
    """以 ledger v1 唯一 JSON 表示编码已验证的数据。"""

    _validate_json_value(value, path="$", depth=0)
    try:
        return json.dumps(
            value,
            ensure_ascii=True,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("ascii")
    except (TypeError, ValueError, UnicodeError) as exc:
        raise PublishedNamespaceLedgerError(
            "canonical_json_invalid",
            "ledger JSON 无法编码为唯一 canonical bytes。",
        ) from exc


def make_root_id(
    *,
    root_binding: Mapping[str, Any],
    lock_binding: Mapping[str, Any],
    ledger_binding: Mapping[str, Any],
) -> str:
    """计算 genesis 后不可变化的 root/lock/ledger 初始绑定。"""

    _validate_identity(root_binding, "root_binding")
    _validate_identity(lock_binding, "lock_binding", require_zero_size=True)
    _validate_identity(ledger_binding, "ledger_binding")
    binding = {
        "ledger_binding": dict(ledger_binding),
        "lock_binding": dict(lock_binding),
        "root_binding": dict(root_binding),
    }
    return "sha256:" + hashlib.sha256(_ROOT_DOMAIN + canonical_json_bytes(binding)).hexdigest()


def encode_genesis_record(
    *,
    ledger_binding: Mapping[str, Any],
    lock_binding: Mapping[str, Any],
    marker_binding: Mapping[str, Any],
    root_binding: Mapping[str, Any],
    state_binding: Mapping[str, Any],
) -> bytes:
    """编码 sequence=1 的唯一 genesis 帧。"""

    payload = {
        "lock_binding": dict(lock_binding),
        "marker_binding": dict(marker_binding),
        "root_binding": dict(root_binding),
        "state_binding": dict(state_binding),
    }
    root_id = make_root_id(
        root_binding=root_binding,
        lock_binding=lock_binding,
        ledger_binding=ledger_binding,
    )
    return encode_record(
        record_type="GENESIS",
        sequence=1,
        prev_record_hash=_ZERO_HASH,
        root_id=root_id,
        ledger_binding=ledger_binding,
        payload=payload,
        checkpoint=None,
    )


def encode_checkpoint_record(
    *,
    committed_prefix: bytes,
    committed_record_hash: str,
    committed_sequence: int,
    ledger_binding: Mapping[str, Any],
    prev_record_hash: str,
    root_id: str,
    sequence: int,
) -> bytes:
    """编码承认其前全部字节的 checkpoint 帧。"""

    if not isinstance(committed_prefix, bytes):
        _fail("checkpoint_prefix_type", "committed_prefix 必须是 bytes。")
    checkpoint = {
        "committed_prefix_sha256": hashlib.sha256(committed_prefix).hexdigest(),
        "committed_size": len(committed_prefix),
    }
    payload = {
        "committed_record_hash": committed_record_hash,
        "committed_sequence": committed_sequence,
    }
    return encode_record(
        record_type="CHECKPOINT",
        sequence=sequence,
        prev_record_hash=prev_record_hash,
        root_id=root_id,
        ledger_binding=ledger_binding,
        payload=payload,
        checkpoint=checkpoint,
    )


def encode_record(
    *,
    record_type: str,
    sequence: int,
    prev_record_hash: str,
    root_id: str,
    ledger_binding: Mapping[str, Any],
    payload: Mapping[str, Any],
    checkpoint: Optional[Mapping[str, Any]],
) -> bytes:
    """编码一帧；当前版本只开放 GENESIS 与 CHECKPOINT。"""

    payload_dict = dict(payload)
    body = {
        "checkpoint": None if checkpoint is None else dict(checkpoint),
        "ledger": dict(ledger_binding),
        "payload": payload_dict,
        "payload_sha256": hashlib.sha256(
            _PAYLOAD_DOMAIN + canonical_json_bytes(payload_dict)
        ).hexdigest(),
        "prev_record_hash": prev_record_hash,
        "protocol": _PROTOCOL,
        "record_type": record_type,
        "root_id": root_id,
        "schema_version": _SCHEMA_VERSION,
        "sequence": sequence,
    }
    _validate_record_body(body)
    body_bytes = canonical_json_bytes(body)
    if not body_bytes or len(body_bytes) > _MAX_BODY_BYTES:
        _fail("body_length", "ledger body 长度超出 1..65536。")
    length_bytes = struct.pack(">I", len(body_bytes))
    crc_bytes = struct.pack(">I", _crc32c(body_bytes))
    record_hash = hashlib.sha256(
        _RECORD_DOMAIN + _MAGIC + length_bytes + body_bytes + crc_bytes
    ).digest()
    frame_len = _FRAME_FIXED_BYTES + len(body_bytes)
    return (
        _MAGIC
        + length_bytes
        + body_bytes
        + crc_bytes
        + record_hash
        + struct.pack(">I", frame_len)
    )


def decode_record(data: bytes, *, offset: int = 0) -> Tuple[DecodedLedgerRecord, int]:
    """从 offset 严格解析一帧，不忽略其后的字节。"""

    if not isinstance(data, bytes):
        _fail("input_type", "ledger 输入必须是 bytes。", offset=offset)
    if isinstance(offset, bool) or not isinstance(offset, int) or offset < 0:
        _fail("offset_invalid", "ledger offset 必须是非负整数。")
    available = len(data) - offset
    if available < 12:
        _fail("torn_header", "ledger 帧头不完整。", offset=offset)
    if data[offset : offset + 8] != _MAGIC:
        _fail("magic_mismatch", "ledger magic 不匹配。", offset=offset)
    body_len = struct.unpack(">I", data[offset + 8 : offset + 12])[0]
    if body_len == 0 or body_len > _MAX_BODY_BYTES:
        _fail("body_length", "ledger body 长度超出 1..65536。", offset=offset)
    frame_len = _FRAME_FIXED_BYTES + body_len
    end = offset + frame_len
    if end > len(data):
        _fail("torn_frame", "ledger 帧在 EOF 前截断。", offset=offset)
    body_start = offset + 12
    body_end = body_start + body_len
    body_bytes = data[body_start:body_end]
    crc_bytes = data[body_end : body_end + 4]
    stored_crc = struct.unpack(">I", crc_bytes)[0]
    if stored_crc != _crc32c(body_bytes):
        _fail("crc32c_mismatch", "ledger body CRC32C 不匹配。", offset=offset)
    stored_hash = data[body_end + 4 : body_end + 36]
    expected_hash = hashlib.sha256(
        _RECORD_DOMAIN
        + _MAGIC
        + data[offset + 8 : offset + 12]
        + body_bytes
        + crc_bytes
    ).digest()
    if stored_hash != expected_hash:
        _fail("record_hash_mismatch", "ledger record hash 不匹配。", offset=offset)
    footer_len = struct.unpack(">I", data[body_end + 36 : end])[0]
    if footer_len != frame_len:
        _fail("frame_length_mismatch", "ledger footer frame_len 不匹配。", offset=offset)
    body = _decode_canonical_object(body_bytes, offset=offset)
    _validate_record_body(body, offset=offset)
    record = DecodedLedgerRecord(
        body=body,
        body_bytes=body_bytes,
        frame_bytes=data[offset:end],
        offset=offset,
        record_hash=stored_hash.hex(),
    )
    return record, end


def replay_ledger(data: bytes) -> LedgerReplay:
    """从 byte 0 重放完整 ledger；任何未知或 torn tail 都 fail closed。"""

    if not isinstance(data, bytes):
        _fail("input_type", "ledger 输入必须是 bytes。")
    if not data:
        _fail("ledger_empty", "ledger 不能为空。", offset=0)
    records: List[DecodedLedgerRecord] = []
    offset = 0
    previous_hash = _ZERO_HASH
    previous_sequence = 0
    root_id: Optional[str] = None
    ledger_binding: Optional[Mapping[str, Any]] = None
    while offset < len(data):
        record, next_offset = decode_record(data, offset=offset)
        if records and records[-1].record_type == "CHECKPOINT":
            _fail(
                "record_after_checkpoint",
                "冻结 ledger 的 CHECKPOINT 必须是 terminal record。",
                offset=offset,
            )
        body = record.body
        sequence = record.sequence
        if not records:
            if record.record_type != "GENESIS" or sequence != 1:
                _fail("genesis_required", "ledger 第一帧必须是 sequence=1 的 GENESIS。", offset=offset)
            root_id = str(body["root_id"])
            ledger_binding = body["ledger"]
        else:
            if record.record_type == "GENESIS":
                _fail("duplicate_genesis", "ledger 只能包含一个 GENESIS。", offset=offset)
            if root_id != body["root_id"]:
                _fail("root_id_mismatch", "ledger root_id 在链中发生变化。", offset=offset)
            if ledger_binding != body["ledger"]:
                _fail("ledger_binding_mismatch", "ledger inode 绑定在链中发生变化。", offset=offset)
        if sequence != previous_sequence + 1 or sequence > _I64_MAX:
            _fail("sequence_mismatch", "ledger sequence 必须严格递增且不得回绕。", offset=offset)
        if body["prev_record_hash"] != previous_hash:
            _fail("prev_record_hash_mismatch", "ledger prev_record_hash 断链。", offset=offset)
        if record.record_type == "CHECKPOINT":
            _validate_checkpoint_against_prefix(record, data[:offset], records[-1] if records else None)
        previous_hash = record.record_hash
        previous_sequence = sequence
        records.append(record)
        offset = next_offset
    if records[-1].record_type != "CHECKPOINT":
        _fail(
            "uncheckpointed_tail",
            "ledger EOF 必须精确落在有效 CHECKPOINT 帧后。",
            offset=records[-1].offset,
        )
    checkpoint = records[-1].body["checkpoint"]
    assert isinstance(checkpoint, Mapping)
    return LedgerReplay(
        records=tuple(records),
        committed_size=int(checkpoint["committed_size"]),
        committed_prefix_sha256=str(checkpoint["committed_prefix_sha256"]),
        head_record_hash=records[-1].record_hash,
        head_sequence=records[-1].sequence,
    )


def _decode_canonical_object(body_bytes: bytes, *, offset: int) -> Mapping[str, Any]:
    try:
        _validate_json_nesting(body_bytes, offset=offset)
        text = body_bytes.decode("utf-8")
    except PublishedNamespaceLedgerError:
        raise
    except (MemoryError, RecursionError) as exc:
        raise PublishedNamespaceLedgerError(
            "json_resource_limit",
            "ledger JSON 解码超出资源限制。",
            offset=offset,
        ) from exc
    except UnicodeDecodeError as exc:
        raise PublishedNamespaceLedgerError(
            "body_utf8_invalid",
            "ledger body 不是合法 UTF-8。",
            offset=offset,
        ) from exc

    def reject_constant(value: str) -> Any:
        raise PublishedNamespaceLedgerError(
            "json_constant_forbidden",
            "ledger JSON 禁止 NaN/Infinity。",
            offset=offset,
        )

    def unique_object(pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
        result: Dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                _fail("duplicate_json_key", "ledger JSON 包含重复 key。", offset=offset)
            result[key] = value
        return result

    try:
        value = json.loads(
            text,
            object_pairs_hook=unique_object,
            parse_constant=reject_constant,
        )
    except PublishedNamespaceLedgerError:
        raise
    except (MemoryError, RecursionError) as exc:
        raise PublishedNamespaceLedgerError(
            "json_resource_limit",
            "ledger JSON 解码超出资源限制。",
            offset=offset,
        ) from exc
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise PublishedNamespaceLedgerError(
            "json_invalid",
            "ledger body 不是合法 JSON。",
            offset=offset,
        ) from exc
    if not isinstance(value, dict):
        _fail("record_type_invalid", "ledger body 必须是 JSON object。", offset=offset)
    try:
        canonical_bytes = canonical_json_bytes(value)
    except PublishedNamespaceLedgerError as exc:
        raise PublishedNamespaceLedgerError(exc.code, str(exc), offset=offset) from exc
    except (MemoryError, RecursionError) as exc:
        raise PublishedNamespaceLedgerError(
            "json_resource_limit",
            "ledger JSON canonical 校验超出资源限制。",
            offset=offset,
        ) from exc
    if canonical_bytes != body_bytes:
        _fail("json_noncanonical", "ledger body 不是唯一 canonical JSON。", offset=offset)
    return value


def _validate_json_nesting(body_bytes: bytes, *, offset: int) -> None:
    depth = 0
    in_string = False
    escaped = False
    for byte in body_bytes:
        if in_string:
            if escaped:
                escaped = False
            elif byte == 0x5C:
                escaped = True
            elif byte == 0x22:
                in_string = False
            continue
        if byte == 0x22:
            in_string = True
        elif byte in (0x5B, 0x7B):
            depth += 1
            if depth > _MAX_JSON_DEPTH:
                _fail("json_depth", "ledger JSON 嵌套深度超限。", offset=offset)
        elif byte in (0x5D, 0x7D) and depth > 0:
            depth -= 1


def _validate_record_body(body: Mapping[str, Any], *, offset: Optional[int] = None) -> None:
    _require_exact_keys(body, _RECORD_KEYS, "record", offset=offset)
    if body["protocol"] != _PROTOCOL:
        _fail("protocol_unknown", "ledger protocol/domain 未知。", offset=offset)
    if body["schema_version"] != _SCHEMA_VERSION or isinstance(body["schema_version"], bool):
        _fail("schema_version_unknown", "ledger schema_version 未知。", offset=offset)
    record_type = body["record_type"]
    if record_type not in _ENABLED_RECORD_TYPES:
        _fail("record_type_unknown", "ledger record_type 未启用。", offset=offset)
    sequence = _require_int(body["sequence"], "sequence", minimum=1, maximum=_I64_MAX, offset=offset)
    _require_hash(body["prev_record_hash"], "prev_record_hash", offset=offset)
    _require_root_id(body["root_id"], offset=offset)
    _validate_identity(body["ledger"], "ledger", offset=offset)
    payload = body["payload"]
    if not isinstance(payload, dict):
        _fail("payload_type", "ledger payload 必须是 object。", offset=offset)
    expected_payload_hash = hashlib.sha256(
        _PAYLOAD_DOMAIN + canonical_json_bytes(payload)
    ).hexdigest()
    _require_hash(body["payload_sha256"], "payload_sha256", offset=offset)
    if body["payload_sha256"] != expected_payload_hash:
        _fail("payload_hash_mismatch", "ledger payload_sha256 不匹配。", offset=offset)
    if record_type == "GENESIS":
        if sequence != 1 or body["prev_record_hash"] != _ZERO_HASH:
            _fail("genesis_identity", "GENESIS 必须使用 sequence=1 与零前序哈希。", offset=offset)
        if body["checkpoint"] is not None:
            _fail("genesis_checkpoint", "GENESIS checkpoint 必须为 null。", offset=offset)
        _validate_genesis_payload(payload, body["ledger"], body["root_id"], offset=offset)
    else:
        _validate_checkpoint(body["checkpoint"], offset=offset)
        _require_exact_keys(payload, _CHECKPOINT_PAYLOAD_KEYS, "checkpoint payload", offset=offset)
        _require_hash(payload["committed_record_hash"], "committed_record_hash", offset=offset)
        committed_sequence = _require_int(
            payload["committed_sequence"],
            "committed_sequence",
            minimum=1,
            maximum=_I64_MAX,
            offset=offset,
        )
        if committed_sequence >= sequence:
            _fail("checkpoint_sequence", "CHECKPOINT 必须承认更早的 sequence。", offset=offset)


def _validate_genesis_payload(
    payload: Mapping[str, Any],
    ledger_binding: Mapping[str, Any],
    root_id: str,
    *,
    offset: Optional[int],
) -> None:
    _require_exact_keys(payload, _GENESIS_PAYLOAD_KEYS, "genesis payload", offset=offset)
    _validate_identity(payload["root_binding"], "root_binding", offset=offset)
    _validate_identity(payload["lock_binding"], "lock_binding", require_zero_size=True, offset=offset)
    _validate_exact_binding(payload["marker_binding"], "marker_binding", offset=offset)
    _validate_exact_binding(payload["state_binding"], "state_binding", offset=offset)
    expected_root_id = make_root_id(
        root_binding=payload["root_binding"],
        lock_binding=payload["lock_binding"],
        ledger_binding=ledger_binding,
    )
    if root_id != expected_root_id:
        _fail("root_id_mismatch", "GENESIS root_id 与初始绑定不一致。", offset=offset)


def _validate_checkpoint(value: Any, *, offset: Optional[int]) -> None:
    if not isinstance(value, dict):
        _fail("checkpoint_type", "CHECKPOINT checkpoint 必须是 object。", offset=offset)
    _require_exact_keys(value, _CHECKPOINT_KEYS, "checkpoint", offset=offset)
    _require_int(value["committed_size"], "committed_size", minimum=1, maximum=_I64_MAX, offset=offset)
    _require_hash(value["committed_prefix_sha256"], "committed_prefix_sha256", offset=offset)


def _validate_checkpoint_against_prefix(
    record: DecodedLedgerRecord,
    prefix: bytes,
    previous: Optional[DecodedLedgerRecord],
) -> None:
    if previous is None:
        _fail("checkpoint_without_record", "CHECKPOINT 前必须紧邻被承认记录。", offset=record.offset)
    checkpoint = record.body["checkpoint"]
    payload = record.body["payload"]
    assert isinstance(checkpoint, Mapping)
    assert isinstance(payload, Mapping)
    if checkpoint["committed_size"] != record.offset or record.offset != len(prefix):
        _fail("checkpoint_size_mismatch", "CHECKPOINT committed_size 不等于帧起始 offset。", offset=record.offset)
    if checkpoint["committed_prefix_sha256"] != hashlib.sha256(prefix).hexdigest():
        _fail("checkpoint_prefix_hash_mismatch", "CHECKPOINT prefix hash 不匹配。", offset=record.offset)
    assert previous is not None
    if payload["committed_record_hash"] != previous.record_hash:
        _fail("checkpoint_record_hash_mismatch", "CHECKPOINT 未指向紧邻前一帧。", offset=record.offset)
    if payload["committed_sequence"] != previous.sequence:
        _fail("checkpoint_record_sequence_mismatch", "CHECKPOINT sequence 未指向紧邻前一帧。", offset=record.offset)


def _validate_identity(
    value: Any,
    name: str,
    *,
    require_zero_size: bool = False,
    offset: Optional[int] = None,
) -> None:
    if not isinstance(value, dict):
        _fail("identity_type", "%s 必须是 object。" % name, offset=offset)
    expected_keys = _LOCK_IDENTITY_KEYS if require_zero_size else _IDENTITY_KEYS
    _require_exact_keys(value, expected_keys, name, offset=offset)
    for key in expected_keys:
        minimum = 1 if key in {"device", "inode", "nlink"} else 0
        _require_int(value[key], "%s.%s" % (name, key), minimum=minimum, maximum=_I64_MAX, offset=offset)
    if require_zero_size and value["size"] != 0:
        _fail("identity_size", "%s.size 必须为 0。" % name, offset=offset)


def _validate_exact_binding(value: Any, name: str, *, offset: Optional[int]) -> None:
    if not isinstance(value, dict):
        _fail("binding_type", "%s 必须是 object。" % name, offset=offset)
    _require_exact_keys(value, _EXACT_BINDING_KEYS, name, offset=offset)
    for key in _EXACT_BINDING_KEYS - {"canonical_sha256"}:
        minimum = 1 if key in {"device", "inode", "nlink"} else 0
        _require_int(value[key], "%s.%s" % (name, key), minimum=minimum, maximum=_I64_MAX, offset=offset)
    _require_hash(value["canonical_sha256"], "%s.canonical_sha256" % name, offset=offset)


def _validate_json_value(value: Any, *, path: str, depth: int) -> None:
    if depth > 32:
        _fail("json_depth", "ledger JSON 嵌套深度超限。")
    if value is None or isinstance(value, (str, bool)):
        return
    if isinstance(value, int):
        if value < 0 or value > _I64_MAX:
            _fail("json_integer_range", "%s 整数超出 0..I64_MAX。" % path)
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_json_value(item, path="%s[%d]" % (path, index), depth=depth + 1)
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                _fail("json_key_type", "%s 包含非字符串 key。" % path)
            _validate_json_value(item, path="%s.%s" % (path, key), depth=depth + 1)
        return
    _fail("json_type_forbidden", "%s 包含禁止的 JSON 类型。" % path)


def _require_exact_keys(
    value: Mapping[str, Any],
    expected: frozenset,
    name: str,
    *,
    offset: Optional[int],
) -> None:
    actual = frozenset(value.keys())
    if actual != expected:
        _fail("unknown_or_missing_keys", "%s 字段集合不匹配。" % name, offset=offset)


def _require_hash(value: Any, name: str, *, offset: Optional[int]) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(char not in _HEX_DIGITS for char in value):
        _fail("hash_invalid", "%s 必须是 64 位小写 hex。" % name, offset=offset)
    return value


def _require_root_id(value: Any, *, offset: Optional[int]) -> str:
    if not isinstance(value, str) or not value.startswith("sha256:"):
        _fail("root_id_invalid", "root_id 必须使用 sha256:<64hex>。", offset=offset)
    _require_hash(value[7:], "root_id", offset=offset)
    return value


def _require_int(
    value: Any,
    name: str,
    *,
    minimum: int,
    maximum: int,
    offset: Optional[int],
) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum or value > maximum:
        _fail("integer_range", "%s 超出允许整数范围。" % name, offset=offset)
    return value


def _crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def _fail(code: str, message: str, *, offset: Optional[int] = None) -> None:
    raise PublishedNamespaceLedgerError(code, message, offset=offset)
