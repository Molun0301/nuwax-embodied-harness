"""原生占据栅格的只读字节视图；公共序列化仍输出整数数组。"""

from collections.abc import Sequence
from dataclasses import dataclass
from functools import cached_property
from typing import List

import numpy as np
from pydantic import TypeAdapter
from pydantic_core import core_schema


_INTEGER_ARRAY = TypeAdapter(List[int])


@dataclass(frozen=True, eq=False)
class PackedOccupancyCells(Sequence):
    """拥有不可变载荷，构造时检查一次值域，内部流转不重复展开单元。"""

    raw: bytes

    def __post_init__(self):
        if not isinstance(self.raw, (bytes, bytearray, memoryview)):
            raise TypeError("栅格载荷必须是字节缓冲。")
        raw = bytes(self.raw)
        cells = np.frombuffer(raw, dtype=np.int8)
        if cells.size and (cells.min() < -1 or cells.max() > 100):
            raise ValueError("OccupancyGrid 的栅格取值必须在 -1 到 100 之间。")
        object.__setattr__(self, "raw", raw)

    def __len__(self):
        return len(self.raw)

    def __getitem__(self, index):
        value = memoryview(self.raw).cast("b")[index]
        return list(value) if isinstance(index, slice) else value

    def __iter__(self):
        return iter(memoryview(self.raw).cast("b"))

    def __eq__(self, other):
        if type(other) is PackedOccupancyCells:
            return self.raw == other.raw
        if isinstance(other, (list, tuple)):
            return self.to_list() == list(other)
        return NotImplemented

    def to_list(self):
        return np.frombuffer(self.raw, dtype=np.int8).tolist()

    def __deepcopy__(self, memo):
        # 显式深复制仍给调用者独立可编辑列表，与普通栅格复制的语义一致。
        result = self.to_list()
        memo[id(self)] = result
        return result

    @cached_property
    def canonical_json(self):
        # 整张大图的编译编码会长时间独占解释器，阻塞同进程的惯性采集。
        # 按固定单元数限制每次展开/编码的工作量；格式和缓存身份仍与原紧凑JSON一致。
        cells = np.frombuffer(self.raw, dtype=np.int8)
        chunk_cells = 16384
        if len(cells) <= chunk_cells:
            return _INTEGER_ARRAY.dump_json(cells.tolist())
        parts = [b"["]
        for start in range(0, len(cells), chunk_cells):
            if start:
                parts.append(b",")
            encoded = _INTEGER_ARRAY.dump_json(cells[start:start + chunk_cells].tolist())
            parts.append(encoded[1:-1])
        parts.append(b"]")
        return b"".join(parts)

    @classmethod
    def __get_pydantic_core_schema__(cls, source, handler):
        return core_schema.json_or_python_schema(
            json_schema=core_schema.list_schema(core_schema.int_schema(ge=-1, le=100)),
            python_schema=core_schema.is_instance_schema(cls),
            serialization=core_schema.plain_serializer_function_ser_schema(
                lambda value: value.to_list() if isinstance(value, cls) else value,
                return_schema=core_schema.list_schema(core_schema.int_schema()),
            ),
        )
