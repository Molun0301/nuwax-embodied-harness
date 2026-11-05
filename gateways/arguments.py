"""HTTP 与 MCP 的标准 JSON 解码约束。"""


def strict_json_constant(value: str) -> None:
    raise ValueError(f"Non-standard JSON constant: {value}")
