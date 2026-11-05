"""全部入口共用的能力定义。"""
from dataclasses import dataclass
from typing import Callable, Literal, Optional

CapabilityKind = Literal["query", "action", "task"]


@dataclass(frozen=True)
class Capability:
    name: str
    description: str
    input_schema: dict
    kind: CapabilityKind
    handler: Callable[..., dict]
    needs: tuple[str, ...] = ()
    motion: bool = False
    agent_tool: bool = True
    timeout_sec: Optional[float] = None
    robots: Optional[frozenset[str]] = None


class CapabilityTable:
    def __init__(self) -> None:
        self._items: dict[str, Capability] = {}

    def register(self, capability: Capability) -> None:
        if capability.name in self._items:
            raise ValueError(f"能力重复：{capability.name}")
        self._items[capability.name] = capability

    def get(self, name: str) -> Optional[Capability]:
        return self._items.get(name)

    def for_robot(self, robot_name: str) -> list[Capability]:
        return [cap for cap in self._items.values() if cap.robots is None or robot_name in cap.robots]

    def agent_tools(self, robot_name: str) -> list[Capability]:
        return [cap for cap in self.for_robot(robot_name) if cap.agent_tool]
