"""全部入口共用的能力定义。"""
from dataclasses import dataclass
from typing import Callable, Dict, FrozenSet, List, Literal, Optional, Tuple

CapabilityKind = Literal["query", "action", "task"]


@dataclass(frozen=True)
class Capability:
    name: str
    # 面向用户的中文标题；任务快照与网页显示都取这里，标识 name 只作次要信息。
    title: str
    description: str
    input_schema: dict
    kind: CapabilityKind
    handler: Callable[..., dict]
    needs: Tuple[str, ...] = ()
    motion: bool = False
    agent_tool: bool = True
    timeout_sec: Optional[float] = None
    robots: Optional[FrozenSet[str]] = None
    normalize_names: Tuple[str, ...] = ()
    before_submit: Optional[Callable] = None


class CapabilityTable:
    def __init__(self) -> None:
        self._items: Dict[str, Capability] = {}

    def register(self, capability: Capability) -> None:
        if capability.name in self._items:
            raise ValueError(f"能力重复：{capability.name}")
        self._items[capability.name] = capability

    def get(self, name: str) -> Optional[Capability]:
        return self._items.get(name)

    def for_robot(self, robot_name: str) -> List[Capability]:
        return [cap for cap in self._items.values() if cap.robots is None or robot_name in cap.robots]

    def agent_tools(self, robot_name: str) -> List[Capability]:
        return [cap for cap in self.for_robot(robot_name) if cap.agent_tool]
