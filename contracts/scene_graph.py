"""空间语义属性图：节点、证据与候选路径共享地图身份。"""
from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import Field

from contracts.base import ContractModel
from contracts.geometry import Vector3


class SceneGraphNode(ContractModel):
    node_id: str
    kind: Literal["region", "passage", "object", "observation"]
    name: str
    record_id: Optional[str] = None
    position: Optional[Vector3] = None
    aliases: List[str] = Field(default_factory=list)
    artifact_ids: List[str] = Field(default_factory=list)
    properties: Dict[str, Any] = Field(default_factory=dict)


class SceneGraphEdge(ContractModel):
    edge_id: str
    kind: Literal["access", "contains", "observed_from"]
    source: str
    target: str
    directed: bool = False
    evidence: Dict[str, Any] = Field(default_factory=dict)


class SceneGraphSnapshot(ContractModel):
    map_binding: Dict[str, Any]
    graph_revision: str
    nodes: List[SceneGraphNode] = Field(default_factory=list)
    edges: List[SceneGraphEdge] = Field(default_factory=list)
    coverage: Dict[str, Any] = Field(default_factory=dict)
    navigation_checked: bool = False
    limitations: List[str] = Field(default_factory=list)
