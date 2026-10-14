"""单批次三维包围盒区间索引；仅保存标量边界和既有实例 ID。"""
from __future__ import annotations

from bisect import bisect_right
from typing import Iterable, Tuple

Bounds = Tuple[Tuple[float, float, float], Tuple[float, float, float]]


def overlaps(left: Bounds, right: Bounds) -> bool:
    return all(left[0][axis] <= right[1][axis] and right[0][axis] <= left[1][axis] for axis in range(3))


class SpatialCandidateIndex:
    """在分布最宽轴排序，并用前缀最大上界裁剪区间查询。

    其余两轴仍精确筛选包围盒。本批改变的少量实例由调用方 overlay 查询；
    没有跨批缓存、失效协议、固定关联半径或尺寸层级。
    """
    def __init__(self, entries: Iterable[Tuple[str, Bounds]]) -> None:
        entries = list(entries)
        self._axis = max(range(3), key=lambda axis: (
            max((item[1][0][axis]+item[1][1][axis] for item in entries), default=0.0)
            - min((item[1][0][axis]+item[1][1][axis] for item in entries), default=0.0)))
        entries.sort(key=lambda item: item[1][0][self._axis])
        self._entries = entries
        self._starts = [item[1][0][self._axis] for item in entries]
        self._prefix_max = []
        high = float('-inf')
        for _, bounds in entries:
            high = max(high, bounds[1][self._axis])
            self._prefix_max.append(high)

    def query(self, bounds: Bounds):
        index = bisect_right(self._starts, bounds[1][self._axis]) - 1
        while index >= 0 and self._prefix_max[index] >= bounds[0][self._axis]:
            instance_id, candidate_bounds = self._entries[index]
            if overlaps(candidate_bounds, bounds):
                yield instance_id
            index -= 1
