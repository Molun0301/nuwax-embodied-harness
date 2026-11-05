"""暂停探索的地图名、原始进度与恢复参数。"""
from __future__ import annotations

import os
from pathlib import Path
import tempfile
import threading
from typing import Optional

from pydantic import BaseModel, ConfigDict


class ExplorationCheckpoint(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    checkpoint_id: str
    map_name: str
    created_at: str
    request: dict
    progress_json: str
    planar_seed: Optional[tuple[float, float, float]] = None


class ExplorationCheckpoints:
    def __init__(self, root):
        self._maps = Path(root)
        self.root = self._maps / ".exploration_progress"
        self._lock = threading.RLock()

    def save(self, value: ExplorationCheckpoint) -> None:
        with self._lock:
            self.root.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.root,
                                             prefix=".exploration-", delete=False) as stream:
                temporary = Path(stream.name)
                try:
                    stream.write(value.model_dump_json())
                    stream.flush()
                    os.replace(temporary, self.root / f"{value.checkpoint_id}.json")
                finally:
                    temporary.unlink(missing_ok=True)

    def load(self, checkpoint_id: str) -> ExplorationCheckpoint:
        with self._lock:
            return ExplorationCheckpoint.model_validate_json(
                (self.root / f"{checkpoint_id}.json").read_text(encoding="utf-8"))

    def list(self, limit: int = 50) -> list[dict]:
        with self._lock:
            values = [ExplorationCheckpoint.model_validate_json(path.read_text(encoding="utf-8"))
                      for path in self.root.glob("*.json")]
        values.sort(key=lambda value: value.created_at, reverse=True)
        return [{"checkpoint_id": value.checkpoint_id, "map_name": value.map_name,
                 "created_at": value.created_at,
                 "available": (self._maps / value.map_name / "geometry").is_dir()}
                for value in values[:limit]]
