from __future__ import annotations

import json
import platform
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from .config import atomic_write


class StateStore:
    def __init__(self, path: Path, *, enabled: bool = True) -> None:
        self.path = path
        self.enabled = enabled
        self.data: Dict[str, Any] = {}
        self.transaction_path: Optional[Path] = None

    def begin(
        self,
        profile: str,
        fingerprint: str,
        release_id: str,
        transaction_id: str,
        actions: List[str],
    ) -> None:
        self.transaction_path = self.path.parent / "transactions" / f"{transaction_id}.json"
        generation = self._next_generation()
        self.data = {
            "schema_version": 2,
            "generation": generation,
            "transaction_id": transaction_id,
            "profile": profile,
            "fingerprint": fingerprint,
            "release_id": release_id,
            "host": {
                "hostname": platform.node(),
                "machine": platform.machine(),
                "system": platform.system(),
                "release": platform.release(),
            },
            "actions": list(actions),
            "status": "running",
            "started_at": time.time(),
            "finished_at": None,
            "failed_phase": None,
            "phases": [],
        }
        self.save()

    def _next_generation(self) -> int:
        generations = []
        candidates = [self.path]
        transactions = self.path.parent / "transactions"
        if transactions.is_dir():
            candidates.extend(transactions.glob("*.json"))
        for candidate in candidates:
            if not candidate.is_file() or candidate.is_symlink():
                continue
            try:
                record = json.loads(candidate.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"cannot determine deployment generation: {candidate}") from exc
            generation = record.get("generation")
            if isinstance(generation, int) and generation >= 0:
                generations.append(generation)
        return max(generations, default=0) + 1

    def record_phase(self, name: str, status: str, message: str = "") -> None:
        phases: List[Dict[str, Any]] = self.data.setdefault("phases", [])
        phases.append(
            {
                "name": name,
                "status": status,
                "message": message,
                "timestamp": time.time(),
            }
        )
        self.save()

    def finish(self, status: str, failed_phase: Optional[str] = None) -> None:
        self.data["status"] = status
        self.data["failed_phase"] = failed_phase
        self.data["finished_at"] = time.time()
        self.save()

    def record_active_release(self, active_release: Optional[str]) -> None:
        self.data["active_release"] = active_release

    def save(self) -> None:
        if not self.enabled:
            return
        if self.transaction_path is not None:
            self._save_path(self.transaction_path)
        self._save_path(self.path)

    def _save_path(self, path: Path) -> None:
        atomic_write(
            path,
            json.dumps(self.data, indent=2, sort_keys=True) + "\n",
            mode=0o600,
        )
