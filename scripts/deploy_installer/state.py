from __future__ import annotations

import json
import platform
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from .config import atomic_write


def valid_installation_inputs(value: Any) -> bool:
    return (
        isinstance(value, dict)
        and type(value.get("version")) is int and value["version"] == 1
        and all(isinstance(value.get(key), dict) for key in ("profile", "source", "files", "options"))
        and type(value.get("reusable")) is bool
    )


def installation_inputs_match(left: Any, right: Any) -> bool:
    # JSON normalization at capture makes records round-trip stable; canonical
    # comparison also preserves scalar distinctions such as true versus 1.
    return bool(
        valid_installation_inputs(left) and valid_installation_inputs(right)
        and json.dumps(left, sort_keys=True, separators=(",", ":"))
        == json.dumps(right, sort_keys=True, separators=(",", ":"))
    )


def installation_inputs_equal(left: Any, right: Any) -> bool:
    return bool(installation_inputs_match(left, right) and left["reusable"] and right["reusable"])


def record_identity(record: dict):
    if "release_identity" in record or "installation_inputs" in record:
        identity = record.get("release_identity")
        if (isinstance(identity, dict) and type(identity.get("version")) is int
                and identity["version"] == 1
                and isinstance(identity.get("owner_id"), str) and identity["owner_id"]):
            return ("release-owner-v1", identity["owner_id"])
        return None
    legacy = record.get("fingerprint")
    if isinstance(legacy, str) and legacy:
        return ("legacy-fingerprint", legacy)
    return None


def plan_identity_fields(plan) -> dict:
    owner = getattr(plan, "release_owner_id", "")
    inputs = getattr(plan, "installation_inputs", None)
    if owner or inputs is not None:
        return {"release_identity": {"version": 1, "owner_id": owner}, "installation_inputs": inputs}
    legacy = getattr(plan, "fingerprint", "")
    return {"fingerprint": legacy} if legacy else {}


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
        *,
        release_owner_id: str = "",
        installation_inputs: Optional[dict] = None,
        resumed_from_transaction_id: str = "",
    ) -> None:
        if installation_inputs is not None and (
            not release_owner_id or not valid_installation_inputs(installation_inputs)
        ):
            raise RuntimeError("new deployment state requires owner and installation inputs")
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
        if installation_inputs is not None:
            self.data.pop("fingerprint")
            self.data.update(
                schema_version=3,
                release_identity={"version": 1, "owner_id": release_owner_id},
                installation_inputs=installation_inputs,
            )
        if resumed_from_transaction_id:
            self.data["resumed_from_transaction_id"] = resumed_from_transaction_id
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
