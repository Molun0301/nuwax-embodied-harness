"""Shared constants and repository discovery for release gates."""

from __future__ import annotations

from pathlib import Path
from typing import Optional


PASS = 0
FAIL = 1
NOT_RUN = 2


def find_repo_root(start: Optional[Path] = None) -> Path:
    """Find the nearest parent containing .git."""
    candidate = (start or Path(__file__)).resolve()
    if candidate.is_file():
        candidate = candidate.parent
    for parent in (candidate,) + tuple(candidate.parents):
        if (parent / ".git").exists():
            return parent
    raise RuntimeError("cannot locate repository root from {}".format(candidate))
