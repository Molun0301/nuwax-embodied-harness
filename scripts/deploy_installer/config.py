from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Mapping


def _validate_value(key: str, value: str) -> None:
    if not key or any(char not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for char in key):
        raise ValueError(f"invalid environment key: {key!r}")
    if "\x00" in value or "\n" in value or "\r" in value:
        raise ValueError(f"environment value for {key} must be one line")


def render_env(source: str, managed_values: Mapping[str, str]) -> str:
    values = {str(key): str(value) for key, value in managed_values.items()}
    for key, value in values.items():
        _validate_value(key, value)

    rendered = []
    replaced = set()
    for raw_line in source.splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#") or "=" not in raw_line:
            rendered.append(raw_line)
            continue
        key = raw_line.split("=", 1)[0].strip()
        if key in values:
            if key not in replaced:
                rendered.append(f"{key}={values[key]}")
                replaced.add(key)
            continue
        rendered.append(raw_line)
    for key in sorted(values):
        if key not in replaced:
            rendered.append(f"{key}={values[key]}")
    return "\n".join(rendered).rstrip("\n") + "\n"


def fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_write(path: Path, content: str, mode: int = 0o640) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.chmod(temp_path, mode)
        os.replace(temp_path, path)
        fsync_directory(path.parent)
    finally:
        try:
            temp_path.unlink()
        except FileNotFoundError:
            pass
