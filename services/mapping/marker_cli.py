#!/usr/bin/env python3
"""Detect AprilTag markers from a JSON request read from stdin or a file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Optional, Sequence


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from services.mapping.apriltag_detector import detect_request  # noqa: E402


MAX_REQUEST_BYTES = 64 * 1024 * 1024


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="JSON request file; stdin is used when omitted")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        if args.input is None:
            request_bytes = sys.stdin.buffer.read(MAX_REQUEST_BYTES + 1)
        else:
            with args.input.open("rb") as request_file:
                request_bytes = request_file.read(MAX_REQUEST_BYTES + 1)
        if len(request_bytes) > MAX_REQUEST_BYTES:
            raise ValueError(f"request exceeds the {MAX_REQUEST_BYTES}-byte limit")
        request = json.loads(request_bytes)
        json.dump(detect_request(request), sys.stdout, separators=(",", ":"), sort_keys=True)
        sys.stdout.write("\n")
        return 0
    except Exception as exc:  # noqa: BLE001 - this process boundary must fail closed.
        json.dump(
            {"error": {"type": type(exc).__name__, "message": str(exc)}},
            sys.stderr,
            separators=(",", ":"),
            sort_keys=True,
        )
        sys.stderr.write("\n")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
