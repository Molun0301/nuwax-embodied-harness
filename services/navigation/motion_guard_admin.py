from __future__ import annotations

import argparse
import json
import sys
from typing import Optional, Sequence

from services.navigation.motion_guard_client import MotionGuardClient, MotionGuardError
from services.navigation.motion_safety_guard.constants import DEFAULT_SOCKET_PATH


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="nuwax-motion-guard-admin",
        description="Privileged motion-guard administration over the private UDS.",
    )
    parser.add_argument("--socket", default=DEFAULT_SOCKET_PATH)
    subcommands = parser.add_subparsers(dest="command", required=True)

    reset = subcommands.add_parser("reset", help="physically confirm and arm the guard")
    reset.add_argument("--reason", required=True)
    reset.add_argument("--operator", required=True)
    reset.add_argument(
        "--physical-confirm",
        action="store_true",
        required=True,
        help="confirm that an operator physically inspected the robot and area",
    )
    reset.add_argument("--operation-id")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        with MotionGuardClient(args.socket) as client:
            confirmation = client.reset_safe_stop(
                physical_confirm=args.physical_confirm,
                reason=args.reason,
                operator=args.operator,
                operation_id=args.operation_id,
            )
    except MotionGuardError as exc:
        error = {
            "ok": False,
            "error_type": type(exc).__name__,
            "message": str(exc),
        }
        code = getattr(exc, "code", "")
        if code:
            error["code"] = code
        print(json.dumps(error, sort_keys=True), file=sys.stderr)
        return 1

    print(
        json.dumps(
            {
                "ok": True,
                "operation_id": confirmation.operation_id,
                "reset_generation": confirmation.reset_generation,
                "zero_ack_id": confirmation.zero.zero_ack_id,
                "zero_applied_monotonic_ns": confirmation.zero.applied_monotonic_ns,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
