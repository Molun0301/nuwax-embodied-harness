from __future__ import annotations

import pytest

from scripts.bootstrap import (
    BootstrapError,
    BootstrapPlanner,
    _build_redacted_nuwaclaw_docker_command,
    build_nuwaclaw_docker_run_command,
    build_parser,
)


def _build_planner(*argv: str) -> BootstrapPlanner:
    parser = build_parser()
    args = parser.parse_args(list(argv))
    return BootstrapPlanner(args)


def test_build_nuwaclaw_docker_run_command_includes_platform_ports_and_ro_mount() -> None:
    planner = _build_planner(
        "apply",
        "--robot",
        "customrobot",
        "--tier",
        "lite",
        "--no-geometry",
        "--memory-mode",
        "disabled",
        "--perception-mode",
        "disabled",
        "--mcp-mode",
        "skip",
        "--install-nuwaclaw-docker",
        "--nuwaclaw-saved-key",
        "12345678-1234-1234-1234-1234567890ab",
        "--nuwaclaw-host-mount",
        "ro",
    )

    command = build_nuwaclaw_docker_run_command(planner)

    assert "--add-host" in command
    assert "host.docker.internal:host-gateway" in command
    assert "6080:6080" in command
    assert "60005:60005" in command
    assert "60006:60006" in command
    assert "nuwax_home:/home/user" in command
    assert "/:/host:ro" in command
    assert "NUWAX_SAVED_KEY=12345678-1234-1234-1234-1234567890ab" in command


def test_build_nuwaclaw_docker_run_command_skips_root_mount_when_not_requested() -> None:
    planner = _build_planner(
        "apply",
        "--robot",
        "customrobot",
        "--tier",
        "lite",
        "--no-geometry",
        "--memory-mode",
        "disabled",
        "--perception-mode",
        "disabled",
        "--mcp-mode",
        "skip",
        "--install-nuwaclaw-docker",
        "--nuwaclaw-saved-key",
        "12345678-1234-1234-1234-1234567890ab",
    )

    command = build_nuwaclaw_docker_run_command(planner)

    assert not any(part.startswith("/:/host:") for part in command)


def test_build_nuwaclaw_docker_run_command_requires_saved_key() -> None:
    planner = _build_planner(
        "apply",
        "--robot",
        "customrobot",
        "--tier",
        "lite",
        "--no-geometry",
        "--memory-mode",
        "disabled",
        "--perception-mode",
        "disabled",
        "--mcp-mode",
        "skip",
        "--install-nuwaclaw-docker",
    )

    with pytest.raises(BootstrapError):
        build_nuwaclaw_docker_run_command(planner)


def test_redacted_docker_command_masks_saved_key() -> None:
    planner = _build_planner(
        "apply",
        "--robot",
        "customrobot",
        "--tier",
        "lite",
        "--no-geometry",
        "--memory-mode",
        "disabled",
        "--perception-mode",
        "disabled",
        "--mcp-mode",
        "skip",
        "--install-nuwaclaw-docker",
        "--nuwaclaw-saved-key",
        "12345678-1234-1234-1234-1234567890ab",
    )

    command = build_nuwaclaw_docker_run_command(planner)
    redacted = _build_redacted_nuwaclaw_docker_command(command)

    assert "12345678-1234-1234-1234-1234567890ab" not in redacted
    assert "1234...90ab" in redacted
