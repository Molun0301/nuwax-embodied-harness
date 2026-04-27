from __future__ import annotations

import scripts.bootstrap as bootstrap_module
import pytest

from scripts.bootstrap import (
    BootstrapError,
    BootstrapPlanner,
    _build_redacted_nuwaclaw_docker_command,
    build_geometry_apt_packages,
    build_geometry_optional_apt_packages,
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


def test_go2_robot_env_overrides_enable_direct_dds_and_official_backend() -> None:
    planner = _build_planner(
        "apply",
        "--robot",
        "go2",
        "--tier",
        "full",
        "--geometry",
        "--data-plane",
        "--memory-mode",
        "cloud",
        "--perception-mode",
        "hybrid",
        "--mcp-mode",
        "skip",
    )

    overrides = planner.robot_env_overrides()

    assert overrides["GO2_DATA_PLANE_ENABLED"] == "true"
    assert overrides["GO2_DIRECT_DDS_ENABLED"] == "true"
    assert overrides["GO2_OFFICIAL_BACKEND_ENABLED"] == "true"
    assert overrides["GO2_NAMED_MAP_ARCHIVE_ROOT"].endswith("runtime_data/go2_named_maps")


def test_geometry_defaults_install_rtabmap_nav2_dependencies() -> None:
    planner = _build_planner(
        "apply",
        "--robot",
        "go2",
        "--tier",
        "full",
        "--geometry",
        "--data-plane",
        "--memory-mode",
        "cloud",
        "--perception-mode",
        "hybrid",
        "--mcp-mode",
        "skip",
    )

    assert planner.geometry_enabled is True
    assert planner.build_geometry is True
    assert planner.install_geometry_deps is True
    assert "install_geometry_deps: true" in planner.summary()


def test_build_geometry_apt_packages_uses_ros_distro_prefix() -> None:
    packages = build_geometry_apt_packages("humble")

    assert "python3-colcon-common-extensions" in packages
    assert "python3-rosdep" in packages
    assert "ros-humble-rtabmap-ros" in packages
    assert "ros-humble-navigation2" in packages
    assert "ros-humble-nav2-bringup" in packages
    assert "ros-humble-rmw-cyclonedds-cpp" in packages
    assert "ros-humble-nav2-simple-commander" not in packages


def test_nav2_simple_commander_is_optional_for_foxy_package_sets() -> None:
    packages = build_geometry_apt_packages("foxy")
    optional_packages = build_geometry_optional_apt_packages("foxy")

    assert "ros-foxy-nav2-bringup" in packages
    assert "ros-foxy-nav2-simple-commander" not in packages
    assert optional_packages == ("ros-foxy-nav2-simple-commander",)


def test_build_geometry_enables_dependency_install_by_default() -> None:
    planner = _build_planner(
        "apply",
        "--robot",
        "customrobot",
        "--tier",
        "lite",
        "--no-geometry",
        "--build-geometry",
        "--memory-mode",
        "disabled",
        "--perception-mode",
        "disabled",
        "--mcp-mode",
        "skip",
    )

    assert planner.geometry_enabled is False
    assert planner.build_geometry is True
    assert planner.install_geometry_deps is True


def test_build_workspaces_only_runs_geometry_workspace_script(monkeypatch) -> None:
    calls: list[tuple[tuple[str, ...], object]] = []

    def _fake_run(command, *, cwd=None, **kwargs):
        del kwargs
        calls.append((tuple(command), cwd))
        return None

    monkeypatch.setattr(bootstrap_module, "_run", _fake_run)
    planner = _build_planner(
        "apply",
        "--robot",
        "go2",
        "--tier",
        "full",
        "--geometry",
        "--data-plane",
        "--memory-mode",
        "cloud",
        "--perception-mode",
        "hybrid",
        "--mcp-mode",
        "skip",
    )

    bootstrap_module.build_workspaces(planner)

    assert calls == [
        (
            (
                "bash",
                str(bootstrap_module.PROJECT_ROOT / "services" / "geometry" / "build_ros2_workspace.sh"),
            ),
            bootstrap_module.PROJECT_ROOT,
        )
    ]
