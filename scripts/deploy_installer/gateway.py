"""Resolve and validate gateway listener addresses without loading the application."""

from __future__ import annotations

import ipaddress
import re
from pathlib import Path
from typing import Mapping, Optional, Tuple

from scripts.systemd.prepare_lcm_network import _configuration_parser


def validate_gateway_listener(host: str, port: int) -> None:
    if not isinstance(host, str):
        raise TypeError("gateway_host must be a numeric IP address or localhost")
    if "%" in host and re.fullmatch(r"[^%]+%[A-Za-z0-9_.-]{1,64}", host) is None:
        raise ValueError("gateway_host has an invalid IPv6 scope identifier")
    if host != "localhost":
        try:
            ipaddress.ip_address(host)
        except ValueError as exc:
            raise ValueError("gateway_host must be a numeric IP address or localhost") from exc
    if not isinstance(port, int) or isinstance(port, bool):
        raise TypeError("gateway_port must be an integer between 1 and 65535")
    if not 1 <= port <= 65535:
        raise ValueError("gateway_port must be between 1 and 65535")


def gateway_health_url(host: str, port: int) -> str:
    validate_gateway_listener(host, port)
    if host == "localhost":
        target = host
    else:
        address = ipaddress.ip_address(host)
        if address.is_unspecified:
            address = ipaddress.ip_address("::1" if address.version == 6 else "127.0.0.1")
        target = str(address).replace("%", "%25")
        if address.version == 6:
            target = f"[{target}]"
    return f"http://{target}:{port}/api/health/full"


def resolve_gateway_listener(
    active_release: Path,
    config_dir: Path,
    *,
    geometry_env: Path = Path("/var/lib/nuwax_robot_bridge/geometry-owner.env"),
    pointcloud_env: Path = Path("/var/lib/nuwax/gateway/pointcloud3d.env"),
    user_env: Path = Path("/var/lib/nuwax_robot_bridge/settings.env"),
    config_overrides: Optional[Mapping[Path, Optional[bytes]]] = None,
) -> Tuple[str, int]:
    """Read the stock service's effective files, including its active robot profile.

    Always read afresh: rollback restores old configuration before calling this.
    Arbitrary service drop-ins and manually launched process environments are not
    discoverable here and must agree with the administrator's managed settings.
    """
    environment = {
        "NUWAX_BOOTSTRAP_ENV_FILE": str(config_dir / "nuwax.env"),
        "GO2_DATA_PLANE_ENV_FILE": str(config_dir / "go2-data-plane.env"),
        "NUWAX_GEOMETRY_WEB_CONFIG_ENV_FILE": str(geometry_env),
        "NUWAX_GATEWAY_WEB_CONFIG_ENV_FILE": str(pointcloud_env),
        "NUWAX_USER_CONFIG_ENV_FILE": str(user_env),
        "NUWAX_ROBOT_PROFILE_MODE": "1",
        "NUWAX_CONFIG_FROM_SYSTEMD_FILES": "1",
    }
    parser = _configuration_parser()
    original_reader = parser.read_env_values

    def read_values(path):
        path = Path(path)
        if config_overrides is None or path not in config_overrides:
            return original_reader(path)
        saved = config_overrides[path]
        source = saved.decode("utf-8") if saved is not None else ""
        values = {}
        for line in source.splitlines():
            parsed = parser.parse_env_line(line)
            if parsed is not None:
                values[parsed[0]] = parsed[1]
        return values

    # Each parser is isolated. Resolve selection from the same previous values as
    # the file merge, without rewriting files or inheriting caller environment.
    parser.read_env_values = read_values
    values = {}
    for path in parser.configuration_sources(root=active_release, environment=environment):
        values.update(parser.configuration_file_values(path, environment=environment, reader=read_values))
    # Match start_gateway.sh's defaults when the selected profile omits a setting.
    host = values.get("NUWAX_GATEWAY_HOST", "").strip() or "0.0.0.0"
    raw_port = values.get("NUWAX_GATEWAY_PORT", "").strip() or "8766"
    if not raw_port.isascii() or not raw_port.isdecimal():
        raise ValueError("configured NUWAX_GATEWAY_PORT must be an integer between 1 and 65535")
    port = int(raw_port)
    validate_gateway_listener(host, port)
    return host, port
