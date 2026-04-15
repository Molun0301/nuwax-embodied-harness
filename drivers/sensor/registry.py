from __future__ import annotations

from typing import Dict, Tuple

from drivers.sensor.base import RegisteredSensorSpec


_REGISTERED_SENSORS: Dict[str, RegisteredSensorSpec] = {}


def register_sensor_spec(spec: RegisteredSensorSpec) -> RegisteredSensorSpec:
    """注册传感器规格。"""

    normalized = str(spec.sensor_name or "").strip()
    if not normalized:
        raise ValueError("sensor_name 不能为空。")
    _REGISTERED_SENSORS[normalized] = spec
    return spec


def get_sensor_spec(sensor_name: str) -> RegisteredSensorSpec:
    """读取传感器规格。"""

    normalized = str(sensor_name or "").strip()
    spec = _REGISTERED_SENSORS.get(normalized)
    if spec is None:
        raise KeyError(f"未注册的传感器: {normalized}")
    return spec


def list_registered_sensors() -> Tuple[RegisteredSensorSpec, ...]:
    """列出当前已注册传感器。"""

    return tuple(_REGISTERED_SENSORS[name] for name in sorted(_REGISTERED_SENSORS))
