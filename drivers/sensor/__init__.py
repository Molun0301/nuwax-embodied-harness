"""传感器复用层导出。"""

from drivers.sensor.base import RegisteredSensorSpec
from drivers.sensor.registry import get_sensor_spec, list_registered_sensors, register_sensor_spec

__all__ = [
    "RegisteredSensorSpec",
    "get_sensor_spec",
    "list_registered_sensors",
    "register_sensor_spec",
]
