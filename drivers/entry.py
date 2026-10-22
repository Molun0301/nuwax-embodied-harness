from __future__ import annotations

from importlib import import_module

from drivers.registry import RobotDriverRegistration, get_robot_driver_registration


def _resolve_configured_robot_name(config) -> str:
    return str(config.robot.active_robot or "").strip().lower()


def get_active_robot_registration(config) -> RobotDriverRegistration:
    """根据配置解析当前激活机器人注册条目。"""

    robot_name = _resolve_configured_robot_name(config)
    if not robot_name:
        raise ValueError("当前未配置 NUWAX_ACTIVE_ROBOT，无法解析机器人入口。")
    return get_robot_driver_registration(robot_name)


def create_active_robot_assembly(config):
    """按当前配置创建激活机器人装配体。"""

    registration = get_active_robot_registration(config)
    module = import_module(registration.package)
    factory = getattr(module, registration.factory_name, None)
    if factory is None or not callable(factory):
        raise RuntimeError(
            f"机器人 {registration.robot_name} 的装配入口不存在: "
            f"{registration.package}:{registration.factory_name}"
        )
    return factory(config, iface=config.dds.iface or None)


def load_active_robot_private_capability_specs(config):
    """加载当前激活机器人私有能力规格。"""

    robot_name = _resolve_configured_robot_name(config)
    if not robot_name:
        return ()
    registration = get_robot_driver_registration(robot_name)
    module = import_module(registration.private_capabilities_module)
    specs = getattr(module, "ROBOT_PRIVATE_CAPABILITY_SPECS", ())
    return tuple(specs)


def create_planar_sensor_source(config_text: str, session: dict):
    """由驱动装配入口创建独占二维传感器来源。"""
    from drivers.common.planar_source import create_planar_sensor_source as create_source
    return create_source(config_text, session)
