"""设施接入适配；不包含机器人速度输出。"""
from .http_provider import HttpFacilityProvider, load_facility_providers

__all__ = ["HttpFacilityProvider", "load_facility_providers"]
