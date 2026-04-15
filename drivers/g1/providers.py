from __future__ import annotations

from dataclasses import dataclass


@dataclass
class G1ProviderBundle:
    """G1 入口骨架占位。"""

    provider_name: str = "g1_placeholder"
    provider_version: str = "0.1.0"
