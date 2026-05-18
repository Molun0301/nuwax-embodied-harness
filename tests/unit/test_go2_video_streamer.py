from __future__ import annotations

import asyncio
import sys
from types import SimpleNamespace

sys.modules.setdefault("websockets", SimpleNamespace())

from drivers.go2 import video_streamer


def test_go2_video_streamer_run_blocking_supports_python38_fallback(monkeypatch) -> None:
    """Go2 实机 Python 3.8 没有 asyncio.to_thread，推流器必须走兼容线程池。"""

    monkeypatch.setattr(video_streamer.asyncio, "to_thread", None, raising=False)

    result = asyncio.run(video_streamer._run_blocking(lambda: "ok"))

    assert result == "ok"


def test_go2_video_streamer_requires_go2_sdk_path(monkeypatch) -> None:
    """Go2 SDK 路径属于端侧 data_plane.env，缺失时应在端侧明确失败。"""

    config = SimpleNamespace(sdk_path="", cyclonedds_lib_dir="", dds_iface="", official=SimpleNamespace(client_timeout_sec=3.0))
    monkeypatch.setattr(video_streamer, "_prepare_go2_sdk_environment", lambda _config: None)
    monkeypatch.setattr(video_streamer.sys, "path", ["/tmp/no-unitree-sdk"])

    try:
        video_streamer._create_video_client(config)
    except video_streamer.Go2VideoStreamerConfigError as exc:
        message = str(exc)
    else:  # pragma: no cover - 失败路径必须抛出
        raise AssertionError("expected Go2VideoStreamerConfigError")

    assert "GO2_SDK_PATH" in message
    assert "drivers/go2/data_plane.env" in message
