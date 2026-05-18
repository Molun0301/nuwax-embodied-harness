from __future__ import annotations

import time

import pytest

from services.video import VideoStreamReceiver


JPEG = b"\xff\xd8fake-jpeg\xff\xd9"


def test_video_stream_receiver_ingests_latest_jpeg_and_exports_image_frame() -> None:
    receiver = VideoStreamReceiver()
    receiver.start_stream(
        client_id="go2-test",
        camera_id="front_camera",
        frame_id="go2/front_camera",
        width_px=1280,
        height_px=720,
        metadata={"source_kind": "go2_side_stream_push"},
    )

    frame = receiver.ingest_jpeg(
        JPEG,
        client_id="go2-test",
        camera_id="front_camera",
        frame_id="go2/front_camera",
        width_px=1280,
        height_px=720,
        metadata={"source_kind": "go2_side_stream_push"},
    )

    waited = receiver.wait_for_frame(0, timeout_sec=0.01)
    image = receiver.get_latest_image("front_camera")
    status = receiver.status_snapshot()

    assert waited is frame
    assert image is not None
    assert image.data == JPEG
    assert image.metadata["source_kind"] == "platform_video_ingest_stream"
    assert image.metadata["upstream_source_kind"] == "go2_side_stream_push"
    assert status["connected"] is True
    assert status["frame_count"] == 1
    assert status["payload_bytes"] == len(JPEG)


def test_video_stream_receiver_rejects_non_jpeg_payload() -> None:
    receiver = VideoStreamReceiver()

    with pytest.raises(ValueError, match="JPEG"):
        receiver.ingest_jpeg(b"not-jpeg", client_id="go2-test")


def test_video_stream_receiver_marks_disconnected_stream_unavailable_without_reusing_cached_frame() -> None:
    receiver = VideoStreamReceiver()
    receiver.start_stream(client_id="go2-test", metadata={"source_kind": "go2_side_stream_push"})
    receiver.ingest_jpeg(JPEG, client_id="go2-test", metadata={"source_kind": "go2_side_stream_push"})

    disconnected = receiver.mark_disconnected(client_id="go2-test")

    assert disconnected["connected"] is False
    assert disconnected["available"] is False
    assert disconnected["cached_frame_available"] is True
    assert receiver.latest_frame() is None
    assert receiver.get_latest_image("front_camera") is None


def test_video_stream_receiver_expires_stale_frame() -> None:
    receiver = VideoStreamReceiver(max_frame_age_sec=0.1)
    receiver.start_stream(client_id="go2-test", metadata={"source_kind": "go2_side_stream_push"})
    receiver.ingest_jpeg(JPEG, client_id="go2-test", metadata={"source_kind": "go2_side_stream_push"})

    time.sleep(0.15)
    status = receiver.status_snapshot()

    assert status["connected"] is True
    assert status["available"] is False
    assert status["stale"] is True
    assert receiver.wait_for_frame(0, timeout_sec=0.01) is None
