#!/usr/bin/env python3
"""Offline MJPEG wire equivalence, owned JPEG input and shared-client lifetime."""
from __future__ import annotations

import asyncio
from datetime import datetime, timezone
from pathlib import Path
import queue
import struct
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gateways.ws import data_streams as ws
from services.geometry import video_shm_reader as shm


def historical_frame(data, *, camera_id="", sequence=None, timestamp_ns=0):
    """Independent frozen pre-change byte formula; not a source-text check."""
    identity = b""
    if camera_id:
        identity += ("X-Camera-Id: " + quote(camera_id, safe="") + "\r\n").encode("ascii")
    if sequence is not None:
        identity += ("X-Frame-Sequence: %d\r\n" % int(sequence)).encode("ascii")
    if timestamp_ns > 0:
        stamp = datetime.fromtimestamp(timestamp_ns // 1000000000, tz=timezone.utc)
        stamp = stamp.replace(microsecond=(timestamp_ns % 1000000000) // 1000)
        identity += ("X-Frame-Timestamp: " + stamp.isoformat() + "\r\n").encode("ascii")
        identity += b"X-Frame-Timestamp-Source: video_relay.write_time\r\n"
    header = b"--frame\r\nContent-Type: image/jpeg\r\n"
    header += ("Content-Length: %d\r\n" % len(data)).encode("ascii")
    header += identity + b"\r\n"
    return header + data + b"\r\n"


class FormatterChecks(unittest.TestCase):
    def test_wire_equivalence_matrix(self):
        payloads = (b"", b"x", b"\xff\xd8\x00\r\n--frame\r\n\xff\xd9", bytes(range(256)) * 2048)
        identities = ({}, {"camera_id": "front_camera", "sequence": 0},
                      {"camera_id": "前摄/α ?%\r\n", "sequence": 2**64 - 1},
                      {"camera_id": "camera", "sequence": 23})
        timestamps = (-1, 0, 1, 999, 1000, 999999999, 1000000000, 1791432000123456789)
        for data in payloads:
            for identity in identities:
                for timestamp in timestamps:
                    with self.subTest(size=len(data), identity=identity, timestamp=timestamp):
                        kwargs = dict(identity, timestamp_ns=timestamp)
                        actual = ws._format_mjpeg_frame(data, **kwargs)
                        self.assertEqual(actual, historical_frame(data, **kwargs))
                        self.assertIs(type(actual), bytes)
                        header, body = actual.split(b"\r\n\r\n", 1)
                        self.assertEqual(body, data + b"\r\n")
                        self.assertIn(("Content-Length: %d" % len(data)).encode("ascii"), header)

    def test_literal_identity_headers(self):
        actual = ws._format_mjpeg_frame(b"J", camera_id="前/ ?\r\n", sequence=0, timestamp_ns=1001)
        self.assertEqual(actual, b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 1\r\n"
                         b"X-Camera-Id: %E5%89%8D%2F%20%3F%0D%0A\r\n"
                         b"X-Frame-Sequence: 0\r\n"
                         b"X-Frame-Timestamp: 1970-01-01T00:00:00.000001+00:00\r\n"
                         b"X-Frame-Timestamp-Source: video_relay.write_time\r\n\r\nJ\r\n")

    def test_actual_reader_returns_detached_bytes(self):
        jpeg = b"\xff\xd8old\x00\xff\xd9"
        with tempfile.TemporaryDirectory(prefix="neh-mjpeg-owned-") as directory:
            path = Path(directory) / "synthetic.shm"
            header = bytearray(shm.HEADER_SIZE)
            struct.pack_into("<QIIQIHHQI", header, 0, shm.SHM_MAGIC, 1, 0, 1, len(jpeg), 2, 2, 1001, 0)
            header[44:60] = b"g" * 16
            with path.open("wb") as handle:
                handle.write(header)
                handle.write(jpeg)
                handle.truncate(shm.HEADER_SIZE + 2 * shm.MAX_FRAME_SIZE)
            reader = shm.VideoShmReader(path)
            try:
                data, sequence, timestamp = reader.read_latest()
                self.assertIs(type(data), bytes)
                self.assertEqual((data, sequence, timestamp), (jpeg, 1, 1001))
                self.assertIsNone(reader.read_latest())
                with path.open("r+b") as handle:
                    handle.seek(shm.HEADER_SIZE)
                    handle.write(b"N" * len(jpeg))
                self.assertEqual(data, jpeg)
            finally:
                reader.close()
            self.assertEqual(ws._format_mjpeg_frame(data), historical_frame(jpeg))


class FakeReader:
    def __init__(self, path):
        self.path = path
        self.frames = queue.Queue()
        self.closed = threading.Event()

    def read_latest(self):
        try:
            return self.frames.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.closed.set()


class SharedSourceChecks(unittest.IsolatedAsyncioTestCase):
    async def test_clients_share_bytes_and_replacement_retains_old_owner(self):
        source = ws._SharedMjpegFrameSource()
        with patch.object(ws, "VideoShmReader", FakeReader):
            self.assertTrue(source.acquire("synthetic", camera_id="front"))
            reader = source._reader
            try:
                self.assertTrue(source.acquire("synthetic", camera_id="front"))
                self.assertFalse(source.acquire("other", camera_id="front"))
                self.assertFalse(source.acquire("synthetic", camera_id="other"))
                first = asyncio.create_task(source.wait_for_frame(-1))
                second = asyncio.create_task(source.wait_for_frame(-1))
                await asyncio.sleep(0)
                reader.frames.put((b"first", 0, 1001))
                (a, cursor), (b, other_cursor) = await asyncio.wait_for(asyncio.gather(first, second), 3)
                self.assertIs(a, b)
                self.assertEqual(cursor, other_cursor)
                self.assertEqual(a, historical_frame(b"first", camera_id="front", sequence=0, timestamp_ns=1001))
                self.assertIs(source.latest_multipart()[0], a)
                reader.frames.put((b"second", 1, 1002))
                newer, next_cursor = await asyncio.wait_for(source.wait_for_frame(cursor), 3)
                self.assertGreater(next_cursor, cursor)
                self.assertEqual(newer, historical_frame(b"second", camera_id="front", sequence=1, timestamp_ns=1002))
                self.assertEqual(a, b)
                self.assertNotEqual(a, newer)
                source.release()
                self.assertFalse(reader.closed.is_set())
                pending = asyncio.create_task(source.wait_for_frame(next_cursor))
                await asyncio.sleep(0)
                source.release()
                self.assertEqual((await asyncio.wait_for(pending, 3))[0], None)
                self.assertTrue(reader.closed.is_set())
                self.assertTrue(source.acquire("synthetic", camera_id="front"))
                self.assertIsNot(source._reader, reader)
                self.assertIsNone(source.latest_multipart()[0])
            finally:
                while source._consumers:
                    source.release()

    async def test_cancelled_waiter_is_removed(self):
        source = ws._SharedMjpegFrameSource()
        with patch.object(ws, "VideoShmReader", FakeReader):
            self.assertTrue(source.acquire("synthetic"))
            try:
                pending = asyncio.create_task(source.wait_for_frame(-1))
                await asyncio.sleep(0)
                self.assertEqual(len(source._waiters), 1)
                pending.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await pending
                self.assertFalse(source._waiters)
            finally:
                source.release()


if __name__ == "__main__":
    print("Python:", sys.version, flush=True)
    print("Actual module:", ws.__file__, flush=True)
    unittest.main(verbosity=2)
