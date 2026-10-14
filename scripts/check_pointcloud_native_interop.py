#!/usr/bin/env python3
"""Read actual Rust-produced v3 SHM while acknowledging each atomic publication."""

from __future__ import annotations

import argparse
import json
import mmap
import os
import select
import subprocess
import sys
import tempfile
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.geometry import pointcloud_map as cloud

TEST = "pointcloud::tests::native_paged_writer_python_reader_interop"
MARKER = "NUWAX_CLOUD_PAGE_PHASE:"


class CountedMmap(mmap.mmap):
    payload_offset = 16384
    payload_bytes = 0

    def __getitem__(self, key):
        result = super().__getitem__(key)
        if isinstance(key, slice) and (key.start or 0) >= self.payload_offset:
            type(self).payload_bytes += len(result)
        return result


def expected_phases():
    initial = np.full((4096, 3), 0.25, dtype="<f4")
    initial[:, 0] += np.arange(4096, dtype="<f4")
    appended = np.vstack([initial, np.array([[5000.25, 0.25, 0.25]], dtype="<f4")])
    edited = appended.copy()
    edited[0] = appended[-1]
    edited[-1] = (6000.25, 0.25, 0.25)
    deleted = edited[:-1].copy()
    deleted[2048] = edited[-1]
    cleared = np.empty((0, 3), dtype="<f4")
    restarted = np.full((1, 3), np.float32(0.05) / np.float32(2), dtype="<f4")
    return (initial, appended, edited, deleted, cleared, restarted)


def run(binary: Path):
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise RuntimeError(f"native test binary is not executable: {binary}")
    expected = expected_phases()
    expected_bytes = (49152, 12, 12300, 12288, 0, 12)
    expected_versions = ((1, 1, 1, 1), (1, 1, 1, 1, 2), (3, 1, 1, 1, 3), (3, 1, 4, 1), (), (6,))
    report = []
    with tempfile.TemporaryDirectory(prefix="neh-native-cloud-interop-") as directory:
        path = Path(directory) / "cloud.mmap"
        env = {**os.environ, "NUWAX_CLOUD_PAGE_INTEROP_PATH": str(path)}
        process = subprocess.Popen(
            [str(binary), TEST, "--exact", "--ignored", "--nocapture", "--test-threads=1"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=False,
            bufsize=0,
            env=env,
        )
        reader = cloud.CloudMapMmapReader(str(path))
        previous = None
        accepted = []
        try:
            with ExitStack() as guards:
                guards.enter_context(patch.object(cloud.mmap, "mmap", CountedMmap))
                guards.enter_context(
                    patch.object(cloud.hashlib, "sha256", side_effect=AssertionError("unexpected content hash"))
                )
                for number, points in enumerate(expected, start=1):
                    while True:
                        ready, _, _ = select.select([process.stdout], [], [], 30)
                        if not ready:
                            raise RuntimeError(f"native producer did not reach phase {number}")
                        line = process.stdout.readline().decode("utf-8", errors="replace")
                        if not line:
                            raise RuntimeError(f"native producer exited before phase {number}: {process.poll()}")
                        if MARKER in line:
                            if int(line.split(MARKER, 1)[1].strip()) != number:
                                raise AssertionError(f"out-of-order native phase: {line}")
                            break
                    # Layout offset belongs to the actual native header, not a duplicated constant.
                    with path.open("rb") as stream:
                        stream.seek(104)
                        CountedMmap.payload_offset = int.from_bytes(stream.read(8), "little")
                    before = CountedMmap.payload_bytes
                    snapshot = reader.read_stable_snapshot()
                    assert snapshot is not None and snapshot.chunked
                    assert snapshot.revision == number
                    assert snapshot.point_count == len(points)
                    assert b"".join(page.xyz_bytes for page in snapshot.chunks) == points.tobytes()
                    assert tuple(page.revision_ordinal for page in snapshot.chunks) == expected_versions[number - 1]
                    copied = CountedMmap.payload_bytes - before
                    assert copied == expected_bytes[number - 1], (number, copied)
                    if previous is not None:
                        assert snapshot.source_epoch == previous.source_epoch
                        if number == 5:
                            assert snapshot.layout_generation > previous.layout_generation and snapshot.reset
                        else:
                            assert snapshot.layout_generation == previous.layout_generation
                        for index, page in enumerate(snapshot.chunks):
                            if index < len(previous.chunks) and page.revision_ordinal < number:
                                assert page is previous.chunks[index]
                    unchanged_before = CountedMmap.payload_bytes
                    assert reader.read_stable_snapshot() is snapshot
                    assert CountedMmap.payload_bytes == unchanged_before
                    accepted.append(snapshot)
                    previous = snapshot
                    report.append(
                        {
                            "phase": number,
                            "points": snapshot.point_count,
                            "payload_read_bytes": copied,
                            "page_revisions": list(expected_versions[number - 1]),
                        }
                    )
                    process.stdin.write(f"{number}\n".encode())
                    process.stdin.flush()
                remaining, _ = process.communicate(timeout=30)
                remaining = remaining.decode("utf-8", errors="replace")
                assert process.returncode == 0, remaining
                assert "1 passed; 0 failed" in remaining, remaining
                for snapshot, points in zip(accepted, expected):
                    assert b"".join(page.xyz_bytes for page in snapshot.chunks) == points.tobytes()
        finally:
            reader.close()
            if process.poll() is None:
                process.kill()
                process.wait()
    return {
        "native_test": TEST,
        "phases": report,
        "unchanged_refs_preserved": True,
        "legacy_full_snapshot_materialized": False,
        "python_content_hash_guard": True,
        "transport_or_device_started": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True, help="Verified geometry runtime Rust test executable")
    arguments = parser.parse_args()
    print(json.dumps(run(arguments.binary.resolve()), indent=2))


if __name__ == "__main__":
    main()
