"""BL4 claim-only diagnostic evidence using actual Rust qualification archives."""
import copy
import json
import math
import os
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.geometry.fine_map_archive import read_fine_archive_root, validate_fine_archive


class DetachedLegacyArchiveCodecTests(unittest.TestCase):
    def _rust_artifact(self, name):
        directory = os.environ.get("NEH_HEIGHTFIELD_ARCHIVE_DIR")
        if not directory:
            self.skipTest("set NEH_HEIGHTFIELD_ARCHIVE_DIR to this batch's actual Rust artifacts")
        # If the environment is set, missing output is a failure, never a skip
        # or substitution with a locally constructed supposedly green fixture.
        return (Path(directory) / name).read_bytes()

    def _single_block(self, payload):
        """Inspect bytes without pre-admitting them; usable for negative controls."""
        magic, root, offset = read_fine_archive_root(payload)
        self.assertEqual(len(root["blocks"]), 1)
        self.assertEqual(struct.unpack_from("<Q", payload, offset)[0], 1)
        offset += 8
        length = struct.unpack_from("<I", payload, offset)[0]
        offset += 4
        self.assertEqual(offset + length, len(payload))
        body = payload[offset:offset + length]
        points, json_size = struct.unpack_from("<II", body, 8)
        end = 16 + points * 12
        self.assertEqual(points, root["blocks"][0]["points"])
        self.assertEqual(end + json_size, len(body))
        evidence = {local: (stamp, hits)
                    for local, stamp, hits in struct.iter_unpack("<HQH", body[16:end])}
        self.assertEqual(len(evidence), points)
        ground = json.loads(body[end:])
        return magic, root, body[:8], evidence, ground

    def test_real_rust_claim_withdrawal_keeps_original_sample_and_occupancy(self):
        before_bytes = self._rust_artifact("h3-detached-before.ar4")
        after_bytes = self._rust_artifact("h3-detached-legacy.ar5")
        before = self._single_block(before_bytes)
        after = self._single_block(after_bytes)
        self.assertEqual(validate_fine_archive(before_bytes, session="heightfield-storage"), before[1])
        self.assertEqual(validate_fine_archive(after_bytes, session="heightfield-storage"), after[1])
        self.assertEqual((before[0], before[2]), (b"NUWAXAR4", b"NUWAXBL3"))
        self.assertEqual((after[0], after[2]), (b"NUWAXAR5", b"NUWAXBL4"))
        self.assertEqual(before[3], {0: (10, 1), 1024: (20, 1)})
        self.assertEqual(after[3], before[3])
        self.assertEqual(before[4][0][1]["members"], [0, 1024])
        expected = copy.deepcopy(before[4][0][1])
        expected["members"] = [0]  # The sole semantic change in the good record.
        self.assertEqual(after[4], [[0, {"representation": "legacy_plane", "data": expected}]])
        self.assertEqual(expected["samples"], [{
            "point": [0.025, 0.025, 0.075],
            "source_sequence": 1, "source_timestamp_ns": 20, "point_index": 4,
        }])
        self.assertEqual(expected["sample_sources"], [{
            "source": {"kind": "lidar", "backend": 2, "runtime_generation": 7,
                       "sensor_frame_id": "lidar"},
            "sample_positions": [0],
        }])
        self.assertEqual(after[1]["blocks"][0]["key"], [0, 0, 0])
        self.assertEqual(after[1]["voxel_size_m"], 0.05)
        sample = expected["samples"][0]
        self.assertGreater(sample["source_timestamp_ns"], after[3][0][0])
        q = struct.unpack("<f", struct.pack("<f", after[1]["voxel_size_m"]))[0]
        self.assertEqual(math.floor(sample["point"][2] * (1.0 / q)), 1)
        self.assertEqual(sample["source_timestamp_ns"], after[3][1024][0])
        print("H3_DETACHED_RUST green=AR4_BL3_to_AR5_BL4 members=0 occupancy=0:10,1024:20 sample=unchanged")

    def test_real_rust_mutations_reject_unproven_sample_evidence(self):
        good_bytes = self._rust_artifact("h3-detached-legacy.ar5")
        _, good_root, good_magic, good_evidence, good_ground = self._single_block(good_bytes)
        self.assertEqual(good_magic, b"NUWAXBL4")
        validate_fine_archive(good_bytes, session="heightfield-storage")
        q = struct.unpack("<f", struct.pack("<f", good_root["voxel_size_m"]))[0]
        cases = ("future-sample", "occupied-wrong-voxel", "missing-voxel",
                 "other-block", "f32-resolution", "invalid-source")
        for case in cases:
            with self.subTest(case=case):
                expected_evidence = dict(good_evidence)
                expected_ground = copy.deepcopy(good_ground)
                layer = expected_ground[0][1]["data"]
                if case == "future-sample":
                    layer["samples"][0]["source_timestamp_ns"] = 21
                elif case == "occupied-wrong-voxel":
                    expected_evidence[1024] = (19, 1)
                    expected_evidence[2048] = (20, 1)
                elif case == "missing-voxel":
                    del expected_evidence[1024]
                    expected_evidence[2048] = (20, 1)
                elif case == "other-block":
                    layer["samples"][0]["point"][2] = q * 33.5
                elif case == "f32-resolution":
                    layer["samples"][0]["point"][2] = 0.05
                    # A decoder using JSON's f64 .05 instead of Rust's f32 q
                    # would accidentally bind this sample to the newer z1 voxel.
                    self.assertEqual(math.floor(0.05 * (1.0 / q)), 0)
                    self.assertEqual(math.floor(0.05 * (1.0 / 0.05)), 1)
                elif case == "invalid-source":
                    layer["sample_sources"][0]["source"]["backend"] = 0
                payload = self._rust_artifact("h3-detached-" + case + ".ar5")
                magic, root, block_magic, evidence, ground = self._single_block(payload)
                self.assertEqual((magic, block_magic), (b"NUWAXAR5", b"NUWAXBL4"))
                self.assertEqual(evidence, expected_evidence, "unexpected occupancy mutation")
                self.assertEqual(ground, expected_ground, "unexpected record/sample mutation")
                self.assertEqual(root["voxel_size_m"], good_root["voxel_size_m"])
                self.assertEqual(root["blocks"][0]["key"], good_root["blocks"][0]["key"])
                self.assertNotEqual(root["blocks"][0]["object"], good_root["blocks"][0]["object"])
                self.assertEqual(root["source_timestamp_ns"], 21 if case == "future-sample" else 20)
                self.assertEqual(root["revision"], 3)
                self.assertEqual(root["geometry_revision"], 3)
                self.assertEqual(root["blocks"][0]["block_revision"], 3)
                self.assertEqual(root["blocks"][0]["geometry_revision"], 3)
                with self.assertRaises(ValueError):
                    validate_fine_archive(payload, session="heightfield-storage")
                print("H3_DETACHED_RUST mutation=%s admission=REJECT" % case)

    def test_real_classified_empty_column_keeps_occupancy_and_strict_tag(self):
        payload = self._rust_artifact("h3-classified-empty.ar5")
        magic, root, offset = read_fine_archive_root(payload)
        self.assertEqual(magic, b"NUWAXAR5")
        self.assertEqual(validate_fine_archive(payload), root)
        count = struct.unpack_from("<Q", payload, offset)[0]
        offset += 8
        blocks = []
        for _ in range(count):
            length = struct.unpack_from("<I", payload, offset)[0]
            offset += 4
            blocks.append(payload[offset:offset + length])
            offset += length
        self.assertEqual(offset, len(payload))
        self.assertEqual(sum(ref["points"] for ref in root["blocks"]), 4)
        marked = [i for i, block in enumerate(blocks) if block[:8] == b"NUWAXBL4"]
        self.assertEqual(marked, [0])
        body = blocks[0]
        end = 16 + root["blocks"][0]["points"] * 12
        record = json.loads(body[end:])
        self.assertEqual(record, [[0, {"representation": "classified_empty_column", "data": {}}]])
        root_bytes = json.dumps(root, separators=(",", ":")).encode()
        for variant in ("unknown", "extra", "no_occupancy", "outside_xy"):
            edited = copy.deepcopy(record)
            if variant == "unknown":
                edited[0][1]["representation"] = "unknown_empty"
            elif variant == "extra":
                edited[0][1]["data"]["height"] = 0
            elif variant == "no_occupancy":
                edited[0][0] = 2
            else:
                edited[0][0] = 1024
            tail = json.dumps(edited, separators=(",", ":")).encode()
            changed = body[:12] + struct.pack("<I", len(tail)) + body[16:end] + tail
            values = [changed] + blocks[1:]
            bad = (magic + struct.pack("<Q", len(root_bytes)) + root_bytes
                   + struct.pack("<Q", len(values))
                   + b"".join(struct.pack("<I", len(v)) + v for v in values))
            with self.subTest(variant=variant), self.assertRaises(ValueError):
                validate_fine_archive(bad)
        print("EMPTY_COLUMN_PY real_rust_roundtrip=true occupancy=4 invalid_tag_or_binding=4_rejected")


if __name__ == "__main__":
    unittest.main()
