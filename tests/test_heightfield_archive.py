"""AR5 storage boundary; no capture, classification or navigation activation."""
import json
import os
import struct
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.geometry.fine_map_archive import validate_fine_archive


def block(records, ground):
    raw = json.dumps(ground, separators=(",", ":"), allow_nan=False).encode() if ground else b""
    return ((b"NUWAXBL4" if ground else b"NUWAXBL1") + struct.pack("<II", len(records), len(raw))
            + b"".join(struct.pack("<HQH", m, 20, 1) for m in records) + raw)


def archive(bodies, version=5, size=.125):
    refs = [dict(key=key, object=dict(kind="assigned", value=[index + 1] * 16),
                 points=struct.unpack("<I", data[8:12])[0], block_revision=1, geometry_revision=1)
            for index, (key, data) in enumerate(bodies)]
    root = dict(version=version, domain=dict(session="heightfield-storage", generation=1,
                frame_generation=0, producer_generation=1), revision=1, geometry_revision=1,
                source_timestamp_ns=20, voxel_size_m=size, blocks=refs)
    raw = json.dumps(root, separators=(",", ":")).encode()
    return (b"NUWAXAR" + str(version).encode() + struct.pack("<Q", len(raw)) + raw
            + struct.pack("<Q", len(bodies))
            + b"".join(struct.pack("<I", len(data)) + data for _, data in bodies))


def fragment(z, mass, height=4.0078125, bounds=None):
    return [[0, dict(representation="heightfield_v1", data=dict(anchor=[0, 0, 0],
             height_m=height, occupied_bounds=[0., 8.125] if bounds is None else bounds,
             members=[[0, [mass, 0., 0., 0., 0., 0., 0., 0.]]], samples=[]))]]


def full(with_middle=True):
    return [([0, 0, i], block([0], fragment(i * 32, mass) if i != 1 or with_middle else []))
            for i, mass in enumerate((.5, 0., .5))]


class HeightfieldArchiveTests(unittest.TestCase):
    def test_complete_root_and_zero_mass_membership_withdrawal(self):
        for keep in (True, False):
            root = validate_fine_archive(archive(full(keep)))
            self.assertEqual(root["version"], 5)
            self.assertEqual(sum(r["points"] for r in root["blocks"]), 3)
        # Missing positive mass is an actual incomplete distribution.
        with self.assertRaisesRegex(ValueError, "完整高程列"):
            validate_fine_archive(archive(full()[:-1]))

    def test_malformed_geometry_mass_members_and_version_are_rejected(self):
        for case in range(8):
            data = fragment(0, .5)
            value = data[0][1]["data"]
            if case == 0: value["height_m"] += .125
            elif case == 1: value["occupied_bounds"][1] += .125
            elif case == 2: value["members"][0][1][0] = -.5
            elif case == 3: value["members"][0][1] = [0.] * 7
            elif case == 4: value["members"][0][0] = 1
            elif case == 5: value["anchor"] = [0, 0, -1]
            elif case == 6: value["normal"] = [0., 0., 1.]
            else: data[0][1]["representation"] = "unrecognized"
            bodies = full(); bodies[0] = ([0, 0, 0], block([0], data))
            with self.subTest(case=case), self.assertRaises(ValueError):
                validate_fine_archive(archive(bodies))
        with self.assertRaises(ValueError):
            validate_fine_archive(archive(full(), version=4))

    def test_original_record_count_and_four_mib_budget(self):
        # A complete old horizontal plane already has 1024 native columns.
        old = [[m, dict(members=[m], plane_origin=[0., 0., .01], normal=[0., 0., 1.],
               samples=[dict(point=[0., 0., .01], source_sequence=1, source_timestamp_ns=20, point_index=m)])]
               for m in range(1024)]
        raw = json.dumps(old, separators=(",", ":")).encode()
        data = b"NUWAXBL2" + struct.pack("<II", 1024, len(raw)) + b"".join(struct.pack("<HQH", m, 20, 1) for m in range(1024)) + raw
        self.assertEqual(validate_fine_archive(archive([([0, 0, 0], data)], version=4))["blocks"][0]["points"], 1024)
        oversized = data + b" " * (4 * 1024 * 1024)
        with self.assertRaisesRegex(ValueError, "块长度"):
            validate_fine_archive(archive([([0, 0, 0], oversized)], version=4))

    def test_actual_rust_archive_artifacts(self):
        directory = os.environ.get("NEH_HEIGHTFIELD_ARCHIVE_DIR")
        if not directory:
            self.skipTest("set NEH_HEIGHTFIELD_ARCHIVE_DIR to Rust storage qualification output")
        expected = {"cross-block.ar5": (5, 5), "zero-mass-withdrawn.ar5": (5, 3),
                    "mixed.ar5": (5, 2), "full-1024.ar5": (5, 1024), "legacy.ar4": (4, 1)}
        for name, (version, points) in expected.items():
            with self.subTest(name=name):
                raw = (Path(directory) / name).read_bytes()
                root = validate_fine_archive(raw, session="heightfield-storage")
                self.assertEqual(root["version"], version)
                self.assertEqual(sum(b["points"] for b in root["blocks"]), points)
                print("H2_RUST_ARCHIVE name=%s bytes=%d version=%d points=%d" % (name, len(raw), version, points))
        with self.assertRaisesRegex(ValueError, "空间绑定"):
            validate_fine_archive((Path(directory) / "invalid-rebound.ar5").read_bytes())

if __name__ == "__main__":
    unittest.main()
