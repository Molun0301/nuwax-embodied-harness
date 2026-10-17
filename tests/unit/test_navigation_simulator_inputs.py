"""Input parity for simulator and matrix through their shared native-worker entry."""
import importlib.util
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "neh_navigation_simulator_input_tests", ROOT / "scripts/diagnostics/navigation_simulator.py"
)
simulator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(simulator)


class NavigationSimulatorInputTests(unittest.TestCase):
    def setUp(self):
        # Input admission and subprocess bytes are real; geometry env setup is unrelated.
        environment = mock.patch.object(simulator, "worker_environment", return_value=dict(os.environ))
        environment.start()
        self.addCleanup(environment.stop)
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-simulator-inputs-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.binary = self.root / "native-fixture"
        self.binary.write_text(
            "#!" + sys.executable + "\n"
            "import json, pathlib, sys\n"
            "data = pathlib.Path(sys.argv[1])\n"
            "rows = []\n"
            "if len(sys.argv) == 4:\n"
            "    rows = [json.loads(line) for line in pathlib.Path(sys.argv[3]).read_text().splitlines()]\n"
            "    assert rows and all(isinstance(row, dict) for row in rows)\n"
            "(data / 'received.json').write_text(json.dumps({'args':sys.argv[1:], 'rows':rows}))\n"
            "print(json.dumps({'ready':True, 'catalog':{'scenes':[{'id':'recorded' if rows else 'straight'}]}}), flush=True)\n"
            "sys.stdin.readline()\n",
            encoding="utf-8",
        )
        self.binary.chmod(0o755)
        self.cloud = self.root / "cloud.xyzt"
        self.cloud.write_bytes(struct.pack("<4f", 1, 2, 3, 0))
        self.rows = [
            {"timestamp_ns": 1, "p": [1, 2, 3], "q": [0, 0, 0, 1]},
            {"timestamp_ns": 2, "p": [2, 2, 3], "q": [0, 0, 0, 1]},
        ]

    def test_array_and_jsonl_both_reach_native_as_object_lines(self):
        for name, contents in [
            ("array.json", json.dumps(self.rows, indent=2)),
            ("poses.jsonl", "\n".join(json.dumps(row) for row in self.rows)),
        ]:
            with self.subTest(name=name):
                poses = self.root / name
                poses.write_text(contents, encoding="utf-8")
                data = self.root / (name + "-output")
                data.mkdir()
                worker = simulator.NativeWorker(self.binary, data, self.cloud, poses, startup_timeout=2)
                try:
                    received = json.loads((data / "received.json").read_text())
                    self.assertEqual(received["rows"], self.rows)
                    self.assertEqual(received["args"][2], str(data / "输入轨迹.jsonl"))
                    self.assertEqual(worker.provenance["source_poses"], str(poses))
                    self.assertEqual(poses.read_text(), contents)
                finally:
                    worker.close()

    def test_invalid_cloud_or_pose_is_rejected_before_native_spawn(self):
        poses = self.root / "poses.json"
        poses.write_text(json.dumps(self.rows), encoding="utf-8")
        data = self.root / "invalid-output"
        data.mkdir()
        for cloud in [b"", b"short", struct.pack("<4f", float("nan"), 2, 3, 0),
                      struct.pack("<4f", 1, 2, 3, -1)]:
            with self.subTest(cloud=cloud), mock.patch.object(simulator.subprocess, "Popen") as popen:
                self.cloud.write_bytes(cloud)
                with self.assertRaises(ValueError):
                    simulator.NativeWorker(self.binary, data, self.cloud, poses)
                popen.assert_not_called()
        self.cloud.write_bytes(struct.pack("<4f", 1, 2, 3, 0))
        poses.write_text("[]", encoding="utf-8")
        with mock.patch.object(simulator.subprocess, "Popen") as popen:
            with self.assertRaises(ValueError):
                simulator.NativeWorker(self.binary, data, self.cloud, poses)
            popen.assert_not_called()

    def test_synthetic_input_does_not_create_recording(self):
        data = self.root / "synthetic-output"
        data.mkdir()
        worker = simulator.NativeWorker(self.binary, data, startup_timeout=2)
        try:
            self.assertEqual(json.loads((data / "received.json").read_text())["rows"], [])
            self.assertFalse((data / "输入轨迹.jsonl").exists())
            self.assertIsNone(worker.provenance["source_poses"])
        finally:
            worker.close()


if __name__ == "__main__":
    unittest.main()
