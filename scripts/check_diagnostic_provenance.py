#!/usr/bin/env python3
"""离线检查诊断来源记录；不运行计时基准、原生模拟器、设备或模型。"""
from __future__ import annotations

import ast
import builtins
from contextlib import ExitStack, redirect_stdout
import io
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.diagnostics import agent_wait_performance as agent
from scripts.diagnostics import navigation_simulator as navigation


class DiagnosticProvenanceChecks(unittest.TestCase):
    def setUp(self):
        work = os.environ.get("NEH_TEST_WORK_DIR")
        if work:
            Path(work).mkdir(parents=True, exist_ok=True)
        temporary = tempfile.TemporaryDirectory(prefix="neh-diagnostic-provenance-", dir=work)
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.guards = ExitStack()
        self.addCleanup(self.guards.close)
        self.guards.enter_context(patch.object(navigation.subprocess, "Popen",
            side_effect=AssertionError("native or subprocess execution is forbidden")))
        self.guards.enter_context(patch.object(navigation, "worker_environment",
            side_effect=AssertionError("robot geometry loading is forbidden")))
        self.guards.enter_context(patch.object(navigation, "NativeWorker",
            side_effect=AssertionError("native worker construction is forbidden")))
        self.guards.enter_context(patch.object(agent, "sample",
            side_effect=AssertionError("timing samples are forbidden")))

    def source_fixture(self):
        contents = {
            agent.SOURCE_PATHS[0]: b"router before\r\n",
            agent.SOURCE_PATHS[1]: b"service before\n",
            agent.SOURCE_PATHS[2]: b"untracked script\r\n\x00\xff",
        }
        for relative, content in contents.items():
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        return contents

    def assert_no_digest_fields(self, value):
        if isinstance(value, dict):
            for key, child in value.items():
                self.assertFalse(any(word in key.lower() for word in ("sha256", "hash", "digest")), key)
                self.assert_no_digest_fields(child)
        elif isinstance(value, list):
            for child in value:
                self.assert_no_digest_fields(child)

    def test_raw_source_evidence_includes_untracked_bytes_and_detects_restored_mtime(self):
        contents = self.source_fixture()
        evidence = self.root / "evidence"
        before, first = agent.capture_source_observation(self.root, evidence, "before")
        changed_path = self.root / agent.SOURCE_PATHS[0]
        stat_before = changed_path.stat()
        changed_bytes = contents[agent.SOURCE_PATHS[0]].replace(b"before", b"AFTER!")
        changed_path.write_bytes(changed_bytes)
        os.utime(changed_path, ns=(stat_before.st_atime_ns, stat_before.st_mtime_ns))
        self.assertEqual(changed_path.stat().st_size, stat_before.st_size)
        self.assertEqual(changed_path.stat().st_mtime_ns, stat_before.st_mtime_ns)
        after, second = agent.capture_source_observation(self.root, evidence, "after")
        self.assertNotEqual(before, after)
        self.assertEqual(before, contents)
        for observation, expected in ((first, before), (second, after)):
            self.assert_no_digest_fields(observation)
            for relative, record in observation["files"].items():
                self.assertEqual(Path(record["evidence_file"]).read_bytes(), expected[relative])
                self.assertEqual(record["size_bytes"], len(expected[relative]))

    def run_fake_assay(self, runner, *, import_hook=None):
        output = self.root / "assay.json"
        commands = []

        def git_observation(command, **kwargs):
            self.assertEqual(kwargs, {"cwd": self.root, "text": True})
            commands.append(command)
            if command == ["git", "rev-parse", "HEAD"]:
                return "observed-version-label\n"
            self.assertEqual(command, ["git", "status", "--porcelain"])
            return "?? scripts/diagnostics/agent_wait_performance.py\n"

        with ExitStack() as patches:
            patches.enter_context(patch.object(agent, "ROOT", self.root))
            patches.enter_context(patch.object(agent, "run", runner))
            patches.enter_context(patch.object(agent.subprocess, "check_output", side_effect=git_observation))
            patches.enter_context(patch.object(agent.importlib.metadata, "version", return_value="test-version"))
            patches.enter_context(patch.object(agent.platform, "platform", return_value="offline-test-platform"))
            patches.enter_context(patch.object(sys, "argv", ["agent-wait", "--output", str(output)]))
            patches.enter_context(redirect_stdout(io.StringIO()))
            if import_hook:
                patches.enter_context(patch.object(builtins, "__import__", side_effect=import_hook))
            agent.main()
        self.assertEqual(commands, [["git", "rev-parse", "HEAD"], ["git", "status", "--porcelain"]])
        return json.loads(output.read_text())

    def test_assay_observes_sources_and_version_before_lazy_imports_and_reports_only_equality(self):
        contents = self.source_fixture()
        imports = []
        original_import = builtins.__import__

        class FakeService:
            pass

        def import_hook(name, *args, **kwargs):
            if name in {"gateways.agent.router", "gateways.agent.service"}:
                observations = list((self.root / "assay.json.sources").glob("*/before"))
                self.assertEqual(len(observations), 1)
                for relative, content in contents.items():
                    self.assertEqual((observations[0] / relative).read_bytes(), content)
                imports.append(name)
                return SimpleNamespace(AgentToolService=FakeService, build_agent_router=lambda service: service)
            return original_import(name, *args, **kwargs)

        async def fake_run(args):
            self.assertEqual(agent.subprocess.check_output.call_count, 2)
            app = SimpleNamespace(include_router=Mock(), get=lambda path: lambda function: function)
            with patch.object(agent, "FastAPI", return_value=app):
                agent.fixture()
            return {"fake_case": {"summary": {"sample_count": 0}, "samples": []}}, 40, "async"

        report = self.run_fake_assay(fake_run, import_hook=import_hook)
        self.assertEqual(imports, ["gateways.agent.router", "gateways.agent.service"])
        self.assertEqual(report["fixture_id"], "agent-wait-four-long-polls-v1")
        self.assertRegex(report["run_id"], r"^agent-wait-\d{8}T\d{6}\.\d{6}Z-pid\d+$")
        self.assertEqual(report["git_commit"], "observed-version-label")
        self.assertIn("before the run", report["git_observation_scope"])
        self.assertTrue(report["git_dirty"])
        self.assertTrue(report["source_bytes_equal_at_observation_points"])
        self.assertNotIn("source_unchanged_during_measurement", report)
        self.assertIn("does not establish continuous stability", report["source_observation_scope"])
        self.assertIn("loaded module identity", report["source_observation_scope"])
        self.assertIn("transitive dependency identity", report["source_observation_scope"])
        for observation in report["source_observations"].values():
            for relative, record in observation["files"].items():
                self.assertEqual(Path(record["evidence_file"]).read_bytes(), contents[relative])
                self.assertEqual(record["size_bytes"], len(contents[relative]))
        self.assert_no_digest_fields(report)

    def test_assay_rejects_changed_bytes_and_keeps_both_observations(self):
        contents = self.source_fixture()
        changed_path = self.root / agent.SOURCE_PATHS[2]

        async def fake_run(args):
            observed = changed_path.stat()
            changed_path.write_bytes(contents[agent.SOURCE_PATHS[2]].replace(b"script", b"SCRIPT"))
            os.utime(changed_path, ns=(observed.st_atime_ns, observed.st_mtime_ns))
            return {}, 40, "sync"

        with self.assertRaisesRegex(RuntimeError, "源文件字节不同"):
            self.run_fake_assay(fake_run)
        self.assertFalse((self.root / "assay.json").exists())
        evidence = list((self.root / "assay.json.sources").iterdir())
        self.assertEqual(len(evidence), 1)
        self.assertEqual((evidence[0] / "before" / agent.SOURCE_PATHS[2]).read_bytes(), contents[agent.SOURCE_PATHS[2]])
        self.assertEqual((evidence[0] / "after" / agent.SOURCE_PATHS[2]).read_bytes(), changed_path.read_bytes())

    def stat_provenance(self):
        binary = self.root / "fake-native-simulator"
        binary.write_bytes(b"not executable")
        cloud, poses = self.root / "cloud.xyzt", self.root / "poses.jsonl"
        observed = binary.stat()
        with patch.object(Path, "read_bytes", side_effect=AssertionError("binary byte reads are forbidden")), \
                patch.object(Path, "open", side_effect=AssertionError("file contents must not be opened")), \
                patch.object(builtins, "open", side_effect=AssertionError("file contents must not be opened")):
            provenance = navigation.binary_provenance(binary, cloud, poses)
        self.assertEqual(provenance["binary"], str(binary))
        self.assertEqual(provenance["source_cloud"], str(cloud))
        self.assertEqual(provenance["source_poses"], str(poses))
        self.assertEqual(provenance["binary_stat"], {
            "size_bytes": observed.st_size, "mtime_ns": observed.st_mtime_ns,
            "ctime_ns": observed.st_ctime_ns, "dev": observed.st_dev, "inode": observed.st_ino})
        return provenance

    def test_binary_provenance_is_stat_only_and_does_not_claim_a_version(self):
        provenance = self.stat_provenance()
        self.assertIsNone(provenance["binary_version"])
        self.assertEqual(provenance["binary_version_status"], "not_reported")
        self.assertIn("not binary content verification", provenance["provenance_scope"])
        self.assertIn("No native version probe", provenance["provenance_scope"])
        self.assert_no_digest_fields(provenance)

    def test_mocked_simulation_export_preserves_uuid_cpu_and_metadata(self):
        provenance = self.stat_provenance()
        cpu_reader = Mock(side_effect=[10.0, 12.0])
        worker = SimpleNamespace(provenance=provenance, exchange=Mock(side_effect=[
            {"status": "running", "frames": [{"pose": [0, 0, 0, 0]}]},
            {"status": "succeeded", "frames": [{"pose": [1, 1, 0, 0]}], "metrics": {"distance": 2}},
        ]))
        request = {"scene_id": "synthetic", "start": [0, 0, 0, 0], "goal": [1, 1, 0, 0]}
        simulator = navigation.Simulator(worker, self.root)
        run = navigation.Run(request, cpu_reader)
        simulator._drive(run)
        result = json.loads((self.root / "results" / (run.id + ".json")).read_text())
        self.assertEqual(uuid.UUID(hex=run.id).version, 4)
        self.assertEqual(result["id"], run.id)
        self.assertEqual(result["request"], request)
        self.assertEqual(result["status"], "succeeded")
        self.assertEqual(result["algorithm"], provenance)
        self.assertEqual(result["total_frames"], 2)
        self.assertEqual(result["trajectory"], [[0, 0, 0], [1, 1, 0]])
        self.assertEqual(result["metrics"]["native_cpu_seconds"], 2.0)
        self.assertEqual(result["metrics"]["native_cpu_percent"],
                         200.0 / max(0.001, result["metrics"]["elapsed_wall_sec"]))
        self.assertEqual(cpu_reader.call_count, 2)
        self.assertEqual(run.snapshot()["metrics"]["native_cpu_seconds"], 2.0)
        self.assertEqual(cpu_reader.call_count, 2)
        self.assert_no_digest_fields(result)

    def test_python38_syntax_and_no_digest_imports(self):
        for path in (Path(agent.__file__), Path(navigation.__file__), Path(__file__)):
            tree = ast.parse(path.read_text(), filename=str(path), feature_version=(3, 8))
            imports = [alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names]
            self.assertNotIn("hashlib", imports)
            self.assertFalse(any(isinstance(node, ast.ImportFrom) and node.module == "hashlib"
                                 for node in ast.walk(tree)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
