#!/usr/bin/env python3
"""CPU/memory budget contracts and optional real Cargo/CMake jobserver check."""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.module_installer import hardware  # noqa: E402
from scripts import neh_source_build  # noqa: E402


class BudgetChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-build-budget-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def detect(self, cpus=64, total=256, available=160, quota="max 100000", limit="max", used=0):
        for name, value in {
            "proc/meminfo": "MemTotal: %s kB\nMemAvailable: %s kB\n" % (total * 1024**2, available * 1024**2),
            "sys/fs/cgroup/cpu.max": quota,
            "sys/fs/cgroup/memory.max": str(limit),
            "sys/fs/cgroup/memory.current": str(used),
        }.items():
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(value)
        with patch.object(hardware.os, "cpu_count", return_value=128), \
             patch.object(hardware.os, "sched_getaffinity", return_value=set(range(cpus))):
            return hardware.detect_hardware(self.root, filesystem_root=self.root, run_commands=False)

    def test_large_machine_uses_all_available_cpus_without_16_cap(self):
        self.assertEqual(self.detect().build_jobs, 64)

    def test_affinity_is_respected(self):
        self.assertEqual(self.detect(cpus=6).build_jobs, 6)

    def test_fractional_cpu_quota_does_not_leave_fraction_idle(self):
        self.assertEqual(self.detect(quota="150000 100000").build_jobs, 2)

    def test_available_memory_bounds_parallelism(self):
        self.assertEqual(self.detect(available=7).build_jobs, 3)

    def test_cgroup_available_memory_bounds_parallelism(self):
        self.assertEqual(self.detect(limit=8 * hardware.GIB, used=5 * hardware.GIB).build_jobs, 1)

    def test_exhausted_or_unknown_memory_stays_bounded(self):
        self.assertEqual(self.detect(available=0).build_jobs, 1)
        self.assertEqual(self.detect(total=0, available=0).build_jobs, 1)

    def test_explicit_budget_precedence(self):
        host = hardware.HardwareInfo(build_jobs=24)
        env = {"CARGO_BUILD_JOBS": "12", "CMAKE_BUILD_PARALLEL_LEVEL": "8"}
        def resolve(jobs=None):
            return hardware.resolve_build_jobs(self.root, jobs, hardware=host, environment=env)
        self.assertEqual(resolve(48), 48)
        self.assertEqual(resolve(), 12)
        del env["CARGO_BUILD_JOBS"]
        self.assertEqual(resolve(), 8)
        env.clear()
        self.assertEqual(resolve(), 24)

    def test_invalid_budget_is_not_silently_ignored(self):
        for name in ("CARGO_BUILD_JOBS", "CMAKE_BUILD_PARALLEL_LEVEL"):
            for value in ("0", "-2", "all", "2.5"):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    hardware.resolve_build_jobs(self.root, environment={name: value})
        for value in (0, -1, True):
            with self.assertRaises(ValueError):
                hardware.resolve_build_jobs(self.root, value, environment={})

    def test_source_bridge_can_build_older_source_without_resolver(self):
        old = SimpleNamespace(detect_hardware=lambda root: hardware.HardwareInfo(build_jobs=3))
        payload = {"root": str(self.root), "home": str(self.root), "candidate": str(self.root),
                   "robot": "customrobot", "modules": ["core"], "operation": "metadata"}
        with patch("scripts.module_installer.hardware", old), patch.object(neh_source_build, "_runner"):
            self.assertEqual(neh_source_build.execute(payload)["build_jobs"], 3)


    def test_installer_plan_reports_effective_cli_and_environment_budget(self):
        environment = dict(os.environ, CARGO_BUILD_JOBS="5", CMAKE_BUILD_PARALLEL_LEVEL="2")
        command = [sys.executable, str(ROOT / "install.py"), "install", "--modules", "core",
                   "--install-dir", str(self.root / "installation"), "--json"]
        for explicit, expected in (([], 5), (["--jobs", "7"], 7)):
            result = subprocess.run(command + explicit, env=environment, check=True,
                                    stdout=subprocess.PIPE, text=True)
            self.assertEqual(json.loads(result.stdout)["plan"]["jobs"], expected)
        self.assertFalse((self.root / "installation").exists())

    def test_remote_tools_bundle_imports_budget_only_after_source_sync(self):
        tools = self.root / "tools"
        shutil.copytree(ROOT / "scripts/go2_update", tools / "scripts/go2_update", ignore=shutil.ignore_patterns("__pycache__"))
        selected = self.root / "source"
        (selected / "scripts").mkdir(parents=True)
        (selected / "scripts/__init__.py").touch()
        shutil.copytree(ROOT / "scripts/module_installer", selected / "scripts/module_installer", ignore=shutil.ignore_patterns("__pycache__"))
        environment = dict(os.environ, CARGO_BUILD_JOBS="3")
        environment.pop("PYTHONPATH", None)
        command = [sys.executable, "-m", "scripts.go2_update.build"]
        subprocess.run(command + ["--help"], cwd=tools, env=environment, check=True, stdout=subprocess.PIPE)
        completed = subprocess.run(command + ["--root", str(selected), "--cache", str(self.root / "cache")],
                                   cwd=tools, env=environment, check=True, stdout=subprocess.PIPE, text=True)
        self.assertIn("构建并行预算：3", completed.stdout)
        self.assertEqual(json.loads((self.root / "cache/artifacts.json").read_text()), {})


def real_jobserver(cargo, cmake, evidence):
    """Two native crates compile 16 tiny C++ objects under one real Cargo pool."""
    cargo, cmake = Path(cargo).resolve(), Path(cmake).resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    tools = evidence / "tools"
    tools.mkdir()
    (tools / "cmake").symlink_to(cmake)
    support = evidence / "build_support"
    support.mkdir()
    shutil.copyfile(ROOT / "services/nuwax-workspace/build_support/native_build.rs", support / "native_build.rs")
    (evidence / "Cargo.toml").write_text('[workspace]\nmembers=["crates/first", "crates/second"]\nresolver="2"\n')
    log = evidence / "compiler-events.jsonl"
    launcher = evidence / "compiler-launcher.py"
    launcher.write_text('''import fcntl, json, os, subprocess, sys, time

def event(kind, child):
    with open(os.environ["NEH_COMPILER_LOG"], "a") as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        stream.write(json.dumps({"kind": kind, "pid": child.pid, "at": time.monotonic(), "argv": sys.argv[1:]}) + "\\n")
child = subprocess.Popen(sys.argv[1:])
event("start", child)
try:
    result = child.wait()
finally:
    event("end", child)
sys.exit(result)
''')
    for name in ("first", "second"):
        crate = evidence / "crates" / name
        (crate / "src").mkdir(parents=True)
        native = crate / "native"
        native.mkdir()
        (crate / "Cargo.toml").write_text('[package]\nname="%s"\nversion="0.1.0"\nedition="2021"\n' % name)
        (crate / "src/lib.rs").write_text("pub fn value() -> u32 { 1 }\n")
        (crate / "build.rs").write_text('''#[path = "../../build_support/native_build.rs"]
mod native_build;
use std::{env, path::PathBuf, process::Command};
fn main() {
    let build = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("native");
    assert!(Command::new("cmake").args(["-S", "native", "-B"]).arg(&build)
        .args(["-G", "Unix Makefiles"]).status().unwrap().success());
    assert!(native_build::command(&build).status().unwrap().success());
    println!("cargo:rerun-if-changed=native");
    println!("cargo:rerun-if-changed=../../build_support/native_build.rs");
}
''')
        sources = []
        for index in range(8):
            source = native / ("unit%d.cpp" % index)
            source.write_text("#include <map>\n#include <vector>\n#include <string>\n"
                              "std::map<std::string, std::vector<int>> value%d(int n) {\n"
                              "  return {{std::to_string(n), std::vector<int>(n, %d)}};\n}\n" % (index, index))
            sources.append(source.name)
        (native / "CMakeLists.txt").write_text('cmake_minimum_required(VERSION 3.16)\nproject(fixture CXX)\n'
            'set(CMAKE_CXX_COMPILER_LAUNCHER "%s;%s")\nadd_library(fixture STATIC %s)\n' %
            (sys.executable, launcher, " ".join(sources)))
    env = {key: value for key, value in os.environ.items() if not key.startswith(("CARGO_", "RUST")) and key not in ("MAKEFLAGS", "MFLAGS", "CMAKE_GENERATOR")}
    env.update(PATH=str(tools) + os.pathsep + str(cargo.parent) + os.pathsep + env.get("PATH", ""),
               RUSTC=str(cargo.with_name("rustc")), RUSTDOC=str(cargo.with_name("rustdoc")),
               CARGO_TARGET_DIR=str(evidence / "target"), CARGO_NET_OFFLINE="true",
               CMAKE_BUILD_PARALLEL_LEVEL="99", NEH_COMPILER_LOG=str(log))
    results = []

    def run(label, jobs, expected_compiles):
        before = log.read_text().splitlines() if log.exists() else []
        completed = subprocess.run([str(cargo), "build", "--offline", "--jobs", str(jobs)], cwd=evidence,
                                   env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (evidence / (label + ".log")).write_text(completed.stdout)
        if completed.returncode:
            raise AssertionError(completed.stdout)
        events = [json.loads(line) for line in log.read_text().splitlines()[len(before):]]
        active = peak = starts = 0
        for event in sorted(events, key=lambda event: event["at"]):
            active += 1 if event["kind"] == "start" else -1
            starts += event["kind"] == "start"
            peak = max(peak, active)
        assert active == 0 and starts == expected_compiles, (label, starts, events)
        assert peak <= jobs, (label, peak, jobs)
        if expected_compiles and jobs > 1:
            assert peak > 1, (label, peak)
        row = {"stage": label, "cargo_jobs": jobs, "native_compiles": starts, "peak_native_compilers": peak}
        results.append(row)
        print(json.dumps(row), flush=True)

    run("first-build", 4, 16)
    run("warm-build", 4, 0)
    env["CMAKE_BUILD_PARALLEL_LEVEL"] = "1"
    run("jobs-only", 2, 0)
    for source in (evidence / "crates").glob("*/native/*.cpp"):
        source.write_text(source.read_text() + "// rebuild for single-job qualification\n")
    run("explicit-one", 1, 16)
    # Inspect the same production helper's fallback without installing Ninja.
    probe = evidence / "helper-probe.rs"
    probe.write_text('''#[path = "build_support/native_build.rs"]
mod native_build;
fn main() {
    let command = native_build::command(std::path::Path::new("generator-cache"));
    println!("args={:?}", command.get_args().collect::<Vec<_>>());
    println!("env={:?}", command.get_envs().collect::<Vec<_>>());
}
''')
    executable = evidence / "helper-probe"
    subprocess.run([str(cargo.with_name("rustc")), "--edition", "2021", str(probe), "-o", str(executable)], check=True)
    generator_cache = evidence / "generator-cache"
    generator_cache.mkdir()
    (generator_cache / "CMakeCache.txt").write_text("CMAKE_GENERATOR:INTERNAL=Ninja\n")
    checked = subprocess.check_output([str(executable)], cwd=evidence,
                                     env=dict(env, CARGO_MAKEFLAGS="fixture-jobserver", CMAKE_BUILD_PARALLEL_LEVEL="99"), text=True)
    assert '"--parallel", "1"' in checked and "cargo:warning=" in checked, checked
    checked = subprocess.check_output([str(executable)], cwd=evidence,
                                     env=dict(env, CMAKE_BUILD_PARALLEL_LEVEL="7"), text=True)
    assert '"--parallel", "7"' in checked, checked
    (evidence / "fallback-checks.txt").write_text("PASS: non-Make Cargo fallback uses one slot; standalone CMake override uses seven.\n")
    (evidence / "results.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cargo", type=Path)
    parser.add_argument("--cmake", type=Path)
    parser.add_argument("--evidence-dir", type=Path)
    args = parser.parse_args()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(BudgetChecks)
    if not unittest.TextTestRunner(verbosity=2).run(suite).wasSuccessful():
        raise SystemExit(1)
    if any((args.cargo, args.cmake, args.evidence_dir)):
        if not all((args.cargo, args.cmake, args.evidence_dir)):
            parser.error("--cargo, --cmake and --evidence-dir are required together")
        real_jobserver(args.cargo, args.cmake, args.evidence_dir.absolute())
