"""Extract real FastLIO configure block into CMake NONE; no SDK/compiler needed."""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FRONTEND = ROOT / "services/nuwax-workspace/crates/nuwax-lio-backends/native/fastlio2/cpp"
SOURCE = Path(os.environ.get("NEH_FASTLIO_CMAKE_SOURCE", FRONTEND / "CMakeLists.txt"))
BASELINE_PATH = os.environ.get("NEH_FASTLIO_BASELINE", "")
START = 'find_program(NUWAX_PATCH_EXECUTABLE patch)\n'
END = '# dimos-lcm C++ message headers\n'
HEADERS = ("fast_lio.hpp", "IMU_Processing.hpp", "laserMapping.hpp")
BASELINE_BLOCK = '''find_program(NUWAX_PATCH_EXECUTABLE patch)
if(NOT NUWAX_PATCH_EXECUTABLE)
  message(FATAL_ERROR "patch is required to apply the FAST-LIO initialization fix")
endif()
set(FASTLIO_PATCHED_DIR "${CMAKE_CURRENT_BINARY_DIR}/fastlio-initialized")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/fastlio-initialization.patch")
foreach(HEADER fast_lio.hpp laserMapping.hpp IMU_Processing.hpp)
  configure_file("${FASTLIO_SOURCE_DIR}/src/${HEADER}"
    "${FASTLIO_PATCHED_DIR}/src/${HEADER}" COPYONLY)
endforeach()
execute_process(
  COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --forward -p1
    -i "${CMAKE_CURRENT_SOURCE_DIR}/fastlio-initialization.patch"
  WORKING_DIRECTORY "${FASTLIO_PATCHED_DIR}"
  RESULT_VARIABLE FASTLIO_PATCH_RESULT
  OUTPUT_VARIABLE FASTLIO_PATCH_OUTPUT
  ERROR_VARIABLE FASTLIO_PATCH_ERROR
)
if(NOT FASTLIO_PATCH_RESULT EQUAL 0)
  message(FATAL_ERROR
    "FAST-LIO initialization patch failed: ${FASTLIO_PATCH_OUTPUT}${FASTLIO_PATCH_ERROR}")
endif()

'''


def extract_block(source):
    if source.count(START) != 1 or source.count(END) != 1:
        raise ValueError("FastLIO initialization extraction boundaries changed")
    start, end = source.index(START), source.index(END)
    if start >= end:
        raise ValueError("FastLIO initialization boundaries are reversed")
    block = source[start:end]
    if block.count("execute_process(") != 1 or block.count("FAST-LIO initialization patch failed:") != 1:
        raise ValueError("FastLIO initialization block does not have its exact one-patch boundary")
    return block


def patch_fixture(text):
    """Build exact old hunks and independently expected new hunks, no patch execution."""
    original, expected = {}, {}
    current = None
    lines = text.splitlines(keepends=True)
    index = 0
    while index < len(lines):
        line = lines[index]
        if line.startswith("--- a/src/"):
            current = line[len("--- a/src/"):].strip()
            if current in original:
                raise ValueError("duplicate patch file")
            original[current], expected[current] = [], []
            if lines[index + 1] != "+++ b/src/" + current + "\n":
                raise ValueError("patch renames/additions/deletions are outside this fixture")
            index += 2
            continue
        match = re.fullmatch(r"@@ -(\d+),(\d+) \+(\d+),(\d+) @@\n", line)
        if not match or current is None:
            raise ValueError("unexpected patch structure: " + line)
        old_start, old_count, new_start, new_count = map(int, match.groups())
        old, new = original[current], expected[current]
        if len(old) > old_start - 1:
            raise ValueError("overlapping patch hunks")
        gap = ["// fixture unchanged line " + str(n + 1) + "\n" for n in range(len(old), old_start - 1)]
        old.extend(gap)
        new.extend(gap)
        if len(new) != new_start - 1:
            raise ValueError("unexpected new hunk position")
        old_hunk, new_hunk = [], []
        index += 1
        while index < len(lines) and not lines[index].startswith(("@@", "---")):
            entry = lines[index]
            if entry[0] in " -":
                old_hunk.append(entry[1:])
            if entry[0] in " +":
                new_hunk.append(entry[1:])
            if entry[0] not in " +-":
                raise ValueError("unexpected hunk entry")
            index += 1
        if (len(old_hunk), len(new_hunk)) != (old_count, new_count):
            raise ValueError("patch hunk counts changed unexpectedly")
        old.extend(old_hunk)
        new.extend(new_hunk)
    if set(original) != {"IMU_Processing.hpp", "laserMapping.hpp"}:
        raise ValueError("initialization patch no longer modifies exactly the two admitted headers")
    return ({name: "".join(value) for name, value in original.items()},
            {name: "".join(value) for name, value in expected.items()})


def identity(path):
    stat = path.stat()
    return stat.st_ino, stat.st_mtime_ns, path.read_bytes()


class FastlioInitializationPublicationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cmake = os.environ.get("NEH_CMAKE") or shutil.which("cmake")
        cls.make = shutil.which("make")
        if not all((cls.cmake, cls.make, shutil.which("patch"))):
            raise RuntimeError("CMake, make and patch required; set NEH_CMAKE if outside PATH")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-fastlio-init-publication-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.patch_text = (FRONTEND / "fastlio-initialization.patch").read_text()
        self.original, self.expected = patch_fixture(self.patch_text)
        self.original["fast_lio.hpp"] = '#pragma once\n#include "laserMapping.hpp"\n'
        self.expected["fast_lio.hpp"] = self.original["fast_lio.hpp"]
        self.block = extract_block(SOURCE.read_text())
        self.project, self.build, self.upstream = self.fixture("candidate", self.block)
        self.final = self.build / "fastlio-initialized/src"
        self.stage = self.build / "fastlio-initialized/initialization-prepare/src"

    def fixture(self, name, block):
        project = self.root / name / "project"
        build = self.root / name / "build"
        upstream = project / "upstream/src"
        upstream.mkdir(parents=True)
        for header, value in self.original.items():
            (upstream / header).write_text(value)
        (project / "fastlio-initialization.patch").write_text(self.patch_text)
        (project / "consume.cmake").write_text('''file(READ "${HEADERS}/fast_lio.hpp" core)
file(READ "${HEADERS}/IMU_Processing.hpp" imu)
file(READ "${HEADERS}/laserMapping.hpp" mapping)
file(WRITE "${OUTPUT}" "${core}${imu}${mapping}")
file(APPEND "${CALLS}" "consume\\n")
''')
        prefix = '''cmake_minimum_required(VERSION 3.14)
project(FastlioInitialization NONE)
set(FASTLIO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/upstream")
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/configures.log" "configure\\n")
'''
        suffix = '''set(headers "${FASTLIO_PATCHED_DIR}/src")
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/consumer.receipt"
  COMMAND "${CMAKE_COMMAND}" "-DHEADERS=${headers}"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/consumer.receipt" "-DCALLS=${CMAKE_CURRENT_BINARY_DIR}/consumers.log"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/consume.cmake"
  DEPENDS "${headers}/fast_lio.hpp" "${headers}/IMU_Processing.hpp" "${headers}/laserMapping.hpp"
  VERBATIM)
add_custom_target(consume ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/consumer.receipt")
'''
        (project / "CMakeLists.txt").write_text(prefix + block + suffix)
        self.assertEqual((project / "CMakeLists.txt").read_text()[len(prefix):len(prefix) + len(block)], block)
        return project, build, upstream

    def command(self, argv, success=True):
        result = subprocess.run(argv, capture_output=True, text=True, check=False, timeout=30,
                                env={**os.environ, "CMAKE_BUILD_PARALLEL_LEVEL": "1"})
        output = result.stdout + result.stderr
        if success:
            self.assertEqual(result.returncode, 0, output)
        else:
            self.assertNotEqual(result.returncode, 0, output)
            self.assertIn("FAST-LIO initialization patch failed:", output)
        return output

    def configure(self, success=True, project=None, build=None):
        return self.command([self.cmake, "-S", str(project or self.project), "-B", str(build or self.build),
                             "-G", "Unix Makefiles", "-DCMAKE_MAKE_PROGRAM=" + self.make], success)

    def consume(self, success=True):
        return self.command([self.cmake, "--build", str(self.build), "--parallel", "1"], success)

    def identities(self):
        return {header: identity(self.final / header) for header in HEADERS}

    def counts(self):
        return tuple(len((self.build / name).read_text().splitlines())
                     for name in ("configures.log", "consumers.log"))

    def test_extraction_boundaries_and_two_patch_targets_are_exact(self):
        source = SOURCE.read_text()
        self.assertEqual(self.block, source[source.index(START):source.index(END)])
        self.assertNotIn("FetchContent", self.block)
        self.assertNotIn("native_plane_fit.cmake", self.block)
        self.assertIn('COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --forward -p1', self.block)
        self.assertIn('set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS', self.block)
        for malformed in (source + START, source.replace(END, "")):
            with self.assertRaises(ValueError):
                extract_block(malformed)
        self.assertEqual(set(self.expected), set(HEADERS))

    def test_first_outputs_match_independent_hunk_oracle_and_fixed_baseline(self):
        before = {name: identity(self.upstream / name) for name in HEADERS}
        block = extract_block(Path(BASELINE_PATH).read_text()) if BASELINE_PATH else BASELINE_BLOCK
        project, build, _upstream = self.fixture("baseline", block)
        self.configure(project=project, build=build)
        self.configure()
        for header in HEADERS:
            self.assertEqual((self.final / header).read_text(), self.expected[header])
            self.assertEqual((self.final / header).read_bytes(), (build / "fastlio-initialized/src" / header).read_bytes())
            self.assertEqual(identity(self.upstream / header), before[header])

    def test_unchanged_configure_preserves_all_final_header_identities(self):
        self.configure()
        for header in HEADERS:
            os.utime(self.final / header, ns=(1_000_000_000, 1_000_000_000))
        before = self.identities()
        self.configure()
        self.assertEqual(self.identities(), before)

    def test_normal_build_and_explicit_unchanged_configure_do_not_reconsume_or_loop(self):
        self.configure()
        self.consume()
        self.consume()
        self.assertEqual(self.counts(), (1, 1))
        before = self.identities()
        self.configure()
        self.consume()
        self.consume()
        self.assertEqual(self.counts(), (2, 1))
        self.assertEqual(self.identities(), before)

    def test_each_patched_source_change_reconfigures_publishes_only_changed_header_then_stabilizes(self):
        self.configure()
        self.consume()
        for header in HEADERS[1:]:
            with self.subTest(header=header):
                before = self.identities()
                counts = self.counts()
                path = self.upstream / header
                path.write_text(path.read_text() + "// changed upstream fixture\n")
                self.consume()
                self.assertEqual(self.counts(), (counts[0] + 1, counts[1] + 1))
                self.assertTrue((self.final / header).read_text().endswith("// changed upstream fixture\n"))
                for other in set(HEADERS) - {header}:
                    self.assertEqual(identity(self.final / other), before[other])
                published = self.identities()
                self.configure()
                self.consume()
                self.assertEqual(self.identities(), published)
                self.assertEqual(self.counts(), (counts[0] + 2, counts[1] + 1))

    def test_patch_dependency_changes_both_outputs_and_then_stabilizes(self):
        self.configure()
        self.consume()
        core = identity(self.final / "fast_lio.hpp")
        patch_path = self.project / "fastlio-initialization.patch"
        changed = self.patch_text.replace('+  V3D acc_s_last = Zero3d;', '+  V3D acc_s_last = Zero3d; // fixture recipe')
        changed = changed.replace('+#include <stdexcept>', '+#include <stdexcept> // fixture recipe')
        patch_path.write_text(changed)
        self.consume()
        self.assertEqual(self.counts(), (2, 2))
        for header in HEADERS[1:]:
            self.assertIn("// fixture recipe", (self.final / header).read_text())
        self.assertEqual(identity(self.final / "fast_lio.hpp"), core)
        before = self.identities()
        self.configure()
        self.consume()
        self.assertEqual(self.identities(), before)
        self.assertEqual(self.counts(), (3, 2))

    def test_second_file_failure_keeps_both_old_outputs_and_normal_build_cannot_fallback(self):
        self.configure()
        self.consume()
        before = self.identities()
        source = self.upstream / "laserMapping.hpp"
        source.write_text("// incompatible second file\n")
        self.configure(success=False)
        self.assertEqual((self.stage / "IMU_Processing.hpp").read_text(), self.expected["IMU_Processing.hpp"])
        self.assertEqual(self.identities(), before)
        self.assertTrue((self.stage / "laserMapping.hpp.rej").exists())
        self.assertFalse(list(self.final.glob("*.rej")))
        self.assertFalse(list(self.final.glob("*.orig")))
        self.consume(success=False)
        self.assertEqual(self.identities(), before)
        self.assertEqual((self.build / "consumers.log").read_text().splitlines(), ["consume"])
        source.write_text(self.original["laserMapping.hpp"])
        self.configure()
        self.consume()
        self.assertEqual(self.identities(), before)

    def test_fresh_partial_failure_publishes_neither_patched_header_then_retry_is_exact(self):
        source = self.upstream / "laserMapping.hpp"
        source.write_text("// incompatible second file\n")
        self.configure(success=False)
        self.assertEqual((self.stage / "IMU_Processing.hpp").read_text(), self.expected["IMU_Processing.hpp"])
        self.assertFalse((self.final / "IMU_Processing.hpp").exists())
        self.assertFalse((self.final / "laserMapping.hpp").exists())
        self.assertEqual((self.final / "fast_lio.hpp").read_text(), self.original["fast_lio.hpp"])
        source.write_text(self.original["laserMapping.hpp"])
        self.configure()
        for header in HEADERS:
            self.assertEqual((self.final / header).read_text(), self.expected[header])
        before = self.identities()
        self.configure()
        self.assertEqual(self.identities(), before)

    def test_unchanged_core_copy_stays_direct_and_source_change_propagates(self):
        self.configure()
        self.consume()
        before = self.identities()
        source = self.upstream / "fast_lio.hpp"
        source.write_text(source.read_text() + "// updated core fixture\n")
        self.consume()
        self.assertEqual(self.counts(), (2, 2))
        self.assertTrue((self.final / "fast_lio.hpp").read_text().endswith("// updated core fixture\n"))
        for header in HEADERS[1:]:
            self.assertEqual(identity(self.final / header), before[header])
        self.assertFalse((self.stage / "fast_lio.hpp").exists())

    def test_missing_final_header_is_recovered_by_normal_build(self):
        self.configure()
        self.consume()
        for header in HEADERS[1:]:
            (self.final / header).unlink()
        self.consume()
        self.assertEqual(self.counts(), (2, 2))
        for header in HEADERS:
            self.assertEqual((self.final / header).read_text(), self.expected[header])

    def test_real_frontend_include_and_shared_helper_paths_are_unchanged(self):
        source = (FRONTEND / "CMakeLists.txt").read_text()
        include_order = source.split("target_include_directories(fastlio2_native PRIVATE", 1)[1].split(")", 1)[0].split()
        self.assertEqual(include_order[0], "${FASTLIO_PATCHED_DIR}/src")
        self.assertLess(include_order.index("${FASTLIO_PATCHED_DIR}/src"), include_order.index("${FASTLIO_SOURCE_DIR}/include"))
        self.assertIn('nuwax_prepare_native_plane_fit("${FASTLIO_SOURCE_DIR}" "${FASTLIO_PATCHED_DIR}")', source)
        self.assertIn('#include "fast_lio.hpp"', (FRONTEND / "main.cpp").read_text())


if __name__ == "__main__":
    unittest.main(verbosity=2)
