"""Tiny CMake/patch fixtures for native header publication; no compiler or SDK."""
from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMMON = ROOT / "services/nuwax-workspace/crates/nuwax-lio-backends/native/common"
SOURCE = Path(os.environ.get("NEH_PLANE_FIT_SOURCE", COMMON / "native_plane_fit.cmake"))
BASELINE_PATH = os.environ.get("NEH_PLANE_FIT_BASELINE", "")
BASELINE = '''set(_NUWAX_PLANE_FIT_MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
function(nuwax_prepare_native_plane_fit source_dir patched_dir)
  configure_file("${source_dir}/include/common_lib.h" "${patched_dir}/src/common_lib.h" COPYONLY)
  configure_file("${_NUWAX_PLANE_FIT_MODULE_DIR}/stable_plane_fit.hpp"
    "${patched_dir}/src/stable_plane_fit.hpp" COPYONLY)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${_NUWAX_PLANE_FIT_MODULE_DIR}/stable-plane-fit.patch")
  execute_process(
    COMMAND "${NUWAX_PATCH_EXECUTABLE}" --batch --forward --fuzz=0 --no-backup-if-mismatch -p1
      -i "${_NUWAX_PLANE_FIT_MODULE_DIR}/stable-plane-fit.patch"
    WORKING_DIRECTORY "${patched_dir}"
    RESULT_VARIABLE patch_result OUTPUT_VARIABLE patch_output ERROR_VARIABLE patch_error)
  if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR "共享平面拟合补丁无法应用：${patch_output}${patch_error}")
  endif()
endfunction()
'''


def identity(path):
    stat = path.stat()
    return stat.st_ino, stat.st_mtime_ns, path.read_bytes()


class NativePlaneFitPublicationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cmake = os.environ.get("NEH_CMAKE") or shutil.which("cmake")
        cls.patch = shutil.which("patch")
        cls.make = shutil.which("make")
        if not all((cls.cmake, cls.patch, cls.make)):
            raise RuntimeError("CMake, patch and make are required; set NEH_CMAKE if CMake is outside PATH")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="neh-plane-fit-publication-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.project, self.build, self.module, self.upstream = self.fixture("candidate", SOURCE.read_text())
        self.header = self.build / "fastlio-initialized/src/common_lib.h"
        self.stable = self.header.with_name("stable_plane_fit.hpp")

    def fixture(self, name, helper):
        directory = self.root / name
        project = directory / "project"
        build = directory / "build"
        module = project / "module"
        upstream = project / "upstream"
        module.mkdir(parents=True)
        (upstream / "include").mkdir(parents=True)
        (module / "native_plane_fit.cmake").write_text(helper)
        patch_text = (COMMON / "stable-plane-fit.patch").read_text()
        (module / "stable-plane-fit.patch").write_text(patch_text)
        shutil.copyfile(COMMON / "stable_plane_fit.hpp", module / "stable_plane_fit.hpp")
        # One actual upstream hunk, with its exact original line position. No SDK.
        lines = patch_text.splitlines(keepends=True)
        old = "".join(line[1:] for line in lines if line.startswith(("-", " ")) and not line.startswith("---"))
        (upstream / "include/common_lib.h").write_text("// harmless fixture prefix\n" * 198 + old)
        (project / "consume.cmake").write_text('''file(READ "${HEADER}" header)
file(READ "${STABLE}" stable)
file(WRITE "${OUTPUT}" "${header}${stable}")
file(APPEND "${CALLS}" "consume\\n")
''')
        (project / "CMakeLists.txt").write_text('''cmake_minimum_required(VERSION 3.14)
project(HeaderPublication NONE)
find_program(NUWAX_PATCH_EXECUTABLE patch REQUIRED)
include("${CMAKE_CURRENT_SOURCE_DIR}/module/native_plane_fit.cmake")
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/configures.log" "configure\\n")
nuwax_prepare_native_plane_fit("${CMAKE_CURRENT_SOURCE_DIR}/upstream" "${CMAKE_CURRENT_BINARY_DIR}/fastlio-initialized")
set(header "${CMAKE_CURRENT_BINARY_DIR}/fastlio-initialized/src/common_lib.h")
set(stable "${CMAKE_CURRENT_BINARY_DIR}/fastlio-initialized/src/stable_plane_fit.hpp")
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/consumer.receipt"
  COMMAND "${CMAKE_COMMAND}" "-DHEADER=${header}" "-DSTABLE=${stable}"
    "-DOUTPUT=${CMAKE_CURRENT_BINARY_DIR}/consumer.receipt" "-DCALLS=${CMAKE_CURRENT_BINARY_DIR}/consumers.log"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/consume.cmake"
  DEPENDS "${header}" "${stable}"
  VERBATIM)
add_custom_target(consume ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/consumer.receipt")
''')
        return project, build, module, upstream

    def command(self, argv, success=True):
        result = subprocess.run(argv, capture_output=True, text=True, timeout=30, check=False,
                                env={**os.environ, "CMAKE_BUILD_PARALLEL_LEVEL": "1"})
        combined = result.stdout + result.stderr
        if success:
            self.assertEqual(result.returncode, 0, combined)
        else:
            self.assertNotEqual(result.returncode, 0, combined)
            self.assertIn("补丁无法应用", combined)
        return combined

    def configure(self, success=True, project=None, build=None):
        return self.command([self.cmake, "-S", str(project or self.project), "-B", str(build or self.build),
                             "-G", "Unix Makefiles", "-DCMAKE_MAKE_PROGRAM=" + self.make], success)

    def consume(self, success=True):
        return self.command([self.cmake, "--build", str(self.build), "--parallel", "1"], success)

    def assert_patched(self, header=None):
        text = (header or self.header).read_text()
        self.assertEqual(text.count('#include "stable_plane_fit.hpp"'), 1)
        self.assertEqual(text.count("return nuwax_lio::stable_plane_fit<T, NUM_MATCH_POINTS>"), 1)
        self.assertNotIn("Matrix<T, NUM_MATCH_POINTS, 3> A;", text)

    def counts(self):
        return tuple(len((self.build / name).read_text().splitlines())
                     for name in ("configures.log", "consumers.log"))

    def test_first_output_matches_frozen_baseline_and_source_is_unchanged(self):
        original = identity(self.upstream / "include/common_lib.h")
        baseline = Path(BASELINE_PATH).read_text() if BASELINE_PATH else BASELINE
        project, build, _module, _upstream = self.fixture("baseline", baseline)
        self.configure(project=project, build=build)
        self.configure()
        for name in ("common_lib.h", "stable_plane_fit.hpp"):
            self.assertEqual((self.header.parent / name).read_bytes(),
                             (build / "fastlio-initialized/src" / name).read_bytes())
        self.assertEqual(identity(self.upstream / "include/common_lib.h"), original)
        self.assert_patched()

    def test_unchanged_configure_preserves_final_header_inode_mtime_and_bytes(self):
        self.configure()
        os.utime(self.header, ns=(1_000_000_000, 1_000_000_000))
        before = identity(self.header)
        stable_before = identity(self.stable)
        self.configure()
        self.assertEqual(identity(self.header), before)
        self.assertEqual(identity(self.stable), stable_before)
        self.assert_patched()

    def test_normal_builds_and_unchanged_reconfigure_do_not_repeat_consumer_or_loop(self):
        self.configure()
        self.consume()
        self.assertEqual(self.counts(), (1, 1))
        before = identity(self.header)
        self.consume()
        self.assertEqual(self.counts(), (1, 1))
        self.configure()
        self.consume()
        self.consume()
        self.assertEqual(self.counts(), (2, 1))
        self.assertEqual(identity(self.header), before)

    def test_source_change_automatically_reconfigures_and_publishes_then_stabilizes(self):
        self.configure()
        self.consume()
        source = self.upstream / "include/common_lib.h"
        source.write_text(source.read_text() + "// changed upstream input\n")
        self.consume()
        self.assertEqual(self.counts(), (2, 2))
        self.assertTrue(self.header.read_text().endswith("// changed upstream input\n"))
        self.assert_patched()
        before = identity(self.header)
        self.configure()
        self.consume()
        self.assertEqual(identity(self.header), before)
        self.assertEqual(self.counts(), (3, 2))

    def test_patch_dependency_automatically_reconfigures_then_stabilizes(self):
        self.configure()
        self.consume()
        patch_path = self.module / "stable-plane-fit.patch"
        patch_path.write_text(patch_path.read_text().replace(
            '+#include "stable_plane_fit.hpp"', '+#include "stable_plane_fit.hpp" // changed recipe comment'))
        self.consume()
        self.assertEqual(self.counts(), (2, 2))
        self.assertIn("// changed recipe comment", self.header.read_text())
        self.assert_patched()
        before = identity(self.header)
        self.configure()
        self.consume()
        self.assertEqual(identity(self.header), before)
        self.assertEqual(self.counts(), (3, 2))

    def test_patch_failure_keeps_old_final_header_and_build_fails_instead_of_fallback(self):
        self.configure()
        self.consume()
        before = identity(self.header)
        patch_path = self.module / "stable-plane-fit.patch"
        original = patch_path.read_text()
        patch_path.write_text(original.replace("-template<typename T>", "-template<typename WrongType>"))
        self.configure(success=False)
        self.assertEqual(identity(self.header), before)
        self.consume(success=False)
        self.assertEqual(identity(self.header), before)
        self.assertEqual((self.build / "consumers.log").read_text().splitlines(), ["consume"])
        patch_path.write_text(original)
        self.configure()
        self.consume()
        self.assert_patched()
        self.assertEqual(identity(self.header), before)

    def test_fresh_patch_failure_does_not_publish_unpatched_header(self):
        patch_path = self.module / "stable-plane-fit.patch"
        original = patch_path.read_text()
        patch_path.write_text(original.replace("-template<typename T>", "-template<typename WrongType>"))
        self.configure(success=False)
        self.assertFalse(self.header.exists())
        patch_path.write_text(original)
        self.configure()
        self.assert_patched()

    def test_incompatible_source_does_not_replace_previous_patched_header(self):
        self.configure()
        before = identity(self.header)
        source = self.upstream / "include/common_lib.h"
        source.write_text(source.read_text().replace("template<typename T>", "template<typename WrongType>"))
        self.configure(success=False)
        self.assertEqual(identity(self.header), before)

    def test_stable_header_change_updates_only_that_output_and_triggers_consumer(self):
        self.configure()
        self.consume()
        before = identity(self.header)
        stable = self.module / "stable_plane_fit.hpp"
        stable.write_text(stable.read_text() + "// changed stable header fixture\n")
        self.consume()
        self.assertEqual(self.counts(), (2, 2))
        self.assertEqual(identity(self.header), before)
        self.assertTrue(self.stable.read_text().endswith("// changed stable header fixture\n"))

    def test_missing_final_header_is_republished_on_normal_build(self):
        self.configure()
        self.consume()
        self.header.unlink()
        self.consume()
        self.assert_patched()
        self.assertEqual(self.counts(), (2, 2))

    def test_two_frontend_style_directories_are_isolated(self):
        project = self.project / "CMakeLists.txt"
        project.write_text(project.read_text() + '\nnuwax_prepare_native_plane_fit("${CMAKE_CURRENT_SOURCE_DIR}/upstream" "${CMAKE_CURRENT_BINARY_DIR}/pointlio-initialized")\n')
        self.configure()
        point = self.build / "pointlio-initialized/src/common_lib.h"
        self.assertEqual(point.read_bytes(), self.header.read_bytes())
        before = (identity(point), identity(self.header))
        self.configure()
        self.assertEqual((identity(point), identity(self.header)), before)

    def test_real_frontend_include_order_and_consumed_output_paths_are_unchanged(self):
        native = COMMON.parent
        for name, variable, entry in (("fastlio2", "FASTLIO_PATCHED_DIR", "fast_lio.hpp"),
                                      ("pointlio", "POINTLIO_PATCHED_DIR", "pointlio.hpp")):
            with self.subTest(frontend=name):
                cmake = (native / name / "cpp/CMakeLists.txt").read_text()
                self.assertIn('nuwax_prepare_native_plane_fit("${FASTLIO_SOURCE_DIR}" "${' + variable + '}")', cmake)
                include_order = cmake.split("target_include_directories(" + name + "_native PRIVATE", 1)[1].split(")", 1)[0].split()
                self.assertEqual(include_order[0], "${" + variable + "}/src")
                self.assertLess(include_order.index("${" + variable + "}/src"), include_order.index("${FASTLIO_SOURCE_DIR}/include"))
                self.assertIn('#include "' + entry + '"', (native / name / "cpp/main.cpp").read_text())


if __name__ == "__main__":
    unittest.main(verbosity=2)
