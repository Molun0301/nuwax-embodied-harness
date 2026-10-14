"""Real lcm-gen checks; no algorithm, communication runtime, or fake generator."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import unittest


class NativeLcmGenerationTest(unittest.TestCase):
    def test_incremental_generation_failure_and_obsolete_headers(self):
        cmake = shutil.which('cmake')
        generator = os.environ.get('NUWAX_TEST_LCM_GEN') or shutil.which('lcm-gen')
        if not cmake or not generator:
            self.skipTest('cmake and a real lcm-gen are required')
        repository = Path(__file__).resolve().parents[1]
        common = repository / 'services/nuwax-workspace/crates/nuwax-lio-backends/native/common/native_lcm_types.cmake'
        records = []
        try:
            with tempfile.TemporaryDirectory(prefix='native-lcm-test-') as temporary:
                root = Path(temporary)
                schemas = root / 'schemas/lcm_types'
                schemas.mkdir(parents=True)
                names = ['std_msgs_Time', 'std_msgs_Header', 'geometry_msgs_Point',
                         'geometry_msgs_Quaternion', 'geometry_msgs_Vector3', 'geometry_msgs_Pose',
                         'geometry_msgs_PoseWithCovariance', 'geometry_msgs_Twist',
                         'geometry_msgs_TwistWithCovariance', 'nav_msgs_Odometry', 'sensor_msgs_Imu',
                         'sensor_msgs_PointField', 'sensor_msgs_PointCloud2']
                for name in names:
                    (schemas / (name + '.lcm')).write_text('package probe; struct ' + name + ' { int32_t value; }\n')
                for name in ('LioInputAck', 'LioInputFrame'):
                    (root / (name + '.lcm')).write_text('package probe; struct ' + name + ' { int32_t value; }\n')
                # A real tool copy permits testing its mtime without touching system tools.
                tool = root / 'lcm-gen'
                shutil.copy2(generator, tool)
                (root / 'CMakeLists.txt').write_text(
                    'cmake_minimum_required(VERSION 3.14)\nproject(lcm_generation NONE)\n'
                    'set(NUWAX_LCM_GEN_EXECUTABLE "' + str(tool) + '")\n'
                    'set(NUWAX_NATIVE_ACK_SCHEMA "' + str(root / 'LioInputAck.lcm') + '" CACHE FILEPATH "" FORCE)\n'
                    'include("' + str(common) + '")\n'
                    'nuwax_generate_native_lcm_types("' + str(root / 'schemas') + '")\n')
                build = root / 'build'
                output = build / 'nuwax-lcm-types'

                def run(label, argv, succeeds=True):
                    result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                            text=True, timeout=30)
                    records.append({'label': label, 'argv': argv, 'exit_code': result.returncode,
                                    'output': result.stdout})
                    if succeeds:
                        self.assertEqual(result.returncode, 0, result.stdout)
                    else:
                        self.assertNotEqual(result.returncode, 0, result.stdout)
                    return result

                def snapshot():
                    return {str(p.relative_to(output)): (p.read_bytes(), p.stat().st_mtime_ns)
                            for p in output.rglob('*.hpp')}

                configure = [cmake, '-S', str(root), '-B', str(build)]
                old_version = run('generator-version-before', [str(tool), '--version']).stdout
                run('initial', configure)
                initial = snapshot()
                self.assertEqual(len(initial), 15)
                run('unchanged', configure)
                self.assertEqual(snapshot(), initial)

                # Changing an IDL triggers CMake regeneration via an ordinary build.
                time.sleep(1.05)
                changed_schema = schemas / 'geometry_msgs_Point.lcm'
                changed_schema.write_text('package probe; struct geometry_msgs_Point { int64_t value; }\n')
                run('actual-idl-change', [cmake, '--build', str(build)])
                changed = snapshot()
                key = 'probe/geometry_msgs_Point.hpp'
                self.assertNotEqual(changed[key][0], initial[key][0])
                self.assertNotEqual(changed[key][1], initial[key][1])
                self.assertEqual({k: v for k, v in changed.items() if k != key},
                                 {k: v for k, v in initial.items() if k != key})

                # A generator replacement/update is a configure dependency too.
                stage_header = build / 'nuwax-lcm-types-staging' / key
                stage_mtime = stage_header.stat().st_mtime_ns
                time.sleep(1.05)
                replacement = os.environ.get('NUWAX_TEST_LCM_GEN_REPLACEMENT')
                if replacement:
                    shutil.copyfile(replacement, tool)
                    new_version = run('generator-version-after', [str(tool), '--version']).stdout
                    self.assertNotEqual(new_version, old_version)
                else:
                    os.utime(tool, None)
                run('generator-updated', [cmake, '--build', str(build)])
                self.assertNotEqual(stage_header.stat().st_mtime_ns, stage_mtime)
                updated = snapshot()
                self.assertEqual(set(updated), set(changed))
                for name, (content, mtime) in updated.items():
                    self.assertEqual(content, (build / 'nuwax-lcm-types-staging' / name).read_bytes())
                    if content == changed[name][0]:
                        self.assertEqual(mtime, changed[name][1])
                    else:
                        self.assertNotEqual(mtime, changed[name][1])
                changed = updated

                # Rename an early type, then fail on the final input. No old
                # consumed header may be replaced or removed on this failure.
                changed_schema.write_text('package probe; struct RenamedPoint { int64_t value; }\n')
                frame = root / 'LioInputFrame.lcm'
                frame.write_text('package probe; struct LioInputFrame { invalid syntax;\n')
                failed = run('invalid-final-idl', configure, succeeds=False)
                self.assertIn('Native LIO message generation failed', failed.stdout)
                self.assertEqual(snapshot(), changed)
                self.assertFalse((output / 'probe/RenamedPoint.hpp').exists())

                # A successful repair removes stale output and ignores failed
                # staging leftovers, while retaining every unchanged header.
                frame.write_text('package probe; struct LioInputFrame { int32_t value; }\n')
                run('repair-and-rename', configure)
                repaired = snapshot()
                self.assertNotIn(key, repaired)
                self.assertIn('probe/RenamedPoint.hpp', repaired)
                self.assertEqual(len(repaired), 15)
                for name in changed:
                    if name != key:
                        self.assertEqual(repaired[name], changed[name])
                changed_schema.write_text('package probe;\n')
                run('idl-type-deleted', configure)
                deleted = snapshot()
                self.assertEqual(len(deleted), 14)
                self.assertNotIn('probe/RenamedPoint.hpp', deleted)
                self.assertEqual(deleted, {k: v for k, v in repaired.items() if k != 'probe/RenamedPoint.hpp'})
                run('unchanged-after-repair', configure)
                self.assertEqual(snapshot(), deleted)
                staging = build / 'nuwax-lcm-types-staging'
                stage_times = {str(p.relative_to(staging)): p.stat().st_mtime_ns for p in staging.rglob('*.hpp')}
                run('unchanged-build-no-reconfigure', [cmake, '--build', str(build)])
                self.assertEqual(snapshot(), deleted)
                self.assertEqual(stage_times, {str(p.relative_to(staging)): p.stat().st_mtime_ns for p in staging.rglob('*.hpp')})
        finally:
            evidence = os.environ.get('NUWAX_NATIVE_TEST_EVIDENCE')
            if evidence:
                Path(evidence).write_text(json.dumps(records, indent=2))


if __name__ == '__main__':
    unittest.main()
