#!/usr/bin/env python3
"""Exercise production PointLIO patch preparation with project(... NONE).

Requires the existing clean locked PointLIO checkout, CMake, patch and make.
Writes only a new --work-dir; never downloads, compiles C++, or alters upstream.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

LOCKED = '288e357e5457723c1cce4d4060f76ed7f85b10d4'
FILES = ('pointlio.hpp', 'laserMapping.hpp', 'IMU_Processing.hpp',
         'Estimator.cpp', 'parameters.cpp')
CPP = Path('services/nuwax-workspace/crates/nuwax-lio-backends/native/pointlio/cpp')


def snapshot(directory):
    result = {}
    for name in FILES:
        path = directory / name
        result[name] = (path.read_bytes(), path.stat().st_mtime_ns)
    return result


def comparison(before, after):
    return {name: {'bytes_equal': before[name][0] == after[name][0],
                   'mtime_equal': before[name][1] == after[name][1]}
            for name in FILES}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--baseline-dir', type=Path, required=True,
                        help='Existing successful baseline five-file output directory')
    parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    repo, upstream, work = (p.resolve() for p in (args.repo, args.source, args.work_dir))
    env = dict(os.environ, GIT_OPTIONAL_LOCKS='0', PYTHONDONTWRITEBYTECODE='1')
    def git(*words):
        return subprocess.check_output(['git', '-C', str(upstream)] + list(words),
                                       env=env, text=True).strip()
    if git('rev-parse', 'HEAD') != LOCKED or git('status', '--short'):
        raise RuntimeError('Use the existing clean locked PointLIO checkout: ' + LOCKED)
    upstream_before = snapshot(upstream / 'src')
    baseline = snapshot(args.baseline_dir.resolve())
    production = (repo / CPP / 'CMakeLists.txt').read_text()
    # Extract production statements verbatim: the test does not implement patching.
    start = production.index('# 在构建目录统一应用Point修复')
    end = production.index('\nif(NOT LIVOX_COMMON_DIR)', start)
    fragment = production[start:end]
    patch_names = re.findall(r'\$\{CMAKE_CURRENT_SOURCE_DIR\}/([^"\n]+\.patch)', fragment)
    if len(patch_names) != 8:
        raise RuntimeError('Expected the production eight-patch sequence')
    original_patches = {name: (repo / CPP / name).read_bytes() for name in patch_names}
    work.mkdir(parents=True, exist_ok=False)
    fixture, build, logs = work / 'fixture', work / 'build', work / 'logs'
    fixture.mkdir(); logs.mkdir()
    copied_source = fixture / 'upstream' / 'src'
    copied_source.mkdir(parents=True)
    for name in FILES:
        (copied_source / name).write_bytes(upstream_before[name][0])
    for name, data in original_patches.items():
        (fixture / name).write_bytes(data)
    (fixture / 'production-fragment.cmake').write_text(fragment)
    (fixture / 'consumer.cmake').write_text(
        'file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/consumer-runs.txt" "run\\n")\n'
        'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/consumer.stamp" "consumed\\n")\n')
    (fixture / 'CMakeLists.txt').write_text(
        'cmake_minimum_required(VERSION 3.14)\nproject(pointlio_publication_check NONE)\n'
        'set(FASTLIO_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/upstream")\n'
        'include("${CMAKE_CURRENT_SOURCE_DIR}/production-fragment.cmake")\n'
        'file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/configure-runs.txt" "run\\n")\n'
        'set(INPUTS)\n'
        'foreach(F pointlio.hpp laserMapping.hpp IMU_Processing.hpp Estimator.cpp parameters.cpp)\n'
        ' list(APPEND INPUTS "${POINTLIO_PATCHED_DIR}/src/${F}")\nendforeach()\n'
        'add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/consumer.stamp" '
        'COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_SOURCE_DIR}/consumer.cmake" '
        'DEPENDS ${INPUTS} VERBATIM)\n'
        'add_custom_target(consumer ALL DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/consumer.stamp")\n')
    report = {'locked_upstream': str(upstream), 'locked_commit': LOCKED,
              'production_fragment': fragment, 'patches': patch_names,
              'commands': [], 'checks': {}, 'limitations':
              'Configure/publication regression only; no native C++ compile or timing qualification.'}
    configure = ['cmake', '-S', str(fixture), '-B', str(build), '-G', 'Unix Makefiles']
    consume = ['cmake', '--build', str(build), '--target', 'consumer']
    published = build / 'pointlio-initialized' / 'src'
    def count(kind):
        path = build / (kind + '-runs.txt')
        return len(path.read_text().splitlines()) if path.exists() else 0
    def run(label, command, expected=0):
        before = {'configure': count('configure'), 'consumer': count('consumer')}
        start_time = time.monotonic()
        result = subprocess.run(command, cwd=str(work), env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=30)
        log = logs / (label + '.log'); log.write_bytes(result.stdout)
        record = {'label': label, 'command': command, 'returncode': result.returncode,
                  'seconds': time.monotonic() - start_time, 'before': before,
                  'after': {'configure': count('configure'), 'consumer': count('consumer')},
                  'log': str(log)}
        report['commands'].append(record)
        if result.returncode != expected:
            raise AssertionError(record)
        return record
    def stable(label, before, after):
        checks = comparison(before, after); report['checks'][label] = checks
        assert all(v['bytes_equal'] and v['mtime_equal'] for v in checks.values()), checks
    def idle(label):
        row = run(label, consume)
        assert row['before'] == row['after'], row
    try:
        run('01-first-configure', configure)
        initial = snapshot(published)
        assert all(initial[n][0] == baseline[n][0] for n in FILES)
        report['checks']['initial_bytes_equal_existing_baseline'] = True
        sentinel = published / 'shared-plane-fit-sentinel.hpp'
        sentinel.write_text('// unrelated generated output must remain intact\n')
        sentinel_before = (sentinel.read_bytes(), sentinel.stat().st_mtime_ns)
        run('02-first-consume', consume)
        assert count('consumer') == 1
        idle('03-first-idle')
        run('04-repeat-configure', configure)
        stable('repeat_publication', initial, snapshot(published))
        idle('05-repeat-consume')
        # Change every copied input independently; plain build must reconfigure.
        after_input = initial
        report['checks']['each_input_change'] = {}
        for index, name in enumerate(FILES):
            before_input = snapshot(published)
            source_file = copied_source / name
            marker = ('// publication input change ' + name).encode()
            source_file.write_bytes(source_file.read_bytes() + b'\n' + marker + b'\n')
            changed = run('06-input-%d-auto-configure' % index, consume)
            after_input = snapshot(published)
            assert changed['after']['configure'] == changed['before']['configure'] + 1
            assert changed['after']['consumer'] == changed['before']['consumer'] + 1
            checks = comparison(before_input, after_input)
            assert not checks[name]['bytes_equal'] and not checks[name]['mtime_equal']
            assert marker in after_input[name][0]
            assert all(v['bytes_equal'] and v['mtime_equal'] for n, v in checks.items() if n != name)
            report['checks']['each_input_change'][name] = checks
            idle('07-input-%d-idle' % index)
        # Change a real added output line in the final production patch copy.
        final_patch = fixture / patch_names[-1]
        old_line = b'+    p_pre->det_range      = params.det_range;'
        new_line = old_line + b' // publication patch change'
        assert original_patches[patch_names[-1]].count(old_line) == 1
        valid_patch = original_patches[patch_names[-1]].replace(old_line, new_line)
        final_patch.write_bytes(valid_patch)
        changed = run('08-patch-change-auto-configure', consume)
        after_patch = snapshot(published)
        assert changed['after']['consumer'] == changed['before']['consumer'] + 1
        assert changed['after']['configure'] == changed['before']['configure'] + 1
        assert b'// publication patch change' in after_patch['parameters.cpp'][0]
        assert after_patch['parameters.cpp'][0] != after_input['parameters.cpp'][0]
        assert all(after_patch[n] == after_input[n] for n in FILES if n != 'parameters.cpp')
        idle('09-patch-change-idle')
        failure_patch = ('--- a/src/parameters.cpp\n+++ b/src/parameters.cpp\n@@ -1 +1 @@\n'
                         '-NUWAX_IMPOSSIBLE_CONTEXT_BEFORE\n+NUWAX_IMPOSSIBLE_CONTEXT_AFTER\n')
        final_patch.write_text(failure_patch)
        (work / 'injected-failure.patch.txt').write_text(failure_patch)
        run('10-last-patch-failure', configure, expected=1)
        stable('failure_preserves_publication', after_patch, snapshot(published))
        consumer_before_failure_build = count('consumer')
        run('10b-build-must-refuse-failed-patch', consume, expected=2)
        stable('failed_build_preserves_publication', after_patch, snapshot(published))
        assert count('consumer') == consumer_before_failure_build
        fresh_build = work / 'fresh-failure-build'
        run('10c-fresh-failure-no-publication',
            ['cmake', '-S', str(fixture), '-B', str(fresh_build), '-G', 'Unix Makefiles'], expected=1)
        assert not any((fresh_build / 'pointlio-initialized' / 'src' / n).exists() for n in FILES)
        report['checks']['failed_build_cannot_consume_old_publication'] = True
        report['checks']['fresh_failure_publishes_none_of_five'] = True
        final_patch.write_bytes(valid_patch)
        run('11-recover-configure', configure)
        stable('recovery_publication', after_patch, snapshot(published))
        idle('12-recovery-idle')
        idle('13-consecutive-idle')
        assert (build / 'pointlio-initialized' / 'patch-prepare' / 'src').is_dir()
        report['checks']['input_change_propagated'] = True
        report['checks']['patch_change_propagated'] = True
        report['checks']['idle_builds_do_not_reconfigure_or_consume'] = True
        report['checks']['prepare_retained'] = True
        assert sentinel_before == (sentinel.read_bytes(), sentinel.stat().st_mtime_ns)
        report['checks']['unrelated_consumer_file_unchanged'] = True
        report['result'] = 'PASS'
    finally:
        report['upstream_unchanged'] = upstream_before == snapshot(upstream / 'src')
        report['production_unchanged'] = production == (repo / CPP / 'CMakeLists.txt').read_text()
        report['production_patches_unchanged'] = all(
            (repo / CPP / name).read_bytes() == data for name, data in original_patches.items())
        report['upstream_status_clean'] = not git('status', '--short')
        report['totals'] = {'configure_successes': count('configure'), 'consumer_runs': count('consumer')}
        (work / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    assert all(report[k] for k in ('upstream_unchanged', 'production_unchanged',
                                  'production_patches_unchanged', 'upstream_status_clean'))
    print(json.dumps({'result': report['result'], 'totals': report['totals'],
                      'report': str(work / 'report.json')}, indent=2))


if __name__ == '__main__':
    main()
