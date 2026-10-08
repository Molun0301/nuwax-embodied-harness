"""Real installer CMake entry-point measurements; no hardware is opened."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

sys.dont_write_bytecode = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--evidence-dir', type=Path, required=True)
    parser.add_argument('--retry-only', action='store_true')
    args = parser.parse_args()
    ROOT, BASE = args.root.resolve(), args.evidence_dir.resolve()
    sys.path.insert(0, str(ROOT))
    from scripts.module_installer.builds import _cmake
    BASE.mkdir(exist_ok=True)
    source = BASE / 'slamtec-source'
    if not source.exists():
        shutil.copytree(ROOT / 'drivers/sensors/slamtec_native', source)
    build = BASE / 'build'
    records = json.loads((BASE / 'measurements.json').read_text()) if args.retry_only else []
    original = (ROOT / 'drivers/sensors/slamtec_native/bridge.cpp').read_text()
    stages = (
        ('01-first', 'generation-1', ()),
        ('02-unchanged', 'generation-1', ()),
        ('03-new-generation', 'generation-2', ()),
        ('04-local-source', 'generation-3', ()),
        ('05-config-change', 'generation-4', ('-DCMAKE_CXX_FLAGS=-DNEH_INCREMENTAL_TEST=1',)),
        ('06-config-unchanged', 'generation-5', ('-DCMAKE_CXX_FLAGS=-DNEH_INCREMENTAL_TEST=1',)),
        ('07-build-failure', 'failed-generation', ('-DCMAKE_CXX_FLAGS=-DNEH_INCREMENTAL_TEST=1',)),
        ('08-retry', 'generation-6', ('-DCMAKE_CXX_FLAGS=-DNEH_INCREMENTAL_TEST=1',)),
    )
    for name, generation, options in stages[6:] if args.retry_only else stages:
        if name == '04-local-source':
            (source / 'bridge.cpp').write_text(original.replace('nw_slamtec_abi_version() { return 1;', 'nw_slamtec_abi_version() { return 2;'))
        if name == '07-build-failure':
            (source / 'bridge.cpp').write_text((source / 'bridge.cpp').read_text() + '\n#error isolated expected failure\n')
        if name == '08-retry':
            (source / 'bridge.cpp').write_text(original.replace('nw_slamtec_abi_version() { return 1;', 'nw_slamtec_abi_version() { return 2;'))
        before = {str(p.relative_to(build)): p.stat().st_mtime_ns for p in build.rglob('*.o')}
        commands = []
        logpath = BASE / (name + '.log')
        with logpath.open('w') as log:
            def run(argv, **kwargs):
                start = time.monotonic()
                log.write('$ ' + ' '.join(map(str, argv)) + '\n')
                log.flush()
                result = subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT, **kwargs)
                commands.append({'argv': list(argv), 'seconds': time.monotonic() - start, 'returncode': result.returncode})
                result.check_returncode()
            started = time.monotonic()
            failed = False
            try:
                _cmake(source, build, BASE / generation, 2, run, options)
            except subprocess.CalledProcessError:
                failed = True
                if name != '07-build-failure':
                    raise
            elapsed = time.monotonic() - started
        assert failed == (name == '07-build-failure')
        after = {str(p.relative_to(build)): p.stat().st_mtime_ns for p in build.rglob('*.o')}
        changed = [p for p, stamp in after.items() if before.get(p) != stamp]
        checked_generation = 'generation-5' if failed else generation
        abi = subprocess.check_output([sys.executable, '-c', 'import ctypes,sys; print(ctypes.CDLL(sys.argv[1]).nw_slamtec_abi_version())', str(BASE / checked_generation / 'lib/libnuwax_slamtec.so')], text=True).strip()
        expected_count = {'01-first':len(after), '02-unchanged':0, '03-new-generation':0, '04-local-source':1, '05-config-change':len(after), '06-config-unchanged':0, '07-build-failure':0, '08-retry':1}[name]
        assert after, 'No real object files were built'
        assert len(changed) == expected_count, (name, changed)
        assert int(abi) == (1 if name in ('01-first', '02-unchanged', '03-new-generation') else 2)
        record = dict(stage=name, seconds=elapsed, object_count=len(after), recompiled=changed, commands=commands, abi=int(abi), expected_failure=failed, log=str(logpath))
        records.append(record)
        (BASE / 'measurements.json').write_text(json.dumps(records, indent=2) + '\n')
        print(json.dumps({k:v for k,v in record.items() if k not in ('commands','recompiled')}, ensure_ascii=False), 'recompiled=', len(changed), flush=True)


if __name__ == '__main__':
    main()
