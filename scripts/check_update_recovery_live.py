"""Real updater/Git/HTTP/process recovery; no native build or system services.

The fixture injects failure or a wait at the target installer import. It does
not claim successful full-module installation or real hardware health coverage.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time


def wait_for(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError('Fixture condition timed out')


SERVER = '''import fcntl,json,os,pathlib,subprocess,sys
from http.server import BaseHTTPRequestHandler,HTTPServer
root=pathlib.Path(sys.argv[1]); prefix=pathlib.Path(sys.argv[2])
revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
lock=open(root/'runtime_data/gateway/start_gateway.lock','a')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
(root/'runtime_data/gateway/gateway.pid').write_text(str(os.getpid()))
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  body=json.dumps({'ready':True} if self.path.endswith('/readiness') else {'running_revision':revision}).encode()
  self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
HTTPServer.allow_reuse_address=True
HTTPServer(('127.0.0.1',int(sys.argv[3])),Handler).serve_forever()
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--evidence-dir', type=Path, required=True)
    args = parser.parse_args()
    source, base = args.root.resolve(), args.evidence_dir.resolve()
    base.mkdir(parents=True)
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(source))
    from scripts import self_update as u
    from scripts.module_installer import lifecycle

    origin, root = base / 'origin', base / 'checkout'
    origin.mkdir()
    for directory in ('scripts', 'scripts/device_manager', 'core'):
        (origin / directory).mkdir(exist_ok=True)
    shutil.copytree(source / 'scripts/module_installer', origin / 'scripts/module_installer', ignore=shutil.ignore_patterns('__pycache__'))
    for path in ('scripts/device_manager/common.py', 'scripts/device_manager/__init__.py', 'core/config_values.py'):
        shutil.copyfile(source / path, origin / path)
    (origin / '.gitignore').write_text('runtime_data/\n__pycache__/\n')
    gitlog = (base / 'git.log').open('w')
    def git(where, *argv):
        gitlog.write('$ git ' + shlex.join(argv) + '\n'); gitlog.flush()
        result = subprocess.run(['git', '-C', str(where), *argv], stdout=subprocess.PIPE, stderr=gitlog, text=True, check=True)
        return result.stdout.strip()
    git(origin, 'init', '-b', 'fixture')
    git(origin, 'config', 'user.name', 'Isolated C1 Test')
    git(origin, 'config', 'user.email', 'c1@example.invalid')
    git(origin, 'add', '.')
    git(origin, 'commit', '-m', 'Unmodified installer fixture')
    old = git(origin, 'rev-parse', 'HEAD')
    git(base, 'clone', str(origin), str(root))
    manager = origin / 'scripts/module_installer/manager.py'
    original = manager.read_text()
    home = root / 'runtime_data/installation'
    prefix = home / 'generations/old'
    prefix.mkdir(parents=True)
    (root / 'runtime_data/gateway').mkdir(parents=True)
    (home / 'current').symlink_to('generations/old')
    cache = home / 'cache/build/retained-object'
    cache.parent.mkdir(parents=True)
    cache.write_text('retained cache fixture, not compiled evidence')
    cache_stat = cache.stat()
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    url = 'http://127.0.0.1:' + str(port)
    (prefix / 'settings.env').write_text('NUWAX_GATEWAY_HOST=127.0.0.1\nNUWAX_GATEWAY_PORT=' + str(port) + '\n')
    descriptor = lifecycle.deployment_descriptor(root, home, prefix, ['core'], 'customrobot')
    u.write_json(prefix / 'manifest.json', dict(schema=1, complete=True, modules=['core'], robot='customrobot', source_root=str(root), python=sys.executable, deployment=descriptor))
    server = root / 'runtime_data/server.py'
    server.write_text(SERVER)
    launcher = '#!/bin/bash\nexport NUWAX_BOOTSTRAP_ENV_FILE=' + shlex.quote(str(prefix / 'settings.env')) + '\nexec ' + shlex.join([sys.executable, str(server), str(root), str(prefix), str(port)]) + '\n'
    (prefix / 'start.sh').write_text(launcher)
    os.environ['NUWAX_UPDATE_INSTALL_DIR'] = str(home)
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    os.environ['NUWAX_GATEWAY_HOST'] = '127.0.0.1'
    os.environ['NUWAX_GATEWAY_PORT'] = str(port)
    processlog = base / 'process.log'
    processes = []
    results = []
    pidfile = root / 'runtime_data/gateway/gateway.pid'
    def start_old():
        p = u.start(root, prefix / 'start.sh', processlog, descriptor=descriptor)
        processes.append(p)
        u.verify(url, old, p, timeout=10)

    def stop_current():
        u.stop(root, prefix, descriptor=descriptor)
        for p in processes:
            if p.poll() is not None:
                p.wait()

    try:
        start_old()
        # Real HTTP wrong-revision failure, separate from the transaction cases.
        try:
            u.verify(url, '0' * 40, None, timeout=.2)
        except RuntimeError as exc:
            results.append(dict(case='real-health-wrong-revision', error=str(exc)))
        else:
            raise AssertionError('Wrong revision accepted')

        for case in ('installer-failure', 'retry-same-target', 'interrupted-installer', 'failed-recovery'):
            if case != 'retry-same-target':
                injection = '\nraise RuntimeError("C1 deliberate target installer import failure")\n'
                if case == 'interrupted-installer':
                    injection = '\nPath(os.environ["NEH_UPDATE_ROOT"]).joinpath("runtime_data/import-waiting").write_text(str(os.getpid()))\ntime.sleep(120)\n'
                manager.write_text(original + injection)
                git(origin, 'add', 'scripts/module_installer/manager.py')
                git(origin, 'commit', '-m', case)
            target = git(origin, 'rev-parse', 'HEAD')
            info = u.check(root)
            assert info['latest_revision'] == target
            queued = u.queue(root, target, url)
            if case == 'failed-recovery':
                (prefix / 'start.sh').write_text('#!/bin/bash\nexit 23\n')
            env = dict(os.environ, PYTHONPATH=str(root))
            with (base / (case + '.log')).open('w') as log:
                worker = subprocess.Popen([sys.executable, str(source / 'scripts/self_update.py'), '--root', str(root), '--request', queued['request_id']], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
                processes.append(worker)
                if case == 'interrupted-installer':
                    wait_for(lambda: (root / 'runtime_data/import-waiting').is_file())
                    worker.send_signal(signal.SIGTERM)
                worker.wait(timeout=25)
            status = u.state(root)
            expected = 'recovery_required' if case == 'failed-recovery' else 'rolled_back'
            assert status['state'] == expected, status
            assert git(root, 'rev-parse', 'HEAD') == old
            assert (home / 'current').resolve() == prefix
            assert cache.read_text() == 'retained cache fixture, not compiled evidence'
            assert cache.stat().st_mtime_ns == cache_stat.st_mtime_ns
            if case == 'interrupted-installer':
                child = int((root / 'runtime_data/import-waiting').read_text())
                assert not Path('/proc', str(child)).exists(), 'Installer child survived interruption'
            if case != 'failed-recovery':
                u.verify(url, old, None, timeout=10)
            else:
                assert status.get('recovery_failure') == '新网关启动后退出', status
                assert status['recovery_failure'] in status['message']
                assert '退出码 1' in status['failure'], status
                try:
                    u.queue(root, target, url)
                except RuntimeError as exc:
                    assert '已有更新事务' in str(exc)
                else:
                    raise AssertionError('Unresolved recovery allowed a new update')
            results.append(dict(case=case, status=status, head=old, current=str(prefix), cache_unchanged=True))
            (base / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2) + '\n')
            print(case + ': ' + expected, flush=True)
    finally:
        stop_current()
        for p in processes:
            if p.poll() is None:
                p.terminate()
            p.wait(timeout=5)
        gitlog.close()


if __name__ == '__main__':
    main()
