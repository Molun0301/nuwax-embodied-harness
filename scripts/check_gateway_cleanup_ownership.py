"""Real-process startup cleanup regressions, always inside a private bwrap sandbox.

Runs production shell functions and a full supervisor start/stop cycle with a
small test worker. It does not start the product gateway or access hardware.
All fixtures and logs are retained under the requested evidence directory.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
import unittest

SLEEP = 'import signal,time,sys; signal.signal(signal.SIGINT,lambda *a:sys.exit(0)); time.sleep(120)'
FUNCTIONS = '''is_truthy geometry_runtime_managed_by_systemd nav_stack_managed_by_systemd
person_follow_managed_by_systemd go2_video_streamer_managed_by_systemd
person_follow_is_alive read_pid_file is_pid_alive pid_pgid process_belongs_to_instance process_managed_by_systemd
send_signal_to_process_scope wait_for_pid_exit terminate_process_scope terminate_person_follow_scope
cleanup_stale_gateway_processes cleanup_stale_geometry_runtime_processes
cleanup_stale_person_follow_processes cleanup_stale_go2_video_streamer
cleanup_external_geometry_processes stop_existing_gateway status_gateway stop_geometry_runtime
stop_nav_stack stop_person_follow stop_go2_video_streamer stop_person_follow_monitor
stop_runtime_activation_monitor stop_runtime_monitors'''.split()


class CleanupTests(unittest.TestCase):
    def setUp(self):
        self.base = EVIDENCE / self._testMethodName
        self.base.mkdir()
        self.root = self.base / 'source'
        (self.root / 'scripts').mkdir(parents=True)
        self.home = self.base / 'installation'
        self.prefix = self.home / 'generations/new'
        self.prefix.mkdir(parents=True)
        self.pidfile = self.base / 'gateway.pid'
        self.environment = dict(os.environ, REPO_ROOT=str(self.root), ENV_FILE=str(self.prefix / 'settings.env'),
                                PID_FILE=str(self.pidfile), PORT='18766', NUWAX_INSTALL_HOME=str(self.home),
                                NUWAX_GATEWAY_STOP_TIMEOUT_SEC='1', NUWAX_GATEWAY_KILL_TIMEOUT_SEC='1',
                                GEOMETRY_RUNTIME_PID='', NUWAX_GEOMETRY_RUNTIME_READY_FILE=str(self.base / 'ready'))
        self.processes = []
        self.counter = 0

    def tearDown(self):
        for p in self.processes:
            if p.poll() is None:
                p.terminate()
        for p in self.processes:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill(); p.wait()

    def spawn(self, name, *, owner='self', markers=None, group=None, code=SLEEP, argv0=None):
        environment = dict(os.environ)
        if owner:
            home = self.home if owner == 'self' else self.base / 'other-installation'
            environment.update(NUWAX_INSTALL_HOME=str(home), NUWAX_BOOTSTRAP_ENV_FILE=str(home / 'generations/old/settings.env'))
        if markers is not None:
            environment.pop('NUWAX_INSTALL_HOME', None)
            environment.pop('NUWAX_BOOTSTRAP_ENV_FILE', None)
            environment.update(markers)
        options = {'start_new_session': True} if group is None else {'preexec_fn': lambda: os.setpgid(0, group)}
        command = [sys.executable, '-c', code, name]
        native_roles = ('nuwax-geometry-runtime', 'nuwax-lcm-runtime', 'nuwax-nav-stack',
                        'nuwax-video-streamer', 'nuwax-person-follow', 'nuwax-go2-video-streamer')
        if name in native_roles:
            command[0] = name
        elif name == 'drivers.go2.video_streamer':
            modules = self.base / 'modules'
            package = modules / 'drivers/go2'
            package.mkdir(parents=True, exist_ok=True)
            (modules / 'drivers/__init__.py').touch()
            (package / '__init__.py').touch()
            (package / 'video_streamer.py').write_text(code)
            environment['PYTHONPATH'] = str(modules)
            command = [sys.executable, '-m', name]
        if argv0 is not None:
            command[0] = argv0
        log = (self.base / ('child-' + str(len(self.processes)) + '.log')).open('w')
        try:
            p = subprocess.Popen(command, executable=sys.executable, env=environment, stdout=log, stderr=subprocess.STDOUT, **options)
        finally:
            log.close()
        self.processes.append(p)
        return p

    def shell(self, code, *, environment=None, expected=0):
        self.counter += 1
        command = 'set -euo pipefail\nsource "$1"\n' + code
        result = subprocess.run(['bash', '-c', command, '--', str(HELPERS)],
                                env=environment or self.environment, text=True, capture_output=True, timeout=15)
        (self.base / ('shell-' + str(self.counter) + '.log')).write_text(result.stdout + result.stderr)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return result

    def alive(self, *processes):
        for p in processes:
            self.assertIsNone(p.poll(), 'Unrelated process was stopped: ' + str(p.pid))

    def exited(self, *processes):
        for p in processes:
            p.wait(timeout=3)

    def test_same_name_and_port_other_instance_survives(self):
        own = self.spawn('uvicorn gateways.app:create_gateway_app_from_env --port 18766')
        other = self.spawn('uvicorn gateways.app:create_gateway_app_from_env --port 18766', owner='other')
        self.shell('cleanup_stale_gateway_processes')
        self.exited(own); self.alive(other)

    def test_owned_gateway_previous_port_is_reclaimed(self):
        own = self.spawn('uvicorn gateways.app:create_gateway_app_from_env --port 19876')
        self.shell('cleanup_stale_gateway_processes')
        self.exited(own)

    def test_native_and_video_residuals_respect_installation(self):
        names = ['nuwax-geometry-runtime', 'nuwax-person-follow', 'drivers.go2.video_streamer', 'nuwax-nav-stack', 'nuwax-lcm-runtime', 'nuwax-video-streamer']
        own = [self.spawn(n) for n in names]
        other = [self.spawn(n, owner='other') for n in names]
        self.shell('cleanup_stale_geometry_runtime_processes\ncleanup_stale_person_follow_processes\ncleanup_stale_go2_video_streamer\ncleanup_external_geometry_processes')
        self.exited(*own); self.alive(*other)

    def test_wrong_pid_file_does_not_stop_or_remove_record(self):
        other = self.spawn('nuwax-gateway-supervisor', owner='other')
        self.pidfile.write_text(str(other.pid))
        self.shell('stop_existing_gateway', expected=1)
        self.alive(other)
        self.assertEqual(self.pidfile.read_text(), str(other.pid))
        self.shell('status_gateway', expected=3)

    def test_old_generation_with_and_without_install_home(self):
        own = self.spawn('nuwax-geometry-runtime')
        older = self.spawn('nuwax-geometry-runtime', markers={'NUWAX_BOOTSTRAP_ENV_FILE': str(self.home / 'generations/legacy/settings.env')})
        self.shell('cleanup_stale_geometry_runtime_processes')
        self.exited(own, older)

    def test_mixed_process_group_preserves_other_owner(self):
        leader = self.spawn('owned-group-leader', group=0)
        own = self.spawn('owned-group-child', group=leader.pid)
        other = self.spawn('other-group-child', owner='other', group=leader.pid)
        self.shell('terminate_process_scope ' + str(leader.pid))
        self.exited(leader, own); self.alive(other)

    def test_public_video_files_and_unrelated_listener_are_preserved(self):
        paths = [Path('/tmp/nuwax_video.sock'), Path('/tmp/nuwax_video.shm')]
        for p in paths:
            p.write_text('other instance')
        ready = self.base / 'listener-ready'
        code = 'import socket,time,pathlib; s=socket.socket();s.bind(("127.0.0.1",0));s.listen(1);pathlib.Path(' + repr(str(ready)) + ').write_text(str(s.getsockname()[1]));time.sleep(120)'
        other = self.spawn('unrelated-listener', owner='other', code=code)
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(ready.exists())
        env = dict(self.environment, NUWAX_GEOMETRY_RUNTIME_PORT=ready.read_text())
        self.shell('cleanup_stale_geometry_runtime_processes\nstop_geometry_runtime', environment=env)
        self.alive(other)
        for p in paths:
            self.assertEqual(p.read_text(), 'other instance')

    def test_mixed_same_installation_group_preserves_managed_components(self):
        leader = self.spawn('uvicorn gateways.app:create_gateway_app_from_env', group=0)
        own = self.spawn('ordinary-owned-child', group=leader.pid)
        managed = [self.spawn(n, group=leader.pid) for n in (
            'nuwax-geometry-runtime', 'nuwax-lcm-runtime', 'nuwax-nav-stack',
            'nuwax-video-streamer', 'nuwax-person-follow', 'drivers.go2.video_streamer')]
        env = dict(self.environment, NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD='true',
                   NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD='true', NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD='true',
                   NUWAX_GO2_VIDEO_STREAMER_MANAGED_BY_SYSTEMD='true')
        self.shell('terminate_process_scope ' + str(leader.pid), environment=env)
        self.exited(leader, own); self.alive(*managed)

    def test_component_words_in_path_and_arguments_do_not_block_stop(self):
        env = dict(self.environment, NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD='true',
                   NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD='true',
                   NUWAX_GO2_VIDEO_STREAMER_MANAGED_BY_SYSTEMD='true')
        for argv0, argument in (
            (str(self.home / 'nuwax-nav-stack/bin/python3'), 'ordinary-owned-process'),
            (sys.executable, '--description=nuwax-geometry-runtime'),
            (sys.executable, '-m drivers.go2.video_streamer')):
            own = self.spawn(argument, argv0=argv0)
            self.pidfile.write_text(str(own.pid))
            self.shell('stop_existing_gateway', environment=env)
            self.exited(own)
            self.assertFalse(self.pidfile.exists())

    def test_retained_live_target_keeps_pid_record(self):
        managed = self.spawn('nuwax-nav-stack')
        self.pidfile.write_text(str(managed.pid))
        env = dict(self.environment, NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD='true')
        result = self.shell('stop_existing_gateway', environment=env, expected=1)
        self.assertIn('still running; keeping PID file', result.stderr)
        self.alive(managed)
        self.assertEqual(self.pidfile.read_text(), str(managed.pid))

    def test_tracked_monitor_stops_preserve_other_and_managed_targets(self):
        for role, function, variable, flag in (
            ('nuwax-person-follow', 'stop_person_follow_monitor', 'PERSON_FOLLOW_MONITOR_PID', 'NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD'),
            ('nuwax-geometry-runtime', 'stop_runtime_activation_monitor', 'RUNTIME_ACTIVATION_MONITOR_PID', 'NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD'),
            ('nuwax-nav-stack', 'stop_runtime_monitors', 'NAV_STACK_MONITOR_PID', 'NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD')):
            own = self.spawn('owned-monitor')
            other = self.spawn(role, owner='other')
            managed = self.spawn(role)
            env = dict(self.environment, NAV_STACK_MONITOR_PID='', GEOMETRY_MONITOR_PID='')
            self.shell(function, environment=dict(env, **{variable: str(own.pid)}))
            self.exited(own)
            for target in (other, managed):
                self.shell(function, environment=dict(env, **{variable: str(target.pid), flag: 'true'}))
            self.alive(other, managed)

    def test_tracked_component_stops_preserve_other_and_managed_targets(self):
        for role, function, variable, flag in (
            ('nuwax-geometry-runtime', 'stop_geometry_runtime', 'GEOMETRY_RUNTIME_PID', 'NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD'),
            ('nuwax-nav-stack', 'stop_nav_stack', 'NAV_STACK_PID', 'NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD'),
            ('nuwax-person-follow', 'stop_person_follow', 'PERSON_FOLLOW_PID', 'NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD'),
            ('drivers.go2.video_streamer', 'stop_go2_video_streamer', 'VIDEO_STREAMER_PID', 'NUWAX_GO2_VIDEO_STREAMER_MANAGED_BY_SYSTEMD')):
            own = self.spawn(role)
            other = self.spawn(role, owner='other')
            self.shell(function, environment=dict(self.environment, **{variable: str(own.pid)}))
            self.exited(own)
            managed = self.spawn(role)
            self.shell(function, environment=dict(self.environment, **{variable: str(other.pid), flag: 'true'}))
            self.shell(function, environment=dict(self.environment, **{variable: str(managed.pid), flag: 'true'}))
            self.alive(other, managed)

    def test_systemd_managed_components_survive_external_cleanup(self):
        own = [self.spawn(n) for n in ('nuwax-geometry-runtime', 'nuwax-lcm-runtime', 'nuwax-nav-stack', 'nuwax-video-streamer', 'nuwax-person-follow', 'drivers.go2.video_streamer')]
        env = dict(self.environment, NUWAX_GEOMETRY_RUNTIME_MANAGED_BY_SYSTEMD='true', NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD='true', NUWAX_PERSON_FOLLOW_MANAGED_BY_SYSTEMD='true', NUWAX_GO2_VIDEO_STREAMER_MANAGED_BY_SYSTEMD='true')
        self.shell('cleanup_stale_geometry_runtime_processes\ncleanup_stale_person_follow_processes\ncleanup_stale_go2_video_streamer\ncleanup_external_geometry_processes', environment=env)
        self.alive(*own)

    def test_direct_bootstrap_instances_are_distinct(self):
        own = self.spawn('nuwax-geometry-runtime', markers={'NUWAX_BOOTSTRAP_ENV_FILE': str(self.root / '.env')})
        other = self.spawn('nuwax-geometry-runtime', markers={'NUWAX_BOOTSTRAP_ENV_FILE': str(self.root / 'other.env')})
        ambiguous = self.spawn('nuwax-geometry-runtime', markers={'NUWAX_BOOTSTRAP_ENV_FILE': '.env'})
        env = dict(self.environment, ENV_FILE=str(self.root / '.env'))
        env.pop('NUWAX_INSTALL_HOME')
        self.shell('cleanup_stale_geometry_runtime_processes', environment=env)
        self.exited(own); self.alive(other, ambiguous)

    def test_legacy_launcher_marker_and_unknown_process(self):
        own = self.spawn('nuwax-geometry-runtime', markers={'NUWAX_SETTINGS_LAUNCHER': str(self.root / 'scripts/start_gateway.sh')})
        unknown = self.spawn('nuwax-geometry-runtime', owner=None)
        env = dict(self.environment, ENV_FILE=str(self.root / '.env'))
        env.pop('NUWAX_INSTALL_HOME')
        self.shell('cleanup_stale_geometry_runtime_processes', environment=env)
        self.exited(own); self.alive(unknown)
        self.pidfile.write_text(str(unknown.pid))
        result = self.shell('stop_existing_gateway', environment=env, expected=1)
        self.assertIn('stop the legacy owner explicitly', result.stderr)

    def test_unmarked_legacy_supervisor_can_stop_then_retry(self):
        legacy = self.root / 'scripts/start_gateway.sh'
        legacy.write_text('#!/bin/bash\ntrap "exit 0" TERM\nwhile true; do sleep 0.1; done\n')
        p = subprocess.Popen(['bash', str(legacy)], cwd=self.root, start_new_session=True)
        self.processes.append(p)
        self.pidfile.write_text(str(p.pid))
        env = dict(self.environment, ENV_FILE=str(self.root / '.env'))
        env.pop('NUWAX_INSTALL_HOME')
        self.shell('stop_existing_gateway', environment=env)
        self.exited(p)
        self.assertFalse(self.pidfile.exists())
        self.shell('stop_existing_gateway', environment=env)

    def prepare_supervisor(self):
        script = self.root / 'scripts/start_gateway.sh'
        shutil.copyfile(SCRIPT, script)
        for relative in ('scripts/lib/config_env.sh', 'core/config_values.py', 'core/config_catalog.json'):
            dest = self.root / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, dest)
        worker = self.root / 'uvicorn'
        worker.mkdir(); (worker / '__init__.py').write_text('')
        (worker / '__main__.py').write_text('import os,pathlib,time\npathlib.Path("worker-ready").write_text(str(os.getpid()))\ntime.sleep(120)\n')
        config = dict(NUWAX_ACTIVE_ROBOT='customrobot', NUWAX_GATEWAY_RESTART='false',
                      NUWAX_GEOMETRY_ENABLED='false', NUWAX_GEOMETRY_EXTERNAL_ENABLED='false',
                      NUWAX_NAV_AUTOSTART='false', NUWAX_PERSON_FOLLOW_AUTOSTART='false',
                      GO2_VIDEO_STREAM_ENABLED='false', NUWAX_GATEWAY_PORT='18766',
                      NUWAX_GATEWAY_PID_FILE=str(self.pidfile), NUWAX_GATEWAY_LOCK_FILE=str(self.base / 'gateway.lock'),
                      NUWAX_GATEWAY_STOP_TIMEOUT_SEC='1', NUWAX_GATEWAY_KILL_TIMEOUT_SEC='1')
        (self.root / '.env').write_text(''.join(k+'='+shlex.quote(v)+'\n' for k,v in config.items()))
        return script

    def cli(self, script, action, expected=0):
        self.counter += 1
        result = subprocess.run(['bash', str(script), action], cwd=self.root, text=True,
                                capture_output=True, timeout=10)
        (self.base / ('cli-' + str(self.counter) + '-' + action + '.log')).write_text(result.stdout + result.stderr)
        self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return result

    def launch_supervisor(self, script, action, label, relative=False, bootstrap=None):
        logpath = self.base / ('supervisor-' + label + '.log')
        with logpath.open('w') as log:
            environment = dict(os.environ)
            if relative:
                environment['NUWAX_BOOTSTRAP_ENV_FILE'] = '.env'
            p = subprocess.Popen(['bash', str(script), action], cwd=self.root, env=environment,
                                 stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(p)
        deadline = time.monotonic() + 8
        while not (self.root / 'worker-ready').exists() and p.poll() is None and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertTrue((self.root / 'worker-ready').exists(), logpath.read_text())
        self.assertEqual(self.pidfile.read_text().strip(), str(p.pid))
        self.assertIn(('NUWAX_BOOTSTRAP_ENV_FILE=' + str(bootstrap or self.root / '.env')).encode(), Path('/proc', str(p.pid), 'environ').read_bytes().split(b'\0'))
        return p

    def test_signal_trap_preserves_ownership_and_managed_predicates(self):
        own = self.spawn('ordinary-owned-child')
        other = self.spawn('ordinary-other-child', owner='other')
        managed = self.spawn('nuwax-nav-stack')
        code = '''check_trap() {
  process_belongs_to_instance "$OWN" || exit 71
  if process_belongs_to_instance "$OTHER"; then exit 72; fi
  process_managed_by_systemd "$MANAGED" || exit 73
  if process_managed_by_systemd "$OWN"; then exit 74; fi
}
trap check_trap TERM
(sleep 0.1; kill -TERM $$) &
wait "$!" || true
'''
        self.shell(code, environment=dict(self.environment, OWN=str(own.pid), OTHER=str(other.pid),
                                         MANAGED=str(managed.pid), NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD='true'))
        self.alive(own, other, managed)

    def test_managed_launcher_supervisor_only_term_releases_children_and_lock(self):
        self.prepare_supervisor()
        (self.root / 'drivers/customrobot').mkdir(parents=True)
        for name in ('manifest.py', 'assembly.py'):
            shutil.copyfile(ROOT / 'drivers/customrobot' / name, self.root / 'drivers/customrobot' / name)
        sys.path.insert(0, str(ROOT))
        from scripts.module_installer.runtime import write_runtime
        (self.prefix / 'venv/bin').mkdir(parents=True)
        (self.prefix / 'venv/bin/python').symlink_to(sys.executable)
        write_runtime(self.root, self.prefix, ('core',), 'customrobot',
                      {'NUWAX_GATEWAY_RESTART': 'on-failure', 'NUWAX_GATEWAY_PORT': '18766',
                       'NUWAX_GATEWAY_STOP_TIMEOUT_SEC': '1', 'NUWAX_GATEWAY_KILL_TIMEOUT_SEC': '1'})
        self.pidfile = self.root / 'runtime_data/gateway/gateway.pid'
        p = self.launch_supervisor(self.prefix / 'start.sh', 'start', 'managed',
                                   bootstrap=self.prefix / 'settings.env')
        other = self.spawn('unrelated-child', owner='other')
        worker = int((self.root / 'worker-ready').read_text())
        p.send_signal(signal.SIGTERM)  # Only the supervisor, as foreground lifecycle.stop_current does.
        self.exited(p)
        marker = ('NUWAX_INSTALL_HOME=' + str(self.home)).encode()
        remaining = []
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            remaining = []
            for path in Path('/proc').iterdir():
                if not path.name.isdigit():
                    continue
                try:
                    if path.joinpath('stat').read_text().rsplit(')', 1)[1].split()[0] == 'Z':
                        continue
                    if marker in path.joinpath('environ').read_bytes().split(b'\0'):
                        remaining.append(int(path.name))
                except OSError:
                    pass
            if not remaining:
                break
            time.sleep(.02)
        self.assertEqual(remaining, [], 'Owned worker/monitor survived supervisor TERM')
        self.assertNotIn(worker, remaining)
        self.assertFalse(self.pidfile.exists())
        with (self.root / 'runtime_data/gateway/start_gateway.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.alive(other)

    def test_full_supervisor_cli_stop_and_second_start(self):
        script = self.prepare_supervisor()
        for cycle in range(2):
            p = self.launch_supervisor(script, 'start', str(cycle), relative=bool(cycle))
            worker_pid = (self.root / 'worker-ready').read_text()
            self.cli(script, 'stop')
            self.exited(p)
            self.assertFalse(self.pidfile.exists())
            self.assertFalse(Path('/proc', worker_pid).exists())
            self.cli(script, 'status', expected=3)
            (self.root / 'worker-ready').unlink()

    def test_full_supervisor_cli_restart(self):
        self.root = self.base / 'nuwax-nav-stack/source'
        (self.root / 'scripts').mkdir(parents=True)
        script = self.prepare_supervisor()
        with (self.root / '.env').open('a') as config:
            config.write('NUWAX_NAV_STACK_MANAGED_BY_SYSTEMD=true\n')
        old = self.launch_supervisor(script, 'start', 'old')
        old_worker = (self.root / 'worker-ready').read_text()
        (self.root / 'worker-ready').unlink()
        new = self.launch_supervisor(script, 'restart', 'new')
        self.exited(old)
        self.assertNotEqual((self.root / 'worker-ready').read_text(), old_worker)
        self.assertFalse(Path('/proc', old_worker).exists())
        self.cli(script, 'status')
        self.cli(script, 'stop')
        self.exited(new)
        self.assertFalse(self.pidfile.exists())

    def test_cli_stop_and_restart_refuse_unmarked_pid(self):
        script = self.prepare_supervisor()
        unknown = self.spawn('nuwax-gateway-supervisor', owner=None)
        self.pidfile.write_text(str(unknown.pid))
        for action in ('stop', 'restart'):
            result = self.cli(script, action, expected=1)
            self.assertIn('stop the legacy owner explicitly', result.stderr)
            self.alive(unknown)
            self.assertEqual(self.pidfile.read_text(), str(unknown.pid))
        self.assertFalse((self.root / 'worker-ready').exists())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--script', type=Path)
    parser.add_argument('--evidence-dir', type=Path, required=True)
    parser.add_argument('--host-pidns')
    args = parser.parse_args()
    global ROOT, EVIDENCE, SCRIPT, HELPERS
    ROOT, EVIDENCE = args.root.resolve(), args.evidence_dir.resolve()
    SCRIPT = (args.script or ROOT / 'scripts/start_gateway.sh').resolve()
    if not args.host_pidns:
        command = ['bwrap', '--unshare-user', '--uid', str(os.getuid()), '--gid', str(os.getgid()),
                   '--unshare-pid', '--unshare-net', '--unshare-ipc', '--unshare-uts', '--die-with-parent', '--new-session',
                   '--cap-drop', 'ALL', '--ro-bind', '/', '/', '--bind', str(EVIDENCE.parent), str(EVIDENCE.parent),
                   '--proc', '/proc', '--tmpfs', '/tmp', '--tmpfs', '/run', '--dev', '/dev', '--clearenv',
                   '--setenv', 'PATH', '/usr/bin:/bin', '--setenv', 'HOME', str(EVIDENCE.parent),
                   '--setenv', 'PYTHONDONTWRITEBYTECODE', '1', '--chdir', str(EVIDENCE.parent),
                   sys.executable, '-B', str(Path(__file__).resolve()), '--root', str(ROOT), '--script', str(SCRIPT),
                   '--evidence-dir', str(EVIDENCE), '--host-pidns', os.readlink('/proc/self/ns/pid')]
        os.execvp(command[0], command)
    if args.host_pidns == os.readlink('/proc/self/ns/pid'):
        raise RuntimeError('Refusing process cleanup tests outside a private PID namespace')
    EVIDENCE.mkdir()
    content = SCRIPT.read_text()
    definitions = []
    for name in FUNCTIONS:
        match = re.search(r'^' + name + r'\(\) \{\n.*?^\}\n', content, re.M | re.S)
        if match:
            definitions.append(match.group())
    HELPERS = EVIDENCE / 'production-functions.sh'
    HELPERS.write_text('\n'.join(definitions))
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CleanupTests))
    (EVIDENCE / 'results.json').write_text(json.dumps(dict(tests=result.testsRun, failures=[(str(t),e) for t,e in result.failures], errors=[(str(t),e) for t,e in result.errors]), indent=2)+'\n')
    return not result.wasSuccessful()


if __name__ == '__main__':
    sys.exit(main())
