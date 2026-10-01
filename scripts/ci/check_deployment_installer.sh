#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

bash -n \
  install.sh \
  scripts/deploy_installer/entrypoint.sh \
  scripts/install_motion_guard_deployment.sh \
  scripts/systemd/check_motion_guard_python.sh

python3 -m compileall -q -f install.py scripts/deploy_installer scripts/module_installer
ruff check scripts/deploy_installer install.py
# .ruff.toml 保留 Python 3.8 所需的运行时 typing 注解。
ruff check scripts/module_installer
python3 scripts/check_installation_lifecycle.py
python3 install.py --help >/dev/null
python3 install.py install --modules core --robot customrobot --json | python3 -c '
import json, sys
plan = json.load(sys.stdin)["plan"]
assert plan["modules"] == ["core"]
assert plan["models"] == [] and plan["builds"] == []
'
python3 install.py install --modules pointlio --robot customrobot --json | python3 -c '
import json, sys
plan = json.load(sys.stdin)["plan"]
assert plan["modules"] == ["core", "geometry", "pointlio"]
assert "libgoogle-glog-dev" in plan["system_packages"]
'
plan_file="$(mktemp -t neh-install-plan.XXXXXX)"
trap 'rm -f "${plan_file}"' EXIT
python3 install.py plan --profile go2-jetson --release-id ci-contract --go2-robot-ip 192.0.2.10 >"${plan_file}"

grep -q 'rust_build' "${plan_file}"
grep -q 'nuwax-person-follow' "${plan_file}"
grep -q '/api/health/full' "${plan_file}"

python3 install.py install --preset map2d --robot customrobot --json | python3 -c '
import json, sys
plan = json.load(sys.stdin)["plan"]
assert plan["modules"] == ["core", "geometry", "slam2d"]
assert plan["models"] == []
'

# 标准库契约检查仅使用临时目录与模拟 runner，不操作实机服务。
python3 - <<'PY'
import io
import json
import tempfile
import unittest
from contextlib import ExitStack, redirect_stderr
from dataclasses import replace
from itertools import product
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from scripts.deploy_installer import operations
from scripts.deploy_installer.cli import _load_json, _newest_transaction_state, _options, _save_transaction_state, _status, build_parser
from scripts.deploy_installer.engine import Action, DeploymentEngine, DeploymentPlan, Phase
from scripts.deploy_installer.lock import InstallerLock
from scripts.deploy_installer.gateway import gateway_health_url, resolve_gateway_listener
from scripts.deploy_installer.model import DeploymentOptions
from scripts.deploy_installer.operations import DeploymentError, load_persisted_recovery
from scripts.deploy_installer.runner import RecordingRunner


class DeploymentContracts(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="neh-deployment-contract-")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def test_invalid_state_is_reported_without_traceback(self):
        for content in ("[]", "null", '"invalid"'):
            with self.subTest(content=content):
                (self.root / "state.json").write_text(content, encoding="utf-8")
                with self.assertRaises(TypeError):
                    _load_json(self.root / "state.json", "deployment state")
                error = io.StringIO()
                with redirect_stderr(error):
                    self.assertEqual(_status(self.root, self.root / "install"), 1)
                self.assertNotIn("Traceback", error.getvalue())
                self.assertIn("must be a JSON object", error.getvalue())

    def test_transaction_types_fail_before_writing(self):
        with self.assertRaises(TypeError):
            _save_transaction_state(self.root, {"transaction_id": None})
        self.assertFalse((self.root / "state.json").exists())
        journal = self.root / "transactions"
        journal.mkdir()
        for identity in ({"transaction_id": 1, "started_at": 1}, {"transaction_id": "bad", "started_at": "1"}):
            with self.subTest(identity=identity):
                (journal / "bad.json").write_text(json.dumps(identity), encoding="utf-8")
                with self.assertRaises(TypeError):
                    _newest_transaction_state(self.root)

    def test_phase_failure_records_rollback_outcome(self):
        for rollback_fails in (False, True):
            with self.subTest(rollback_fails=rollback_fails):
                options = replace(DeploymentOptions.for_tests(source_root=self.root), persist_state=True)
                calls = []

                def fail(context):
                    raise ValueError("phase failure")

                def rollback(context):
                    calls.append("rollback")
                    if rollback_fails:
                        raise OSError("rollback failure")
                    return True

                plan = DeploymentPlan(
                    "contract", "fingerprint", (Phase("preflight", "contract", (Action.callback("fail", fail),)),),
                    rollback=rollback,
                )
                runner = RecordingRunner()
                with self.assertLogs("scripts.deploy_installer.engine", level="ERROR") as logs:
                    result = DeploymentEngine(runner).execute(plan, options)
                self.assertEqual(calls, ["rollback"])
                self.assertEqual(runner.executed, [])
                self.assertEqual(result.status, "failed")
                self.assertEqual(result.failed_phase, "preflight")
                state = json.loads(options.state_file.read_text(encoding="utf-8"))
                self.assertEqual(state["status"], "rollback_failed" if rollback_fails else "rolled_back")
                self.assertIn("phase failure", result.message)
                self.assertEqual(len(logs.records), 2 if rollback_fails else 1)
                if rollback_fails:
                    self.assertIn("rollback failure", result.message)

    def test_installer_lock_still_excludes_other_instances(self):
        path = self.root / "lock" / "installer.lock"
        first = InstallerLock(path)
        with first as held:
            self.assertIs(held, first)
            with self.assertRaises(RuntimeError):
                with InstallerLock(path):
                    self.fail("second installer acquired the same lock")
        with InstallerLock(path):
            pass

    def test_recovery_rejects_unrecorded_current_release(self):
        cases = product(
            ("prepared", "activated", "rolling_back"),
            (None, "previous", "candidate"),
            (None, "previous", "candidate", "unrelated", "directory"),
        )
        for number, (state, start, current) in enumerate(cases):
            with self.subTest(state=state, start=start, current=current):
                options = DeploymentOptions.for_tests(source_root=self.root / str(number))
                targets = {
                    "candidate": options.release_dir,
                    "previous": options.releases_dir / "previous",
                    "unrelated": options.releases_dir / "unrelated",
                    None: None,
                }
                for target in targets.values():
                    if target is not None:
                        target.mkdir(parents=True)
                if current == "directory":
                    options.current_link.mkdir()
                elif current is not None:
                    options.current_link.symlink_to(targets[current])
                same_current = start == "candidate"
                recovery = {
                    "schema_version": 2,
                    "current_link": str(options.current_link),
                    "candidate_release": str(options.release_dir),
                    "activation_state": state,
                    "current_at_start": str(targets[start]) if start is not None else None,
                    "previous_release": str(targets[start]) if start is not None and not same_current else None,
                    "same_current": same_current,
                }
                backup = options.state_dir / "backups" / options.transaction_id
                backup.mkdir(parents=True)
                (backup / "recovery.json").write_text(json.dumps(recovery), encoding="utf-8")
                context = SimpleNamespace(options=options, notes={}, config_backups={}, release_switched=False)
                expected_valid = (
                    current == "candidate"
                    if state == "activated" or same_current
                    else current in (start, "candidate")
                )
                if expected_valid:
                    load_persisted_recovery(context)
                    self.assertEqual(context.release_switched, current == "candidate" and not same_current)
                else:
                    with self.assertRaises(DeploymentError):
                        load_persisted_recovery(context)

    def test_gateway_options_and_cli(self):
        options = DeploymentOptions.for_tests(source_root=self.root)
        for port in (1, 9876, 65535):
            self.assertEqual(replace(options, gateway_port=port).gateway_port, port)
        for port in (0, -1, 65536, True, "9876", None):
            with self.subTest(port=port), self.assertRaises((TypeError, ValueError)):
                replace(options, gateway_port=port)
        for host in ("", "http://127.0.0.1", "127.0.0.1:9000", "127.0.0.1/path", "bad\n", "::1%eth0/other"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                replace(options, gateway_host=host)
        args = build_parser().parse_args([
            "plan", "--release-id", "contract", "--source-root", str(self.root),
            "--gateway-host", "::", "--gateway-port", "9001", "--require-update-readiness",
        ])
        parsed = _options(args, dry_run=True)
        self.assertEqual((parsed.gateway_host, parsed.gateway_port), ("::", 9001))
        self.assertTrue(parsed.require_update_readiness)
        self.assertFalse(options.require_update_readiness)

    def test_gateway_configuration_respects_profile_and_restored_values(self):
        release = self.root / "release"
        config = self.root / "config"
        config.mkdir()
        root_config = config / "nuwax.env"
        root_config.write_text('NUWAX_GATEWAY_HOST="127.0.0.2"\nNUWAX_GATEWAY_PORT="9011" # custom\n')
        paths = {name: self.root / (name + ".env") for name in ("geometry_env", "pointcloud_env", "user_env")}
        paths["pointcloud_env"].write_text("NUWAX_GATEWAY_PORT=9999\n")
        self.assertEqual(resolve_gateway_listener(release, config, **paths), ("127.0.0.2", 9011))
        paths["user_env"].write_text("NUWAX_GATEWAY_HOST=localhost\nNUWAX_GATEWAY_PORT=9022\n")
        self.assertEqual(resolve_gateway_listener(release, config, **paths), ("localhost", 9022))
        driver = release / "drivers" / "customrobot"
        (driver / "configs").mkdir(parents=True)
        for name in ("manifest.py", "assembly.py"):
            (driver / name).write_text("")
        selected = driver / "configs" / "selected.env"
        selected.write_text("NUWAX_GATEWAY_HOST=::\nNUWAX_GATEWAY_PORT=9033\n")
        active = release / "runtime_data" / "settings" / "active.env"
        active.parent.mkdir(parents=True)
        active.symlink_to(selected)
        self.assertEqual(resolve_gateway_listener(release, config, **paths), ("::", 9033))
        selected.write_text("NUWAX_ACTIVE_ROBOT=customrobot\n")
        self.assertEqual(resolve_gateway_listener(release, config, **paths), ("0.0.0.0", 8766))
        active.unlink()
        paths["user_env"].unlink()
        saved = b"NUWAX_GATEWAY_HOST=::1\nNUWAX_GATEWAY_PORT=9044\n"
        self.assertEqual(resolve_gateway_listener(release, config, config_overrides={root_config: saved}, **paths),
                         ("::1", 9044))
        self.assertEqual(resolve_gateway_listener(release, config, **paths), ("127.0.0.2", 9011))

    def test_health_probe_uses_listener_without_proxies_or_redirects(self):
        cases = (("0.0.0.0", "127.0.0.1"), ("::", "[::1]"), ("::1", "[::1]"),
                 ("192.0.2.7", "192.0.2.7"), ("localhost", "localhost"))
        for host, target in cases:
            with self.subTest(host=host):
                response = MagicMock(status=200)
                response.__enter__.return_value = response
                response.read.return_value = b'{"ready": true, "overall": "healthy"}'
                with patch.object(operations.urllib.request, "build_opener") as build:
                    build.return_value.open.return_value = response
                    operations._health_ready(gateway_host=host, gateway_port=9876)
                    expected = "http://" + target + ":9876/api/health/full"
                    self.assertEqual(gateway_health_url(host, 9876), expected)
                    build.return_value.open.assert_called_once_with(expected, timeout=5)
                    proxy, redirect = build.call_args.args
                    self.assertEqual(proxy.proxies, {})
                    self.assertIsNone(redirect.redirect_request(None, None, 302, "", {}, "http://other/"))
        response.read.return_value = b'{"ready": true, "overall": "unhealthy"}'
        with patch.object(operations.urllib.request, "build_opener") as build:
            build.return_value.open.return_value = response
            with patch.object(operations.time, "monotonic", side_effect=[0, 0, 100]), patch.object(operations.time, "sleep"):
                with self.assertRaises(DeploymentError):
                    operations._health_ready(gateway_host="::1", gateway_port=9876)

    def test_readiness_requires_literal_true_from_current_listener(self):
        options = DeploymentOptions.for_tests(source_root=self.root)
        previous = options.releases_dir / "previous"
        previous.mkdir(parents=True)
        options.current_link.symlink_to(previous)
        saved = {options.config_dir / "nuwax.env": b"NUWAX_GATEWAY_PORT=9090\n"}
        context = SimpleNamespace(options=options, config_backups=saved)
        for payload in ({"ready": True}, {"ready": False}, {"ready": 1}, {}, []):
            with self.subTest(payload=payload):
                response = MagicMock(status=200)
                response.__enter__.return_value = response
                response.read.return_value = json.dumps(payload).encode()
                with patch.object(operations, "resolve_gateway_listener", return_value=("::", 9090)) as resolve:
                    with patch.object(operations.urllib.request, "build_opener") as build:
                        build.return_value.open.return_value = response
                        if payload == {"ready": True} and isinstance(payload.get("ready"), bool):
                            operations._require_update_readiness(context)
                        else:
                            with self.assertRaises(DeploymentError):
                                operations._require_update_readiness(context)
                        build.return_value.open.assert_called_once_with("http://[::1]:9090/api/updates/readiness", timeout=5)
                        self.assertEqual(build.call_args.args[0].proxies, {})
                        resolve.assert_called_once_with(previous, options.config_dir, config_overrides=saved)

    def test_activation_checks_readiness_before_stopping_or_switching(self):
        cases = ((True, False, False), (True, False, True), (True, True, False),
                 (True, True, True), (False, False, False))
        for number, (required, no_systemd, denied) in enumerate(cases):
            with self.subTest(required=required, no_systemd=no_systemd, denied=denied):
                options = replace(DeploymentOptions.for_tests(source_root=self.root / str(number)),
                                  require_update_readiness=required, no_systemd=no_systemd)
                previous = options.releases_dir / "previous"
                (previous / "drivers").mkdir(parents=True)
                options.release_dir.mkdir()
                (options.release_dir / "release-manifest.json").write_text('{"fingerprint": "expected"}')
                options.current_link.symlink_to(previous)
                calls = []
                context = SimpleNamespace(options=options, plan=SimpleNamespace(fingerprint="expected"),
                                          runner=SimpleNamespace(run=lambda *args, **kwargs: calls.append("stop")))

                def gate(_context):
                    calls.append("readiness")
                    if denied:
                        raise DeploymentError("not ready")

                with ExitStack() as stack:
                    stack.enter_context(patch.object(operations, "_validate_managed_release", return_value=previous))
                    stack.enter_context(patch.object(operations, "_deployment_env", return_value={}))
                    for name, callback in (("_require_update_readiness", gate),
                                           ("_migrate_robot_profiles", lambda *args: calls.append("migrate")),
                                           ("_switch_current", lambda *args: calls.append("switch"))):
                        stack.enter_context(patch.object(operations, name, side_effect=callback))
                    if denied:
                        with self.assertRaises(DeploymentError):
                            operations.activate_release(context)
                        self.assertNotIn("stop", calls)
                        self.assertNotIn("switch", calls)
                    else:
                        operations.activate_release(context)
                        expected = ["migrate", "readiness", "switch"] if no_systemd else ["readiness", "stop", "migrate", "switch"]
                        if not required:
                            expected.remove("readiness")
                        self.assertEqual(calls, expected)

    def test_configuration_writes_managed_gateway_port(self):
        options = replace(DeploymentOptions.for_tests(source_root=self.root), gateway_host="::", gateway_port=9876)
        (self.root / ".env.example").write_text("NUWAX_GATEWAY_PORT=8766\n")
        context = SimpleNamespace(options=options, profile=SimpleNamespace(robot="customrobot", models={}),
                                  notes={}, config_backups={})
        operations.write_configuration(context)
        rendered = (options.config_dir / "nuwax.env").read_text()
        self.assertIn("NUWAX_GATEWAY_HOST=::\n", rendered)
        self.assertIn("NUWAX_GATEWAY_PORT=9876\n", rendered)


unittest.main(verbosity=2)
PY

# 测试按项目约定仅存在于开发机；远端检查不依赖未入仓的文件。
if [[ "${1:-}" == "--with-tests" ]]; then
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
    tests/deployment \
    tests/unit/test_runtime_deployment_config_contract.py \
    tests/unit/test_motion_guard_deployment.py \
    tests/unit/test_systemd_runtime_readiness.py
elif [[ $# -gt 0 ]]; then
  echo "用法：$0 [--with-tests]" >&2
  exit 2
fi
bash scripts/install_motion_guard_deployment.sh check-sources

echo "deployment installer checks: PASS"
