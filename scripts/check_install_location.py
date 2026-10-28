#!/usr/bin/env python3
"""Offline install-location argument, prompt and entry-script checks (Python 3.8+)."""
from __future__ import annotations

# ruff: noqa: E402
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import neh_bootstrap as bootstrap
from scripts.device_manager.cli import parser as manager_parser


class LocationChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-location-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.default = self.root / "Nuwax-embodied-harness"
        self.legacy = self.root / ".local/share/neh"
        self.custom = self.root / "NEH 安装 空间"
        self.addCleanup(patch.stopall)
        patch.dict(os.environ, {"HOME": str(self.root), "NEH_HOME": ""}).start()

    def choose(self, arguments=(), *, tty=False, answer=""):
        with patch.object(sys.stdin, "isatty", return_value=tty), patch("builtins.input", return_value=answer) as prompt:
            home = bootstrap.install_home(bootstrap.parser().parse_args(arguments))
        return home, prompt

    def test_defaults_match_manager_without_prompting(self):
        home, prompt = self.choose(tty=True)
        self.assertEqual(home, self.default)
        self.assertEqual(Path(manager_parser().parse_args(["status"]).home).expanduser(), home)
        prompt.assert_not_called()

    def test_legacy_installed_root_is_reused_without_moving_files(self):
        identity = self.legacy / "config/instance-id"
        identity.parent.mkdir(parents=True)
        identity.write_text("existing-installation")
        self.default.mkdir()
        home, prompt = self.choose(["--cli"], tty=True)
        self.assertEqual(home, self.legacy)
        self.assertEqual(Path(manager_parser().parse_args(["status"]).home), self.legacy)
        prompt.assert_not_called()
        self.assertEqual(identity.read_text(), "existing-installation")

    def test_empty_or_unrelated_legacy_directory_does_not_change_default(self):
        self.legacy.mkdir(parents=True)
        for marker in (None, "unrelated-note"):
            if marker:
                (self.legacy / marker).write_text("preserved")
            home, _ = self.choose()
            self.assertEqual(home, self.default)
            self.assertEqual(Path(manager_parser().parse_args(["status"]).home), self.default)
        self.assertEqual((self.legacy / "unrelated-note").read_text(), "preserved")

    def test_default_recognizes_existing_and_interrupted_entry_links(self):
        for relative in ("current", "manager/current"):
            marker = self.legacy / relative
            marker.parent.mkdir(parents=True, exist_ok=True)
            marker.symlink_to(self.root / "missing-version")
            home, prompt = self.choose(["--cli"], tty=True)
            self.assertEqual(home, self.legacy)
            prompt.assert_not_called()
            marker.unlink()

    def test_new_installed_default_wins_when_both_exist(self):
        for home in (self.default, self.legacy):
            (home / "config").mkdir(parents=True)
            (home / "config/instance-id").write_text("installed")
        self.assertEqual(self.choose()[0], self.default)

    def test_inaccessible_default_candidates_do_not_block_explicit_cli_paths(self):
        with patch.object(Path, "is_file", side_effect=PermissionError("cannot inspect old installation")):
            self.assertEqual(Path(manager_parser().parse_args(["--home", str(self.custom), "status"]).home), self.custom)
            self.assertEqual(self.choose()[0], self.default)

    def test_explicit_legacy_and_other_exact_paths_are_not_renamed(self):
        for target in (self.legacy, self.default, self.custom):
            self.assertEqual(self.choose(["--install-dir", str(target)])[0], target)

    def test_both_aliases_override_environment_with_spaces_and_unicode(self):
        os.environ["NEH_HOME"] = str(self.root / "other")
        for flag in ("--install-dir", "--home"):
            for arguments in ([flag, str(self.custom)], [flag + "=" + str(self.custom)]):
                with self.subTest(arguments=arguments):
                    home, prompt = self.choose(["--cli"] + arguments, tty=True)
                    self.assertEqual(home, self.custom)
                    self.assertEqual(Path(manager_parser().parse_args(arguments + ["status"]).home), self.custom)
                    prompt.assert_not_called()

    def test_environment_is_shared_and_tilde_expanded(self):
        os.environ["NEH_HOME"] = "~/NEH 安装 空间"
        home, prompt = self.choose(["--cli"], tty=True)
        self.assertEqual(home, self.custom)
        self.assertEqual(Path(manager_parser().parse_args(["status"]).home).expanduser(), home)
        prompt.assert_not_called()

    def test_first_cli_install_can_choose_location(self):
        home, prompt = self.choose(["--cli"], tty=True, answer="~/NEH 安装 空间")
        self.assertEqual(home, self.custom)
        prompt.assert_called_once()
        self.assertIn(str(self.default), prompt.call_args.args[0])
        self.assertFalse(home.exists(), "Location selection must not create a second installation")

    def test_enter_keeps_default(self):
        home, prompt = self.choose(["--cli"], tty=True)
        self.assertEqual(home, self.default)
        prompt.assert_called_once()

    def test_eof_cancels_before_creating_installation(self):
        with patch.object(sys.stdin, "isatty", return_value=True), patch("builtins.input", side_effect=EOFError):
            with self.assertRaisesRegex(bootstrap.BootstrapError, "已取消"):
                bootstrap.install_home(bootstrap.parser().parse_args(["--cli"]))
        self.assertFalse(self.default.exists())

    def test_cli_without_tty_does_not_prompt(self):
        home, prompt = self.choose(["--cli"])
        self.assertEqual(home, self.default)
        prompt.assert_not_called()

    def test_configured_install_does_not_prompt(self):
        home, prompt = self.choose(["--configure", "fixture.json"], tty=True)
        self.assertEqual(home, self.default)
        prompt.assert_not_called()

    def test_existing_default_and_interrupted_rerun_do_not_prompt(self):
        self.default.mkdir(parents=True)
        home, prompt = self.choose(["--cli"], tty=True)
        self.assertEqual(home, self.default)
        prompt.assert_not_called()

    def test_relative_directory_resolves_from_invoking_directory(self):
        home, _ = self.choose(["--install-dir", "NEH 安装 空间"])
        self.assertEqual(home, Path.cwd() / "NEH 安装 空间")

    def test_help_exposes_location_flag(self):
        for parser in (bootstrap.parser(), manager_parser()):
            help_text = parser.format_help()
            self.assertIn("--install-dir", help_text)
            self.assertIn("--home", help_text)
            self.assertIn("NEH_HOME", help_text)

    def test_systemd_keeps_custom_home_as_one_argument(self):
        home = self.custom / "100% $data"
        service = bootstrap.service_text(home, sys.executable, "127.0.0.1", 8765)
        line = next(line for line in service.splitlines() if line.startswith("ExecStart="))
        self.assertIn('"--home" ' + bootstrap.systemd_quote(str(home)), line)
        self.assertIn("WorkingDirectory=" + str(home / "manager/current").replace("%", "%%"), service)


class EntryChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-entry-location-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.capture = self.root / "captured.json"
        self.custom = self.root / "NEH 安装 空间"
        self.capture_script = self.root / "capture.py"
        self.capture_script.write_text(
            "import json, os, sys\nfrom pathlib import Path\n"
            "Path(os.environ['NEH_TEST_CAPTURE']).write_text(json.dumps({'args': sys.argv[1:], 'home': os.environ.get('NEH_HOME'), 'tty': sys.stdin.isatty()}))\n")
        self.environment = dict(os.environ, NEH_TEST_CAPTURE=str(self.capture), NEH_TEST_PYTHON=sys.executable,
                                NEH_TEST_CAPTURE_SCRIPT=str(self.capture_script),
                                NEH_HOME=str(self.root / "environment home"), PATH=str(self.root) + os.pathsep + os.environ["PATH"])

    def run_public_entry(self, arguments, *, terminal=False):
        python = self.root / "python3"
        python.write_text('#!/usr/bin/env bash\nset -eu\nif [[ $1 == - ]]; then\n'
                          '  mkdir -p "$NEH_BOOTSTRAP_STAGE/scripts"\n'
                          '  cp "$NEH_TEST_CAPTURE_SCRIPT" "$NEH_BOOTSTRAP_STAGE/scripts/neh_bootstrap.py"\n'
                          '  printf "%s" aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa > "$NEH_BOOTSTRAP_STAGE/revision"\n'
                          'else\n  exec "$NEH_TEST_PYTHON" "$@"\nfi\n')
        python.chmod(0o755)
        command = ["bash", "get-neh.sh"] + arguments
        if terminal:
            command = [sys.executable, "-c", "import os, pty, sys; status = pty.spawn(sys.argv[1:]); "
                       "sys.exit(os.WEXITSTATUS(status) if os.WIFEXITED(status) else 1)"] + command
        subprocess.run(command, cwd=str(ROOT), env=self.environment, check=True,
                       capture_output=True, text=True, start_new_session=True, timeout=15)
        captured = json.loads(self.capture.read_text())
        self.assertEqual(captured["args"], arguments + ["--revision", "a" * 40])
        if terminal:
            self.assertTrue(captured["tty"])

    def test_public_script_forwards_location_after_preparing_bootstrap(self):
        self.run_public_entry(["--install-dir", str(self.custom)])

    def test_cli_without_controlling_terminal_still_reaches_bootstrap(self):
        self.run_public_entry(["--cli", "--install-dir", str(self.custom)])

    def test_cli_with_controlling_terminal_passes_tty_to_bootstrap(self):
        self.run_public_entry(["--cli", "--install-dir", str(self.custom)], terminal=True)


if __name__ == "__main__":
    unittest.main()
