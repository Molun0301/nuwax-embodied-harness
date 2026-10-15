#!/usr/bin/env python3
"""Offline first-stage TLS retry, transport isolation and entry integration checks."""
import http.client
import io
import json
import os
from pathlib import Path
import ssl
import subprocess
import sys
import tempfile
import unittest
import urllib.error
import urllib.request
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
ENTRY = (ROOT / "get-neh.sh").read_text()
EMBEDDED = ENTRY.split("python3 - <<'PY'\n", 1)[1].split("\nPY\n", 1)[0]
DEFINITIONS = EMBEDDED.split('\nlocal = ', 1)[0]
URL = "https://api.github.com/repos/owner/project/commits/main"
ACCEPT = "application/vnd.github+json"
SECRET = "fixture-secret-do-not-print"


class Response(io.BytesIO):
    length = 0


class DownloadChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-download-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.token_file = self.root / "token"
        self.token_file.write_text(SECRET)
        self.token_file.chmod(0o600)
        self.stderr = io.StringIO()
        self.addCleanup(patch.stopall)
        self.sleep = patch("time.sleep").start()
        patch("sys.stderr", self.stderr).start()
        self.load()

    def load(self, *, token=False, gh=False):
        values = dict(NEH_BOOTSTRAP_REPOSITORY="https://github.com/owner/project.git", NEH_BOOTSTRAP_BRANCH="main",
                      NEH_BOOTSTRAP_REVISION="", NEH_BOOTSTRAP_STAGE=str(self.root),
                      NEH_BOOTSTRAP_TOKEN_FILE=str(self.token_file) if token else "")
        self.ns = {}
        with patch.dict(os.environ, values), patch("shutil.which", return_value="/fixture/gh" if gh else None), \
                patch("subprocess.run", return_value=subprocess.CompletedProcess([], 0)):
            exec(compile(DEFINITIONS, "get-neh.sh:embedded", "exec"), self.ns)
        self.opener = Mock()
        self.ns["opener"] = self.opener
        self.download = self.ns["download"]

    def gh_run(self, status=200, body=b"complete", *, returncode=0, stderr=b"", header=True):
        def run(command, **kwargs):
            if header:
                kwargs["stdout"].write(b"HTTP/2.0 " + str(status).encode() + b" Test\nX-Fixture: yes\r\n\r\n")
            kwargs["stdout"].write(body)
            return subprocess.CompletedProcess(command, returncode, stderr=stderr)
        return run

    def assert_clean_failure(self, reason):
        with self.assertRaises(SystemExit) as caught:
            self.download(URL, ACCEPT)
        diagnostic = str(caught.exception) + self.stderr.getvalue()
        self.assertIn(reason, diagnostic)
        self.assertNotIn(SECRET, diagnostic)
        self.assertNotIn("Traceback", diagnostic)
        return diagnostic

    def test_anonymous_uses_python_even_when_gh_installed(self):
        self.load(gh=True)
        self.opener.open.return_value = Response(b"public")
        self.assertEqual(self.download(URL, ACCEPT), b"public")
        self.assertIsNone(self.ns["gh"])
        request = self.opener.open.call_args.args[0]
        self.assertIsNone(request.get_header("Authorization"))

    def test_token_without_gh_keeps_python_fallback(self):
        self.load(token=True)
        self.opener.open.return_value = Response(b"private")
        self.assertEqual(self.download(URL, ACCEPT), b"private")
        self.assertEqual(self.opener.open.call_args.args[0].get_header("Authorization"), "Bearer " + SECRET)

    def test_ssl_eof_retries_then_succeeds(self):
        self.opener.open.side_effect = [ssl.SSLEOFError(8, SECRET),
                                       urllib.error.URLError(ssl.SSLEOFError(8, SECRET)), Response(b"complete")]
        self.assertEqual(self.download(URL, ACCEPT), b"complete")
        self.assertEqual(self.opener.open.call_count, 3)
        self.assertEqual([call.args for call in self.sleep.call_args_list], [(1,), (2,)])
        self.assertEqual([call.kwargs["timeout"] for call in self.opener.open.call_args_list], [45] * 3)
        self.assertNotIn(SECRET, self.stderr.getvalue())

    def test_read_interruption_discards_partial_body_before_retry(self):
        broken = Mock()
        broken.__enter__ = Mock(return_value=broken)
        broken.__exit__ = Mock(return_value=False)
        broken.read.side_effect = http.client.IncompleteRead(b"partial " + SECRET.encode())
        self.opener.open.side_effect = [broken, Response(b"complete")]
        self.assertEqual(self.download(URL, ACCEPT), b"complete")
        self.assertEqual(self.opener.open.call_count, 2)
        self.assertNotIn(SECRET, self.stderr.getvalue())

    def test_silently_truncated_content_length_is_retried(self):
        class Socket:
            def makefile(self, *args):
                return io.BytesIO(b"HTTP/1.1 200 OK\r\nContent-Length: 100\r\n\r\nprint(1)\n")
        truncated = http.client.HTTPResponse(Socket())
        truncated.begin()
        self.opener.open.side_effect = [truncated, Response(b"complete")]
        self.assertEqual(self.download(URL, ACCEPT), b"complete")
        self.assertEqual(self.opener.open.call_count, 2)

    def test_retry_exhaustion_has_safe_actionable_error(self):
        self.opener.open.side_effect = ssl.SSLEOFError(8, SECRET)
        diagnostic = self.assert_clean_failure("尝试3/3次")
        self.assertIn("重新运行原安装命令", diagnostic)
        self.assertEqual(self.opener.open.call_count, 3)
        self.assertEqual(self.sleep.call_count, 2)

    def test_timeout_and_connection_reset_are_transient(self):
        for error in (TimeoutError(SECRET), ConnectionResetError(SECRET)):
            with self.subTest(error=type(error)):
                self.opener.open.side_effect = [error, Response(b"complete")]
                self.assertEqual(self.download(URL, ACCEPT), b"complete")

    def test_certificate_and_unknown_tls_errors_fail_once(self):
        for error in (ssl.SSLCertVerificationError(1, SECRET),
                      urllib.error.URLError(ssl.SSLCertVerificationError(1, SECRET)),
                      ssl.SSLError(1, SECRET)):
            with self.subTest(error=type(error)):
                self.opener.open.reset_mock()
                self.opener.open.side_effect = error
                self.assert_clean_failure("TLS")
                self.assertEqual(self.opener.open.call_count, 1)
        self.sleep.assert_not_called()

    def test_http_auth_and_not_found_errors_never_retry(self):
        for status in (400, 401, 403, 404, 422):
            with self.subTest(status=status):
                self.opener.open.reset_mock()
                self.opener.open.side_effect = urllib.error.HTTPError(URL, status, SECRET, {}, None)
                self.assert_clean_failure("HTTP " + str(status))
                self.assertEqual(self.opener.open.call_count, 1)
        self.sleep.assert_not_called()

    def test_transient_http_statuses_retry(self):
        for status in (408, 429, 500, 502, 503, 504):
            with self.subTest(status=status):
                self.opener.open.side_effect = [urllib.error.HTTPError(URL, status, SECRET, {}, None), Response(b"complete")]
                self.assertEqual(self.download(URL, ACCEPT), b"complete")

    def test_oversized_python_body_fails_once(self):
        self.opener.open.return_value = Response(b"x" * (self.ns["MAX_DOWNLOAD"] + 1))
        self.assert_clean_failure("超过大小限制")
        self.opener.open.assert_called_once()
        self.sleep.assert_not_called()

    def test_python_redirect_keeps_https_and_strips_cross_host_credentials(self):
        redirect = self.ns["Redirect"]()
        request = urllib.request.Request(URL, headers={"Authorization": "Bearer " + SECRET})
        same = redirect.redirect_request(request, None, 302, "", {}, URL + "/next")
        self.assertEqual(same.get_header("Authorization"), "Bearer " + SECRET)
        other = redirect.redirect_request(request, None, 302, "", {}, "https://codeload.github.com/archive")
        self.assertIsNone(other.get_header("Authorization"))
        with self.assertRaisesRegex(ValueError, "不安全"):
            redirect.redirect_request(request, None, 302, "", {}, "http://api.github.com/archive")

    def test_gh_existing_token_is_isolated_and_never_an_argument(self):
        self.load(token=True, gh=True)
        self.ns["environment"].update(GH_DEBUG="api", DEBUG="1", GH_HOST="untrusted.example",
                                     GH_FORCE_TTY="1", CLICOLOR_FORCE="1", GITHUB_TOKEN="other-token")
        with patch("subprocess.run", side_effect=self.gh_run(body=b"\nraw source\x00\n")) as run:
            self.assertEqual(self.download(URL, ACCEPT), b"\nraw source\x00\n")
        command = run.call_args.args[0]
        options = run.call_args.kwargs
        self.assertEqual(command[:4], ["/fixture/gh", "api", "--hostname", "github.com"])
        self.assertEqual(command[-1], URL)
        self.assertNotIn(SECRET, " ".join(command))
        self.assertEqual(options["env"]["GH_TOKEN"], SECRET)
        self.assertEqual(options["env"]["GH_HOST"], "github.com")
        self.assertEqual(options["env"]["GH_PROMPT_DISABLED"], "1")
        self.assertEqual(options["env"]["NO_COLOR"], "1")
        for key in ("GH_DEBUG", "DEBUG", "GH_FORCE_TTY", "CLICOLOR_FORCE", "GITHUB_TOKEN"):
            self.assertNotIn(key, options["env"])
        self.assertEqual(options["stdin"], subprocess.DEVNULL)
        self.assertEqual(options["timeout"], 45)
        self.opener.open.assert_not_called()

    def test_gh_retries_failed_body_even_after_200_status(self):
        self.load(token=True, gh=True)
        effects = [self.gh_run(body=b"partial", returncode=1, stderr=b"unexpected EOF " + SECRET.encode()),
                   self.gh_run(body=b"complete")]
        with patch("subprocess.run", side_effect=lambda *args, **kwargs: effects.pop(0)(*args, **kwargs)) as run:
            self.assertEqual(self.download(URL, ACCEPT), b"complete")
        self.assertEqual(run.call_count, 2)
        self.assertNotIn(SECRET, self.stderr.getvalue())

    def test_gh_http_auth_status_overrides_transient_looking_stderr(self):
        self.load(token=True, gh=True)
        for status in (401, 403, 404):
            with self.subTest(status=status), patch("subprocess.run", side_effect=self.gh_run(
                    status, returncode=1, stderr=b"unexpected EOF " + SECRET.encode())) as run:
                self.assert_clean_failure("HTTP " + str(status))
                run.assert_called_once()
        self.sleep.assert_not_called()
        self.opener.open.assert_not_called()

    def test_gh_transient_http_retries_then_succeeds(self):
        self.load(token=True, gh=True)
        for status in (408, 429, 500, 502, 503, 504):
            with self.subTest(status=status):
                effects = [self.gh_run(status, returncode=1), self.gh_run()]
                with patch("subprocess.run", side_effect=lambda *args, **kwargs: effects.pop(0)(*args, **kwargs)) as run:
                    self.assertEqual(self.download(URL, ACCEPT), b"complete")
                self.assertEqual(run.call_count, 2)

    def test_gh_timeout_exhaustion_is_bounded_and_redacted(self):
        self.load(token=True, gh=True)
        error = subprocess.TimeoutExpired([SECRET], 45, output=SECRET.encode(), stderr=SECRET.encode())
        with patch("subprocess.run", side_effect=error) as run:
            self.assert_clean_failure("尝试3/3次")
        self.assertEqual(run.call_count, 3)
        self.assertEqual(self.sleep.call_count, 2)

    def test_gh_certificate_unknown_and_auth_exit_fail_once(self):
        self.load(token=True, gh=True)
        for code, error in ((1, b"x509: certificate signed by unknown authority "),
                            (1, b"invalid option "), (4, b"authentication required ")):
            with self.subTest(code=code, error=error), patch("subprocess.run", side_effect=self.gh_run(
                    returncode=code, stderr=error + SECRET.encode(), header=False)) as run:
                self.assert_clean_failure("无法获取安装源码")
                run.assert_called_once()
        self.sleep.assert_not_called()

    def test_gh_oversized_body_is_rejected(self):
        self.load(token=True, gh=True)
        with patch("subprocess.run", side_effect=self.gh_run(body=b"x" * (self.ns["MAX_DOWNLOAD"] + 1))) as run:
            self.assert_clean_failure("超过大小限制")
        run.assert_called_once()

    def test_gh_malformed_response_is_rejected(self):
        self.load(token=True, gh=True)
        with patch("subprocess.run", side_effect=self.gh_run(header=False)) as run:
            self.assert_clean_failure("缺少有效状态")
        run.assert_called_once()

    def test_gh_rejects_unofficial_initial_endpoint(self):
        self.load(token=True, gh=True)
        for url in ("http://api.github.com/repos/owner/project", "https://other.example/repos/owner/project",
                    "https://api.github.com@other.example/repos/owner/project"):
            with self.subTest(url=url), patch("subprocess.run") as run:
                with self.assertRaisesRegex(SystemExit, "非GitHub官方API"):
                    self.download(url, ACCEPT)
                run.assert_not_called()


@unittest.skipIf(os.geteuid() == 0, "The real entry deliberately refuses root")
class EntryIntegrationChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="neh-download-entry-")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.capture = self.root / "captured.json"
        self.log = self.root / "requests.jsonl"
        self.token = self.root / "token"
        self.token.write_text(SECRET)
        self.token.chmod(0o600)
        capture_source = ("import json, os, sys\nfrom pathlib import Path\n"
                          "Path(os.environ['NEH_TEST_CAPTURE']).write_text(json.dumps(sys.argv[1:]))\n")
        self.environment = dict(os.environ, PATH=str(self.root) + os.pathsep + os.environ["PATH"],
                                NEH_TEST_CAPTURE=str(self.capture), NEH_TEST_LOG=str(self.log))
        (self.root / "python3").symlink_to(sys.executable)
        gh = self.root / "gh"
        gh.write_text("#!" + sys.executable + "\n" +
                      "import json, os, sys\nfrom pathlib import Path\n"
                      "args = sys.argv[1:]\n"
                      "if args[:2] == ['auth', 'status']: sys.exit(0)\n"
                      "if args[:2] == ['auth', 'token']:\n    print(" + repr(SECRET) + "); sys.exit(0)\n"
                      "if 'api' not in args: sys.exit(2)\n"
                      "if '--include' not in args:\n    sys.stdout.write(" + repr(ENTRY) + "); sys.exit(0)\n"
                      "assert os.environ['GH_TOKEN'] == " + repr(SECRET) + "\n"
                      "assert os.environ['GH_HOST'] == 'github.com'\n"
                      "with open(os.environ['NEH_TEST_LOG'], 'a') as log: log.write(json.dumps(args) + '\\n')\n"
                      "url = args[-1]\n"
                      "if '/commits/' in url: body = json.dumps({'sha': 'a' * 40})\n"
                      "elif '/contents/scripts/neh_bootstrap.py?' in url: body = " + repr(capture_source) + "\n"
                      "elif '/contents/scripts/neh_source.py?' in url: body = '# fixture source\\n'\n"
                      "else: sys.exit(3)\n"
                      "sys.stdout.write('HTTP/2.0 200 OK\\nContent-Type: application/octet-stream\\r\\n\\r\\n' + body)\n")
        gh.chmod(0o755)

    def run_entry(self, prefix, arguments):
        completed = subprocess.run(prefix + arguments, cwd=str(ROOT), env=self.environment,
                                   capture_output=True, text=True, timeout=15)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertNotIn(SECRET, completed.stdout + completed.stderr)
        captured = json.loads(self.capture.read_text())
        self.assertEqual(captured[-len(arguments) - 2:], arguments + ["--revision", "a" * 40])
        requests = [json.loads(line) for line in self.log.read_text().splitlines()]
        self.assertEqual(len(requests), 3)
        self.assertIn("/commits/main", requests[0][-1])
        for request in requests[1:]:
            self.assertTrue(request[-1].endswith("?ref=" + "a" * 40))
        self.assertNotIn(SECRET, self.log.read_text())
        return captured

    def test_public_script_token_path_reuses_gh_and_forwards_arguments(self):
        self.run_entry(["bash", "get-neh.sh", "--token-file", str(self.token)],
                       ["--install-dir", str(self.root / "NEH 安装 空间"), "--cli"])

    def test_private_script_uses_existing_gh_and_token_through_real_bootstrap(self):
        captured = self.run_entry(["bash", "scripts/neh_private_install.sh"],
                                  ["--install-dir", str(self.root / "NEH 安装 空间")])
        self.assertEqual(captured[0], "--token-file")
        self.assertFalse(Path(captured[1]).exists(), "Private entry must remove its temporary token")


if __name__ == "__main__":
    unittest.main()
