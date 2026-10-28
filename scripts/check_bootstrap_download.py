#!/usr/bin/env python3
"""离线检查公开安装入口的下载重试、证书验证和日志脱敏。"""
import http.client
import io
import os
from pathlib import Path
import ssl
import subprocess
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
        self.stderr = io.StringIO()
        self.addCleanup(patch.stopall)
        self.sleep = patch("time.sleep").start()
        patch("sys.stderr", self.stderr).start()
        self.load()

    def load(self):
        values = dict(NEH_BOOTSTRAP_REPOSITORY="https://github.com/owner/project.git", NEH_BOOTSTRAP_BRANCH="main",
                      NEH_BOOTSTRAP_REVISION="", NEH_BOOTSTRAP_STAGE=str(self.root))
        self.ns = {}
        with patch.dict(os.environ, values), patch("subprocess.run", return_value=subprocess.CompletedProcess([], 0)):
            exec(compile(DEFINITIONS, "get-neh.sh:embedded", "exec"), self.ns)
        self.opener = Mock()
        self.ns["opener"] = self.opener
        self.download = self.ns["download"]

    def assert_clean_failure(self, reason):
        with self.assertRaises(SystemExit) as caught:
            self.download(URL, ACCEPT)
        diagnostic = str(caught.exception) + self.stderr.getvalue()
        self.assertIn(reason, diagnostic)
        self.assertNotIn(SECRET, diagnostic)
        self.assertNotIn("Traceback", diagnostic)
        return diagnostic

    def test_anonymous_download_omits_authorization(self):
        self.opener.open.return_value = Response(b"public")
        self.assertEqual(self.download(URL, ACCEPT), b"public")
        request = self.opener.open.call_args.args[0]
        self.assertIsNone(request.get_header("Authorization"))

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



if __name__ == "__main__":
    unittest.main()
