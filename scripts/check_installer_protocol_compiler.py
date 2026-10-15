#!/usr/bin/env python3
"""协议编译器离线契约：模拟官方传输和版本查询，禁止真实进程与网络。"""
from __future__ import annotations

import hashlib
import io
import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
import zipfile
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.module_installer import builds


class ProtocolCompilerChecks(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.root = Path(self.stack.enter_context(tempfile.TemporaryDirectory(prefix="neh-protoc-check-")))
        self.cache = self.root / "cache"
        self.destination = self.cache / "tools/protoc-25.3-x86_64"
        self.archive = self.cache / "downloads/protoc-25.3-linux-x86_64.zip"
        self.url = "https://github.com/protocolbuffers/protobuf/releases/download/v25.3/" + self.archive.name
        self.payload = self.zip_bytes()
        self.version = "libprotoc 25.3\n"
        self.final_url = "https://release-assets.githubusercontent.com/fixture/protoc.zip"
        self.stack.enter_context(patch.dict(os.environ, {"PROTOC": ""}))
        self.stack.enter_context(patch.object(builds.platform, "system", return_value="Linux"))
        self.machine = self.stack.enter_context(patch.object(builds.platform, "machine", return_value="x86_64"))
        self.which = self.stack.enter_context(patch.object(builds.shutil, "which", return_value=None))
        self.stack.enter_context(patch.object(subprocess, "run", side_effect=AssertionError("real command forbidden")))
        self.stack.enter_context(patch.object(subprocess, "Popen", side_effect=AssertionError("real process forbidden")))
        self.probes = self.stack.enter_context(patch.object(subprocess, "check_output", side_effect=self.probe))
        self.downloads = self.stack.enter_context(patch.object(builds.urllib.request, "urlopen", side_effect=self.response))
        self.stack.enter_context(patch.object(builds.time, "sleep"))
        for name in ("sha256", "sha1", "md5", "blake2b", "blake2s", "new"):
            self.stack.enter_context(patch.object(hashlib, name, side_effect=AssertionError("application hash forbidden")))

    def zip_bytes(self, extra=(), *, compiler=b"\x7fELFfixture protoc", headers=True):
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w") as archive:
            archive.writestr("bin/protoc", compiler)
            if headers:
                archive.writestr("include/google/protobuf/descriptor.proto", "fixture descriptor")
                archive.writestr("include/google/protobuf/empty.proto", "fixture empty")
            for name, content in extra:
                archive.writestr(name, content)
        return buffer.getvalue()

    def response(self, url, *, timeout):
        self.assertIn(url, (self.url, self.url.replace("x86_64", "aarch_64")))
        self.assertEqual(timeout, 60)
        value = io.BytesIO(self.payload)
        value.geturl = lambda: self.final_url
        return value

    def probe(self, argv, **kwargs):
        self.assertEqual(argv[1:], ["--version"])
        self.assertEqual(kwargs, {"text": True, "timeout": 30})
        return self.version

    def install(self):
        return builds._protocol_compiler(self.cache)

    def expected(self):
        return {"PROTOC": str(self.destination / "bin/protoc"), "PROTOC_INCLUDE": str(self.destination / "include")}

    def test_pinned_official_download_produces_executable_and_complete_receipt(self):
        self.assertEqual(self.install(), self.expected())
        self.downloads.assert_called_once_with(self.url, timeout=60)
        compiler = self.destination / "bin/protoc"
        self.assertTrue(os.access(str(compiler), os.X_OK))
        self.assertEqual(compiler.read_bytes(), b"\x7fELFfixture protoc")
        receipt = json.loads((self.destination / "neh-protoc-install.json").read_text())
        self.assertEqual(receipt["source"], self.url)
        self.assertEqual(set(receipt["files"]), {"bin/protoc", "include/google/protobuf/descriptor.proto", "include/google/protobuf/empty.proto"})
        self.assertEqual(list((self.cache / "tools").glob(".protoc-*")), [])

    def test_warm_cache_never_reads_or_unpacks_archive_or_file_contents(self):
        self.install()
        self.probes.reset_mock()
        with patch.object(builds.zipfile, "ZipFile", side_effect=AssertionError("warm ZIP reopened")), patch.object(
            Path, "read_bytes", side_effect=AssertionError("warm content read")
        ), patch.object(builds.shutil, "copyfileobj", side_effect=AssertionError("warm copy")):
            self.assertEqual(self.install(), self.expected())
        self.assertEqual(self.downloads.call_count, 1)
        self.probes.assert_called_once_with([str(self.destination / "bin/protoc"), "--version"], text=True, timeout=30)

    def test_completed_cache_survives_download_cache_removal(self):
        self.install()
        shutil.rmtree(self.archive.parent)
        self.assertEqual(self.install(), self.expected())
        self.assertEqual(self.downloads.call_count, 1)

    def test_system_supported_compiler_is_reused_without_cache_writes(self):
        for version in ("libprotoc 3.15.0", "libprotoc 3.21.12", "libprotoc 25.3"):
            with self.subTest(version=version):
                self.version = version
                self.which.return_value = "/usr/bin/protoc"
                self.assertEqual(self.install(), {"PROTOC": str(Path("/usr/bin/protoc").resolve())})
        self.assertFalse(self.cache.exists())
        self.downloads.assert_not_called()

    def test_explicit_supported_compiler_path_is_retained(self):
        configured = self.root / "chosen compiler"
        with patch.dict(os.environ, {"PROTOC": str(configured)}):
            self.assertEqual(self.install(), {"PROTOC": str(configured)})
        self.probes.assert_called_once_with([str(configured), "--version"], text=True, timeout=30)
        self.downloads.assert_not_called()

    def test_explicit_invalid_or_old_compiler_fails_without_network(self):
        for version in ("libprotoc 3.14.0", "not protoc", "libprotoc 3"):
            with self.subTest(version=version), patch.dict(os.environ, {"PROTOC": "/fixture/protoc"}):
                self.version = version
                with self.assertRaisesRegex(RuntimeError, "指定的PROTOC"):
                    self.install()
        self.downloads.assert_not_called()
        self.assertFalse(self.cache.exists())

    def test_explicit_compiler_probe_failure_does_not_silently_replace_choice(self):
        with patch.dict(os.environ, {"PROTOC": "/fixture/protoc"}):
            self.probes.side_effect = OSError("fixture unavailable")
            with self.assertRaisesRegex(RuntimeError, "指定的PROTOC"):
                self.install()
        self.downloads.assert_not_called()

    def test_old_path_compiler_falls_back_to_pinned_official_program(self):
        self.which.return_value = "/fixture/old-protoc"
        self.probes.side_effect = ["libprotoc 3.6.1", "libprotoc 25.3"]
        self.assertEqual(self.install(), self.expected())
        self.assertEqual(self.downloads.call_count, 1)

    def test_arm64_archive_alias_and_matching_destination(self):
        self.machine.return_value = "arm64"
        result = self.install()
        self.assertEqual(result["PROTOC"], str(self.cache / "tools/protoc-25.3-aarch64/bin/protoc"))
        self.downloads.assert_called_once_with(self.url.replace("x86_64", "aarch_64"), timeout=60)

    def test_unsupported_architecture_and_os_fail_before_download(self):
        for system, machine in (("Linux", "riscv64"), ("Darwin", "arm64")):
            with self.subTest(system=system), patch.object(builds.platform, "system", return_value=system):
                self.machine.return_value = machine
                with self.assertRaisesRegex(RuntimeError, "当前架构"):
                    self.install()
        self.downloads.assert_not_called()

    def test_unreceipted_existing_archive_is_replaced_before_execution(self):
        self.archive.parent.mkdir(parents=True)
        self.archive.write_bytes(self.zip_bytes(compiler=b"\x7fELFuntrusted fixture"))
        self.install()
        self.assertEqual(self.downloads.call_count, 1)
        self.assertEqual((self.destination / "bin/protoc").read_bytes(), b"\x7fELFfixture protoc")

    def test_modified_program_is_repaired_without_probing_modified_bytes(self):
        self.install()
        compiler = self.destination / "bin/protoc"
        compiler.write_bytes(b"\x7fELFuntrusted changed fixture")
        self.probes.reset_mock()
        self.install()
        self.assertEqual(compiler.read_bytes(), b"\x7fELFfixture protoc")
        self.assertEqual(self.downloads.call_count, 1)
        self.assertNotEqual(self.probes.call_args[0][0][0], str(compiler))

    def test_missing_or_modified_include_is_repaired_from_admitted_zip(self):
        for operation in ("remove", "modify"):
            with self.subTest(operation=operation):
                self.install()
                header = self.destination / "include/google/protobuf/empty.proto"
                header.unlink() if operation == "remove" else header.write_text("changed")
                self.install()
                self.assertEqual(header.read_text(), "fixture empty")
        self.assertEqual(self.downloads.call_count, 1)

    def test_interrupted_completion_record_retries_offline_without_partial_publication(self):
        original = builds.atomic_write
        def write(path, *args, **kwargs):
            if path.name == "neh-protoc-install.json":
                raise OSError("fixture interrupted extraction")
            return original(path, *args, **kwargs)
        with patch.object(builds, "atomic_write", side_effect=write), self.assertRaisesRegex(OSError, "interrupted"):
            self.install()
        self.assertFalse(self.destination.exists())
        self.assertEqual(list((self.cache / "tools").glob(".protoc-*")), [])
        self.downloads.side_effect = AssertionError("offline retry attempted download")
        self.assertEqual(self.install(), self.expected())

    def test_failed_publication_restores_previous_installation(self):
        self.install()
        marker = self.destination / "neh-protoc-install.json"
        marker.write_text("{}")
        original = builds._publish_source_directory
        def publish(source, destination):
            if source.name.startswith(".protoc-") and not source.name.startswith(".protoc-previous-"):
                raise OSError("fixture publish failure")
            return original(source, destination)
        with patch.object(builds, "_publish_source_directory", side_effect=publish), self.assertRaisesRegex(OSError, "publish failure"):
            self.install()
        self.assertEqual(marker.read_text(), "{}")
        self.assertEqual((self.destination / "bin/protoc").read_bytes(), b"\x7fELFfixture protoc")
        self.assertEqual(list((self.cache / "tools").glob(".protoc-*")), [])
        self.assertEqual(self.install(), self.expected())
        self.assertEqual(self.downloads.call_count, 1)

    def test_interrupted_old_style_directory_is_rebuilt_without_executing_it(self):
        program = self.destination / "bin/protoc"
        program.parent.mkdir(parents=True)
        program.write_bytes(b"\x7fELFold incomplete fixture")
        program.chmod(0o755)
        self.install()
        self.assertNotEqual(self.probes.call_args[0][0][0], str(program))
        self.assertEqual(program.read_bytes(), b"\x7fELFfixture protoc")

    def test_changed_download_is_not_readmitted_by_filename(self):
        self.install()
        shutil.rmtree(self.destination)
        self.archive.write_bytes(self.zip_bytes(compiler=b"\x7fELFchanged archive fixture"))
        self.install()
        self.assertEqual(self.downloads.call_count, 2)
        self.assertEqual((self.destination / "bin/protoc").read_bytes(), b"\x7fELFfixture protoc")

    def test_wrong_official_version_never_publishes_or_retains_admission(self):
        self.version = "libprotoc 24.0"
        with self.assertRaisesRegex(RuntimeError, "版本不符"):
            self.install()
        self.assertFalse(self.destination.exists())
        self.assertFalse(self.archive.with_suffix(".download.json").exists())

    def test_missing_headers_and_non_elf_program_never_execute(self):
        for payload in (self.zip_bytes(headers=False), self.zip_bytes(compiler=b"#!/bin/sh\nexit 0")):
            with self.subTest(payload=payload[:20]):
                self.payload = payload
                with self.assertRaises(RuntimeError):
                    self.install()
        self.probes.assert_not_called()
        self.assertFalse(self.destination.exists())

    def test_archive_traversal_and_absolute_paths_are_rejected_before_execution(self):
        for name in ("include/../../outside", "/tmp/escape", "include\\escape", "../ignored", "include//ambiguous.proto"):
            with self.subTest(name=name):
                self.payload = self.zip_bytes(((name, "unsafe"),))
                with self.assertRaisesRegex(RuntimeError, "越界路径"):
                    self.install()
        self.probes.assert_not_called()
        self.assertFalse((self.cache / "outside").exists())
        self.assertFalse(self.destination.exists())

    def test_archive_symlinks_and_duplicate_members_are_rejected(self):
        link = zipfile.ZipInfo("include/linked.proto")
        link.create_system = 3
        link.external_attr = (stat.S_IFLNK | 0o777) << 16
        self.payload = self.zip_bytes(((link, "../../outside"),))
        with self.assertRaisesRegex(RuntimeError, "链接"):
            self.install()
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            self.payload = self.zip_bytes((("bin/protoc", b"\x7fELFduplicate"),))
        with self.assertRaisesRegex(RuntimeError, "重复文件"):
            self.install()
        self.probes.assert_not_called()

    def test_linked_destination_and_completed_cache_parent_are_rejected(self):
        outside = self.root / "outside"
        outside.mkdir()
        self.destination.parent.mkdir(parents=True)
        self.destination.symlink_to(outside, target_is_directory=True)
        with self.assertRaises(RuntimeError):
            self.install()
        self.downloads.assert_not_called()
        self.destination.unlink()
        self.install()
        include = self.destination / "include"
        shutil.rmtree(include)
        include.symlink_to(outside, target_is_directory=True)
        self.probes.reset_mock()
        with self.assertRaisesRegex(RuntimeError, "链接"):
            self.install()
        self.probes.assert_not_called()
        self.assertEqual(list(outside.iterdir()), [])

    def test_linked_completion_and_download_records_are_rejected(self):
        self.install()
        marker = self.destination / "neh-protoc-install.json"
        other = self.root / "external-record"
        other.write_text("keep")
        marker.unlink()
        marker.symlink_to(other)
        with self.assertRaisesRegex(RuntimeError, "记录不接受链接"):
            self.install()
        shutil.rmtree(self.destination)
        record = self.archive.with_suffix(".download.json")
        record.unlink()
        record.symlink_to(other)
        with self.assertRaisesRegex(RuntimeError, "记录不接受链接"):
            self.install()
        self.assertEqual(other.read_text(), "keep")

    def test_untrusted_or_unencrypted_redirect_is_never_published(self):
        for url in ("http://release-assets.githubusercontent.com/file", "https://untrusted.invalid/file", "https://github.com.untrusted.invalid/file"):
            with self.subTest(url=url):
                self.final_url = url
                with self.assertRaises(RuntimeError):
                    self.install()
                self.assertFalse(self.archive.exists())
        self.probes.assert_not_called()
        self.assertEqual(list(self.archive.parent.glob(".download-*")), [])

    def test_download_retries_failure_without_partial_receipt_or_program(self):
        self.downloads.side_effect = OSError("fixture network outage")
        with self.assertRaisesRegex(OSError, "outage"):
            self.install()
        self.assertEqual(self.downloads.call_count, 3)
        self.assertFalse(self.archive.exists())
        self.assertFalse(self.archive.with_suffix(".download.json").exists())
        self.assertFalse(self.destination.exists())
        self.assertEqual(list(self.archive.parent.glob(".download-*")), [])


if __name__ == "__main__":
    unittest.main()
