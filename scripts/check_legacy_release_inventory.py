#!/usr/bin/env python3
"""Isolated legacy installer manifest migration; no host install or service operations."""

import copy
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from scripts.deploy_installer import operations as op  # noqa: E402
from scripts.deploy_installer.model import DeploymentOptions  # noqa: E402
from scripts.deploy_installer.runner import RecordingRunner  # noqa: E402


class ReleaseInventoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="release-inventory-")
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.release = self.root / "release"
        self.make_release(self.release)

    def make_release(self, r):
        for name in (
            "app.py",
            ".venv/bin/python",
            ".venv/lib/pkg.py",
            "vendor/sdk/mod.py",
            "models/model.bin",
            "bin/nav",
            "scripts/install_motion_guard_deployment.sh",
        ):
            p = r / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(b"original")
        (r / ".venv/bin/python-link").symlink_to("python")

    def manifest(self, r=None, schema=3):
        r = r or self.release
        result = dict(
            schema_version=schema,
            profile="fixture",
            release_id=r.name,
            fingerprint="owner-unchanged",
            system_package_versions={},
        )
        if schema == 3:
            result.update(
                binary_paths={"nav": "bin/nav"},
                source_files=op._release_source_files(r),
                venv_inventory=op._venv_inventory(r),
                vendor_files={"sdk": op._release_vendor_files(r / "vendor/sdk")},
                model_files=op._release_model_files(r),
            )
        else:
            # Synthetic historical-shape strings, deliberately opaque to this reader.
            # They do not authenticate the fixture bytes and require no old Git object.
            result.update(
                binary_sha256={"nav": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
                source_tree_sha256="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
                venv_inventory_sha256="cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
                vendor_tree_sha256={"sdk": "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"},
                model_sha256={"model.bin": "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"},
            )
        return result

    def no_content_reads(self):
        stack = ExitStack()
        for name in ("_sha256", "_tree_digest"):
            stack.enter_context(patch.object(op, name, side_effect=AssertionError("local digest invoked")))
        stack.enter_context(patch.object(Path, "open", side_effect=AssertionError("local file body read")))
        return stack

    def test_schema2_read_does_not_rewrite_or_invent_history(self):
        with self.no_content_reads(), patch.object(
            subprocess, "check_output", side_effect=AssertionError("fixture invoked Git")
        ), patch.object(subprocess, "run", side_effect=AssertionError("fixture invoked a process")):
            m = self.manifest(schema=2)
        p = self.release / "release-manifest.json"
        p.write_text(json.dumps(m))
        before = p.read_bytes()
        original = copy.deepcopy(m)
        with self.no_content_reads():
            op._validate_completed_release(self.release, m)
        self.assertEqual(m, original)
        self.assertEqual(p.read_bytes(), before)
        self.assertNotIn("venv_inventory", m)

    def test_byte_drift_accepted_without_claiming_local_integrity(self):
        for schema in (2, 3):
            m = self.manifest(schema=schema)
            for relative in ("app.py", ".venv/lib/pkg.py", "vendor/sdk/mod.py", "models/model.bin", "bin/nav"):
                p = self.release / relative
                p.write_bytes(p.read_bytes() + b" changed length and bytes")
            with self.no_content_reads():
                op._validate_completed_release(self.release, m)

    def test_schema3_generation_omits_local_hashes_and_body_scans(self):
        source = self.root / "source"
        source.mkdir()
        (source / "Cargo.lock").write_text("lock")
        (source / "requirements.txt").write_text("pin")
        options = SimpleNamespace(
            release_dir=self.release,
            releases_dir=self.root,
            source_root=source,
            build_dir=self.root / "build",
            no_systemd=True,
            release_id="release",
            source_tree_id="input-owner",
            rust_target_dir=lambda _: self.root / "target",
        )
        profile = SimpleNamespace(
            name="fixture",
            robot="custom",
            rust=SimpleNamespace(
                builds=[SimpleNamespace(manifest="Cargo.toml", binary="nav", output="bin/nav")], toolchain="existing"
            ),
            python=SimpleNamespace(requirements=["requirements.txt"], constraints=None, source_artifact_manifest=None),
        )
        c = SimpleNamespace(
            options=options,
            profile=profile,
            notes={"artifact_sha256:approved-input": "trust-pin"},
            plan=SimpleNamespace(fingerprint="owner-unchanged"),
            runner=RecordingRunner(),
        )
        with ExitStack() as stack:
            for name in (
                "_prepare_backup_root",
                "_copy_release_source",
                "_copy_rust_build_output",
                "_install_geometry_release",
                "_record_recovery",
            ):
                stack.enter_context(patch.object(op, name))
            stack.enter_context(patch.object(op, "_venv_python", return_value=self.release / ".venv/bin/python"))
            stack.enter_context(patch.object(op, "_source_revision", return_value="fixture-revision"))
            stack.enter_context(
                patch.object(op, "resolve_build_account", return_value=SimpleNamespace(pw_uid=os.getuid()))
            )
            stack.enter_context(patch.object(op, "_validate_release_binary", return_value="fixture architecture"))
            stack.enter_context(patch.object(op, "_sha256", side_effect=AssertionError("local digest invoked")))
            stack.enter_context(
                patch.object(op, "_tree_digest", side_effect=AssertionError("local tree digest invoked"))
            )
            stack.enter_context(patch.object(Path, "open", side_effect=AssertionError("local generation body read")))
            op.install_release(c)
        m = json.loads((self.release / "release-manifest.json").read_text())
        self.assertEqual(m["schema_version"], 3)
        self.assertEqual(m["fingerprint"], "owner-unchanged")
        self.assertEqual(m["cargo_lock_files"], ["Cargo.lock"])
        self.assertEqual(m["python_input_files"], ["requirements.txt"])
        self.assertEqual([k for k in m if k.endswith("_sha256")], ["third_party_sha256"])
        self.assertEqual(m["third_party_sha256"], {"approved-input": "trust-pin"})
        with self.no_content_reads():
            op._validate_completed_release(self.release, m)
        self.assertEqual(c.runner.executed, [])

    def test_schema3_missing_files_and_layout_rejected(self):
        for relative in (
            "app.py",
            ".venv/lib/pkg.py",
            "vendor/sdk/mod.py",
            "models/model.bin",
            "bin/nav",
            ".venv/bin/python",
        ):
            m = self.manifest()
            p = self.release / relative
            body = p.read_bytes()
            p.unlink()
            with self.assertRaises(op.DeploymentError):
                op._validate_completed_release(self.release, m)
            p.write_bytes(body)
        m = self.manifest()
        os.chmod(self.release / ".venv/lib/pkg.py", 0o700)
        with self.assertRaises(op.DeploymentError):
            op._validate_completed_release(self.release, m)

    def test_legacy_missing_required_files_rejected_without_stat_proof(self):
        m = self.manifest(schema=2)
        for relative in ("bin/nav", "models/model.bin", ".venv/bin/python"):
            p = self.release / relative
            body = p.read_bytes()
            p.unlink()
            with self.assertRaises(op.DeploymentError):
                op._validate_completed_release(self.release, m)
            p.write_bytes(body)

    def test_invalid_manifest_paths_and_schemas_rejected(self):
        for schema in (True, 1, 4, "3"):
            m = self.manifest()
            m["schema_version"] = schema
            with self.assertRaises(op.DeploymentError):
                op._validate_completed_release(self.release, m)
        for schema in (2, 3):
            for name in ("../nav", "/tmp/nav", "nested/nav", ".", ""):
                m = self.manifest(schema=schema)
                m["binary_sha256" if schema == 2 else "binary_paths"] = {name: "bin/nav"}
                with self.assertRaises(op.DeploymentError):
                    op._validate_completed_release(self.release, m)
            m = self.manifest(schema=schema)
            m["vendor_tree_sha256" if schema == 2 else "vendor_files"] = {"../escape": []}
            with self.assertRaises(op.DeploymentError):
                op._validate_completed_release(self.release, m)
        m = self.manifest()
        m["binary_paths"]["nav"] = "../elsewhere"
        with self.assertRaises(op.DeploymentError):
            op._validate_completed_release(self.release, m)

    def test_symlink_special_nodes_and_venv_links(self):
        m = self.manifest()
        op._validate_completed_release(self.release, m)
        for relative in ("app.py", "vendor/sdk/mod.py", "models/model.bin", "bin/nav"):
            p = self.release / relative
            body = p.read_bytes()
            p.unlink()
            p.symlink_to(self.release / ".venv/bin/python")
            with self.assertRaises(op.DeploymentError):
                op._validate_completed_release(self.release, m)
            p.unlink()
            p.write_bytes(body)
        for directory in (
            self.release,
            self.release / ".venv/lib",
        ):
            p = directory / "fifo"
            os.mkfifo(p)
            with self.assertRaises(op.DeploymentError):
                op._validate_completed_release(self.release, m)
            p.unlink()

    def test_vendor_legacy_spellings_and_unlisted_special_nodes(self):
        for schema in (2, 3):
            for spelling in ("sdk", "./sdk", "sdk/"):
                manifest = self.manifest(schema=schema)
                field = "vendor_tree_sha256" if schema == 2 else "vendor_files"
                manifest[field] = {spelling: manifest[field]["sdk"]}
                fifo = self.release / "vendor/sdk/unlisted-fifo"
                os.mkfifo(fifo)
                try:
                    with self.no_content_reads():
                        op._validate_completed_release(self.release, manifest)
                finally:
                    fifo.unlink()

    def test_legacy_models_ignore_unregistered_symlink_and_special_node(self):
        manifest = self.manifest(schema=2)
        (self.release / "models/unlisted-link").symlink_to(self.release / "app.py")
        os.mkfifo(self.release / "models/unlisted-fifo")
        with self.no_content_reads():
            op._validate_completed_release(self.release, manifest)

    def test_legacy_models_root_alias_remains_readable(self):
        manifest = self.manifest(schema=2)
        root = self.release / "models"
        target = self.root / "model-alias-target"
        root.rename(target)
        root.symlink_to(target)
        with self.no_content_reads():
            op._validate_completed_release(self.release, manifest)

    def test_geometry_binary_internal_ancestor_link_allowed_external_escape_rejected(self):
        target = self.release / "services/geometry/nuwax_geometry_runtime/target"
        generation = target / "nuwax-geometry-releases/g"
        generation.mkdir(parents=True)
        (generation / "nuwax-geometry-runtime").write_bytes(b"fixture")
        active = target / "release"
        active.symlink_to("nuwax-geometry-releases/g")
        m = self.manifest()
        m["binary_paths"]["nuwax-geometry-runtime"] = op._release_binary_path("nuwax-geometry-runtime")
        op._validate_completed_release(self.release, m)
        outside = self.root / "outside"
        outside.mkdir()
        (outside / "nuwax-geometry-runtime").write_bytes(b"fixture")
        active.unlink()
        active.symlink_to(outside)
        with self.assertRaisesRegex(op.DeploymentError, "escapes release layout"):
            op._validate_completed_release(self.release, m)

    def test_legacy_rollback_restores_config_then_health_and_stops_on_failure(self):
        options = replace(
            DeploymentOptions.for_tests(source_root=self.root / "options"), no_systemd=False, no_verify=False
        )
        candidate = options.release_dir
        previous = options.releases_dir / "previous"
        self.make_release(candidate)
        self.make_release(previous)
        (previous / "release-manifest.json").write_text(json.dumps(self.manifest(previous, 2)))
        options.current_link.symlink_to(candidate)
        config = options.config_dir / "test.env"
        config.parent.mkdir(parents=True)
        config.write_text("new")
        snapshot = self.root / "systemd"
        snapshot.mkdir()
        (snapshot / "snapshot.complete").write_text("fixture")
        calls = []

        class Runner:
            def run(self, argv, **kw):
                calls.append(argv[-1])

        c = SimpleNamespace(
            options=options,
            profile=SimpleNamespace(name="fixture", services=()),
            notes={},
            previous_release=str(previous),
            release_switched=True,
            config_backups={config: b"old"},
            runner=Runner(),
        )

        def health(_):
            self.assertEqual(options.current_link.resolve(), previous)
            self.assertEqual(config.read_text(), "old")
            calls.append("health")
            raise op.DeploymentError("fixture health failure")

        with patch.object(op, "_deployment_env", return_value={}), patch.object(
            op, "_systemd_backup_dir", return_value=snapshot
        ), patch.object(
            op, "_mark_recovery_rolling_back", side_effect=lambda _: calls.append("rolling_back")
        ), patch.object(op, "_secure_config_owner"), patch.object(op, "verify_deployment", side_effect=health):
            with self.assertRaisesRegex(op.DeploymentError, "fixture health failure"):
                op.rollback(c)
        self.assertEqual(calls, ["rolling_back", "stop", "restore-systemd", "health", "stop"])

    def test_approved_wheel_and_source_pins_still_reject_drift(self):
        for is_wheel in (True, False):
            base = self.root / ("wheel" if is_wheel else "source-pin")
            base.mkdir()
            item = base / "input.whl" if is_wheel else base / "sdk"
            if is_wheel:
                item.write_bytes(b"approved")
            else:
                item.mkdir()
                (item / "module.py").write_bytes(b"approved")
            expected = op._sha256(item) if is_wheel else op._tree_digest(item)
            entry = {
                "distribution": "clip",
                "source": {},
                "integrity": {"wheel_sha256" if is_wheel else "source_sha256": expected},
            }
            (base / "inputs.json").write_text(json.dumps({"schema_version": 1, "artifacts": [entry]}))
            c = SimpleNamespace(
                options=SimpleNamespace(source_root=base, allow_unverified_third_party=True),
                profile=SimpleNamespace(
                    production=False, python=SimpleNamespace(source_artifact_manifest="inputs.json")
                ),
                notes={"clip_artifact": item},
            )
            op._verify_source_artifacts(c)
            (item if is_wheel else item / "module.py").write_bytes(b"changed")
            with self.assertRaisesRegex(op.DeploymentError, "hash mismatch"):
                op._verify_source_artifacts(c)

    def test_yolo_download_pin_still_rejects_wrong_bytes(self):
        root = self.root / "model-cache"
        root.mkdir(mode=0o700)
        with patch.object(op.urllib.request, "urlopen", return_value=io.BytesIO(b"wrong-model")):
            with self.assertRaisesRegex(op.DeploymentError, "SHA-256 does not match"):
                op._materialize_verified_model(
                    "https://fixture.invalid/model", "0" * 64, root / "model.pt", os.getgid()
                )
        self.assertFalse((root / "model.pt").exists())

    def test_real_rsa_signature_signed_files_public_key_pin_and_profile(self):
        private = self.root / "key.pem"
        public = self.root / "key.pub"
        subprocess.run(
            ["openssl", "genpkey", "-algorithm", "RSA", "-pkeyopt", "rsa_keygen_bits:2048", "-out", str(private)],
            check=True,
            capture_output=True,
        )
        subprocess.run(
            ["openssl", "pkey", "-in", str(private), "-pubout", "-out", str(public)], check=True, capture_output=True
        )
        bundle = self.root / "bundle"
        bundle.mkdir()
        for name in op._REQUIRED_BUNDLE_DIRS:
            (bundle / name).mkdir()
        for name in ("cargo", "rustc", "rustup"):
            p = bundle / "cargo-bin" / name
            p.write_bytes(b"fixture")
            p.chmod(0o755)
        files = {
            p.relative_to(bundle).as_posix(): {"size": p.stat().st_size, "sha256": op._sha256(p)}
            for p in (bundle / "cargo-bin").iterdir()
        }
        manifest = {"schema_version": 1, "signature_algorithm": "rsa-sha256", "metadata": {}, "files": files}
        p = bundle / "bundle-manifest.json"
        p.write_text(json.dumps(manifest))
        signature = bundle / "bundle-manifest.sig"
        subprocess.run(
            ["openssl", "dgst", "-sha256", "-sign", str(private), "-out", str(signature), str(p)],
            check=True,
            capture_output=True,
        )
        op.verify_offline_bundle(bundle, public)
        (bundle / "cargo-bin/cargo").write_bytes(b"CHANGED")
        with self.assertRaisesRegex(op.DeploymentError, "checksum mismatch"):
            op.verify_offline_bundle(bundle, public)
        (bundle / "cargo-bin/cargo").write_bytes(b"fixture")
        p.write_text(p.read_text() + " ")
        with self.assertRaisesRegex(op.DeploymentError, "signature verification failed"):
            op.verify_offline_bundle(bundle, public)
        profile = SimpleNamespace(production=False, metadata={"bundle_public_key_sha256": "0" * 64})
        with self.assertRaisesRegex(op.DeploymentError, "public key does not match"):
            op.validate_bundle_public_key(profile, SimpleNamespace(allow_unverified_third_party=True), public)
        profile.name = "fixture"
        with patch.object(op, "profile_sha256", return_value="trusted-profile"):
            with self.assertRaisesRegex(op.DeploymentError, "profile digest"):
                op.validate_offline_bundle_contract(
                    profile, SimpleNamespace(), {"metadata": {"profile": "fixture", "profile_sha256": "different"}}
                )


if __name__ == "__main__":
    unittest.main(verbosity=2)
