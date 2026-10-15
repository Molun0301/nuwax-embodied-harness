#!/usr/bin/env python3
"""Offline preparation regressions using tiny files and fake runtime loaders.

No real models, tensors, network requests, downloads or build jobs are used.
The fake loaders exercise the preparation contract, including sharded layouts;
they do not claim to test the third-party model libraries themselves.
"""
from __future__ import annotations

# Standalone repository checks add the source root before project imports.
# ruff: noqa: E402

import hashlib
import io
import json
import os
import socket
import sys
import tempfile
import unittest
from contextlib import ExitStack, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts import prepare_memory_models as models
from scripts.module_installer import manager

TEXT = ("modules.json",)
IMAGE = ("config.json", "preprocessor_config.json")


def forbidden(*args, **kwargs):
    raise AssertionError("External access, content hashing or unexpected model load")


class ModelPreparationChecks(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.base = Path(self.stack.enter_context(tempfile.TemporaryDirectory(prefix="neh-model-ready-")))
        self.output = self.stack.enter_context(redirect_stdout(io.StringIO()))
        self.stack.enter_context(patch.object(socket, "create_connection", side_effect=forbidden))
        for name in ("sha256", "sha1", "md5", "new"):
            self.stack.enter_context(patch.object(hashlib, name, side_effect=forbidden))
        self.text_loader = Mock(side_effect=self.load_text)
        self.image_loader = Mock(side_effect=self.load_weights)
        self.processor_loader = Mock(side_effect=self.load_processor)
        self.stack.enter_context(patch.dict(sys.modules, {
            "sentence_transformers": SimpleNamespace(SentenceTransformer=self.text_loader),
            "transformers": SimpleNamespace(
                CLIPModel=SimpleNamespace(from_pretrained=self.image_loader),
                CLIPProcessor=SimpleNamespace(from_pretrained=self.processor_loader),
            ),
        }))
        self.download = self.stack.enter_context(patch.object(models, "_download_snapshot", side_effect=forbidden))

    def write(self, target, name, contents):
        path = target / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")

    def fixture(self, target=None, required=TEXT, *, sharded=False, nested=False):
        target = target or self.base / "model"
        for name in required:
            self.write(target, name, "{}")
        weights = target / "0_Transformer" if nested else target
        if required == TEXT:
            self.write(target, "modules.json", json.dumps([{"path": "0_Transformer" if nested else "", "type": "Transformer"}]))
        self.write(weights, "config.json", "{}")
        self.write(weights, "tokenizer.json", "{}")
        if sharded:
            index = {"weight_map": {"one": "model-00001-of-00002.safetensors", "two": "model-00002-of-00002.safetensors"}}
            self.write(weights, "model.safetensors.index.json", json.dumps(index))
            for name in index["weight_map"].values():
                self.write(weights, name, "valid miniature weight")
        else:
            self.write(weights, "model.safetensors", "valid miniature weight")
        return target

    def check_offline(self, kwargs):
        self.assertIs(kwargs.get("local_files_only"), True)
        self.assertEqual(os.environ.get("HF_HUB_OFFLINE"), "1")
        self.assertEqual(os.environ.get("TRANSFORMERS_OFFLINE"), "1")

    def load_weights(self, directory, **kwargs):
        self.check_offline(kwargs)
        directory = Path(directory)
        self.assertTrue((directory / "config.json").is_file())
        index = directory / "model.safetensors.index.json"
        names = set(json.loads(index.read_text())["weight_map"].values()) if index.is_file() else {"model.safetensors"}
        for name in names:
            if (directory / name).read_text() != "valid miniature weight":
                raise ValueError("incomplete miniature weights")
        return object()

    def load_processor(self, directory, **kwargs):
        self.check_offline(kwargs)
        for name in ("preprocessor_config.json", "tokenizer.json"):
            json.loads((Path(directory) / name).read_text())
        return object()

    def load_text(self, directory, **kwargs):
        self.check_offline(kwargs)
        self.assertEqual(kwargs.get("device"), "cpu")
        self.assertIs(kwargs.get("trust_remote_code"), False)
        directory = Path(directory)
        modules = json.loads((directory / "modules.json").read_text())
        if not isinstance(modules, list) or not modules:
            raise ValueError("invalid modules")
        for module in modules:
            target = directory / module["path"]
            json.loads((target / "tokenizer.json").read_text())
            self.load_weights(str(target), local_files_only=True)
        return object()

    def prepare(self, target, required=TEXT, **kwargs):
        return models._prepare_one_model(label="fixture", repo_id="fixture/model", target_dir=target,
            required_files=required, token="", timeout_sec=10, force_download=kwargs.pop("force_download", False), **kwargs)

    def receipt(self, target):
        return target / models.COMPLETION_FILE

    def test_configs_only_never_claim_ready(self):
        for required in (TEXT, IMAGE):
            target = self.base / str(len(required))
            for name in required:
                self.write(target, name, "{}")
            self.assertFalse(models._looks_prepared(target, required))
            with self.assertRaises(SystemExit):
                self.prepare(target, required, validate_only=True)
            self.assertFalse(self.receipt(target).exists())
        self.download.assert_not_called()

    def test_incomplete_download_resumes_then_can_be_reused(self):
        target = self.base / "partial"
        self.write(target, "modules.json", "{}")
        self.write(target, ".cache/huggingface/download/weights.incomplete", "partial download")
        def complete(**kwargs):
            self.assertFalse(kwargs["force_download"])
            self.assertEqual((target / ".cache/huggingface/download/weights.incomplete").read_text(), "partial download")
            self.fixture(target)
            return target
        self.download.side_effect = complete
        self.prepare(target)
        self.assertTrue(models._looks_prepared(target, TEXT))
        self.download.assert_called_once()
        with patch.object(models, "_validate_local_model", side_effect=forbidden):
            self.prepare(target)
        self.download.assert_called_once()

    def test_download_returning_only_configs_is_not_completed(self):
        target = self.base / "false-success"
        def incomplete(**kwargs):
            self.write(target, "modules.json", "{}")
            return target
        self.download.side_effect = incomplete
        with self.assertRaises(SystemExit):
            self.prepare(target)
        self.assertFalse(self.receipt(target).exists())
        self.assertFalse(models._looks_prepared(target, TEXT))

    def test_interrupted_download_preserves_resume_files(self):
        target = self.base / "interrupted"
        def interrupted(**kwargs):
            self.write(target, "modules.json", "{}")
            self.write(target, ".cache/huggingface/download/file.incomplete", "resume me")
            raise OSError("synthetic disconnect")
        self.download.side_effect = interrupted
        with self.assertRaises(OSError):
            self.prepare(target)
        self.assertEqual((target / ".cache/huggingface/download/file.incomplete").read_text(), "resume me")
        self.assertFalse(self.receipt(target).exists())

    def test_valid_legacy_models_are_adopted_without_downloading(self):
        for required in (TEXT, IMAGE):
            target = self.fixture(self.base / str(len(required)), required)
            self.prepare(target, required)
            self.assertTrue(models._looks_prepared(target, required))
        self.download.assert_not_called()

    def test_sharded_nested_sentence_transformer_and_clip(self):
        for required in (TEXT, IMAGE):
            target = self.fixture(self.base / str(len(required)), required, sharded=True, nested=(required == TEXT))
            self.prepare(target, required, validate_only=True)
            self.assertTrue(models._looks_prepared(target, required))
            weight = next(target.rglob("model-00002-of-00002.safetensors"))
            weight.unlink()
            self.assertFalse(models._looks_prepared(target, required))
            with self.assertRaises(SystemExit):
                self.prepare(target, required, validate_only=True)
        self.download.assert_not_called()

    def test_weight_truncation_invalidates_completion(self):
        target = self.fixture()
        self.prepare(target)
        (target / "model.safetensors").write_text("")
        self.assertFalse(models._looks_prepared(target, TEXT))
        with self.assertRaises(SystemExit):
            self.prepare(target, validate_only=True)

    def test_tokenizer_removal_invalidates_completion(self):
        target = self.fixture(required=IMAGE)
        self.prepare(target, IMAGE)
        (target / "tokenizer.json").unlink()
        self.assertFalse(models._looks_prepared(target, IMAGE))
        with self.assertRaises(SystemExit):
            self.prepare(target, IMAGE, validate_only=True)

    def test_new_weight_file_invalidates_old_inventory(self):
        target = self.fixture()
        self.prepare(target)
        self.write(target, "pytorch_model.bin", "alternative checkpoint")
        self.assertFalse(models._looks_prepared(target, TEXT))
        self.prepare(target)
        self.assertTrue(models._looks_prepared(target, TEXT))
        self.download.assert_not_called()

    def test_download_cache_activity_does_not_invalidate_model(self):
        target = self.fixture()
        self.prepare(target)
        self.write(target, ".cache/huggingface/download/other.lock", "")
        self.write(target, "__pycache__/module.pyc", "ignored cache")
        self.assertTrue(models._looks_prepared(target, TEXT))
        with patch.object(models, "_validate_local_model", side_effect=forbidden):
            self.prepare(target)

    def test_corrupt_or_wrong_kind_receipt_requires_validation_not_download(self):
        target = self.fixture()
        for payload in ("{", "[]", '{"schema": 1, "complete": true, "kind": "image", "files": {}}'):
            self.receipt(target).write_text(payload)
            self.assertFalse(models._looks_prepared(target, TEXT))
            self.prepare(target)
            self.assertTrue(models._looks_prepared(target, TEXT))
        self.download.assert_not_called()

    def test_force_download_invalidates_receipt_before_interruption(self):
        target = self.fixture()
        self.prepare(target)
        def interrupted(**kwargs):
            self.assertTrue(kwargs["force_download"])
            self.assertFalse(self.receipt(target).exists())
            raise OSError("synthetic disconnect")
        self.download.side_effect = interrupted
        with self.assertRaises(OSError):
            self.prepare(target, force_download=True)
        self.assertFalse(models._looks_prepared(target, TEXT))
        self.assertTrue((target / "model.safetensors").is_file())

    def test_read_only_valid_custom_model_does_not_require_receipt(self):
        target = self.fixture()
        with patch.object(models.tempfile, "NamedTemporaryFile", side_effect=PermissionError("read only")):
            self.prepare(target, validate_only=True)
        self.assertFalse(self.receipt(target).exists())
        self.download.assert_not_called()

    def test_reuse_never_reads_weights_or_calls_loader(self):
        target = self.fixture(sharded=True)
        self.prepare(target)
        original_open = Path.open
        def guard(path, *args, **kwargs):
            if path.suffix in (".bin", ".safetensors"):
                forbidden()
            return original_open(path, *args, **kwargs)
        with patch.object(Path, "open", guard), patch.object(models, "_validate_local_model", side_effect=forbidden):
            self.prepare(target)
        self.download.assert_not_called()

    def test_environment_restored_after_failed_validation(self):
        target = self.fixture()
        with patch.dict(os.environ, {"HF_HUB_OFFLINE": "0", "TRANSFORMERS_OFFLINE": "custom"}):
            (target / "model.safetensors").unlink()
            with self.assertRaises(SystemExit):
                self.prepare(target, validate_only=True)
            self.assertEqual(os.environ["HF_HUB_OFFLINE"], "0")
            self.assertEqual(os.environ["TRANSFORMERS_OFFLINE"], "custom")

    def test_mutation_during_validation_does_not_create_receipt(self):
        target = self.fixture()
        original = models._validate_local_model
        def mutation(*args):
            original(*args)
            self.write(target, "new-model-file.json", "{}")
        with patch.object(models, "_validate_local_model", side_effect=mutation):
            with self.assertRaises(SystemExit):
                self.prepare(target, validate_only=True)
        self.assertFalse(self.receipt(target).exists())

    def test_custom_module_layout_is_delegated_to_runtime_loader(self):
        target = self.base / "custom-embedding"
        self.write(target, "modules.json", '[{"path": "custom", "type": "supported-local-module"}]')
        self.write(target, "custom/embeddings.custom", "valid custom artifact")
        with patch.object(models, "_validate_local_model") as loader:
            self.prepare(target, validate_only=True)
            loader.assert_called_once()
        self.assertTrue(models._looks_prepared(target, TEXT))
        self.download.assert_not_called()

    def test_symlinked_weights_and_module_directory_are_supported(self):
        original = self.fixture(self.base / "original", nested=True)
        target = self.base / "symlinked"
        target.mkdir()
        (target / "modules.json").symlink_to(original / "modules.json")
        (target / "0_Transformer").symlink_to(original / "0_Transformer", target_is_directory=True)
        self.prepare(target, validate_only=True)
        self.assertTrue(models._looks_prepared(target, TEXT))
        (original / "0_Transformer/model.safetensors").unlink()
        self.assertFalse(models._looks_prepared(target, TEXT))

    def run_cli(self, argv):
        with patch.object(sys, "argv", [argv[1]] + argv[2:]):
            return models.main()

    def installation(self, *, custom=False):
        root = self.base / "source with spaces"
        root.mkdir()
        prefix = self.base / "candidate"
        prefix.mkdir()
        request = manager.InstallRequest(root=root, home=self.base, modules=("core", "memory"), robot="customrobot", jobs=1)
        text = self.base / "custom text" if custom else root / "runtime_data/models/BAAI__bge-m3"
        image = self.base / "custom image" if custom else root / "runtime_data/models/openai__clip-vit-base-patch32"
        (prefix / "settings.env").write_text("NUWAX_MEMORY_TEXT_EMBEDDING_MODEL=" + str(text) + "\nNUWAX_MEMORY_IMAGE_EMBEDDING_MODEL=" + str(image) + "\n")
        return request, prefix, text, image

    def test_manager_validates_both_custom_paths_without_downloading_defaults(self):
        request, prefix, text, image = self.installation(custom=True)
        self.fixture(text)
        self.fixture(image, IMAGE)
        run = Mock(side_effect=self.run_cli)
        manager._prepare_models(request, prefix, run)
        self.assertEqual(run.call_count, 2)
        for call in run.call_args_list:
            self.assertIn("--validate-only", call.args[0])
        self.assertTrue(models._looks_prepared(text, TEXT))
        self.assertTrue(models._looks_prepared(image, IMAGE))
        self.download.assert_not_called()

    def test_manager_rejects_incomplete_custom_path_without_default_download(self):
        request, prefix, text, image = self.installation(custom=True)
        self.write(text, "modules.json", "{}")
        self.fixture(image, IMAGE)
        with self.assertRaises(SystemExit):
            manager._prepare_models(request, prefix, self.run_cli)
        self.download.assert_not_called()
        self.assertFalse((request.root / "runtime_data/models").exists())

    def test_manager_configs_only_default_download_cannot_mark_success(self):
        request, prefix, text, image = self.installation()
        for target, required in ((text, TEXT), (image, IMAGE)):
            for name in required:
                self.write(target, name, "{}")
        self.download.side_effect = lambda **kwargs: kwargs["target_dir"]
        with self.assertRaises(SystemExit):
            manager._prepare_models(request, prefix, self.run_cli)
        self.assertFalse(self.receipt(text).exists())
        self.assertFalse(self.receipt(image).exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
