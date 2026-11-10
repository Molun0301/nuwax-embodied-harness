#!/usr/bin/env python3
"""Offline provider/config regressions; no real model, network, install or device use.

Run with the existing project test interpreter. Constructor tests replace learned
providers with sentinels; generated runtime files stay in temporary directories.
"""
from __future__ import annotations

from contextlib import ExitStack
from pathlib import Path
import sys
import json
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.module_installer import runtime
from core.config import ConfigStore, flatten, read_config
from services.model_providers import resolve_text_embedding_provider_spec


TEXT_MODEL = "memory.text_embedding_model"
TEXT_DIM = "memory.text_embedding_dim"
IMAGE_MODEL = "memory.image_embedding_model"
IMAGE_DIM = "memory.image_embedding_dim"
SAVED = "_NUWAX_INSTALLER_SAVED_"
ONLINE = "tongyi-embedding-vision-flash-2026-03-06"


class ProviderSelectionChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from services.memory import vectorizer
        cls.vectorizer = vectorizer

    def test_no_model_and_legacy_aliases_have_no_vectorizer_or_capability(self):
        aliases = ("", "  ", "lexical-only", "lexical_only", "disabled", "none", "off",
                   "hashing-v1", "HASHING-V2", "hashing-custom-old")
        with ExitStack() as stack:
            local = stack.enter_context(patch.object(self.vectorizer, "SentenceTransformerTextEmbedder"))
            cloud = stack.enter_context(patch.object(self.vectorizer, "DashScopeTextEmbedder"))
            for alias in aliases:
                with self.subTest(alias=alias):
                    self.assertIsNone(self.vectorizer.build_text_embedder(alias, 256))
                    spec = resolve_text_embedding_provider_spec(alias)
                    metadata = spec.to_metadata()
                    self.assertFalse(spec.is_enabled())
                    self.assertEqual(metadata["provider_kind"], "disabled")
                    self.assertEqual(metadata["provider_name"], "lexical_only")
                    self.assertEqual(metadata["model_name"], "lexical-only")
                    self.assertEqual(metadata["availability"], "disabled")
                    self.assertFalse(any(metadata["capabilities"].values()))
                    self.assertEqual(metadata["requested_model_name"], alias.strip())
                    self.assertEqual(metadata["resolved_model_name"], "lexical-only")
                    legacy = alias.lower().startswith("hashing")
                    self.assertEqual(metadata["fallback_used"], legacy)
                    self.assertEqual(metadata["fallback_reason"], "legacy_hashing_removed" if legacy else "")
            local.assert_not_called()
            cloud.assert_not_called()
        self.assertFalse(hasattr(self.vectorizer, "HashingTextEmbedder"))

    def test_no_model_does_not_validate_an_unused_vector_dimension(self):
        self.assertIsNone(self.vectorizer.build_text_embedder("lexical-only", 0))
        self.assertIsNone(self.vectorizer.build_text_embedder("hashing-v1", -7))

    def test_real_local_provider_constructor_and_declared_capability_remain(self):
        model = "sentence-transformers:BAAI/bge-small-zh-v1.5"
        sentinel = object()
        with patch.object(self.vectorizer, "SentenceTransformerTextEmbedder", return_value=sentinel) as build:
            self.assertIs(self.vectorizer.build_text_embedder(model, 512), sentinel)
        build.assert_called_once_with(model_name=model, dimension=512)
        self.assertTrue(resolve_text_embedding_provider_spec(model).to_metadata()["capabilities"]["embedText"])

    def test_real_cloud_provider_constructor_arguments_remain(self):
        sentinel = object()
        with patch.object(self.vectorizer, "DashScopeTextEmbedder", return_value=sentinel) as build:
            actual = self.vectorizer.build_text_embedder(
                ONLINE, 768, api_key="fixture-only", base_url="https://example.invalid", timeout_sec=3.0,
            )
        self.assertIs(actual, sentinel)
        build.assert_called_once_with(
            model_name=ONLINE, dimension=768, api_key="fixture-only",
            base_url="https://example.invalid", timeout_sec=3.0,
        )
        self.assertEqual(resolve_text_embedding_provider_spec(ONLINE).provider_name, "dashscope")

    def test_image_provider_remains_available_without_text_provider(self):
        sentinel = object()
        with patch.object(self.vectorizer, "DashScopeVisionLanguageEmbedder", return_value=sentinel) as build:
            self.assertIsNone(self.vectorizer.build_text_embedder("lexical-only", 256))
            self.assertIs(self.vectorizer.build_vision_language_embedder(ONLINE, 768), sentinel)
        self.assertEqual(build.call_count, 1)

    def test_real_provider_failure_is_not_converted_to_a_fake_vectorizer(self):
        with patch.object(self.vectorizer, "SentenceTransformerTextEmbedder", side_effect=RuntimeError("fixture")):
            with self.assertRaisesRegex(RuntimeError, "fixture"):
                self.vectorizer.build_text_embedder("sentence-transformers:fixture", 16)
        with self.assertRaises(ValueError):
            self.vectorizer.build_text_embedder("unrecognized-provider", 16)


class GeneratedMemorySettingsChecks(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="neh-provider-config-")
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.home = self.root / "install"
        self.prefix = self.home / "generations/one"
        self.prefix.mkdir(parents=True)
        (self.prefix / "venv").symlink_to(sys.prefix, target_is_directory=True)
        (self.prefix / "source").symlink_to(ROOT, target_is_directory=True)
        (self.home / "current").symlink_to("generations/one")
        self.store = ConfigStore(self.home)

    def settings(self, existing=None, enabled=True):
        if existing is not None:
            self.store.install("测试", existing)
        # 停用只影响合成配置；再次安装必须从用户方案读取，不能回存停用后的运行快照。
        configured = self.store.request("existing")["values"]
        modules = ["core", "memory"] if enabled else ["core"]
        values = runtime.write_runtime(ROOT, self.prefix, modules, "customrobot", configured)
        self.store.install("测试", values)
        (self.prefix / "installed.json").write_text(json.dumps({"robot": "customrobot", "modules": modules}))
        self.store.compose()
        return flatten(read_config(self.home))

    def legacy_settings(self, values):
        # 旧键只作为升级导入输入，恢复规则由现行导入器验证。
        directory = self.home / "config/profiles/customrobot"
        directory.mkdir(parents=True)
        (directory / "测试.env").write_text("".join(key + "=" + str(value) + "\n" for key, value in values.items()))
        (directory / ".active").write_text("测试")
        (self.prefix / "installed.json").write_text(json.dumps({"robot": "customrobot", "modules": ["core"]}))
        return self.settings(enabled=False)

    def test_disabled_module_uses_lexical_only(self):
        result = self.settings({}, enabled=False)
        self.assertEqual(result[TEXT_MODEL], "lexical-only")
        self.assertEqual(result[TEXT_DIM], 256)
        self.assertEqual(result[IMAGE_MODEL], "disabled")
        self.assertEqual(result[IMAGE_DIM], 512)

    def test_enabled_fresh_module_keeps_existing_local_model_defaults(self):
        result = self.settings({})
        self.assertEqual(result[TEXT_MODEL], str(self.home / "data/models/BAAI__bge-small-zh-v1.5"))
        self.assertEqual(result[IMAGE_MODEL], str(self.home / "data/models/openai__clip-vit-base-patch32"))
        self.assertEqual(result[TEXT_DIM], 512)
        self.assertEqual(result[IMAGE_DIM], 512)

    def test_real_cloud_models_are_not_replaced_by_local_defaults(self):
        existing = {TEXT_MODEL: ONLINE, TEXT_DIM: 768, IMAGE_MODEL: ONLINE, IMAGE_DIM: 768}
        result = self.settings(existing)
        for key, value in existing.items():
            self.assertEqual(result[key], value)

    def test_custom_models_and_dimensions_survive_disable_and_reenable(self):
        original = {TEXT_MODEL: "/models/my-bge", TEXT_DIM: 384, IMAGE_MODEL: "/models/my-clip", IMAGE_DIM: 768}
        disabled = self.settings(original, enabled=False)
        self.assertEqual(disabled[TEXT_MODEL], "lexical-only")
        disabled_again = self.settings(enabled=False)
        self.assertEqual(disabled_again, disabled)
        restored = self.settings()
        for key, value in original.items():
            self.assertEqual(restored[key], value)

    def test_changed_real_model_replaces_saved_dimension_even_when_equal_to_disabled_default(self):
        current = {
            "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": "sentence-transformers:new-model", "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "256",
            "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL": "/models/new-clip", "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM": "512",
            SAVED + "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": ONLINE, SAVED + "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "768",
            SAVED + "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL": ONLINE, SAVED + "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM": "768",
        }
        self.legacy_settings(current)
        restored = self.settings()
        self.assertEqual(restored[TEXT_MODEL], current["NUWAX_MEMORY_TEXT_EMBEDDING_MODEL"])
        self.assertEqual(restored[TEXT_DIM], 256)
        self.assertEqual(restored[IMAGE_MODEL], current["NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL"])
        self.assertEqual(restored[IMAGE_DIM], 512)

    def test_legacy_no_model_does_not_overwrite_saved_real_provider_dimension(self):
        prior = {"NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": "hashing-old", "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "32",
                 SAVED + "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL": ONLINE, SAVED + "NUWAX_MEMORY_TEXT_EMBEDDING_DIM": "768"}
        self.legacy_settings(prior)
        restored = self.settings()
        self.assertEqual(restored[TEXT_MODEL], ONLINE)
        self.assertEqual(restored[TEXT_DIM], 768)




    def test_generated_disabled_file_preserves_saved_real_model(self):
        self.settings({TEXT_MODEL: ONLINE, TEXT_DIM: 768}, enabled=False)
        result = runtime.read_settings(self.home / "run/config.toml")
        saved = self.store.profile_values(self.store.active())
        self.assertEqual(result[TEXT_MODEL], "lexical-only")
        self.assertEqual(saved[TEXT_MODEL], ONLINE)
        self.assertEqual(saved[TEXT_DIM], 768)
        self.assertEqual(self.settings()[TEXT_MODEL], ONLINE)






if __name__ == "__main__":
    unittest.main()
