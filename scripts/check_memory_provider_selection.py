#!/usr/bin/env python3
"""Offline provider/config regressions; no real model, network, install or device use.

Run with the existing project test interpreter. Constructor tests replace learned
providers with sentinels; generated runtime files stay in temporary directories.
"""
from __future__ import annotations

from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.module_installer import runtime
from services.model_providers import resolve_text_embedding_provider_spec


TEXT_MODEL = "NUWAX_MEMORY_TEXT_EMBEDDING_MODEL"
TEXT_DIM = "NUWAX_MEMORY_TEXT_EMBEDDING_DIM"
IMAGE_MODEL = "NUWAX_MEMORY_IMAGE_EMBEDDING_MODEL"
IMAGE_DIM = "NUWAX_MEMORY_IMAGE_EMBEDDING_DIM"
OLD_MODEL = "NUWAX_MEMORY_EMBEDDING_MODEL"
OLD_DIM = "NUWAX_MEMORY_EMBEDDING_DIM"
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
        model = "sentence-transformers:BAAI/bge-m3"
        sentinel = object()
        with patch.object(self.vectorizer, "SentenceTransformerTextEmbedder", return_value=sentinel) as build:
            self.assertIs(self.vectorizer.build_text_embedder(model, 1024), sentinel)
        build.assert_called_once_with(model_name=model, dimension=1024)
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

    def settings(self, existing, enabled=True):
        values = dict(existing)
        result = runtime._memory_settings(self.root, values, enabled)
        values.update(result)
        return values

    def test_disabled_module_uses_lexical_only_with_synchronized_legacy_keys(self):
        result = self.settings({}, enabled=False)
        self.assertEqual(result[TEXT_MODEL], "lexical-only")
        self.assertEqual(result[OLD_MODEL], "lexical-only")
        self.assertEqual(result[TEXT_DIM], result[OLD_DIM])
        self.assertEqual(result[IMAGE_MODEL], "disabled")
        self.assertEqual(result[IMAGE_DIM], "512")

    def test_enabled_fresh_module_keeps_existing_local_model_defaults(self):
        result = self.settings({})
        self.assertEqual(result[TEXT_MODEL], str(self.root / "runtime_data/models/BAAI__bge-m3"))
        self.assertEqual(result[IMAGE_MODEL], str(self.root / "runtime_data/models/openai__clip-vit-base-patch32"))
        self.assertEqual(result[TEXT_DIM], "1024")
        self.assertEqual(result[IMAGE_DIM], "512")

    def test_real_cloud_models_are_not_replaced_by_local_defaults(self):
        existing = {TEXT_MODEL: ONLINE, TEXT_DIM: "768", IMAGE_MODEL: ONLINE, IMAGE_DIM: "768"}
        result = self.settings(existing)
        for key, value in existing.items():
            self.assertEqual(result[key], value)
        self.assertEqual(result[OLD_MODEL], ONLINE)
        self.assertEqual(result[OLD_DIM], "768")

    def test_custom_models_and_dimensions_survive_disable_and_reenable(self):
        original = {TEXT_MODEL: "/models/my-bge", TEXT_DIM: "384", IMAGE_MODEL: "/models/my-clip", IMAGE_DIM: "768"}
        disabled = self.settings(original, enabled=False)
        self.assertEqual(disabled[TEXT_MODEL], "lexical-only")
        disabled_again = self.settings(disabled, enabled=False)
        restored = self.settings(disabled_again)
        for key, value in original.items():
            self.assertEqual(restored[key], value)

    def test_changed_real_model_replaces_saved_dimension_even_when_equal_to_disabled_default(self):
        current = {
            TEXT_MODEL: "sentence-transformers:new-model", TEXT_DIM: "256",
            IMAGE_MODEL: "/models/new-clip", IMAGE_DIM: "512",
            SAVED + TEXT_MODEL: ONLINE, SAVED + TEXT_DIM: "768",
            SAVED + IMAGE_MODEL: ONLINE, SAVED + IMAGE_DIM: "768",
        }
        restored = self.settings(self.settings(current, enabled=False))
        self.assertEqual(restored[TEXT_MODEL], current[TEXT_MODEL])
        self.assertEqual(restored[TEXT_DIM], "256")
        self.assertEqual(restored[IMAGE_MODEL], current[IMAGE_MODEL])
        self.assertEqual(restored[IMAGE_DIM], "512")

    def test_legacy_no_model_does_not_overwrite_saved_real_provider_dimension(self):
        prior = {TEXT_MODEL: "hashing-old", TEXT_DIM: "32",
                 SAVED + TEXT_MODEL: ONLINE, SAVED + TEXT_DIM: "768"}
        restored = self.settings(self.settings(prior, enabled=False))
        self.assertEqual(restored[TEXT_MODEL], ONLINE)
        self.assertEqual(restored[TEXT_DIM], "768")

    def test_old_text_keys_are_read_and_new_keys_win(self):
        legacy = self.settings({OLD_MODEL: "sentence-transformers:custom", OLD_DIM: "384"})
        self.assertEqual(legacy[TEXT_MODEL], "sentence-transformers:custom")
        self.assertEqual(legacy[TEXT_DIM], "384")
        current = self.settings({TEXT_MODEL: ONLINE, TEXT_DIM: "768", OLD_MODEL: "old-model", OLD_DIM: "32"})
        self.assertEqual(current[TEXT_MODEL], ONLINE)
        self.assertEqual(current[OLD_MODEL], ONLINE)
        self.assertEqual(current[TEXT_DIM], "768")
        self.assertEqual(current[OLD_DIM], "768")

    def test_legacy_text_keys_survive_disable_and_reenable(self):
        result = self.settings(self.settings({OLD_MODEL: ONLINE, OLD_DIM: "768"}, enabled=False))
        self.assertEqual(result[TEXT_MODEL], ONLINE)
        self.assertEqual(result[TEXT_DIM], "768")

    def test_generated_file_template_cannot_shadow_user_legacy_model(self):
        (self.root / ".env.example").write_text(f"{TEXT_MODEL}=template-bge\n{TEXT_DIM}=1024\n")
        prefix = self.root / "install/generations/one"
        runtime.write_runtime(self.root, prefix, ["core", "memory"], "customrobot", {
            OLD_MODEL: ONLINE, OLD_DIM: "768",
        })
        result = runtime.read_settings(prefix / "settings.env")
        self.assertEqual(result[TEXT_MODEL], ONLINE)
        self.assertEqual(result[TEXT_DIM], "768")
        self.assertEqual(result[OLD_MODEL], ONLINE)

    def test_generated_disabled_file_preserves_saved_real_model(self):
        prefix = self.root / "install/generations/one"
        runtime.write_runtime(self.root, prefix, ["core"], "customrobot", {TEXT_MODEL: ONLINE, TEXT_DIM: "768"})
        result = runtime.read_settings(prefix / "settings.env")
        self.assertEqual(result[TEXT_MODEL], "lexical-only")
        self.assertEqual(result[SAVED + TEXT_MODEL], ONLINE)
        self.assertEqual(result[SAVED + TEXT_DIM], "768")
        self.assertEqual(self.settings(result)[TEXT_MODEL], ONLINE)






if __name__ == "__main__":
    unittest.main()
