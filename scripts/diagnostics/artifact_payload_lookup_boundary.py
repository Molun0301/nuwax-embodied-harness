#!/usr/bin/env python3
"""Two owned fixtures showing why canonical payload guessing was not adopted.

Python 3.8 compatible; no timing, network, hashes, or existing artifact access.
"""
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from contracts.artifacts import ArtifactKind
from services.artifact_store import LocalArtifactStore


def inspect_boundary():
    result = {"python": sys.version, "production_change": False}
    with tempfile.TemporaryDirectory(prefix="neh-payload-choice-audit-") as directory:
        root = Path(directory)
        seed = LocalArtifactStore(directory, "http://diagnostic.invalid")
        ref = seed.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"canonical")
        canonical = seed.get_file_path(ref.artifact_id)
        orphan = root / (ref.artifact_id + ".legacy")
        orphan.write_bytes(b"legacy-root")
        cold = LocalArtifactStore(directory, "http://diagnostic.invalid")
        selected = cold.get_file_path(ref.artifact_id)
        entries = LocalArtifactStore(directory, "http://diagnostic.invalid")._scan_entries()
        assert len(entries) == 1 and entries[0].file_path == canonical
        assert selected == orphan and selected.read_bytes() != canonical.read_bytes()
        result["accepted_orphan_inventory"] = {
            "inventory_entries": len(entries),
            "cold_lookup_selects_root_orphan": selected == orphan,
            "inventory_selects_canonical_payload": entries[0].file_path == canonical,
            "canonical_probe_would_change_bytes": selected.read_bytes() != canonical.read_bytes(),
        }
    with tempfile.TemporaryDirectory(prefix="neh-payload-extension-audit-") as directory:
        seed = LocalArtifactStore(directory, "http://diagnostic.invalid")
        ref = seed.save_bytes(kind=ArtifactKind.IMAGE, mime_type="image/jpeg", data=b"custom", extension="custom")
        fresh = LocalArtifactStore(directory, "http://diagnostic.invalid")
        persisted = fresh.get_ref(ref.artifact_id)
        selected = fresh.get_file_path(ref.artifact_id)
        guess = Path(directory) / "image" / (ref.artifact_id + seed._guess_extension(persisted.mime_type))
        assert selected.name == ref.artifact_id + "custom" and not guess.exists()
        assert persisted.metadata == {}
        result["custom_extension"] = {
            "persisted_reference_fields": list(persisted.model_dump()),
            "persisted_metadata": persisted.metadata,
            "custom_suffix_resolved_by_existing_lookup": True,
            "mime_guessed_path_exists": guess.exists(),
        }
    return result


if __name__ == "__main__":
    print(json.dumps(inspect_boundary(), indent=2))
