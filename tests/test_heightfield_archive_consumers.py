"""独立离线AR5字节解析；不恢复旧资产库、checkpoint上传或原生激活。"""
import os
from pathlib import Path
import unittest

from services.geometry.checkpoint_payload import CHUNK_BYTES, FrozenPayload
from services.geometry.fine_map_archive import read_fine_archive_root, validate_fine_archive
from tests.test_heightfield_archive import archive, full


class HeightfieldArchiveConsumerTests(unittest.TestCase):
    def test_ar5_direct_file_and_frozen_bytes_preserve_complete_geometry(self):
        raw = archive(full())
        expected = validate_fine_archive(raw)
        payload = FrozenPayload(raw[offset:offset + CHUNK_BYTES] for offset in range(0, len(raw), CHUNK_BYTES))
        try:
            self.assertEqual(validate_fine_archive(payload), expected)
            self.assertEqual(read_fine_archive_root(payload)[1], expected)
            self.assertEqual(payload[:len(payload)], raw)
        finally:
            payload._data.close()

    def test_ar5_malformed_and_source_mismatch_keep_original_rejection(self):
        raw = archive(full())
        for malformed in (raw[:-1], archive(full()[:-1])):
            with self.subTest(size=len(malformed)), self.assertRaises(ValueError):
                validate_fine_archive(malformed)
        with self.assertRaisesRegex(ValueError, '原占据/点云提交不一致'):
            validate_fine_archive(raw, session='different-source')

    def test_actual_rust_archives_are_decoded_from_unmodified_bytes(self):
        directory = os.environ.get('NEH_HEIGHTFIELD_ARCHIVE_DIR')
        if not directory:
            self.skipTest('set NEH_HEIGHTFIELD_ARCHIVE_DIR to actual Rust storage qualification output')
        for name in ('cross-block.ar5', 'zero-mass-withdrawn.ar5', 'mixed.ar5', 'full-1024.ar5', 'legacy.ar4'):
            with self.subTest(name=name):
                raw = (Path(directory) / name).read_bytes()
                expected = validate_fine_archive(raw, session='heightfield-storage')
                payload = FrozenPayload(raw[offset:offset + CHUNK_BYTES] for offset in range(0, len(raw), CHUNK_BYTES))
                try:
                    self.assertEqual(validate_fine_archive(payload, session='heightfield-storage'), expected)
                finally:
                    payload._data.close()
