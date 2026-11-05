"""制品真实文件读取、元数据独立副本与库存错误。"""
import os
from pathlib import Path

import pytest

from contracts.artifacts import ArtifactKind
from core.errors import ArtifactNotFoundError, GatewayError
from services.artifact_store import LocalArtifactStore


@pytest.fixture
def stores(tmp_path):
    return tmp_path, LocalArtifactStore(str(tmp_path), 'http://test'), LocalArtifactStore(str(tmp_path), 'http://test')


def save(store, kind=ArtifactKind.IMAGE):
    return store.save_bytes(kind=kind,mime_type='application/octet-stream',data=b'fixture',metadata={'marker':'a'})


def test_all_kinds_cold_and_warm_preserve_metadata_and_payload(stores):
    _, seed, reader=stores
    for kind in ArtifactKind:
        ref=save(seed,kind)
        assert reader.get_ref(ref.artifact_id)==ref
        assert reader.read_bytes(ref.artifact_id)==b'fixture'
        assert reader.get_ref(ref.artifact_id)==ref


def test_moved_pair_remains_readable_from_actual_files(stores):
    root, seed, reader=stores
    ref=save(seed)
    meta=root/ref.kind.value/(ref.artifact_id+'.meta.json')
    payload=seed.get_file_path(ref.artifact_id)
    destination=root/'nested'/'photos';destination.mkdir(parents=True)
    meta.rename(destination/meta.name);payload.rename(destination/payload.name)
    assert reader.get_ref(ref.artifact_id)==ref
    assert reader.read_bytes(ref.artifact_id)==b'fixture'


def test_sidecar_can_be_read_but_missing_payload_is_a_real_error(stores):
    _, seed, reader=stores
    ref=save(seed);seed.get_file_path(ref.artifact_id).unlink()
    assert reader.get_ref(ref.artifact_id)==ref
    with pytest.raises(ArtifactNotFoundError):reader.read_bytes(ref.artifact_id)


def test_edits_replacement_and_copy_isolation(stores):
    root, seed, reader=stores
    ref=save(seed);reader.get_ref(ref.artifact_id).metadata['marker']='external'
    assert reader.get_ref(ref.artifact_id).metadata['marker']=='a'
    path=root/ref.kind.value/(ref.artifact_id+'.meta.json');stat=path.stat()
    path.write_text(path.read_text().replace('"marker": "a"','"marker": "b"'))
    os.utime(path,ns=(stat.st_atime_ns,stat.st_mtime_ns))
    assert reader.get_ref(ref.artifact_id).metadata['marker']=='b'
    replacement=path.with_suffix('.new')
    replacement.write_text(ref.model_copy(update={'metadata':{'marker':'replacement'}}).model_dump_json())
    os.replace(replacement,path)
    assert reader.get_ref(ref.artifact_id).metadata['marker']=='replacement'


def test_missing_sidecar_and_mismatched_metadata_are_real_errors(stores):
    root, seed, reader=stores
    ref,other=save(seed),save(seed)
    path=root/ref.kind.value/(ref.artifact_id+'.meta.json');path.write_text(other.model_dump_json())
    with pytest.raises(GatewayError) as error:reader.get_ref(ref.artifact_id)
    assert error.value.error_code=='artifact_inventory_invalid'
    path.unlink()
    with pytest.raises(ArtifactNotFoundError):reader.get_ref(ref.artifact_id)


def test_duplicate_sidecar_is_reported_by_real_inventory(stores):
    root,seed,reader=stores
    ref=save(seed)
    (root/(ref.artifact_id+'.meta.json')).write_text(ref.model_dump_json())
    with pytest.raises(GatewayError) as error:reader.list_refs()
    assert error.value.error_code=='artifact_inventory_invalid'
