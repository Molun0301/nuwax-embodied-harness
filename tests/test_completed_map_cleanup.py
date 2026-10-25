"""Cleanup orchestration uses real checkpoint publication and save/reload."""
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import struct
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from contracts.geometry import Pose
from contracts.geometry_runtime import PlatformGeometryConfig
from contracts.map_workspace import (AssetIdentityV2, MapAsset, MapAssetBundle,
    MapGeometryAssetBinding, MapGeometryAssetRef, MapGeometryAssetType)
from contracts.maps import OccupancyGrid
from contracts.runtime_views import MapSnapshot
from services.geometry.mapping_backend_checkpoint_store import (MappingBackendCheckpointStore,
    MappingBackendCheckpoint, CheckpointMemberIdentity, CheckpointArtifactReference)
from services.geometry.runtime import PlatformGeometryRuntime
from services.mapping.asset_store import MapAssetBundleRepository
from services.mapping.catalog import MapCatalogRepository
from services.mapping.completed_cleanup import CompletedMapCleanup, copy_raw_scan_history


def fine_bytes(revision):
    root = dict(version=2, domain=dict(session='source', generation=3, frame_generation=0, producer_generation=8),
        revision=revision, geometry_revision=revision, source_timestamp_ns=11, voxel_size_m=.125,
        blocks=[dict(key=[0, 0, 0], points=1, object=dict(kind='assigned', value=[40]*16),
                     block_revision=7, geometry_revision=7)])
    raw = json.dumps(root).encode()
    block = b'NUWAXBL1' + struct.pack('<IIHQH', 1, 0, 0, 11, 1)
    return b'NUWAXAR2' + struct.pack('<Q', len(raw)) + raw + struct.pack('<QI', 1, len(block)) + block


def dense_bytes(revision):
    header = json.dumps(dict(version=1, session_id='source', session_generation=3, cloud_revision=revision,
        source_timestamp_ns=11, frame_id='map', point_count=2, bounds_min=[.0625]*3, bounds_max=[.0625]*3)).encode()
    return b'NUWAXDC1' + struct.pack('<I', len(header)) + header + struct.pack('<fff', *([.0625]*3))*2


class Owner:
    def __init__(self, root):
        self._repository = MapCatalogRepository(str(root))
        self._asset_repository = MapAssetBundleRepository(str(root))
        self._geometry_runtime = PlatformGeometryRuntime.__new__(PlatformGeometryRuntime)
        self._geometry_runtime.config = PlatformGeometryConfig(stack='test')
        self._geometry_runtime.get_mapping_session_context = lambda: {'active': False}
    def get_map_asset_bundle(self, *, map_name):
        asset = self._repository.get_map_asset(map_name)
        return self._asset_repository.get_map_asset_bundle(map_name, migrate_legacy=False,
            expected_asset_identity=asset.asset_identity if asset else None)
    def _require_map_name(self, name):
        if not name or '/' in name or name in {'.', '..'}:
            raise ValueError('bad name')
        return name


def fixture(root):
    owner = Owner(root)
    name, checkpoint_id = 'original', 'a'*32
    receipt = dict(commit_id=dict(boot_nonce=b'N'*16, sequence=5), session_id='source', session_generation=3,
        occupancy_revision=6, source_timestamp_ns=11, backend_kind='voxel', backend_generation=2)
    cloud_receipt = dict(commit_id=receipt['commit_id'], session_generation=3, occupancy_revision=6,
        cloud_revision=7, source_timestamp_ns=11, point_count=1)
    def member(role, producer_generation=0):
        return CheckpointMemberIdentity(checkpoint_id, role, receipt, cloud_receipt, 'map', producer_generation=producer_generation)
    body = b'NWMBCKP\0'+struct.pack('<IBBHQQQQddd',1,0,0,0,16,1,1,11,.0625,.0625,.0625)
    fine, dense, xyz = fine_bytes(7), dense_bytes(7), struct.pack('<fff', *([.0625]*3))
    checkpoint = MappingBackendCheckpoint(2, 'voxel', 'source', 3, b'', len(body), 12, body, receipt, cloud_receipt,
        checkpoint_id=checkpoint_id, canonical_occupancy_data=b'\x64\xff', canonical_cloud_xyz=xyz,
        backend_identity=member('backend_state'), occupancy_identity=member('canonical_occupancy'), cloud_identity=member('canonical_cloud'),
        fine_archive=fine, fine_reference=CheckpointArtifactReference('b'*32,len(fine),2,member('fine_map',8)),
        dense_archive=dense, dense_reference=CheckpointArtifactReference('c'*32,len(dense),2,member('dense_cloud')))
    occupancy = dict(raw_data_size=2,width=2,height=1,resolution=.125,origin_x=0.,origin_y=0.,frame_id='map',source_occupancy_revision=6,timestamp_ns=11)
    cloud = dict(xyz_size=12,frame_id='map',source_cloud_revision=7,point_count=1,voxel_size=.125,payload_schema_version=2)
    staging = owner._asset_repository.create_staging_directory(name)
    stored = MappingBackendCheckpointStore(staging).publish(checkpoint,occupancy_asset=occupancy,cloud_asset=cloud)
    grid=OccupancyGrid(map_id=name,frame_id='map',width=2,height=1,resolution_m=.125,origin=Pose(frame_id='map'),data=[100,-1])
    snapshot=MapSnapshot(map_name=name,source_name='test',source_epoch='source-3',update_sequence=6,frame_id='map',occupancy_grid=grid)
    paths={'db':'geometry/test/mapping/original.db','pcd':'geometry/test/pcd/cloud.pcd','nav':'geometry/test/navigation/map.yaml'}
    geometry=MapGeometryAssetBinding(map_name=name,geometry_stack='test',frame_id='map',mapping_database_path=paths['db'],pcd_path=paths['pcd'],
        occupancy_projection='canonical',costmap_source='canonical',assets=[
            MapGeometryAssetRef(asset_type=MapGeometryAssetType.MAPPING_DATABASE,path=paths['db'],frame_id='map'),
            MapGeometryAssetRef(asset_type=MapGeometryAssetType.PCD,path=paths['pcd'],frame_id='map'),
            MapGeometryAssetRef(asset_type=MapGeometryAssetType.NAVIGATION_MAP,path=paths['nav'],frame_id='map')],
        metadata={'geometry_asset_path':'geometry/test/geometry_asset.json','mapping_backend_checkpoint':dict(present=True,schema_version=2,
            backend_kind='voxel',checkpoint_id=checkpoint_id,manifest_path=stored.manifest_path.relative_to(staging).as_posix())})
    identity=AssetIdentityV2(asset_id='d'*32)
    bundle=MapAssetBundle(map_name=name,source_name='test',frame_id='map',snapshot=snapshot,geometry_asset=geometry,
        identity_schema_version=2,asset_identity=identity,asset_state='candidate')
    for path,data in ((paths['db'],b'state'),(paths['pcd'],b'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n'+xyz),(paths['nav'],b'image: map.pgm\nresolution: 0.125\n'),
                      ('geometry/test/navigation/map.pgm',b'P5\n2 1\n255\n\x00\xcd')):
        target=staging/path;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    from services.geometry.geometry_runtime_provider import MappingBackendCheckpointExportBundle
    owner._geometry_runtime._materialize_native_bundle_database(staging/paths['db'],bundle=bundle,
        checkpoint_bundle=MappingBackendCheckpointExportBundle(checkpoint,occupancy,cloud),mapping_context={'session_id':'source','session_generation':3})
    owner._asset_repository.finalize_assigned_map_asset_bundle(bundle,staging_dir=staging,asset_identity=identity)
    owner._repository.save_map_asset(MapAsset(map_name=name,display_name=name,bound_memory_library_name=name,
        identity_schema_version=2,asset_identity=identity,finalized=True))
    return owner


def prepared(owner):
    service=CompletedMapCleanup(owner)
    with patch.object(service.executor,'submit'):
        result=service.start(map_name='original',strength_percent=50)
    job=service.jobs[result['id']];folder=job['folder']/ 'result';folder.mkdir()
    for name,data in [('fine.bin',fine_bytes(8)),('dense.bin',dense_bytes(8)),('backend.bin',job['stored'].checkpoint.payload),
                      ('occupancy.bin',b'\x64\xff'),('cloud.xyzf32',struct.pack('<fff',*([.0625]*3)))]:
        (folder/name).write_bytes(data)
    job.update(state='ready',report={'source_fine_sha256':hashlib.sha256((job['folder']/'fine.source').read_bytes()).hexdigest(),
        'source_dense_sha256':hashlib.sha256((job['folder']/'dense.source').read_bytes()).hexdigest(),
        'preview':{'candidate_points':0,'removed_keys':[]},'offline_complete':False})
    return service,job


def test_confirmed_save_copy_reloads_all_members_and_preserves_original(tmp_path):
    owner=fixture(tmp_path);service,job=prepared(owner)
    original=owner._asset_repository.current_directory('original')
    before={p.relative_to(original).as_posix():p.read_bytes() for p in original.rglob('*') if p.is_file()}
    result=service.apply(job['id'],new_map_name='cleaned',confirmed=True)
    assert result['state']=='applied' and result['result_map']=='cleaned'
    assert {p.relative_to(original).as_posix():p.read_bytes() for p in original.rglob('*') if p.is_file()}==before
    bundle=owner.get_map_asset_bundle(map_name='cleaned')
    reference=bundle.geometry_asset.metadata['mapping_backend_checkpoint']
    stored=MappingBackendCheckpointStore(owner._asset_repository.current_directory('cleaned')).load('voxel',schema_version=2,
        expected_checkpoint_id=reference['checkpoint_id'])
    assert stored.checkpoint.fine_archive==fine_bytes(8)
    assert stored.checkpoint.dense_archive==dense_bytes(8)
    assert stored.checkpoint.cloud_receipt['cloud_revision']==8
    assert stored.checkpoint.canonical_cloud_xyz==struct.pack('<fff',*([.0625]*3))
    assert stored.checkpoint.payload==job['stored'].checkpoint.payload
    assert owner._repository.get_active_map_name() is None
    service.executor.shutdown(wait=True)


@pytest.mark.parametrize('confirmed,name',[(False,'cleaned'),(True,'original')])
def test_confirmation_and_new_name_are_required(tmp_path,confirmed,name):
    owner=fixture(tmp_path);service,job=prepared(owner)
    with pytest.raises(Exception):service.apply(job['id'],new_map_name=name,confirmed=confirmed)
    assert owner._repository.get_map_asset('cleaned') is None
    service.executor.shutdown(wait=True)


def test_cancel_never_writes_source_or_publishes_map(tmp_path):
    owner=fixture(tmp_path);service,job=prepared(owner)
    source=job['folder']/'fine.source';before=source.read_bytes()
    assert service.cancel(job['id'])['state']=='cancelled'
    with pytest.raises(Exception):service.apply(job['id'],new_map_name='cleaned',confirmed=True)
    assert source.read_bytes()==before and owner._repository.get_map_asset('cleaned') is None
    service.executor.shutdown(wait=True)


def test_raw_scan_capture_stops_at_saved_timestamp_and_keeps_original_indexes(tmp_path):
    source=tmp_path/'logs';source.mkdir();target=tmp_path/'saved';target.mkdir()
    checkpoint=SimpleNamespace(source_session_id='test',source_session_generation=3,cloud_receipt={'source_timestamp_ns':20,'cloud_revision':2})
    stem=hashlib.sha256(b'test:3').hexdigest()
    def header(value):
        b=json.dumps(value).encode();return struct.pack('<I',len(b))+b
    initial=b'NEHSCAN1'+header({'session_id':'test','generation':3})
    frames=[]
    for stamp in [10,20,30]:
        frames.append(header({'source_timestamp_ns':stamp,'point_count':2})+struct.pack('<dddddd',1,2,3,1,2,3)+b'\x04\x06')
    (source/(stem+'.scan')).write_bytes(initial+b''.join(frames))
    with patch.dict(os.environ,NUWAX_RAW_SCAN_DIRECTORY=str(source)):
        report=copy_raw_scan_history(checkpoint,target)
    assert report['complete'] and report['frames']==2
    assert (target/'raw-scans.scan').read_bytes()==initial+b''.join(frames[:2])


def native_fixture(root, case):
    """Publish exact assets emitted by the native cumulative-map acceptance run."""
    owner=Owner(root);name='original';checkpoint_id='e'*32
    with (case/'original-fine.bin').open('rb') as stream:
        stream.read(8);native_root=json.loads(stream.read(struct.unpack('<Q',stream.read(8))[0]))
    session=native_root['domain']['session'];generation=native_root['domain']['generation'];revision=native_root['revision'];stamp=native_root['source_timestamp_ns']
    descriptor=json.loads((case/'occupancy-descriptor.json').read_text())
    fine=(case/'original-fine.bin').read_bytes();dense=(case/'original-dense.bin').read_bytes();xyz=(case/'original-cloud.xyzf32').read_bytes();backend=(case/'original-backend.bin').read_bytes();occ=(case/'original-occupancy.bin').read_bytes()
    receipt=dict(commit_id=dict(boot_nonce=(133).to_bytes(16,'little'),sequence=190),session_id=session,session_generation=generation,occupancy_revision=190,source_timestamp_ns=stamp,backend_kind='voxel',backend_generation=1)
    cloud_receipt=dict(commit_id=receipt['commit_id'],session_generation=generation,occupancy_revision=190,cloud_revision=revision,source_timestamp_ns=stamp,point_count=len(xyz)//12)
    def member(role,producer_generation=0):return CheckpointMemberIdentity(checkpoint_id,role,receipt,cloud_receipt,'map',producer_generation=producer_generation)
    checkpoint=MappingBackendCheckpoint(2,'voxel',session,generation,b'',len(backend),len(xyz),backend,receipt,cloud_receipt,checkpoint_id=checkpoint_id,
        canonical_occupancy_data=occ,canonical_cloud_xyz=xyz,backend_identity=member('backend_state'),occupancy_identity=member('canonical_occupancy'),cloud_identity=member('canonical_cloud'),
        fine_archive=fine,fine_reference=CheckpointArtifactReference('f'*32,len(fine),2,member('fine_map',native_root['domain']['producer_generation'])),
        dense_archive=dense,dense_reference=CheckpointArtifactReference('1'*32,len(dense),2,member('dense_cloud')))
    occupancy=dict(descriptor,raw_data_size=len(occ),source_occupancy_revision=190)
    cloud=dict(xyz_size=len(xyz),frame_id='map',source_cloud_revision=revision,point_count=len(xyz)//12,voxel_size=native_root['voxel_size_m'],payload_schema_version=2)
    staging=owner._asset_repository.create_staging_directory(name)
    stored=MappingBackendCheckpointStore(staging).publish(checkpoint,occupancy_asset=occupancy,cloud_asset=cloud)
    grid=OccupancyGrid(map_id=name,frame_id='map',width=descriptor['width'],height=descriptor['height'],resolution_m=descriptor['resolution'],origin=Pose(frame_id='map',position={'x':descriptor['origin_x'],'y':descriptor['origin_y'],'z':0.}),data=[v if v<128 else v-256 for v in occ])
    snapshot=MapSnapshot(map_name=name,source_name='native-cumulative-fixture',source_epoch=session,update_sequence=190,frame_id='map',occupancy_grid=grid)
    paths={'db':'geometry/test/mapping/original.db','pcd':'geometry/test/pcd/cloud.pcd','nav':'geometry/test/navigation/map.yaml'}
    geometry=MapGeometryAssetBinding(map_name=name,geometry_stack='test',frame_id='map',mapping_database_path=paths['db'],pcd_path=paths['pcd'],occupancy_projection='canonical',costmap_source='canonical',
        assets=[MapGeometryAssetRef(asset_type=MapGeometryAssetType.MAPPING_DATABASE,path=paths['db'],frame_id='map'),MapGeometryAssetRef(asset_type=MapGeometryAssetType.PCD,path=paths['pcd'],frame_id='map'),MapGeometryAssetRef(asset_type=MapGeometryAssetType.NAVIGATION_MAP,path=paths['nav'],frame_id='map')],
        metadata={'geometry_asset_path':'geometry/test/geometry_asset.json','mapping_backend_checkpoint':dict(present=True,schema_version=2,backend_kind='voxel',checkpoint_id=checkpoint_id,manifest_path=stored.manifest_path.relative_to(staging).as_posix())})
    identity=AssetIdentityV2(asset_id='2'*32)
    bundle=MapAssetBundle(map_name=name,source_name='native-cumulative-fixture',frame_id='map',snapshot=snapshot,geometry_asset=geometry,identity_schema_version=2,asset_identity=identity,asset_state='candidate',
        metadata={'raw_scan_history':{'path':'raw-scans.scan','complete':True,'frames':190,'last_timestamp_ns':stamp}})
    import shutil
    shutil.copyfile(case/'raw-scans.scan',staging/'raw-scans.scan')
    pcd=staging/paths['pcd'];pcd.parent.mkdir(parents=True,exist_ok=True);pcd.write_bytes(('VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n'%(len(xyz)//12,len(xyz)//12)).encode()+xyz)
    from services.geometry.geometry_runtime_provider import MappingBackendCheckpointExportBundle
    owner._geometry_runtime._materialize_native_bundle_database(staging/paths['db'],bundle=bundle,checkpoint_bundle=MappingBackendCheckpointExportBundle(checkpoint,occupancy,cloud),mapping_context={'session_id':session,'session_generation':generation})
    owner._geometry_runtime._materialize_nav2_map_assets(bundle,bundle_root=staging)
    owner._asset_repository.finalize_assigned_map_asset_bundle(bundle,staging_dir=staging,asset_identity=identity)
    owner._repository.save_map_asset(MapAsset(map_name=name,display_name=name,bound_memory_library_name=name,identity_schema_version=2,asset_identity=identity,finalized=True))
    return owner


@pytest.mark.skipif(not os.environ.get('NEH_NATIVE_SAVED_CASE'),reason='requires exact native cumulative assets, not fabricated fixture data')
def test_native_cumulative_preview_save_reload_and_cancel(tmp_path):
    import shutil
    case=Path(os.environ['NEH_NATIVE_SAVED_CASE']);expected_result=Path(os.environ.get('NEH_NATIVE_EXPECTED_RESULT',str(case/'cleaned')));owner=native_fixture(tmp_path,case);service=CompletedMapCleanup(owner)
    original=owner._asset_repository.current_directory('original')
    def hashes(folder):
        result={}
        for path in folder.rglob('*'):
            if path.is_file():
                with path.open('rb') as stream:
                    digest=hashlib.sha256()
                    for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
                result[path.relative_to(folder).as_posix()]=digest.hexdigest()
        return result
    before=hashes(original)
    if os.environ.get('NEH_CLEANUP_BINARY'):
        import time
        owner._geometry_runtime.config.runtime_binary=os.environ['NEH_CLEANUP_BINARY']
        status=service.start(map_name='original',strength_percent=50)
        while status['state'] in {'queued','running'}:
            time.sleep(.1);status=service.status(status['id'])
        assert status['state']=='ready',status
        job=service.jobs[status['id']];result=job['folder']/'result'
        report=json.loads((result/'preview.json').read_text())
        expected=json.loads((expected_result/'preview.json').read_text())
        assert report['preview']['removed_keys']==expected['preview']['removed_keys']
        assert report['dense_after']==expected['dense_after']
    else:
        with patch.object(service.executor,'submit'):status=service.start(map_name='original',strength_percent=50)
        job=service.jobs[status['id']];result=job['folder']/'result';shutil.copytree(expected_result,result)
        report=json.loads((result/'preview.json').read_text());job.update(state='ready',report=report)
    saved=service.apply(job['id'],new_map_name='cleaned',confirmed=True)
    assert saved['state']=='applied' and hashes(original)==before
    bundle=owner.get_map_asset_bundle(map_name='cleaned');reference=bundle.geometry_asset.metadata['mapping_backend_checkpoint']
    reopened=MappingBackendCheckpointStore(owner._asset_repository.current_directory('cleaned')).load('voxel',schema_version=2,expected_checkpoint_id=reference['checkpoint_id']).checkpoint
    for actual,filename in [(reopened.fine_archive,'fine.bin'),(reopened.dense_archive,'dense.bin'),(reopened.payload,'backend.bin'),(reopened.canonical_cloud_xyz,'cloud.xyzf32'),(reopened.canonical_occupancy_data,'occupancy.bin')]:assert actual==(result/filename).read_bytes()
    assert reopened.cloud_receipt['point_count']==len(reopened.canonical_cloud_xyz)//12
    assert job['stored'].checkpoint.cloud_receipt['point_count']-reopened.cloud_receipt['point_count']==report['preview']['candidate_points']-report['recovered_voxels']
    if os.environ.get('NEH_CLEANUP_BINARY'):
        cancelled=service.start(map_name='original',strength_percent=50)
        while cancelled['state']=='queued':
            time.sleep(.01);cancelled=service.status(cancelled['id'])
        service.cancel(cancelled['id'])
        while service.status(cancelled['id'])['state'] in {'queued','running'}:time.sleep(.01)
        assert service.status(cancelled['id'])['state']=='cancelled'
    else:
        with patch.object(service.executor,'submit'):cancelled=service.start(map_name='original',strength_percent=50)
        service.cancel(cancelled['id'])
    assert hashes(original)==before
    assert owner._repository.get_active_map_name() is None
    service.executor.shutdown(wait=True)


def test_changed_source_requires_new_preview(tmp_path):
    owner=fixture(tmp_path);service,job=prepared(owner)
    original=owner.get_map_asset_bundle(map_name='original')
    changed=SimpleNamespace(asset_identity=original.asset_identity,manifest_sha256='9'*64)
    with patch.object(owner,'get_map_asset_bundle',return_value=changed):
        with pytest.raises(Exception,match='原地图已改变'):
            service.apply(job['id'],new_map_name='cleaned',confirmed=True)
    assert owner._repository.get_map_asset('cleaned') is None
    service.executor.shutdown(wait=True)


def test_live_mapping_must_finish_before_preview(tmp_path):
    owner=fixture(tmp_path);service=CompletedMapCleanup(owner)
    owner._geometry_runtime.get_mapping_session_context=lambda: {'active':True,'map_name':'original'}
    with pytest.raises(Exception,match='请先停止建图'):
        service.start(map_name='original',strength_percent=50)
    assert not service.jobs
    service.executor.shutdown(wait=True)
