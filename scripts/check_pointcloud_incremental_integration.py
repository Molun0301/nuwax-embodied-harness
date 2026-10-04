"""校验实际网关发布/兼容边界，保持检查独立于原生编译和设备。"""
from __future__ import annotations

import base64
import importlib.util
import json
import os
import struct
import sys
import tempfile
import threading
import unittest
from dataclasses import replace
from pathlib import Path
from types import MethodType
from types import SimpleNamespace as NS
from unittest.mock import patch

STAGE=Path(__file__).resolve().parents[1]
REPO=Path(sys.argv.pop(1)) if len(sys.argv)>1 and not sys.argv[1].startswith('-') else Path.cwd()
sys.path.insert(0,str(REPO))
import gateways.ws

# Only this process changes package lookup; no live checkout edits.
gateways.ws.__path__.insert(0,str(STAGE/'gateways/ws'))
ws_spec=importlib.util.spec_from_file_location('gateways.ws.data_streams',STAGE/'gateways/ws/data_streams.py')
ws=importlib.util.module_from_spec(ws_spec);sys.modules[ws_spec.name]=ws;ws_spec.loader.exec_module(ws)
gateways.ws.data_streams=ws
assert Path(ws.__file__).resolve()==(STAGE/'gateways/ws/data_streams.py').resolve()
from contracts.streams import StreamFrame, StreamPayloadKind

spec=importlib.util.spec_from_file_location('staged_runtime',STAGE/'gateways/runtime.py')
runtime=importlib.util.module_from_spec(spec);sys.modules[spec.name]=runtime;spec.loader.exec_module(runtime)


def snapshot(source='live',chunked=True):
    raw=struct.pack('<fff',1.,2.,3.)
    return NS(source=source,source_epoch='epoch',revision='cloud:7',revision_ordinal=7,
              frame_id='map',source_timestamp_ns=8,point_count=1,format='float32-xyz-le',
              stream_generation=3,provenance={},chunked=chunked,layout_generation=2,
              chunk_size_points=1024 if chunked else 0,
              chunks=(NS(xyz_bytes=raw),),xyz_bytes=b'' if chunked else raw)

class Sink:
    def __init__(self):self.frames=[]
    def publish(self,topic,**kwargs):self.frames.append((topic,kwargs))

class Publication(unittest.TestCase):
    def fake(self):
        owner=NS(_pointcloud_map_stream_stop=threading.Event(),_lock=threading.RLock(),stream_bus=Sink())
        owner._publish_pointcloud_map_compatibility=MethodType(runtime.GatewayRuntime._publish_pointcloud_map_compatibility,owner)
        return owner
    def test_native_publication_has_no_full_bytes_or_compat_payload(self):
        owner=self.fake();source=snapshot()
        with patch.object(base64,'b64encode',side_effect=AssertionError('no eager base64')):
            runtime.GatewayRuntime._publish_pointcloud_map_snapshot(owner,source,reason='source_update')
        self.assertEqual(len(owner.stream_bus.frames),2)
        topic,frame=owner.stream_bus.frames[0]
        self.assertEqual(topic,'pointcloud_map.snapshot');self.assertEqual(frame['payload_kind'],StreamPayloadKind.JSON)
        self.assertEqual(frame['payload'],{'chunked':True,'point_count':1})
        self.assertTrue(owner.stream_bus.frames[1][1]['metadata']['legacy_pointcloud_topic'])
    def test_saved_heartbeat_does_not_make_base64(self):
        owner=self.fake();source=snapshot('saved',False)
        with patch.object(base64,'b64encode',side_effect=AssertionError('no eager base64')):
            runtime.GatewayRuntime._publish_pointcloud_map_snapshot(owner,source,reason='heartbeat')
        self.assertIs(owner.stream_bus.frames[0][1]['payload'],source.xyz_bytes)
        self.assertEqual(owner.stream_bus.frames[-1][1]['payload'],{'legacy_pointcloud_topic':True})
    def test_getter_returns_same_current_snapshot_without_materialization(self):
        source=snapshot();owner=NS(_lock=threading.RLock(),_pointcloud_map_current_snapshot=source)
        self.assertIs(runtime.GatewayRuntime.get_pointcloud_map_snapshot(owner),source)

class Compatibility(unittest.IsolatedAsyncioTestCase):
    async def test_real_ws_legacy_native_materializes_only_on_send(self):
        source=snapshot();owner=NS(get_pointcloud_map_snapshot=lambda:source)
        frame=StreamFrame(topic='pointcloud_map.snapshot',source='probe',cursor=1,payload={'chunked':True},metadata={'chunked':True})
        class Socket:
            async def send_bytes(self,value):self.data=value
            async def send_text(self,value):self.data=value
        socket=Socket();await ws._send_ws_frame(socket,frame,runtime=owner)
        _magic,version,_size=struct.unpack_from('<8sBI',socket.data)
        self.assertEqual(version,1);self.assertEqual(socket.data[-12:],source.chunks[0].xyz_bytes)
    async def test_real_sse_explicit_legacy_saved_has_payload(self):
        source=snapshot('saved',False);owner=NS(get_pointcloud_map_snapshot=lambda:source)
        frame=StreamFrame(topic='pointcloud_map.chunk',source='probe',cursor=1,payload={'legacy_pointcloud_topic':True},metadata={'legacy_pointcloud_topic':True})
        with patch.object(ws.base64,'b64encode',wraps=ws.base64.b64encode) as encode:
            text=await ws._format_sse_frame(frame,runtime=owner)
        self.assertEqual(encode.call_count,1);self.assertIn('"encoding": "base64"',text)
        self.assertIn('AACAPwAAAEAAAEBA',text)


class ActualPipeline(unittest.TestCase):
    def test_real_mmap_source_to_pinned_ws_page_protocol(self):
        checkpoint=Path(os.environ.get('NEH_PIPELINE_CHECKPOINT',str(STAGE if (STAGE/'services/geometry/pointcloud_map.py').exists() else STAGE.parent/'checkpoint')))
        source_path=checkpoint/'services/geometry/pointcloud_map.py'
        old=os.environ.get('NEH_POINTCLOUD_MODULE_PATH')
        os.environ['NEH_POINTCLOUD_MODULE_PATH']=str(source_path)
        fixture_spec=importlib.util.spec_from_file_location('pipeline_mmap_fixture',checkpoint/'scripts/check_pointcloud_incremental_pages.py')
        fixture=importlib.util.module_from_spec(fixture_spec);sys.modules[fixture_spec.name]=fixture
        try: fixture_spec.loader.exec_module(fixture)
        finally:
            if old is None: os.environ.pop('NEH_POINTCLOUD_MODULE_PATH',None)
            else: os.environ['NEH_POINTCLOUD_MODULE_PATH']=old
        cloud=fixture.cloud
        self.assertEqual(Path(cloud.__file__).resolve(),source_path.resolve())
        import numpy as np

        from gateways.ws.pointcloud_stream import PointCloudTransfer, snapshot_identity
        with tempfile.TemporaryDirectory() as directory:
            producer=fixture.Producer(Path(directory)/'cloud.mmap',capacity=16384)
            reader=cloud.CloudMapMmapReader(str(producer.path))
            source=cloud.LivePointCloudMapSource(reader)
            records=[];accepted=None
            try:
                points=np.arange(5000*3,dtype='<f4').reshape(5000,3)
                for revision,dirty,action in [(10,None,'baseline'),(20,[1],'edit'),(30,[0,4],'delete'),(40,[4],'regrow')]:
                    if action=='edit': points[1100]=[1.5,2.5,3.5]
                    if action=='delete': points[2]=points[-1];points=points[:-1]
                    if action=='regrow': points=np.concatenate([points,np.array([[7,8,9]],dtype='<f4')])
                    producer.publish(revision,points,dirty=dirty,session='mapping-actual',generation=7)
                    current=replace(source.read(),stream_generation=1)
                    self.assertIsInstance(current,cloud.PointCloudMapSnapshot)
                    self.assertTrue(current.chunked);self.assertEqual(current.xyz_bytes,b'')
                    self.assertEqual(current.provenance['mapping_source_epoch'],'mapping-actual:7')
                    if accepted is not None and action=='edit': self.assertIs(current.chunks[0],accepted.chunks[0])
                    transfer=PointCloudTransfer(current,None if accepted is None else snapshot_identity(accepted),len(records)+1)
                    packets=[]
                    while not transfer.complete: packets.append(base64.b64encode(transfer.next_packet()).decode())
                    if action=='edit':self.assertEqual(transfer.encoded_xyz_bytes,1024*12)
                    if action=='delete':self.assertEqual(len(transfer.chunks),2)
                    records.append({'action':action,'packets':packets,'point_count':len(points),'expected_xyz':points.ravel().tolist()})
                    accepted=current
                output=Path(os.environ.get('NEH_POINTCLOUD_FIXTURE_DIR',str(Path(tempfile.gettempdir())/'neh-pointcloud-fixtures')))
                output.mkdir(parents=True,exist_ok=True)
                (output/'pointcloud_native_wire_fixtures.json').write_text(json.dumps(records,separators=(',',':')))
            finally:
                source.close();reader.close();producer.close()

if __name__=='__main__':unittest.main()
