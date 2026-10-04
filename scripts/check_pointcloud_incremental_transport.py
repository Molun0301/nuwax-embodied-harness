"""离线验证分页 WS 编码、精确基线、慢客户端和不可变事务。"""
import asyncio
import importlib.util
import json
import os
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('cloud_pages_under_test', ROOT / 'gateways/ws/pointcloud_stream.py')
cloud = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = cloud
spec.loader.exec_module(cloud)


def snapshot(*, revision=20, total=1024*12, changed=(), epoch='producer-a', layout=1, generation=1):
    chunks = tuple(NS(chunk_index=i, revision_ordinal=revision if i in changed else 1,
                      point_count=min(1024,total-i*1024), xyz_bytes=struct.pack('<fff',float(i),2.,3.)*min(1024,total-i*1024))
                   for i in range((total+1023)//1024))
    return NS(source='live',source_epoch=epoch,stream_generation=generation,layout_generation=layout,
              frame_id='map',revision_ordinal=revision,revision=f'cloud:{revision}',source_timestamp_ns=2**53+1,
              point_count=total,chunk_size_points=1024,chunked=True,chunks=chunks,provenance={'mapping_source_epoch':'session:1'})


def decode(packet):
    magic,version,size=struct.unpack_from('<8sBI',packet)
    return magic,version,json.loads(packet[13:13+size]),packet[13+size:]


class IncrementalTransport(unittest.TestCase):
    def test_bootstrap_bounded_pinned_and_ack(self):
        source=snapshot(revision=2**53+7,total=1024*10+5)
        transfer=cloud.PointCloudTransfer(source,None,1)
        packets=[]
        while not transfer.complete:
            packet=transfer.next_packet(); packets.append(packet)
            self.assertLess(len(packet),64*1024)
            _magic,version,header,payload=decode(packet)
            self.assertEqual(version,2)
            self.assertEqual((len(packet)-len(payload))%4,0)
            self.assertEqual(header['revision_ordinal'],str(source.revision_ordinal))
            self.assertEqual(header['mapping_source_epoch'],'session:1')
            self.assertEqual(header['source_timestamp_ns'],str(2**53+1))
        self.assertEqual(len(packets),3)
        self.assertEqual(transfer.encoded_xyz_bytes,source.point_count*12)
        self.assertTrue(transfer.accepts_ack({'type':'pointcloud.ack','transfer_id':'1','base':cloud.snapshot_identity(source)}))
        self.assertFalse(transfer.accepts_ack({'type':'pointcloud.ack','transfer_id':'0','base':cloud.snapshot_identity(source)}))

    def test_lag_only_changed_pages_no_full_join(self):
        source=snapshot(revision=10_000,total=1024*1000,changed=(4,800))
        base={**cloud.snapshot_identity(source),'revision_ordinal':'500'}
        transfer=cloud.PointCloudTransfer(source,base,2)
        self.assertEqual([c.chunk_index for c in transfer.chunks],[4,800])
        self.assertIs(transfer.chunks[0].xyz_bytes,source.chunks[4].xyz_bytes)
        _,_,header,payload=decode(transfer.next_packet())
        self.assertFalse(header['reset']); self.assertEqual(header['base_revision_ordinal'],'500')
        self.assertEqual(len(payload),2*1024*12)

    def test_epoch_layout_restart_and_zero_reset(self):
        source=snapshot(total=0)
        for field in ('source_epoch','layout_generation','stream_generation','frame_id'):
            base=cloud.snapshot_identity(source); base[field]='other' if field in ('source_epoch','frame_id') else '2'
            transfer=cloud.PointCloudTransfer(source,base,1)
            _,_,header,payload=decode(transfer.next_packet())
            self.assertTrue(header['reset']); self.assertEqual(header['packet_count'],1); self.assertEqual(payload,b'')

    def test_truncate_without_dirty_page_payload(self):
        source=snapshot(total=1024*8,revision=20)
        transfer=cloud.PointCloudTransfer(source,{**cloud.snapshot_identity(source),'revision_ordinal':'10'},1)
        _,_,header,payload=decode(transfer.next_packet())
        self.assertEqual(header['changed_chunk_count'],0); self.assertEqual(header['point_count'],1024*8); self.assertEqual(payload,b'')

    def test_exact_u64_rejects_float_bool_unicode_and_overflow(self):
        for value in (1.9,True,'01','١',-1,2**64,None):
            with self.assertRaises((TypeError,ValueError)):
                cloud._u64(value)
        self.assertEqual(cloud._u64(2**64-1),str(2**64-1))

    def test_legacy_segmented_not_claimed_incremental(self):
        source=snapshot(total=1025); source.chunked=False; source.chunk_size_points=0
        source.xyz_bytes=b''.join(c.xyz_bytes for c in source.chunks)
        transfer=cloud.PointCloudTransfer(source,None,1)
        _,_,header,payload=decode(transfer.next_packet())
        self.assertFalse(header['source_incremental']); self.assertEqual(len(payload),1025*12)

    def test_fixture(self):
        directory=Path(os.environ.get('NEH_POINTCLOUD_FIXTURE_DIR',str(Path(tempfile.gettempdir())/'neh-pointcloud-fixtures')))
        directory.mkdir(parents=True,exist_ok=True)
        fixtures=directory/'pointcloud_incremental_fixtures.json'
        if '--fixtures' not in sys.argv: return
        source=snapshot(revision=2**53+7,total=1024*5+3)
        baseline=cloud.PointCloudTransfer(source,None,1)
        import base64
        packets=[]
        while not baseline.complete: packets.append(base64.b64encode(baseline.next_packet()).decode())
        newer=snapshot(revision=source.revision_ordinal+1,total=1024*5+3,changed=(2,))
        delta=cloud.PointCloudTransfer(newer,cloud.snapshot_identity(source),2)
        fixtures.write_text(json.dumps({'baseline':packets,'delta':[base64.b64encode(delta.next_packet()).decode()]}))


class Router:
    def websocket(self, _path):
        def register(endpoint):
            self.endpoint = endpoint
            return endpoint
        return register


class Socket:
    def __init__(self):
        self.incoming = asyncio.Queue()
        self.sent = asyncio.Queue()
    async def accept(self): pass
    async def send_json(self, value): await self.sent.put(value)
    async def send_bytes(self, value): await self.sent.put(value)
    async def receive(self): return await self.incoming.get()
    async def close(self, **kwargs): await self.sent.put({'closed': kwargs})
    async def control(self, value): await self.incoming.put({'type':'websocket.receive', 'text':json.dumps(value)})


class StreamBus:
    def __init__(self): self.callback = None
    def subscribe(self, callback, **kwargs): self.callback=callback; return 'one'
    def unsubscribe(self, subscription): self.callback=None
    def notify(self):
        if self.callback: self.callback(None)


class SocketLifecycle(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.bus=StreamBus();self.current=snapshot(total=1024*5)
        self.runtime=NS(stream_bus=self.bus,get_pointcloud_map_snapshot=lambda:self.current)
        self.router=Router();cloud.install_pointcloud_stream(self.router,self.runtime)
        self.socket=Socket();self.task=asyncio.create_task(self.router.endpoint(self.socket))
        self.assertEqual(await self.socket.sent.get(),{'type':'pointcloud.ready','version':2})
    async def asyncTearDown(self):
        await self.socket.incoming.put({'type':'websocket.disconnect'})
        await asyncio.wait_for(self.task,1)
        self.assertIsNone(self.bus.callback)
    async def transaction(self):
        packets=[]
        while True:
            packet=await asyncio.wait_for(self.socket.sent.get(),1)
            self.assertIsInstance(packet,bytes)
            header=decode(packet)[2];packets.append(header)
            if header['packet_index']==header['packet_count']-1: return packets
    async def ack(self, packets):
        header=packets[-1]
        await self.socket.control({'type':'pointcloud.ack','transfer_id':header['transfer_id'],
                                   'base':{key:header[key] for key in (*cloud._IDENTITY_FIELDS,'revision_ordinal')}})
    async def test_slow_ack_coalesces_to_latest_and_does_not_send_unbounded(self):
        await self.socket.control({'type':'pointcloud.resume','base':None})
        bootstrap=await self.transaction()
        for rev in range(21,31):
            self.current=snapshot(revision=rev,total=1024*5,changed=(2,));self.bus.notify()
        await asyncio.sleep(.02)
        self.assertTrue(self.socket.sent.empty())
        await self.ack(bootstrap)
        update=await self.transaction()
        self.assertEqual(len(update),1);self.assertEqual(update[0]['revision_ordinal'],'30')
        self.assertEqual(update[0]['base_revision_ordinal'],'20')
        self.assertEqual([x['chunk_index'] for x in update[0]['chunks']],[2])
    async def test_map_fence_waits_without_repeat_bootstrap_and_ignores_old_ack(self):
        await self.socket.control({'type':'pointcloud.resume','base':None})
        first=await self.transaction()
        await self.socket.control({'type':'pointcloud.map','mapping_source_epoch':'session:2'})
        await self.ack(first)
        await asyncio.sleep(.02);self.assertTrue(self.socket.sent.empty())
        self.current=snapshot(epoch='producer-b');self.current.provenance['mapping_source_epoch']='session:2'
        self.bus.notify();newer=await self.transaction()
        self.assertTrue(newer[0]['reset']);self.assertEqual(newer[0]['mapping_source_epoch'],'session:2')
    async def test_reconnect_with_current_base_sends_no_snapshot(self):
        await self.socket.control({'type':'pointcloud.resume','base':cloud.snapshot_identity(self.current)})
        await asyncio.sleep(.02);self.assertTrue(self.socket.sent.empty())
        self.current=snapshot(revision=21,total=1024*5,changed=(1,));self.bus.notify()
        delta=await self.transaction();self.assertFalse(delta[0]['reset']);self.assertEqual(delta[0]['changed_chunk_count'],1)


if __name__=='__main__':
    fixtures='--fixtures' in sys.argv
    if fixtures:
        # unittest does not recognize our generation option.
        case=IncrementalTransport('test_fixture'); case.test_fixture(); sys.argv.remove('--fixtures')
    unittest.main()
