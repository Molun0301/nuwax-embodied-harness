"""真实mmap采集与分页网页字节协议的端到端离线检查。"""
import base64,json,os,struct,sys,tempfile,unittest
from dataclasses import replace
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
class ActualPipeline(unittest.TestCase):
    def test_real_mmap_source_to_pinned_ws_page_protocol(self):
        from scripts import check_pointcloud_incremental_pages as fixture
        cloud=fixture.cloud
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
