"""Block evidence equivalence against the frozen pre-vectorization implementation."""
import json, random, struct, unittest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from unittest.mock import patch
from services.geometry import fine_map_archive as fine_module
from services.geometry.fine_map_archive import validate_fine_archive as candidate
from services.geometry.checkpoint_payload import FrozenPayload
from tests.fine_scalar_oracle import validate_fine_archive as oracle

U64=2**64-1
ERROR='精细地图块观察证据错误'
def archive(records, legacy=False, source=U64):
    ref=dict(key=[0,0,0],points=len(records))
    if legacy: ref.update(sha256=[1]*32,geometry_sha256=[2]*32)
    else: ref.update(object=dict(kind='assigned',value=[1]*16),block_revision=1,geometry_revision=1)
    root=dict(version=1 if legacy else 2,domain=dict(session='s',generation=1,frame_generation=0,producer_generation=1),revision=1,source_timestamp_ns=source,voxel_size_m=.05,blocks=[ref])
    if not legacy:root['geometry_revision']=1
    raw=json.dumps(root,separators=(',',':')).encode()
    block=b'NUWAXBL1'+struct.pack('<II',len(records),0)+b''.join(struct.pack('<HQH',*r) for r in records)
    return (b'NUWAXAR1' if legacy else b'NUWAXAR2')+struct.pack('<Q',len(raw))+(b'X'*32 if legacy else b'')+raw+struct.pack('<QI',1,len(block))+block

class Borrowed(FrozenPayload):
    """Same bounded slicing contract as the ordinary repository's borrowed view."""
    def __init__(self, value):self.value=value;self.slices=[]
    def __len__(self):return len(self.value)
    def __getitem__(self,key):self.slices.append(key);return self.value[key]
    def view(self,*a,**k):raise AssertionError('borrowed views have no mmap')

def outcome(fn,payload,**kw):
    try:return ('ok',fn(payload,**kw))
    except Exception as e:return (type(e).__name__,str(e))

class FineVectorTests(unittest.TestCase):
    def check(self,raw,error=None,**kw):
        expected=outcome(oracle,raw,**kw)
        if error is not None:self.assertEqual(expected,('ValueError',error))
        for payload in ([raw,Borrowed(raw)] + ([FrozenPayload([raw])] if raw else [])):
            self.assertEqual(outcome(candidate,payload,**kw),expected)
    def test_packed_dtype_offsets_and_itemsize(self):
        from services.geometry.fine_map_archive import _FINE_RECORD_DTYPE as dtype
        self.assertEqual(dtype.itemsize,12)
        self.assertEqual([dtype.fields[name][1] for name in ('local','stamp','hits')],[0,2,10])
        self.assertFalse(dtype.isalignedstruct)
    def test_valid_limits_both_versions_and_payload_forms(self):
        for old in (False,True):
            for records in ([(0,0,0)],[(32767,U64,65535)],[(i,U64,1) for i in range(32768)]):
                self.check(archive(records,old))
    def test_invalid_evidence_first_last_and_unsigned_order(self):
        for old in (False,True):
            for records in ([(32768,1,1)],[(65535,1,1)],[(1,1,1),(0,1,1)],[(0,1,1),(0,1,1)],[(0,0,1)],[(0,1,0)],[(0,U64,1)]):
                self.check(archive(records,old,source=U64-1),ERROR)
            for i in (0,32767):
                records=[(v,1,1) for v in range(32768)];records[i]=(i,0,1)
                self.check(archive(records,old),ERROR)
    def test_stamp_limits_and_zero_pairs(self):
        for source in (1,2**63-1,2**63,U64-1,U64):
            for stamp in (0,1,2**63-1,2**63,U64-1,U64):
                for hits in (0,1,65535):
                    for n in (1,512):
                        records=[(i,1,1) for i in range(n)]
                        records[-1]=(n-1,stamp,hits)
                        self.check(archive(records,source=source))
    def test_truncation_format_and_error_priority(self):
        for old in (False,True):
            raw=archive([(0,0,1)],old)
            for stop in (0,7,15,23,len(raw)-13,len(raw)-1):self.check(raw[:stop])
            self.check(raw+b'x',ERROR) # evidence precedes trailing bytes
            self.check(raw, '精细地图资产与原占据/点云提交不一致', session='other')
            p=raw.rfind(b'NUWAXBL1');bad=bytearray(raw);bad[p]=0
            self.check(bytes(bad),'精细地图块格式错误')
            bad=bytearray(raw);bad[p-4:p]=struct.pack('<I',len(raw)-p-1)
            self.check(bytes(bad),'精细地图块长度错误')
    def test_implementation_boundary_routes_both_paths(self):
        with patch.object(fine_module,'_validate_vector_block_evidence',wraps=fine_module._validate_vector_block_evidence) as vector:
            for n,expected_calls in ((1,0),(511,0),(512,1),(513,2),(32768,3)):
                candidate(archive([(i,1,1) for i in range(n)]))
                self.assertEqual(vector.call_count,expected_calls)
    def test_invalid_evidence_both_paths_and_boundaries(self):
        for old in (False,True):
            for n in (8,511,512,1024):
                for index in (0,n-1):
                    for local,stamp,hits in ((32768,1,1),(65535,1,1),(index,0,1),(index,1,0),(index,U64,1)):
                        records=[(i,1,1) for i in range(n)];records[index]=(local,stamp,hits)
                        self.check(archive(records,old,source=U64-1),ERROR)
                for local in (0,1):
                    records=[(i,1,1) for i in range(n)];records[2]=(local,1,1)
                    self.check(archive(records,old),ERROR)
    def test_bounded_random_differential(self):
        rng=random.Random(670806525)
        for trial in range(400):
            n=rng.randint(1,511) if trial%2==0 else rng.randint(512,2048)
            source=rng.choice((1,2**63-1,2**63,U64))
            records=[(v,rng.choice((0,1,source)),1) for v in sorted(rng.sample(range(32768),n))]
            records=[(l,s,0 if s==0 else h) for l,s,h in records]
            if rng.randrange(4):
                i=rng.randrange(n);records[i]=(rng.choice((0,32767,32768,65535,records[i][0])),rng.choice((0,1,source,U64)),rng.choice((0,1,65535)))
            self.check(archive(records,bool(rng.randrange(2)),source))

if __name__=='__main__':unittest.main()
