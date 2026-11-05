"""当前制品元数据与文件定位的离线耗时测量；不包含网络传输。"""
from pathlib import Path
import argparse
import json
import statistics
import sys
import tempfile
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from contracts.artifacts import ArtifactKind
from services.artifact_store import LocalArtifactStore


def run(count,samples):
    with tempfile.TemporaryDirectory(prefix="a2d-artifact-metadata-") as directory:
        store=LocalArtifactStore(directory,"")
        refs=[store.save_bytes(kind=ArtifactKind.IMAGE,mime_type="image/png",data=b"fixture") for _ in range(count)]
        values={"metadata_only":[],"metadata_and_payload":[]}
        for name in values:
            for index in range(samples):
                ref=refs[index%len(refs)];started=time.perf_counter_ns();store.get_ref(ref.artifact_id)
                if name=="metadata_and_payload":assert store.get_file_path(ref.artifact_id).read_bytes()==b"fixture"
                values[name].append((time.perf_counter_ns()-started)/1e6)
        return {"artifact_count":count,"paths":{name:{"samples_ms":items,"median_ms":statistics.median(items)} for name,items in values.items()}}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--counts",type=int,nargs="+",default=[100,1000,10000]);parser.add_argument("--samples",type=int,default=7);parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if not 1<=args.samples<=50 or any(not 1<=count<=10000 for count in args.counts):parser.error("samples 1..50; counts 1..10000")
    result={"python":sys.version,"scope":"当前真实制品存储方法、临时合成文件；无基线性能改进宣称","results":[run(count,args.samples) for count in args.counts]}
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(result,ensure_ascii=False))

if __name__=="__main__":main()
