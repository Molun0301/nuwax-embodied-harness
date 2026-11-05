"""当前制品保存及保留清理的离线耗时测量，仅操作临时合成文件。"""
from pathlib import Path
import argparse
import json
import statistics
import sys
import tempfile
import time
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from contracts.artifacts import ArtifactKind,ArtifactRetentionPolicy
from contracts.image import ImageFrame,ImageEncoding
from services.artifact_store import LocalArtifactStore
from services.artifact_service import ArtifactService


def scaling(count,samples,background):
    with tempfile.TemporaryDirectory(prefix="a2d-artifact-save-") as directory:
        store=LocalArtifactStore(directory,"")
        for _ in range(count):store.save_bytes(kind=ArtifactKind.IMAGE,mime_type="image/jpeg",data=b"fixture")
        service=ArtifactService(store,retention_policy=ArtifactRetentionPolicy(max_count=count))
        if background:service.start_maintenance()
        frame=ImageFrame(camera_id="fixture",frame_id="camera",width_px=1,height_px=1,encoding=ImageEncoding.JPEG,data=b"fixture")
        values=[]
        try:
            for _ in range(samples):
                started=time.perf_counter_ns();ref=service.save_image_frame(frame)
                if not background:service.cleanup_if_needed()
                values.append((time.perf_counter_ns()-started)/1e6)
                assert store.read_bytes(ref.artifact_id)==frame.data
        finally:service.close()
        summary=store.build_summary()
        return {"artifact_count":count,"samples_ms":values,"median_ms":statistics.median(values),"remaining_artifacts":summary.artifact_count,"remaining_bytes":summary.total_size_bytes}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--counts",type=int,nargs="+",default=[100,1000,10000]);parser.add_argument("--samples",type=int,default=7);parser.add_argument("--output",type=Path);parser.add_argument("--async-maintenance",action="store_true")
    args=parser.parse_args()
    if args.samples<1 or any(count<1 or count>100000 for count in args.counts):parser.error("samples >= 1, artifact counts 1..100000")
    result={"python":sys.version,"async_maintenance":args.async_maintenance,"scope":"当前存储和保留清理、临时合成数据；共享主机测量不代表设备延迟","scaling":[scaling(count,args.samples,args.async_maintenance) for count in args.counts]}
    payload=json.dumps(result,ensure_ascii=False,indent=2)
    if args.output:args.output.write_text(payload)
    print(payload)

if __name__=="__main__":main()
