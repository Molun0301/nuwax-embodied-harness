#!/usr/bin/env python3
"""无设备、无摘要的发布捕获和快照读取开销诊断；不代表 native/实机吞吐。"""
from __future__ import annotations
import json
import statistics
import sys
import time
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from contracts.geometry import Pose
from contracts.maps import OccupancyGrid
from contracts.occupancy_cells import PackedOccupancyCells
from core import StateStore
from services.mapping.service import MappingService, _capture_map_layer

def main():
    rows=[]
    for count in (1000,250000,1000000):
        for representation in ('packed','list'):
            data=PackedOccupancyCells(bytes(count)) if representation=='packed' else [0]*count
            grid=OccupancyGrid(map_id='synthetic',frame_id='map',width=count,height=1,resolution_m=.1,origin=Pose(frame_id='map'),data=data)
            times=[]
            for _ in range(7):
                started=time.perf_counter_ns();captured=_capture_map_layer(grid);times.append((time.perf_counter_ns()-started)/1e6)
                if representation=='packed':assert captured.data is grid.data
                else:assert captured.data is not grid.data
                del captured
            owner=MappingService(provider_owner=SimpleNamespace(),state_store=StateStore())
            accepted=owner.ingest_external_snapshot(map_name='synthetic',source_name='synthetic',source_epoch='run',update_sequence_override=1,occupancy_grid=grid)
            started=time.perf_counter_ns()
            for _ in range(10000):assert owner.get_latest_snapshot() is accepted
            read_ms=(time.perf_counter_ns()-started)/1e6
            rows.append({'cells':count,'representation':representation,'capture_samples':7,
                         'capture_median_ms':round(statistics.median(times),4),'capture_max_ms':round(max(times),4),
                         'read_calls':10000,'total_read_ms':round(read_ms,4),'read_copies':0,
                         'immutable_bytes_reused':representation=='packed'})
    print(json.dumps({'scope':'synthetic host-only capture and O(1) reads; no prior implementation executed','rows':rows},indent=2))
if __name__=='__main__':main()
