#!/usr/bin/env python3
"""合成完整点云入库/加载扫描对比；不代表 RPC、定位或实机激活总耗时。"""
import argparse
import gc
import inspect
import json
from pathlib import Path
import platform
import statistics
import struct
import sys
import tempfile
import time
from types import ModuleType

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
# 仅装载存储实现，避免启动服务或要求相机/原生定位等无关依赖。
for name in ('services', 'services.geometry'):
    package = ModuleType(name)
    package.__path__ = [str(ROOT.joinpath(*name.split('.')))]
    sys.modules[name] = package

from services.geometry import dense_cloud
from services.geometry import mapping_backend_checkpoint_store as store


def prior_validator():
    """同一验证函数仅恢复此前两个 axis=0 归约，隔离本次优化。"""
    source = inspect.getsource(dense_cloud.validate_dense_archive)
    for old, new in (
        ('[points[:, axis].min() for axis in range(3)]', 'points.min(axis=0)'),
        ('[points[:, axis].max() for axis in range(3)]', 'points.max(axis=0)'),
    ):
        if source.count(old) != 1:
            raise RuntimeError('当前扫描实现已改变，请重新核对对比基线')
        source = source.replace(old, new)
    namespace = dict(vars(dense_cloud))
    exec(compile(source, '<prior-axis0-validation>', 'exec'), namespace)
    return namespace['validate_dense_archive']


def fixture(count):
    # 完整扫描云可以多于降采样 canonical cloud；保留它们各自的真实点数。
    metadata = dict(version=1, session_id='source', session_generation=3,
                    cloud_revision=7, source_timestamp_ns=11, frame_id='map',
                    point_count=count, bounds_min=[1, 2, 3], bounds_max=[1, 2, 3])
    header = json.dumps(metadata).encode('utf-8')
    dense = b'NUWAXDC1' + struct.pack('<I', len(header)) + header + struct.pack('<fff', 1, 2, 3) * count
    occupancy_receipt = dict(commit_id=dict(boot_nonce=b'N' * 16, sequence=5),
        session_id='source', session_generation=3, occupancy_revision=6,
        source_timestamp_ns=11, backend_kind='voxel', backend_generation=2)
    cloud_receipt = dict(commit_id=occupancy_receipt['commit_id'], session_generation=3,
        occupancy_revision=6, cloud_revision=7, source_timestamp_ns=11, point_count=1)
    def member(role):
        return store.CheckpointMemberIdentity('a' * 32, role, occupancy_receipt, cloud_receipt, 'map')
    backend = b'NWMBCKP\0' + struct.pack('<IBBHQQQQddd', 1, 0, 0, 0, 16, 1, 1, 11, 1, 2, 3)
    checkpoint = store.MappingBackendCheckpoint(2, 'voxel', 'source', 3, b'', len(backend), 12,
        backend, occupancy_receipt, cloud_receipt, checkpoint_id='a' * 32,
        canonical_occupancy_data=b'\xff\x64', canonical_cloud_xyz=struct.pack('<fff', 1, 2, 3),
        backend_identity=member('backend_state'), occupancy_identity=member('canonical_occupancy'),
        cloud_identity=member('canonical_cloud'), dense_archive=dense,
        dense_reference=store.CheckpointArtifactReference('b' * 32, len(dense), 2, member('dense_cloud')))
    occupancy = dict(raw_data_size=2, width=2, height=1, resolution=.125, origin_x=1.25,
        origin_y=-2.5, frame_id='map', source_occupancy_revision=6, timestamp_ns=11)
    cloud = dict(xyz_size=12, frame_id='map', source_cloud_revision=7, point_count=1,
        voxel_size=.125, payload_schema_version=2)
    return checkpoint, occupancy, cloud


def measure(samples, functions):
    for function in functions.values():
        function()
    timings = {name: [] for name in functions}
    for repetition in range(samples):
        order = list(functions.items())
        if repetition % 2:
            order.reverse()
        for name, function in order:
            gc.collect()
            started = time.perf_counter_ns()
            function()
            timings[name].append((time.perf_counter_ns() - started) / 1e6)
    return {name: dict(median_ms=statistics.median(values), samples_ms=values)
            for name, values in timings.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points', nargs='+', type=int, default=[65536, 1000000, 4000000])
    parser.add_argument('--samples', type=int, default=7)
    args = parser.parse_args()
    if not 1 <= args.samples <= 31 or any(not 1 <= count <= 4000000 for count in args.points):
        parser.error('样本次数限制为 1..31；每个合成云限制为 1..4000000 点（约 48 MB）')
    baseline = prior_validator()
    candidate = dense_cloud.validate_dense_archive
    rows = []
    for count in args.points:
        checkpoint, occupancy, cloud = fixture(count)
        payload = checkpoint.dense_archive
        with tempfile.TemporaryDirectory(prefix='neh-dense-admission-') as directory:
            repository = store.MappingBackendCheckpointStore(Path(directory))
            repository.publish(checkpoint, occupancy_asset=occupancy, cloud_asset=cloud)
            def load_with(validator):
                previous = store.validate_dense_archive
                store.validate_dense_archive = validator
                try:
                    loaded = repository.load('voxel', schema_version=2, expected_checkpoint_id='a' * 32)
                    assert loaded.checkpoint.checkpoint_id == checkpoint.checkpoint_id
                    assert loaded.checkpoint.dense_reference == checkpoint.dense_reference
                    assert len(loaded.checkpoint.dense_archive) == len(payload)
                finally:
                    store.validate_dense_archive = previous
            scans = measure(args.samples, dict(
                baseline=lambda: baseline(payload), candidate=lambda: candidate(payload)))
            loads = measure(args.samples, dict(
                baseline=lambda: load_with(baseline), candidate=lambda: load_with(candidate)))
            rows.append(dict(points=count, archive_bytes=len(payload), full_validation=scans,
                             checkpoint_load_warm_files=loads))
    print(json.dumps(dict(scope='Synthetic host-only archive validation and checkpoint-store load; '
        'warm filesystem cache; excludes provider RPC, native restore, localization and UI',
        baseline='Current validation with only the prior axis=0 min/max reductions restored',
        python=platform.python_version(), numpy=np.__version__, rows=rows), indent=2))


if __name__ == '__main__':
    main()
