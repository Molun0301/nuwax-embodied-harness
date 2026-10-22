import sys
from pathlib import Path

# 保证从任意 cwd 运行都能 import 顶层包（contracts/services/...）
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 协议传输和加载夹具独立于测试用例，供按包路径运行的各入口共用。
import copy
import hashlib
import os
import json
import struct
import threading
from copy import deepcopy

import grpc
import pytest

from contracts.map_relocalization import current_assigned_relocalization_policy
from services.geometry import geometry_runtime_provider as fixture_provider
from services.geometry.checkpoint_payload import FrozenPayload

fixture_pb = fixture_provider.pb2


def fine_bytes(legacy=False):
    ref = dict(key=[0, 0, 0], points=1)
    if legacy:
        ref.update(sha256=[20] * 32, geometry_sha256=[30] * 32)
    else:
        ref.update(object=dict(kind='assigned', value=[40] * 16), block_revision=7, geometry_revision=7)
    root = dict(version=1 if legacy else 2, domain=dict(session='source', generation=3,
        frame_generation=0, producer_generation=8), revision=7, source_timestamp_ns=11,
        voxel_size_m=0.125, blocks=[ref])
    if not legacy:
        root['geometry_revision'] = 7
    raw = json.dumps(root, separators=(',', ':')).encode()
    block = b'NUWAXBL1' + struct.pack('<IIHQH', 1, 0, 0, 11, 1)
    return ((b'NUWAXAR1' if legacy else b'NUWAXAR2') + struct.pack('<Q', len(raw))
        + (b'X' * 32 if legacy else b'') + raw + struct.pack('<QI', 1, len(block)) + block)


def dense_bytes():
    metadata = dict(version=1, session_id='source', session_generation=3, cloud_revision=7,
        source_timestamp_ns=11, frame_id='map', point_count=2, bounds_min=[1, 2, 3], bounds_max=[4, 5, 6])
    raw = json.dumps(metadata, separators=(',', ':')).encode()
    return b'NUWAXDC1' + struct.pack('<I', len(raw)) + raw + struct.pack('<ffffff', 1, 2, 3, 4, 5, 6)


def response_fixture(checkpoint_id='a' * 32, fine_id='b' * 32, dense_id='c' * 32):
    occupancy = fixture_pb.CanonicalOccupancyCommitReceipt(commit_id=fixture_pb.MappingCommitIdentity(boot_nonce=b'N' * 16, sequence=5),
        session_id='source', session_generation=3, occupancy_revision=6, source_timestamp_ns=11,
        backend_kind='voxel', backend_generation=2)
    cloud = fixture_pb.CanonicalCloudCommitReceipt(commit_id=occupancy.commit_id, session_generation=3, occupancy_revision=6,
        cloud_revision=7, source_timestamp_ns=11, point_count=1)
    def identity(role, producer=0):
        return fixture_pb.CheckpointMemberIdentity(checkpoint_id=checkpoint_id, role=role,
            occupancy_receipt=occupancy, cloud_receipt=cloud, frame_id='map', producer_generation=producer)
    backend = b'NWMBCKP\0' + struct.pack('<IBBHQQQQddd', 1, 0, 0, 0, 16, 1, 1, 11, 1, 2, 3)
    response = fixture_pb.MappingBackendCheckpoint(schema_version=2, backend_kind='voxel', source_session_id='source',
        source_session_generation=3, checkpoint_id=checkpoint_id, backend_payload_schema_version=1,
        payload_size=len(backend), payload=backend, created_at_ns=12, occupancy_receipt=occupancy, cloud_receipt=cloud,
        backend_identity=identity(1), occupancy_identity=identity(2),
        canonical_occupancy_checkpoint=fixture_pb.OccupancyGridData(meta=fixture_pb.OccupancyGridMeta(width=2, height=1, resolution=0.125,
            origin_x=1.25, origin_y=-2.5, frame_id='map', revision=6, timestamp_ns=11), data=b'\xff\x64'),
        canonical_cloud_checkpoint=fixture_pb.CloudMapCheckpoint(frame_id='map', source_revision=7, point_count=1, voxel_size=0.125,
            xyz_f32_le=struct.pack('<fff', 1, 2, 3), payload_schema_version=2, member_identity=identity(3)),
        fine_map_artifact=fixture_pb.GraphArtifactRef(id=fine_id, size=len(fine_bytes()), identity_schema_version=2, member_identity=identity(4, 8)),
        dense_cloud_artifact=fixture_pb.GraphArtifactRef(id=dense_id, size=len(dense_bytes()), identity_schema_version=2, member_identity=identity(5)))
    return fixture_pb.MappingBackendCheckpoint.FromString(response.SerializeToString())


class TransferStub:
    def __init__(self, response):
        self.response = response
        self.entries = {
            response.fine_map_artifact.id: [copy.deepcopy(response.fine_map_artifact), bytearray(fine_bytes()), True],
            response.dense_cloud_artifact.id: [copy.deepcopy(response.dense_cloud_artifact), bytearray(dense_bytes()), True],
        }
        self.requests = []
        self.exports = []
    def ExportMappingBackendCheckpoint(self, request, timeout):
        assert isinstance(request, fixture_pb.ExportMappingBackendCheckpointRequest)
        self.exports.append(request.SerializeToString())
        return copy.deepcopy(self.response)
    def TransferGraphArtifact(self, request, timeout):
        self.requests.append(copy.deepcopy(request))
        ref, action = request.artifact, request.action
        if action == 1 and ref.id not in self.entries:
            self.entries[ref.id] = [copy.deepcopy(ref), bytearray(), False]
        if action == 5:
            if ref.id in self.entries:
                assert self.entries[ref.id][0] == ref
                self.entries.pop(ref.id)
            return fixture_pb.GraphArtifactTransferResponse(artifact=ref)
        retained, data, sealed = self.entries[ref.id]
        assert retained == ref
        if action == 2:
            end = request.offset + len(request.data)
            if request.offset < len(data):
                assert data[request.offset:end] == request.data
            else:
                assert request.offset == len(data)
                data.extend(request.data)
        if action == 3:
            assert len(data) == ref.size
            self.entries[ref.id][2] = sealed = True
        chunk = bytes(data[request.offset:request.offset + 1024 * 1024]) if action == 4 else b''
        return fixture_pb.GraphArtifactTransferResponse(artifact=ref, offset=len(data), sealed=sealed, data=chunk)


def runtime(stub):
    instance = fixture_provider.GeometryRuntimeProvider(None)
    instance._connected = True
    instance._stub = stub
    instance._map_state_lock = threading.Lock()
    instance._stopping = False
    instance._map_connection_epoch = 1
    return instance


def request_dict():
    return {'activation_request_id': 'activate:retained', 'session_id': 'loaded:retained',
            'map_name': 'room', 'asset_identity': {'asset_id': 'd' * 32},
            'checkpoint_id': 'a' * 32, 'expected_runtime_generation': 7}


def predecessor(active=False):
    return fixture_pb.MappingSessionState(success=True, phase='ACTIVE' if active else 'IDLE', active=active,
        session_id='predecessor' if active else '', map_name='previous' if active else '',
        session_generation=3 if active else 0, runtime_generation=7, mapping_backend_kind='voxel')


def ack(request, *, replay=False, available=True):
    retained = fixture_pb.ActivationRequestMetadataV2(activation_request_id=request.activation_request_id,
        session_id=request.session_id, map_name=request.map_name, asset_identity=request.asset_identity,
        checkpoint_id=request.mapping_backend_checkpoint.checkpoint_id,
        expected_runtime_generation=request.expected_runtime_generation)
    activation = fixture_pb.InstalledActivationIdentityV2(request=retained, runtime_generation=7,
        lifecycle_epoch=5, session_generation=6, backend_kind='voxel', backend_generation=8)
    occupancy = deepcopy(request.mapping_backend_checkpoint.occupancy_receipt)
    occupancy.commit_id.boot_nonce = b'Z' * 16
    occupancy.commit_id.sequence = 20
    occupancy.session_id = request.session_id
    occupancy.session_generation = 6
    occupancy.occupancy_revision = 10
    occupancy.backend_generation = 8
    cloud = deepcopy(request.mapping_backend_checkpoint.cloud_receipt)
    cloud.commit_id.CopyFrom(occupancy.commit_id)
    cloud.session_generation = 6
    cloud.occupancy_revision = 10
    cloud.cloud_revision = 11
    target = fixture_pb.InstalledTargetReceiptV2(activation=activation,
        source_cloud_member=request.mapping_backend_checkpoint.canonical_cloud_checkpoint.member_identity,
        occupancy_receipt=occupancy, cloud_receipt=cloud)
    policy = current_assigned_relocalization_policy()
    policy = {key: bytes.fromhex(value) if key.endswith('parameters') else value for key, value in policy.items()}
    return fixture_pb.MappingSessionState(success=True, phase='ACTIVE', active=True,
        session_id=request.session_id, map_name=request.map_name, session_generation=6, revision=10,
        idempotent_replay=replay, mapping_backend_kind='voxel', runtime_generation=7, backend_generation=8,
        activation_request_id=request.activation_request_id, activation_receipt_schema_version=2,
        activation_receipt_disposition=3 if replay else 2, relocalization_target_available=available,
        activation_receipt_v2=fixture_pb.ActivationReceiptV2(request=retained, activation=activation, target=target),
        relocalization_policy_v2=fixture_pb.RelocalizationPolicyDescriptorV2(**policy))


class LostReply(grpc.RpcError):
    def code(self):
        return grpc.StatusCode.UNAVAILABLE


class LoaderStub(TransferStub):
    def __init__(self, response):
        super().__init__(response)
        self.entries.clear()
        self.state = predecessor()
        self.calls = []
        self.owner = None
        self.during = None
        self.reply = None
        self.error = None
        self.cleanup_error = False
        self.installs = 0

    def visit(self, method):
        self.calls.append(method)
        if self.owner is not None:
            assert not self.owner._map_state_lock.locked()
            assert not self.owner._map_delivery_lock._is_owned()
        if self.during is not None:
            self.during(method)

    def GetMappingState(self, request, timeout):
        self.visit('GetMappingState')
        return deepcopy(self.state)

    def StopMapping(self, request, timeout):
        self.visit('StopMapping')
        assert request.session_id == self.state.session_id
        self.state.active = False
        self.state.phase = 'STOPPED'
        return deepcopy(self.state)

    def ActivateMap(self, request, timeout):
        self.visit('ActivateMap')
        self.last_request = deepcopy(request)
        replay = self.installs > 0
        self.installs += int(not replay)
        self.state = ack(request, replay=replay)
        if self.error:
            raise self.error
        return self.reply(request) if self.reply is not None else deepcopy(self.state)

    def TransferGraphArtifact(self, request, timeout):
        self.visit(f'Transfer:{request.action}')
        if request.action == 5 and self.cleanup_error:
            raise RuntimeError('release unavailable')
        return super().TransferGraphArtifact(request, timeout)


def create_geometry_runtime(catalog_root, **attributes):
    """真实构造几何拥有方，只替换被测边界的输入和外部读取。"""
    from types import SimpleNamespace
    from contracts.geometry_runtime import PlatformGeometryConfig
    from services.geometry.runtime import PlatformGeometryRuntime

    instance = PlatformGeometryRuntime(
        config=PlatformGeometryConfig(), catalog_root=str(catalog_root),
        robot=SimpleNamespace(providers=SimpleNamespace()),
    )
    for name, value in attributes.items():
        setattr(instance, name, value)
    return instance


def create_geometry_provider(*, is_connected, connection_generation=0, **attributes):
    """真实构造提供器，按用例设置连接状态和远程输入。"""
    instance = fixture_provider.GeometryRuntimeProvider(None)
    instance._connected = is_connected
    instance._map_connection_epoch = connection_generation
    for name, value in attributes.items():
        setattr(instance, name, value)
    return instance


@pytest.fixture
def geometry_runtime_factory(tmp_path):
    from itertools import count
    sequence = count()

    def create(**attributes):
        return create_geometry_runtime(tmp_path / f"geometry-{next(sequence)}", **attributes)

    return create


def isolated_gateway_config(tmp_path):
    """真实网关测试共用隔离路径，只关闭设备与模型的自动启动。"""
    from core.settings import load_config

    config = load_config()
    config.robot.active_robot = "customrobot"
    config.geometry.enabled = False
    config.relay.enabled = False
    config.runtime_data.map_active_restore_on_start = False
    config.runtime_data.map_catalog_root = str(tmp_path / "maps")
    config.runtime_data.memory_db_path = str(tmp_path / "memory.db")
    config.runtime_data.memory_embedding_model = "disabled"
    config.runtime_data.memory_image_embedding_model = "disabled"
    config.gateway.artifact_dir = str(tmp_path / "artifacts")
    config.logging.log_dir = str(tmp_path / "logs")
    config.logging.log_file = str(tmp_path / "logs/gateway.log")
    config.perception.yolo_world.enabled = False
    config.perception.openai_vision.enabled = False
    config.perception.segmentation.enabled = False
    config.perception.stream_runtime.enabled = False
    config.perception.stream_runtime.auto_start = False
    return config


def create_isolated_gateway(directory, *, event_bus=None, task_manager=None, robot_id="offline-robot",
                            register_default_capabilities=False):
    """真实构造隔离网关；调用方必须在finally或清理钩子中执行stop。"""
    from dataclasses import replace
    from drivers.customrobot import create_customrobot_assembly
    from gateways.runtime import GatewayRuntime

    config = isolated_gateway_config(Path(directory))
    robot = create_customrobot_assembly(config)
    robot.defaults = replace(robot.defaults, parameters={**robot.defaults.parameters, "robot_id": robot_id})
    return GatewayRuntime(config=config, robot=robot, event_bus=event_bus, task_manager=task_manager,
                          register_default_capabilities=register_default_capabilities)


@pytest.fixture
def gateway_runtime_factory(tmp_path, monkeypatch):
    """先撤销外部边界替身，再由真实网关拥有方关闭其资源。"""
    runtimes = []

    def create(**kwargs):
        runtime = create_isolated_gateway(tmp_path / f"gateway-{len(runtimes)}", **kwargs)
        runtimes.append(runtime)
        return runtime

    yield create
    monkeypatch.undo()
    for runtime in reversed(runtimes):
        runtime.stop()


def identity_call_store(path, *, identity_name="scope", gateway_instance_id="owner"):
    """用真实机器人身份构造测试账本，identity_name不是原始数据库scope。"""
    from core.agent_call_store import AgentCallStore

    path = Path(path)
    return AgentCallStore(path, robot_identity={"robot_model": "offline", "robot_id": identity_name},
                          artifact_dir=path.parent / "artifacts", gateway_instance_id=gateway_instance_id)


def insert_historical_scope_binding(path, *, legacy_scope, owner_scope):
    """直接准备已存在的历史绑定行；只用于本次测试自行创建的SQLite数据。"""
    import sqlite3

    with sqlite3.connect(path) as connection:
        connection.execute("INSERT OR REPLACE INTO agent_scope_bindings(scope,owner_scope) VALUES (?,?)",
                           (legacy_scope, owner_scope))


def create_navigation_service(directory):
    """离线领域测试使用完整构造及隔离禁行区目录，不连接设备。"""
    from core import StateStore
    from services.localization.service import LocalizationService
    from services.mapping.service import MappingService
    from services.navigation.forbidden_zones import ForbiddenZoneStore
    from services.navigation.service import NavigationService
    state = StateStore()
    return NavigationService(provider_owner=None,
        localization_service=LocalizationService(provider_owner=None, state_store=state),
        mapping_service=MappingService(provider_owner=None, state_store=state), state_store=state,
        forbidden_zone_store=ForbiddenZoneStore(Path(directory) / "navigation-zones.json"))


# 历史输入由测试侧明确写入；生产检查点发布器仅写当前格式。
def asset_document(asset, *, sha256_field):
    return {**asset, sha256_field: asset[sha256_field].hex()}


def _receipt_document(receipt):
    if receipt is None:
        return None
    return {**receipt, "commit_id": {**receipt["commit_id"],
        "boot_nonce": receipt["commit_id"]["boot_nonce"].hex()}}


def _encode(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def write_private_file(path, data):
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(descriptor, "wb") as stream:
        for chunk in data.chunks() if isinstance(data, FrozenPayload) else (data,):
            stream.write(chunk)


def write_legacy_checkpoint(store, bundle):
    """写入明确的旧selector/manifest/object文件，返回正式历史读取结果。"""
    checkpoint, occupancy, cloud = bundle
    backend = store.version_dir / "mapping_backend" / checkpoint.backend_kind
    for path in (backend.parent, backend, backend / "objects", backend / "manifests"):
        path.mkdir(mode=0o700, parents=False, exist_ok=True)
    alias = checkpoint.content_sha256.hex()
    relative = "objects/sha256-" + alias + ".bin"
    write_private_file(backend / relative, checkpoint.payload)
    document = {"schema_version": 1, "backend_kind": checkpoint.backend_kind,
        "source_session_id": checkpoint.source_session_id,
        "source_session_generation": checkpoint.source_session_generation,
        "created_at_ns": checkpoint.created_at_ns,
        "payload": {"sha256": alias, "path": relative, "size": checkpoint.payload_size},
        "canonical_receipts": {"occupancy": _receipt_document(checkpoint.occupancy_receipt),
            "cloud": _receipt_document(checkpoint.cloud_receipt)},
        "canonical_assets": {"occupancy": asset_document(occupancy, sha256_field="raw_data_sha256"),
            "cloud": asset_document(cloud, sha256_field="xyz_sha256") if cloud is not None else None}}
    for name, data in (("fine_archive", checkpoint.fine_archive), ("dense_archive", checkpoint.dense_archive)):
        if data is not None:
            payload = bytes(data)
            alias = hashlib.sha256(payload).hexdigest()
            relative = "objects/sha256-" + alias + ".bin"
            write_private_file(backend / relative, payload)
            document[name] = {"sha256": alias, "path": relative, "size": len(payload)}
    encoded = _encode(document)
    manifest_alias = hashlib.sha256(encoded).hexdigest()
    relative = "manifests/sha256-" + manifest_alias + ".json"
    write_private_file(backend / relative, encoded)
    write_private_file(backend / "checkpoint-v1.json", _encode({"schema_version": 1,
        "backend_kind": checkpoint.backend_kind, "manifest": {"sha256": manifest_alias, "path": relative}}))
    return store.load(checkpoint.backend_kind, schema_version=1)
