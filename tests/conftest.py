import sys
from pathlib import Path

# 保证从任意 cwd 运行都能 import 顶层包（contracts/services/...）
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# 协议传输和加载夹具独立于测试用例，供按包路径运行的各入口共用。
import copy
import json
import struct
import threading
from copy import deepcopy

import grpc

from contracts.map_relocalization import current_assigned_relocalization_policy
from services.geometry import geometry_runtime_provider as fixture_provider

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
        self.exports.append(request.checkpoint_schema_version)
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
    instance = fixture_provider.GeometryRuntimeProvider.__new__(fixture_provider.GeometryRuntimeProvider)
    instance._connected = True
    instance._stub = stub
    instance._map_state_lock = threading.Lock()
    instance._stopping = False
    instance._map_connection_epoch = 1
    instance._raise_if_stopping = lambda *args: None
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
