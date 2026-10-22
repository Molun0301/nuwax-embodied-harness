#!/usr/bin/env python3
"""标定拥有方、可逆独立引用和配组/录制/线协议的纯离线检查。"""
from __future__ import annotations

import base64
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack, contextmanager
from dataclasses import replace
import hashlib
import importlib
import json
from pathlib import Path
import pickle
import sqlite3
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch
import uuid

import numpy as np
import pydantic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
ATTEMPTS = []


@contextmanager
def no_hash():
    def forbidden(*args, **kwargs):
        ATTEMPTS.append(True)
        raise AssertionError('禁止执行摘要或名称UUID')
    with ExitStack() as stack:
        for name in ('new', 'md5', 'sha1', 'sha224', 'sha256', 'sha384', 'sha512',
                     'sha3_224', 'sha3_256', 'sha3_384', 'sha3_512', 'shake_128', 'shake_256', 'blake2b', 'blake2s'):
            stack.enter_context(patch.object(hashlib, name, side_effect=forbidden))
        for name in ('uuid3', 'uuid5'):
            stack.enter_context(patch.object(uuid, name, side_effect=forbidden))
        yield


# 跳过聚合服务/设备包的初始化，实际模块仍从仓库装载，绝不读取真实settings。
for name in ('contracts', 'adapters', 'adapters.sensors', 'drivers', 'drivers.go2',
             'core', 'services', 'services.geometry'):
    package = types.ModuleType(name); package.__path__ = [str(ROOT.joinpath(*name.split('.')))]
    sys.modules[name] = package
with no_hash():
    calibration = importlib.import_module('adapters.sensors.go2_calibration')
    adapter = importlib.import_module('adapters.sensors.go2_l2')
    assembler = importlib.import_module('adapters.sensors.lio_measurement')
    bridge_module = importlib.import_module('drivers.go2.data_plane')
    process_module = importlib.import_module('drivers.go2.sensor_process')
    wire = importlib.import_module('services.geometry.sensor_wire')
    recording = importlib.import_module('services.geometry.recording')
    from contracts.sensors import RegistrationState

OWNER = calibration.Go2CalibrationProfileOwner
capture = calibration.go2_calibration_profile_from_config
DEFAULT = calibration.GO2_CALIBRATION_PROFILE
WORK = Path(tempfile.gettempdir()) / 'neh-calibration-profile-identity-checks'
ROTATION = [1, 0, 0, 0, 1, 0, 0, 0, 1]


def transform(translation=(1, 2, 3), rotation=ROTATION):
    return json.dumps({'translation_m': list(translation), 'rotation_row_major': list(rotation)})


def config(**changes):
    result = types.SimpleNamespace(lio_backend='fastlio2', lio_body_from_sensor=transform(),
        lio_body_from_imu=transform((0, 0, 0)), direct_dds=types.SimpleNamespace(enabled=True))
    for key, value in changes.items(): setattr(result, key, value)
    return result


def decoded():
    return adapter.Go2L2DecodedPointCloud(source_frame=DEFAULT.lidar_source_frame, width=2, height=1,
        point_step=12, row_step=24, is_dense=True, fields={},
        xyz_array=np.asarray([[0, 1, 2], [2, 3, 4]], dtype='<f4'),
        intensity_array=None, offset_ns_array=np.asarray([0, 10], dtype='<u8'))


def lidar(profile, *, timestamp=100, sequence=1, state=RegistrationState.BODY):
    return adapter.adapt_go2_l2_lidar(decoded(), calibration_profile=profile,
        timestamp_ns=timestamp, source_sequence=sequence, registration_state=state)


def imu(profile, *, timestamp=100, sequence=1, state=RegistrationState.BODY):
    return adapter.adapt_go2_l2_imu(calibration_profile=profile, timestamp_ns=timestamp,
        source_sequence=sequence, frame_id='world/go2/imu', accel_mps2=(1, 2, 3), gyro_rps=(4, 5, 6),
        registration_state=state)


def legacy_factory(config):
    return config


def profile_factory(config, *, calibration_profile):
    return config, calibration_profile


class CalibrationIdentityChecks(unittest.TestCase):
    def test_default_profile_and_reference_stay_identical(self):
        self.assertIs(capture(), DEFAULT)
        self.assertEqual(DEFAULT.calibration_ref, 'go2-urdf-base-v1')
        self.assertEqual(capture(owner=OWNER()), DEFAULT)

    def test_standalone_repeat_uses_no_uuid_and_exact_same_reference(self):
        with patch.object(uuid, 'uuid4', side_effect=AssertionError('独立调用不随机生成身份')):
            left = capture(transform()); right = capture(transform())
        self.assertEqual(left, right)
        self.assertTrue(left.calibration_ref.startswith('g2cal2:'))

    def test_standalone_full_values_are_reversible_little_endian(self):
        profile = capture(transform((1.25, -2.5, 3.75)), transform((4.5, 5.25, -6.125)))
        raw = base64.b85decode(profile.calibration_ref[len('g2cal2:'):])
        self.assertEqual(len(raw), 192)
        self.assertEqual(raw[:8], struct.pack('<d', 1.25))
        self.assertEqual(struct.unpack('<24d', raw), calibration._profile_identity(profile)[1])

    def test_standalone_every_transform_group_changes_identity(self):
        original = capture(transform(), transform())
        turned = [0, -1, 0, 1, 0, 0, 0, 0, 1]
        variants = [capture(transform((2, 2, 3)), transform()), capture(transform(rotation=turned), transform()),
                    capture(transform(), transform((2, 2, 3))), capture(transform(), transform(rotation=turned))]
        self.assertEqual(len({original.calibration_ref, *(value.calibration_ref for value in variants)}), 5)

    def test_formatting_numeric_representation_and_negative_zero_are_equal(self):
        a = transform((0, 0, 0))
        b = json.dumps({'rotation_row_major': [float(x) if x else -0.0 for x in ROTATION],
                        'translation_m': [-0.0, 0.0, 0]}, indent=3)
        self.assertEqual(capture(a).calibration_ref, capture(b).calibration_ref)
        owner = OWNER()
        self.assertEqual(capture(a, owner=owner).calibration_ref, capture(b, owner=owner).calibration_ref)
        raw = base64.b85decode(capture(b).calibration_ref[7:])
        self.assertEqual(raw[:24], struct.pack('<3d', 0, 0, 0))

    def test_owner_allocates_epoch_once_and_reuses_equal_capture(self):
        with patch.object(uuid, 'uuid4', wraps=uuid.uuid4) as assigned:
            owner = OWNER(); left = capture(transform(), owner=owner)
            for _ in range(10): self.assertIs(capture(transform(), owner=owner), left)
            self.assertEqual(assigned.call_count, 1)
        self.assertRegex(left.calibration_ref, r'^g2cal-owner:[0-9a-f]{32}:1$')

    def test_changed_capture_advances_revision_but_old_snapshot_is_immutable(self):
        owner = OWNER(); old = capture(transform(), owner=owner)
        changed = capture(transform((2, 2, 3)), owner=owner)
        self.assertTrue(changed.calibration_ref.endswith(':2'))
        self.assertEqual(old.lidar_translation_m, (1, 2, 3))
        mutable = old.lidar_rotation; mutable[0, 0] = 100
        self.assertEqual(old.lidar_rotation[0, 0], 1)
        self.assertEqual(capture(transform(), owner=owner).calibration_ref.rsplit(':', 1)[1], '3')

    def test_new_owner_does_not_guess_cross_run_identity(self):
        left = capture(transform(), owner=OWNER()); right = capture(transform(), owner=OWNER())
        self.assertEqual(left.lidar_translation_m, right.lidar_translation_m)
        self.assertNotEqual(left.calibration_ref, right.calibration_ref)

    def test_override_default_override_advances_owner_revision(self):
        owner = OWNER(); first = capture(transform(), owner=owner)
        self.assertEqual(capture(owner=owner).calibration_ref, DEFAULT.calibration_ref)
        self.assertNotEqual(capture(transform(), owner=owner).calibration_ref, first.calibration_ref)

    def test_invalid_configuration_never_advances_owner_revision(self):
        owner = OWNER(); first = capture(transform(), owner=owner)
        invalid = ['{}', '{"translation_m":[1,2,3],"translation_m":[1,2,3],"rotation_row_major":' + json.dumps(ROTATION) + '}',
            transform((float('nan'), 0, 0)), transform((float('inf'), 0, 0)), transform(rotation=[1] * 9),
            transform(rotation=[-1, 0, 0, 0, 1, 0, 0, 0, 1]), transform((True, 0, 0)), '[]']
        for raw in invalid:
            with self.subTest(raw=raw), self.assertRaises(ValueError): capture(raw, owner=owner)
        self.assertIs(capture(transform(), owner=owner), first)
        with self.assertRaises(ValueError): capture(None, owner=owner)

    def test_nontransform_context_cannot_silently_alias(self):
        changes = [{'calibration_ref': 'other-model-version'}, {'registration_frame': 'other-body'},
                   {'lidar_source_frame': 'other-lidar'}, {'imu_source_frames': frozenset({'other-imu'})}]
        owner = OWNER(); old = capture(transform(), owner=owner)
        for fields in changes:
            with self.subTest(fields=fields), patch.object(calibration, 'GO2_CALIBRATION_PROFILE', replace(DEFAULT, **fields)):
                self.assertNotEqual(capture(transform(), owner=owner).calibration_ref, old.calibration_ref)
                with self.assertRaisesRegex(ValueError, '升级编码版本'): capture(transform())

    def test_concurrent_equal_captures_share_one_revision(self):
        owner = OWNER()
        with ThreadPoolExecutor(max_workers=4) as executor:
            results = list(executor.map(lambda _: capture(transform(), owner=owner), range(24)))
        self.assertEqual(len({item.calibration_ref for item in results}), 1)
        self.assertTrue(results[0].calibration_ref.endswith(':1'))

    def test_real_lidar_imu_adapters_share_reference_and_full_transform(self):
        profile = capture(transform(), transform((0, 0, 0)), owner=OWNER())
        cloud = lidar(profile); sample = imu(profile)
        self.assertEqual(cloud.calibration_ref, sample.calibration_ref)
        self.assertEqual(cloud.transform_provenance, 'calibration_applied:' + profile.calibration_ref)
        np.testing.assert_allclose(np.frombuffer(cloud.xyz_f32_le, dtype='<f4').reshape(-1, 3), decoded().xyz_array + [1, 2, 3])
        self.assertEqual(sample.accel_mps2, (1, 2, 3))

    def test_repeated_samples_do_not_capture_or_allocate(self):
        profile = capture(transform(), owner=OWNER())
        with patch.object(OWNER, 'capture', side_effect=AssertionError('逐帧不捕获')), \
              patch.object(uuid, 'uuid4', side_effect=AssertionError('逐帧不分配')):
            for index in range(50):
                self.assertEqual(lidar(profile, sequence=index + 1).calibration_ref, imu(profile, sequence=index + 1).calibration_ref)

    def test_assembler_rejects_other_owner_and_revision_until_reset(self):
        owner = OWNER(); profile = capture(transform(), owner=owner)
        for other in [capture(transform(), owner=OWNER()), capture(transform((2, 2, 3)), owner=owner)]:
            value = assembler.LioMeasurementAssembler(); value.add_imu(imu(profile, timestamp=99))
            with self.assertRaises(assembler.LioMeasurementAssemblerError): value.add_lidar(lidar(other))
            value.reset(); value.add_imu(imu(other, timestamp=99)); value.add_lidar(lidar(other))
            measurement = value.add_imu(imu(other, timestamp=111, sequence=2))
            self.assertEqual(measurement.lidar.calibration_ref, other.calibration_ref)

    def test_bridge_stop_start_and_old_generation_keep_captured_profile(self):
        bridge = bridge_module.Go2DirectDdsBridge(config()); profile = bridge._calibration_profile
        callback = Mock(); bridge.subscribe_sensor_sample(callback)
        with patch.object(bridge, '_start_direct_dds_backend', side_effect=lambda: setattr(bridge, '_direct_dds_started', True)), \
              patch.object(bridge, '_stop_direct_dds_backend', side_effect=lambda: setattr(bridge, '_direct_dds_started', False)), \
              patch.object(uuid, 'uuid4', side_effect=AssertionError('重启不创建标定身份')):
            bridge.start(); generation = bridge._active_generation; bridge.start()
            self.assertEqual(bridge._active_generation, generation)
            bridge.stop(); bridge.start(); self.assertGreater(bridge._active_generation, generation)
            bridge._publish_sensor_sample(imu(profile), generation)
            callback.assert_not_called()
            bridge._publish_sensor_sample(imu(profile), bridge._active_generation)
            callback.assert_called_once()
            bridge.stop()
        self.assertIs(bridge._calibration_profile, profile)

    def test_mutating_config_does_not_claim_new_transform_is_already_applied(self):
        settings = config(); old = bridge_module.Go2DirectDdsBridge(settings)
        settings.lio_body_from_sensor = transform((9, 8, 7))
        self.assertEqual(old._calibration_profile.lidar_translation_m, (1, 2, 3))
        new = bridge_module.Go2DirectDdsBridge(settings)
        self.assertEqual(new._calibration_profile.lidar_translation_m, (9, 8, 7))
        self.assertNotEqual(new._calibration_profile.calibration_ref, old._calibration_profile.calibration_ref)

    def test_device_mode_still_ignores_unused_native_overrides(self):
        with patch.object(uuid, 'uuid4', side_effect=AssertionError('设备默认无需owner')):
            bridge = bridge_module.Go2DirectDdsBridge(config(lio_backend='device', lio_body_from_sensor='bad'))
        self.assertIs(bridge._calibration_profile, DEFAULT)

    def test_isolated_bridge_passes_exact_profile_to_existing_worker_factory(self):
        bridge = bridge_module.Go2DirectDdsBridge(config(), isolate_native_input=True)
        worker = Mock()
        with patch.object(process_module, 'Go2SensorProcess', return_value=worker) as factory:
            bridge.start()
            self.assertIs(factory.call_args.kwargs['calibration_profile'], bridge._calibration_profile)
            worker.start.assert_called_once()
            bridge.stop()

    def worker(self, **kwargs):
        # 构造现有owner但不start，不创建子进程或SDK连接。
        return process_module.Go2SensorProcess(config(), on_sample=Mock(), on_snapshot=Mock(), on_error=Mock(), **kwargs)

    def test_default_factory_pickle_keeps_profile_and_does_not_recapture(self):
        profile = capture(transform(), owner=OWNER())
        worker = self.worker(calibration_profile=profile)
        factory = pickle.loads(pickle.dumps(worker.source_factory))
        with patch.object(uuid, 'uuid4', side_effect=AssertionError('子实例不再分配')):
            child = factory(config())
        self.assertEqual(child._calibration_profile, profile)
        self.assertIsNone(child._calibration_owner)

    def test_legacy_custom_factory_signature_is_unchanged(self):
        worker = self.worker(source_factory=legacy_factory)
        settings = config()
        self.assertIs(worker.source_factory(settings), settings)

    def test_explicit_custom_factory_receives_pickled_profile(self):
        profile = capture(transform(), owner=OWNER())
        worker = self.worker(source_factory=profile_factory, calibration_profile=profile)
        factory = pickle.loads(pickle.dumps(worker.source_factory))
        self.assertEqual(factory(config())[1], profile)

    def test_wire_accepts_new_legacy_and_default_refs_with_size_and_char_rules(self):
        refs = [capture(transform()).calibration_ref, capture(transform(), owner=OWNER()).calibration_ref,
                'go2-configured-v1-' + 'a' * 64, DEFAULT.calibration_ref]
        self.assertEqual(len(refs[0].encode('utf-8')), 247)
        for reference in refs:
            self.assertLessEqual(len(reference.encode('utf-8')), 255)
            self.assertEqual(reference, reference.strip()); self.assertNotIn('\x00', reference)
            identity = wire.SensorIdentity(1, 'imu', 1, 100, 'unix_ns', 'go2_base_link', 'body', 'go2_base_link', wire.PayloadOwnership.INLINE, reference)
            frame = wire.ImuFrame(identity, (1, 2, 3), (4, 5, 6))
            self.assertEqual(wire.decode_sensor_frame(wire.encode_sensor_frame(frame)), frame)
        for bad in ['x' * 256, 'bad\x00ref']:
            identity = replace(identity, calibration_ref=bad)
            with self.assertRaises(wire.SensorWireError): wire.encode_sensor_frame(wire.ImuFrame(identity, (1, 2, 3), (4, 5, 6)))

    def test_recording_roundtrip_keeps_opaque_ref_and_provenance_without_reconstruction(self):
        WORK.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='fixture-', dir=WORK) as folder:
            path = Path(folder) / 'recording.sqlite3'
            db = sqlite3.connect(path)
            db.execute('CREATE TABLE records (id INTEGER PRIMARY KEY,kind TEXT,source_ns TEXT,received_ns INTEGER,metadata TEXT,payload BLOB)')
            profiles = [capture(transform()), capture(transform(), owner=OWNER()), replace(DEFAULT, calibration_ref='go2-configured-v1-' + 'b' * 64), DEFAULT]
            samples = [sample for profile in profiles for sample in (imu(profile), lidar(profile))]
            for index, sample in enumerate(samples, 1):
                db.execute('INSERT INTO records VALUES (?,?,?,?,?,?)', recording.MappingRecorder._encode_record(
                    (index, 'sensor.' + sample.sample_kind.value, sample.timestamp_ns, 0, sample, 0)))
            db.commit(); db.close()
            with patch.object(calibration, 'go2_calibration_profile_from_config', side_effect=AssertionError('回放不重新标定')):
                self.assertEqual(list(recording.iter_recorded_sensors(path)), samples)


if __name__ == '__main__':
    with no_hash():
        result = unittest.main(verbosity=2, exit=False)
    print('Forbidden digest/name-UUID attempts:', len(ATTEMPTS))
    raise SystemExit(0 if result.result.wasSuccessful() and not ATTEMPTS else 1)
