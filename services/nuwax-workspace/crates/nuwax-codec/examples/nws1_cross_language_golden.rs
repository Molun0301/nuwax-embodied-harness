// 重放 NWS1 v1 Python facade 与 Rust codec 共用字节级黄金样本。

use std::collections::BTreeMap;

use nuwax_codec::lcm::{
    canonical_measurement::assemble,
    sensors::{
        decode, encode, CanonicalCloudFrame, CanonicalImuFrame, CloudPayload, PayloadOwnership,
        SensorFrame, SensorIdentity,
    },
};

fn identity(
    sensor_id: &str,
    source_sequence: u64,
    timestamp_ns: u64,
    payload_ownership: PayloadOwnership,
) -> SensorIdentity {
    SensorIdentity {
        schema_version: 1,
        sensor_id: sensor_id.into(),
        source_sequence,
        timestamp_ns,
        clock_domain: "unix_ns".into(),
        frame_id: "go2_base_link".into(),
        registration_state: "body".into(),
        registration_frame_id: "go2_base_link".into(),
        payload_ownership,
        calibration_ref: Some("go2-urdf-base-v1".into()),
    }
}

fn frames() -> Vec<(&'static str, SensorFrame)> {
    let xyz_f32_le = [1.0_f32, -2.0, 0.5, 0.0, 3.25, -4.5, 10.0, 0.125, -0.25]
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect();
    let intensity_f32_le = [0.0_f32, 42.5, 255.0]
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect();
    let ring_u16_le = [0_u16, 17, u16::MAX]
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect();
    let time_offset_ns_le = [0_u64, 60_000, 120_000]
        .iter()
        .flat_map(|value| value.to_le_bytes())
        .collect();

    vec![
        (
            "inline_cloud",
            SensorFrame::Cloud(CanonicalCloudFrame {
                identity: identity(
                    "go2_l2",
                    42,
                    1_787_030_400_123_456_789,
                    PayloadOwnership::Inline,
                ),
                point_count: 3,
                scan_duration_ns: 120_000,
                payload: CloudPayload::Inline {
                    xyz_f32_le,
                    intensity_f32_le: Some(intensity_f32_le),
                    ring_u16_le: Some(ring_u16_le),
                    time_offset_ns_le: Some(time_offset_ns_le),
                },
            }),
        ),
        (
            "shm_cloud",
            SensorFrame::Cloud(CanonicalCloudFrame {
                identity: identity(
                    "go2_l2",
                    43,
                    1_787_030_400_123_456_790,
                    PayloadOwnership::Shm,
                ),
                point_count: 2,
                scan_duration_ns: 80_000,
                payload: CloudPayload::External {
                    ownership: PayloadOwnership::Shm,
                    payload_ref: "shm://nws1-golden-cloud".into(),
                    payload_layout: vec![
                        "xyz_f32_le".into(),
                        "intensity_f32_le".into(),
                        "ring_u16_le".into(),
                        "time_offset_ns_le".into(),
                    ],
                    payload_size_bytes: 52,
                    lease_epoch: 7,
                    expiry_ns: 1_787_030_401_000_000_000,
                },
            }),
        ),
        (
            "imu_before",
            SensorFrame::Imu(CanonicalImuFrame {
                identity: identity(
                    "go2_imu",
                    9_001,
                    1_787_030_400_123_446_789,
                    PayloadOwnership::Inline,
                ),
                accel_mps2: [0.0, -9.80665, 0.125],
                gyro_rps: [0.01, -0.02, 1.5],
            }),
        ),
        (
            "imu_after",
            SensorFrame::Imu(CanonicalImuFrame {
                identity: identity(
                    "go2_imu",
                    9_002,
                    1_787_030_400_123_576_789,
                    PayloadOwnership::Inline,
                ),
                accel_mps2: [0.25, -9.75, 0.0],
                gyro_rps: [-0.03, 0.04, 1.25],
            }),
        ),
    ]
}

fn goldens() -> BTreeMap<&'static str, Vec<u8>> {
    include_str!("../reference/nws1_v1_cross_language.hex")
        .lines()
        .filter_map(|raw_line| {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                return None;
            }
            let mut fields = line.split_whitespace();
            let name = fields.next().expect("黄金样本缺少名称");
            let encoded = fields.next().expect("黄金样本缺少字节");
            assert!(fields.next().is_none(), "黄金样本存在多余字段");
            Some((name, decode_hex(encoded)))
        })
        .collect()
}

fn main() {
    let expected = goldens();
    let frames = frames();
    assert_eq!(expected.len(), frames.len());

    for (name, frame) in frames {
        let encoded = encode(&frame).unwrap();
        assert_eq!(encoded, expected[name], "{name} 的 Rust 编码字节发生漂移");
        assert_eq!(decode(&expected[name]).unwrap(), frame);
    }

    let imu_before = match decode(&expected["imu_before"]).unwrap() {
        SensorFrame::Imu(frame) => frame,
        SensorFrame::Cloud(_) => panic!("imu_before 不是 IMU 帧"),
    };
    let imu_after = match decode(&expected["imu_after"]).unwrap() {
        SensorFrame::Imu(frame) => frame,
        SensorFrame::Cloud(_) => panic!("imu_after 不是 IMU 帧"),
    };
    for name in ["inline_cloud", "shm_cloud"] {
        let cloud = match decode(&expected[name]).unwrap() {
            SensorFrame::Cloud(frame) => frame,
            SensorFrame::Imu(_) => panic!("{name} 不是点云帧"),
        };
        let scan_begin = cloud.identity.timestamp_ns;
        let scan_end = scan_begin.checked_add(cloud.scan_duration_ns).unwrap();
        let measurement = assemble(
            cloud,
            vec![imu_before.clone(), imu_after.clone()],
            scan_begin,
            scan_end,
            "calibration_applied:go2-urdf-base-v1",
        )
        .unwrap();
        assert_eq!(measurement.imu_samples.len(), 2);
        assert_eq!(measurement.scan_end_timestamp_ns, scan_end);
    }

    println!(
        "[PASS] nws1_cross_language_golden {} frames, 2 measurements",
        expected.len()
    );
}

fn decode_hex(value: &str) -> Vec<u8> {
    assert_eq!(value.len() % 2, 0);
    (0..value.len())
        .step_by(2)
        .map(|offset| u8::from_str_radix(&value[offset..offset + 2], 16).unwrap())
        .collect()
}
