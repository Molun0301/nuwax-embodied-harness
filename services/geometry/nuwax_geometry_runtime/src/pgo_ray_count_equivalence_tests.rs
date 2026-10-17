//! 真 ReplayBuilder 入口的连续帧资格；不把单独调用核函数等同于PGO覆盖。
use super::*;
use crate::lio_ingress::{CorrelatedOdometry, RegistrationBinding};
use crate::mapping_backend::ray_count_equivalence_tests::{
    five_cm_config, five_cm_spec, grid_snapshot, kernel_snapshot, write_business_snapshot,
};
use serde_json::{json, Value};

fn replay_fixture() -> (Vec<ArchivedScan>, Vec<OptimizedKeyframe>) {
    let scans = (0..48_u64).map(|index| {
        let pose = Isometry3::translation(index as f64 * 0.002, 0.0, 0.0);
        let group = (index / 4).min(5) as f64;
        // 前24帧增加局部区域，后24帧只重访最后一块，历史仍由真实输入积累。
        let world = (0..6).flat_map(|x| (0..6).map(move |y| {
            Point3::new(1.025 + group * 0.35 + x as f64 * 0.05,
                -0.125 + y as f64 * 0.05, 0.0)
        })).chain((0..8).map(|z| Point3::new(3.525, 0.075, 0.05 + z as f64 * 0.05)))
            .collect::<Vec<_>>();
        let stamp = 1_000_000_000 + index * 1_000_000;
        let sequence = index as i64 + 1;
        let binding = RegistrationBinding {
            runtime_generation: 7, cloud_source_sequence: sequence,
            correlated_odometry: CorrelatedOdometry {
                source_sequence: sequence, source_timestamp_ns: stamp as i64,
            },
            mapping_session_id: "ray-count-pgo".into(), mapping_session_generation: 1,
            map_frame_generation: 1, frozen_odom_from_body: pose,
            base_anchor_map_from_odom: Isometry3::identity(),
            registration_map_from_odom: Isometry3::identity(), registration_correction_revision: 0,
            sensor_origin_map: Some(pose.translation.vector.into()),
            frame_capture_relocalization_epoch: None,
        };
        let source = AdmittedRegisteredScan::from_map_points_for_test(stamp, binding, &world);
        ArchivedScan {
            source_timestamp_ns: stamp, received_at_ns: stamp,
            odom_from_body: pose, sensor_origin_body: Point3::origin(),
            body_points: source.body_points().clone(), source: ArchivedSource::Lidar(source),
        }
    }).collect::<Vec<_>>();
    let first = OptimizedKeyframe {
        source_timestamp_ns: scans[0].source_timestamp_ns as i64,
        translation_m: [0.0; 3], rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
    };
    let last = OptimizedKeyframe {
        source_timestamp_ns: scans.last().unwrap().source_timestamp_ns as i64,
        translation_m: [47.0 * 0.002, 0.03, 0.0], rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
    };
    (scans, vec![first, last])
}

fn new_builder(scans: &[ArchivedScan], trajectory: &[OptimizedKeyframe]) -> ReplayBuilder {
    ReplayBuilder::new(scans, trajectory.to_vec(), Isometry3::identity(), five_cm_spec(),
        five_cm_config(), "map".into(), Duration::from_secs(120), &|| false).unwrap()
}

fn cloud_snapshot(cloud: &CommittedCloudMapSnapshot) -> Value {
    // 每个私有mmap新建时生成随机producer_epoch；仅此字段不作跨实例字节比较。
    // 下方另断言同实例prepare→finish的epoch保持。其余云身份、布局和XYZ全部比较。
    assert_ne!(cloud.producer_epoch, [0; 16]);
    json!({"layout_generation": cloud.layout_generation, "revision": cloud.revision,
        "point_count": cloud.point_count, "voxel_size_bits": cloud.voxel_size.to_bits(),
        "session_generation": cloud.session_generation, "xyz_f32_le": cloud.xyz_f32_le})
}

fn rebuilt_snapshot(rebuilt: &RebuiltMap) -> Value {
    json!({"source_timestamp_ns": rebuilt.source_timestamp_ns,
        "persistent": grid_snapshot(&rebuilt.persistent.grid), "cloud": cloud_snapshot(&rebuilt.cloud),
        "backend_checkpoint": rebuilt.backend_checkpoint})
}

#[test]
fn ray_count_pgo_interval_zero_continuous_frames_and_parallel_slices() {
    let (scans, trajectory) = replay_fixture();
    let mut builder = new_builder(&scans, &trajectory);
    let mut per_frame = Vec::new();
    for (index, scan) in scans.iter().enumerate() {
        builder.append(std::slice::from_ref(scan), &|| false).unwrap();
        assert_eq!(builder.processed, index + 1);
        assert_eq!(builder.latest_timestamp_ns(), scan.source_timestamp_ns);
        // 每帧间隔仅1ms。误用在线33ms会留下dirty并使这次真实checkpoint导出失败。
        let checkpoint = mapping_backend::export_mapping_kernel_checkpoint(&builder.kernel)
            .expect("PGO append_kernel_stage interval=0必须逐帧完成候选");
        let state = kernel_snapshot(&builder.kernel);
        assert_eq!(state["dirty_since_candidate"], false);
        assert_eq!(state["last_candidate_received_at_ns"], json!(scan.received_at_ns));
        assert_eq!(state["last_candidate_point_count"], state["healthy_voxels"]);
        let input = builder.last_input.as_ref().unwrap();
        let source = scan.source.lidar().unwrap().source_identity();
        per_frame.push(json!({
            "processed": builder.processed, "kernel": state, "checkpoint": checkpoint,
            "persistent": grid_snapshot(&builder.persistent.as_ref().unwrap().grid),
            "input": {"source_backend": source.backend, "runtime_generation": source.runtime_generation,
                "source_sequence": source.source_sequence, "source_timestamp_ns": source.source_timestamp_ns,
                "frame_sequence": input.frame_sequence, "frame_id": input.registered_frame_id,
                "origin_bits": input.sensor_origin_map.map(f64::to_bits),
                "points_xyz_bits": input.registered_points_map.iter()
                    .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect::<Vec<_>>()},
        }));
    }
    let prepared = builder.prepare(&|| false).unwrap();
    let epoch = prepared.cloud.producer_epoch;
    let expected = builder.finish(prepared, &|| false).unwrap();
    assert_eq!(expected.cloud.producer_epoch, epoch);
    assert_eq!(expected.cloud.voxel_size.to_bits(), 0.05_f32.to_bits());
    let expected_snapshot = rebuilt_snapshot(&expected);

    let mut sliced = new_builder(&scans, &trajectory).into_background();
    sliced.force_parallel_lanes = true;
    let mut cursor = 0;
    while cursor < scans.len() {
        let end = (cursor + 7).min(scans.len());
        let (next, processed) = sliced.append_slice(&scans[cursor..end], &|| false).unwrap();
        assert!(processed > 0 && processed <= end - cursor);
        sliced = next;
        cursor += processed;
        assert_eq!(sliced.processed, cursor);
        assert!(sliced.cloud_worker.is_none());
        assert!(sliced.cloud.is_some());
    }
    let prepared = sliced.prepare(&|| false).unwrap();
    let epoch = prepared.cloud.producer_epoch;
    let actual = sliced.finish(prepared, &|| false).unwrap();
    assert_eq!(actual.cloud.producer_epoch, epoch);
    assert_eq!(rebuilt_snapshot(&actual), expected_snapshot,
        "分片及并行点云lane不能改变同输入PGO业务输出");
    write_business_snapshot("ray-count-pgo-continuous", &json!({
        "input_frame_count": scans.len(), "source_interval_ns": 1_000_000,
        "per_frame": per_frame, "final": expected_snapshot,
    }));
}

#[test]
fn ray_count_pgo_cancel_does_not_consume_the_next_frame() {
    let (scans, trajectory) = replay_fixture();
    let mut builder = new_builder(&scans, &trajectory).into_background();
    builder.append(&scans[..3], &|| false).unwrap();
    let checkpoint = mapping_backend::export_mapping_kernel_checkpoint(&builder.kernel).unwrap();
    let before = kernel_snapshot(&builder.kernel);
    let error = builder.append(&scans[3..], &|| true).unwrap_err();
    assert_eq!(error, RebuildError::Cancelled);
    assert_eq!(builder.processed, 3);
    assert_eq!(builder.latest_timestamp_ns(), scans[2].source_timestamp_ns);
    assert_eq!(kernel_snapshot(&builder.kernel), before);
    assert_eq!(mapping_backend::export_mapping_kernel_checkpoint(&builder.kernel).unwrap(), checkpoint);
    // 后续显式恢复输入，仍从第4帧继续；前缀不会重放。
    builder.append(&scans[3..4], &|| false).unwrap();
    assert_eq!(builder.processed, 4);
    assert_eq!(builder.latest_timestamp_ns(), scans[3].source_timestamp_ns);
    let prepared = builder.prepare(&|| false).unwrap();
    let result = builder.finish(prepared, &|| false).unwrap();
    write_business_snapshot("ray-count-pgo-cancel-resume", &json!({
        "cancel_error": error.to_string(), "before": before, "before_checkpoint": checkpoint,
        "after_resume": rebuilt_snapshot(&result),
    }));
}
