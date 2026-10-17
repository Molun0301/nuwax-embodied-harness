//! 旧核与候选优化共用的真实入口业务快照；不依赖将被删除的派生缓存字段。
use super::*;
use serde_json::{json, Value};

pub(crate) fn five_cm_spec() -> MappingBackendSpec {
    MappingBackendSpec::from_lookup(MappingBackendKind::Raytracing, |key| {
        (key == RAY_VOXEL_SIZE_ENV).then(|| "0.05".to_owned())
    }).unwrap()
}

pub(crate) fn five_cm_config() -> MappingBackendComputeConfig {
    MappingBackendComputeConfig {
        resolution: 0.05,
        initial_safe_radius_meters: 0.0,
        window_capacity: 16_384,
    }
}

pub(crate) fn write_business_snapshot(name: &str, value: &Value) {
    if let Some(directory) = std::env::var_os("NEH_RAY_COUNT_REFERENCE_DIR") {
        let path = std::path::PathBuf::from(directory).join(format!("{name}.json"));
        let expected: Value = serde_json::from_slice(&std::fs::read(&path).unwrap()).unwrap();
        assert_eq!(&expected, value, "业务快照与基线不一致：{}", path.display());
    }
    if let Some(directory) = std::env::var_os("NEH_RAY_COUNT_SNAPSHOT_DIR") {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join(format!("{name}.json"));
        std::fs::write(&path, serde_json::to_vec_pretty(value).unwrap()).unwrap();
        eprintln!("ray_count_snapshot name={name} path={}", path.display());
    }
}

pub(crate) fn grid_snapshot(grid: &OccupancyGrid) -> Value {
    json!({
        "width": grid.width, "height": grid.height,
        "resolution_bits": grid.resolution.to_bits(),
        "origin_bits": grid.origin.map(f64::to_bits),
        "orientation_bits": grid.origin_orientation.map(f64::to_bits),
        "frame_id": grid.frame_id, "timestamp_bits": grid.ts.to_bits(), "cells": grid.data,
    })
}

pub(crate) fn kernel_snapshot(kernel: &MappingKernel) -> Value {
    let MappingKernel::Raytracing(ray) = kernel else { panic!("必须使用真实射线内核"); };
    json!({
        "voxel_size_bits": ray.config.voxel_size.to_bits(),
        "stored_voxels": ray.map.voxels.len(), "healthy_voxels": ray.map.healthy_count(),
        "last_candidate_point_count": ray.last_candidate_point_count,
        "last_candidate_received_at_ns": ray.last_candidate_received_at_ns,
        "dirty_since_candidate": ray.dirty_since_candidate,
    })
}

fn candidate_snapshot(candidate: &MappingBackendCandidate) -> Value {
    json!({
        "session_id": candidate.session_id,
        "session_generation": candidate.session_generation,
        "backend_generation": candidate.backend_generation,
        "source_frame_sequence": candidate.source_frame_sequence,
        "source_timestamp_ns": candidate.source_timestamp_ns,
        "received_at_ns": candidate.received_at_ns,
        "backend_kind": candidate.backend_kind.stable_name(),
        "live": grid_snapshot(&candidate.live_window_occupancy),
        "latest": grid_snapshot(&candidate.latest_frame_occupancy),
        "accepted_xyz_bits": candidate.accepted_points_map.iter()
            .map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect::<Vec<_>>(),
        // compute_elapsed_us 是唯一被排除的候选字段。
    })
}

fn frame(sequence: u64, timestamp_ns: u64) -> MappingBackendInput {
    let points = (0..6).flat_map(|x| (0..6).map(move |y| {
        Point3::new(1.025 + x as f64 * 0.05, -0.125 + y as f64 * 0.05, 0.0)
    })).collect();
    MappingBackendInput {
        session_id: "ray-count-five-cm".into(), session_generation: 1, backend_generation: 1,
        frame_sequence: sequence, source_timestamp_ns: timestamp_ns, received_at_ns: timestamp_ns,
        registered_frame_id: "map".into(), sensor_origin_map: [0.025, 0.025, 0.0],
        registered_points_map: points,
    }
}

fn token_for(input: &MappingBackendInput) -> MappingComputeToken {
    MappingComputeToken {
        token_id: u128::from(input.frame_sequence), session_id: input.session_id.clone(),
        session_generation: input.session_generation, backend_generation: input.backend_generation,
        source_frame_sequence: input.frame_sequence, source_timestamp_ns: input.source_timestamp_ns,
        prepared_at_ns: input.received_at_ns, base_occupancy_revision: 0,
    }
}

#[test]
fn ray_count_positive_zero_negative_use_the_export_predicate() {
    use nuwax_mapping_raytracing::voxel_ray_tracer::Voxel;
    let mut map = VoxelMap::default();
    for (key, health) in [((1, 2, 3), 1), ((4, 5, 6), 0), ((7, 8, 9), -1)] {
        map.voxels.insert(key, Voxel::with_health(health));
    }
    let exported = nuwax_mapping_raytracing::voxel_ray_tracer::iter_global_points(&map, 0.05)
        .map(|(x, y, z)| [x.to_bits(), y.to_bits(), z.to_bits()]).collect::<Vec<_>>();
    assert_eq!(map.voxels.len(), 3);
    assert_eq!(map.healthy_count(), 1);
    assert_eq!(exported.len(), map.healthy_count());
    write_business_snapshot("ray-count-health-predicate", &json!({
        "stored_voxels": map.voxels.len(), "healthy_voxels": map.healthy_count(),
        "exported_xyz_bits": exported,
    }));
}

#[test]
fn ray_count_compute_skip_flush_checkpoint_restore_continue() {
    let spec = five_cm_spec();
    let config = five_cm_config();
    let mut kernel = spec.into_kernel(config.window_capacity);
    let mut observations = Vec::new();
    for sequence in 1..=3 {
        let input = frame(sequence, 1_000_000_000 + sequence * 50_000_000);
        validate_backend_input(&kernel, &input).unwrap();
        let candidate = compute_candidate_in_place(&mut kernel, &input, config).unwrap().unwrap();
        observations.push(json!({"kind": "candidate", "candidate": candidate_snapshot(&candidate),
            "kernel": kernel_snapshot(&kernel)}));
    }
    assert!(kernel_snapshot(&kernel)["healthy_voxels"].as_u64().unwrap() > 0,
        "至少三帧真实命中后必须存在健康历史，不能用空物化掩盖问题");
    let skipped = frame(4, 1_151_000_000);
    assert!(compute_candidate_in_place(&mut kernel, &skipped, config).unwrap().is_none());
    let dirty_error = export_mapping_kernel_checkpoint(&kernel).unwrap_err();
    assert!(dirty_error.contains("flushed"));
    let before_wrong_flush = kernel_snapshot(&kernel);
    let mut wrong = token_for(&skipped);
    wrong.source_frame_sequence += 1;
    let wrong_flush_error = flush_candidate(&mut kernel, &skipped, &wrong, config).unwrap_err();
    assert_eq!(kernel_snapshot(&kernel), before_wrong_flush);
    observations.push(json!({"kind": "skipped", "kernel": before_wrong_flush,
        "export_error": dirty_error, "wrong_flush_error": wrong_flush_error}));

    let flush_token = token_for(&skipped);
    let candidate = flush_candidate(&mut kernel, &skipped, &flush_token, config).unwrap().unwrap();
    observations.push(json!({"kind": "flush", "candidate": candidate_snapshot(&candidate),
        "kernel": kernel_snapshot(&kernel)}));
    let checkpoint = export_mapping_kernel_checkpoint(&kernel).unwrap();
    let mut restored = restore_mapping_kernel_checkpoint(spec, config, &checkpoint).unwrap();
    assert_eq!(export_mapping_kernel_checkpoint(&restored).unwrap(), checkpoint);
    let before_clean_flush = kernel_snapshot(&restored);
    assert!(flush_candidate(&mut restored, &skipped, &flush_token, config).unwrap().is_none());
    assert_eq!(kernel_snapshot(&restored), before_clean_flush);

    let continued = frame(5, 1_201_000_000);
    let expected = compute_candidate_in_place(&mut kernel, &continued, config).unwrap().unwrap();
    let actual = compute_candidate_in_place(&mut restored, &continued, config).unwrap().unwrap();
    assert_eq!(candidate_snapshot(&actual), candidate_snapshot(&expected));
    let continued_checkpoint = export_mapping_kernel_checkpoint(&kernel).unwrap();
    assert_eq!(export_mapping_kernel_checkpoint(&restored).unwrap(), continued_checkpoint);
    observations.push(json!({"kind": "continued", "candidate": candidate_snapshot(&actual),
        "kernel": kernel_snapshot(&restored)}));

    let mut empty = frame(6, 1_251_000_000);
    empty.registered_points_map.clear();
    assert!(compute_candidate_in_place(&mut restored, &empty, config).unwrap().is_none());
    assert_eq!(export_mapping_kernel_checkpoint(&restored).unwrap(), continued_checkpoint);
    let mut invalid = frame(6, 1_251_000_000);
    invalid.registered_points_map[0].x = f64::NAN;
    let invalid_error = validate_backend_input(&restored, &invalid)
        .and_then(|()| compute_candidate_in_place(&mut restored, &invalid, config)).unwrap_err();
    assert_eq!(export_mapping_kernel_checkpoint(&restored).unwrap(), continued_checkpoint);

    // 真实 in-place 构图失败后保留dirty并退出；不借测试用clone包装伪造回滚语义。
    let mut rejected = restore_mapping_kernel_checkpoint(spec, config, &continued_checkpoint).unwrap();
    let previous = kernel_snapshot(&rejected);
    let mut invalid_config = config;
    invalid_config.resolution = 0.0;
    let invalid_resolution_error = compute_candidate_in_place(&mut rejected,
        &frame(6, 1_251_000_000), invalid_config).unwrap_err();
    let rejected_state = kernel_snapshot(&rejected);
    assert_eq!(rejected_state["last_candidate_point_count"], previous["last_candidate_point_count"]);
    assert_eq!(rejected_state["last_candidate_received_at_ns"], previous["last_candidate_received_at_ns"]);
    assert_eq!(rejected_state["dirty_since_candidate"], true);
    assert!(export_mapping_kernel_checkpoint(&rejected).is_err());
    write_business_snapshot("ray-count-kernel-lifecycle", &json!({
        "observations": observations, "checkpoint": checkpoint,
        "continued_checkpoint": continued_checkpoint, "invalid_input_error": invalid_error,
        "invalid_resolution_error": invalid_resolution_error, "failed_kernel": rejected_state,
    }));
}

#[test]
fn ray_count_restore_preserves_serialized_count_until_next_candidate() {
    let spec = five_cm_spec();
    let config = five_cm_config();
    let mut kernel = spec.into_kernel(config.window_capacity);
    let mut latest = frame(1, 1_050_000_000);
    for sequence in 1..=3 {
        latest = frame(sequence, 1_000_000_000 + sequence * 50_000_000);
        compute_candidate_in_place(&mut kernel, &latest, config).unwrap().unwrap();
    }
    let MappingKernel::Raytracing(ray) = &mut kernel else { unreachable!() };
    let healthy = ray.map.healthy_count();
    // 旧协议只要求该历史标量未超过容量，恢复不得偷偷重算为当前健康数。
    ray.last_candidate_point_count = healthy + 7;
    let checkpoint = export_mapping_kernel_checkpoint(&kernel).unwrap();
    let mut restored = restore_mapping_kernel_checkpoint(spec, config, &checkpoint).unwrap();
    assert_eq!(export_mapping_kernel_checkpoint(&restored).unwrap(), checkpoint);
    let before = kernel_snapshot(&restored);
    assert_eq!(before["last_candidate_point_count"], json!(healthy + 7));
    assert!(flush_candidate(&mut restored, &latest, &token_for(&latest), config).unwrap().is_none());
    assert_eq!(kernel_snapshot(&restored), before);
    let next = frame(4, 1_200_000_000);
    let candidate = compute_candidate_in_place(&mut restored, &next, config).unwrap().unwrap();
    let after = kernel_snapshot(&restored);
    assert_eq!(after["last_candidate_point_count"], after["healthy_voxels"]);
    write_business_snapshot("ray-count-restore-original-counter", &json!({
        "before": before, "after": after, "restored_checkpoint": checkpoint,
        "continued_candidate": candidate_snapshot(&candidate),
        "continued_checkpoint": export_mapping_kernel_checkpoint(&restored).unwrap(),
    }));
}

#[test]
fn ray_count_interval_zero_commits_every_continuous_frame() {
    let config = five_cm_config();
    let mut kernel = five_cm_spec().into_kernel(config.window_capacity);
    let mut observations = Vec::new();
    for sequence in 1..=48 {
        let input = frame(sequence, 1_000_000_000 + sequence * 1_000_000);
        // 相邻1ms，不允许误换成在线33ms节拍而漏掉大部分PGO帧。
        let candidate = compute_candidate_in_place_with_interval(&mut kernel, &input, config, 0)
            .unwrap().expect("interval=0必须逐帧生成候选");
        assert_eq!(candidate.source_frame_sequence, sequence);
        let state = kernel_snapshot(&kernel);
        assert_eq!(state["last_candidate_point_count"], state["healthy_voxels"]);
        observations.push(json!({"candidate": candidate_snapshot(&candidate), "kernel": state}));
    }
    assert!(kernel_snapshot(&kernel)["healthy_voxels"].as_u64().unwrap() > 0);
    write_business_snapshot("ray-count-interval-zero", &json!({
        "frames": observations, "checkpoint": export_mapping_kernel_checkpoint(&kernel).unwrap(),
    }));
}
