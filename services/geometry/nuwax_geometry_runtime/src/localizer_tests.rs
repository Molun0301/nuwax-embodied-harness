use super::*;

fn room() -> Vec<Point3<f64>> {
    let mut points = Vec::new();
    for x in 0..47 { for y in 0..33 {
        let (x, y) = (-3.0 + x as f64 * 0.13, -2.0 + y as f64 * 0.13);
        points.extend([Point3::new(x, y, -1.0), Point3::new(x, y, 1.73)]);
    }}
    for z in 0..22 {
        let z = -1.0 + z as f64 * 0.13;
        for x in 0..47 { let x = -3.0 + x as f64 * 0.13; points.extend([Point3::new(x, -2.0, z), Point3::new(x, 2.16, z)]); }
        for y in 0..33 { let y = -2.0 + y as f64 * 0.13; points.extend([Point3::new(-3.0, y, z), Point3::new(2.98, y, z)]); }
        for u in 0..5 { points.push(Point3::new(1.9, 0.8 + u as f64 * 0.13, z)); }
    }
    points
}
fn frame(map: &[Point3<f64>], truth: Isometry3<f64>, odom: Isometry3<f64>, stamp: u64) -> MappingFrame {
    MappingFrame {
        session_id: 1, odom_pose: pose_from_isometry(odom, "odom", stamp),
        points_body: map.iter().map(|p| { let p = truth.inverse() * p; [p.x, p.y, p.z] }).collect(),
        ray_origin_body: [0.0; 3], mapping_only: false, sensor_id: "合成雷达".into(), registered_cloud: None,
    }
}

#[test]
fn sixty_rounds_follow_drift_and_report_room_and_large_map_timings() {
    let points = room();
    let target = TargetCache::new(&points, Default::default());
    let truth = Isometry3::from_parts(Translation3::new(0.4, -0.3, 0.2), UnitQuaternion::from_euler_angles(0.0, 0.0, 0.3));
    let mut correction = Isometry3::identity();
    let mut maximum_translation: f64 = 0.0;
    let mut maximum_angle: f64 = 0.0;
    for second in 1..=60 {
        let odom = Isometry3::from_parts(Translation3::from(truth.translation.vector + Vector3::new(0.02 * second as f64, 0.0, 0.0)),
            truth.rotation * UnitQuaternion::from_euler_angles(0.0, 0.0, (0.1 * second as f64).to_radians()));
        let round = compute_round(&[frame(&points, truth, odom, second * 1_000_000_000)], correction, &target).unwrap();
        correction = round.correction.expect("房间扫描应通过评分");
        let error = truth.inverse() * correction * odom;
        maximum_translation = maximum_translation.max(error.translation.vector.norm());
        maximum_angle = maximum_angle.max(error.rotation.angle().to_degrees());
        assert!(maximum_translation <= 0.05, "第{second}轮平移={maximum_translation}");
        assert!(maximum_angle <= 1.0, "第{second}轮转角={maximum_angle}");
        if second == 1 { eprintln!("普通房间：源={} 30米目标={} 配准={:.3}ms 评分={:.3}ms", round.source_points, round.target_points, round.registration_ms, round.scoring_ms); }
    }
    let untracked = Vector3::new(0.02 * 60.0, 0.0, 0.0).norm();
    assert!(untracked > maximum_translation);
    eprintln!("60轮：跟踪最大平移={maximum_translation:.6}m 最大角度={maximum_angle:.6}deg；不开启60秒平移={untracked:.3}m 偏航=6deg");
    let mut large = points.clone();
    for x in -60..=60 { for y in -60..=60 {
        large.extend([Point3::new(x as f64 * 0.5, y as f64 * 0.5, -1.0), Point3::new(x as f64 * 0.5, y as f64 * 0.5, 1.73)]);
    }}
    let target = TargetCache::new(&large, Default::default());
    let odom = Isometry3::from_parts(Translation3::from(truth.translation.vector + Vector3::new(0.02, 0.0, 0.0)), truth.rotation * UnitQuaternion::from_euler_angles(0.0, 0.0, 0.1_f64.to_radians()));
    let round = compute_round(&[frame(&points, truth, odom, 1)], Isometry3::identity(), &target).unwrap();
    assert!(round.correction.is_some());
    eprintln!("60米方形地图：全图目标={} 源={} 30米目标={} 配准={:.3}ms 评分={:.3}ms", target.points.len(), round.source_points, round.target_points, round.registration_ms, round.scoring_ms);
}

#[test]
fn corridor_keeps_unconstrained_translation_and_corrects_across_walls() {
    let points: Vec<_> = (-150..=150).flat_map(|x| (-10..=20).flat_map(move |z|
        [Point3::new(x as f64 * 0.13, -2.0, z as f64 * 0.13), Point3::new(x as f64 * 0.13, 2.0, z as f64 * 0.13)])).collect();
    let target = TargetCache::new(&points, Default::default());
    let truth = Isometry3::identity();
    let odom = Isometry3::translation(0.08, 0.04, 0.0);
    // 长地图只观测机器人附近两面墙，避免远处贴墙掠射线的体素量化混入退化方向测试。
    let scan: Vec<_> = points.iter().filter(|p| p.x.abs() <= 2.5).copied().collect();
    let round = compute_round(&[frame(&scan, truth, odom, 1)], Isometry3::identity(), &target).unwrap();
    let correction = round.correction.unwrap();
    assert!(correction.translation.x.abs() < 1e-9, "不约束方向被改动：{correction:?}");
    assert!(correction.translation.y < -0.019, "垂直方向应修正：{correction:?}");
}
