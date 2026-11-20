use super::*;
use nalgebra::Vector3;

fn corridor_frame(step: u64, x: f64, drift: f64) -> (MappingFrame, Vec<usize>) {
    let sensor = Point3::new(x + 0.03, 0.03, 0.65);
    let mut points = Vec::new();
    let mut walls = Vec::new();
    for row in 0..31 {
        let elevation = -0.4 + row as f64 * 0.025;
        for col in 0..121 {
            let azimuth = -1.3 + col as f64 * 2.6 / 120.;
            let direction = Vector3::new(elevation.cos() * azimuth.cos(),
                elevation.cos() * azimuth.sin(), elevation.sin());
            let mut distance = (12. - sensor.x) / direction.x;
            if direction.z < 0. { distance = distance.min(-sensor.z / direction.z); }
            if direction.z > 0. { distance = distance.min((3. - sensor.z) / direction.z); }
            if direction.y != 0. {
                distance = distance.min(((if direction.y > 0. { 3. } else { -3. }) - sensor.y) / direction.y);
            }
            let point = sensor + direction * distance;
            if (point.y.abs() - 3.).abs() < 1e-8 { walls.push(points.len()); }
            points.push([point.x - x, point.y, point.z]);
        }
    }
    (MappingFrame {
        session_id: 961, odom_pose: pose_from_isometry(Isometry3::translation(x, drift, 0.), "odom", step * 1_000_000_000),
        points_body: points, ray_origin_body: [0.03, 0.03, 0.65],
        mapping_only: false, sensor_id: "走廊雷达".into(), registered_cloud: None,
    }, walls)
}

struct Corridor {
    /// 去程、重放回程、采用新地图后继续送回程帧三段的墙面点动态比例。
    ratios: [f64; 3],
    walls: [usize; 3],
    flags: Vec<Vec<bool>>,
}

/// 沿走廊走出去再回来：去程无漂移，回程里程计带横向漂移，校正把回程拉回。
/// 前两段按重放处理（每帧的证据坐标取该帧校正），第三段按采用新地图后的在线帧处理（取最新校正）。
/// `corrected` 为假时证据始终留在原始里程计坐标，即旧做法，只作对照。
fn run_corridor(drift: f64, corrected: bool) -> Corridor {
    let mut map = CloudMap::new(961, None, None, &MappingConfirmationConfig::single_frame_for_test()).unwrap();
    map.disable_recording();
    let mut counts = [(0usize, 0usize); 3];
    let mut flags = Vec::new();
    let mut step = 0;
    for phase in 0..3 {
        let positions: Vec<_> = match phase {
            0 => (0..25).map(|i| i as f64 * 0.2).collect(),
            1 => (0..25).rev().map(|i| i as f64 * 0.2).collect(),
            _ => (0..10).map(|i| i as f64 * 0.05).collect(),
        };
        for x in positions {
            step += 1;
            let current_drift = if phase == 0 { 0. } else { drift };
            let (frame, walls) = corridor_frame(step, x, current_drift);
            let correction = Isometry3::translation(0., -current_drift, 0.);
            let evidence = if corrected { correction } else { Isometry3::identity() };
            let body = correction * pose_isometry(&frame.odom_pose);
            let points: Vec<_> = frame.points_body.iter().map(|p| body * Point3::from(*p)).collect();
            let origin = body * Point3::from(frame.ray_origin_body);
            let observation = map.accumulate(step, &frame, &points,
                [origin.x, origin.y, origin.z], correction, evidence).unwrap();
            let frame_flags: Vec<_> = points.iter().map(|p| observation.dynamic.is_dynamic(p.coords.into())).collect();
            counts[phase].0 += walls.iter().filter(|i| frame_flags[**i]).count();
            counts[phase].1 += walls.len();
            flags.push(frame_flags);
        }
    }
    Corridor { ratios: counts.map(|c| c.0 as f64 / c.1 as f64), walls: counts.map(|c| c.1), flags }
}

fn ratios_json(run: &Corridor) -> serde_json::Value {
    serde_json::json!({ "去程": run.ratios[0], "重放回程": run.ratios[1], "采用后继续": run.ratios[2],
        "墙面点数": run.walls })
}

#[test]
fn corrected_evidence_keeps_revisited_walls_static_after_odometry_drift() {
    let still_before = run_corridor(0., false);
    let still_after = run_corridor(0., true);
    let drift_before = run_corridor(0.5, false);
    let drift_after = run_corridor(0.5, true);
    assert_eq!(still_before.flags, still_after.flags, "没有漂移时改前改后去动态结果相同");
    assert!(drift_before.ratios[1] > 0.2 && drift_before.ratios[2] > 0.1, "对照：旧做法把漂移回程的墙面判为动态");
    for phase in 1..3 {
        assert!(drift_after.ratios[phase] <= drift_after.ratios[0] + 0.01,
            "回程墙面动态比例应接近去程：{:?}", drift_after.ratios);
    }
    let result = serde_json::json!({
        "无漂移": { "改前": ratios_json(&still_before), "改后": ratios_json(&still_after) },
        "横向漂移半米": { "改前": ratios_json(&drift_before), "改后": ratios_json(&drift_after) },
    });
    eprintln!("G3走廊动态比例：{result}");
    if let Some(root) = std::env::var_os("NUWAX_G3_METRICS_DIR") {
        let root = PathBuf::from(root);
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join("走廊动态比例.json"), serde_json::to_vec_pretty(&result).unwrap()).unwrap();
    }
}
