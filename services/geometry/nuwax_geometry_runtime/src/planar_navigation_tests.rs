use super::*;
use crate::{mapper::MapperConfig, proto};
fn session() -> Session {
    Session {
        session_id: "51".into(),
        map_name: "二维地图".into(),
        session_generation: 51,
        sensor_id: "laser".into(),
        frame_id: "laser".into(),
        map_frame_id: "map".into(),
        odom_frame_id: "odom".into(),
        body_frame_id: "base_link".into(),
        clock_domain: "unix_ns".into(),
        clock_id: String::new(),
        calibration_ref: "laser".into(),
        odometry_source: OdometrySource::Device,
        core_config: serde_json::json!({"base_from_laser":[0.,0.,0.]}),
    }
}
fn scan(stamp: u64, sequence: i64, range: f32) -> PlanarScan {
    PlanarScan {
        runtime_generation: 1,
        sensor_id: "laser".into(),
        source_sequence: sequence,
        timestamp_ns: stamp as i64,
        clock_domain: "unix_ns".into(),
        clock_id: String::new(),
        frame_id: "laser".into(),
        calibration_ref: "laser".into(),
        beam_count: 1,
        angular_layout: 1,
        angle_min_rad: 0.,
        angle_increment_rad: 1.,
        range_min_m: 0.1,
        range_max_m: 10.,
        scan_duration_ns: 0,
        timestamp_quality: 1,
        time_offsets_quality: 0,
        timing_uncertainty_ns: -1,
        motion_compensated: false,
        ranges_f32_le: range.to_le_bytes().to_vec(),
        angles_f32_le: Vec::new(),
        beam_states_u8: vec![1],
        intensities_f32_le: Vec::new(),
        time_offsets_ns_le: Vec::new(),
    }
}
fn sample(stamp: u64, x: f64) -> TimedPose {
    TimedPose {
        timestamp_ns: stamp,
        clock_domain: "unix_ns".into(),
        clock_id: String::new(),
        odom_frame_id: "odom".into(),
        body_frame_id: "base_link".into(),
        pose: [x, 0.5, 0.],
    }
}
#[tokio::test]
async fn current_scan_clears_moved_obstacle_and_preserves_unobserved_cells() {
    let mapper = Mapper::start(
        51,
        MapperConfig {
            backend: crate::mapping_backend::MappingBackendSpec::SlamToolbox,
            ..Default::default()
        },
    )
    .unwrap();
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;
    let mut data = vec![255; 60];
    data[3] = 100;
    data[13] = 100;
    mapper.feed_grid(
        51,
        proto::OccupancyGrid {
            map_name: "二维地图".into(),
            meta: Some(proto::OccupancyGridMeta {
                width: 20,
                height: 3,
                resolution: 1.,
                frame_id: "map".into(),
                timestamp_ns: stamp - 100_000_000,
                ..Default::default()
            }),
            data: data.clone(),
        },
    );
    mapper.snapshot().await.unwrap();
    let settings = Settings {
        template: session(),
        binary: "/bin/true".into(),
        data_root: "/tmp".into(),
        unix_offset_ns: 0,
    };
    let navigation = Owner::default();
    navigation.odometry(sample(stamp, 0.5)).unwrap();
    navigation.capture(scan(stamp, 7, 6.));
    let estimate = Estimate {
        disposition: "accepted".into(),
        source_timestamp_ns: stamp,
        source_sequence: 7,
        graph_revision: 1,
        odom_from_base: [0.5, 0.5, 0.],
        ..Default::default()
    };
    let observation = navigation
        .compose(&mapper, &settings, &session(), &estimate)
        .unwrap()
        .unwrap();
    assert_eq!(
        observation.grid.data[3], 0,
        "本帧真实自由射线必须清除已移动障碍"
    );
    assert_eq!(observation.grid.data[6], 100);
    assert_eq!(observation.grid.data[13], 100, "射线未观测到的单元必须保留");
    assert_eq!(
        mapper.occupancy(false).unwrap().data,
        data,
        "派生局部观测不得改物理图"
    );
    assert_eq!(observation.source_timestamp_ns, stamp as i64);
    assert_eq!(
        observation.map_source_timestamp_ns,
        stamp as i64 - 100_000_000
    );
    assert_eq!(
        planar_navigation::decode_body(&planar_navigation::encode_body(&observation).unwrap())
            .unwrap(),
        observation
    );
    mapper.shutdown().await.unwrap();
}
#[test]
fn fresh_device_scan_uses_real_history_without_fabricating_native_match() {
    let navigation = Owner::default();
    let basis = Estimate {
        disposition: "accepted".into(),
        source_timestamp_ns: 1_000_000_000,
        source_sequence: 1,
        graph_revision: 9,
        odom_from_base: [1., 0., 0.],
        ..Default::default()
    };
    navigation.capture(scan(1_100_000_000, 2, 3.));
    assert!(navigation.source(&session(), &basis).unwrap().is_none());
    navigation.odometry(sample(1_000_000_000, 1.)).unwrap();
    navigation.odometry(sample(1_200_000_000, 3.)).unwrap();
    let (selected, pose) = navigation.source(&session(), &basis).unwrap().unwrap();
    assert_eq!(selected.source_sequence, 2);
    assert_eq!(pose, [2., 0.5, 0.]);
    assert_eq!(basis.source_sequence, 1);
    assert_eq!(basis.source_timestamp_ns, 1_000_000_000);
    assert_eq!(basis.graph_revision, 9);
    let mut rejected = basis;
    rejected.disposition = "match_rejected".into();
    assert!(navigation.source(&session(), &rejected).unwrap().is_none());
    navigation.clear();
    assert!(navigation.source(&session(), &rejected).unwrap().is_none());
}
