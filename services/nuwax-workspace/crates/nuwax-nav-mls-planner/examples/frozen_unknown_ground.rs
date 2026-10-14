//! Read-only replay of the supplied root365 archived fine geometry and body pose.
//! No simulator floor, scoring coordinates, live service, actuator, or model calls.
//! Pass the archived input directory, then optional --require-pass to assert the
//! new unknown-ground contract. Baseline diagnostics remain in the supplied log.
use nuwax_nav_mls_planner::{SurfaceBodyEnvelope, mls_planner::Config};
use nuwax_nav_mls_planner::{
    SurfaceSnapshot,
    surface::{BodyRejection, OccupancySource, SurfaceObservation, SurfaceObservations},
};
use std::{collections::BTreeMap, sync::Arc};
struct DebugLog;
impl tracing::Subscriber for DebugLog {
    fn enabled(&self, metadata: &tracing::Metadata<'_>) -> bool {
        metadata.level() <= &tracing::Level::DEBUG
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::span::Id {
        tracing::span::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::span::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::span::Id, _: &tracing::span::Id) {}
    fn enter(&self, _: &tracing::span::Id) {}
    fn exit(&self, _: &tracing::span::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        struct Fields(String);
        impl tracing::field::Visit for Fields {
            fn record_debug(&mut self, field: &tracing::field::Field, value: &dyn std::fmt::Debug) {
                use std::fmt::Write;
                let _ = write!(self.0, " {}={value:?}", field.name());
            }
        }
        let mut fields = Fields(String::new());
        event.record(&mut fields);
        eprintln!(
            "TRACE {}:{} {}",
            event.metadata().file().unwrap_or("?"),
            event.metadata().line().unwrap_or(0),
            fields.0
        );
    }
}
type BodyPose = ([f64; 3], [f64; 4]);

#[allow(dead_code)]
fn main() {
    run_replay(|_, _, _, _| None);
}

/// Shared diagnostic driver; optional downstream qualification uses the exact
/// same decoded immutable snapshot and requested departure as the MLS probe.
pub fn run_replay(
    mut downstream: impl FnMut(
        &SurfaceSnapshot,
        BodyPose,
        BodyPose,
        &serde_json::Value,
    ) -> Option<Result<(), String>>,
) {
    let dir = std::path::PathBuf::from(std::env::args().nth(1).unwrap());
    let original_files: Vec<_> = ["精细地图.bin", "净空环境变量.json", "pose.json"]
        .into_iter()
        .map(|name| (name, std::fs::read(dir.join(name)).unwrap()))
        .collect();
    let load = |name: &str| -> serde_json::Value {
        serde_json::from_slice(&std::fs::read(dir.join(name)).unwrap()).unwrap()
    };
    let _ = tracing::subscriber::set_global_default(DebugLog);
    let env = load("净空环境变量.json");
    let g: serde_json::Value =
        serde_json::from_str(env["values"]["NUWAX_ROBOT_GEOMETRY_JSON"].as_str().unwrap()).unwrap();
    let model = SurfaceBodyEnvelope {
        footprint_xy_m: serde_json::from_value(g["footprint_xy_m"].clone()).unwrap(),
        z_bounds_m: [
            g["z_min_m"].as_f64().unwrap(),
            g["z_max_m"].as_f64().unwrap(),
        ],
        body_to_support_z_m: env["values"]["NUWAX_NAV_BODY_TO_SUPPORT_Z_M"]
            .as_str()
            .unwrap()
            .parse()
            .unwrap(),
        max_step_up_m: 0.,
        max_step_down_m: 0.,
        max_cross_slope_deg: 18.,
        support_uncertainty_m: 0.,
    };
    let cfg = Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: 0.1,
        surface_closing_radius: 0.,
        node_spacing_m: 0.3,
        wall_clearance_m: 0.,
        wall_buffer_m: (model.radius() + 0.1) as f32,
        wall_buffer_weight: 2.,
        step_threshold_m: 0.,
        step_penalty_weight: 2.,
        max_slope_deg: 18.,
        min_step_tread_depth_m: 0.2,
        goal_tolerance: 0.2,
        viz_publish_hz: 0.,
    };
    let mut archive = nuwax_map_chunks::ArchiveReader::open(
        std::fs::File::open(dir.join("精细地图.bin")).unwrap(),
    )
    .unwrap();
    let root = match archive.root() {
        nuwax_map_chunks::ArchivedRoot::Current(root) => root.clone(),
        _ => panic!("本诊断针对新版精细根"),
    };
    let fine = root.voxel_size_m;
    let mut obs = SurfaceObservations::new(fine, 32).unwrap();
    let mut points = Vec::new();
    let mut layers = 0;
    let mut original_blocks = BTreeMap::new();
    for reference in &root.blocks {
        let bytes = archive.block(reference).unwrap();
        let block = nuwax_map_chunks::decode_block(&bytes).unwrap();
        let measured = nuwax_map_chunks::decode_layer_observations(&bytes).unwrap();
        layers += measured.len();
        for layer in measured.values() {
            let members: Vec<_> = layer
                .members
                .iter()
                .map(|&member| nuwax_map_chunks::global_key(reference.key, member).unwrap())
                .collect();
            if members
                .iter()
                .any(|key| *key == [5, -5, -19] || *key == [5, -3, -19])
            {
                println!(
                    "WITNESS_LAYER {}",
                    serde_json::json!({"members":members,"plane_origin":layer.plane_origin,"normal":layer.normal,"samples":layer.samples.len()})
                );
            }
        }
        let mut values = BTreeMap::new();
        for (local, e) in block {
            let key = nuwax_map_chunks::global_key(reference.key, local).unwrap();
            points.push((
                (key[0] as f32 + 0.5) * fine,
                (key[1] as f32 + 0.5) * fine,
                (key[2] as f32 + 0.5) * fine,
            ));
            values.insert(
                key,
                SurfaceObservation {
                    observed_ns: e.observed_ns,
                    hits: e.hits,
                },
            );
        }
        original_blocks.insert(reference.key, values.clone());
        obs.replace_block(reference.key, values).unwrap();
        obs.replace_ground_layers(reference.key, Arc::new(measured))
            .unwrap();
    }
    obs.mark_complete_geometry_source();
    for column in [[5, -5], [5, -3]] {
        println!(
            "WITNESS_COLUMN {:?} measured_layers={}",
            column,
            obs.ground_layers(column).count()
        );
    }
    println!(
        "ROOT={} points={} measured_layers={layers}",
        root.revision,
        points.len()
    );
    let p = load("pose.json");
    let body: [f64; 3] = serde_json::from_value(p["position"].clone()).unwrap();
    let q: [f64; 4] = serde_json::from_value(p["quaternion"].clone()).unwrap();
    assert_eq!(
        (root.revision, points.len(), layers),
        (365, 88558, 1320),
        "wrong frozen input set"
    );
    let source = SurfaceSnapshot::from_points(&points, cfg, fine)
        .unwrap()
        .with_observations(Some(Arc::new(obs.clone())));
    let original_source = source.clone();
    println!("GROUND_EVIDENCE {:?}", source.ground_evidence());
    let require_pass = std::env::args().any(|arg| arg == "--require-pass");
    let mut failures = 0;
    for uncertainty in [0.0, 0.02] {
        let mut model = model.clone();
        model.support_uncertainty_m = uncertainty;
        let map = source.with_body_envelope(model.clone()).unwrap();
        let mut witness = None;
        println!("QUERY body={body:?} q={q:?} uncertainty={uncertainty}");
        let pose_check = map.validate_body_pose_with_witness(body, q, &mut witness);
        println!("POSE_CHECK {:?} WITNESS {:?}", pose_check, witness);
        println!(
            "SUPPORT_POSITION {:?}",
            map.support_position_from_body(body, q, model.body_to_support_z_m)
        );
        let view = map.with_current_body_support(body, q);
        println!("CURRENT_SUPPORT {:?}", view.as_ref().map(|_| ()));
        let same_geometry = view.as_ref().ok().map(|view| map.same_geometry(view));
        println!("RETURNED_VIEW_SHARES_SOURCE_GEOMETRY {:?}", same_geometry);
        let reference = map
            .support_position_from_body(body, q, model.body_to_support_z_m)
            .unwrap();
        println!(
            "GROUND_QUERY {:?}",
            map.query_ground([reference[0], reference[1]], reference[2])
        );
        if pose_check.is_err() || view.is_err() || same_geometry != Some(true) {
            failures += 1;
        }
        // Synthetic requested motion, not a captured task goal or simulator-world
        // target: move 2/5/10 cm along the archived body's own current heading.
        // Every probe restarts from the unchanged original body position/quaternion.
        let yaw =
            (2.0 * (q[3] * q[2] + q[0] * q[1])).atan2(1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]));
        for distance in [0.02, 0.05, 0.10] {
            let xy = [
                body[0] + distance * yaw.cos(),
                body[1] + distance * yaw.sin(),
            ];
            let mut departure_witness = None;
            let projected =
                map.body_pose_after_motion_with_witness((body, q), xy, yaw, &mut departure_witness);
            println!(
                "OFFLINE_REQUESTED_DEPARTURE distance_m={distance} projection={projected:?} witness={departure_witness:?}"
            );
            match projected {
                Ok(to) => {
                    let mut sweep_witness = None;
                    let swept = map.validate_body_motion_with_witness(
                        (body, q),
                        to,
                        &[],
                        &mut sweep_witness,
                    );
                    println!(
                        "OFFLINE_REQUESTED_DEPARTURE distance_m={distance} swept={swept:?} witness={sweep_witness:?}"
                    );
                    if swept.is_err() {
                        failures += 1;
                    }
                    if let Some(result) = downstream(&map, (body, q), to, &env) {
                        println!(
                            "OFFLINE_REQUESTED_DEPARTURE distance_m={distance} downstream={result:?}"
                        );
                        if result.is_err() {
                            failures += 1;
                        }
                    }
                }
                Err(_) => failures += 1,
            }
        }
        // Deliberately synthetic negative controls, separate from the unchanged archive
        // replay: known cabinet and tabletop occupancy above its real observed floor.
        for (label, keys) in [
            (
                "cabinet",
                (-4_i32..=4)
                    .flat_map(|y| (-18..=6).map(move |z| [5, y, z]))
                    .collect::<Vec<_>>(),
            ),
            (
                "tabletop",
                (5_i32..=8)
                    .flat_map(|x| (-4_i32..=4).map(move |y| [x, y, -7]))
                    .collect::<Vec<_>>(),
            ),
        ] {
            let mut altered = obs.clone();
            let mut blocks = original_blocks.clone();
            let mut altered_points = points.clone();
            for &key in &keys {
                blocks
                    .entry(key.map(|v| v.div_euclid(32)))
                    .or_default()
                    .insert(
                        key,
                        SurfaceObservation {
                            observed_ns: root.source_timestamp_ns,
                            hits: 1,
                        },
                    );
                altered_points.push((
                    (key[0] as f32 + 0.5) * fine,
                    (key[1] as f32 + 0.5) * fine,
                    (key[2] as f32 + 0.5) * fine,
                ));
            }
            for (key, values) in blocks {
                altered.replace_block(key, values).unwrap();
            }
            altered.mark_complete_geometry_source();
            let counterfactual =
                SurfaceSnapshot::from_points(&altered_points, map.config().clone(), fine)
                    .unwrap()
                    .with_observations(Some(Arc::new(altered)))
                    .with_body_envelope(model.clone())
                    .unwrap();
            let mut obstacle_witness = None;
            let result =
                counterfactual.validate_body_pose_with_witness(body, q, &mut obstacle_witness);
            println!(
                "COUNTERFACTUAL_SYNTHETIC_{label} {:?} WITNESS {:?}",
                result, obstacle_witness
            );
            let rejects_inserted_voxel =
                obstacle_witness.is_some_and(|witness| match witness.rejection {
                    BodyRejection::Occupancy { cell, .. } => keys.contains(&cell),
                });
            println!("COUNTERFACTUAL_SYNTHETIC_{label}_INSERTED_VOXEL {rejects_inserted_voxel}");
            if result.is_ok() || !rejects_inserted_voxel {
                failures += 1;
            }
        }
        let recent = map
            .with_recent_obstacle_points(1, root.source_timestamp_ns, 0, &[], [(0, body)])
            .unwrap();
        let mut recent_witness = None;
        let result = recent.validate_body_pose_with_witness(body, q, &mut recent_witness);
        println!(
            "COUNTERFACTUAL_SYNTHETIC_recent_torso {:?} WITNESS {:?}",
            result, recent_witness
        );
        let rejects_recent = recent_witness.is_some_and(|witness| {
            matches!(
                witness.rejection,
                BodyRejection::Occupancy {
                    source: OccupancySource::Recent,
                    ..
                }
            )
        });
        if result.is_ok() || !rejects_recent {
            failures += 1;
        }
    }
    let files_unchanged = original_files
        .iter()
        .all(|(name, bytes)| std::fs::read(dir.join(name)).unwrap() == *bytes);
    let observations_unchanged = source.observations().unwrap().same_geometry(&obs);
    let source_geometry_unchanged = source.same_geometry(&original_source);
    println!("ORIGINAL_SOURCE_GEOMETRY_UNCHANGED {source_geometry_unchanged}");
    println!("INPUT_FILES_BYTE_IDENTICAL {files_unchanged}");
    println!("SOURCE_OBSERVATION_GEOMETRY_UNCHANGED {observations_unchanged}");
    println!(
        "SOURCE_COUNTS_UNCHANGED points={} measured_layers={}",
        points.len(),
        layers
    );
    assert!(files_unchanged && observations_unchanged && source_geometry_unchanged);
    assert_eq!((points.len(), layers), (88558, 1320));
    if require_pass {
        assert_eq!(
            failures, 0,
            "frozen root365 unknown-ground contract still rejects"
        );
    }
}
