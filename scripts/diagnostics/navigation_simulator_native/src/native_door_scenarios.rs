//! New full-simulator fixtures, not a rerun or repair of the historical 1 cm
//! narrow/wide-door helper. All native planning and execution remain unchanged.
use crate::{GroundEvidence, Scene, SurfaceNavigationConfig, Terrain};
use serde_json::{json, Value};
use std::{collections::HashMap, path::PathBuf};

const SOURCE_VOXEL_M: f32 = 0.05;
const DOOR_CENTER_Y_CELL: i32 = 3;
// Exact historical request coordinates are retained, not rounded to this new map.
const REQUEST_CENTER_Y_M: f64 = 0.1649999963119626;
const REQUEST_WALL_CENTER_X_M: f64 = 0.019999999552965164;

struct DoorFixture {
    id: &'static str,
    name: &'static str,
    width_cells: i32,
}

const FIXTURES: [DoorFixture; 3] = [
    DoorFixture {
        id: "native-door-050-v1",
        name: "新增完整原生模拟 · 0.50米门",
        width_cells: 10,
    },
    DoorFixture {
        id: "native-door-080-v1",
        name: "新增完整原生模拟 · 0.80米门",
        width_cells: 16,
    },
    DoorFixture {
        id: "native-door-100-v1",
        name: "新增完整原生模拟 · 1.00米门",
        width_cells: 20,
    },
];

impl DoorFixture {
    fn aperture_cells(&self) -> [i32; 2] {
        [
            DOOR_CENTER_Y_CELL - self.width_cells / 2,
            DOOR_CENTER_Y_CELL + self.width_cells / 2,
        ]
    }

    fn points(&self) -> Vec<(f32, f32, f32)> {
        let q = f64::from(SOURCE_VOXEL_M);
        let point = |x: i32, y: i32, z: i32| {
            let p = [x, y, z].map(|v| ((f64::from(v) + 0.5) * q) as f32);
            (p[0], p[1], p[2])
        };
        let [lower, upper] = self.aperture_cells();
        let mut points: Vec<_> = (-20..20)
            .flat_map(|x| (-10..20).map(move |y| point(x, y, -1)))
            .collect();
        // One 5 cm source column is the closest representable width to the old
        // 4 cm wall. The 80 cm height and floor extents retain their old values.
        for y in -10..20 {
            if y >= lower && y < upper {
                continue;
            }
            for z in 0..16 {
                points.push(point(0, y, z));
            }
        }
        points
    }

    fn cases(&self) -> Vec<Value> {
        [0.0_f64, 5.0]
            .into_iter()
            .map(|degrees| {
                let heading = degrees.to_radians();
                let pose = |x: f64| {
                    [
                        x,
                        REQUEST_CENTER_Y_M + (x - REQUEST_WALL_CENTER_X_M) * heading.tan(),
                        0.4,
                        heading,
                    ]
                };
                json!({"id": format!("{}-yaw-{:02}", self.id, degrees as u32),
                "name": format!("{} · {}°原起终点", self.name, degrees),
                "start": pose(-0.5), "goal": pose(0.5), "expected_reachable": true,
                "requires_orientation": true, "local_planning": true,
                "fixture_id": self.id, "fixture_generation": "new_native_005m_door_v1",
                "request_coordinates": "historical_aligned_door_request_unchanged",
                "nominal_door_width_m": f64::from(self.width_cells) * 0.05,
                "source_voxel_m": q(), "new_wall_thickness_m": q(),
                "historical_wall_thickness_m": 4.0 * f64::from(0.01_f32)})
            })
            .collect()
    }

    fn make_unmodified_scene(
        &self,
        config: &SurfaceNavigationConfig,
        data: &PathBuf,
    ) -> Result<Scene, String> {
        Scene::make(
            self.id,
            self.name,
            self.points(),
            Vec::new(),
            config.clone(),
            true,
            GroundEvidence::OccupiedVolume,
            data,
        )
    }
}

fn q() -> f64 {
    f64::from(SOURCE_VOXEL_M)
}

/// Only this new fixture declares full occupied wall columns to the unchanged
/// world judge. Scene::make's legacy exposed-top response is not solid geometry.
/// No map samples, navigation inputs, old scenes or collision math are changed.
fn declare_solid_world(scene: &mut Scene) -> Result<(), String> {
    let body = scene
        .config
        .body_envelope
        .as_ref()
        .ok_or("新增门场景缺少原机体标定")?;
    scene.terrain = Terrain::new(
        &scene.points,
        true,
        &body.footprint_xy_m,
        SOURCE_VOXEL_M,
        [body.max_step_up_m, body.max_step_down_m],
    );
    Ok(())
}

pub(super) fn register(
    config: &SurfaceNavigationConfig,
    data: &PathBuf,
    scenes: &mut HashMap<String, Scene>,
    catalog: &mut Value,
) -> Result<(), String> {
    for fixture in &FIXTURES {
        let mut scene = fixture.make_unmodified_scene(config, data)?;
        declare_solid_world(&mut scene)?;
        scene.cases = fixture.cases();
        let body = config
            .body_envelope
            .as_ref()
            .ok_or("新增门场景缺少原机体标定")?;
        let extent = |axis: usize| {
            let values = body.footprint_xy_m.iter().map(|p| p[axis]);
            [
                values.clone().fold(f64::INFINITY, f64::min),
                values.fold(f64::NEG_INFINITY, f64::max),
            ]
        };
        let x = extent(0);
        let y = extent(1);
        let aperture = fixture.aperture_cells().map(|v| f64::from(v) * q());
        scene.metadata["description"] = json!("新增完整原生模拟门场景；5厘米源占据格、原Go2标定、原生规划跟随与独立机身扫掠。不是旧1厘米helper复测。");
        scene.metadata["default_start"] = scene.cases[0]["start"].clone();
        scene.metadata["default_goal"] = scene.cases[0]["goal"].clone();
        scene.metadata["ground_evidence"] = json!("OccupiedVolume");
        scene.metadata["native_door_fixture"] = json!({
            "schema": "new-native-door-fixture-v1", "fixture_id": fixture.id,
            "historical_helper_unchanged": true,
            "historical_helper_is_this_result": false,
            "nominal_door_width_m": f64::from(fixture.width_cells) * 0.05,
            "source_voxel_m": q(),
            "surface_map_voxel_m": f64::from(config.geometry.voxel_size),
            "aperture_y_bounds_m": aperture,
            "effective_door_width_m": aperture[1] - aperture[0],
            "door_center_y_m": f64::from(DOOR_CENTER_Y_CELL) * q(),
            "wall_x_bounds_m": [0.0, q()], "wall_z_bounds_m": [0.0, 16.0 * q()],
            "floor_xy_bounds_m": [-20.0 * q(), 20.0 * q(), -10.0 * q(), 20.0 * q()],
            "historical_wall_thickness_m": 4.0 * f64::from(0.01_f32),
            "historical_door_center_y_m": REQUEST_CENTER_Y_M,
            "request_coordinates": "historical_aligned_door_request_unchanged",
            "body_calibration": "unmodified_runtime_config_not_historical_helper_rectangle",
            "body_length_m": x[1] - x[0], "body_width_m": y[1] - y[0],
            "body_x_bounds_m": x, "body_y_bounds_m": y, "body_z_bounds_m": body.z_bounds_m,
            "body_to_support_z_m": body.body_to_support_z_m,
            "independent_judge": "unchanged_Scene_validate_world_motion_and_world_motion_check_swept_boxes",
            "independent_wall_input": "new_fixture_full_occupied_column_bands",
            "legacy_exposed_top_collapse_applied_to_world_solids": false,
            "acceptance_protocol": {"wall_budget_sec": 180, "simulation_budget_sec": 180,
                "dt_sec": crate::DT, "position_tolerance_m": 0.35, "yaw_tolerance_rad": 0.15,
                "stop_linear_mps": crate::STOP_LINEAR_MPS,
                "stop_angular_rps": crate::STOP_ANGULAR_RPS,
                "additional_full_passage_check": "minimum_world_x_of_full_body_strictly_greater_than_wall_x_high",
                "arrival_inside_position_tolerance_alone_is_not_full_passage": true}
        });
        catalog["scenes"]
            .as_array_mut()
            .ok_or("场景目录不是数组")?
            .push(scene.metadata.clone());
        scenes.insert(fixture.id.into(), scene);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn diagnostic_config() -> (SurfaceNavigationConfig, &'static str) {
        if std::env::var("NUWAX_ROBOT_GEOMETRY_JSON").is_ok_and(|value| !value.trim().is_empty()) {
            return (
                SurfaceNavigationConfig::from_env().unwrap(),
                "unchanged_launcher_runtime_calibration",
            );
        }
        let mut config =
            SurfaceNavigationConfig::new("map".into(), 0.4, 0.328, 0.6, 0.1, 0.15, 18.0, 0.2)
                .unwrap();
        // Synthetic fixture-contract body only; runtime scenes keep the full
        // actual Go2 convex hull loaded by the unchanged launcher.
        config
            .apply_calibration(
                Some(nuwax_nav_types_messages::RobotGeometry {
                    self_collision: None,
                    footprint_xy_m: vec![
                        [-0.2394, -0.164],
                        [0.34, -0.164],
                        [0.34, 0.164],
                        [-0.2394, 0.164],
                    ],
                    z_min_m: -0.448,
                    z_max_m: 0.057,
                    source: "new-door-validator-contract-test-only".into(),
                }),
                None,
            )
            .unwrap();
        (config, "synthetic_contract_body_not_navigation_acceptance")
    }

    #[test]
    fn new_native_door_points_and_requests_are_fixed_and_explicitly_distinct() {
        for fixture in &FIXTURES {
            let points = fixture.points();
            assert_eq!(
                points.len(),
                1200 + (30 - fixture.width_cells) as usize * 16
            );
            let [lo, hi] = fixture.aperture_cells();
            let cells: Vec<_> = points
                .iter()
                .map(|&(x, y, z)| [x, y, z].map(|v| (f64::from(v) / q()).floor() as i32))
                .collect();
            assert!(cells
                .iter()
                .filter(|p| p[2] >= 0)
                .all(|p| p[0] == 0 && (p[1] < lo || p[1] >= hi) && p[2] < 16));
            for (request, heading) in fixture
                .cases()
                .into_iter()
                .zip([0.0_f64, 5.0_f64.to_radians()])
            {
                assert_eq!(request["start"][0], -0.5);
                assert_eq!(request["goal"][0], 0.5);
                assert_eq!(request["start"][2], 0.4);
                assert_eq!(request["goal"][2], 0.4);
                assert_eq!(
                    request["start"][3].as_f64().unwrap().to_bits(),
                    heading.to_bits()
                );
                assert_eq!(
                    request["goal"][3].as_f64().unwrap().to_bits(),
                    heading.to_bits()
                );
                assert_eq!(request["requires_orientation"], true);
                assert_eq!(request["local_planning"], true);
            }
        }
        assert_eq!(FIXTURES.iter().map(|f| f.cases().len()).sum::<usize>(), 6);
    }

    #[test]
    fn same_input_native_door_validator_before_after_diagnostic() {
        let (config, body_source) = diagnostic_config();
        let directory =
            std::env::temp_dir().join(format!("new-native-door-diagnostic-{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let mut evidence = Vec::new();
        for fixture in &FIXTURES {
            let mut scene = fixture.make_unmodified_scene(&config, &directory).unwrap();
            let original_points = scene.points.clone();
            let original_map = std::sync::Arc::clone(&scene.map);
            let wall_body = ([0.025, -0.4, 0.4], [0.0, 0.0, 0.0, 1.0]);
            let doorway_body = ([0.025, 0.15, 0.4], [0.0, 0.0, 0.0, 1.0]);
            let before_wall = scene.validate_world_motion(wall_body, wall_body);
            let before_door = scene.validate_world_motion(doorway_body, doorway_body);
            let before_band = scene.terrain.columns.get(&(0, -8)).unwrap().clone();
            // Diagnostic-only after state: the map, source points, poses, body,
            // ground evidence and validator are unchanged. This restores the
            // solid occupied columns declared by this new fixture's own points.
            declare_solid_world(&mut scene).unwrap();
            let after_wall = scene.validate_world_motion(wall_body, wall_body);
            let after_door = scene.validate_world_motion(doorway_body, doorway_body);
            let after_band = scene.terrain.columns.get(&(0, -8)).unwrap().clone();
            assert_eq!(scene.points, original_points);
            assert!(std::sync::Arc::ptr_eq(&scene.map, &original_map));
            assert_eq!(scene.ground_evidence, GroundEvidence::OccupiedVolume);
            assert!(
                before_wall.is_ok(),
                "must reproduce original false-negative before changing the fixture"
            );
            assert!(before_door.is_ok());
            assert!(
                after_wall.is_err(),
                "same independent judge must reject the full solid wall"
            );
            assert!(
                after_door.is_ok(),
                "same independent judge must preserve the real opening"
            );
            let body = config.body_envelope.as_ref().unwrap();
            evidence.push(json!({"fixture_id":fixture.id, "same_points_exact":scene.points==original_points,
                "same_navigation_map_identity":std::sync::Arc::ptr_eq(&scene.map,&original_map),
                "point_count_before":original_points.len(), "point_count_after":scene.points.len(),
                "same_wall_pose_before_after":wall_body, "same_doorway_pose_before_after":doorway_body,
                "wall_band_before":before_band, "wall_band_after":after_band,
                "wall_accepted_before":before_wall.is_ok(), "wall_accepted_after":after_wall.is_ok(),
                "wall_error_after":after_wall.err(), "doorway_accepted_before":before_door.is_ok(),
                "doorway_accepted_after":after_door.is_ok(),
                "body_source":body_source,
                "body_footprint_xy_m":body.footprint_xy_m,"body_z_bounds_m":body.z_bounds_m}));
            let _ = std::fs::remove_file(directory.join(format!("{}.bin", fixture.id)));
        }
        eprintln!(
            "NEW_NATIVE_DOOR_SAME_INPUT_VALIDATOR_DIAGNOSTIC={}",
            json!(evidence)
        );
        let _ = std::fs::remove_dir(directory);
    }

    #[test]
    fn registered_native_doors_keep_runtime_body_and_original_simulator_contract() {
        let (config, _) = diagnostic_config();
        let original_body = config.body_envelope.as_ref().unwrap();
        let directory =
            std::env::temp_dir().join(format!("new-native-door-register-{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        let sentinel = json!({"id":"existing_scene_do_not_change"});
        let mut catalog =
            json!({"scenes":[sentinel],"default_scene":"existing_scene_do_not_change"});
        let mut scenes = HashMap::new();
        register(&config, &directory, &mut scenes, &mut catalog).unwrap();
        assert_eq!(catalog["scenes"][0], sentinel);
        assert_eq!(catalog["default_scene"], "existing_scene_do_not_change");
        assert_eq!(catalog["scenes"].as_array().unwrap().len(), 4);
        assert_eq!(scenes.len(), 3);
        for fixture in &FIXTURES {
            let scene = &scenes[fixture.id];
            assert_eq!(scene.cases, fixture.cases());
            assert_eq!(scene.points, fixture.points());
            assert_eq!(
                scene.config.body_envelope.as_ref().unwrap().footprint_xy_m,
                original_body.footprint_xy_m
            );
            assert_eq!(
                scene.config.body_envelope.as_ref().unwrap().z_bounds_m,
                original_body.z_bounds_m
            );
            assert_eq!(scene.config.geometry.voxel_size, config.geometry.voxel_size);
            let wall_body = ([0.025, -0.4, 0.4], [0.0, 0.0, 0.0, 1.0]);
            let door_body = ([0.025, 0.15, 0.4], [0.0, 0.0, 0.0, 1.0]);
            assert!(scene.validate_world_motion(wall_body, wall_body).is_err());
            assert!(scene.validate_world_motion(door_body, door_body).is_ok());
            for request in &scene.cases {
                let simulation = crate::Simulation::new(scene, request).unwrap();
                assert_eq!(simulation.options.position_tolerance_m, 0.35);
                assert_eq!(simulation.options.yaw_tolerance_rad, 0.15);
                assert!(simulation.local_planning);
                assert_eq!(simulation.goal, crate::body4(&request["goal"]).unwrap());
                let requested_start = crate::body4(&request["start"]).unwrap();
                assert_eq!(simulation.current.0, requested_start.0);
                // The existing flat-world initializer reconstructs q from
                // yaw(unit_orientation(q)), not the input quaternion's bits.
                // At 5 degrees that trigonometric round trip changes z by one
                // ULP. Match this documented conversion exactly, with no epsilon.
                assert!(scene.terrain.flat);
                let expected_orientation = crate::quaternion(0.0, 0.0, crate::yaw(requested_start));
                assert_eq!(
                    simulation.current.1.map(f64::to_bits),
                    expected_orientation.map(f64::to_bits)
                );
            }
            let _ = std::fs::remove_file(directory.join(format!("{}.bin", fixture.id)));
        }
        assert_eq!(crate::DT, 0.05);
        assert_eq!(crate::STOP_LINEAR_MPS, 0.005);
        assert_eq!(crate::STOP_ANGULAR_RPS, 0.01);
        let _ = std::fs::remove_dir(directory);
    }
}
