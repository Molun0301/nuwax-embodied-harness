// Production cache boundary tests and an opt-in direct-function diagnostic.
// Included by surface_dataflow::tests; no transport, controller, or actuator.
use super::*;
use nuwax_map_chunks::{Repository, Transfer, Writer, MAX_BLOCK_BYTES};
use nuwax_nav_runtime::native_map::{NativeMap,from_surface};
use std::collections::BTreeSet;

const SOURCE_NS: i64 = 1_700_000_000_000_000_000;

fn calibrated_config() -> SurfaceNavigationConfig {
    // Make a new owner: cloning this configuration would share the PCT cache.
    let mut result = SurfaceNavigationConfig::new(
        "map".into(), 0.4, 0.3, 0.6, 0.1, 0.15, 18.0, 0.2,
    ).unwrap();
    result.apply_calibration(Some(nuwax_nav_types_messages::RobotGeometry {
        self_collision: None,
        footprint_xy_m: vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
        z_min_m: -0.05,
        z_max_m: 0.2,
        source: "mask-reuse synthetic fixture".into(),
    }), None).unwrap();
    result
}

fn set_source_clock(message: &mut schema::NavigationMap3D, timestamp_ns: i64) {
    message.source_timestamp_ns = timestamp_ns;
    message.committed_map.source_timestamp_ns = timestamp_ns;
    message.cloud.header.stamp_sec = (timestamp_ns / 1_000_000_000) as i32;
    message.cloud.header.stamp_nsec = (timestamp_ns % 1_000_000_000) as i32;
}

fn flat_message(width: i32, height: i32, session: &str) -> schema::NavigationMap3D {
    let mut message = map_message(1, session);
    set_source_clock(&mut message, SOURCE_NS);
    message.cloud.data.clear();
    for x in 0..width {
        for y in 0..height {
            for value in [(x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.05] {
                message.cloud.data.extend_from_slice(&value.to_le_bytes());
            }
        }
    }
    message.cloud.width = width * height;
    message.cloud.row_step = width * height * 12;
    let mut mask = virtual_mask(&message, false);
    mask.info.width = width;
    mask.info.height = height;
    mask.data = vec![0; (width * height) as usize];
    message.virtual_obstacles = Some(mask);
    schema::validate_navigation_map_3d(&message).unwrap();
    message
}

// Retain cloud receipt, source timestamp and physical revision. Only occupancy
// authority advances; cloning the fixture occurs before any measured work.
fn next_occupancy(previous: &schema::NavigationMap3D) -> schema::NavigationMap3D {
    let mut next = previous.clone();
    next.committed_map.revision += 1;
    next.committed_map.commit_sequence += 1;
    next.committed_map.source_timestamp_ns += 10;
    if let Some(mask) = next.virtual_obstacles.as_mut() {
        mask.header.seq = next.committed_map.revision as i32;
        mask.header.stamp_sec = (next.committed_map.source_timestamp_ns / 1_000_000_000) as i32;
        mask.header.stamp_nsec = (next.committed_map.source_timestamp_ns % 1_000_000_000) as i32;
    }
    next
}

fn route_request(message: &schema::NavigationMap3D, wide: bool, height: f64)
    -> (PoseStamped, schema::GoalEnvelope)
{
    let (from_x, to_x, y) = if wide { (1.0, 7.0, 3.0) } else { (0.8, 3.2, 1.5) };
    let mut goal = goal_for(message, 1, [to_x, y, height]);
    goal.options = nuwax_nav_types_messages::GoalOptions {
        allow_reverse: false, position_tolerance_m: 0.12, yaw_tolerance_rad: 0.15,
    };
    goal.source_timestamp_ns = SOURCE_NS;
    goal.goal.header.stamp_sec = (SOURCE_NS / 1_000_000_000) as i32;
    goal.goal.header.stamp_nsec = 0;
    goal.owner_kind=schema::NAVIGATION_OWNER_KIND_EXPLORATION_CHILD;
    goal.boundary_frame_id="map".into();
    goal.boundary_id="mask-test".into();
    let (width,height)=if wide {(8.,6.)} else {(4.,3.)};
    goal.boundary_points=vec![[0.,0.,0.],[width,0.,0.],[width,height,0.],[0.,height,0.]];
    schema::validate_goal_envelope(&goal).unwrap();
    let mut start = pose_from_schema(&goal.goal).unwrap();
    start.pose.position.x = from_x;
    (start, goal)
}

type CellBits=([i32;2],u8,[u64;2]);
fn native_input(surface:&SurfaceSnapshot)->Result<NativeMap,String> {from_surface(surface)}
fn canonical_cells(map:&NativeMap)->Vec<CellBits> {
    let mut records:Vec<_>=map.cells.iter().map(|cell|([cell.x,cell.y],0,[cell.ground,cell.ceiling].map(f64::to_bits)))
        .chain(map.bands.iter().map(|band|([band.x,band.y],1,[band.low,band.high].map(f64::to_bits))))
        .chain(map.forbidden_columns.iter().map(|&column|(column,2,[0;2]))).collect();
    records.sort_unstable();records
}

fn canonical_columns(surface: &SurfaceSnapshot) -> Vec<([i32; 2], Vec<[u64; 2]>)> {
    let mut columns: Vec<_> = surface.navigation_height_columns().map(|(key, bands)| {
        let mut bands: Vec<_> = bands.into_iter().map(|band| band.map(f64::to_bits)).collect();
        bands.sort_unstable();
        (key, bands)
    }).collect();
    columns.sort_unstable();
    columns
}

fn path_bits(path: &Path3D) -> Vec<([u64; 3], [u64; 4])> {
    path.poses.iter().map(|pose| {
        let p = pose.pose.position;
        let q = pose.pose.orientation;
        ([p.x,p.y,p.z].map(f64::to_bits), [q.x,q.y,q.z,q.w].map(f64::to_bits))
    }).collect()
}

// Independent from every incremental owner: consume only the current complete
// message in a fresh production cache and a separately constructed PCT owner.
fn assert_current_message_oracle(
    actual: &SurfaceSnapshot,
    message: &schema::NavigationMap3D,
    configured: &SurfaceNavigationConfig,
    compare_routes: bool,
) {
    let cold_config = calibrated_config();
    let cold = SurfaceGeometryCache::configured(&cold_config).build(message).unwrap();
    assert_eq!(actual.cell_keys().collect::<BTreeSet<_>>(), cold.cell_keys().collect());
    assert_eq!(canonical_columns(actual), canonical_columns(&cold));
    let actual_pct = native_input(actual).unwrap();
    let cold_pct = native_input(&cold).unwrap();
    assert_eq!(actual_pct.resolution.to_bits(), cold_pct.resolution.to_bits());
    assert_eq!(canonical_cells(&actual_pct), canonical_cells(&cold_pct));
    if compare_routes {
        let height = if message.fine.is_some() { 0.45 } else { 0.5 };
        let (start, goal) = route_request(message, message.cloud.width > 1200, height);
        let actual_body = body_path(actual, &start, &goal, configured).unwrap();
        let cold_body = body_path(&cold, &start, &goal, &cold_config).unwrap();
        assert_eq!(path_bits(&actual_body), path_bits(&cold_body));
        assert!(actual_body.poses.len() >= 2);
        assert_eq!(actual_body.poses.first().unwrap(), &start);
    }
}

#[test]
fn occupancy_only_same_mask_reuses_surface_and_pct_with_current_receipt() {
    let configured = calibrated_config();
    let mut cache = SurfaceGeometryCache::configured(&configured);
    let first_message = flat_message(40, 30, "same-mask");
    let first_received = Instant::now() - Duration::from_secs(2);
    let first = prepare_surface_map(first_message.clone(), first_received, &configured, &mut cache);
    let old = first.snapshot.as_ref().unwrap();
    let physical = cache.physical_snapshot().unwrap();
    let mut next_message = next_occupancy(&first_message);
    let next_received = first_received + Duration::from_millis(250);
    let next = prepare_surface_map(next_message.clone(), next_received, &configured, &mut cache);
    let current = next.snapshot.as_ref().unwrap();
    assert!(Arc::ptr_eq(old, current));
    assert!(Arc::ptr_eq(&physical, &cache.physical_snapshot().unwrap()));
    assert!(old.same_body_query_view(current));
    assert_eq!(next.authority, map_authority(&next_message));
    assert_ne!(next.authority, first.authority);
    assert_eq!(next.received_at, next_received);
    assert_eq!(first.received_at, first_received);
    assert_eq!(next.source_timestamp_ns, SOURCE_NS);
    assert_eq!(next.authority.cloud_source_timestamp_ns, SOURCE_NS);
    assert_eq!(next.authority.committed_map.source_timestamp_ns, SOURCE_NS + 10);
    assert_eq!(cache.last_build_stats().unwrap().input_points, 0);
    assert_current_message_oracle(current, &next_message, &configured, true);

    // Same occupancy revision still has its existing direct reuse fast path;
    // non-geometric map-load clock metadata is irrelevant to derived geometry.
    next_message.virtual_obstacles.as_mut().unwrap().info.map_load_time_nsec = 7;
    let same_revision = cache.build(&next_message).unwrap();
    assert!(Arc::ptr_eq(current, &same_revision));
    let third_message = next_occupancy(&next_message);
    let third = cache.build(&third_message).unwrap();
    assert!(Arc::ptr_eq(current, &third));
}

#[test]
fn mask_cells_presence_extent_and_layout_bits_use_cold_current_message_geometry() {
    for variant in ["cell", "none", "extent", "dimensions", "resolution_bit", "origin_x",
        "origin_y", "origin_x_sign", "origin_y_sign", "origin_z_sign", "quaternion_sign"]
    {
        let configured = calibrated_config();
        let mut cache = SurfaceGeometryCache::configured(&configured);
        let first_message = flat_message(40, 30, "mask-layout");
        let old = cache.build(&first_message).unwrap();
        let physical = cache.physical_snapshot().unwrap();
        let old_pct = native_input(&old).unwrap();
        let mut next = next_occupancy(&first_message);
        if variant == "none" {
            next.virtual_obstacles = None;
        } else {
            let mask = next.virtual_obstacles.as_mut().unwrap();
            match variant {
                "cell" => mask.data[10 * 40 + 20] = 100,
                "extent" => {
                    mask.info.origin.position = [0.5, 0.5, 0.0];
                    mask.info.width = 30;
                    mask.info.height = 20;
                    mask.data = vec![0; 600];
                },
                "dimensions" => { mask.info.width = 30; mask.info.height = 40; },
                "resolution_bit" => mask.info.resolution = f32::from_bits(mask.info.resolution.to_bits() + 1),
                "origin_x" => mask.info.origin.position[0] = 0.1,
                "origin_y" => mask.info.origin.position[1] = 0.1,
                "origin_x_sign" => mask.info.origin.position[0] = -0.0,
                "origin_y_sign" => mask.info.origin.position[1] = -0.0,
                "origin_z_sign" => mask.info.origin.position[2] = -0.0,
                "quaternion_sign" => mask.info.origin.orientation_xyzw[0] = -0.0,
                _ => unreachable!(),
            }
        }
        schema::validate_navigation_map_3d(&next).unwrap();
        let current = cache.build(&next).unwrap();
        assert!(Arc::ptr_eq(&physical, &cache.physical_snapshot().unwrap()), "{variant}");
        assert!(!old.same_geometry(&current), "{variant} must not reuse the old derived graph");
        let current_pct = native_input(&current).unwrap();
        assert_current_message_oracle(&current, &next, &configured, false);
        assert!(Arc::ptr_eq(&current, &cache.build(&next).unwrap()), "{variant}");
        if variant=="extent" {
            assert_eq!(current_pct.cells.len(),old_pct.cells.len(),"空掩码范围外仍保持未知可规划");
            assert!(current_pct.forbidden_columns.is_empty());
        }
    }
}

#[test]
fn changed_mask_and_removal_restore_current_routes_without_mutating_retained_snapshots() {
    let configured = calibrated_config();
    let mut cache = SurfaceGeometryCache::configured(&configured);
    let mut blocked_message = flat_message(40, 30, "mask-removal");
    for y in 0..30 { blocked_message.virtual_obstacles.as_mut().unwrap().data[y * 40 + 20] = 100; }
    let blocked = cache.build(&blocked_message).unwrap();
    let (start, goal) = route_request(&blocked_message, false, 0.5);
    assert!(body_path(&blocked, &start, &goal, &configured).is_err());
    let blocked_cells = canonical_cells(&native_input(&blocked).unwrap());
    let mut zero_message = next_occupancy(&blocked_message);
    zero_message.virtual_obstacles.as_mut().unwrap().data.fill(0);
    let zero = cache.build(&zero_message).unwrap();
    assert!(!blocked.same_geometry(&zero));
    assert_current_message_oracle(&zero, &zero_message, &configured, true);
    let mut absent_message = next_occupancy(&zero_message);
    absent_message.virtual_obstacles = None;
    let absent = cache.build(&absent_message).unwrap();
    assert!(!zero.same_geometry(&absent));
    assert_current_message_oracle(&absent, &absent_message, &configured, true);
    let mut restored_message = next_occupancy(&absent_message);
    restored_message.virtual_obstacles = Some(virtual_mask(&restored_message, false));
    let restored = cache.build(&restored_message).unwrap();
    assert!(!absent.same_geometry(&restored));
    assert_current_message_oracle(&restored, &restored_message, &configured, true);
    assert_eq!(blocked_cells, canonical_cells(&native_input(&blocked).unwrap()));
    assert!(body_path(&blocked, &start, &goal, &configured).is_err());
}

struct FineFixture {
    repository: Repository,
    writer: Writer,
    previous: BTreeSet<[i32; 3]>,
}
impl FineFixture {
    fn new() -> Self {
        let repository = Repository::temporary().unwrap();
        let writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
        Self { repository, writer, previous: BTreeSet::new() }
    }
    fn message(&mut self, cells: &BTreeSet<[i32; 3]>, revision: i64) -> schema::NavigationMap3D {
        let mut message = map_message(revision, "mask-fine-source");
        set_source_clock(&mut message, SOURCE_NS + revision * 100);
        message.source_voxel_size = 0.05;
        message.cloud.width = 0;
        message.cloud.row_step = 0;
        message.cloud.data.clear();
        self.writer.remove(self.previous.difference(cells).copied()).unwrap();
        self.writer.observe(cells.iter().copied(), message.source_timestamp_ns as u64).unwrap();
        let root = self.writer.snapshot(schema::navigation_map_3d_domain(&message),
            revision as u64, message.source_timestamp_ns as u64).unwrap();
        let included = root.blocks.iter().map(|block| (block.object,
            self.repository.read_object(block.object, MAX_BLOCK_BYTES).unwrap())).collect();
        message.fine = Some(Transfer { root: root.encode().unwrap(), included });
        message.virtual_obstacles = Some(virtual_mask(&message, false));
        self.previous = cells.clone();
        schema::validate_navigation_map_3d(&message).unwrap();
        message
    }
}
fn fine_ground() -> BTreeSet<[i32; 3]> {
    (0..80).flat_map(|x| (0..60).map(move |y| [x,y,0])).collect()
}

#[test]
fn fine_mask_reuse_keeps_original_evidence_and_refresh_retires_only_body_query_view() {
    let configured = calibrated_config();
    let mut cache = SurfaceGeometryCache::configured(&configured);
    let mut fixture = FineFixture::new();
    let cells = fine_ground();
    let first_message = fixture.message(&cells, 1);
    let first = cache.build(&first_message).unwrap();
    let first_observations = Arc::clone(first.observations().unwrap());
    let evidence = first_observations.get([40,30,0]).unwrap();
    let physical = cache.physical_snapshot().unwrap();
    let occupied_message = next_occupancy(&first_message);
    let receipt = Instant::now() - Duration::from_secs(3);
    let occupied = prepare_surface_map(occupied_message.clone(), receipt, &configured, &mut cache);
    let occupied_surface = occupied.snapshot.as_ref().unwrap();
    assert!(Arc::ptr_eq(&first, occupied_surface));
    assert!(Arc::ptr_eq(&first_observations, occupied_surface.observations().unwrap()));
    assert_eq!(occupied_surface.observations().unwrap().get([40,30,0]), Some(evidence));
    assert_eq!(occupied.received_at, receipt);
    assert_eq!(occupied.source_timestamp_ns, first_message.source_timestamp_ns);
    assert_eq!(occupied.authority, map_authority(&occupied_message));
    assert_current_message_oracle(occupied_surface, &occupied_message, &configured, true);

    let refreshed_message = fixture.message(&cells, 3);
    let refreshed = cache.build(&refreshed_message).unwrap();
    assert!(Arc::ptr_eq(&physical, &cache.physical_snapshot().unwrap()));
    assert!(first.same_geometry(&refreshed));
    assert!(!Arc::ptr_eq(&first, &refreshed));
    assert!(!first.same_body_query_view(&refreshed));
    assert!(!Arc::ptr_eq(&first_observations, refreshed.observations().unwrap()));
    assert!(first_observations.same_geometry(refreshed.observations().unwrap()));
    let fresh_evidence = refreshed.observations().unwrap().get([40,30,0]).unwrap();
    assert_eq!(fresh_evidence.observed_ns, refreshed_message.source_timestamp_ns as u64);
    assert!(fresh_evidence.observed_ns > evidence.observed_ns);
    assert!(fresh_evidence.hits > evidence.hits);
    assert_eq!(first_observations.get([40,30,0]), Some(evidence));
    assert_current_message_oracle(&refreshed, &refreshed_message, &configured, true);
}

#[test]
fn fine_geometry_change_inside_one_coarse_cell_invalidates_pct_despite_mask_reuse() {
    let configured = calibrated_config();
    let mut cache = SurfaceGeometryCache::configured(&configured);
    let mut fixture = FineFixture::new();
    let mut cells = fine_ground();
    let first_message = fixture.message(&cells, 1);
    let first = cache.build(&first_message).unwrap();
    let physical = cache.physical_snapshot().unwrap();
    let pct = native_input(&first).unwrap();
    // Three samples in the same 0.1 m coarse column still retain exactly the
    // same coarse occupied cell and height band. The 0.05 m source has a hole.
    assert!(cells.remove(&[10,10,0]));
    let next_message = fixture.message(&cells, 2);
    let next = cache.build(&next_message).unwrap();
    assert!(Arc::ptr_eq(&physical, &cache.physical_snapshot().unwrap()));
    assert!(first.same_geometry(&next));
    assert_eq!(canonical_columns(&first), canonical_columns(&next));
    assert!(!first.same_body_query_view(&next));
    assert!(!first.observations().unwrap().same_geometry(next.observations().unwrap()));
    assert!(first.observations().unwrap().get([10,10,0]).is_some());
    assert!(next.observations().unwrap().get([10,10,0]).is_none());
    let next_pct = native_input(&next).unwrap();
    assert_ne!(canonical_cells(&pct), canonical_cells(&next_pct));
    assert_eq!(pct.bands.len(), next_pct.bands.len() + 1);
    assert_current_message_oracle(&next, &next_message, &configured, true);
}

#[test]
fn reset_clear_and_domain_changes_discard_derived_mask_reuse() {
    for variant in ["reset", "clear", "session", "session_generation", "runtime", "frame_generation", "boot_nonce"] {
        let configured = calibrated_config();
        let mut cache = SurfaceGeometryCache::configured(&configured);
        let first_message = flat_message(40, 30, "mask-reset");
        let first = cache.build(&first_message).unwrap();
        let mut next_message = next_occupancy(&first_message);
        match variant {
            "reset" => cache.reset(),
            "clear" => {
                let mut clear = next_message.clone();
                clear.cleared = true;
                clear.virtual_obstacles = None;
                clear.cloud.width = 0;
                clear.cloud.row_step = 0;
                clear.cloud.data.clear();
                assert!(cache.build(&clear).is_err());
                assert!(cache.last_build_stats().is_none());
                assert!(cache.physical_snapshot().is_none());
                next_message = next_occupancy(&next_message);
            },
            "session" => next_message.committed_map.session_id = "replacement-domain".into(),
            "session_generation" => next_message.committed_map.session_generation += 1,
            "runtime" => next_message.producer_runtime_generation += 1,
            "frame_generation" => next_message.map_frame_generation += 1,
            "boot_nonce" => {
                next_message.commit_boot_nonce = [0x55; 16];
                next_message.committed_map.commit_boot_nonce = [0x55; 16];
            },
            _ => unreachable!(),
        }
        schema::validate_navigation_map_3d(&next_message).unwrap();
        let next = cache.build(&next_message).unwrap();
        assert!(!first.same_geometry(&next), "{variant}");
        assert_current_message_oracle(&next, &next_message, &configured, true);
    }
}

// Append inside the existing surface_dataflow::tests::mask_reuse_tests module.
// Reuses FineFixture/calibrated_config/virtual_mask; no new production API.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn incomplete_fine_retains_present_mask_only_for_same_contents_and_domain() {
    use crate::surface_map::SurfaceMapBuildError;
    use nuwax_map_chunks::Manifest;

    for variant in ["same_mask_new_receipt", "cell", "layout", "domain"] {
        let mut fixture = FineFixture::new();
        let cells = fine_ground();
        let mut first = fixture.message(&cells, 1);
        let mut candidate = fixture.message(&cells, 2);
        // Cover both occupied and free cells in an actually present mask. This
        // is a legal protocol mask, not a claim about Geometry's zones store.
        first.virtual_obstacles.as_mut().unwrap().data[0] = 100;
        candidate.virtual_obstacles.as_mut().unwrap().data[0] = 100;
        let first_mask = first.virtual_obstacles.as_ref().unwrap();
        let next_mask = candidate.virtual_obstacles.as_ref().unwrap();
        assert_eq!(first_mask.data, next_mask.data);
        assert_eq!(first_mask.info.origin, next_mask.info.origin);
        assert_eq!(first_mask.info.resolution, next_mask.info.resolution);
        assert_ne!(first_mask.header.seq, next_mask.header.seq);
        assert_ne!(
            (first_mask.info.map_load_time_sec, first_mask.info.map_load_time_nsec),
            (next_mask.info.map_load_time_sec, next_mask.info.map_load_time_nsec),
        );
        assert!(candidate.committed_map.revision > first.committed_map.revision);
        match variant {
            "same_mask_new_receipt" => {},
            "cell" => candidate.virtual_obstacles.as_mut().unwrap().data[1] = 100,
            "layout" => candidate.virtual_obstacles.as_mut().unwrap().info.origin.position[0] = 0.25,
            "domain" => {
                candidate.committed_map.session_id = "f1-present-mask-new-domain".into();
                // The replacement domain is sealed by the same real Writer,
                // rather than forging a root header that admission would reject.
                let root = fixture.writer.snapshot(
                    schema::navigation_map_3d_domain(&candidate),
                    candidate.cloud_revision as u64,
                    candidate.source_timestamp_ns as u64,
                ).unwrap();
                let included = root.blocks.iter().map(|block| (block.object,
                    fixture.repository.read_object(block.object, MAX_BLOCK_BYTES).unwrap())).collect();
                candidate.fine = Some(Transfer { root: root.encode().unwrap(), included });
            },
            _ => unreachable!(),
        }
        let old_root = Manifest::decode(&first.fine.as_ref().unwrap().root).unwrap();
        let next_root = Manifest::decode(&candidate.fine.as_ref().unwrap().root).unwrap();
        let missing = next_root.blocks.iter().find(|next|
            !old_root.blocks.iter().any(|old| old.object == next.object))
            .expect("real repeated observations must produce an uncached object").object;
        assert!(candidate.fine.as_mut().unwrap().included.remove(&missing).is_some());
        // Both messages traverse real codec/schema admission. Omission is legal;
        // bad cells, dimensions or mismatched root domain must not fake Incomplete.
        schema::validate_navigation_map_3d(&first).unwrap();
        schema::validate_navigation_map_3d(&candidate).unwrap();
        let first = schema::decode_navigation_map_3d_body(
            &schema::encode_navigation_map_3d_body(&first).unwrap()).unwrap();
        let candidate = schema::decode_navigation_map_3d_body(
            &schema::encode_navigation_map_3d_body(&candidate).unwrap()).unwrap();

        // Use the same blocking context required by FineMapReader's real RPC.
        // This fresh cache has only first's complete objects; the omitted new
        // object really reaches no-provider memq timeout, never an injected Err.
        tokio::task::spawn_blocking(move || {
            let configured = calibrated_config();
            let mut cache = SurfaceGeometryCache::configured_with_lcm_url(&configured, "memq://");
            let old = cache.build_typed(&first).unwrap();
            let physical = cache.physical_snapshot().unwrap();
            let error = match cache.build_typed(&candidate) {
                Err(error) => error,
                Ok(_) => panic!("{variant}: missing object unexpectedly completed"),
            };
            assert!(matches!(error, SurfaceMapBuildError::IncompleteFineSource(_)),
                "{variant}: expected real typed Incomplete, got {error}");
            let retained = cache.retained_source_for_incomplete(&candidate);
            if variant == "same_mask_new_receipt" {
                assert!(retained.as_ref().is_some_and(|source| Arc::ptr_eq(source, &old)),
                    "present mask contents are unchanged despite newer receipt/load clock");
            } else {
                assert!(retained.is_none(), "{variant}: changed geometry/domain cannot retain old mask");
            }
            assert!(Arc::ptr_eq(&physical, &cache.physical_snapshot().unwrap()));
            assert!(Arc::ptr_eq(&old, &cache.build_typed(&first).unwrap()),
                "failed candidate must leave the complete source honest");
        }).await.expect("present-mask RPC build must finish and join");
    }
}
