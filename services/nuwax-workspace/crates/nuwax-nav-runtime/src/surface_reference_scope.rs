//! Bounded advisory only: the complete reference's XY box plus its MLS read radius.
//! Box interiors, historical segments and all heights deliberately remain conservative.
use crate::SurfaceSnapshot;
use nuwax_nav_types_messages::Path3D;

/// Conservative all-height XY reference bounds plus the existing MLS read radius.
pub struct ReferenceCorridor {
    min: [f64; 2],
    max: [f64; 2],
    sizes: [f32; 2],
}

impl ReferenceCorridor {
    /// The owner first budgets body vertices + all poses + both changed-column sets.
    /// No cached map/domain, body validation, segment queries or new task lifecycle.
    pub fn new(surface: &SurfaceSnapshot, path: &Path3D) -> Option<Self> {
        path.poses.first()?;
        let (mut min, mut max) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        let mut pure_yaw = true;
        for pose in &path.poses {
            let p = pose.pose.position; let q = pose.pose.orientation;
            let norm2 = [q.x, q.y, q.z, q.w].into_iter().map(|v| v * v).sum::<f64>();
            if ![p.x, p.y, p.z, norm2].into_iter().all(f64::is_finite) || norm2 <= 0.0
                || pose.frame_id.as_str() != surface.config().world_frame { return None; }
            // Original normalize/shortest-sign nlerp preserves exact zero X/Y
            // over every segment, including repeated-position turns. No tilt epsilon.
            pure_yaw &= q.x == 0.0 && q.y == 0.0;
            for (axis, value) in [p.x, p.y].into_iter().enumerate() {
                min[axis] = min[axis].min(value); max[axis] = max[axis].max(value);
            }
        }
        let scale = min.into_iter().chain(max).map(f64::abs).fold(0.0, f64::max);
        let radius = surface.reference_geometry_scope_radius(pure_yaw, scale)?;
        let min = min.map(|v| v - radius); let max = max.map(|v| v + radius);
        if !min.into_iter().chain(max).all(f64::is_finite) { return None; }
        Some(Self { min, max, sizes: [surface.config().voxel_size, surface.source_voxel_size_m()] })
    }

    /// False proves disjointness; invalid resolution remains conservative Unknown.
    /// Closed XY rectangles include touches, negative columns and deleted columns.
    pub fn intersects_columns(&self, size: f32, columns: &[[i32; 2]]) -> bool {
        if !size.is_finite() || size <= 0.0
            || !self.sizes.into_iter().any(|s| s.to_bits() == size.to_bits()) { return true; }
        let size = f64::from(size);
        columns.iter().any(|column| (0..2).all(|axis| {
            let lower = f64::from(column[axis]) * size;
            // Convert before +1 so i32::MAX cannot wrap.
            let upper = (f64::from(column[axis]) + 1.0) * size;
            lower <= self.max[axis] && upper >= self.min[axis]
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use nuwax_nav_mls_planner::{mls_planner::Config, SurfaceBodyEnvelope, SurfaceObservations};
    use nuwax_nav_types_messages::{NavFrameId, Point3, Pose3, PoseStamped, Quaternion};

    fn scene(s: f32, f: f32, half: f64, offset: f64, clearance: f32) -> SurfaceSnapshot {
        let config = Config { world_frame: "map".into(), voxel_size: s, robot_height: 0.2,
            surface_closing_radius: 0.0, node_spacing_m: 0.3, wall_clearance_m: clearance,
            wall_buffer_m: 0.5, wall_buffer_weight: 2.0, step_threshold_m: 0.2,
            max_slope_deg: 18.0, min_step_tread_depth_m: 0.2, step_penalty_weight: 2.0,
            goal_tolerance: 0.1, viz_publish_hz: 0.0 };
        let model = SurfaceBodyEnvelope {fixed_collision:None, footprint_xy_m: vec![[-half, -half], [half, -half],
            [half, half], [-half, half]], z_bounds_m: [-half, half], body_to_support_z_m: offset,
            max_step_up_m: 0.2, max_step_down_m: 0.2, max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0 };
        SurfaceSnapshot::from_points(&[(0.05, 0.05, 0.025)], config, f).unwrap()
            .with_body_envelope(model).unwrap()
    }
    fn original() -> SurfaceSnapshot { scene(0.1, 0.05, 0.1, 0.4, 0.0) }
    fn path(points: &[[f64; 3]]) -> Path3D {
        Path3D { poses: points.iter().map(|p| PoseStamped::from_pose(Pose3 {
            position: Point3::new(p[0], p[1], p[2]), orientation: Quaternion::from_yaw(0.0),
        }, NavFrameId::Map)).collect() }
    }

    #[test]
    fn original_a_is_disjoint_without_moving_its_fine_or_coarse_column() {
        let surface = original();
        let route = path(&[[0.55, 1.55, 0.5], [5.55, 1.55, 0.5]]);
        let corridor = ReferenceCorridor::new(&surface, &route).unwrap();
        // Original A=[71,11,47] also changes coarse [35,5,23]; never clip Z.
        assert!(!corridor.intersects_columns(0.05, &[[71, 11]]));
        assert!(!corridor.intersects_columns(0.1, &[[35, 5]]));
        assert!(corridor.intersects_columns(0.05, &[[71, 31]]));
        assert!((0.67..0.68).contains(&(1.55 - corridor.min[1])));
    }

    #[test]
    fn exact_yaw_shortest_sign_and_zero_length_tilt_use_the_right_bound() {
        let surface = original(); let mut route = path(&[[3.55, 1.55, 0.5]; 2]);
        route.poses[1].pose.orientation = Quaternion { x: 0.0, y: -0.0, z: 0.0, w: -2.0 };
        assert!(!ReferenceCorridor::new(&surface, &route).unwrap().intersects_columns(0.05, &[[71, 11]]));
        route.poses[1].pose.orientation.x = 1e-30;
        assert!(ReferenceCorridor::new(&surface, &route).unwrap().intersects_columns(0.05, &[[71, 11]]));
    }

    #[test]
    fn sparse_loop_singleton_and_closed_negative_cells_keep_the_full_box() {
        let surface = original();
        for route in [path(&[[-10.0, -2.0, -20.0], [10.0, -2.0, 300.0]]),
            path(&[[-3.0, -3.0, 0.5], [3.0, 3.0, 0.5], [-3.0, 3.0, 0.5], [3.0, -3.0, 0.5], [-3.0, -3.0, 0.5]])] {
            assert!(ReferenceCorridor::new(&surface, &route).unwrap().intersects_columns(0.1, &[[0, -20]]));
        }
        assert!(ReferenceCorridor::new(&surface, &path(&[[-0.05, -0.05, 999.0]])).unwrap()
            .intersects_columns(0.05, &[[-1, -1]]));
        let touch = ReferenceCorridor { min: [-1.0; 2], max: [-1.0; 2], sizes: [1.0; 2] };
        assert!(touch.intersects_columns(1.0, &[[-2, -2], [-1, -1]]));
        assert!(!touch.intersects_columns(1.0, &[[-3, -3]]));
    }

    #[test]
    fn independent_fine_tread_uses_f32_ceil_even_when_it_dominates() {
        // Tiny admitted body makes the fine-tread branch determine the maximum.
        let surface = scene(0.01, 0.01, 0.001, 0.001, 0.0);
        let radius = surface.reference_geometry_scope_radius(true, 0.0).unwrap();
        let f = f64::from(0.01_f32);
        let expected = f64::from((0.2_f32 / 0.01_f32).ceil()) * f + 2.0 * std::f64::consts::SQRT_2 * f;
        assert!(radius >= expected && radius < expected + 1e-4);
        assert_eq!((0.2_f32 / 0.01_f32).ceil(), 20.0);
        // Non-dividing source precision can extend farther than coarse p*s.
        assert!(f64::from((0.2_f32 / 0.06_f32).ceil()) * f64::from(0.06_f32)
            > 2.0 * f64::from(0.1_f32));
    }

    #[test]
    fn uncertain_pose_resolution_and_nonlocal_conditions_stay_unknown() {
        let surface = original(); let route = path(&[[0.0, 0.0, 0.5]]);
        for q in [[0.0; 4], [f64::NAN, 0.0, 0.0, 1.0], [0.0, 0.0, f64::MAX, 1.0],
            [0.0, 0.0, f64::MIN_POSITIVE, 0.0]] {
            let mut invalid = route.clone();
            invalid.poses[0].pose.orientation = Quaternion { x: q[0], y: q[1], z: q[2], w: q[3] };
            assert!(ReferenceCorridor::new(&surface, &invalid).is_none());
        }
        assert!(ReferenceCorridor::new(&surface, &Path3D::empty()).is_none());
        assert!(ReferenceCorridor::new(&surface, &path(&[[f64::NAN, 0.0, 0.5]])).is_none());
        assert!(ReferenceCorridor::new(&scene(0.1, 0.05, 0.1, 0.4, 0.01), &route).is_none());
        let mismatched = surface.with_observations(Some(Arc::new(SurfaceObservations::new(0.04, 32).unwrap())));
        assert!(ReferenceCorridor::new(&mismatched, &route).is_none());
        let corridor = ReferenceCorridor::new(&surface, &route).unwrap();
        for size in [0.0, -0.05, 0.025, f32::NAN, f32::INFINITY] {
            assert!(corridor.intersects_columns(size, &[[1000, 1000]]));
        }
    }

    #[test]
    fn full_owner_work_budget_can_still_prove_original_a_disjoint() {
        let surface = original(); let mut route = path(&[[0.55, 1.55, 0.5]]);
        route.poses = vec![route.poses[0].clone(); 4091];
        // Owner budgets 4 body vertices + 4091 poses + 1 changed column = 4096.
        // Its over-budget Unknown branch is tested at that owner gate.
        assert!(!ReferenceCorridor::new(&surface, &route).unwrap().intersects_columns(0.05, &[[71, 11]]));
    }

    fn with_outlying_fixed(surface: &SurfaceSnapshot, shape: serde_json::Value) -> SurfaceSnapshot {
        let mut model=surface.body_envelope().unwrap().clone();
        let mut primitive=serde_json::json!({"link_name":"actual_outlying_fixed",
            "translation_m":[0.0,-1.0,0.0],"rotation_row_major":[1,0,0,0,1,0,0,0,1]});
        primitive.as_object_mut().unwrap().extend(shape.as_object().unwrap().clone());
        model.fixed_collision=Some(Arc::new(serde_json::from_value(serde_json::json!({
            "model_base_link":"base","primitives":[primitive],
            "excluded_moving_collision_count":0,"unsupported_collision_count":0})).unwrap()));
        surface.with_body_envelope(model).unwrap()
    }

    #[test]
    fn fixed_outside_envelope_is_inside_refresh_scope_and_actual_collision_query() {
        use nuwax_nav_mls_planner::{SurfaceObservation, SurfaceError};
        use nuwax_nav_mls_planner::surface::{BodyRejection, OccupancySource};
        let original=original();
        let route=path(&[[0.55,1.55,0.5],[5.55,1.55,0.5]]);
        assert!(!ReferenceCorridor::new(&original,&route).unwrap().intersects_columns(0.05,&[[71,11]]));
        let box_shape=serde_json::json!({"shape":"box","size_m":[0.2,0.2,0.2]});
        let fixed=with_outlying_fixed(&original,box_shape);
        let corridor=ReferenceCorridor::new(&fixed,&route).unwrap();
        assert!(corridor.intersects_columns(0.05,&[[71,11]]));
        assert!(corridor.intersects_columns(0.1,&[[35,5]]));
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        observations.replace_block([2,0,0],std::collections::BTreeMap::from([
            ([71,11,10],SurfaceObservation {observed_ns:1,hits:1})])).unwrap();
        let blocked=fixed.with_observations(Some(Arc::new(observations)));
        let pose=([3.55,1.55,0.5],[0.0,0.0,0.0,1.0]);
        let query=blocked.body_query();let ground=query.start_ground_state(pose).unwrap();
        let mut witness=None;
        assert_eq!(query.validate_motion_from_ground(pose,pose,&ground,&[],&mut witness),
            Err(SurfaceError::InsufficientClearance));
        assert!(matches!(witness.unwrap().rejection,BodyRejection::Occupancy {
            cell:[71,11,10],source:OccupancySource::Fine|OccupancySource::FineFringe,..}));
        eprintln!("FIXED_SCOPE outlying_box_fine_and_coarse_related=true actual_fine_collision=[71,11,10]");
    }

    #[test]
    fn fixed_scope_bounds_world_aabb_corners_and_counts_original_advisory_budget() {
        use crate::surface_changes::{SurfaceGeometryChanges,SurfaceJobTicket,MAX_REFERENCE_SCOPE_WORK};
        let base=original();
        for (shape,radius) in [
            (serde_json::json!({"shape":"box","size_m":[0.2,0.2,0.2]}),0.1_f64*3.0_f64.sqrt()),
            (serde_json::json!({"shape":"sphere","radius_m":0.2}),0.2),
            (serde_json::json!({"shape":"cylinder","radius_m":0.2,"length_m":0.4}),0.2_f64.hypot(0.2))] {
            let map=Arc::new(with_outlying_fixed(&base,shape));
            let yaw=map.reference_geometry_scope_radius(true,5.55).unwrap();
            let tilted=map.reference_geometry_scope_radius(false,5.55).unwrap();
            assert!(yaw>=1.0+std::f64::consts::SQRT_2*radius);
            assert!(tilted>=1.0+3.0_f64.sqrt()*radius && tilted>=yaw);
            let mut ticket=SurfaceJobTicket::new(Some(map.clone()));
            ticket.observe(&map,Some(&map),&Arc::new(SurfaceGeometryChanges::Columns {
                fine_size:0.05,fine:vec![[1000,1000]],planner_size:0.1,coarse:vec![]}));
            let pose=path(&[[0.55,1.55,0.5]]).poses.remove(0);
            // Four original vertices + one actual primitive + one column.
            let mut route=Path3D {poses:vec![pose.clone();MAX_REFERENCE_SCOPE_WORK-6]};
            assert!(ticket.permits_path(&map,&route));
            route.poses.push(pose);
            assert!(!ticket.permits_path(&map,&route));
        }
    }
    #[test]
    fn fixed_union_aabb_corners_from_different_primitives_are_in_scope() {
        let base=original();let mut model=base.body_envelope().unwrap().clone();
        let primitives=[[10.0,0.0,0.0],[0.0,10.0,0.0]].map(|translation|
            serde_json::json!({"link_name":"union_corner","translation_m":translation,
                "rotation_row_major":[1,0,0,0,1,0,0,0,1],"shape":"sphere","radius_m":0.1}));
        model.fixed_collision=Some(Arc::new(serde_json::from_value(serde_json::json!({
            "model_base_link":"base","primitives":primitives,
            "excluded_moving_collision_count":0,"unsupported_collision_count":0})).unwrap()));
        let source=base.with_body_envelope(model).unwrap();
        let union_xy=10.1_f64.hypot(10.1);
        let union_xyz=union_xy.hypot(0.1);
        let yaw=source.reference_geometry_scope_radius(true,20.0).unwrap();
        let tilt=source.reference_geometry_scope_radius(false,20.0).unwrap();
        assert!(yaw>=union_xy,"{yaw} < {union_xy}");
        assert!(tilt>=union_xyz && tilt>=yaw);
        eprintln!("FIXED_UNION corners_xy={union_xy} corners_xyz={union_xyz} yaw_scope={yaw} tilt_scope={tilt}");
    }

}
