//! Offline public PCT -> Scan qualification over the exact archived fine source.
//! No service/transport/controller/actuator is started. Requested 2/5/10 cm
//! departures are synthetic probes, never captured world or scoring targets.
#[path = "../../nuwax-nav-mls-planner/examples/frozen_unknown_ground.rs"]
mod frozen;
use nuwax_nav_pct::PctPlanner;
use nuwax_nav_scan::ScanConfig;
use nuwax_nav_types_messages::{GoalOptions, MotionKinematics, MotionModel};

fn main() {
    frozen::run_replay(|map, start, goal, env| {
        Some((|| {
            let number = |name: &str| -> Result<f64, String> {
                env["values"][name]
                    .as_str()
                    .ok_or_else(|| format!("missing archived {name}"))?
                    .parse()
                    .map_err(|_| format!("invalid archived {name}"))
            };
            if env["values"]["NUWAX_NAV_ACTUATOR_MOTION_MODEL"].as_str() != Some("omnidirectional")
            {
                return Err(
                    "diagnostic requires the supplied declared omnidirectional model".into(),
                );
            }
            let linear = number("NUWAX_NAV_ACTUATOR_MAX_LINEAR_MPS")?;
            let kinematics = MotionKinematics {
                model: MotionModel::Omnidirectional,
                max_forward_mps: linear,
                max_reverse_mps: linear,
                max_lateral_mps: linear,
                max_angular_rps: number("NUWAX_NAV_ACTUATOR_MAX_YAW_RPS")?,
                minimum_turning_radius_m: 0.0,
                can_rotate_in_place: true,
            };
            kinematics.validate()?;
            let kinematics =
                nuwax_nav_runtime::surface_follower::forward_navigation_kinematics(kinematics);
            let mut options = GoalOptions::default();
            options.position_tolerance_m = 0.005;
            let planner = PctPlanner::default();
            let path = planner
                .body_route(map, start, goal, options, &[], None)
                .map_err(|reason| format!("PCT: {reason}"))?;
            if path.len() < 2 {
                return Err("PCT returned no nontrivial departure".into());
            }
            // Derive the same straight-route hint as ReferenceSource rather than
            // forcing a favorable path-selection mode for the diagnostic.
            let mut direction: Option<[f64; 2]> = None;
            let mut straight = true;
            for pair in path.windows(2) {
                let delta = [pair[1].0[0] - pair[0].0[0], pair[1].0[1] - pair[0].0[1]];
                let length = delta[0].hypot(delta[1]);
                if length > 1e-9 {
                    let next = delta.map(|value| value / length);
                    if let Some(first) = direction {
                        straight &= first[0] * next[0] + first[1] * next[1] > 0.0
                            && (first[0] * next[1] - first[1] * next[0]).abs() < 1e-9;
                    } else {
                        direction = Some(next);
                    }
                }
            }
            let offset = map.body_envelope().unwrap().body_to_support_z_m;
            // Same body-XY/nominal-support-Z representation consumed by the runtime
            // ReferenceSource; include the actual original 3D arrival goal at the end.
            let mut reference: Vec<_> = path
                .iter()
                .map(|pose| [pose.0[0], pose.0[1], pose.0[2] - offset])
                .collect();
            let target = [goal.0[0], goal.0[1], goal.0[2] - offset];
            if reference.last() != Some(&target) {
                reference.push(target);
            }
            let tomogram = planner.map(map)?;
            let scan = ScanConfig::default();
            println!(
                "PCT_SCAN_CONFIG kinematics={kinematics:?} scan={scan:?} arrival_tolerance_m=0.005"
            );
            let trajectory = nuwax_nav_scan::plan(
                map,
                &tomogram,
                start,
                &reference,
                &kinematics,
                &scan,
                None,
                Some((goal.0, 0.005)),
                &[],
                false,
                straight,
            )
            .map_err(|reason| format!("Scan: {reason}"))?;
            if trajectory.poses.len() < 2 {
                return Err("Scan returned no nontrivial departure".into());
            }
            for pair in trajectory.poses.windows(2) {
                let mut witness = None;
                map.validate_body_motion_with_witness(pair[0], pair[1], &[], &mut witness)
                    .map_err(|reason| {
                        format!("Scan output sweep: {reason}; witness={witness:?}")
                    })?;
            }
            let end = trajectory.poses.last().unwrap().0;
            let error = (0..3)
                .map(|axis| (end[axis] - goal.0[axis]).powi(2))
                .sum::<f64>()
                .sqrt();
            if error >= 0.005 {
                return Err(format!(
                    "Scan stopped outside requested arrival sphere: {error}"
                ));
            }
            println!(
                "PCT_SCAN_SUCCESS global_poses={} local_poses={} arrival_error_m={error} smoothed={}",
                path.len(),
                trajectory.poses.len(),
                trajectory.smoothed
            );
            Ok(())
        })())
    });
}
