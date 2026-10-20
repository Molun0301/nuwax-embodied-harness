//! Offline analytic-world qualification. No HTTP listener, device or motion port.
#[path = "../world_motion.rs"]
mod world_motion;
use nuwax_nav_mls_planner::surface::GroundEvidence;
use nuwax_nav_mls_planner::{SurfaceMapBuilder, SurfaceObservation, SurfaceObservations};
use nuwax_nav_runtime::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, RouteFollower};
use nuwax_nav_stack::surface_config::SurfaceNavigationConfig;
use nuwax_nav_types_messages::{
    GoalOptions, MotionKinematics, MotionLimits, MotionModel, NavFrameId, Path3D, Point2, Point3,
    Pose3, PoseStamped, Quaternion, Timestamp, Twist2,
};
use serde_json::json;
use std::{
    collections::BTreeMap,
    fs,
    io::Write,
    path::PathBuf,
    sync::{atomic::AtomicBool, Arc},
    time::{Duration, Instant},
};
use world_motion::{actuate, advance, Body, Plane, Solid, World};
const DT: f64 = 0.05;
fn stamped(p: Body, t: f64) -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(1 + t.floor() as i64, (t.fract() * 1e9) as u32),
        pose: Pose3 {
            position: Point3::new(p.0[0], p.0[1], p.0[2]),
            orientation: Quaternion {
                x: p.1[0],
                y: p.1[1],
                z: p.1[2],
                w: p.1[3],
            },
        },
    }
}
fn twist(v: [f64; 3]) -> Twist2 {
    Twist2 {
        linear: Point2::new(v[0], v[1]),
        angular_z: v[2],
    }
}
fn distance(a: Body, b: Body) -> f64 {
    a.0.iter()
        .zip(b.0)
        .map(|(a, b)| (a - b).powi(2))
        .sum::<f64>()
        .sqrt()
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 4 {
        return Err(
            "usage: analytic_world straight|obstacle|slope off|on OUTPUT (new directory)".into(),
        );
    }
    let case = &args[1];
    let enabled = match args[2].as_str() {
        "off" => false,
        "on" => true,
        _ => return Err("mode must be off/on".into()),
    };
    let world = match case.as_str() {
        "straight" => World {
            plane: Plane {
                a: 0.,
                b: 0.,
                c: 0.,
            },
            solids: vec![],
        },
        "obstacle" => World {
            plane: Plane {
                a: 0.,
                b: 0.,
                c: 0.,
            },
            solids: vec![Solid {
                low: [3.0, -0.4, 0.0],
                high: [3.6, 0.4, 0.8],
            }],
        },
        "slope" => World {
            plane: Plane {
                a: 0.10,
                b: 0.,
                c: 0.,
            },
            solids: vec![],
        },
        _ => return Err("unknown fixed world".into()),
    };
    let out = PathBuf::from(&args[3]);
    fs::create_dir(&out)?;
    let mut config = SurfaceNavigationConfig::from_env()?;
    let body = config
        .body_envelope
        .clone()
        .ok_or("missing calibrated body")?;
    let d = config.follower.body_to_support_z_m;
    let start = world.plane.body([0., 0.], 0., d);
    let goal = world.plane.body([7., 0.], 0., d);
    let points = world.observations();
    let q = f64::from(0.05_f32);
    let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
    for &(x, y, z) in &points {
        let k = [x, y, z].map(|v| (f64::from(v) / q).floor() as i32);
        blocks
            .entry(k.map(|v| v.div_euclid(32)))
            .or_default()
            .insert(
                k,
                SurfaceObservation {
                    observed_ns: 0,
                    hits: 0,
                },
            );
    }
    let mut obs = SurfaceObservations::new(0.05, 32)?;
    for (k, v) in blocks {
        obs.replace_block(k, v)?;
    }
    obs.mark_complete_geometry_source();
    let mut builder =
        SurfaceMapBuilder::configured(config.geometry.clone(), config.body_envelope.clone());
    let map = Arc::new(
        builder
            .build(&points, 0.05)?
            .snapshot
            .with_observations(Some(Arc::new(obs)))
            .with_ground_evidence(GroundEvidence::MeasuredSurface),
    );
    // Same actuator/simulation configuration as the retained native driver;
    // planner budgets, costs and geometry tolerances remain at loaded values.
    config.follower.local_planning_enabled = enabled;
    config.follower.controller.speed = 0.5;
    config.follower.controller.control_frequency = 1. / DT;
    config.follower.controller.simulation = true;
    config.follower.max_angular_speed_rps = 0.8;
    config.follower.basic_kinematics = Some(MotionKinematics {
        model: MotionModel::Omnidirectional,
        max_forward_mps: 0.5,
        max_reverse_mps: 0.5,
        max_lateral_mps: 0.5,
        max_angular_rps: 0.8,
        minimum_turning_radius_m: 0.,
        can_rotate_in_place: true,
    });
    config.follower.motion_limits = Some(MotionLimits {
        model: MotionModel::Omnidirectional,
        max_forward_mps: 0.5,
        max_reverse_mps: 0.5,
        max_lateral_mps: 0.5,
        max_angular_rps: 0.8,
        linear_acceleration_mps2: 1.,
        guaranteed_braking_mps2: 1.,
        angular_acceleration_rps2: 2.,
        guaranteed_angular_braking_rps2: 2.,
        reaction_time_sec: 0.05,
        minimum_turning_radius_m: 0.,
        can_rotate_in_place: true,
    });
    fs::write(
        out.join("case.json"),
        serde_json::to_vec_pretty(
            &json!({"case":case,"local_planning":enabled,"world":format!("{world:?}"),"start":start,"goal":goal,"source_voxel_m":q,"point_count":points.len(),"max_ticks":1200,"max_wall_seconds":90,"dt":DT,"speed_mps":0.5,"geometry":{"voxel_size":config.geometry.voxel_size,"step_threshold_m":config.geometry.step_threshold_m,"max_slope_deg":config.geometry.max_slope_deg,"wall_clearance_m":config.geometry.wall_clearance_m,"min_step_tread_depth_m":config.geometry.min_step_tread_depth_m},"scan":format!("{:?}",config.follower.scan),"body":format!("{body:?}"),"collision_scope":"plane nominal support constraint plus full calibrated convex envelope versus solid boxes; no leg/gait ground penetration claim","quantization":"ground nearest source top face <= half source voxel; solids conservative occupied cells"}),
        )?,
    )?;
    let options = GoalOptions {
        allow_reverse: false,
        position_tolerance_m: 0.35,
        yaw_tolerance_rad: std::f64::consts::PI,
    };
    let cancel = AtomicBool::new(false);
    let mut follower = RouteFollower::new(config.follower.clone())?;
    follower.update_execution_map(ExecutionMap::Surface(map.clone()))?;
    follower.update_odometry(stamped(start, 0.));
    follower.configure_goal(&stamped(goal, 0.), options)?;
    let mut states = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(out.join("ticks.jsonl"))?;
    let began = Instant::now();
    let mut current = start;
    let mut applied = [0.; 3];
    let mut need_plan = true;
    let mut braking = false;
    let mut plans = 0;
    let mut ticks = 0;
    let mut time = 0.;
    let mut outcome = "simulation_time_limit";
    let mut failure = None;
    let mut control_times = Vec::new();
    let mut plan_times = Vec::new();
    let mut world_checks = 0;
    let mut world_failures = 0;
    let mut production_checks = 0;
    let mut production_failures = 0;
    for tick in 0..1200 {
        let tick_start = Instant::now();
        if began.elapsed().as_secs_f64() > 90. {
            outcome = "wall_time_limit";
            break;
        }
        if need_plan {
            let p = Instant::now();
            let route =
                config
                    .follower
                    .pct
                    .body_route(&map, current, goal, options, &[], Some(&cancel));
            plan_times.push(p.elapsed().as_secs_f64() * 1000.);
            plans += 1;
            match route {
                Ok(route) => {
                    writeln!(
                        states,
                        "{}",
                        json!({"event":"route","tick":tick,"plan":plans,"poses":route})
                    )?;
                    follower.replace_execution_path(
                        ExecutionPath::Surface(Path3D {
                            poses: route.iter().map(|p| stamped(*p, time)).collect(),
                        }),
                        None,
                        stamped(goal, time),
                    )?;
                    need_plan = false;
                }
                Err(e) => {
                    outcome = "planning_failure";
                    failure = Some(e);
                    break;
                }
            }
        }
        let ctl = Instant::now();
        let step = if braking {
            PlannerStep::Command(Twist2::ZERO)
        } else {
            follower.step(config.robot_width_m)
        };
        control_times.push(ctl.elapsed().as_secs_f64() * 1000.);
        let mut arriving = false;
        let mut note = String::new();
        let command = match step {
            PlannerStep::Command(t) => [t.linear.x, t.linear.y, t.angular_z],
            PlannerStep::Pending(f) => {
                note = format!("pending {f:?}");
                [0.; 3]
            }
            PlannerStep::Succeeded => {
                arriving = true;
                [0.; 3]
            }
            PlannerStep::ReplanRequired(reason) => {
                note = reason;
                braking = true;
                [0.; 3]
            }
            PlannerStep::GroundArrival(_) => {
                outcome = "unsupported_ground_arrival";
                failure = Some("Ground arrival is unsupported in the Body-only analytic world".into());
                break;
            }
            PlannerStep::Failed(reason) => {
                outcome = "controller_failure";
                failure = Some(reason);
                break;
            }
        };
        let (next_velocity, mean) = actuate(applied, command, DT);
        let next = advance(&world, current, mean, DT, d);
        world_checks += 1;
        let physical =
            world.check_motion(current, mean, DT, d, &body.footprint_xy_m, body.z_bounds_m);
        production_checks += 1;
        let production = map
            .with_current_body_support(current.0, current.1)
            .and_then(|m| m.validate_body_motion(current, next, &[]));
        if physical.is_err() {
            world_failures += 1;
        }
        if production.is_err() {
            production_failures += 1;
        }
        writeln!(
            states,
            "{}",
            json!({"event":"motion","tick":tick,"time":time,"from":current,"next":next,"command":command,"applied":next_velocity,"held_mean_twist":mean,"world_check":physical,"production_check":production.as_ref().map(|_|()).map_err(|e|e.to_string()),"control_ms":control_times.last(),"note":note})
        )?;
        ticks = tick + 1;
        if world_failures > 0 || production_failures > 0 {
            outcome = "motion_disagreement_or_collision";
            failure = Some(format!("world={physical:?}; production={production:?}"));
            break;
        }
        current = next;
        applied = next_velocity;
        time += DT;
        if braking && applied[0].hypot(applied[1]) < 0.005 && applied[2].abs() < 0.01 {
            braking = false;
            need_plan = true;
        }
        follower.observe_applied_command(twist(applied));
        follower.update_odometry(stamped(current, time));
        if arriving
            && applied[0].hypot(applied[1]) < 0.005
            && applied[2].abs() < 0.01
            && distance(current, goal) <= options.position_tolerance_m
        {
            outcome = "arrived";
            break;
        }
        if tick % 200 == 0 {
            eprintln!(
                "ANALYTIC_PROGRESS {case} {} tick={tick} remaining={:.3}",
                args[2],
                distance(current, goal)
            );
        }
        if let Some(remaining) = Duration::from_secs_f64(DT).checked_sub(tick_start.elapsed()) {
            std::thread::sleep(remaining);
        }
    }
    control_times.sort_by(f64::total_cmp);
    let median = control_times.get(control_times.len() / 2);
    let p95 = control_times
        .get((control_times.len() * 95 / 100).min(control_times.len().saturating_sub(1)));
    let summary = json!({"case":case,"local_planning":enabled,"outcome":outcome,"failure":failure,"ticks":ticks,"sim_seconds":time,"wall_seconds":began.elapsed().as_secs_f64(),"final_pose":current,"distance_remaining_m":distance(current,goal),"world_checks":world_checks,"world_failures":world_failures,"production_checks":production_checks,"production_failures":production_failures,"global_plans":plans,"local_planning_calls":follower.local_planning_calls(),"control_ms":{"median":median,"p95":p95,"max":control_times.last(),"over50ms":control_times.iter().filter(|v|**v>50.).count()},"global_plan_ms":plan_times});
    fs::write(
        out.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
