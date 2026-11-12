//! 贴柱末段的离线业务闭环；实体裁判复用独立分析世界，不操作运动端或平台。
#[path = "../world_motion.rs"]
mod world_motion;
use nuwax_nav_mls_planner::{
    surface::GroundEvidence, SurfaceMapBuilder, SurfaceObservation, SurfaceObservations,
};
use nuwax_nav_runtime::{
    ExecutionMap, ExecutionPath, MeasuredMotion, NavPlanner, PlannerStep, SurfaceFollower,
    TerminalPhase,
};
use nuwax_nav_stack::surface_config::SurfaceNavigationConfig;
use nuwax_nav_types_messages::{
    GoalOptions, MotionLimits, MotionModel, NavFrameId, Path3D, Point2, Point3, Pose3, PoseStamped,
    Quaternion, Timestamp, Twist2,
};
use serde_json::{json, Value};
use std::{
    collections::BTreeMap,
    sync::Arc,
    time::{Duration, Instant},
};
use world_motion::{actuate, advance, heading, Body, Plane, Solid, World};

const DT: f64 = 0.05;
fn stamped(body: Body, tick: usize) -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(1 + (tick / 20) as i64, ((tick % 20) * 50_000_000) as u32),
        pose: Pose3 {
            position: Point3::new(body.0[0], body.0[1], body.0[2]),
            orientation: Quaternion {
                x: body.1[0],
                y: body.1[1],
                z: body.1[2],
                w: body.1[3],
            },
        },
    }
}
fn run_pillar(lateral: bool, goal_yaw: f64) -> (Value, Vec<Value>) {
    let mut config = SurfaceNavigationConfig::from_runtime().unwrap();
    let body = config.body_envelope.clone();
    let d = config.follower.body_to_support_z_m;
    let edge = -body
        .footprint_xy_m
        .iter()
        .map(|p| p[1])
        .fold(f64::INFINITY, f64::min);
    let world = World {
        plane: Plane {
            a: 0.,
            b: 0.,
            c: 0.,
        },
        solids: vec![Solid {
            low: [2. + edge + 0.06, -0.15, 0.],
            high: [2. + edge + 0.36, 0.15, 0.8],
        }],
    };
    let start = world.plane.body([0., 0.], 0., d);
    let goal = world.plane.body([2., 0.], goal_yaw, d);
    let goal_edge = body.footprint_xy_m.iter().map(|p| goal_yaw.cos()*p[0]-goal_yaw.sin()*p[1])
        .fold(f64::NEG_INFINITY,f64::max);
    let goal_clearance = world.solids[0].low[0]-goal.0[0]-goal_edge;
    let points = world.observations();
    let q = f64::from(0.05_f32);
    let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
    for &(x, y, z) in &points {
        let key = [x, y, z].map(|v| (f64::from(v) / q).floor() as i32);
        blocks
            .entry(key.map(|v| v.div_euclid(32)))
            .or_default()
            .insert(
                key,
                SurfaceObservation {
                    observed_ns: 0,
                    hits: 0,
                },
            );
    }
    let mut observations = SurfaceObservations::new(0.05, 32).unwrap();
    for (key, values) in blocks {
        observations.replace_block(key, values).unwrap();
    }
    observations.mark_complete_geometry_source();
    let mut builder = SurfaceMapBuilder::configured(config.geometry.clone(), Some(body.clone()));
    let built = builder.build(&points, 0.05).unwrap();
    let source = Arc::new(
        built
            .snapshot
            .with_observations(Some(Arc::new(observations)))
            .with_ground_evidence(GroundEvidence::MeasuredSurface),
    );
    config.follower.controller.speed = 0.5;
    config.follower.controller.control_frequency = 1. / DT;
    config.follower.max_angular_speed_rps = 0.8;
    config.follower.motion_limits = Some(MotionLimits {
        model: MotionModel::Omnidirectional,
        max_forward_mps: 0.5,
        max_reverse_mps: 0.,
        max_lateral_mps: if lateral { 0.5 } else { 0. },
        max_angular_rps: 0.8, min_executable_linear_mps: 0., min_executable_angular_rps: 0.,
        linear_acceleration_mps2: 1.,
        guaranteed_braking_mps2: 1.,
        angular_acceleration_rps2: 2.,
        guaranteed_angular_braking_rps2: 2.,
        reaction_time_sec: DT,
        minimum_turning_radius_m: 0.,
        can_rotate_in_place: true,
    });
    // 静态分析世界显式初始化占据；不向生产注册帧入口伪造传感器数据。
    let mut follower = SurfaceFollower::with_map_initializer(
        config.follower,
        Some(|map, obstacles, ground, boundary, forbidden, position| {
            let occupied: Vec<_> = obstacles.iter().chain(ground).copied().collect();
            map.seed_occupied(&occupied, position);
            map.update_surface(&[], &[], boundary, forbidden)
        }),
    )
    .unwrap();
    let options = GoalOptions {
        allow_reverse: false,
        position_tolerance_m: 0.15,
        yaw_tolerance_rad: 0.08,
    };
    follower.configure_goal(&stamped(goal, 0), options).unwrap();
    follower
        .update_execution_map(ExecutionMap::Surface(source))
        .unwrap();
    follower.update_odometry(stamped(start, 0));
    // 来向沿X轴，目标朝向为90或96度；完整参考保留原业务目标。
    follower
        .replace_execution_path(
            ExecutionPath::Surface(Path3D {
                poses: vec![stamped(start, 0), stamped(goal, 0)],
            }),
            None,
            stamped(goal, 0),
        )
        .unwrap();
    let mut current = start;
    let mut applied = [0.; 3];
    let mut rows = Vec::new();
    let mut phases = [0usize; 3];
    let mut succeeded = false;
    let mut world_failures = 0;
    let mut failure = None;
    let began = Instant::now();
    for tick in 0..1200 {
        let tick_start = Instant::now();
        if began.elapsed().as_secs_f64() > 90. {
            failure = Some("分析世界原90秒墙钟上限".to_string());
            break;
        }
        let now = nuwax_nav_runtime::motion_feedback::monotonic_ns();
        follower.observe_measured_motion(Some(MeasuredMotion {
            source_id: "analytic-pillar".into(),
            source_timestamp_ns: 1_000_000_000 + tick as u64 * 50_000_000,
            received_monotonic_ns: now,
            valid_until_monotonic_ns: now + 1_000_000_000,
            frame_id: "base_link".into(),
            body_linear_xyz: [applied[0], applied[1], 0.],
            body_angular_xyz: [0., 0., applied[2]],
            orientation_wxyz: Some([current.1[3], current.1[0], current.1[1], current.1[2]]),
        }));
        let step = follower.step();
        let command = match step {
            PlannerStep::Command(v) => [v.linear.x, v.linear.y, v.angular_z],
            PlannerStep::Succeeded => {
                succeeded = true;
                [0.; 3]
            }
            PlannerStep::Pending(_) => [0.; 3],
            other => {
                failure = Some(format!("{other:?}"));
                break;
            }
        };
        let phase = follower.terminal_phase();
        phases[match phase {
            TerminalPhase::Travel => 0,
            TerminalPhase::Turning => 1,
            TerminalPhase::Translating => 2,
        }] += 1;
        let (next_applied, mean) = actuate(applied, command, DT);
        let checked =
            world.check_motion(current, mean, DT, d, &body.footprint_xy_m, body.z_bounds_m);
        if checked.is_err() {
            world_failures += 1;
        }
        let next = advance(&world, current, mean, DT, d);
        let selected = follower
            .terminal_points()
            .map(|p| json!({"S":p.stop}));
        rows.push(
            json!({"tick":tick,"time_s":tick as f64*DT,"position":current.0,"yaw":heading(current),
            "command":command,"applied":applied,"phase":format!("{phase:?}"),"terminal":selected,
            "world_check":checked,"wait_reason":follower.execution_wait_reason()}),
        );
        if world_failures != 0 {
            failure = Some("完整机身与独立实体相交".into());
            break;
        }
        current = next;
        applied = next_applied;
        follower.observe_applied_command(Twist2 {
            linear: Point2::new(applied[0], applied[1]),
            angular_z: applied[2],
        });
        follower.update_odometry(stamped(current, tick + 1));
        if succeeded {
            break;
        }
        if let Some(wait) = Duration::from_secs_f64(DT).checked_sub(tick_start.elapsed()) {
            std::thread::sleep(wait);
        }
    }
    let error = (current.0[0] - goal.0[0]).hypot(current.0[1] - goal.0[1]);
    let yaw_error = (heading(current) - heading(goal))
        .sin()
        .atan2((heading(current) - heading(goal)).cos())
        .abs();
    let stopped = applied[0].hypot(applied[1]) < 0.02 && applied[2].abs() < 0.02;
    (
        json!({"lateral":lateral,"succeeded":succeeded,"ticks":rows.len(),"sim_seconds":rows.len() as f64*DT,
        "wall_seconds":began.elapsed().as_secs_f64(),"position_error_m":error,"yaw_error_rad":yaw_error,
        "stopped":stopped,"final_pose":current,"world_failures":world_failures,"failure":failure,
        "phase_ticks":{"行进":phases[0],"转向":phases[1],"平移":phases[2]},"terminal":rows.last().and_then(|r|r.get("terminal")),
        "goal":goal,"goal_physical_clearance_m":goal_clearance,"source_voxel_m":q,
        "method":"生产SurfaceFollower及原生规划/跟踪；独立分析世界的完整机身实体扫掠；按原分析世界执行器加减速积分"}),
        rows,
    )
}
fn main() {
    let output = std::path::PathBuf::from(std::env::args().nth(1).expect("提供证据输出目录"));
    std::fs::create_dir_all(&output).unwrap();
    for (name,lateral,yaw) in [("可横移90度",true,90_f64.to_radians()),
        ("可横移96度",true,96_f64.to_radians()),("不可横移",false,90_f64.to_radians())] {
        let (summary, rows) = run_pillar(lateral,yaw);
        std::fs::write(
            output.join(format!("{name}_汇总.json")),
            serde_json::to_vec_pretty(&summary).unwrap(),
        )
        .unwrap();
        let text = rows.iter().map(|r| format!("{r}\n")).collect::<String>();
        std::fs::write(output.join(format!("{name}_逐拍.jsonl")), text).unwrap();
        println!("{summary}");
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pillar_goal_with_lateral_motion_arrives_without_any_solid_intersection() {
        let (result, _) = run_pillar(true,90_f64.to_radians());
        println!("{result}");
        assert_eq!(result["world_failures"], 0);
        assert_eq!(result["succeeded"], true, "{result}");
        assert!(result["position_error_m"].as_f64().unwrap() < 0.15);
        assert!(result["yaw_error_rad"].as_f64().unwrap() < 0.08);
        assert_eq!(result["stopped"], true);
    }
    #[test]
    fn pillar_goal_without_lateral_motion_keeps_the_pre_rule4_behavior() {
        let (result, rows) = run_pillar(false,90_f64.to_radians());
        println!("{result}");
        assert_eq!(result["world_failures"], 0, "{result}");
        assert_eq!(result["succeeded"],false);
        assert_eq!(result["position_error_m"],2.0);
        assert!(rows.iter().all(|r|r["command"]==json!([0.,0.,0.])));
    }
    #[test]
    fn pillar_goal_at_96_degrees_projects_reverse_then_arrives_without_intersection() {
        let (result, _) = run_pillar(true,96_f64.to_radians());
        println!("{result}");
        assert_eq!(result["world_failures"],0);
        assert_eq!(result["succeeded"],true,"{result}");
        assert!(result["position_error_m"].as_f64().unwrap()<0.15);
        assert!(result["yaw_error_rad"].as_f64().unwrap()<0.08);
        assert_eq!(result["stopped"],true);
    }
}
