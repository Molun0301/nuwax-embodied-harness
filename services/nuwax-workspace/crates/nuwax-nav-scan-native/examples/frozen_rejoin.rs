//! 原严格迟到接回场景的原生闭环；真值碰撞只使用实体墙与旋转矩形。
use nuwax_nav_scan_native::{Config, PlanRequest, Planner};
use serde_json::{json, Value};
use std::{collections::BTreeSet, fs, path::PathBuf, time::Instant};

#[derive(Clone, Copy)]
struct Pose {
    x: f64,
    y: f64,
    z: f64,
    yaw: f64,
}
fn quaternion_yaw(q: [f64; 4]) -> f64 {
    2.0 * q[2].atan2(q[3])
}
fn body_clear(p: Pose) -> bool {
    let (cs, sn) = (p.yaw.cos(), p.yaw.sin());
    let corners = [[-0.3, -0.16], [0.3, -0.16], [0.3, 0.16], [-0.3, 0.16]]
        .map(|[x, y]| [p.x + cs * x - sn * y, p.y + sn * x + cs * y]);
    if corners
        .iter()
        .any(|[x, y]| *x < -1.7 || *x > 1.7 || *y < -1.15 || *y > 1.45)
    {
        return false;
    }
    if p.z + 0.15 <= 0.0 || p.z - 0.15 >= 0.65 {
        return true;
    }
    // 实体墙是 [0,0.04]×[-0.25,0.55]×[0,0.65]；四轴分离定理独立复核。
    let wall = [[0.0, -0.25], [0.04, -0.25], [0.04, 0.55], [0.0, 0.55]];
    let axes = [[1.0, 0.0], [0.0, 1.0], [cs, sn], [-sn, cs]];
    for axis in axes {
        let extent = |points: &[[f64; 2]; 4]| {
            points
                .iter()
                .map(|p| p[0] * axis[0] + p[1] * axis[1])
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                    (lo.min(v), hi.max(v))
                })
        };
        let (a, b) = extent(&corners);
        let (c, d) = extent(&wall);
        if b <= c || d <= a {
            return true;
        }
    }
    false
}
fn integrate(p: Pose, v: f64, w: f64, dt: f64) -> Pose {
    let mut q = p;
    if w.abs() > 1e-12 {
        q.x += v / w * ((p.yaw + w * dt).sin() - p.yaw.sin());
        q.y += v / w * (p.yaw.cos() - (p.yaw + w * dt).cos());
    } else {
        q.x += v * dt * p.yaw.cos();
        q.y += v * dt * p.yaw.sin();
    }
    q.yaw += w * dt;
    q
}
fn physical_step(
    p: Pose,
    applied: [f64; 2],
    command: [f64; 2],
    dt: f64,
) -> Result<(Pose, [f64; 2]), String> {
    let next = [
        applied[0] + (command[0] - applied[0]).clamp(-dt, dt),
        applied[1] + (command[1] - applied[1]).clamp(-2.0 * dt, 2.0 * dt),
    ];
    let mean = [(applied[0] + next[0]) / 2.0, (applied[1] + next[1]) / 2.0];
    // 采用原反馈模型的平均速度积分；每毫米角点弧长采样独立实体碰撞。
    let distance = (mean[0].abs() + 0.34 * mean[1].abs()) * dt;
    let samples = (distance / 0.001).ceil().max(1.0) as usize;
    for i in 0..=samples {
        let at = integrate(p, mean[0], mean[1], dt * i as f64 / samples as f64);
        if !body_clear(at) {
            return Err(format!(
                "实体墙碰撞，x={}, y={}, yaw={}",
                at.x, at.y, at.yaw
            ));
        }
    }
    Ok((integrate(p, mean[0], mean[1], dt), next))
}
fn config() -> Config {
    Config {
        resolution: 0.1,
        body_length: 0.6,
        body_width: 0.32,
        body_height: 0.3,
        body_to_ground: 0.4,
        max_velocity: 0.5,
        max_acceleration: 1.0,
        max_yaw_rate: 0.8,
        planning_horizon: 5.0,
        sample_dt: 0.1,
        body_center_x: 0.0,
        body_center_y: 0.0,
        body_z_min: -0.15,
        body_z_max: 0.15,
        max_step_up: 0.0,
        max_step_down: 0.0,
        max_slope: 18.0_f64.to_radians(),
        max_yaw_acceleration: 2.0,
    }
}
fn run_case(
    name: &str,
    start: Pose,
    route: &[[f64; 3]],
    obstacles: &[[f64; 3]],
    ground: &[[f64; 3]],
    directory: &PathBuf,
) -> Value {
    let boundary = [[-1.7, -1.15], [1.7, -1.15], [1.7, 1.45], [-1.7, 1.45]];
    let mut pose = start;
    let mut applied = [0.0; 2];
    let mut acceleration = [0.0; 3];
    let mut trace = vec![];
    let mut nonzero = false;
    let mut turn_tick = None;
    let mut translated = false;
    let mut collisions = 0;
    let mut failure = None;
    let began = Instant::now();
    let mut planner = Planner::new();
    assert!(body_clear(start));
    for tick in 0..450 {
        let velocity = [
            applied[0] * pose.yaw.cos(),
            applied[0] * pose.yaw.sin(),
            0.0,
        ];
        let req = PlanRequest {
            obstacles,
            ground,
            reference: route,
            start_position: [pose.x, pose.y, pose.z],
            start_velocity: velocity,
            start_acceleration: acceleration,
            start_yaw: pose.yaw,
            start_yaw_rate: applied[1],
            odometry_timestamp_ns: tick as i64 * 100_000_000,
            config: config(),
            boundary: &boundary,
            forbidden_columns: &[],
            goal_yaw: None,
        };
        let result = match planner.step(&req) {
            Ok(value) => value,
            Err(reason) => {
                trace.push(json!({"tick":tick,"pose":[pose.x,pose.y,pose.z,pose.yaw],"applied":applied,"error":reason}));
                failure = Some(reason);
                break;
            }
        };
        let command = [result.command_forward, result.command_yaw_rate];
        let step = physical_step(pose, applied, command, result.command_duration);
        let (next, next_applied) = match step {
            Ok(value) => value,
            Err(reason) => {
                collisions += 1;
                failure = Some(reason.clone());
                trace.push(json!({"tick":tick,"pose":[pose.x,pose.y,pose.z,pose.yaw],"command":command,"collision":reason}));
                break;
            }
        };
        if command[0] != 0.0 || command[1] != 0.0 {
            nonzero = true;
        }
        if (next.yaw - start.yaw).abs() > 0.02 && turn_tick.is_none() {
            turn_tick = Some(tick);
        }
        if turn_tick.is_some_and(|previous| tick > previous)
            && (next.x - pose.x).hypot(next.y - pose.y) > 1e-6
        {
            translated = true;
        }
        trace.push(json!({"tick":tick,"time_s":tick as f64*0.1,"pose":[pose.x,pose.y,pose.z,pose.yaw],"applied":applied,
            "command":command,"next_pose":[next.x,next.y,next.z,next.yaw],"next_applied":next_applied,
            "reference_progress":result.reference_progress,"local_target":result.local_target,
            "local_trajectory":result.samples.iter().map(|p|json!({"t":p.time,"position":p.position,"velocity":p.velocity})).collect::<Vec<_>>() }));
        let next_velocity = [
            next_applied[0] * next.yaw.cos(),
            next_applied[0] * next.yaw.sin(),
            0.0,
        ];
        acceleration = std::array::from_fn(|i| (next_velocity[i] - velocity[i]) / 0.1);
        pose = next;
        applied = next_applied;
        if translated && (pose.x - start.x).hypot(pose.y - start.y) > 0.08 {
            break;
        }
    }
    // 真实制动末段同样核对，不能在首个平移之后直接删掉运动尾巴。
    while applied[0].abs() > 1e-12 || applied[1].abs() > 1e-12 {
        match physical_step(pose, applied, [0.0, 0.0], 0.1) {
            Ok((next, next_applied)) => {
                trace.push(json!({"stop":true,"pose":[pose.x,pose.y,pose.z,pose.yaw],"applied":applied,"command":[0.0,0.0],"next_pose":[next.x,next.y,next.z,next.yaw],"next_applied":next_applied}));
                pose = next;
                applied = next_applied;
            }
            Err(reason) => {
                collisions += 1;
                failure = Some(reason);
                break;
            }
        }
    }
    let blocked_before_motion = !nonzero && failure.is_some();
    let summary = json!({"case":name,"initial_pose":[start.x,start.y,start.z,start.yaw],"initial_body_clear":true,
        "first_turn_then_translation":translated,"rejected_before_nonzero_motion":blocked_before_motion,"collision_count":collisions,
        "failure":failure,"final_pose":[pose.x,pose.y,pose.z,pose.yaw],"final_applied":applied,"wall_seconds":began.elapsed().as_secs_f64(),"trace_frames":trace.len()});
    fs::write(
        directory.join(format!("{name}_命令轨迹.json")),
        serde_json::to_vec_pretty(&json!({"summary":summary,"trace":trace})).unwrap(),
    )
    .unwrap();
    summary
}
fn main() {
    let directory = PathBuf::from(std::env::args().nth(1).expect("需要证据输出目录"));
    fs::create_dir_all(&directory).unwrap();
    let mut keys = BTreeSet::new();
    for x in 0..4 {
        for y in -25..55 {
            for z in 0..65 {
                let point = [x, y, z].map(|v| ((v as f32 + 0.5) * 0.01_f32) as f64);
                keys.insert(point.map(|v| (v / 0.1).floor() as i32));
            }
        }
    }
    let obstacles: Vec<_> = keys
        .into_iter()
        .map(|p| p.map(|v| (v as f64 + 0.5) * 0.1))
        .collect();
    let ground: Vec<_> = (-18..18)
        .flat_map(|x| (-12..15).map(move |y| [(x as f64 + 0.5) * 0.1, (y as f64 + 0.5) * 0.1, 0.0]))
        .collect();
    let wide_q = [0.0, 0.0, 0.029995500202495466, 0.9995500337489875];
    let short_req_q = [0.0, 0.0, 0.04498481403765994, 0.9989876708478425];
    let short_late_q = [0.0, 0.0, 0.09115734222015381, 0.9958365021223905];
    let cases = [
        (
            "宽接近_请求姿态",
            Pose {
                x: -0.625,
                y: 0.15,
                z: 0.4,
                yaw: 0.0,
            },
            -1.2,
        ),
        (
            "宽接近_迟到姿态",
            Pose {
                x: -0.33401961360183036,
                y: 0.15117473073791413,
                z: 0.4,
                yaw: quaternion_yaw(wide_q),
            },
            -1.2,
        ),
        (
            "短接近_请求姿态",
            Pose {
                x: -0.45977023959198365,
                y: 0.15258247380354076,
                z: 0.4,
                yaw: quaternion_yaw(short_req_q),
            },
            -0.75,
        ),
        (
            "短接近_迟到姿态",
            Pose {
                x: -0.33607583041988814,
                y: 0.17963204671760727,
                z: 0.4,
                yaw: quaternion_yaw(short_late_q),
            },
            -0.75,
        ),
    ];
    let scene = json!({"source":"HEAD:route_partial_priority_tests.rs / route_partial_rejoin_tests.rs / route_follower.rs::comparison_config",
        "source_cell_size_f32":0.01,"source_floor_cells_half_open":[[-180,181],[-125,156],[-1,0]],"source_wall_cells_half_open":[[0,4],[-25,55],[0,65]],
        "source_point_formula":"(index as f32 + 0.5) * 0.01_f32","truth_wall_bounds":[[0.0,0.04],[-0.25,0.55],[0.0,0.65]],"truth_ground_z":0.0,
        "body_footprint":[[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],"body_z_bounds":[-0.15,0.15],"body_to_ground":0.4,
        "goal":[1.1,0.15,0.4],"boundary":[[-1.7,-1.15],[1.7,-1.15],[1.7,1.45],[-1.7,1.45]],
        "motion":{"max_forward":0.5,"max_reverse":0.0,"max_lateral":0.0,"max_yaw_rate":0.8,"linear_acceleration":1.0,"linear_braking":1.0,"angular_acceleration":2.0,"angular_braking":2.0,"reaction_time":0.05,"dt":0.1},
        "native_resolution":0.1,"planning_horizon":5.0,"obstacles":obstacles,"ground":ground,
        "original_quaternions":{"wide_requested":[0.0,0.0,0.0,1.0],"wide_late":wide_q,"short_requested":short_req_q,"short_late":short_late_q},
        "cases":cases.iter().map(|(name,p,start)|json!({"name":name,"pose":[p.x,p.y,p.z,p.yaw],"original_route":[[start,0.15,0.4],[1.1,0.15,0.4]]})).collect::<Vec<_>>()});
    fs::write(
        directory.join("原始场景.json"),
        serde_json::to_vec_pretty(&scene).unwrap(),
    )
    .unwrap();
    let summaries: Vec<_> = cases
        .iter()
        .map(|(name, p, start)| {
            run_case(
                name,
                *p,
                &[[*start, 0.15, 0.4], [1.1, 0.15, 0.4]],
                &obstacles,
                &ground,
                &directory,
            )
        })
        .collect();
    fs::write(
        directory.join("验证结果.json"),
        serde_json::to_vec_pretty(&summaries).unwrap(),
    )
    .unwrap();
    println!("{}", serde_json::to_string_pretty(&summaries).unwrap());
}
