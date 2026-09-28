//! 开发机导航模拟工作进程：复用生产规划与跟随，环境运动独立积分速度命令。
use nuwax_nav_mls_planner::{
    SurfaceMapBuilder, SurfaceObservation, SurfaceObservations, SurfaceSnapshot,
};
use nuwax_nav_runtime::{ExecutionMap, ExecutionPath, NavPlanner, PlannerStep, SurfaceFollower};
use nuwax_nav_stack::{surface_config::SurfaceNavigationConfig, surface_oriented};
use nuwax_nav_types_messages::{
    GoalOptions, MotionLimits, MotionModel, NavFrameId, Path3D, Point3, Pose3, PoseStamped,
    Quaternion, Timestamp, Twist2,
};
use serde_json::{json, Value};
use std::{
    collections::{BTreeMap, HashMap, HashSet},
    fs,
    io::{self, BufRead, Write},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Instant,
};

type Body = ([f64; 3], [f64; 4]);
const DT: f64 = 0.05;
static CANCEL: AtomicBool = AtomicBool::new(false);
extern "C" fn interrupt(_: i32) {
    CANCEL.store(true, Ordering::Relaxed);
}
// 朝向与四元数表示的整体尺度无关；原测量分量仍保留在实际位姿中。
fn unit_orientation(q: [f64; 4]) -> [f64; 4] {
    if q.iter().map(|v| v * v).sum::<f64>() == 1.0 {
        return q;
    }
    let scale = q.into_iter().map(f64::abs).fold(0.0, f64::max);
    let scaled = q.map(|v| v / scale);
    let norm = scaled.iter().map(|v| v * v).sum::<f64>().sqrt();
    scaled.map(|v| v / norm)
}
fn yaw(p: Body) -> f64 {
    let [x, y, z, w] = unit_orientation(p.1);
    (2. * (w * z + x * y)).atan2(1. - 2. * (y * y + z * z))
}
fn quaternion(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.).sin_cos();
    let (sp, cp) = (pitch / 2.).sin_cos();
    let (sy, cy) = (yaw / 2.).sin_cos();
    [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]
}
fn tilt(q: [f64; 4]) -> [f64; 2] {
    let [x, y, z, w] = unit_orientation(q);
    [
        (2. * (w * x + y * z)).atan2(1. - 2. * (x * x + y * y)),
        (2. * (w * y - z * x)).clamp(-1., 1.).asin(),
    ]
}
fn multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [x, y, z, w] = a;
    let [u, v, t, s] = b;
    [
        w * u + x * s + y * t - z * v,
        w * v - x * t + y * s + z * u,
        w * t + x * v - y * u + z * s,
        w * s - x * u - y * v - z * t,
    ]
}
fn pose4(p: Body) -> [f64; 4] {
    [p.0[0], p.0[1], p.0[2], yaw(p)]
}
fn body4(v: &Value) -> Result<Body, String> {
    let values = v.as_array().ok_or("位置需要XYZ及方向角")?;
    if values.len() != 4 {
        return Err("位置需要四个数值".into());
    }
    let mut p = [0.; 4];
    for (i, value) in values.iter().enumerate() {
        p[i] = value
            .as_f64()
            .filter(|x| x.is_finite())
            .ok_or("位置数值无效")?;
    }
    Ok(([p[0], p[1], p[2]], quaternion(0., 0., p[3])))
}
fn stamped(p: Body, t: f64) -> PoseStamped {
    PoseStamped {
        frame_id: NavFrameId::Map,
        stamp: Timestamp::from_unix(1 + (t.floor() as i64), ((t.fract()) * 1e9) as u32),
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
fn distance(a: Body, b: Body) -> f64 {
    (0..3)
        .map(|i| (a.0[i] - b.0[i]).powi(2))
        .sum::<f64>()
        .sqrt()
}
fn integrate(p: Body, v: [f64; 3], dt: f64) -> Body {
    // 恒定机体系速度的解析积分，独立实现，避免欧拉近似在转弯时制造轨迹差异。
    let angle = v[2] * dt;
    let (a, b) = if v[2].abs() < 1e-9 {
        (dt, 0.)
    } else {
        (angle.sin() / v[2], 2. * (angle / 2.).sin().powi(2) / v[2])
    };
    let dx = a * v[0] - b * v[1];
    let dy = b * v[0] + a * v[1];
    let y = yaw(p);
    let (s, c) = (angle / 2.).sin_cos();
    let q = p.1;
    (
        [
            p.0[0] + y.cos() * dx - y.sin() * dy,
            p.0[1] + y.sin() * dx + y.cos() * dy,
            p.0[2],
        ],
        [
            c * q[0] - s * q[1],
            c * q[1] + s * q[0],
            c * q[2] + s * q[3],
            c * q[3] - s * q[2],
        ],
    )
}
/// 环境地形变化作用于当前真实站姿，与规划路径无关。
// 机体标定高度沿自身法向；地面查询需要机体中心XY处的接触参考。
fn contact_reference(body: Body, offset: f64) -> f64 {
    let [roll, pitch] = tilt(body.1);
    body.0[2] - offset / (roll.cos() * pitch.cos())
}

fn body_after_ground_change(
    current: Body,
    planar: Body,
    before_ground: Body,
    after_ground: Body,
    support_offset: f64,
) -> Body {
    // 环境只提供地面变化，不能每拍把机器人重设到拟合高度和倾斜。
    let before_tilt = tilt(before_ground.1);
    let after_tilt = tilt(after_ground.1);
    let orientation = if before_ground.1 == after_ground.1 {
        current.1
    } else if before_tilt == [0.0; 2] && after_tilt == [0.0; 2] {
        planar.1
    } else {
        let q = before_ground.1;
        multiply(
            multiply(after_ground.1, [-q[0], -q[1], -q[2], q[3]]),
            current.1,
        )
    };
    // 地形改变横滚、俯仰和高度，平面命令的解析偏航仍是本拍实际偏航。
    let correction = yaw(planar) - yaw((planar.0, orientation));
    let correction = correction.sin().atan2(correction.cos());
    let orientation = if correction == 0.0 {
        orientation
    } else {
        multiply(quaternion(0.0, 0.0, correction), orientation)
    };
    // 地面高度在机体中心XY查询，实际支撑点还沿机体竖轴偏移。
    // 用两法向独立求该点与地平面的交高；坡面对齐时结果为d/n_z。
    let contact_height = |ground: [f64; 4], body: [f64; 4]| {
        let normal = |q: [f64; 4]| {
            let [x, y, z, w] = unit_orientation(q);
            [
                2.0 * (x * z + w * y),
                2.0 * (y * z - w * x),
                1.0 - 2.0 * (x * x + y * y),
            ]
        };
        let g = normal(ground);
        let b = normal(body);
        support_offset * (g[0] * b[0] + g[1] * b[1] + g[2] * b[2]) / g[2]
    };
    let height = if planar.0[..2] == current.0[..2] {
        current.0[2]
    } else {
        let contact_delta = contact_height(after_ground.1, orientation)
            - contact_height(before_ground.1, current.1);
        current.0[2] + (after_ground.0[2] - before_ground.0[2]) + contact_delta
    };
    let next = ([planar.0[0], planar.0[1], height], orientation);
    next
}

fn as_twist(v: [f64; 3]) -> Twist2 {
    Twist2 {
        linear: nuwax_nav_types_messages::Point2::new(v[0], v[1]),
        angular_z: v[2],
    }
}
fn velocity(t: Twist2) -> [f64; 3] {
    [t.linear.x, t.linear.y, t.angular_z]
}
fn rss_mb() -> f64 {
    fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("VmHWM:"))
                .and_then(|l| l.split_whitespace().nth(1))
                .and_then(|s| s.parse::<f64>().ok())
        })
        .unwrap_or(0.)
        / 1024.
}

/// 点云环境的地面查询独立于导航路线；按原始体素位置拟合局部地面。
struct Terrain {
    columns: HashMap<(i32, i32), Vec<(f64, f64)>>,
    flat: bool,
    source_voxel_m: f64,
    response_radius_m: f64,
    step_m: [f64; 2],
}
impl Terrain {
    fn new(
        points: &[(f32, f32, f32)],
        flat: bool,
        footprint: &[[f64; 2]],
        source_voxel_m: f32,
        step_m: [f64; 2],
    ) -> Self {
        // 实录是带高度误差的观测表面，同列连续样本形成接触区间；单层仍是原真实面。
        let source = f64::from(source_voxel_m);
        // 地形响应只取声明的机体足迹和源分辨率；独立于规划网格和规划结果。
        let response_radius_m = footprint
            .iter()
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f64::max)
            + source;
        let mut occupied: HashMap<(i32, i32), Vec<i32>> = HashMap::new();
        for &(x, y, z) in points {
            occupied
                .entry((
                    (x as f64 / source).floor() as i32,
                    (y as f64 / source).floor() as i32,
                ))
                .or_default()
                .push((z as f64 / source).floor() as i32);
        }
        let columns = occupied
            .into_iter()
            .map(|(key, mut cells)| {
                cells.sort_unstable();
                cells.dedup();
                let bands = cells
                    .iter()
                    .enumerate()
                    .filter_map(|(i, &z)| {
                        if cells.get(i + 1).copied() == Some(z + 1) {
                            return None;
                        }
                        let mut bottom = i;
                        while bottom > 0 && cells[bottom - 1] + 1 == cells[bottom] {
                            bottom -= 1;
                        }
                        // 原始点只声明占据格；区间两端取真实占据格顶，不扩到格底。
                        Some((
                            f64::from(cells[bottom] + 1) * source,
                            f64::from(z + 1) * source,
                        ))
                    })
                    .collect();
                (key, bands)
            })
            .collect();
        Self {
            columns,
            flat,
            source_voxel_m: source,
            response_radius_m,
            step_m,
        }
    }

    fn contact_samples(
        &self,
        xy: [f64; 2],
        reference: f64,
    ) -> Result<Vec<(f64, f64, f64, f64, f64)>, String> {
        let source = self.source_voxel_m;
        let key = (
            (xy[0] / source).floor() as i32,
            (xy[1] / source).floor() as i32,
        );
        let enumeration = self.response_radius_m + std::f64::consts::SQRT_2 * source;
        let cells = (enumeration / source).ceil() as i32;
        let gap = |low: f64, high: f64| (low - reference).max(reference - high).max(0.0);
        let mut selected = HashMap::new();
        for ix in key.0 - cells..=key.0 + cells {
            for iy in key.1 - cells..=key.1 + cells {
                let x = (ix as f64 + 0.5) * source - xy[0];
                let y = (iy as f64 + 0.5) * source - xy[1];
                let radius2 = x * x + y * y;
                if radius2 > enumeration * enumeration {
                    continue;
                }
                let Some(bands) = self.columns.get(&(ix, iy)) else {
                    continue;
                };
                // 先关联整个区间；不把近厚带删掉后再改选远薄带。
                let Some(&(low, high)) = bands.iter().min_by(|a, b| {
                    gap(a.0, a.1)
                        .total_cmp(&gap(b.0, b.1))
                        .then_with(|| (a.1 - reference).abs().total_cmp(&(b.1 - reference).abs()))
                        .then_with(|| a.1.total_cmp(&b.1))
                }) else {
                    continue;
                };
                selected.insert((ix, iy), (x, y, low, high, radius2));
            }
        }
        let seed = selected
            .iter()
            .filter(|(_, p)| {
                p.3 - reference <= self.step_m[0] + source + 1e-6
                    && reference - p.3 <= self.step_m[1] + source + 1e-6
            })
            .min_by(|a, b| {
                a.1 .4
                    .total_cmp(&b.1 .4)
                    .then_with(|| gap(a.1 .2, a.1 .3).total_cmp(&gap(b.1 .2, b.1 .3)))
                    .then_with(|| {
                        (a.1 .3 - reference)
                            .abs()
                            .total_cmp(&(b.1 .3 - reference).abs())
                    })
                    .then_with(|| a.1 .3.total_cmp(&b.1 .3))
                    .then_with(|| a.0.cmp(b.0))
            })
            .map(|(key, _)| *key)
            .ok_or("模拟环境缺少接触参考附近的真实暴露地面")?;
        let mut queue = vec![seed];
        let mut seen = HashSet::from([seed]);
        let mut cursor = 0;
        let mut samples = Vec::with_capacity(selected.len());
        while cursor < queue.len() {
            let key = queue[cursor];
            cursor += 1;
            let point = selected[&key];
            if point.4 < self.response_radius_m * self.response_radius_m {
                let taper = 1.0 - point.4 / (self.response_radius_m * self.response_radius_m);
                samples.push((point.0, point.1, point.2, point.3, taper * taper));
            }
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    // 估计域跨过量化空行，只取该方向第一列真实观测；不补点、不跨墙。
                    let neighbor = (1..=2*cells).map(|step|(key.0+dx*step,key.1+dy*step))
                        .find(|candidate|selected.contains_key(candidate));
                    let Some(neighbor)=neighbor else {continue;};
                    let next=&selected[&neighbor];
                    let rise = next.3 - point.3;
                    if rise <= self.step_m[0] + source + 1e-6
                        && -rise <= self.step_m[1] + source + 1e-6
                        && seen.insert(neighbor)
                    {
                        queue.push(neighbor);
                    }
                }
            }
        }
        samples.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
        if samples.len() < 3 {
            return Err("模拟环境的当前连通地面观测不足".into());
        }
        Ok(samples)
    }

    fn body_at(
        &self,
        xy: [f64; 2],
        reference: f64,
        heading: f64,
        offset: f64,
    ) -> Result<Body, String> {
        if self.flat {
            return Ok(([xy[0], xy[1], offset], quaternion(0., 0., heading)));
        }
        let samples = self.contact_samples(xy, reference)?;
        let (mut sw, mut sx, mut sy, mut xx, mut yy, mut xy_sum) = (0., 0., 0., 0., 0., 0.);
        for &(x, y, _, _, w) in &samples {
            sw += w;
            sx += w * x;
            sy += w * y;
            xx += w * x * x;
            yy += w * y * y;
            xy_sum += w * x * y;
        }
        let a = xx - sx * sx / sw;
        let b = yy - sy * sy / sw;
        let c = xy_sum - sx * sy / sw;
        let determinant = a * b - c * c;
        if determinant <= 0.0 {
            return Err("模拟地面采样共线，无法确定支撑平面".into());
        }
        // 直接最小化平面到观测区间的残差。每轮投影当前平面，不能把样本
        // 固定为每拍变化的实际参考；否则相对地形旋转会在台阶往返中累计漂移。
        // 单层等价于原加权最小二乘，共同可行的水平面无需反复求解。
        let mut plane = [0.0, 0.0, reference];
        for _ in 0..128 {
            let (fx, fy, low, high, _) = samples[0];
            let origin_z = (plane[0] * fx + plane[1] * fy + plane[2]).clamp(low, high);
            let (mut sz, mut xz, mut yz) = (0., 0., 0.);
            for &(x, y, low, high, w) in &samples {
                let z = (plane[0] * x + plane[1] * y + plane[2]).clamp(low, high) - origin_z;
                sz += w * z;
                xz += w * x * z;
                yz += w * y * z;
            }
            let d = xz - sx * sz / sw;
            let e = yz - sy * sz / sw;
            let gx = (d * b - e * c) / determinant;
            let gy = (e * a - d * c) / determinant;
            let next = [gx, gy, origin_z + (sz - gx * sx - gy * sy) / sw];
            let change = (0..3)
                .map(|i| (next[i] - plane[i]).abs())
                .fold(0.0, f64::max);
            plane = next;
            if change < 1e-12 {
                break;
            }
        }
        let [gx, gy, z] = plane;
        let pitch = -(gx * heading.cos() + gy * heading.sin()).atan();
        // 两个欧拉角共同表示同一世界地面法向，不能分别近似后随偏航改变平面。
        let roll = ((-gx * heading.sin() + gy * heading.cos()) * pitch.cos()).atan();
        Ok(([xy[0], xy[1], z + offset], quaternion(roll, pitch, heading)))
    }
}

struct Scene {
    id: String,
    points: Vec<(f32, f32, f32)>,
    poses: Vec<Body>,
    cases: Vec<Value>,
    terrain: Terrain,
    config: SurfaceNavigationConfig,
    builder: SurfaceMapBuilder,
    base_observations: Arc<SurfaceObservations>,
    map: Arc<SurfaceSnapshot>,
    obstacles: Vec<Value>,
    metadata: Value,
}
fn observation_index(points: &[(f32, f32, f32)]) -> Result<SurfaceObservations, String> {
    let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
    for &(x, y, z) in points {
        let k = [x, y, z].map(|v| (f64::from(v) / f64::from(0.05_f32)).floor() as i32);
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
    let mut result = SurfaceObservations::new(0.05, 32).map_err(|e| e.to_string())?;
    for (key, values) in blocks {
        result
            .replace_block(key, values)
            .map_err(|e| e.to_string())?;
    }
    result.mark_complete_geometry_source();
    Ok(result)
}
impl Scene {
    fn make(
        id: &str,
        name: &str,
        points: Vec<(f32, f32, f32)>,
        poses: Vec<Body>,
        config: SurfaceNavigationConfig,
        data: &PathBuf,
    ) -> Result<Self, String> {
        let offset = config.follower.body_to_support_z_m;
        let body_model = config.body_envelope.as_ref().ok_or("模拟机体足迹缺失")?;
        let terrain = Terrain::new(
            &points,
            id != "recorded",
            &body_model.footprint_xy_m,
            0.05,
            [body_model.max_step_up_m, body_model.max_step_down_m],
        );
        let mut builder =
            SurfaceMapBuilder::configured(config.geometry.clone(), config.body_envelope.clone());
        let observations = Arc::new(observation_index(&points)?);
        let map = Arc::new(
            builder
                .build(&points, 0.05)
                .map_err(|e| e.to_string())?
                .snapshot
                .with_observations(Some(observations.clone())),
        );
        let mut cases = Vec::new();
        if id == "recorded" {
            for i in (0..poses.len()).step_by(800) {
                if let Some(j) = ((i + 1)..poses.len()).find(|&j| {
                    (7.0..=7.15).contains(&distance(poses[i], poses[j]))
                        && (poses[i].0[2] - poses[j].0[2]).abs() < 0.15
                }) {
                    cases.push(json!({"id":format!("recorded-{i}-{j}"),"name":format!("七米实录 · {i} → {j}"),"start":pose4(poses[i]),"goal":pose4(poses[j]),"start_frame":i,"goal_frame":j}));
                }
            }
        } else {
            cases.push(json!({"id":format!("{id}-seven"),"name":if id=="straight"{"七米直线"}else{"七米绕障"},"start":[0.,0.,offset,0.],"goal":[7.,0.,offset,0.]}));
        }
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for &(x, y, z) in &points {
            for (i, v) in [x as f64, y as f64, z as f64].into_iter().enumerate() {
                bounds[i] = bounds[i].min(v);
                bounds[i + 3] = bounds[i + 3].max(v);
            }
        }
        let stride = points.len().div_ceil(220_000).max(1);
        let mut bytes = Vec::with_capacity(points.len() / stride * 12);
        for &(x, y, z) in points.iter().step_by(stride) {
            for v in [x, y, z] {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        fs::write(data.join(format!("{id}.bin")), &bytes).map_err(|e| e.to_string())?;
        let default = cases
            .first()
            .cloned()
            .unwrap_or(json!({"start":[0.,0.,offset,0.],"goal":[7.,0.,offset,0.]}));
        let model = config.body_envelope.as_ref().unwrap();
        let metadata = json!({"id":id,"name":name,"description":if id=="recorded"{"指定实录三维点云，独立体素地面运动模型"}else{"已知平面环境，独立速度积分与加减速响应"},"default_start":default["start"],"default_goal":default["goal"],"point_count":points.len(),"display_point_count":bytes.len()/12,"bounds":bounds,"body_to_support_z_m":offset,"robot":{"footprint_xy":model.footprint_xy_m,"z_bounds":model.z_bounds_m},"terrain_response":{"radius_m":terrain.response_radius_m,"source_voxel_m":terrain.source_voxel_m,"weight":"(1-r²/R²)²","calibration":"declared_footprint_plus_source_voxel","domain":"nearest_observed_columns_in_eight_directions","max_step_up_m":terrain.step_m[0],"max_step_down_m":terrain.step_m[1]},"world_model":if id=="recorded"{"pointcloud_terrain"}else{"flat"}});
        Ok(Self {
            id: id.into(),
            points,
            poses,
            cases,
            terrain,
            config,
            builder,
            base_observations: observations,
            map,
            obstacles: Vec::new(),
            metadata,
        })
    }
    fn change_obstacles(&mut self, values: Vec<Value>) -> Result<(), String> {
        if values == self.obstacles {
            return Ok(());
        }
        let mut extra = Vec::new();
        for obstacle in &values {
            let x = obstacle["x"].as_f64().ok_or("障碍X缺失")?;
            let y = obstacle["y"].as_f64().ok_or("障碍Y缺失")?;
            let radius = obstacle["radius"].as_f64().unwrap_or(0.3).clamp(0.05, 2.);
            let height = obstacle["height"].as_f64().unwrap_or(0.8).clamp(0.1, 2.);
            let reference = obstacle["z"].as_f64().unwrap_or_else(|| {
                self.poses
                    .iter()
                    .min_by(|a, b| {
                        ((a.0[0] - x).hypot(a.0[1] - y))
                            .total_cmp(&((b.0[0] - x).hypot(b.0[1] - y)))
                    })
                    .map_or(0., |p| p.0[2] - self.config.follower.body_to_support_z_m)
            });
            for ix in ((x - radius) / 0.05).floor() as i32..=((x + radius) / 0.05).ceil() as i32 {
                for iy in ((y - radius) / 0.05).floor() as i32..=((y + radius) / 0.05).ceil() as i32
                {
                    let px = (ix as f64 + 0.5) * 0.05;
                    let py = (iy as f64 + 0.5) * 0.05;
                    if (px - x).hypot(py - y) > radius {
                        continue;
                    }
                    for iz in 1..=(height / 0.05).ceil() as i32 {
                        extra.push((px as f32, py as f32, (reference + iz as f64 * 0.05) as f32));
                    }
                }
            }
        }
        let mut points = self.points.clone();
        points.extend(extra.iter().copied());
        let built = self
            .builder
            .build(&points, 0.05)
            .map_err(|e| e.to_string())?;
        // 地图构建器只重建变化列，原观察块共享；交互障碍变化不进入每拍热路径。
        let mut observations = (*self.base_observations).clone();
        let mut blocks = BTreeMap::<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>>::new();
        for &(x, y, z) in &extra {
            let key = [x, y, z].map(|v| (f64::from(v) / f64::from(0.05_f32)).floor() as i32);
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
        for (block, mut values) in blocks {
            for x in block[0] * 32..(block[0] + 1) * 32 {
                for y in block[1] * 32..(block[1] + 1) * 32 {
                    for z in block[2] * 32..(block[2] + 1) * 32 {
                        if let Some(v) = self.base_observations.get([x, y, z]) {
                            values.entry([x, y, z]).or_insert(v);
                        }
                    }
                }
            }
            observations
                .replace_block(block, values)
                .map_err(|e| e.to_string())?;
        }
        self.map = Arc::new(
            built
                .snapshot
                .with_observations(Some(Arc::new(observations))),
        );
        self.obstacles = values;
        Ok(())
    }
}

struct Simulation {
    scene: String,
    follower: SurfaceFollower,
    options: GoalOptions,
    current: Body,
    goal: Body,
    applied: [f64; 3],
    command: [f64; 3],
    speed: f64,
    time: f64,
    status: String,
    message: String,
    need_plan: bool,
    plans: u64,
    last_plan_ms: f64,
    last_control_ms: f64,
    path: Vec<Body>,
    pending_frames: Vec<Value>,
    history: BTreeMap<String, Value>,
    body_history: BTreeMap<String, Value>,
    keep_body_history: bool,
    motion_failures: usize,
    motion_failure: Option<Value>,
    reverse_travel_m: f64,
    lateral_travel_m: f64,
    max_reverse_command_mps: f64,
    max_lateral_command_mps: f64,
    pose_mode: String,
    braking_for_replan: bool,
}
impl Simulation {
    fn new(scene: &Scene, request: &Value) -> Result<Self, String> {
        let start = body4(&request["start"])?;
        let goal = body4(&request["goal"])?;
        let speed = request["speed_mps"].as_f64().unwrap_or(1.).clamp(0.1, 1.5);
        let mut config = scene.config.follower.clone();
        config.controller.speed = speed;
        config.controller.control_frequency = 1. / DT;
        config.controller.simulation = true;
        config.max_angular_speed_rps = 0.8;
        config.basic_kinematics = Some(nuwax_nav_types_messages::MotionKinematics {
            model: MotionModel::Omnidirectional,
            max_forward_mps: speed,
            max_reverse_mps: speed,
            max_lateral_mps: speed,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.,
            can_rotate_in_place: true,
        });
        config.motion_limits = Some(MotionLimits {
            model: MotionModel::Omnidirectional,
            max_forward_mps: speed,
            max_reverse_mps: speed,
            max_lateral_mps: speed,
            max_angular_rps: 0.8,
            linear_acceleration_mps2: 1.,
            guaranteed_braking_mps2: 1.,
            angular_acceleration_rps2: 2.,
            guaranteed_angular_braking_rps2: 2.,
            reaction_time_sec: 0.05,
            minimum_turning_radius_m: 0.,
            can_rotate_in_place: true,
        });
        let options = GoalOptions {
            allow_reverse: false,
            position_tolerance_m: 0.35,
            yaw_tolerance_rad: if request["requires_orientation"].as_bool().unwrap_or(false) {
                0.15
            } else {
                std::f64::consts::PI
            },
        };
        let mut current = start;
        // 自定义起点按所选地面初始化站姿；实录预设随后恢复原始测量姿态。
        if let Ok(ground) = scene.terrain.body_at(
            [start.0[0], start.0[1]],
            start.0[2] - scene.config.follower.body_to_support_z_m,
            yaw(start),
            scene.config.follower.body_to_support_z_m,
        ) {
            current.1 = ground.1;
        }
        if let Some(case_id) = request["case_id"].as_str() {
            if let Some(case) = scene
                .cases
                .iter()
                .find(|c| c["id"].as_str() == Some(case_id))
            {
                if let Some(i) = case["start_frame"].as_u64() {
                    let measured = scene.poses[i as usize];
                    if distance(start, measured) < 0.01 {
                        let t = tilt(measured.1);
                        current.1 = quaternion(t[0], t[1], yaw(start));
                    }
                }
            }
        }
        let mut follower = SurfaceFollower::new(config).map_err(|e| e.to_string())?;
        follower
            .update_execution_map(ExecutionMap::Surface(scene.map.clone()))
            .map_err(|e| e.to_string())?;
        follower.update_odometry(stamped(current, 0.));
        follower
            .configure_goal(&stamped(goal, 0.), options)
            .map_err(|e| e.to_string())?;
        Ok(Self {
            scene: scene.id.clone(),
            follower,
            options,
            current,
            goal,
            applied: [0.; 3],
            command: [0.; 3],
            speed,
            time: 0.,
            status: "planning".into(),
            message: "计算完整三维路线".into(),
            need_plan: true,
            plans: 0,
            last_plan_ms: 0.,
            last_control_ms: 0.,
            path: Vec::new(),
            pending_frames: Vec::new(),
            history: BTreeMap::new(),
            body_history: BTreeMap::new(),
            keep_body_history: request["include_body_paths"].as_bool().unwrap_or(false),
            motion_failures: 0,
            motion_failure: None,
            reverse_travel_m: 0.0,
            lateral_travel_m: 0.0,
            max_reverse_command_mps: 0.0,
            max_lateral_command_mps: 0.0,
            pose_mode: if scene.terrain.flat {
                "flat"
            } else {
                "pointcloud_terrain"
            }
            .into(),
            braking_for_replan: false,
        })
    }
    fn plan(&mut self, scene: &Scene) -> Result<(), String> {
        let started = Instant::now();
        let motion = self.follower_config_motion();
        let map = scene
            .map
            .with_current_body_support(self.current.0, self.current.1)
            .map_err(|e| e.to_string())?;
        let route = surface_oriented::plan_with_motion_cancellable(
            &map,
            self.current,
            self.goal,
            self.options,
            Some(&motion),
            &[],
            30_000,
            Some(&CANCEL),
        );
        self.last_plan_ms = started.elapsed().as_secs_f64() * 1000.;
        self.plans += 1;
        self.path = route?;
        if self.keep_body_history {self.body_history.insert(self.plans.to_string(),json!(self.path));}
        self.history.insert(
            self.plans.to_string(),
            json!(self.path.iter().map(|p| p.0).collect::<Vec<_>>()),
        );
        self.follower
            .replace_execution_path(
                ExecutionPath::Surface(Path3D {
                    poses: self.path.iter().map(|p| stamped(*p, self.time)).collect(),
                }),
                None,
                stamped(self.goal, self.time),
            )
            .map_err(|e| e.to_string())?;
        self.need_plan = false;
        self.status = "running".into();
        self.message = "沿完整全局路线前进".into();
        Ok(())
    }
    fn follower_config_motion(&self) -> nuwax_nav_types_messages::MotionKinematics {
        nuwax_nav_types_messages::MotionKinematics {
            model: MotionModel::Omnidirectional,
            max_forward_mps: self.speed,
            max_reverse_mps: self.speed,
            max_lateral_mps: self.speed,
            max_angular_rps: 0.8,
            minimum_turning_radius_m: 0.,
            can_rotate_in_place: true,
        }
    }
    fn tick(&mut self, scene: &Scene) -> Result<(), String> {
        let planned_this_tick = self.need_plan;
        if self.need_plan {
            self.plan(scene)?;
        }
        let began = Instant::now();
        let step = if self.braking_for_replan {
            PlannerStep::Command(Twist2::ZERO)
        } else {
            self.follower.step(scene.config.robot_width_m)
        };
        self.last_control_ms = began.elapsed().as_secs_f64() * 1000.;
        let mut arriving = false;
        match step {
            PlannerStep::Command(command) => self.command = velocity(command),
            PlannerStep::Succeeded => {
                self.command = [0.; 3];
                arriving = true;
            }
            PlannerStep::ReplanRequired(reason) => {
                self.command = [0.; 3];
                self.message = reason;
                self.braking_for_replan = true;
                self.status = "planning".into();
            }
            PlannerStep::Failed(reason) => return Err(reason),
        }
        self.max_reverse_command_mps=self.max_reverse_command_mps.max(-self.command[0]);
        self.max_lateral_command_mps=self.max_lateral_command_mps.max(self.command[1].abs());
        let old = self.applied;
        let dx = self.command[0] - old[0];
        let dy = self.command[1] - old[1];
        let change = dx.hypot(dy);
        let scale = (DT / change.max(1e-12)).min(1.);
        let applied = [
            old[0] + dx * scale,
            old[1] + dy * scale,
            old[2] + (self.command[2] - old[2]).clamp(-2. * DT, 2. * DT),
        ];
        let mean = std::array::from_fn(|i| (old[i] + applied[i]) * 0.5);
        let planar = integrate(self.current, mean, DT);
        let offset = scene.config.follower.body_to_support_z_m;
        let heading = yaw(planar);
        let before_ground = scene.terrain.body_at(
            [self.current.0[0], self.current.0[1]],
            contact_reference(self.current, offset),
            yaw(self.current),
            offset,
        )?;
        let after_ground = scene.terrain.body_at(
            [planar.0[0], planar.0[1]],
            contact_reference(self.current, offset),
            heading,
            offset,
        )?;
        let next = body_after_ground_change(
            self.current,
            planar,
            before_ground,
            after_ground,
            scene.config.follower.body_to_support_z_m,
        );
        let valid = scene
            .map
            .with_current_body_support(self.current.0, self.current.1)
            .and_then(|m| m.validate_body_motion(self.current, next, &[]));
        if let Err(error) = valid {
            self.motion_failures += 1;
            let predicted = scene
                .map
                .with_current_body_support(self.current.0, self.current.1)
                .and_then(|map| {
                    map.body_pose_after_motion(self.current, [planar.0[0], planar.0[1]], heading)
                });
            let predicted_check = predicted.as_ref().ok().map(|next| {
                scene
                    .map
                    .with_current_body_support(self.current.0, self.current.1)
                    .and_then(|map| map.validate_body_motion(self.current, *next, &[]))
                    .map_err(|e| e.to_string())
            });
            self.motion_failure = Some(json!({"from":self.current,"next":next,
                "previous_velocity":old,"velocity":applied,"command":self.command,
                "model_ground_from":before_ground,"model_ground_next":after_ground,
                "production_prediction":predicted.map_err(|e| e.to_string()),"prediction_check":predicted_check}));
            return Err(format!(
                "模拟实际运动与几何检查不一致：{error}，位置 {:?} → {:?}",
                self.current.0, next.0
            ));
        }
        // 按真实机体速度统计；圆弧的世界坐标横向位移不等于机器狗横着走。
        self.reverse_travel_m+=(-mean[0]).max(0.0)*DT;
        self.lateral_travel_m+=mean[1].abs()*DT;
        self.current = next;
        self.applied = applied;
        self.time += DT;
        if self.braking_for_replan
            && self.applied[0].hypot(self.applied[1]) < 0.005
            && self.applied[2].abs() < 0.01
        {
            self.braking_for_replan = false;
            self.need_plan = true;
        }
        self.follower
            .observe_applied_command(as_twist(self.applied));
        self.follower
            .update_odometry(stamped(self.current, self.time));
        if arriving
            && self.applied[0].hypot(self.applied[1]) < 0.005
            && self.applied[2].abs() < 0.01
        {
            let angle = (yaw(self.current) - yaw(self.goal) + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            if distance(self.current, self.goal) <= self.options.position_tolerance_m
                && (!self.options.requires_orientation()
                    || angle.abs() <= self.options.yaw_tolerance_rad)
            {
                self.status = "succeeded".into();
                self.message = "到达目标并停稳".into();
            } else {
                self.need_plan = true;
            }
        }
        self.pending_frames.push(json!({"t":self.time,"pose":pose4(self.current),"orientation":self.current.1,"command":self.command,"velocity":self.applied,"path_revision":self.plans,"status":self.status,"message":self.message,"control_ms":self.last_control_ms,"plan_ms":if planned_this_tick {self.last_plan_ms} else {0.0}}));
        Ok(())
    }
    fn state(&mut self, scene: &Scene) -> Value {
        let local: Vec<_> = (0..=12)
            .map(|i| integrate(self.current, self.command, i as f64 * 0.05).0)
            .collect();
        json!({"status":self.status,"sim_time_sec":self.time,"pose":pose4(self.current),"orientation":self.current.1,"goal":pose4(self.goal),"command":self.command,"velocity":self.applied,"global_path":self.path.iter().map(|p|p.0).collect::<Vec<_>>(),"path_revision":self.plans,"path_history":self.history,"local_path":local,"frames":std::mem::take(&mut self.pending_frames),"obstacles":scene.obstacles,"world_model":self.pose_mode,"terrain_response":scene.metadata["terrain_response"],"motion_failure":self.motion_failure,"metrics":{"speed_mps":self.applied[0].hypot(self.applied[1]),"configured_speed_mps":self.speed,"distance_remaining_m":distance(self.current,self.goal),"global_plans":self.plans,"last_plan_ms":self.last_plan_ms,"last_control_ms":self.last_control_ms,"max_rss_mb":rss_mb(),"motion_validation_failures":self.motion_failures,"reverse_travel_m":self.reverse_travel_m,"lateral_travel_m":self.lateral_travel_m,"max_reverse_command_mps":self.max_reverse_command_mps,"max_lateral_command_mps":self.max_lateral_command_mps},"motion_policy":"forward_only","message":self.message})
    }
}
fn synthetic(blocked: bool) -> Vec<(f32, f32, f32)> {
    let mut points = Vec::new();
    for x in -20..180 {
        for y in -70..70 {
            points.push(((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05, -0.025));
        }
    }
    if blocked {
        for x in 60..68 {
            for y in -18..18 {
                for z in 0..20 {
                    points.push((
                        (x as f32 + 0.5) * 0.05,
                        (y as f32 + 0.5) * 0.05,
                        (z as f32 + 0.5) * 0.05,
                    ));
                }
            }
        }
    }
    points
}
fn run() -> Result<(), String> {
    unsafe {
        libc::signal(libc::SIGUSR1, interrupt as libc::sighandler_t);
    }
    let args: Vec<_> = std::env::args().collect();
    let data = PathBuf::from(args.get(1).ok_or("需要模拟数据目录")?);
    fs::create_dir_all(&data).map_err(|e| e.to_string())?;
    let config = SurfaceNavigationConfig::from_env().map_err(|e| e.to_string())?;
    let mut scenes = HashMap::new();
    for (id, name, blocked) in [
        ("straight", "七米开阔直线", false),
        ("obstacle", "三维障碍绕行", true),
    ] {
        let scene = Scene::make(
            id,
            name,
            synthetic(blocked),
            Vec::new(),
            config.clone(),
            &data,
        )?;
        scenes.insert(id.to_owned(), scene);
    }
    if let (Some(cloud), Some(trajectory)) = (args.get(2), args.get(3)) {
        if let Ok(bytes) = fs::read(cloud) {
            let points = bytes
                .chunks_exact(16)
                .map(|b| {
                    (
                        f32::from_le_bytes(b[0..4].try_into().unwrap()),
                        f32::from_le_bytes(b[4..8].try_into().unwrap()),
                        f32::from_le_bytes(b[8..12].try_into().unwrap()),
                    )
                })
                .collect();
            let poses = fs::read_to_string(trajectory)
                .map_err(|e| e.to_string())?
                .lines()
                .map(|s| {
                    let v: Value = serde_json::from_str(s).unwrap();
                    (
                        std::array::from_fn(|i| v["p"][i].as_f64().unwrap()),
                        std::array::from_fn(|i| v["q"][i].as_f64().unwrap()),
                    )
                })
                .collect();
            let scene = Scene::make(
                "recorded",
                "9月18日最新代码遥控建图-1512",
                points,
                poses,
                config.clone(),
                &data,
            )?;
            scenes.insert("recorded".into(), scene);
        }
    }
    let catalog = json!({"scenes":[scenes["straight"].metadata,scenes["obstacle"].metadata],"default_scene":"straight"});
    let mut catalog = catalog;
    if let Some(scene) = scenes.get("recorded") {
        catalog["scenes"]
            .as_array_mut()
            .unwrap()
            .push(scene.metadata.clone());
    }
    fs::write(
        data.join("scenes.json"),
        serde_json::to_vec_pretty(&catalog).unwrap(),
    )
    .map_err(|e| e.to_string())?;
    for (id, scene) in &scenes {
        fs::write(
            data.join(format!("{id}-cases.json")),
            serde_json::to_vec(&json!({"cases":scene.cases})).unwrap(),
        )
        .map_err(|e| e.to_string())?;
    }
    println!("{}", json!({"ready":true,"catalog":catalog}));
    io::stdout().flush().unwrap();
    let mut simulation: Option<Simulation> = None;
    for line in io::stdin().lock().lines() {
        let line = line.map_err(|e| e.to_string())?;
        let response = (|| -> Result<Value, String> {
            let request: Value = serde_json::from_str(&line).map_err(|e| e.to_string())?;
            match request["op"].as_str().unwrap_or("") {
                "start" => {
                    CANCEL.store(false, Ordering::Relaxed);
                    let id = request["scene_id"].as_str().ok_or("场景缺失")?;
                    let scene = scenes.get_mut(id).ok_or("场景不存在")?;
                    scene.change_obstacles(
                        request["obstacles"].as_array().cloned().unwrap_or_default(),
                    )?;
                    let mut session = Simulation::new(scene, &request)?;
                    let state = session.state(scene);
                    simulation = Some(session);
                    Ok(state)
                }
                // 完整站姿仅按诊断请求导出，避免每拍重复传输四元数历史。
                "body_paths" => Ok(json!({"body_path_history":simulation.as_ref().ok_or("尚未开始")?.body_history})),
                "step" => {
                    let session = simulation.as_mut().ok_or("尚未开始")?;
                    let scene = scenes.get(&session.scene).unwrap();
                    for _ in 0..request["count"].as_u64().unwrap_or(1).clamp(1, 8) {
                        if !matches!(session.status.as_str(), "running" | "planning") {
                            break;
                        }
                        if let Err(error) = session.tick(scene) {
                            session.status = "failed".into();
                            session.command = [0.; 3];
                            session.message = error;
                            break;
                        }
                    }
                    Ok(session.state(scene))
                }
                "obstacles" => {
                    let session = simulation.as_mut().ok_or("尚未开始")?;
                    let scene = scenes.get_mut(&session.scene).unwrap();
                    scene.change_obstacles(
                        request["obstacles"].as_array().cloned().unwrap_or_default(),
                    )?;
                    session
                        .follower
                        .update_execution_map(ExecutionMap::Surface(scene.map.clone()))
                        .map_err(|e| e.to_string())?;
                    if session.status == "failed" {
                        session.need_plan = true;
                        session.status = "planning".into();
                    }
                    Ok(session.state(scene))
                }
                "pose" => {
                    let id = request["scene_id"].as_str().ok_or("场景缺失")?;
                    let scene = scenes.get(id).ok_or("场景不存在")?;
                    let p = &request["position"];
                    let xyz = [
                        p[0].as_f64().ok_or("X缺失")?,
                        p[1].as_f64().ok_or("Y缺失")?,
                        p[2].as_f64().ok_or("Z缺失")?,
                    ];
                    let mut body = scene.terrain.body_at(
                        [xyz[0], xyz[1]],
                        xyz[2],
                        0.,
                        scene.config.follower.body_to_support_z_m,
                    )?;
                    // 点选创建的是贴合坡面的初始机体，按旋转后的标定支撑点设置中心高度。
                    // 运行中的地面查询继续只提供中心高度场，由相对运动单独处理接触差分。
                    let [x, y, _, _] = unit_orientation(body.1);
                    let normal_z = 1.0 - 2.0 * (x * x + y * y);
                    body.0[2] += scene.config.follower.body_to_support_z_m * (1.0 / normal_z - 1.0);
                    Ok(
                        json!({"pose":pose4(body),"orientation":body.1,"message":"按本场景地面给出机体位置"}),
                    )
                }
                _ => Err("未知模拟命令".into()),
            }
        })();
        println!(
            "{}",
            match response {
                Ok(value) => value,
                Err(error) => json!({"error":error}),
            }
        );
        io::stdout().flush().unwrap();
    }
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quaternion_scale_does_not_change_environment_orientation_or_motion() {
        let q = quaternion(0.17, -0.09, 0.7);
        let original = ([1.0, 2.0, 0.4], q);
        for scale in [0.25, -2.5, 1.0 + 1e-14] {
            let scaled = (original.0, q.map(|v| v * scale));
            assert!((yaw(original) - yaw(scaled)).abs() < 1e-14);
            for (a, b) in tilt(q).into_iter().zip(tilt(scaled.1)) {
                assert!((a - b).abs() < 1e-14);
            }
            let mut a = original;
            let mut b = scaled;
            for _ in 0..100 {
                a = integrate(a, [0.8, 0.2, 0.3], DT);
                b = integrate(b, [0.8, 0.2, 0.3], DT);
            }
            assert!(distance(a, b) < 1e-12, "相同旋转的缩放表示不能改变真实位移");
        }
    }

    fn test_terrain(points: &[(f32, f32, f32)], flat: bool) -> Terrain {
        Terrain::new(
            points,
            flat,
            &[[-0.3, -0.16], [0.3, -0.16], [0.3, 0.16], [-0.3, 0.16]],
            0.05,
            [0.15, 0.15],
        )
    }

    #[test]
    fn declared_footprints_share_the_same_analytic_plane() {
        // 相同真实平面的解析高度和法向不能随响应域或机体朝向改变。
        let source = f64::from(0.05_f32);
        let mut points = Vec::new();
        for ix in -30..=30 {
            for iy in -30..=30 {
                let x = (ix as f64 + 0.5) * source;
                let y = (iy as f64 + 0.5) * source;
                points.push((
                    x as f32,
                    y as f32,
                    ((ix + iy - 20) as f64 + 0.5) as f32 * source as f32,
                ));
            }
        }
        let norm = 3.0_f64.sqrt();
        let expected = [-1.0 / norm, -1.0 / norm, 1.0 / norm];
        for half in [[0.3, 0.16], [0.6, 0.32]] {
            let footprint = [
                [-half[0], -half[1]],
                [half[0], -half[1]],
                half,
                [-half[0], half[1]],
            ];
            let terrain = Terrain::new(&points, false, &footprint, 0.05, [0.15, 0.15]);
            for heading in [-2.0, 0.0, 1.2] {
                let body = terrain.body_at([0.13, -0.21], -1.0, heading, 0.4).unwrap();
                assert!((body.0[2] - (-20.0 * source + 0.13 - 0.21 + 0.4)).abs() < 1e-6);
                let [x, y, z, w] = body.1;
                let actual = [
                    2.0 * (x * z + w * y),
                    2.0 * (y * z - w * x),
                    1.0 - 2.0 * (x * x + y * y),
                ];
                assert!(actual
                    .into_iter()
                    .zip(expected)
                    .all(|(a, b)| (a - b).abs() < 1e-6));
            }
        }
    }

    fn terrain_points(height: impl Fn(f64, f64) -> f64) -> Vec<(f32, f32, f32)> {
        let mut points = Vec::new();
        for x in -24..=24 {
            for y in -24..=24 {
                let x = (x as f64 + 0.5) * f64::from(0.05_f32);
                let y = (y as f64 + 0.5) * f64::from(0.05_f32);
                points.push((x as f32, y as f32, (height(x, y) - 0.025) as f32));
            }
        }
        points
    }

    #[test]
    fn environment_ground_fit_is_continuous_across_hash_grid_boundaries() {
        // 输入是连续起伏地形；散列表换行不能制造一个可见的机体姿态跳变。
        let points = terrain_points(|x, y| -2.0 + 0.03 * (x * 7.0).sin() + 0.04 * (y * 6.0).cos());
        let terrain = test_terrain(&points, false);
        for x in [-0.400001, -0.399999, 0.13] {
            let before = terrain.body_at([x, -0.400001], -2.0, 0.3, 0.4).unwrap();
            let after = terrain.body_at([x, -0.399999], -2.0, 0.3, 0.4).unwrap();
            let a = tilt(before.1);
            let b = tilt(after.1);
            assert!(
                (before.0[2] - after.0[2]).abs() < 1e-5,
                "两微米平移的高度跳变：{before:?} -> {after:?}"
            );
            assert!(
                (a[0] - b[0]).abs().max((a[1] - b[1]).abs()) < 1e-5,
                "两微米平移的倾斜跳变：{a:?} -> {b:?}"
            );
        }
    }

    #[test]
    fn environment_ground_fit_preserves_flat_ground_and_real_slopes() {
        let flat = test_terrain(&terrain_points(|_, _| -2.0), false);
        let a = flat.body_at([0.13, -0.4], -2.0, 0.7, 0.4).unwrap();
        assert_eq!(a, flat.body_at([0.13, -0.4], -2.0, 0.7, 0.4).unwrap());
        assert_eq!(a.1, quaternion(0.0, 0.0, 0.7));
        let slope = test_terrain(&terrain_points(|x, _| -2.0 + 0.1 * x), false);
        let a = slope.body_at([0.1, 0.1], -2.0, 0.0, 0.4).unwrap();
        let b = slope.body_at([0.2, 0.1], -2.0, 0.0, 0.4).unwrap();
        assert!((0.007..0.013).contains(&(b.0[2] - a.0[2])));
        assert!((-0.12..-0.05).contains(&tilt(a.1)[1]));
    }

    #[test]
    fn internal_observations_preserve_an_already_supported_top_reference() {
        let top = terrain_points(|_, _| -2.0);
        let mut thick = top.clone();
        for &point in &top {
            for layer in 1..=4 {
                thick.push((point.0, point.1, point.2 - layer as f32 * 0.05));
            }
        }
        let a = test_terrain(&top, false)
            .body_at([0.13, 0.1], -2.0, 0.4, 0.4)
            .unwrap();
        let b = test_terrain(&thick, false)
            .body_at([0.13, 0.1], -2.0, 0.4, 0.4)
            .unwrap();
        assert!(
            distance(a, b) < 1e-7,
            "实际参考已在带顶时，增加内部观测不能降低接触表面：{a:?} {b:?}"
        );
    }

    #[test]
    fn continuous_wall_interior_cannot_supply_a_ground_surface() {
        let bottom = terrain_points(|_, _| -2.0);
        let mut wall = bottom.clone();
        for &point in &bottom {
            for layer in 1..=40 {
                wall.push((point.0, point.1, point.2 + layer as f32 * 0.05));
            }
        }
        assert!(
            test_terrain(&wall, false)
                .body_at([0.1, 0.1], -2.0, 0.0, 0.4)
                .is_err(),
            "连续墙柱内部的占据点没有可踩踏的暴露顶"
        );
    }

    #[test]
    fn closest_thick_band_is_selected_before_testing_its_exposed_top() {
        let source = f64::from(0.05_f32);
        let mut points = Vec::new();
        for ix in -12..=12 {
            for iy in -12..=12 {
                let levels: Vec<i32> = if ix < 2 {
                    vec![-1]
                } else if ix == 2 {
                    vec![2]
                } else if ix == 3 {
                    vec![5]
                } else {
                    vec![-4, 0, 1, 2, 3, 4, 5, 6]
                };
                for iz in levels {
                    points.push((
                        (ix as f64 + 0.5) as f32 * source as f32,
                        (iy as f64 + 0.5) as f32 * source as f32,
                        (iz as f64 + 0.5) as f32 * source as f32,
                    ));
                }
            }
        }
        let terrain = test_terrain(&points, false);
        let samples = terrain.contact_samples([0.0, 0.0], 0.0).unwrap();
        let selected = samples
            .iter()
            .find(|p| (p.0 - 4.5 * source).abs() < 1e-12 && (p.1 - 0.5 * source).abs() < 1e-12)
            .unwrap();
        assert!(
            (selected.2 - source).abs() < 1e-12 && (selected.3 - 7.0 * source).abs() < 1e-12,
            "最近厚带须沿真实阶差连入，不能改选远处的孤立低薄层：{selected:?}"
        );
    }

    #[test]
    fn environment_contact_domain_preserves_directional_step_limits() {
        let source = f64::from(0.05_f32);
        let mut points = Vec::new();
        for ix in -16..=16 {
            for iy in -16..=16 {
                let iz = if ix < 0 { -1 } else { 2 };
                points.push((
                    (ix as f64 + 0.5) as f32 * source as f32,
                    (iy as f64 + 0.5) as f32 * source as f32,
                    (iz as f64 + 0.5) as f32 * source as f32,
                ));
            }
        }
        let footprint = [[-0.3, -0.16], [0.3, -0.16], [0.3, 0.16], [-0.3, 0.16]];
        let terrain = Terrain::new(&points, false, &footprint, 0.05, [0.2, 0.0]);
        let up = terrain.contact_samples([-0.1, 0.0], 0.0).unwrap();
        let down = terrain.contact_samples([0.1, 0.0], 3.0 * source).unwrap();
        assert!(
            up.iter().any(|p| (p.3 - 3.0 * source).abs() < 1e-12),
            "上阶能力应连到高台"
        );
        assert!(
            down.iter().all(|p| (p.3 - 3.0 * source).abs() < 1e-12),
            "没有下阶能力时不能借低层拟合高台姿态"
        );
    }

    #[test]
    fn single_observed_layer_does_not_create_contact_height_inside_its_voxel() {
        let terrain = test_terrain(&terrain_points(|_, _| 0.0), false);
        for reference in [-0.02, 0.02] {
            let body = terrain.body_at([0.1, 0.1], reference, 0.0, 0.4).unwrap();
            assert!(
                (body.0[2] - 0.4).abs() < 1e-8,
                "单层格顶必须保持真实踏面：{body:?}"
            );
        }
    }

    #[test]
    fn outer_isolated_band_cannot_replace_ground_under_the_robot() {
        let source = f64::from(0.05_f32);
        let mut points = Vec::new();
        for ix in -6..=5 {
            for iy in -4..=3 {
                points.push((
                    (ix as f64 + 0.5) as f32 * source as f32,
                    (iy as f64 + 0.5) as f32 * source as f32,
                    (-0.5 * source) as f32,
                ));
            }
        }
        // 足迹外隔着一个空列的厚带虽包含参考高度，也不能劫持足下平地的种子。
        for iz in [-1, 0] {
            points.push((
                (4.5 * source) as f32,
                (5.5 * source) as f32,
                ((iz as f64 + 0.5) * source) as f32,
            ));
        }
        let terrain = test_terrain(&points, false);
        let body = terrain
            .body_at([0.0, 0.0], 0.01, 0.0, 0.4)
            .expect("真实足下平地应可拟合");
        assert!((body.0[2] - 0.4).abs() < 1e-10);
        assert!(tilt(body.1).iter().all(|value|value.abs()<1e-10),
            "同层厚带即使进入估计域也不能取代脚下平面或制造倾斜");
    }

    #[test]
    fn world_ground_normal_is_independent_of_heading() {
        let terrain = test_terrain(&terrain_points(|x, y| -2.0 + 0.3 * x + 0.2 * y), false);
        let normal = |q: [f64; 4]| {
            let [x, y, z, w] = unit_orientation(q);
            [
                2.0 * (x * z + w * y),
                2.0 * (y * z - w * x),
                1.0 - 2.0 * (x * x + y * y),
            ]
        };
        let reference = normal(terrain.body_at([0.1, 0.1], -2.0, 0.0, 0.4).unwrap().1);
        for heading in [-2.7, -1.1, 0.7, 2.4] {
            let actual = normal(terrain.body_at([0.1, 0.1], -2.0, heading, 0.4).unwrap().1);
            assert!(
                actual
                    .into_iter()
                    .zip(reference)
                    .all(|(a, b)| (a - b).abs() < 1e-12),
                "同一地面在偏航{heading}下的世界法向发生变化：{reference:?} {actual:?}"
            );
        }
    }

    #[test]
    fn unchanged_ground_motion_preserves_actual_height_exactly() {
        let orientation = quaternion(0.017, -0.023, 0.31);
        let mut current = ([0.1, 0.2, -0.044312130807108406], orientation);
        let z = current.0[2];
        for _ in 0..100 {
            let planar = integrate(current, [0.7, 0.1, 0.0], DT);
            let before = (
                [current.0[0], current.0[1], 0.4],
                quaternion(0.0, 0.0, 0.31),
            );
            let after = ([planar.0[0], planar.0[1], 0.4], before.1);
            current = body_after_ground_change(current, planar, before, after, 0.4);
            assert_eq!(current.0[2], z, "相同接触高度的前后差必须精确为零");
        }
    }

    #[test]
    fn ground_adaptation_preserves_geometric_support_contact_on_slope_changes() {
        // 支撑点沿机体竖轴偏移d；机体贴合坡面时，中心高于同XY地面d/cos(pitch)。
        let d = 0.4;
        for slope in [-0.4_f64, 0.3] {
            let pitch = -slope.atan();
            let before = ([0.0, 0.0, d], quaternion(0.0, 0.0, 0.0));
            let after = ([0.2, 0.0, 0.1 + d], quaternion(0.0, pitch, 0.0));
            let current = ([0.0, 0.0, d + 0.037], before.1);
            let planar = ([0.2, 0.0, current.0[2]], current.1);
            let next = body_after_ground_change(current, planar, before, after, d);
            let expected = 0.1 + d / pitch.cos() + 0.037;
            assert!(
                (next.0[2] - expected).abs() < 1e-12,
                "坡面接触高度必须计入支撑点水平偏移，且保留实际偏差：{} != {expected}",
                next.0[2]
            );
            let returned =
                body_after_ground_change(next, ([0.0, 0.0, next.0[2]], next.1), after, before, d);
            assert!((returned.0[2] - current.0[2]).abs() < 1e-12);
        }
    }

    #[test]
    fn ground_adaptation_preserves_integrated_yaw_with_actual_tilt_bias() {
        let terrain = test_terrain(&terrain_points(|x, y| -2.0 + 0.15 * x + 0.1 * y), false);
        let current = ([0.1, 0.1, -1.6], quaternion(0.17, -0.09, 0.7));
        let planar = integrate(current, [0.0, 0.0, 0.4], 0.5);
        let before = terrain
            .body_at([0.1, 0.1], -2.0, yaw(current), 0.4)
            .unwrap();
        let after = terrain.body_at([0.1, 0.1], -2.0, yaw(planar), 0.4).unwrap();
        let next = body_after_ground_change(current, planar, before, after, 0.4);
        assert_eq!(next.0, current.0, "原地转向没有机体平移");
        assert!(
            (yaw(next) - yaw(planar)).abs() < 1e-12,
            "地形相对姿态不能改写已积分偏航：{} {}",
            yaw(next),
            yaw(planar)
        );
    }

    #[test]
    fn sparse_real_columns_keep_slope_response_continuous_without_filling_gaps() {
        let source=f64::from(0.05_f32);
        let mut points=Vec::new();
        for ix in (-16..=16).step_by(2) {for iy in (-16..=16).step_by(2) {
            let top=(f64::from(ix)*0.25).floor();
            points.push((((ix as f64+0.5)*source)as f32,((iy as f64+0.5)*source)as f32,
                ((top-0.5)*source)as f32));
        }}
        let terrain=test_terrain(&points,false);
        let original_columns=terrain.columns.len();
        for boundary in [0.5,1.5,3.5,5.5] {
            let x=boundary*source;
            let before=terrain.body_at([x-1e-7,0.025],0.0,0.0,0.4).unwrap();
            let after=terrain.body_at([x+1e-7,0.025],0.0,0.0,0.4).unwrap();
            assert!((before.0[2]-after.0[2]).abs()<1e-6,"跨采样空行不能突然换高度");
            assert!(tilt(before.1).into_iter().zip(tilt(after.1)).all(|(a,b)|(a-b).abs()<1e-5),
                "不可观测方向不能在域边界突然被压平");
        }
        assert_eq!(terrain.columns.len(),original_columns,"估计不能填造原始观测");
    }

    #[test]
    fn nearest_real_column_domain_stops_at_wall_and_rejects_isolated_support() {
        let mut points=Vec::new();
        for ix in -10..=10 {for iy in -10..=10 {
            if ix!=0 && (ix%2!=0 || iy%2!=0) {continue;}
            points.push(((ix as f32+0.5)*0.05,(iy as f32+0.5)*0.05,
                if ix==0 {0.975}else{-0.025}));
        }}
        let terrain=test_terrain(&points,false);
        let xy=[-0.075,0.025];
        let samples=terrain.contact_samples(xy,0.0).unwrap();
        assert!(samples.iter().all(|p|p.0+xy[0]<0.0),"第一真实墙列不能被穿过");
        let isolated=test_terrain(&[(0.025,0.025,-0.025)],false);
        assert!(isolated.body_at([0.025,0.025],0.0,0.0,0.4).is_err(),
            "只有孤立真实列时不能补造可观测平面");
    }

    #[test]
    fn exposed_floor_layers_remain_separate() {
        let lower = terrain_points(|_, _| -2.0);
        let mut points = lower.clone();
        points.extend(lower.iter().map(|&(x, y, z)| (x, y, z + 1.0)));
        let terrain = test_terrain(&points, false);
        let below = terrain.body_at([0.13, 0.1], -2.0, 0.7, 0.4).unwrap();
        let above = terrain.body_at([0.13, 0.1], -1.0, 0.7, 0.4).unwrap();
        assert!(
            (above.0[2] - below.0[2] - 1.0).abs() < 1e-6,
            "同列两个楼层必须保持各自的暴露顶"
        );
    }

    #[test]
    fn known_planar_surface_is_fitted_without_flatness_bias() {
        // 解析坡面在观测格中精确可表示，不用已经丢弃的亚体素高度作真值。
        let source = f64::from(0.05_f32);
        let mut points = Vec::new();
        for ix in -12..=12 {
            for iy in -12..=12 {
                let x = (ix as f64 + 0.5) * source;
                let y = (iy as f64 + 0.5) * source;
                points.push((
                    x as f32,
                    y as f32,
                    ((ix - iy - 41) as f64 + 0.5) as f32 * source as f32,
                ));
            }
        }
        let terrain = test_terrain(&points, false);
        let body = terrain.body_at([0.1, 0.1], -2.0, 0.7, 0.4).unwrap();
        let [x, y, z, w] = unit_orientation(body.1);
        let normal = [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ];
        let length = 3.0_f64.sqrt();
        let expected = [-1.0 / length, 1.0 / length, 1.0 / length];
        assert!(
            normal
                .into_iter()
                .zip(expected)
                .all(|(a, b)| (a - b).abs() < 1e-6),
            "无噪声真实坡面不能被先验压平：{normal:?} {expected:?}"
        );
        assert!((body.0[2] - (-40.0 * source + 0.4)).abs() < 1e-6);
    }
    #[test]
    fn tilted_contact_reference_does_not_select_a_separate_upper_layer() {
        let mut points = terrain_points(|_, _| 0.0);
        points.extend(terrain_points(|_, _| 0.1));
        let terrain = test_terrain(&points, false);
        let pitch = 30.0_f64.to_radians();
        let current = ([0.0, 0.0, 0.4 / pitch.cos()], quaternion(0.0, pitch, 0.0));
        let actual = terrain
            .body_at([0.0, 0.0], contact_reference(current, 0.4), 0.0, 0.4)
            .unwrap();
        assert!(
            (actual.0[2] - 0.4).abs() < 1e-7,
            "倾斜站姿实际关联下层地面：{actual:?}"
        );
        let wrong = terrain
            .body_at([0.0, 0.0], current.0[2] - 0.4, 0.0, 0.4)
            .unwrap();
        assert!(
            (wrong.0[2] - 0.5).abs() < 1e-7,
            "未换算参考会错误选择独立上层：{wrong:?}"
        );
    }

    #[test]
    fn interval_ground_tracks_repeated_real_thin_steps() {
        let terrain = test_terrain(
            &terrain_points(|x, _| {
                if x < 0.0 {
                    0.0
                } else if x < 0.3 {
                    0.1
                } else if x < 0.6 {
                    0.2
                } else {
                    0.3
                }
            }),
            false,
        );
        let mut actual = ([-0.7, 0.0, 0.4], quaternion(0.0, 0.0, 0.0));
        for i in 1..=165 {
            let x = -0.7 + i as f64 * 0.01;
            let before = terrain
                .body_at(
                    [actual.0[0], 0.0],
                    contact_reference(actual, 0.4),
                    yaw(actual),
                    0.4,
                )
                .unwrap();
            let after = terrain
                .body_at([x, 0.0], contact_reference(actual, 0.4), yaw(actual), 0.4)
                .unwrap();
            actual = body_after_ground_change(
                actual,
                ([x, 0.0, actual.0[2]], actual.1),
                before,
                after,
                0.4,
            );
        }
        assert!(
            (actual.0[2] - 0.7).abs() < 0.02,
            "连续三阶必须正常抬升：{actual:?}"
        );
    }
    #[test]
    fn uncertain_height_bands_preserve_a_common_feasible_flat_contact() {
        let mut points = terrain_points(|_, _| 0.0);
        for point in terrain_points(|_, _| 0.0) {
            for j in -2..=2 {
                if j < 1 || point.0 > 0.0 {
                    points.push((point.0, point.1, point.2 + 0.05 * j as f32));
                }
            }
        }
        let terrain = test_terrain(&points, false);
        for x in [-0.1, 0.0, 0.1] {
            let q = terrain.body_at([x, 0.0], 0.0, 0.0, 0.4).unwrap();
            assert!(
                (q.0[2] - 0.4).abs() < 1e-7 && tilt(q.1).iter().all(|v| v.abs() < 1e-7),
                "共同平面不能变成厚带顶的假斜坡：{q:?}"
            );
        }
    }

    #[test]
    fn repeated_step_risers_do_not_accumulate_height_or_attitude_drift() {
        let mut points = Vec::new();
        for x in -40..60 {
            for y in -20..20 {
                let top = 2 * (i32::from(x >= 0) + i32::from(x >= 8) + i32::from(x >= 16));
                let first = if [0, 8, 16].contains(&x) {
                    top - 2
                } else {
                    top - 1
                };
                for z in first..top {
                    points.push((
                        (x as f32 + 0.5) * 0.05,
                        (y as f32 + 0.5) * 0.05,
                        (z as f32 + 0.5) * 0.05,
                    ));
                }
            }
        }
        let terrain = test_terrain(&points, false);
        for bias in [0.0, 0.037] {
            let mut actual = ([-0.6, 0.025, 0.4 + bias], quaternion(0.0, 0.0, 0.0));
            for _ in 0..3 {
                for i in (1..=220).chain((0..220).rev()) {
                    let x = -0.6 + f64::from(i) * 0.01;
                    let before = terrain
                        .body_at(
                            [actual.0[0], 0.025],
                            contact_reference(actual, 0.4),
                            yaw(actual),
                            0.4,
                        )
                        .unwrap();
                    let after = terrain
                        .body_at([x, 0.025], contact_reference(actual, 0.4), yaw(actual), 0.4)
                        .unwrap();
                    actual = body_after_ground_change(
                        actual,
                        ([x, 0.025, actual.0[2]], actual.1),
                        before,
                        after,
                        0.4,
                    );
                    if i == 220 {
                        assert!(
                            (actual.0[2] - 0.7 - bias).abs() < 1e-7,
                            "真实三阶应抬升0.3米：{actual:?}"
                        );
                    }
                }
                assert!(
                    (contact_reference(actual, 0.4) - bias).abs() < 1e-7,
                    "返回同一平面不能累计高度偏差：{actual:?}"
                );
                assert!(
                    tilt(actual.1).iter().all(|v| v.abs() < 1e-7),
                    "返回同一平面不能累计倾斜：{actual:?}"
                );
            }
        }
    }
}
