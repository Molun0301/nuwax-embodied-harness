//! 三维机体凸柱包络、支撑接触与方向性地形约束。
use super::*;
use std::collections::VecDeque;
pub(super) type CacheSlot =
    std::sync::Mutex<AHashMap<(VoxelKey, u64), Arc<AHashMap<(i32, i32), VoxelKey>>>>;

/// 当前占据只证明精确足迹内的支撑，原始快照用于下一次位姿重新派生。
pub(super) struct CurrentBodySupport {
    pub(super) source: SurfaceSnapshot,
    pub(super) body: [f64; 3],
    pub(super) quaternion: [f64; 4],
    pub(super) polygon: Vec<[f64; 2]>,
    pub(super) cells: AHashSet<VoxelKey>,
}

impl CurrentBodySupport {
    fn permits_overlap(&self, polygon: &[[f64; 2]], x: i32, y: i32, size: f64) -> bool {
        // 将候选足迹裁到当前列，逐顶点核对其仍在原凸足迹中；相交格不能
        // 把列内未占据的半格也变成可通行支撑。
        let mut clipped = polygon.to_vec();
        for (axis, limit, sign) in [(0, f64::from(x)*size, 1.0),
            (0, f64::from(x+1)*size, -1.0), (1, f64::from(y)*size, 1.0),
            (1, f64::from(y+1)*size, -1.0)] {
            let mut next = Vec::with_capacity(clipped.len()+1);
            for (&a, &b) in clipped.iter().zip(clipped.iter().cycle().skip(1)) {
                let da = (a[axis]-limit)*sign;
                let db = (b[axis]-limit)*sign;
                if da >= 0.0 { next.push(a); }
                if (da >= 0.0) != (db >= 0.0) {
                    let t = da / (da-db);
                    next.push([a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])]);
                }
            }
            clipped = next;
        }
        clipped.into_iter().all(|p| inside_boundary(p, &self.polygon))
    }
}

/// 数学核心使用的机体和地形参数；外层从唯一标定契约转换。
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceBodyEnvelope {
    /// 机体坐标中的逆时针凸包。
    pub footprint_xy_m: Vec<[f64; 2]>,
    /// 机体最低/最高局部高度。
    pub z_bounds_m: [f64; 2],
    /// 机体到支撑参考点的正距离。
    pub body_to_support_z_m: f64,
    /// 最大上阶高度。
    pub max_step_up_m: f64,
    /// 最大下阶高度。
    pub max_step_down_m: f64,
    /// 最大横坡，角度。
    pub max_cross_slope_deg: f64,
    /// 支撑高度额外误差界。
    pub support_uncertainty_m: f64,
}
impl SurfaceBodyEnvelope {
    /// 检查静态几何和数学预算。
    pub fn validate(&self) -> Result<(), SurfaceError> {
        validate_boundary(&self.footprint_xy_m)?;
        if self.footprint_xy_m.len() > 256
            || !self
                .z_bounds_m
                .into_iter()
                .chain([
                    self.body_to_support_z_m,
                    self.max_step_up_m,
                    self.max_step_down_m,
                    self.max_cross_slope_deg,
                    self.support_uncertainty_m,
                ])
                .all(f64::is_finite)
            || self.z_bounds_m[0] >= self.z_bounds_m[1]
            || self.body_to_support_z_m <= 0.0
            || self.max_step_up_m < 0.0
            || self.max_step_down_m < 0.0
            || !(0.0..90.0).contains(&self.max_cross_slope_deg)
            || self.support_uncertainty_m < 0.0
        {
            return Err(SurfaceError::InvalidConfig(
                "invalid body envelope or terrain limits".into(),
            ));
        }
        for i in 0..self.footprint_xy_m.len() {
            let a = self.footprint_xy_m[i];
            let b = self.footprint_xy_m[(i + 1) % self.footprint_xy_m.len()];
            let c = self.footprint_xy_m[(i + 2) % self.footprint_xy_m.len()];
            if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) < -1e-10 {
                return Err(SurfaceError::InvalidConfig(
                    "body footprint must be convex and counterclockwise".into(),
                ));
            }
        }
        Ok(())
    }
    /// 水平外包半径，用于局部查询预算。
    pub fn radius(&self) -> f64 {
        self.footprint_xy_m
            .iter()
            .map(|p| p[0].hypot(p[1]))
            .fold(0.0, f64::max)
    }
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = dot(v, v).sqrt();
    v.map(|x| x / n)
}
fn rotation(q: [f64; 4]) -> Result<[[f64; 3]; 3], SurfaceError> {
    rotated_support_offset(q, 1.0)?;
    let scale = q.into_iter().map(f64::abs).fold(0.0, f64::max);
    let q = q.map(|x| x / scale);
    let norm = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    let [x, y, z, w] = q.map(|x| x / norm);
    Ok([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ],
        [
            2.0 * (x * y - w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + w * x),
        ],
        [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])
}
fn world(r: [[f64; 3]; 3], origin: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| origin[i] + r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2])
}
fn hull(mut points: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then_with(|| a[1].total_cmp(&b[1])));
    points.dedup();
    let turn = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    // 分别构建上下包络，避免退化竖直边穿过已有前缀。
    let mut lower = Vec::new();
    for &p in &points {
        while lower.len() >= 2 && turn(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper = Vec::new();
    for &p in points.iter().rev() {
        while upper.len() >= 2 && turn(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}
fn polygon_intersects_cell(polygon: &[[f64; 2]], x: i32, y: i32, size: f64) -> bool {
    let square = [
        [x as f64 * size, y as f64 * size],
        [(x + 1) as f64 * size, y as f64 * size],
        [(x + 1) as f64 * size, (y + 1) as f64 * size],
        [x as f64 * size, (y + 1) as f64 * size],
    ];
    let axes = polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .map(|(a, b)| [b[1] - a[1], a[0] - b[0]])
        .chain([[1.0, 0.0], [0.0, 1.0]]);
    for axis in axes {
        let project = |p: &[f64; 2]| p[0] * axis[0] + p[1] * axis[1];
        let (a, b) = polygon
            .iter()
            .map(project)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
                (a.min(v), b.max(v))
            });
        let (c, d) = square
            .iter()
            .map(project)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
                (a.min(v), b.max(v))
            });
        if b < c - 1e-10 || d < a - 1e-10 {
            return false;
        }
    }
    true
}
fn prism_intersects_voxel(
    model: &SurfaceBodyEnvelope,
    r: [[f64; 3]; 3],
    origin: [f64; 3],
    min: [f64; 3],
    max: [f64; 3],
) -> bool {
    let body: Vec<_> = model
        .footprint_xy_m
        .iter()
        .flat_map(|p| model.z_bounds_m.map(|z| [p[0], p[1], z]))
        .collect();
    let voxel: Vec<_> = [min[0], max[0]]
        .into_iter()
        .flat_map(|x| {
            [min[1], max[1]].into_iter().flat_map(move |y| {
                [min[2], max[2]].into_iter().map(move |z| {
                    let p = [x - origin[0], y - origin[1], z - origin[2]];
                    [dot(r[0], p), dot(r[1], p), dot(r[2], p)]
                })
            })
        })
        .collect();
    let voxel_axes: [[f64; 3]; 3] = std::array::from_fn(|i| [r[0][i], r[1][i], r[2][i]]);
    let mut axes = vec![[0.0, 0.0, 1.0]];
    axes.extend(voxel_axes);
    for (a, b) in model
        .footprint_xy_m
        .iter()
        .zip(model.footprint_xy_m.iter().cycle().skip(1))
    {
        let edge = [b[0] - a[0], b[1] - a[1], 0.0];
        axes.push([edge[1], -edge[0], 0.0]);
        for axis in voxel_axes {
            axes.push(cross(edge, axis));
        }
    }
    for axis in voxel_axes {
        axes.push(cross([0.0, 0.0, 1.0], axis));
    }
    for axis in axes {
        if dot(axis, axis) < 1e-18 {
            continue;
        }
        let (a, b) = body
            .iter()
            .map(|p| dot(axis, *p))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
                (a.min(v), b.max(v))
            });
        let (c, d) = voxel
            .iter()
            .map(|p| dot(axis, *p))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), v| {
                (a.min(v), b.max(v))
            });
        if b < c - 1e-10 || d < a - 1e-10 {
            return false;
        }
    }
    true
}

/// 重算选定单元的能力代价；从本次物理墙距重建基础值，允许显式换模型重新开放方向。
pub(super) fn apply_directions(graph:&mut PlannerGraph,heights:&HeightBounds,config:&Config,source:f32,model:&SurfaceBodyEnvelope,ids:&[CellId]) {
    let (profile,step)=validate_config(config,source).expect("已验证物理配置");
    for &id in ids {
        let from=graph.cells.coord(id);
        let from_z=heights.get(&from).map_or((from.2 as f32+1.0)*config.voxel_size,|b|b[1]);
        let p=penalty_of(graph.wall_state.dist[id as usize],config.wall_clearance_m,config.wall_buffer_m,config.wall_buffer_weight);
        let edges=graph.cells.neighbors(id).iter().map(|edge| {
            let to=graph.cells.coord(edge.dest);
            let to_z=heights.get(&to).map_or((to.2 as f32+1.0)*config.voxel_size,|b|b[1]);
            let rise=f64::from(to_z)-f64::from(from_z);
            let allowed=match classify_transition(&graph.surface_lookup,from,to,config,heights,source,profile,step) {
                Some(SurfaceTransition::StepUp)=>rise+2.0*model.support_uncertainty_m<=model.max_step_up_m+1e-6,
                Some(SurfaceTransition::StepDown)=>-rise+2.0*model.support_uncertainty_m<=model.max_step_down_m+1e-6,
                Some(_)=>true,None=>false,
            };
            let mut edge=*edge;
            let q=penalty_of(graph.wall_state.dist[edge.dest as usize],config.wall_clearance_m,config.wall_buffer_m,config.wall_buffer_weight);
            edge.cost=if allowed {edge.base_cost*(p+q)/2.0+config.step_penalty_weight*edge.rise} else {f32::INFINITY};
            edge
        }).collect();
        graph.cells.replace_edges(id,edges);
    }
}

impl SurfaceSnapshot {
    /// 为已通过来源及新鲜度检查的当前接地机体建立私有规划视图。
    /// 仅处理当前占据投影；周边地面须围住足迹，观测到的落差和禁区仍拒绝。
    /// 调用者必须从原环境快照重新派生，不能累计历次当前足迹。
    pub fn with_current_body_support(
        &self,
        body: [f64; 3],
        quaternion: [f64; 4],
    ) -> Result<Self, SurfaceError> {
        if let Some(current) = &self.current_body_support {
            return current.source.with_observations(self.observations.clone())
                .with_current_body_support(body, quaternion);
        }
        let Some(model) = self.body_envelope() else {
            return Ok(self.clone());
        };
        if self.validate_body_pose(body, quaternion).is_ok() {
            return Ok(self.clone());
        }
        let r = rotation(quaternion)?;
        if r[2][2] < f64::from(self.config().max_slope_deg).to_radians().cos() {
            return Err(SurfaceError::UntraversableSegment);
        }
        let support =
            self.support_position_from_body(body, quaternion, model.body_to_support_z_m)?;
        let size = f64::from(self.config().voxel_size);
        let quantum = f64::from(self.source_voxel_size_m());
        let vertices: Vec<_> = model
            .footprint_xy_m
            .iter()
            .flat_map(|p| {
                [
                    model.z_bounds_m[0],
                    model.z_bounds_m[1],
                    -model.body_to_support_z_m,
                ]
                .map(|z| world(r, body, [p[0], p[1], z]))
            })
            .collect();
        if vertices
            .iter()
            .flatten()
            .chain(support.iter())
            .any(|v| !v.is_finite() || (v / size).abs() >= f64::from(i32::MAX) * 0.5)
        {
            return Err(SurfaceError::InvalidPose);
        }
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let low: [i32; 2] = std::array::from_fn(|i| {
            (polygon.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min) / size).floor() as i32
        });
        let high: [i32; 2] = std::array::from_fn(|i| {
            (polygon
                .iter()
                .map(|p| p[i])
                .fold(f64::NEG_INFINITY, f64::max)
                / size)
                .floor() as i32
        });
        let halo = ((model.body_to_support_z_m + quantum) / size).ceil() as i32;
        if halo > MAX_PROFILE_CELLS {
            return Err(SurfaceError::InvalidConfig(
                "当前机体地面证据超出局部预算".into(),
            ));
        }
        let work = (i64::from(high[0]) - i64::from(low[0]) + 1 + 2 * i64::from(halo))
            * (i64::from(high[1]) - i64::from(low[1]) + 1 + 2 * i64::from(halo));
        if work > 16_384 || halo > MAX_PROFILE_CELLS {
            return Err(SurfaceError::InvalidConfig(
                "当前机体地面证据超出局部预算".into(),
            ));
        }
        let ground = |x: i32, y: i32| {
            let px = (f64::from(x) + 0.5) * size;
            let py = (f64::from(y) + 0.5) * size;
            support[2] - ((px - support[0]) * r[2][0] + (py - support[1]) * r[2][1]) / r[2][2]
        };
        let bounds = |x: i32, y: i32, z: i32| {
            self.data.height_bounds.get(&(x, y, z)).copied().unwrap_or([
                z as f32 * self.config().voxel_size,
                (z as f32 + 1.0) * self.config().voxel_size,
            ])
        };
        let covered = |x: i32, y: i32, b: [f32; 2]| {
            [x,x+1].into_iter().all(|ix| [y,y+1].into_iter().all(|iy| b.into_iter().all(|z| {
                let v=[f64::from(ix)*size-body[0],f64::from(iy)*size-body[1],f64::from(z)-body[2]];
                let p=[dot(r[0],v),dot(r[1],v),dot(r[2],v)];
                inside_boundary([p[0],p[1]],&model.footprint_xy_m)
                    && p[2]>=model.z_bounds_m[0].min(-model.body_to_support_z_m)-1e-6
                    && p[2]<=model.z_bounds_m[1]+1e-6
            })))
        };
        let mut evidence = Vec::new();
        let mut footprint = Vec::new();
        for x in low[0] - halo..=high[0] + halo {
            for y in low[1] - halo..=high[1] + halo {
                let z = ground(x, y);
                let column = self.data.occupied_columns.get(&(x, y));
                let supports = column.is_some_and(|zs| {
                    zs.iter().any(|&iz| {
                        let b = bounds(x, y, iz);
                        f64::from(b[0]) <= z + quantum && f64::from(b[1]) >= z - quantum
                    })
                });
                if supports {
                    evidence.push([(f64::from(x) + 0.5) * size, (f64::from(y) + 0.5) * size]);
                }
                if !polygon_intersects_cell(&polygon, x, y, size) {
                    continue;
                }
                let iz = ((z - quantum * 0.5) / size).floor() as i32;
                if self
                    .data
                    .virtual_obstacles
                    .as_ref()
                    .is_some_and(|mask| !mask.permits_cell((x, y, iz), size, 0.0))
                {
                    return Err(SurfaceError::VirtualObstacle);
                }
                // 看见脚下存在更低的真实地面而接地平面无证据时，不能以盲区名义补桥。
                if !supports
                    && column.is_some_and(|zs| {
                        zs.iter().any(|&iz| {
                            f64::from(bounds(x, y, iz)[1]) < z - model.max_step_down_m - quantum
                        })
                    })
                {
                    tracing::debug!(x, y, ground=z, "当前占据区存在已观测下方落差");
                    return Err(SurfaceError::MissingSupport);
                }
                // 支撑接触和完整机体检查使用同一相交格集合；自体占据清除
                // 仍由 covered 单独核对整体素，边缘格中的外部障碍不能清除。
                footprint.push((x, y, z));
            }
        }
        let evidence_hull = hull(evidence);
        if evidence_hull.len() < 3 || polygon.iter().any(|&p| !inside_boundary(p, &evidence_hull)) {
            tracing::debug!(?polygon, ?evidence_hull, "当前占据区缺少周边地面包围证据");
            return Err(SurfaceError::MissingSupport);
        }
        let mut updates = Vec::with_capacity(footprint.len());
        let mut inferred = AHashSet::new();
        for (x, y, z) in footprint {
            let top = z + (model.body_to_support_z_m + model.z_bounds_m[1]) / r[2][2];
            // 已观测到的合法足下层顶继续保留，避免清除自体时把厚地面挖成
            // 人造凹坑，进而在当前足迹边缘制造台阶。
            let maximum_ground = z
                + (model.body_to_support_z_m + model.z_bounds_m[0]) / r[2][2]
                + model.max_step_up_m.max(model.max_step_down_m)
                + quantum;
            let z = self
                .data
                .physical_graph
                .surface_lookup
                .get(&(x, y))
                .into_iter()
                .flatten()
                .filter(|&&iz| self.observed_support_lower_bound((x, y, iz)) <= z + quantum)
                .map(|&iz| self.cell_position((x, y, iz))[2])
                .filter(|&height| {
                    height >= z - quantum && height <= maximum_ground.min(z + model.max_step_up_m)
                })
                .max_by(f64::total_cmp)
                .unwrap_or(z);
            let mut cells = std::collections::BTreeMap::new();
            for &iz in self
                .data
                .occupied_columns
                .get(&(x, y))
                .into_iter()
                .flatten()
            {
                let b = bounds(x, y, iz);
                if f64::from(b[1]) < z - quantum || f64::from(b[0]) > top + quantum
                    || !covered(x,y,b) {
                    cells.insert(iz, b);
                }
            }
            let iz = ((z - quantum * 0.5) / size).floor() as i32;
            if !self.data.physical_graph.surface_lookup.get(&(x, y)).is_some_and(|zs| {
                zs.iter().any(|&old| (self.cell_position((x, y, old))[2]-z).abs() <= 1e-6)
            }) {
                inferred.insert((x, y, iz));
            }
            cells.insert(iz, [(z - quantum) as f32, z as f32]);
            updates.push(SurfaceColumnUpdate {
                column: [x, y],
                cells: cells.into_iter().collect(),
            });
        }
        let mut view=self.with_current_body_columns(&updates)?;
        view.current_body_support = Some(Arc::new(CurrentBodySupport {
            source: self.clone(), body, quaternion, polygon, cells: inferred,
        }));
        view.validate_body_pose(body,quaternion)?;
        Ok(view)
    }

    /// 沿真实相邻占据高度读取支撑层下界，不跨越未观测的竖直空隙。
    fn observed_support_lower_bound(&self, cell: VoxelKey) -> f64 {
        let size = f64::from(self.config().voxel_size);
        let bounds = |z| {
            self.data
                .height_bounds
                .get(&(cell.0, cell.1, z))
                .map_or([f64::from(z) * size, (f64::from(z) + 1.0) * size], |b| {
                    [f64::from(b[0]), f64::from(b[1])]
                })
        };
        let mut lower = bounds(cell.2)[0];
        if let Some(column) = self.data.occupied_columns.get(&(cell.0, cell.1)) {
            for &z in column.iter().rev().filter(|&&z| z < cell.2) {
                let b = bounds(z);
                if lower - b[1] > f64::from(self.source_voxel_size_m()) + 1e-6 {
                    break;
                }
                lower = b[0];
            }
        }
        lower
    }

    /// 装配同一不可变几何上的完整机体包络；上下阶方向直接进入实际邻接。
    pub fn with_body_envelope(&self, model: SurfaceBodyEnvelope) -> Result<Self, SurfaceError> {
        if let Some(current) = &self.current_body_support {
            return current.source.with_observations(self.observations.clone())
                .with_body_envelope(model)?.with_current_body_support(current.body, current.quaternion);
        }
        model.validate()?;
        let mut physical=(*self.data.physical_graph).clone();
        let ids=physical.cells.ids().collect::<Vec<_>>();
        apply_directions(&mut physical,&self.data.height_bounds,self.config(),self.source_voxel_size_m(),&model,&ids);
        let physical=Arc::new(physical);
        let graph=if self.data.virtual_obstacles.is_some() {
            let mut graph=(*self.data.graph).clone();
            let ids=graph.cells.ids().collect::<Vec<_>>();
            apply_directions(&mut graph,&self.data.height_bounds,self.config(),self.source_voxel_size_m(),&model,&ids);
            graph
        } else {(*physical).clone()};
        let regions=Arc::new(regions::RegionGraph::full(&graph,self.config().voxel_size));
        Ok(Self {
            observations: self.observations.clone(),
            current_body_support: None,
            data: Arc::new(SurfaceData {
                graph: Arc::new(graph),
                regions,
                physical_graph: physical,
                occupied_columns: Arc::clone(&self.data.occupied_columns),
                occupied_voxels: self.data.occupied_voxels,
                height_bounds: Arc::clone(&self.data.height_bounds),
                virtual_obstacles: self.data.virtual_obstacles.clone(),
                virtual_blocked_cells: Arc::clone(&self.data.virtual_blocked_cells),
                config: self.config().clone(),
                source_voxel_size_m: self.source_voxel_size_m(),
                profile_cells: self.data.profile_cells,
                candidate_step_cells: self.data.candidate_step_cells,
                frontier_cache: Default::default(),
                body_cache: Default::default(),
                body_envelope: Some(Arc::new(model)),
            }),
        })
    }
    /// 跨阶时机体连续抬升，法向参考点可以暂在两个真实踏面高度之间。
    /// 仅在完整投影外形下存在合法阶边、完整真实支撑且高度被两端夹住时选实际支撑。
    pub(super) fn locate_step_contact(&self,body:[f64;3],q:[f64;4],reference:[f64;3],distance:f64)->Result<SurfaceLocation,SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::MissingSupport)?;
        if (model.body_to_support_z_m-distance).abs()>1e-9 {return Err(SurfaceError::MissingSupport);}
        let size=f64::from(self.config().voxel_size);
        let quantum=f64::from(self.source_voxel_size_m());
        let maximum=model.max_step_up_m.max(model.max_step_down_m);
        let candidate=self.locate_near_support(reference,maximum+quantum)?;
        let r=rotation(q)?;
        if r[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        let polygon:Vec<_>=model.footprint_xy_m.iter().map(|p| {
            let v=world(r,body,[p[0],p[1],0.0]);[v[0],v[1]]
        }).collect();
        let patch=self.ground_patch(candidate,model.radius()+distance+size*2.0);
        let lower:[i32;2]=std::array::from_fn(|i|(polygon.iter().map(|p|p[i]).fold(f64::INFINITY,f64::min)/size).floor() as i32);
        let upper:[i32;2]=std::array::from_fn(|i|(polygon.iter().map(|p|p[i]).fold(f64::NEG_INFINITY,f64::max)/size).floor() as i32);
        if (i64::from(upper[0])-i64::from(lower[0])+1)*(i64::from(upper[1])-i64::from(lower[1])+1)>16_384 {return Err(SurfaceError::MissingSupport);}
        let mut contact=std::collections::BTreeSet::new();let mut min=f64::INFINITY;let mut max=f64::NEG_INFINITY;
        for x in lower[0]..=upper[0] {for y in lower[1]..=upper[1] {
            if !polygon_intersects_cell(&polygon,x,y,size) {continue;}
            let cell=*patch.get(&(x,y)).ok_or_else(|| {
                tracing::debug!(?body, x, y, centre=?candidate.cell, "机体足迹缺少连通支撑");
                SurfaceError::MissingSupport
            })?;
            // 中心已检查足迹余量；各接触格只核对真实支撑，不能再次膨胀整个足迹。
            if self.data.virtual_obstacles.as_ref().is_some_and(|mask| !mask.permits_cell(cell,size,0.0)) {
                return Err(SurfaceError::VirtualObstacle);
            }
            let height=self.cell_position(cell)[2];min=min.min(height);max=max.max(height);contact.insert(cell);
        }}
        let graph = &self.data.physical_graph;
        let contact_neighbors = |cell: VoxelKey| self.contact_neighbors(cell);
        let legal_step=contact.iter().any(|&cell|contact_neighbors(cell).into_iter().any(|next|
            contact.contains(&next)&&matches!(classify_transition(&graph.surface_lookup,cell,next,self.config(),
                &self.data.height_bounds,self.source_voxel_size_m(),self.data.profile_cells,self.data.candidate_step_cells),
                Some(SurfaceTransition::StepUp|SurfaceTransition::StepDown))));
        let mut connected=std::collections::BTreeSet::from([candidate.cell]);
        let mut queue=VecDeque::from([candidate.cell]);
        while let Some(cell)=queue.pop_front() {
            for next in contact_neighbors(cell) {
                if contact.contains(&next)&&connected.insert(next) {queue.push_back(next);}
            }
        }
        // 多次观测会把地面表示成连续占据层。实测接触落在该层内部时，不把层顶
        // 当成唯一真实地面；候选仍受原有腿部跨阶范围、完整足迹及连通性约束。
        let observed_contact = reference[2] >= self.observed_support_lower_bound(candidate.cell)-quantum
            && reference[2] <= candidate.surface_xyz[2]+quantum;
        if (!legal_step && !observed_contact) || !contact.is_subset(&connected)
            || (!observed_contact && reference[2]<min-quantum) || reference[2]>max+quantum {
                tracing::debug!(?body, ?reference, legal_step, observed_contact, min, max,
                    connected=contact.is_subset(&connected), "机体接触证据不足");
                return Err(SurfaceError::MissingSupport);
            }
        let actual=[reference[0],reference[1],candidate.surface_xyz[2]];
        self.locate_support(actual)
    }

    /// 是否装配机体模型。
    pub fn body_envelope(&self) -> Option<&SurfaceBodyEnvelope> {
        self.data.body_envelope.as_deref()
    }

    // 脚下相邻接触点无需容纳完整机体中心和踏面窗口。只连接实际观测的
    // 邻列及腿部可达高度；机体移动仍由原路径连边与包络扫掠验证。
    fn contact_neighbors(&self, cell: VoxelKey) -> Vec<VoxelKey> {
        let Some(model) = self.body_envelope() else {
            return vec![];
        };
        let graph = &self.data.physical_graph;
        let quantum = f64::from(self.source_voxel_size_m());
        let height = self.cell_position(cell)[2];
        DIRECTIONS
            .iter()
            .flat_map(|&(dx, dy)| {
                graph
                    .surface_lookup
                    .get(&(cell.0 + dx, cell.1 + dy))
                    .into_iter()
                    .flatten()
                    .map(move |&z| (cell.0 + dx, cell.1 + dy, z))
            })
            .filter(|&next| {
                let rise = self.cell_position(next)[2] - height;
                rise <= model.max_step_up_m + quantum + 1e-6
                    && -rise <= model.max_step_down_m + quantum + 1e-6
            })
            .collect()
    }

    fn ground_patch(
        &self,
        centre: SurfaceLocation,
        radius: f64,
    ) -> Arc<AHashMap<(i32, i32), VoxelKey>> {
        let key = (centre.cell, radius.to_bits());
        if let Some(patch) = self
            .data
            .body_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .get(&key)
        {
            return Arc::clone(patch);
        }
        let graph = &self.data.physical_graph;
        let Some(start) = graph.cells.id(centre.cell) else {
            return Arc::new(AHashMap::new());
        };
        let mut queue = VecDeque::from([start]);
        let mut seen = AHashSet::from_iter([start]);
        let mut patch = AHashMap::new();
        while let Some(id) = queue.pop_front() {
            if seen.len() > 16_384 {
                return Arc::new(AHashMap::new());
            }
            let cell = graph.cells.coord(id);
            let p = self.cell_position(cell);
            patch
                .entry((cell.0, cell.1))
                .and_modify(|old: &mut VoxelKey| {
                    if (self.cell_position(*old)[2] - centre.surface_xyz[2]).abs()
                        > (p[2] - centre.surface_xyz[2]).abs()
                    {
                        *old = cell;
                    }
                })
                .or_insert(cell);
            for next in self.contact_neighbors(cell) {
                let Some(next_id)=graph.cells.id(next) else {continue;};
                let point = self.cell_position(next);
                if (point[0] - centre.surface_xyz[0]).hypot(point[1] - centre.surface_xyz[1])
                        <= radius
                    && seen.insert(next_id)
                {
                    queue.push_back(next_id);
                }
            }
        }
        let patch = Arc::new(patch);
        let mut cache = self
            .data
            .body_cache
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if cache.len() >= 512 {
            cache.clear();
        }
        cache.insert(key, Arc::clone(&patch));
        patch
    }

    /// 用完整姿态检查机体凸柱与占据体素；允许已确认支撑面的接触。
    pub fn validate_body_pose(
        &self,
        body: [f64; 3],
        quaternion: [f64; 4],
    ) -> Result<(), SurfaceError> {
        let model = self
            .body_envelope()
            .ok_or_else(|| SurfaceError::InvalidConfig("body envelope is unavailable".into()))?;
        let centre = self.locate_support_from_body(body, quaternion, model.body_to_support_z_m)?;
        let r = rotation(quaternion)?;
        if r[2][2] <= 0.0 {
            return Err(SurfaceError::InvalidPose);
        }
        let vertices: Vec<_> = model
            .footprint_xy_m
            .iter()
            .flat_map(|p| model.z_bounds_m.map(|z| world(r, body, [p[0], p[1], z])))
            .collect();
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let (min, max) = vertices.iter().fold(
            ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
            |(mut min, mut max), p| {
                for i in 0..3 {
                    min[i] = min[i].min(p[i]);
                    max[i] = max[i].max(p[i]);
                }
                (min, max)
            },
        );
        let size = f64::from(self.config().voxel_size);
        let x0 = (min[0] / size).floor() as i32;
        let x1 = (max[0] / size).floor() as i32;
        let y0 = (min[1] / size).floor() as i32;
        let y1 = (max[1] / size).floor() as i32;
        if (i64::from(x1) - i64::from(x0) + 1) * (i64::from(y1) - i64::from(y0) + 1) > 16_384 {
            return Err(SurfaceError::InvalidConfig(
                "body footprint query exceeds budget".into(),
            ));
        }
        let radius = model.radius()
            + model.z_bounds_m[0].abs().max(model.z_bounds_m[1].abs())
            + model.body_to_support_z_m
            + size * 2.0;
        let patch = self.ground_patch(centre, radius);
        let uncertainty = f64::from(self.source_voxel_size_m()) + model.support_uncertainty_m;
        let mut ground_points = Vec::new();
        for x in x0..=x1 {
            for y in y0..=y1 {
                if !polygon_intersects_cell(&polygon, x, y, size) {
                    continue;
                }
                let ground = *patch.get(&(x, y)).ok_or_else(|| {
                    tracing::debug!(?body,x,y,centre=?centre.cell,"机体包络下缺少连通地面");
                    SurfaceError::MissingSupport
                })?;
                if self.current_body_support.as_ref().is_some_and(|current| {
                    current.cells.contains(&ground) && !current.permits_overlap(&polygon, x, y, size)
                }) {
                    tracing::debug!(?body, x, y, "候选机体进入当前足迹外的缺测支撑");
                    return Err(SurfaceError::MissingSupport);
                }
                if self
                    .data
                    .virtual_obstacles
                    .as_ref()
                    .is_some_and(|mask| !mask.permits_cell(ground, size, 0.0))
                {
                    return Err(SurfaceError::VirtualObstacle);
                }
                let ground_height = self.cell_position(ground)[2];
                let px = (x as f64 + 0.5) * size;
                let py = (y as f64 + 0.5) * size;
                let plane =
                    body[2] - ((px - body[0]) * r[2][0] + (py - body[1]) * r[2][1]) / r[2][2];
                let lower = plane + model.z_bounds_m[0] / r[2][2];
                let contact = model.max_step_up_m.max(model.max_step_down_m) + uncertainty;
                // 外形最低点包含足端行程，跨阶高度以标定支撑平面为基准；
                // 否则足端低于名义支撑的部分会再次扣减可跨阶高度。
                let support_plane = plane - model.body_to_support_z_m / r[2][2];
                if lower.max(support_plane) + contact + 1e-6 < ground_height || plane < ground_height - uncertainty {
                    return Err(SurfaceError::InsufficientClearance);
                }
                ground_points.push([px, py, ground_height]);
                if let Some(zs) = self.data.occupied_columns.get(&(x, y)) {
                    for &z in zs {
                        let bounds = self.data.height_bounds.get(&(x, y, z)).copied().unwrap_or([
                            z as f32 * self.config().voxel_size,
                            (z as f32 + 1.0) * self.config().voxel_size,
                        ]);
                        if f64::from(bounds[1]) <= ground_height + 1e-6
                            || f64::from(bounds[0]) > max[2]
                            || f64::from(bounds[1]) < min[2]
                        {
                            continue;
                        }
                        if prism_intersects_voxel(
                            model,
                            r,
                            body,
                            [x as f64 * size, y as f64 * size, f64::from(bounds[0])],
                            [
                                (x + 1) as f64 * size,
                                (y + 1) as f64 * size,
                                f64::from(bounds[1]),
                            ],
                        ) {
                            return Err(SurfaceError::InsufficientClearance);
                        }
                    }
                }
            }
        }
        if let Some((a, b, _)) = fit_plane(&ground_points, centre.support_xyz) {
            let yaw = r[0][1].atan2(r[0][0]);
            let cross_slope = (-a * yaw.sin() + b * yaw.cos()).abs().atan().to_degrees();
            if cross_slope > model.max_cross_slope_deg + 1e-5 {
                return Err(SurfaceError::UntraversableSegment);
            }
        }
        Ok(())
    }

    /// 给定机体水平位置与偏航，用已连接的实际支撑生成完整地形姿态。
    pub fn body_pose_on_surface(
        &self,
        xy: [f64; 2],
        yaw: f64,
        previous_support: [f64; 3],
    ) -> Result<([f64; 3], [f64; 4]), SurfaceError> {
        if !xy.into_iter().chain([yaw]).all(f64::is_finite) {
            return Err(SurfaceError::InvalidPose);
        }
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let projection=self.sample_segment_support(previous_support,[xy[0],xy[1],previous_support[2]],1.0)?.support_xyz;
        // 姿态在机体水平投影处估计一次，避免支撑偏移反复改变体素和拟合邻域。
        let q=self.body_orientation_at(projection,yaw)?;
        let offset=rotated_support_offset(q,model.body_to_support_z_m)?;
        let support=self.sample_segment_support(previous_support,[xy[0]+offset[0],xy[1]+offset[1],projection[2]],1.0)?.support_xyz;
        let p = self.body_position_from_support(support, q, model.body_to_support_z_m)?;
        self.validate_body_pose(p, q)?;
        Ok((p, q))
    }

    /// 逐段检查真实支撑及完整包络扫掠；姿态旋转不因零平移被跳过。
    pub fn validate_body_motion(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        boundary: &[[f64; 2]],
    ) -> Result<(), SurfaceError> {
        self.validate_body_motion_checked(from, to, boundary, |_| Ok(()))
    }

    /// 当前实际占据的精细支撑位置；只用于本次控制检查的足下豁免。
    pub fn body_observation_footprint(
        &self, pose: ([f64; 3], [f64; 4]),
    ) -> Result<std::collections::BTreeSet<[i32; 3]>, SurfaceError> {
        let observations = self.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        if observations.size_m.to_bits() != self.source_voxel_size_m().to_bits() {
            return Err(SurfaceError::InvalidConfig("观察精度与几何来源不一致".into()));
        }
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let centre = self.locate_support_from_body(pose.0, pose.1, model.body_to_support_z_m)?;
        let r = rotation(pose.1)?;
        let vertices: Vec<_> = model.footprint_xy_m.iter()
            .flat_map(|p| model.z_bounds_m.map(|z| world(r, pose.0, [p[0], p[1], z])))
            .collect();
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let size = f64::from(observations.size_m);
        let planner_size = f64::from(self.config().voxel_size);
        let min: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min) / size).floor() as i32);
        let max: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::NEG_INFINITY, f64::max) / size).floor() as i32);
        if (i64::from(max[0]) - i64::from(min[0]) + 1) * (i64::from(max[1]) - i64::from(min[1]) + 1) > 16_384 {
            return Err(SurfaceError::InvalidConfig("近场精细支撑查询超过预算".into()));
        }
        let radius = model.radius() + model.z_bounds_m[0].abs().max(model.z_bounds_m[1].abs())
            + model.body_to_support_z_m + planner_size * 2.0;
        let patch = self.ground_patch(centre, radius);
        let mut cells = std::collections::BTreeSet::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                if !polygon_intersects_cell(&polygon, x, y, size) { continue; }
                let column = (((x as f64 + 0.5) * size / planner_size).floor() as i32,
                    ((y as f64 + 0.5) * size / planner_size).floor() as i32);
                let ground = *patch.get(&column).ok_or(SurfaceError::MissingSupport)?;
                let height = self.cell_position(ground)[2];
                // 粗格跨坡面/阶边时，低侧真实细格可以属于同列较低的占据粗格。
                // 查询范围由一格内允许的坡差/阶差限制，并要求候选确在规范物理占据内。
                // 始终先选实际最高回波再查时效，旧顶部不能被新低面替代。
                let top = ((height / size).round() - 1.0) as i32;
                let height_span = (planner_size * std::f64::consts::SQRT_2
                    * f64::from(self.config().max_slope_deg).to_radians().tan())
                    .max(model.max_step_up_m).max(model.max_step_down_m)
                    + size + model.support_uncertainty_m;
                let count = (height_span / size).ceil().min(128.0) as i32;
                let bottom = top.saturating_sub(count);
                let z = (bottom..=top).rev().find(|&z| {
                    if observations.get([x,y,z]).is_none() { return false; }
                    let centre = (z as f64 + 0.5) * size;
                    let coarse_z = (centre / planner_size).floor() as i32;
                    self.data.occupied_columns.get(&column)
                        .is_some_and(|zs| zs.binary_search(&coarse_z).is_ok())
                }).unwrap_or(top);
                cells.insert([x, y, z]);
            }
        }
        Ok(cells)
    }

    /// 验证完整扫掠中新进入位置的实际观察；静态远端路线无需新鲜证据。
    pub fn validate_observed_body_motion(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        occupied: &std::collections::BTreeSet<[i32; 3]>,
        now_ns: u64,
        max_age_ns: u64,
    ) -> Result<(), SurfaceError> {
        let observations = self.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        if now_ns == 0 || max_age_ns == 0 { return Err(SurfaceError::MissingSupport); }
        self.validate_body_motion_checked(from, to, &[], |pose| {
            for cell in self.body_observation_footprint(pose)? {
                if occupied.contains(&cell) { continue; }
                let evidence = observations.get(cell).unwrap_or(SurfaceObservation {observed_ns:0,hits:0});
                let reason = if evidence.observed_ns == 0 {Some("缺少真实观察")}
                    else if evidence.hits < 2 {Some("独立采样命中不足")}
                    else if evidence.observed_ns > now_ns.saturating_add(50_000_000) {Some("采样时间在未来")}
                    else if now_ns.saturating_sub(evidence.observed_ns) > max_age_ns {Some("观察已过期")}
                    else {None};
                if let Some(reason) = reason {
                    return Err(SurfaceError::NearFieldObservation {cell,reason,observed_ns:evidence.observed_ns,checked_ns:now_ns});
                }
            }
            Ok(())
        })
    }

    fn validate_body_motion_checked(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        boundary: &[[f64; 2]],
        check_observations: impl Fn(([f64; 3], [f64; 4])) -> Result<(), SurfaceError>,
    ) -> Result<(), SurfaceError> {
        validate_boundary(boundary)?;
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        self.validate_body_pose(from.0, from.1)?;
        self.validate_body_pose(to.0, to.1)?;
        check_observations(from)?;
        check_observations(to)?;
        let a = self.locate_support_from_body(from.0, from.1, model.body_to_support_z_m)?.support_xyz;
        let b = self.locate_support_from_body(to.0, to.1, model.body_to_support_z_m)?.support_xyz;
        if !segment_inside_boundary([a[0], a[1]], [b[0], b[1]], boundary) {
            return Err(SurfaceError::InvalidBoundary);
        }
        self.validate_segment(a, b)?;
        let norm = |q: [f64; 4]| {
            let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
            q.map(|x| x / n)
        };
        let qa = norm(from.1);
        let mut qb = norm(to.1);
        let cosine = qa.iter().zip(qb).map(|(a, b)| a * b).sum::<f64>();
        if cosine < 0.0 {
            qb = qb.map(|x| -x);
        }
        let angle = 2.0 * cosine.abs().min(1.0).acos();
        let distance = (a[0] - b[0]).hypot(a[1] - b[1]).hypot(a[2] - b[2]);
        let count = ((distance + model.radius() * angle)
            / (f64::from(self.source_voxel_size_m()) * 0.5))
            .max(angle / 0.05)
            .ceil()
            .max(1.0) as usize;
        if count > 4096 {
            return Err(SurfaceError::UntraversableSegment);
        }
        for i in 1..count {
            let t = i as f64 / count as f64;
            let q = norm(std::array::from_fn(|j| qa[j] * (1.0 - t) + qb[j] * t));
            let support = self.sample_segment_support(a, b, t)?.support_xyz;
            let p = self.body_position_from_support(support, q, model.body_to_support_z_m)?;
            self.validate_body_pose(p, q)?;
            check_observations((p, q))?;
        }
        Ok(())
    }

    /// 从同一可达支撑邻域估计坡面姿态，位置仍由真实支撑提供。
    pub fn body_orientation_at(
        &self,
        support: [f64; 3],
        yaw: f64,
    ) -> Result<[f64; 4], SurfaceError> {
        let model = self
            .body_envelope()
            .ok_or_else(|| SurfaceError::InvalidConfig("body envelope is unavailable".into()))?;
        let centre = self.locate_support(support)?;
        let radius = model.radius() + f64::from(self.config().voxel_size);
        let patch = self.ground_patch(centre, radius);
        let points: Vec<_> = patch
            .values()
            .map(|&cell| {
                let mut point = self.cell_position(cell);
                // 重复观测形成的连续厚层不是台阶剖面。拟合姿态时选取层内
                // 距当前支撑最近的高度，保留薄踏面和有实际高差的坡面。
                let lower = (self.observed_support_lower_bound(cell)
                    + f64::from(self.source_voxel_size_m())).min(point[2]);
                point[2] = support[2].clamp(lower,point[2]);
                point
            })
            .collect();
        let (a, b, _) = fit_plane(&points, support).unwrap_or((0.0, 0.0, support[2]));
        let normal = normalize([-a, -b, 1.0]);
        let forward = normalize([yaw.cos(), yaw.sin(), a * yaw.cos() + b * yaw.sin()]);
        let left = cross(normal, forward);
        Ok(quaternion_from_columns([forward, left, normal]))
    }
}
fn fit_plane(points: &[[f64; 3]], origin: [f64; 3]) -> Option<(f64, f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let mut matrix = [[0.0; 4]; 3];
    for p in points {
        let v = [p[0] - origin[0], p[1] - origin[1], 1.0];
        for i in 0..3 {
            for j in 0..3 {
                matrix[i][j] += v[i] * v[j];
            }
            matrix[i][3] += v[i] * (p[2] - origin[2]);
        }
    }
    for col in 0..3 {
        let pivot =
            (col..3).max_by(|&a, &b| matrix[a][col].abs().total_cmp(&matrix[b][col].abs()))?;
        if matrix[pivot][col].abs() < 1e-10 {
            return None;
        }
        matrix.swap(col, pivot);
        let scale = matrix[col][col];
        for j in col..4 {
            matrix[col][j] /= scale;
        }
        for i in 0..3 {
            if i == col {
                continue;
            }
            let factor = matrix[i][col];
            for j in col..4 {
                matrix[i][j] -= factor * matrix[col][j];
            }
        }
    }
    Some((
        matrix[0][3],
        matrix[1][3],
        origin[2] + matrix[2][3] - matrix[0][3] * origin[0] - matrix[1][3] * origin[1],
    ))
}
fn quaternion_from_columns(r: [[f64; 3]; 3]) -> [f64; 4] {
    let trace = r[0][0] + r[1][1] + r[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            (r[1][2] - r[2][1]) / s,
            (r[2][0] - r[0][2]) / s,
            (r[0][1] - r[1][0]) / s,
            0.25 * s,
        ]
    } else {
        let i = (0..3).max_by(|&a, &b| r[a][a].total_cmp(&r[b][b])).unwrap();
        let j = (i + 1) % 3;
        let k = (j + 1) % 3;
        let s = (1.0 + r[i][i] - r[j][j] - r[k][k]).sqrt() * 2.0;
        let mut q = [0.0; 4];
        q[i] = 0.25 * s;
        q[j] = (r[j][i] + r[i][j]) / s;
        q[k] = (r[k][i] + r[i][k]) / s;
        q[3] = (r[j][k] - r[k][j]) / s;
        q
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn observed_ground_band_does_not_consume_leg_travel_twice_or_accept_floating_ground() {
        let base = scene(&[], false);
        let mut model = base.body_envelope().unwrap().clone();
        model.z_bounds_m = [-0.448, 0.057];
        model.max_step_up_m = 0.15;
        model.max_step_down_m = 0.15;
        let mut cfg = base.config().clone();
        cfg.step_threshold_m = 0.15;
        let make = |levels: &[f32]| {
            let points: Vec<_> = (-15..15)
                .flat_map(|x| {
                    (-15..15).flat_map(move |y| {
                        levels
                            .iter()
                            .map(move |&z| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z))
                    })
                })
                .collect();
            SurfaceSnapshot::from_points(&points, cfg.clone(), 0.05)
                .unwrap()
                .with_body_envelope(model.clone())
                .unwrap()
        };
        let p = [0.05, 0.05, 0.4];
        let q = [0.0, 0.0, 0.0, 1.0];
        make(&[-0.025, 0.025, 0.075, 0.125, 0.175])
            .validate_body_pose(p, q)
            .unwrap();
        assert!(
            make(&[-0.025, 0.025, 0.075, 0.125, 0.175, 0.225])
                .validate_body_pose(p, q)
                .is_err(),
            "超过真实腿部接触范围仍须拒绝"
        );
        assert!(
            make(&[0.175]).validate_body_pose(p, q).is_err(),
            "悬空参考点不能借地面厚层规则通过"
        );
    }

    #[test]
    fn body_attitude_fits_observed_ground_band_and_preserves_real_step_tops() {
        let base = scene(&[], false);
        let make = |thick: bool| {
            let mut points = vec![];
            for x in -15..15 {
                for y in -15..15 {
                    let z = if x >= 0 { 0.125 } else { 0.025 };
                    points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z));
                    if thick && x >= 0 {
                        for z in [0.025, 0.075] {
                            points.push(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z));
                        }
                    }
                }
            }
            SurfaceSnapshot::from_points(&points, base.config().clone(), 0.05)
                .unwrap()
                .with_body_envelope(base.body_envelope().unwrap().clone())
                .unwrap()
        };
        let support = [-0.05, 0.05, 0.05];
        let band = make(true).body_orientation_at(support, 0.0).unwrap();
        assert!(
            band[0].abs() < 1e-6 && band[1].abs() < 1e-6,
            "连续观测厚层不应制造倾斜: {band:?}"
        );
        let step = make(false).body_orientation_at(support, 0.0).unwrap();
        assert!(step[1].abs() > 0.01, "实际高踏面不得被压成平地");
    }
    fn observation_grid(edit: impl Fn(&mut std::collections::BTreeMap<[i32; 3], SurfaceObservation>)) -> Arc<SurfaceObservations> {
        let mut values = std::collections::BTreeMap::new();
        for x in -30..30 { for y in -30..30 {
            values.insert([x, y, 0], SurfaceObservation { observed_ns: 10_000_000_000, hits: 2 });
        }}
        edit(&mut values);
        let mut blocks: std::collections::BTreeMap<_, std::collections::BTreeMap<_, _>> = Default::default();
        for (key, value) in values { blocks.entry(key.map(|v| v.div_euclid(32))).or_default().insert(key, value); }
        let mut grid = SurfaceObservations::new(0.1, 32).unwrap();
        for (key, values) in blocks { grid.replace_block(key, values).unwrap(); }
        Arc::new(grid)
    }
    #[test]
    fn observed_body_motion_checks_fresh_top_support_and_independent_hits() {
        let base = scene(&[], false);
        let map = base.with_observations(Some(observation_grid(|_| {})));
        let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let to = ([0.45, 0.05, 0.5], from.1);
        let occupied = map.body_observation_footprint(from).unwrap();
        map.validate_observed_body_motion(from, to, &occupied, 10_100_000_000, 1_000_000_000).unwrap();
        assert!(base.same_geometry(&map));
        for kind in 0..5 {
            let bad = base.with_observations(Some(observation_grid(|cells| {
                let evidence = cells.get_mut(&[8, 0, 0]).unwrap();
                match kind {
                    0 => evidence.observed_ns = 1,
                    1 => evidence.hits = 1,
                    2 => evidence.observed_ns = 11_000_000_000,
                    3 => { cells.remove(&[8, 0, 0]); },
                    _ => {
                        evidence.observed_ns = 1;
                        cells.insert([8, 0, -1], SurfaceObservation { observed_ns: 10_000_000_000, hits: 3 });
                    },
                }
            })));
            assert!(bad.validate_observed_body_motion(from, to, &occupied, 10_100_000_000, 1_000_000_000).is_err(), "故障{kind}");
        }
        // 新查询不能为原不可变观察续期。
        assert!(map.validate_observed_body_motion(from, to, &occupied, 12_000_000_000, 1_000_000_000).is_err());
        // 当前实际占据区域仍需通过几何检查，但可以从旧足下证据开始停稳。
        map.validate_observed_body_motion(from, from, &occupied, 12_000_000_000, 1_000_000_000).unwrap();
    }
    #[test]
    fn observed_body_motion_checks_intermediate_sweep_and_new_rotation_space() {
        let map = scene(&[], false).with_observations(Some(observation_grid(|cells| {
            cells.get_mut(&[10, 0, 0]).unwrap().observed_ns = 1;
            cells.get_mut(&[0, 4, 0]).unwrap().observed_ns = 1;
        })));
        let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let to = ([1.85, 0.05, 0.5], from.1);
        let occupied = map.body_observation_footprint(from).unwrap();
        map.validate_body_motion(from, to, &[]).unwrap();
        assert!(map.validate_observed_body_motion(from, to, &occupied, 10_100_000_000, 1_000_000_000).is_err());
        let turn = (from.0, [0.0, 0.0, std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2]);
        assert!(map.validate_observed_body_motion(from, turn, &occupied, 10_100_000_000, 1_000_000_000).is_err());
    }
    #[test]
    fn observations_do_not_cross_vertical_layers() {
        let upper: Vec<_> = (-30..30).flat_map(|x| (-30..30).map(move |y| ((x as f32+0.5)*0.1, (y as f32+0.5)*0.1, 0.825))).collect();
        let map = scene(&upper, false).with_observations(Some(observation_grid(|_| {})));
        let from = ([0.05, 0.05, 1.3], [0.0, 0.0, 0.0, 1.0]);
        let to = ([0.45, 0.05, 1.3], from.1);
        let occupied = map.body_observation_footprint(from).unwrap();
        map.validate_body_motion(from, to, &[]).unwrap();
        assert!(map.validate_observed_body_motion(from, to, &occupied, 10_100_000_000, 1_000_000_000).is_err());
    }
    #[test]
    fn observed_slope_uses_each_fine_positions_actual_top() {
        let points: Vec<_> = (-50..50).flat_map(|x| (-30..30).map(move |y| {
            let px = (x as f32 + 0.5) * 0.05;
            (px, (y as f32 + 0.5) * 0.05, px * 0.13 + 0.021)
        })).collect();
        let mut blocks: std::collections::BTreeMap<_, std::collections::BTreeMap<_, _>> = Default::default();
        for &(x,y,z) in &points {
            let key = [x,y,z].map(|v| (v / 0.05).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation { observed_ns: 10_000_000_000, hits: 2 });
        }
        let mut grid = SurfaceObservations::new(0.05,32).unwrap();
        for (key,block) in blocks { grid.replace_block(key,block).unwrap(); }
        let map = SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.05).unwrap()
            .with_body_envelope(model()).unwrap().with_observations(Some(Arc::new(grid)));
        let support = map.locate_support([-0.55,0.05,-0.05]).unwrap().support_xyz;
        let q = map.body_orientation_at(support,0.0).unwrap();
        let from = (map.body_position_from_support(support,q,0.4).unwrap(),q);
        let to = map.body_pose_on_surface([0.55,0.05],0.0,support).unwrap();
        let occupied = map.body_observation_footprint(from).unwrap();
        map.validate_body_motion(from,to,&[]).unwrap();
        map.validate_observed_body_motion(from,to,&occupied,10_100_000_000,1_000_000_000).unwrap();
    }
    #[test]
    fn observed_ramp_crossing_coarse_height_boundary_uses_actual_fine_support() {
        let mut tops=std::collections::BTreeMap::new();
        for x in -40..80 { for y in -20..20 {
            let world_x=(x as f64+0.5)*0.05;
            let height=0.2*(world_x-0.5).clamp(0.0,1.8)-0.35;
            tops.insert([x,y],(height/0.05).floor() as i32);
        }}
        let points:Vec<_>=tops.iter().map(|(&[x,y],&z)|
            (((x as f64+0.5)*0.05) as f32,((y as f64+0.5)*0.05) as f32,((z as f64+0.5)*0.05) as f32)).collect();
        let mut blocks:std::collections::BTreeMap<_,std::collections::BTreeMap<_,_>>=Default::default();
        for (&[x,y],&z) in &tops {
            let key=[x,y,z];blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation{observed_ns:10_000_000_000,hits:2});
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,block) in blocks {observations.replace_block(key,block).unwrap();}
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]];
        let map=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.05).unwrap()
            .with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)));
        let mut support=map.locate_support([0.15,0.025,-0.3]).unwrap().support_xyz;
        for x in (4..50).chain((4..50).rev()) {
            let pose=map.body_pose_on_surface([x as f64*0.05,0.025],0.0,support).unwrap();
            support=map.support_position_from_body(pose.0,pose.1,0.4).unwrap();
            for cell in map.body_observation_footprint(pose).unwrap() {
                assert_eq!(cell[2],tops[&[cell[0],cell[1]]],"坡面位置{cell:?}");
            }
            map.validate_observed_body_motion(pose,pose,&Default::default(),10_100_000_000,1_000_000_000).unwrap();
        }
    }
    #[test]
    fn continuous_body_pose_can_straddle_real_step_but_not_float_or_bridge_missing_ground() {
        let points:Vec<_>=(-50..50).flat_map(|x|(-20..20).map(move|y|{
            ((x as f32+0.5)*0.025,(y as f32+0.5)*0.025,if x<0 {0.0} else {0.15})
        })).collect();
        let mut cfg=scene(&[],false).config().clone();cfg.step_threshold_m=0.25;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.max_step_up_m=0.25;envelope.max_step_down_m=0.25;envelope.support_uncertainty_m=0.025;
        let map=SurfaceSnapshot::from_points(&points,cfg.clone(),0.025).unwrap().with_body_envelope(envelope.clone()).unwrap();
        let mut poses=Vec::new();
        for i in -45..=45 {
            let x=i as f64*0.01;
            let dx:Vec<_>=(0..25).map(|n|-0.3+n as f64*0.025).collect();
            let heights:Vec<_>=dx.iter().map(|d|if x+d<0.0 {0.0} else {0.15}).collect();
            let mean=heights.iter().sum::<f64>()/heights.len() as f64;
            let slope=dx.iter().zip(&heights).map(|(x,y)|x*y).sum::<f64>()/dx.iter().map(|x|x*x).sum::<f64>();
            let pitch=-slope.atan();let pose=([x,0.0,mean+0.4/pitch.cos()],[0.0,(pitch/2.0).sin(),0.0,(pitch/2.0).cos()]);
            map.validate_body_pose(pose.0,pose.1).unwrap_or_else(|e|panic!("连续跨阶{x}: {e}, {pose:?}"));
            poses.push(pose);
        }
        for pair in poses.windows(2) {
            map.validate_body_motion(pair[0],pair[1],&[]).unwrap();
            map.validate_body_motion(pair[1],pair[0],&[]).unwrap();
        }
        let floating=([-0.8,0.0,0.55],[0.0,0.0,0.0,1.0]);
        assert!(map.validate_body_pose(floating.0,floating.1).is_err());
        let missing:Vec<_>=points.into_iter().filter(|p|!(p.0>0.0&&p.0<0.15)).collect();
        let hole=SurfaceSnapshot::from_points(&missing,cfg,0.025).unwrap().with_body_envelope(envelope).unwrap();
        let middle=poses[45];assert!(hole.validate_body_pose(middle.0,middle.1).is_err());
    }
    fn model() -> SurfaceBodyEnvelope {
        SurfaceBodyEnvelope {
            footprint_xy_m: vec![[-0.55, -0.14], [0.55, -0.14], [0.55, 0.14], [-0.55, 0.14]],
            z_bounds_m: [-0.1, 0.25],
            body_to_support_z_m: 0.4,
            max_step_up_m: 0.2,
            max_step_down_m: 0.1,
            max_cross_slope_deg: 10.0,
            support_uncertainty_m: 0.0,
        }
    }
    fn scene(extra: &[(f32, f32, f32)], hole: bool) -> SurfaceSnapshot {
        let mut points: Vec<_> = (-30..30)
            .flat_map(|x| {
                (-30..30).filter_map(move |y| {
                    if hole && x == 4 && y == 0 {
                        None
                    } else {
                        Some(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
                    }
                })
            })
            .collect();
        points.extend(extra);
        let cfg = Config {
            voxel_size: 0.1,
            robot_height: 0.25,
            surface_closing_radius: 0.0,
            wall_clearance_m: 0.01,
            step_threshold_m: 0.21,
            min_step_tread_depth_m: 0.2,
            world_frame: "map".into(),
            node_spacing_m: 0.3,
            wall_buffer_m: 0.0,
            wall_buffer_weight: 0.0,
            max_slope_deg: 25.0,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        SurfaceSnapshot::from_points(&points, cfg, 0.1)
            .unwrap()
            .with_body_envelope(model())
            .unwrap()
    }
    #[test]
    fn body_length_ceiling_and_unknown_support_are_checked() {
        let q = [0.0, 0.0, 0.0, 1.0];
        let p = [0.05, 0.05, 0.5];
        scene(&[], false).validate_body_pose(p, q).unwrap();
        assert!(scene(&[(0.55, 0.05, 0.55)], false)
            .validate_body_pose(p, q)
            .is_err());
        assert!(scene(&[(0.05, 0.05, 0.75)], false)
            .validate_body_pose(p, q)
            .is_err());
        assert!(scene(&[], true).validate_body_pose(p, q).is_err());
        let turn = [
            0.0,
            0.0,
            (std::f64::consts::FRAC_PI_4).sin(),
            (std::f64::consts::FRAC_PI_4).cos(),
        ];
        scene(&[(0.55, 0.05, 0.55)], false)
            .validate_body_pose(p, turn)
            .unwrap();
    }
    #[test]
    fn body_model_preserves_geometric_visibility_beyond_motion_clearance() {
        let map=scene(&[],false);let candidates=map.reachable_frontiers([0.05,0.05,0.1],&[]).unwrap();
        assert!(!candidates.is_empty());
        let valid=candidates.iter().filter(|c|{
            let yaw=c.support_xyz[1].atan2(c.support_xyz[0]);let q=map.body_orientation_at(c.support_xyz,yaw).unwrap();
            let p=map.body_position_from_support(c.support_xyz,q,0.4).unwrap();map.validate_body_pose(p,q).is_ok()
        }).count();
        assert!(valid>0,"完整机体不能抹去有真实支撑的观察停留点");
    }
    #[test]
    fn convex_prism_uses_full_roll_pitch_and_rotation_roundtrip() {
        let m = model();
        let a = 0.4_f64;
        let q = [a.sin(), 0.0, 0.0, a.cos()];
        let r = rotation(q).unwrap();
        let restored = rotation(quaternion_from_columns(r)).unwrap();
        for i in 0..3 {
            for j in 0..3 {
                assert!((r[i][j] - restored[i][j]).abs() < 1e-10);
            }
        }
        assert!(!prism_intersects_voxel(
            &m,
            rotation([0.0, 0.0, 0.0, 1.0]).unwrap(),
            [0.0; 3],
            [-0.1, 0.20, 0.0],
            [0.1, 0.24, 0.04]
        ));
        assert!(prism_intersects_voxel(
            &m,
            r,
            [0.0; 3],
            [-0.1, -0.20, 0.12],
            [0.1, -0.16, 0.16]
        ));
    }
    #[test]
    fn asymmetric_step_limits_filter_actual_directed_connections() {
        let mut points = Vec::new();
        for x in 0..60 {
            for y in 0..20 {
                points.push((
                    (x as f32 + 0.5) * 0.1,
                    (y as f32 + 0.5) * 0.1,
                    if x < 30 { 0.025 } else { 0.175 },
                ));
            }
        }
        let cfg = Config {
            voxel_size: 0.1,
            robot_height: 0.25,
            surface_closing_radius: 0.0,
            wall_clearance_m: 0.01,
            step_threshold_m: 0.21,
            min_step_tread_depth_m: 0.3,
            world_frame: "map".into(),
            node_spacing_m: 0.3,
            wall_buffer_m: 0.0,
            wall_buffer_weight: 0.0,
            max_slope_deg: 25.0,
            step_penalty_weight: 1.0,
            goal_tolerance: 0.1,
            viz_publish_hz: 0.0,
        };
        let s = SurfaceSnapshot::from_points(&points, cfg, 0.025)
            .unwrap()
            .with_body_envelope(model())
            .unwrap();
        s.plan([1.05, 1.05, 0.025], [4.05, 1.05, 0.175]).unwrap();
        assert!(s.plan([4.05, 1.05, 0.175], [1.05, 1.05, 0.025]).is_err());
    }
    #[test]
    fn current_body_view_clears_only_its_occupied_cells_and_keeps_environment_immutable() {
        let mut points: Vec<_> = (-30..30)
            .flat_map(|x| {
                (-30..30).filter_map(move |y| {
                    if (x == 4 || x == 14) && y == 0 {
                        None
                    } else {
                        Some(((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))
                    }
                })
            })
            .collect();
        points.extend([(0.05, 0.05, 0.4), (0.85, 0.05, 0.4)]);
        let base = SurfaceSnapshot::from_points(&points, scene(&[], false).config().clone(), 0.1)
            .unwrap()
            .with_body_envelope(model())
            .unwrap();
        let pose = ([0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]);
        assert!(base.validate_body_pose(pose.0, pose.1).is_err());
        let view = base.with_current_body_support(pose.0, pose.1).unwrap();
        view.validate_body_pose(pose.0, pose.1).unwrap();
        assert!(
            base.validate_body_pose(pose.0, pose.1).is_err(),
            "环境快照不能被清除"
        );
        assert_eq!(
            base.data.occupied_columns.get(&(8, 0)),
            view.data.occupied_columns.get(&(8, 0))
        );
        assert_eq!(
            base.data.occupied_columns.get(&(14, 0)),
            view.data.occupied_columns.get(&(14, 0))
        );
        assert!(
            view.validate_body_pose([1.0, 0.0, 0.5], pose.1).is_err(),
            "不能清除将进入区域的障碍"
        );
        assert!(
            view.locate_support([1.45, 0.05, 0.1]).is_err(),
            "足迹外未知孔洞仍不可走"
        );
        assert!(
            base.with_current_body_support([0.0, 0.0, 2.0], pose.1)
                .is_err(),
            "不能为悬空位姿虚构地面"
        );
    }

    #[test]
    fn current_body_view_preserves_obstacle_in_a_partially_overlapping_column() {
        let base=scene(&[(0.575,0.05,0.4)],false);
        let pose=([0.0,0.0,0.5],[0.0,0.0,0.0,1.0]);
        assert!(base.validate_body_pose(pose.0,pose.1).is_err());
        assert!(base.with_current_body_support(pose.0,pose.1).is_err(),
            "足迹右端为0.55米，不能清除同一体素中0.575米处的环境障碍");
    }

    #[test]
    fn current_body_blind_footprint_can_depart_into_observed_ground() {
        let mut config = scene(&[], false).config().clone();
        config.wall_clearance_m = 0.15;
        let mut envelope = model();
        envelope.footprint_xy_m = vec![[-0.24, -0.164], [0.34, -0.164],
            [0.34, 0.164], [-0.24, 0.164]];
        envelope.z_bounds_m = [-0.448, 0.057];
        for (yaw, pitch) in [(0.0_f64, 0.0_f64), (0.37, 0.0), (1.1, 0.0), (0.0, 0.0072)] {
            let pose = ([0.027, 0.013, 0.451], [-(yaw/2.0).sin()*(pitch/2.0).sin(),
                (yaw/2.0).cos()*(pitch/2.0).sin(), (yaw/2.0).sin()*(pitch/2.0).cos(),
                (yaw/2.0).cos()*(pitch/2.0).cos()]);
            let polygon: Vec<_> = envelope.footprint_xy_m.iter().map(|p| {
                [pose.0[0] + p[0] * yaw.cos() - p[1] * yaw.sin(),
                 pose.0[1] + p[0] * yaw.sin() + p[1] * yaw.cos()]
            }).collect();
            let points: Vec<_> = (-40..40).flat_map(|x| (-40..40).map(move |y|
                ((x as f32 + 0.5) * 0.05, (y as f32 + 0.5) * 0.05, 0.025)))
                .filter(|&(x,y,_)| !inside_boundary([f64::from(x),f64::from(y)], &polygon)).collect();
            let base = SurfaceSnapshot::from_points(&points, config.clone(), 0.05).unwrap()
                .with_body_envelope(envelope.clone()).unwrap();
            assert!(base.validate_body_pose(pose.0, pose.1).is_err());
            let view = base.with_current_body_support(pose.0, pose.1)
                .unwrap_or_else(|error| panic!("当前足迹盲区不能阻止起步，朝向{yaw}: {error}"));
            view.validate_body_pose(pose.0, pose.1).unwrap();
            let target = ([pose.0[0] + yaw.cos() * 0.8, pose.0[1] + yaw.sin() * 0.8, pose.0[2]], pose.1);
            view.validate_body_motion(pose, target, &[]).unwrap();
            assert!(base.validate_body_pose(pose.0, pose.1).is_err(), "原图仍保留真实盲区");
        }
    }

    #[test]
    fn current_body_blind_support_keeps_real_observations_and_does_not_accumulate() {
        let points: Vec<_> = (-30..30).flat_map(|x| (-30..30).filter_map(move |y| {
            (!(x >= -5 && x <= 4 && y == 0)).then_some(
                ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))
        })).collect();
        let observations = observation_grid(|cells| {
            cells.retain(|key,_| !(key[0] >= -5 && key[0] <= 4 && key[1] == 0));
        });
        let base = SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.1)
            .unwrap().with_body_envelope(model()).unwrap().with_observations(Some(observations.clone()));
        let start = ([0.0,0.0,0.5],[0.0,0.0,0.0,1.0]);
        let target = ([0.8,0.0,0.5],start.1);
        let view = base.with_current_body_support(start.0,start.1).unwrap();
        assert!(Arc::ptr_eq(view.observations.as_ref().unwrap(), &observations));
        let occupied = view.body_observation_footprint(start).unwrap();
        view.validate_observed_body_motion(start,target,&occupied,10_100_000_000,1_000_000_000).unwrap();
        assert!(view.validate_observed_body_motion(start,target,&occupied,12_000_000_000,1_000_000_000).is_err());
        let advanced = view.with_current_body_support(target.0,target.1).unwrap();
        assert!(advanced.validate_body_pose(start.0,start.1).is_err(), "新查询不能保留旧足迹的补足");
        assert!(base.validate_body_pose(start.0,start.1).is_err());
        assert!(observations.get([0,0,0]).is_none(), "不能伪造盲区的观察时间");
    }

    #[test]
    fn current_body_blind_edge_does_not_authorize_unknown_outside_its_polygon() {
        let points: Vec<_> = (-30..30).flat_map(|x| (-30..30).filter_map(move |y| {
            (x != 5 || y != 0).then_some(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))
        })).collect();
        let base=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.1)
            .unwrap().with_body_envelope(model()).unwrap();
        let pose=([0.0,0.0,0.5],[0.0,0.0,0.0,1.0]);
        let view=base.with_current_body_support(pose.0,pose.1).unwrap();
        view.validate_body_pose(pose.0,pose.1).unwrap();
        view.validate_body_motion(pose,([-0.3,0.0,0.5],pose.1),&[]).unwrap();
        assert!(view.validate_body_motion(pose,([0.025,0.0,0.5],pose.1),&[]).is_err(),
            "当前足迹止于0.55米，不能借相交格走进0.575米的未知区域");
    }

    #[test]
    fn current_body_view_rejects_observed_drop_and_virtual_boundary() {
        let mut points: Vec<_> = (-30..30)
            .flat_map(|x| {
                (-30..30).map(move |y| {
                    (
                        (x as f32 + 0.5) * 0.1,
                        (y as f32 + 0.5) * 0.1,
                        if x == 0 && y == 0 { -0.975 } else { 0.025 },
                    )
                })
            })
            .collect();
        let config = scene(&[], false).config().clone();
        let base = SurfaceSnapshot::from_points(&points, config.clone(), 0.1)
            .unwrap()
            .with_body_envelope(model())
            .unwrap();
        assert!(base
            .with_current_body_support([0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0])
            .is_err());
        points.push((0.05, 0.05, 0.025));
        let base = SurfaceSnapshot::from_points(&points, config, 0.1)
            .unwrap()
            .with_body_envelope(model())
            .unwrap();
        let mut blocked = vec![false; 60 * 60];
        blocked[30 * 60 + 30] = true;
        let forbidden = base
            .with_virtual_obstacles([-3.0, -3.0], 0.1, 60, 60, &blocked)
            .unwrap();
        assert_eq!(
            forbidden
                .with_current_body_support([0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0])
                .unwrap_err(),
            SurfaceError::VirtualObstacle
        );
    }

    #[test]
    fn office_corridor_uses_physical_width_and_still_rejects_unsafe_rotation() {
        let mut config=scene(&[],false).config().clone();
        config.robot_height=0.6;config.wall_clearance_m=0.15;
        let mut envelope=model();
        envelope.footprint_xy_m=vec![[-0.3,-0.164],[0.3,-0.164],[0.3,0.164],[-0.3,0.164]];
        envelope.z_bounds_m=[-0.3,0.06];
        let points:Vec<_>=(0..60).flat_map(|x|(0..4).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025))).collect();
        let map=SurfaceSnapshot::from_points(&points,config,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let q=[0.0,0.0,0.0,1.0];
        map.validate_body_pose([0.55,0.2,0.45],q).unwrap();
        let path=map.plan([0.55,0.2,0.05],[5.45,0.2,0.05]).unwrap();
        for pair in path.windows(2) {
            map.validate_body_motion(([pair[0][0],pair[0][1],0.45],q),([pair[1][0],pair[1][1],0.45],q),&[]).unwrap();
        }
        let turn=[0.0,0.0,std::f64::consts::FRAC_1_SQRT_2,std::f64::consts::FRAC_1_SQRT_2];
        assert!(map.validate_body_motion(([1.0,0.2,0.45],q),([1.0,0.2,0.45],turn),&[]).is_err());
    }

}
