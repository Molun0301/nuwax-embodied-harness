//! 三维机体凸柱包络、支撑接触与方向性地形约束。
use super::*;
use std::collections::VecDeque;
use super::body_witness::{CachedBodyResult, reject, height_rejection};
#[cfg(feature = "body-query-profile")]
use crate::body_query_profile::{self as query_profile, Counter as QueryCounter, Stage as QueryStage};
#[path = "body_translation.rs"]
mod translation;
#[path = "body_reference_field.rs"]
pub(super) mod reference_field;
#[path = "body_dynamic_interval.rs"]
mod dynamic_interval;
type GroundPatchCache = AHashMap<(VoxelKey, u64, u64), Arc<AHashMap<(i32, i32), VoxelKey>>>;
pub(super) type CacheSlot = std::sync::Mutex<GroundPatchCache>;

/// 只复用同一不可变视图的一个精确足迹；不保存观察许可或失败。
#[cfg(test)]
struct ObservationFootprintCache<'a> {
    snapshot: &'a SurfaceSnapshot,
    entry: Option<([u64; 7], Vec<[i32; 3]>)>,
    #[cfg(test)]
    builds: usize,
}

#[cfg(test)]
impl<'a> ObservationFootprintCache<'a> {
    fn new(snapshot: &'a SurfaceSnapshot) -> Self {
        Self { snapshot, entry: None, #[cfg(test)] builds: 0 }
    }

    fn cells(&mut self, pose: ([f64; 3], [f64; 4]))
        -> Result<&[[i32; 3]], SurfaceError> {
        let key = [pose.0[0], pose.0[1], pose.0[2],
            pose.1[0], pose.1[1], pose.1[2], pose.1[3]].map(f64::to_bits);
        if self.entry.as_ref().is_none_or(|(cached, _)| *cached != key) {
            #[cfg(feature = "body-query-profile")] query_profile::count(QueryCounter::ObservationFootprintCacheMisses);
            #[cfg(test)] { self.builds += 1; }
            let cells = self.snapshot.build_body_observation_footprint::<Vec<_>>(pose)?;
            self.entry = Some((key, cells));
        } else {
            #[cfg(feature = "body-query-profile")] query_profile::count(QueryCounter::ObservationFootprintCacheHits);
        }
        Ok(self.entry.as_ref().expect("successful footprint entry").1.as_slice())
    }
}

// Both inputs are XYZ ordered. Bound gap traversal: after one ordinary step,
// seek rather than scanning arbitrarily many occupied keys between two cells.
#[cfg(test)]
fn unoccupied_observation_cells<'a>(cells: &'a [[i32; 3]],
    occupied: &'a std::collections::BTreeSet<[i32; 3]>) -> impl Iterator<Item=&'a [i32; 3]> {
    let first = cells.first().copied().unwrap_or([i32::MAX; 3]);
    let mut range = occupied.range(first..);
    let mut next = range.next();
    cells.iter().filter(move |&&cell| {
        if next.is_some_and(|key| *key < cell) {
            next = range.next();
            if next.is_some_and(|key| *key < cell) {
                range = occupied.range(cell..);
                next = range.next();
            }
        }
        if next == Some(&cell) { next = range.next(); false } else { true }
    })
}

#[cfg(all(test,feature = "body-query-profile"))]
#[derive(Default)]
struct ObservationHeightCounts {columns:u64,probes:u64,absent:u64}
#[cfg(all(test,feature = "body-query-profile"))]
impl Drop for ObservationHeightCounts {
    fn drop(&mut self) {
        query_profile::add(QueryCounter::ObservationHeightColumns,self.columns);
        query_profile::add(QueryCounter::ObservationHeightProbes,self.probes);
        query_profile::add(QueryCounter::ObservationHeightAbsent,self.absent);
    }
}

// Three assigned-column work counts, accumulated once per actual pose check.
// No timestamps, geometry queries or per-cell TLS access are added.
#[cfg(feature = "body-query-profile")]
#[derive(Default)]
struct AssignedZCounts { dense_z_span:u64, actual_z_candidates:u64, fine_get_absent:u64 }
#[cfg(feature = "body-query-profile")]
impl Drop for AssignedZCounts {
    fn drop(&mut self) {
        query_profile::add(QueryCounter::AssignedDenseZSpan,self.dense_z_span);
        query_profile::add(QueryCounter::AssignedActualZCandidates,self.actual_z_candidates);
        query_profile::add(QueryCounter::AssignedFineGetAbsent,self.fine_get_absent);
    }
}

// Actual coarse-column branch choices; publish once per uncached pose, including errors.
// Kept separate from the existing assigned-Z counts and absent from default builds.
#[cfg(feature = "body-query-profile")]
#[derive(Default)]
struct PoseColumnCounts { assigned:u64, unassigned:u64 }
#[cfg(feature = "body-query-profile")]
impl Drop for PoseColumnCounts {
    fn drop(&mut self) {
        query_profile::add(QueryCounter::PoseAssignedColumns,self.assigned);
        query_profile::add(QueryCounter::PoseUnassignedColumns,self.unassigned);
    }
}

/// 数学核心使用的机体和地形参数；外层从唯一标定契约转换。
/// 检查完整凸柱碰撞与已知地形约束；未知地面不构成第二次准入条件。
/// 不模拟离散足端接触或足步执行。
/// 凸柱包络不能用来推断实时腿部姿态、点的自体身份或地面承载材质。
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceBodyEnvelope {
    /// 机体坐标中的逆时针凸包。
    pub footprint_xy_m: Vec<[f64; 2]>,
    /// 机体最低/最高局部高度。
    pub z_bounds_m: [f64; 2],
    /// 机体到支撑参考点的正标定距离；不是实时腿部状态，也不声明固定机身高度能力。
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
    // 已是单位四元数时归一化为恒等，保留输入以免制造末位姿态变化。
    let [x,y,z,w]=if q.iter().map(|x|x*x).sum::<f64>()==1.0 {q} else {
        let scale = q.into_iter().map(f64::abs).fold(0.0, f64::max);
        let q = q.map(|x| x / scale);
        let norm = q.iter().map(|x| x * x).sum::<f64>().sqrt();
        q.map(|x| x / norm)
    };
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
fn interpolated_body_pose(from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),t:f64)->([f64;3],[f64;4]) {
    if t==0.0 {return from;}
    if t==1.0 {return to;}
    let norm=|q:[f64;4]| {let n=q.iter().map(|v|v*v).sum::<f64>().sqrt();q.map(|v|v/n)};
    let a=norm(from.1);let mut b=norm(to.1);
    if a.iter().zip(b).map(|(a,b)|a*b).sum::<f64>()<0.0 {b=b.map(|v|-v);}
    (std::array::from_fn(|i|if from.0[i]==to.0[i] {from.0[i]}else{from.0[i]*(1.0-t)+to.0[i]*t}),
        norm(std::array::from_fn(|i|a[i]*(1.0-t)+b[i]*t)))
}
fn world(r: [[f64; 3]; 3], origin: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| origin[i] + r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2])
}
#[cfg(test)]
fn projected_voxel_radius(r: [[f64; 3]; 3], size: f64) -> f64 {
    let mut radius = 0.0_f64;
    for x in [-0.5, 0.5] { for y in [-0.5, 0.5] { for z in [-0.5, 0.5] {
        let corner = [x*size,y*size,z*size];
        radius = radius.max(dot(r[0],corner).hypot(dot(r[1],corner)));
    }}}
    radius
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
/// 每个位姿只投影一次凸包；同一足迹下的粗细格共用精确分离轴区间。
struct PolygonQuery<'a>(Vec<([f64;2],f64,f64)>, &'a [[f64;2]]);
impl<'a> PolygonQuery<'a> {
    fn new(polygon:&'a [[f64;2]])->Self {
        Self(polygon.iter().zip(polygon.iter().cycle().skip(1))
            .map(|(a,b)|[b[1]-a[1],a[0]-b[0]]).chain([[1.0,0.0],[0.0,1.0]])
            .map(|axis| {
                let (low,high)=polygon.iter().map(|p|p[0]*axis[0]+p[1]*axis[1])
                    .fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),v|(a.min(v),b.max(v)));
                (axis,low,high)
            }).collect(),polygon)
    }
    /// 支撑平面在实际足迹与占据列交叠区域内的最小值。
    /// 整格下界已证明安全时直接复用；边缘仅检查凸多边形交集的顶点。
    fn contact_plane(&self,body:[f64;3],normal:[f64;3],x:i32,y:i32,size:f64,
        ground_ceiling:f64,allowance:f64)->f64 {
        let slope=[-normal[0]/normal[2],-normal[1]/normal[2]];
        if slope==[0.0,0.0] {return body[2];}
        let low=[f64::from(x)*size,f64::from(y)*size];
        let high=[f64::from(x+1)*size,f64::from(y+1)*size];
        let value=|p:[f64;2]|body[2]+(p[0]-body[0])*slope[0]+(p[1]-body[1])*slope[1];
        let worst=std::array::from_fn(|i|if slope[i]>=0.0 {low[i]}else{high[i]});
        let lower=value(worst);
        if ground_ceiling<=lower+allowance {return lower;}
        let inside=|p:[f64;2]|self.0.iter().all(|&(axis,lo,hi)| {
            let v=axis[0]*p[0]+axis[1]*p[1];v>=lo-1e-10 && v<=hi+1e-10
        });
        if inside(worst) {return lower;}
        let in_cell=|p:[f64;2]|(0..2).all(|i|p[i]>=low[i]-1e-10 && p[i]<=high[i]+1e-10);
        let mut minimum=f64::INFINITY;
        for &p in self.1 {if in_cell(p) {minimum=minimum.min(value(p));}}
        for i in 0..4 {
            let p=std::array::from_fn(|axis|if i&(1<<axis)==0 {low[axis]}else{high[axis]});
            if inside(p) {minimum=minimum.min(value(p));}
        }
        for (a,b) in self.1.iter().zip(self.1.iter().cycle().skip(1)) {
            for axis in 0..2 {
                let delta=b[axis]-a[axis];if delta==0.0 {continue;}
                for bound in [low[axis],high[axis]] {
                    let t=(bound-a[axis])/delta;
                    if !(0.0..=1.0).contains(&t) {continue;}
                    let mut p=std::array::from_fn(|i|a[i]*(1.0-t)+b[i]*t);p[axis]=bound;
                    if in_cell(p) {minimum=minimum.min(value(p));}
                }
            }
        }
        if minimum.is_finite() {minimum} else {lower}
    }
    fn intersects(&self,x:i32,y:i32,size:f64)->bool {
        let centre=[(f64::from(x)+0.5)*size,(f64::from(y)+0.5)*size];
        self.0.iter().all(|&(axis,low,high)| {
            let c=centre[0]*axis[0]+centre[1]*axis[1];
            let radius=(axis[0].abs()+axis[1].abs())*size*0.5;
            high>=c-radius-1e-10 && c+radius>=low-1e-10
        })
    }
}
#[cfg(test)]
fn polygon_intersects_cell(polygon:&[[f64;2]],x:i32,y:i32,size:f64)->bool {
    PolygonQuery::new(polygon).intersects(x,y,size)
}

/// 凸柱与轴对齐体素的精确分离轴检查。机体区间预计算，体素用中心和半宽投影。
struct PrismQuery {origin:[f64;3], axes:Vec<([f64;3],f64,f64)>}
impl PrismQuery {
    fn new(model:&SurfaceBodyEnvelope,r:[[f64;3];3],origin:[f64;3])->Self {
        let body:Vec<_>=model.footprint_xy_m.iter()
            .flat_map(|p|model.z_bounds_m.map(|z|[p[0],p[1],z])).collect();
        let voxel_axes:[[f64;3];3]=std::array::from_fn(|i|[r[0][i],r[1][i],r[2][i]]);
        let mut axes=vec![[0.0,0.0,1.0]];axes.extend(voxel_axes);
        for (a,b) in model.footprint_xy_m.iter().zip(model.footprint_xy_m.iter().cycle().skip(1)) {
            let edge=[b[0]-a[0],b[1]-a[1],0.0];axes.push([edge[1],-edge[0],0.0]);
            for axis in voxel_axes {axes.push(cross(edge,axis));}
        }
        for axis in voxel_axes {axes.push(cross([0.0,0.0,1.0],axis));}
        let axes=axes.into_iter().filter(|a|dot(*a,*a)>=1e-18).map(|axis| {
            let (low,high)=body.iter().map(|p|dot(axis,*p))
                .fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),v|(a.min(v),b.max(v)));
            let world=std::array::from_fn(|i|axis[0]*r[0][i]+axis[1]*r[1][i]+axis[2]*r[2][i]);
            (world,low,high)
        }).collect();
        Self {origin,axes}
    }
    fn intersects(&self,min:[f64;3],max:[f64;3])->bool {
        let centre=std::array::from_fn(|i|(min[i]+max[i])*0.5-self.origin[i]);
        let half:[f64;3]=std::array::from_fn(|i|(max[i]-min[i])*0.5);
        self.axes.iter().all(|&(axis,low,high)| {
            let c=dot(axis,centre);let radius=dot(axis.map(f64::abs),half);
            high>=c-radius-1e-10 && c+radius>=low-1e-10
        })
    }
}
#[cfg(test)]
fn prism_intersects_voxel(model:&SurfaceBodyEnvelope,r:[[f64;3];3],origin:[f64;3],min:[f64;3],max:[f64;3])->bool {
    PrismQuery::new(model,r,origin).intersects(min,max)
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

/// Whether a queried terrain height is measured, legacy geometry, locally estimated or unknown.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum GroundQueryState { Observed, LegacyGeometry, Estimated, Unknown }

/// Relationship of the current nominal contact height to known terrain.
/// Unknown carries no permission or collision claim; bounded projection can still proceed.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum BodyGroundRelation { KnownCompatible, KnownIncompatible, Unknown }

/// Terrain interpretation, never a collision-free or execution permission.
#[derive(Clone,Debug)]
pub struct GroundQuery {
    /// Unknown has no fabricated height or normal.
    pub state: GroundQueryState,
    pub height: Option<f64>,
    pub normal: Option<[f64;3]>,
    /// Immutable source-layer anchors used by this interpretation.
    pub sources: Vec<[i32;3]>,
    /// Bounded interpolation reach; zero for a direct source observation.
    pub radius_m: f64,
    /// Geometric residual/empirical scatter, not calibrated statistical confidence.
    pub max_residual_m: Option<f64>,
}
impl GroundQuery {
    fn unknown()->Self {Self {state:GroundQueryState::Unknown,height:None,normal:None,
        sources:Vec::new(),radius_m:0.0,max_residual_m:None}}
}
#[derive(Clone)]
struct GroundEstimate {
    origin:[f64;3],normal:[f64;3],sources:Vec<[i32;3]>,hull:Vec<[f64;2]>,radius:f64,residual:f64,
}
impl GroundEstimate {
    fn at(&self,xy:[f64;2])->GroundQuery {
        if !inside_boundary(xy,&self.hull) {return GroundQuery::unknown();}
        let height=self.origin[2]-(self.normal[0]*(xy[0]-self.origin[0])
            +self.normal[1]*(xy[1]-self.origin[1]))/self.normal[2];
        GroundQuery {state:GroundQueryState::Estimated,height:Some(height),normal:Some(self.normal),
            sources:self.sources.clone(),radius_m:self.radius,max_residual_m:Some(self.residual)}
    }
}

// Immutable source membership and its unchanged fitting interval share one cache entry.
// Pose-dependent contact classifications are never stored here.
#[derive(Clone, Copy, Debug, PartialEq)]
struct FineContactBand {
    fit: [f64; 2],
    run: [i32; 2],
}

// Query-local contact membership. A single run needs no heap allocation.
#[derive(Default)]
enum FineContacts {
    #[default]
    Empty,
    One([i32;2]),
    Many(Vec<[i32;2]>),
}
impl FineContacts {
    fn as_slice(&self)->&[[i32;2]] {
        match self {Self::Empty=>&[],Self::One(run)=>std::slice::from_ref(run),Self::Many(runs)=>runs}
    }
    fn last_mut(&mut self)->Option<&mut [i32;2]> {
        match self {Self::Empty=>None,Self::One(run)=>Some(run),Self::Many(runs)=>runs.last_mut()}
    }
    fn push(&mut self,run:[i32;2]) {
        match self {
            Self::Empty=>*self=Self::One(run),
            Self::One(first)=>*self=Self::Many(vec![*first,run]),
            Self::Many(runs)=>runs.push(run),
        }
    }
}

// Query-local selection borrows the immutable measured record; it never copies
// members or stores pose-dependent selection in the shared compatibility cache.
enum FineGroundInput<'a> {
    Measured(Option<&'a ObservedGroundLayer>),
    Legacy(Option<Arc<Vec<FineContactBand>>>),
}

struct FineGroundColumn {
    xy: [i32; 2],
    // Missing source remains None; never a zero-height permission.
    height: Option<f64>,
    // Exact original occupied voxels eligible for contact; source bands stay intact.
    contacts: FineContacts,
}
// Output type fixes whether contact membership is computed; height-only rows
// cannot be passed to collision consumers expecting FineGroundColumn.
struct FineGroundHeight(Option<f64>);
trait FineGroundValue {
    const CONTACTS: bool;
    fn from_parts(xy:[i32;2],height:Option<f64>,contacts:FineContacts)->Self;
}
impl FineGroundValue for FineGroundColumn {
    const CONTACTS: bool=true;
    fn from_parts(xy:[i32;2],height:Option<f64>,contacts:FineContacts)->Self {
        Self{xy,height,contacts}
    }
}
impl FineGroundValue for FineGroundHeight {
    const CONTACTS: bool=false;
    fn from_parts(_xy:[i32;2],height:Option<f64>,_contacts:FineContacts)->Self {
        Self(height)
    }
}
// Recent surface points need the selected height at an exact fine XY, but do
// not consume static contact membership. Keep aggregate height rows compact.
struct FineGroundHeightRow {
    xy: [i32; 2],
    height: Option<f64>,
}
impl FineGroundValue for FineGroundHeightRow {
    const CONTACTS: bool=false;
    fn from_parts(xy:[i32;2],height:Option<f64>,_contacts:FineContacts)->Self {
        Self{xy,height}
    }
}
impl FineGroundColumn {
    fn permits_contact(&self,z:i32)->bool {
        self.contacts.as_slice().iter().any(|range|range[0]<=z && z<=range[1])
    }
}

impl SurfaceSnapshot {
    /// Compatibility entry point for a current-body geometry query.
    /// It shares the immutable source: no inferred floor, convex-hull admission,
    /// or pose-dependent removal of occupied voxels is performed.
    pub fn with_current_body_support(
        &self, body: [f64; 3], quaternion: [f64; 4],
    ) -> Result<Self, SurfaceError> {
        self.validated_body_snapshot(body, quaternion, &mut None)
    }

    pub(super) fn validated_body_snapshot(
        &self, body: [f64; 3], quaternion: [f64; 4],
        slot: &mut Option<BodyFailureWitness>,
    ) -> Result<Self, SurfaceError> {
        *slot = None;
        self.validate_complete_source_resolution()?;
        if self.body_envelope().is_some() {
            self.validate_body_pose_with_witness(body, quaternion, slot)?;
        } else {
            rotation(quaternion)?;
            if !body.into_iter().all(f64::is_finite) { return Err(SurfaceError::InvalidPose); }
        }
        Ok(self.clone())
    }

    /// 沿真实相邻占据高度读取支撑层下界，不跨越未观测的竖直空隙。
    fn observed_support_lower_bound(&self, cell: VoxelKey) -> f64 {
        if let Some(lower)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner()).lower_bounds.get(&cell).copied() {
            return lower;
        }
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
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.lower_bounds.len()>=8192 {cache.lower_bounds.clear();}
        cache.lower_bounds.insert(cell,lower);
        lower
    }

    /// 装配同一不可变几何上的完整机体包络；上下阶方向直接进入实际邻接。
    pub fn with_body_envelope(&self, model: SurfaceBodyEnvelope) -> Result<Self, SurfaceError> {

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
            ground_evidence:self.ground_evidence,recent_obstacles:self.recent_obstacles.clone(),
            observations: self.observations.clone(),
            body_query_cache: Default::default(),
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
                routing_cache: Default::default(),
                body_envelope: Some(Arc::new(model)),
            }),
        })
    }
    /// 本次既有足迹遍历的细列关联；不增加覆盖或连通许可条件。
    fn fine_ground_columns(&self,ground:Option<VoxelKey>,polygon:&PolygonQuery,prism:&PrismQuery,
        body:[f64;3],normal:[f64;3],x:i32,y:i32)->Option<Vec<FineGroundColumn>> {
        self.fine_ground_values::<FineGroundColumn>(ground,polygon,prism,body,normal,x,y)
    }

    fn fine_ground_height(&self,ground:VoxelKey,polygon:&PolygonQuery,prism:&PrismQuery,
        body:[f64;3],normal:[f64;3],x:i32,y:i32)->Option<f64> {
        self.fine_ground_values::<FineGroundHeight>(Some(ground),polygon,prism,body,normal,x,y)
            .map(|rows|rows.into_iter().filter_map(|row|row.0).max_by(f64::total_cmp)
                .unwrap_or_else(||self.ground_cell_position(ground)[2]))
    }

    fn fine_ground_values<T:FineGroundValue>(&self,ground:Option<VoxelKey>,polygon:&PolygonQuery,prism:&PrismQuery,
        body:[f64;3],normal:[f64;3],x:i32,y:i32)->Option<Vec<T>> {
        let observations=self.observations.as_ref()?;
        let model=self.body_envelope()?;
        // Without a source terrain anchor carry the calibrated support height
        // in the gravity frame. Body pitch/roll belongs to the physical prism,
        // and cannot relabel a horizontal ground band as an obstacle.
        let (body,normal)=if ground.is_none() {
            ([body[0],body[1],body[2]+model.body_to_support_z_m*(1.0-normal[2])],[0.0,0.0,1.0])
        } else {(body,normal)};
        let fine=f64::from(observations.size_m);let size=f64::from(self.config().voxel_size);
        let mut columns=Vec::new();let mut has_legacy=false;
        for ix in (f64::from(x)*size/fine).floor() as i32..(f64::from(x+1)*size/fine).ceil() as i32 {
            for iy in (f64::from(y)*size/fine).floor() as i32..(f64::from(y+1)*size/fine).ceil() as i32 {
                if !polygon.intersects(ix,iy,fine) {continue;}
                let xy=[ix,iy];
                let input=match self.select_measured_fine_ground_layer(observations,model,ground,body,normal,xy) {
                    Some(record)=>FineGroundInput::Measured(record),
                    None=>{has_legacy=true;FineGroundInput::Legacy(None)},
                };
                columns.push((xy,input));
            }
        }
        // Only compatibility columns use the existing one-batch cache path.
        // Measured rejection is final too: it must not fall back to a voxel band.
        if has_legacy {
            {
                let cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
                for (xy,input) in &mut columns {if let FineGroundInput::Legacy(bands)=input {
                    *bands=cache.fine_bands.get(&(xy[0],xy[1])).cloned();
                }}
            }
            let mut missing=Vec::new();
            for (xy,input) in &mut columns {if let FineGroundInput::Legacy(bands)=input {
                if bands.is_none() {
                    let values=Arc::new(self.read_fine_contact_bands(xy[0],xy[1]));
                    missing.push(((xy[0],xy[1]),Arc::clone(&values)));*bands=Some(values);
                }
            }}
            if !missing.is_empty() {
                let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
                if cache.fine_bands.len()>=8192 {cache.fine_bands.clear();}
                cache.fine_bands.extend(missing);
            }
        }
        Some(columns.into_iter().map(|(xy,input)|match input {
            FineGroundInput::Measured(record)=>Self::measured_fine_ground_value::<T>(xy,fine,record),
            FineGroundInput::Legacy(bands)=>{
                let bands=bands.expect("批读已填充");
                self.select_legacy_fine_ground_value::<T>(observations,model,polygon,prism,
                    body,normal,xy,Some(bands.as_slice()))
            },
        }).collect())
    }

    /// None requests compatibility; Some(None) is a final measured-layer miss.
    /// Inputs already use the same anchor/gravity frame as the legacy selector.
    fn select_measured_fine_ground_layer<'a>(&self,observations:&'a SurfaceObservations,
        model:&SurfaceBodyEnvelope,ground:Option<VoxelKey>,body:[f64;3],normal:[f64;3],xy:[i32;2])
        ->Option<Option<&'a ObservedGroundLayer>> {
        if self.ground_evidence!=GroundEvidence::MeasuredSurface {return None;}
        let fine=f64::from(observations.size_m);let size=f64::from(self.config().voxel_size);
        let center=xy.map(|v|(f64::from(v)+0.5)*fine);
        let mut records=observations.ground_layers(xy).peekable();
        let any_measured=records.peek().is_some();
        // The physical anchor constrains membership to its coarse cell.
        // Within that cell retain the layer nearest the calibrated support
        // plane, as in support lookup; never merge the other layer's members.
        let reference=body[2]-(normal[0]*(center[0]-body[0])
            +normal[1]*(center[1]-body[1])+model.body_to_support_z_m)/normal[2];
        let uncertainty=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        let record=records.filter(|record| {
            if let Some(ground)=ground {
                record.members.iter().any(|&z|(((f64::from(z)+0.5)*fine)/size).floor() as i32==ground.2)
            } else {
                record.height_at(center).is_some_and(|height|
                    height-reference<=model.max_step_up_m+uncertainty+1e-6
                    && reference-height<=model.max_step_down_m+uncertainty+1e-6
                    && record.normal[2]>=f64::from(self.config().max_slope_deg).to_radians().cos())
            }
        }).min_by(|a,b|(a.height_at(center).unwrap_or(f64::INFINITY)-reference).abs()
            .total_cmp(&(b.height_at(center).unwrap_or(f64::INFINITY)-reference).abs()));
        any_measured.then_some(record)
    }

    fn measured_fine_ground_value<T:FineGroundValue>(xy:[i32;2],fine:f64,record:Option<&ObservedGroundLayer>)->T {
        if let Some(record)=record {
            let center=xy.map(|v|(f64::from(v)+0.5)*fine);
            let height=record.height_at(center);
            let mut contacts=FineContacts::default();
            if T::CONTACTS {for &z in &record.members {
                if let Some(last)=contacts.last_mut().filter(|last|last[1].checked_add(1)==Some(z)) {last[1]=z;}
                else {contacts.push([z,z]);}
            }}
            return T::from_parts(xy,height,contacts);
        }
        T::from_parts(xy,None,FineContacts::default())
    }

    /// One source-owned fine column, shared by batched body checks and live guide hints.
    /// Measured layers return before any compatibility-band cache access.
    fn select_fine_ground_value<T:FineGroundValue>(&self,observations:&SurfaceObservations,
        model:&SurfaceBodyEnvelope,ground:Option<VoxelKey>,polygon:&PolygonQuery,prism:&PrismQuery,
        body:[f64;3],normal:[f64;3],xy:[i32;2],bands:Option<&[FineContactBand]>)->T {
        // Without a source terrain anchor carry the calibrated support height
        // in the gravity frame. Body pitch/roll belongs to the physical prism,
        // and cannot relabel a horizontal ground band as an obstacle.
        let (body,normal)=if ground.is_none() {
            ([body[0],body[1],body[2]+model.body_to_support_z_m*(1.0-normal[2])],[0.0,0.0,1.0])
        } else {(body,normal)};
        if let Some(record)=self.select_measured_fine_ground_layer(observations,model,ground,body,normal,xy) {
            return Self::measured_fine_ground_value::<T>(xy,f64::from(observations.size_m),record);
        }
        self.select_legacy_fine_ground_value::<T>(observations,model,polygon,prism,body,normal,xy,bands)
    }

    /// Compatibility math is unchanged; body/normal are already anchor-adjusted.
    fn select_legacy_fine_ground_value<T:FineGroundValue>(&self,observations:&SurfaceObservations,
        model:&SurfaceBodyEnvelope,polygon:&PolygonQuery,prism:&PrismQuery,
        body:[f64;3],normal:[f64;3],xy:[i32;2],bands:Option<&[FineContactBand]>)->T {
        let fine=f64::from(observations.size_m);
        let contact=model.max_step_up_m.max(model.max_step_down_m)+f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        let support_offset=model.body_to_support_z_m/normal[2];
        let body_lower_offset=model.z_bounds_m[0]/normal[2];
        let uncertainty=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        let allowance=(-support_offset+contact+1e-6).min(uncertainty);
        // contact_plane has only two query-local return values. The original
        // height-dependent fast branch still chooses between them for each Z.
        let flat=[-normal[0]/normal[2],-normal[1]/normal[2]]==[0.0,0.0];
        let (rectangular_plane,clipped_plane,upper_plane)=if flat {
            (body[2],body[2],body[2])
        } else {
            (polygon.contact_plane(body,normal,xy[0],xy[1],fine,f64::NEG_INFINITY,0.0),
             polygon.contact_plane(body,normal,xy[0],xy[1],fine,f64::INFINITY,0.0),
             -polygon.contact_plane([body[0],body[1],-body[2]],[-normal[0],-normal[1],normal[2]],
                xy[0],xy[1],fine,f64::INFINITY,0.0))
        };
        let lower=clipped_plane-support_offset;
        let upper=upper_plane-support_offset;
        let nominal_upper=lower+contact+1e-6;
        let contact_plane=|top:f64|if top<=rectangular_plane+allowance {rectangular_plane}else{clipped_plane};
        let reference=(lower+upper)*0.5;
        let distance=|b:[f64;2]|(b[0]-reference).max(reference-b[1]).max(0.0);
        // Batched body checks pass their existing prefetched bands. A single
        // guide column loads only its own legacy band, after the measured fast path.
        let cached;
        let bands=match bands {
            Some(bands)=>bands,
            None=>{cached=self.fine_contact_bands(xy[0],xy[1]);cached.as_slice()},
        };
        let band=bands.iter().min_by(|a,b|distance(a.fit).total_cmp(&distance(b.fit))
            .then_with(||(a.fit[1]-reference).abs().total_cmp(&(b.fit[1]-reference).abs()))
            .then_with(||a.fit[1].total_cmp(&b.fit[1])));
        // Associate the original fit interval with the existing actual-overlap
        // nominal window. Intersection is a predicate, never a truncated band.
        let band=band.filter(|candidate|
            candidate.fit[0].max(upper-contact-1e-6)
                <= candidate.fit[1].min(nominal_upper));
        let height=band.and_then(|band| {
            // OccupiedVolume still selects only the collapsed fit top here.
            let first=(band.fit[0]/fine-1.0).round() as i32;
            let last=band.run[1];
            (first..=last).rev().find_map(|iz| {
                let key=[xy[0],xy[1],iz];
                let top=f64::from(iz+1)*fine;
                (top<=nominal_upper || prism.intersects(key.map(|v|f64::from(v)*fine),
                    key.map(|v|f64::from(v+1)*fine))).then_some(top)
            })
        });
        let mut contacts=FineContacts::default();
        if T::CONTACTS {
            for candidate in bands.iter() {
                let [first,last]=candidate.run;
                if Some(candidate)==band {
                    // Check each original selected-band voxel with the existing
                    // nominal upper bound and height predicate. Higher intersecting
                    // voxels remain in `height` and in the original prism checks.
                    for iz in first..=last {
                        let top=f64::from(iz+1)*fine;
                        if top>nominal_upper {continue;}
                        let plane=contact_plane(top);
                        let support=plane-support_offset;
                        let body_lower=plane+body_lower_offset;
                        if height_rejection(body_lower,support,plane,contact,top,uncertainty).is_none() {
                            if let Some(run)=contacts.last_mut().filter(|run|run[1].checked_add(1)==Some(iz)) {
                                run[1]=iz;
                            } else {contacts.push([iz,iz]);}
                        }
                    }
                    continue;
                }
                // An extra band is not a new support anchor. Its complete exposed
                // top must fit the existing nominal-support travel at this overlap;
                // rigid-body bottom height must not enlarge that travel.
                let plane=contact_plane(candidate.fit[1]);
                let support=plane-support_offset;
                let body_lower=plane+body_lower_offset;
                if candidate.fit[1]<=support+contact+1e-6
                    && height_rejection(body_lower,support,plane,contact,candidate.fit[1],uncertainty).is_none() {
                    contacts.push([first,last]);
                }
            }
        }
        T::from_parts(xy,height,contacts)
    }

    /// 接触关联与包络复用实际相交精细列的地面高度；缺精细依据时仍保留粗层。
    fn footprint_ground_height(
        &self,
        ground: VoxelKey,
        polygon_query: &PolygonQuery,
        prism_query: &PrismQuery,
        support_plane: f64,
        contact: f64,
        selected: Option<&[FineGroundColumn]>,
    ) -> Result<f64, SurfaceError> {
        if let Some(columns)=selected {
            return Ok(columns.iter().filter_map(|c|c.height).max_by(f64::total_cmp)
                .unwrap_or_else(||self.ground_cell_position(ground)[2]));
        }
        let (x, y, _) = ground;
        let size = f64::from(self.config().voxel_size);
        let mut ground_height = self.ground_cell_position(ground)[2];
        if let Some(observations) = self.observations.as_ref() {
            let fine = f64::from(observations.size_m);
            let lower = self.observed_support_lower_bound(ground);

            let mut height = f64::NEG_INFINITY;
            for ix in (f64::from(x)*size/fine).floor() as i32..(f64::from(x+1)*size/fine).ceil() as i32 {
                for iy in (f64::from(y)*size/fine).floor() as i32..(f64::from(y+1)*size/fine).ceil() as i32 {
                    if !polygon_query.intersects(ix,iy,fine) { continue; }
                    for iz in (lower/fine+1e-5).floor() as i32..(ground_height/fine-1e-5).ceil() as i32 {
                        if observations.get([ix,iy,iz]).is_some() {
                            let top = f64::from(iz + 1) * fine;
                            // 倾斜机体的二维投影包含并未被机体占据的空间。
                            // 超过足端接触范围的侧面回波，必须实际与三维包络
                            // 相交才会抬高本次地面检查；原始占据仍完整保留。
                            if top <= support_plane + contact + 1e-6
                                || prism_query.intersects(
                                    [f64::from(ix)*fine, f64::from(iy)*fine, f64::from(iz)*fine],
                                    [f64::from(ix+1)*fine, f64::from(iy+1)*fine, top]) {
                                height = height.max(top);
                            }
                        }
                    }
                }
            }
            if height.is_finite() { ground_height = ground_height.min(height); }
        }
        Ok(ground_height)
    }

    /// Choose a nearby physical layer for projection when the nominal reference
    /// lies inside a measured band. This does not grant footprint support.
    pub(super) fn locate_step_contact(&self,_body:[f64;3],_q:[f64;4],reference:[f64;3],distance:f64)->Result<SurfaceLocation,SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::MissingSupport)?;
        if (model.body_to_support_z_m-distance).abs()>1e-9 {return Err(SurfaceError::MissingSupport);}
        // Select a real terrain anchor for projection. No footprint-wide support,
        // hits, completeness or duplicate static feasibility permission is built.
        let tolerance=model.max_step_up_m.max(model.max_step_down_m)
            +f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        let mut anchor=self.locate_near_support(reference,tolerance)?;
        anchor.support_xyz=[reference[0],reference[1],anchor.surface_xyz[2]];
        Ok(anchor)
    }

    /// 是否装配机体模型。
    pub fn body_envelope(&self) -> Option<&SurfaceBodyEnvelope> {
        self.data.body_envelope.as_deref()
    }

    // 脚下相邻接触点无需容纳完整机体中心和踏面窗口。只连接实际观测的
    // 邻列及腿部可达高度；机体移动仍由原路径连边与包络扫掠验证。
    fn contact_neighbors(&self, cell: VoxelKey, reference: &impl Fn(VoxelKey) -> f64) -> Vec<VoxelKey> {
        let Some(model) = self.body_envelope() else {
            return vec![];
        };
        let graph = &self.data.physical_graph;
        let quantum = f64::from(self.source_voxel_size_m());
        let height = self.ground_cell_position_near(cell, Some(reference(cell)))[2];
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
                let rise = self.ground_cell_position_near(next, Some(reference(next)))[2] - height;
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
        self.ground_patch_at_height(centre,radius,centre.surface_xyz[2])
    }

    fn ground_patch_at_height(&self,centre:SurfaceLocation,radius:f64,height:f64)
        -> Arc<AHashMap<(i32,i32),VoxelKey>> {
        let key = (centre.cell, radius.to_bits(), height.to_bits());
        // 实测平面属于观察快照，地形占据不变时其高度仍可能改变。
        // 这类邻域随现有查询缓存退休；无实测层的邻域仍共享物理图缓存。
        let measured = self.ground_evidence == GroundEvidence::MeasuredSurface
            && self.observations.as_ref().is_some_and(|s|s.has_measured_ground_layers());
        let cached = if measured {
            self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner())
                .measured_ground_patches.get(&key).cloned()
        } else {
            self.data.body_cache.lock().unwrap_or_else(|e|e.into_inner()).get(&key).cloned()
        };
        if let Some(patch) = cached {return patch;}
        // This source plane is determined by the existing cell/height cache key.
        // Extrapolation only selects an existing layer in each neighbour; it
        // never creates observed members or changes the actual step-height test.
        let center = self.cell_position(centre.cell);
        let reference_plane = self.measured_ground_layer_near(centre.cell,
            [center[0], center[1]], Some(height));
        let reference_at = |cell| reference_plane.and_then(|plane| {
            let point = self.cell_position(cell);
            plane.height_at([point[0],point[1]])
        }).unwrap_or(height);
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
            let reference = reference_at(cell);
            let p = self.ground_cell_position_near(cell, Some(reference));
            patch
                .entry((cell.0, cell.1))
                .and_modify(|old: &mut VoxelKey| {
                    if (self.ground_cell_position_near(*old, Some(reference))[2] - reference).abs()
                        > (p[2] - reference).abs()
                    {
                        *old = cell;
                    }
                })
                .or_insert(cell);
            for next in self.contact_neighbors(cell, &reference_at) {
                let Some(next_id)=graph.cells.id(next) else {continue;};
                let point = self.ground_cell_position_near(next, Some(reference_at(next)));
                if (point[0] - centre.surface_xyz[0]).hypot(point[1] - centre.surface_xyz[1])
                        <= radius
                    && seen.insert(next_id)
                {
                    queue.push_back(next_id);
                }
            }
        }
        let patch = Arc::new(patch);
        if measured {
            let mut owner = self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
            let cache = &mut owner.measured_ground_patches;
            if cache.len() >= 512 {cache.clear();}
            cache.insert(key, Arc::clone(&patch));
        } else {
            let mut cache = self.data.body_cache.lock().unwrap_or_else(|e|e.into_inner());
            if cache.len() >= 512 {cache.clear();}
            cache.insert(key, Arc::clone(&patch));
        }
        patch
    }

    fn validate_complete_source_resolution(&self)->Result<(),SurfaceError> {
        if self.observations.as_ref().is_some_and(|observations|observations.is_complete_geometry_source()
            && observations.size_m.to_bits()!=self.source_voxel_size_m().to_bits()) {
            return Err(SurfaceError::InvalidConfig("完整精细源与当前几何的来源分辨率不一致".into()));
        }
        Ok(())
    }

    /// 完整精细源仍须覆盖粗列里每一层已知占据；缺层或分辨率冲突不能解释掉粗障碍。
    fn complete_observations_cover_column(&self,x:i32,y:i32)->bool {
        if let Some(value)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner())
            .complete_columns.get(&(x,y)).copied() {return value;}
        let value=(|| {
            let observations=self.observations.as_ref()?;
            if !observations.is_complete_geometry_source()
                || observations.size_m.to_bits()!=self.source_voxel_size_m().to_bits() {return None;}
            let fine=f64::from(observations.size_m);let size=f64::from(self.config().voxel_size);
            let source=self;
            let Some(levels)=source.data.occupied_columns.get(&(x,y)) else {return Some(());};
            let first_x=(f64::from(x)*size/fine-0.5).ceil() as i32;
            let last_x=(f64::from(x+1)*size/fine-0.5).ceil() as i32;
            let first_y=(f64::from(y)*size/fine-0.5).ceil() as i32;
            let last_y=(f64::from(y+1)*size/fine-0.5).ceil() as i32;
            for &z in levels {
                let first=(f64::from(z)*size/fine-0.5).ceil() as i32;
                let last=(f64::from(z+1)*size/fine-0.5).ceil() as i32;
                let found=(first_x..last_x).any(|ix|(first_y..last_y).any(|iy| {
                    let Some(column)=observations.column_levels(ix,iy) else {return false;};
                    column.get(column.partition_point(|&iz|iz<first)).is_some_and(|&iz|iz<last)
                }));
                if !found {return None;}
            }
            Some(())
        })().is_some();
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.complete_columns.len()>=8192 {cache.complete_columns.clear();}
        cache.complete_columns.insert((x,y),value);value
    }

    /// 粗列被未相交的墙半列覆盖时，仅从实际相交细列读取真实暴露地面。
    fn fine_footprint_ground(&self,query:&PolygonQuery,body:[f64;3],normal:[f64;3],x:i32,y:i32)
        -> Option<(VoxelKey,f64)> {
        let observations=self.observations.as_ref()?;let model=self.body_envelope()?;
        if observations.size_m>=self.config().voxel_size {return None;}
        let size=f64::from(self.config().voxel_size);let fine=f64::from(observations.size_m);
        let reach=model.max_step_up_m.max(model.max_step_down_m)+f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        let occupied=|key:[i32;3]|observations.get(key).is_some();
        let coarse_levels=self.data.occupied_columns.get(&(x,y))?;
        let complete=self.complete_observations_cover_column(x,y);
        if observations.is_complete_geometry_source() && !complete {return None;}
        let mut height=f64::NEG_INFINITY;let mut chosen=None;
        for ix in (f64::from(x)*size/fine).floor() as i32..(f64::from(x+1)*size/fine).ceil() as i32 {
            for iy in (f64::from(y)*size/fine).floor() as i32..(f64::from(y+1)*size/fine).ceil() as i32 {
                if !query.intersects(ix,iy,fine) {continue;}
                let lower=query.contact_plane(body,normal,ix,iy,fine,f64::INFINITY,0.0)-model.body_to_support_z_m/normal[2];
                let upper=-query.contact_plane([body[0],body[1],-body[2]],[-normal[0],-normal[1],normal[2]],
                    ix,iy,fine,f64::INFINITY,0.0)-model.body_to_support_z_m/normal[2];
                let low=upper-reach;let high=lower+reach;let reference=(lower+upper)*0.5;
                let mut best:Option<(f64,VoxelKey,f64)>=None;
                let first=((low-1e-6)/fine).ceil() as i32-1;
                let last=((high+1e-6)/fine).floor() as i32-1;
                for iz in first..=last {
                    if !occupied([ix,iy,iz]) || (!complete && !observations.has_complete_block([ix,iy,iz+1]))
                        || occupied([ix,iy,iz+1]) {continue;}
                    // 必须是原几何中的真实占据带顶，不能在行程上限截断高墙。
                    let top=f64::from(iz+1)*fine;let z=(top/size).ceil() as i32-1;
                    if coarse_levels.binary_search(&z).is_err() {continue;}
                    let cell=(x,y,z);let top=top.min(self.ground_cell_position(cell)[2]);
                    if top<low-1e-6 || top>high+1e-6 || top<f64::from(iz)*fine-1e-6 {continue;}
                    let distance=(top-reference).abs();
                    if best.as_ref().is_none_or(|best|distance<best.0) {best=Some((distance,cell,top));}
                }
                let (_,cell,top)=best?;
                if top>height {height=top;chosen=Some(cell);}
            }
        }
        chosen.map(|cell|(cell,height))
    }

    /// 精细补充仅属于本次足迹查询，并须按原腿部高差连到已有支撑分量。
    fn refine_footprint_support(&self,patch:Arc<AHashMap<(i32,i32),VoxelKey>>,query:&PolygonQuery,
        prism:&PrismQuery,body:[f64;3],r:[[f64;3];3],bounds:([i32;2],[i32;2]))
        -> (Arc<AHashMap<(i32,i32),VoxelKey>>,AHashMap<(i32,i32),f64>) {
        let mut precise=AHashMap::new();
        let Some(model)=self.body_envelope() else {return (patch,precise);};
        if self.observations.as_ref().is_none_or(|o|o.size_m>=self.config().voxel_size) {return (patch,precise);}
        let size=f64::from(self.config().voxel_size);let quantum=f64::from(self.source_voxel_size_m());
        let contact=model.max_step_up_m.max(model.max_step_down_m)+quantum+model.support_uncertainty_m;
        let mut pending=Vec::new();
        for x in bounds.0[0]..=bounds.1[0] {for y in bounds.0[1]..=bounds.1[1] {
            if patch.contains_key(&(x,y)) || !query.intersects(x,y,size) {continue;}
            if let Some((cell,height))=self.fine_footprint_ground(query,body,r[2],x,y) {pending.push((cell,height));}
        }}
        if pending.is_empty() {return (patch,precise);}
        let mut extended=None;let mut known_heights=AHashMap::new();
        loop {
            let mut progress=false;
            pending.retain(|&(cell,height)| {
                let mut attached=false;
                for (dx,dy) in DIRECTIONS {
                    let key=(cell.0+dx,cell.1+dy);
                    let source=extended.as_ref().unwrap_or(patch.as_ref());
                    let Some(&neighbor)=source.get(&key) else {continue;};
                    if !query.intersects(key.0,key.1,size) {continue;}
                    let neighbor_height=if let Some(&height)=precise.get(&key) {height}
                        else if let Some(&height)=known_heights.get(&key) {height}
                        else {
                            let plane=query.contact_plane(body,r[2],key.0,key.1,size,self.ground_cell_position(neighbor)[2],
                                -model.body_to_support_z_m/r[2][2]+contact+1e-6);
                            let height=if let Some(height)=self.fine_ground_height(neighbor,query,prism,body,r[2],key.0,key.1) {height}
                                else {
                                    let Ok(height)=self.footprint_ground_height(neighbor,query,prism,
                                        plane-model.body_to_support_z_m/r[2][2],contact,None) else {continue;};
                                    height
                                };
                            known_heights.insert(key,height);height
                        };
                    let rise=height-neighbor_height;
                    if rise<=model.max_step_up_m+quantum+1e-6 && -rise<=model.max_step_down_m+quantum+1e-6 {
                        attached=true;break;
                    }
                }
                if attached {
                    extended.get_or_insert_with(||patch.as_ref().clone()).insert((cell.0,cell.1),cell);
                    precise.insert((cell.0,cell.1),height);progress=true;false
                } else {true}
            });
            if !progress {break;}
        }
        (extended.map(Arc::new).unwrap_or(patch),precise)
    }

    /// 用完整姿态检查机体凸柱与占据体素；允许已确认支撑面的接触。
    pub fn validate_body_pose(&self, body:[f64;3], quaternion:[f64;4])->Result<(),SurfaceError> {
        self.validate_body_pose_with_witness(body,quaternion,&mut None)
    }

    /// Same query and legacy error; the slot describes only this rejected pose.
    pub fn validate_body_pose_with_witness(&self, body:[f64;3], quaternion:[f64;4],
        slot:&mut Option<BodyFailureWitness>)->Result<(),SurfaceError> {
        *slot=None;
        let key = [body[0].to_bits(),body[1].to_bits(),body[2].to_bits(),
            quaternion[0].to_bits(),quaternion[1].to_bits(),quaternion[2].to_bits(),quaternion[3].to_bits()];
        let mut rejection=None;
        let cached=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner()).poses.get(&key).cloned();
        let result=if let Some(cached)=cached {
            #[cfg(feature = "body-query-profile")]
            query_profile::count(QueryCounter::PoseCacheHits);
            cached.result(&mut rejection)
        } else {
            #[cfg(feature = "body-query-profile")]
            query_profile::count(QueryCounter::PoseCacheMisses);
            let result=self.check_body_pose(body,quaternion,&mut rejection);
            let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
            if cache.poses.len()>=8192 {
                #[cfg(feature = "body-query-profile")]
                query_profile::count(QueryCounter::PoseCacheCapacityClears);
                cache.poses.clear();
            }
            cache.poses.insert(key,CachedBodyResult::new(result.clone(),rejection));
            result
        };
        *slot=rejection.map(|rejection|BodyFailureWitness{pose:(body,quaternion),rejection});
        result
    }

    fn recent_surface_contact(&self,point:[f64;3],height:Option<f64>,body:[f64;3],r:[[f64;3];3])->bool {
        let Some(model)=self.body_envelope() else {return false;};
        let Some(height)=height else {return false;};
        let uncertainty=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        if (point[2]-height).abs()>uncertainty+1e-6 {return false;}
        let contact=model.max_step_up_m.max(model.max_step_down_m)+uncertainty;
        let plane=body[2]-((point[0]-body[0])*r[2][0]+(point[1]-body[1])*r[2][1])/r[2][2];
        height_rejection(plane+model.z_bounds_m[0]/r[2][2],plane-model.body_to_support_z_m/r[2][2],
            plane,contact,point[2],uncertainty).is_none()
    }

    /// Local guide-only obstacle bounds. Measured surface returns retain their
    /// actual coordinates; unclassified returns retain occupied voxel volumes.
    /// This is not a motion permission: callers must use the latest swept checker.
    pub fn recent_obstacle_hints(&self,bounds_xy:[[f64;2];2],reference_body:([f64;3],[f64;4]))
        ->Result<Vec<([f64;3],[f64;3])>,SurfaceError> {
        let Some(recent)=&self.recent_obstacles else {return Ok(vec![]);};
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        if !bounds_xy.into_iter().flatten().all(f64::is_finite)
            || (0..2).any(|i|bounds_xy[0][i]>bounds_xy[1][i])
            || bounds_xy.into_iter().flatten().any(|v|(v/recent.size).abs()>=f64::from(i32::MAX)*0.5) {
            return Err(SurfaceError::InvalidConfig("invalid recent-obstacle hint bounds".into()));
        }
        if !reference_body.0.into_iter().all(f64::is_finite) {return Err(SurfaceError::InvalidPose);}
        let r=rotation(reference_body.1)?;
        if r[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        let fine=recent.size;let coarse=f64::from(self.config().voxel_size);
        let low=bounds_xy[0].map(|v|(v/fine).floor() as i32);
        let high=bounds_xy[1].map(|v|(v/fine).floor() as i32);
        let centre=self.locate_support_from_body(reference_body.0,reference_body.1,model.body_to_support_z_m).ok();
        let radius=bounds_xy.into_iter().map(|p|(p[0]-reference_body.0[0]).hypot(p[1]-reference_body.0[1]))
            .fold(0.0,f64::max)+coarse*2.0;
        let patch=centre.map(|c|self.ground_patch(c,radius));
        // A detour hint must interpret the same source-local floor as the body
        // checker, even when the center has no observed/connected ground anchor.
        // The query rectangle only selects existing columns; it creates no floor.
        let rectangle=[bounds_xy[0],[bounds_xy[1][0],bounds_xy[0][1]],
            bounds_xy[1],[bounds_xy[0][0],bounds_xy[1][1]]];
        let polygon=PolygonQuery::new(&rectangle);
        let prism=PrismQuery::new(model,r,reference_body.0);
        let mut result=Vec::new();
        for frame in &recent.frames {for (&[x,y],zs) in frame.columns.range(low..=high) {
            if y<low[1] || y>high[1] {continue;}
            let xy=[x,y].map(|v|(f64::from(v)+0.5)*fine);
            let column=xy.map(|v|(v/coarse).floor() as i32);
            let ground=patch.as_ref().and_then(|p|p.get(&(column[0],column[1]))).copied();
            let height=if let Some(observations)=&self.observations {
                self.select_fine_ground_value::<FineGroundHeight>(observations,model,ground,&polygon,&prism,
                    reference_body.0,r[2],[x,y],None).0
            } else {ground.map(|ground|self.ground_cell_position(ground)[2])};
            // 参考沿地形改变高度，不能按起点高度裁掉其它路段的真实障碍。
            for &z in zs {
                if let Some(points)=frame.surface_points.get(&[x,y,z]) {
                    for &point in points {
                        if !self.recent_surface_contact(point,height,reference_body.0,r) {result.push((point,point));}
                    }
                } else {
                    let a=[x,y,z].map(|v|f64::from(v)*fine);let b=[x,y,z].map(|v|f64::from(v+1)*fine);
                    result.push((a,b));
                }
            }
        }}
        Ok(result)
    }

    fn check_body_pose(&self,body:[f64;3],quaternion:[f64;4],slot:&mut Option<BodyRejection>)->Result<(),SurfaceError> {
        *slot=None;
        // Declared before all kernel locals so normal/error cleanup stays inside T.
        #[cfg(feature = "body-query-profile")]
        let _pose_core_profile=query_profile::guard(QueryStage::PoseCoreTotal);
        #[cfg(feature = "body-query-profile")]
        let mut pose_column_counts=PoseColumnCounts::default();
        #[cfg(feature = "body-query-profile")]
        let mut assigned_z_counts=AssignedZCounts::default();
        #[cfg(feature = "body-query-profile")]
        let pose_prelude_profile=query_profile::guard(QueryStage::PosePrelude);
        self.validate_complete_source_resolution()?;
        let model = self
            .body_envelope()
            .ok_or_else(|| SurfaceError::InvalidConfig("body envelope is unavailable".into()))?;
        let r=rotation(quaternion)?;
        if r[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        // Ground is useful for contact interpretation, not a second permission to
        // execute geometry checking. Missing samples are not occupied obstacles.
        let centre=match self.locate_support_from_body(body,quaternion,model.body_to_support_z_m) {
            Ok(value)=>Some(value),Err(SurfaceError::MissingSupport)=>None,Err(error)=>return Err(error),
        };
        let vertices: Vec<_> = model
            .footprint_xy_m
            .iter()
            .flat_map(|p| model.z_bounds_m.map(|z| world(r, body, [p[0], p[1], z])))
            .collect();
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let polygon_query=PolygonQuery::new(&polygon);
        let prism_query=PrismQuery::new(model,r,body);
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
        // 查询半径取实际投影足迹到支撑格中心的最远距离；高度和支撑
        // 距离已经包含在旋转后的顶点里，不能再各加一次扩大地面遍历。
        // 向上对齐体素以复用同一格的邻域，并保留两格接触连接余量。
        let radius = ((vertices.iter().map(|p|(p[0]-centre.map_or(body[0],|c|c.surface_xyz[0])).hypot(p[1]-centre.map_or(body[1],|c|c.surface_xyz[1])))
            .fold(0.0,f64::max)+size*2.0)/size).ceil()*size;
        let patch=centre.map_or_else(||Arc::new(AHashMap::new()),|centre|self.ground_patch(centre,radius));
        let (patch,precise_ground)=self.refine_footprint_support(patch,&polygon_query,&prism_query,body,r,([x0,y0],[x1,y1]));
        #[cfg(feature = "body-query-profile")]
        drop(pose_prelude_profile);
        if let Some(recent)=&self.recent_obstacles {
            let fine=recent.size;
            // All row inputs (pose, patch, source and prism)
            // stay fixed during this query. Reuse only the last coarse column;
            // every recent frame, fine XY and Z still executes in original order.
            let mut selected_column=None;
            let mut selected_rows:Option<Vec<FineGroundHeightRow>>=None;
            for frame in &recent.frames {
                let low=[(min[0]/fine).floor() as i32,(min[1]/fine).floor() as i32];
                let high=[(max[0]/fine).floor() as i32,(max[1]/fine).floor() as i32];
                for (&[x,y],zs) in frame.columns.range(low..=high) {
                    if y<low[1] || y>high[1] {continue;}
                    let first=zs.partition_point(|&z|f64::from(z+1)*fine<min[2]);
                    let last=zs.partition_point(|&z|f64::from(z)*fine<=max[2]);
                    let zs=&zs[first..last];if zs.is_empty() {continue;}
                    {
                            let coarse=f64::from(self.config().voxel_size);
                            let cx=(((f64::from(x)+0.5)*fine)/coarse).floor() as i32;
                            let cy=(((f64::from(y)+0.5)*fine)/coarse).floor() as i32;
                            let ground=patch.get(&(cx,cy)).copied();
                            if selected_column!=Some((cx,cy)) {
                                selected_rows=self.fine_ground_values::<FineGroundHeightRow>(
                                    ground,&polygon_query,&prism_query,body,r[2],cx,cy);
                                selected_column=Some((cx,cy));
                            }
                            let height=if let Some(rows)=&selected_rows {rows.iter().find(|row|row.xy==[x,y]).and_then(|row|row.height)}
                                else {ground.map(|g|self.ground_cell_position(g)[2])};
                            for &z in zs {
                                if let Some(points)=frame.surface_points.get(&[x,y,z]) {
                                    if !points.iter().any(|&point|!self.recent_surface_contact(point,height,body,r)
                                        &&prism_query.intersects(point,point)) {continue;}
                                }
                                let low=[x,y,z].map(|v|f64::from(v)*fine);
                                let high=[x,y,z].map(|v|f64::from(v+1)*fine);
                                if high[2]>=min[2] && low[2]<=max[2] && prism_query.intersects(low,high) {
                                    return Err(reject(slot,BodyRejection::Occupancy{cell:[x,y,z],
                                        source:OccupancySource::Recent,z_bounds:[low[2],high[2]],ground_height:height}));
                                }
                            }
                        }
                }
            }
        }
        self.check_fine_column_fringe(body,r,&polygon_query,&prism_query,&patch,min,max,([x0,y0],[x1,y1]),slot)?;
        let uncertainty = f64::from(self.source_voxel_size_m()) + model.support_uncertainty_m;
        let mut ground_points = Vec::new();
        // One guard for the entire loop, never a clock/TLS update per XY or Z.
        #[cfg(feature = "body-query-profile")]
        let pose_columns_profile=query_profile::guard(QueryStage::PoseCoarseColumns);
        for x in x0..=x1 {
            for y in y0..=y1 {
                if !polygon_query.intersects(x,y,size) {
                    continue;
                }
                let Some(&ground)=patch.get(&(x,y)) else {
                    #[cfg(feature = "body-query-profile")]
                    { pose_column_counts.unassigned=pose_column_counts.unassigned.saturating_add(1); }
                    // A sparse/unknown footprint column does not veto the pose.
                    // It still receives the exact same actual-volume collision pass.
                    if self.data.virtual_obstacles.as_ref().is_some_and(|mask|
                        !mask.permits_cell((x,y,0),size,0.0)) {return Err(SurfaceError::VirtualObstacle);}
                    self.check_unassigned_body_column(body,r,&polygon_query,&prism_query,x,y,slot)?;
                    continue;
                };
                #[cfg(feature = "body-query-profile")]
                { pose_column_counts.assigned=pose_column_counts.assigned.saturating_add(1); }
                if self
                    .data
                    .virtual_obstacles
                    .as_ref()
                    .is_some_and(|mask| !mask.permits_cell(ground, size, 0.0))
                {
                    return Err(SurfaceError::VirtualObstacle);
                }
                let px = (x as f64 + 0.5) * size;
                let py = (y as f64 + 0.5) * size;
                let contact = model.max_step_up_m.max(model.max_step_down_m) + uncertainty;
                let plane=polygon_query.contact_plane(body,r[2],x,y,size,self.ground_cell_position(ground)[2],
                    (-model.body_to_support_z_m/r[2][2]+contact+1e-6).min(uncertainty));
                let support_plane = plane - model.body_to_support_z_m / r[2][2];
                // Borrow the original coarse levels. Only the original strict Z-separation
                // predicate proves that no collision consumer can use fine contact rows.
                let occupied_zs=self.data.occupied_columns.get(&(x,y));
                let has_vertical_overlap=occupied_zs.is_some_and(|zs|zs.iter().any(|&z| {
                    let bounds=self.data.height_bounds.get(&(x,y,z)).copied().unwrap_or([
                        z as f32*self.config().voxel_size,
                        (z as f32+1.0)*self.config().voxel_size,
                    ]);
                    !(f64::from(bounds[0])>max[2] || f64::from(bounds[1])<min[2])
                }));
                if !has_vertical_overlap {
                    // Same ordered height selection/reduction, without unused contact membership.
                    // Some(all-missing rows) still falls back inside fine_ground_height;
                    // only a genuine None permits the original precise-ground precedence.
                    let ground_height=if let Some(height)=self.fine_ground_height(
                        ground,&polygon_query,&prism_query,body,r[2],x,y) {height}
                        else if precise_ground.contains_key(&(x,y)) {precise_ground[&(x,y)]}
                        else {self.footprint_ground_height(
                            ground,&polygon_query,&prism_query,support_plane,contact,None)?};
                    ground_points.push([px,py,ground_height]);
                    continue;
                }
                let selected=self.fine_ground_columns(Some(ground),&polygon_query,&prism_query,body,r[2],x,y);
                let ground_height = if selected.is_none() && precise_ground.contains_key(&(x,y)) {precise_ground[&(x,y)]} else {
                    self.footprint_ground_height(ground,&polygon_query,&prism_query,support_plane,contact,selected.as_deref())?
                };
                // Static height/ceiling feasibility belongs to PCT. Preserve the
                // original occupied voxels for actual-pose collision below instead
                // of repeating a coarse max-height veto and a fine rescue pass.
                ground_points.push([px, py, ground_height]);
                // This only decides whether an occupied ground band is contact.
                // A failed contact interpretation is rejected only if the actual
                // remaining occupied volume intersects the prism below.
                let coarse_contact=height_rejection(plane+model.z_bounds_m[0]/r[2][2],
                    support_plane,plane,contact,ground_height,uncertainty).is_none();
                if let Some(zs) = occupied_zs {
                    for &z in zs {
                        let bounds = self.data.height_bounds.get(&(x, y, z)).copied().unwrap_or([
                            z as f32 * self.config().voxel_size,
                            (z as f32 + 1.0) * self.config().voxel_size,
                        ]);
                        if (selected.is_none() && coarse_contact && f64::from(bounds[1]) <= ground_height + 1e-6)
                            || f64::from(bounds[0]) > max[2]
                            || f64::from(bounds[1]) < min[2]
                        {
                            continue;
                        }
                        let mut fine_hit=None;
                        let coarse_intersects = || prism_query.intersects(
                            [x as f64 * size, y as f64 * size, f64::from(bounds[0])],
                            [(x + 1) as f64 * size, (y + 1) as f64 * size, f64::from(bounds[1])],
                        );
                        let intersects = if let Some(observations) = &self.observations {
                            let fine = f64::from(observations.size_m);
                            let mut intersects = false;
                            let mut found = false;
                            let mut missing_slice_hit=false;
                            for ix in (f64::from(x)*size/fine).round() as i32..(f64::from(x+1)*size/fine).round() as i32 {
                                for iy in (f64::from(y)*size/fine).round() as i32..(f64::from(y+1)*size/fine).round() as i32 {
                                    let column=selected.as_ref().and_then(|columns|columns.iter().find(|c|c.xy==[ix,iy]));
                                    let fine_ground=if selected.is_some() {column.and_then(|c|c.height)} else {Some(ground_height)};
                                    // A partial fine source cannot erase the unknown
                                    // XY slice of an already occupied coarse volume.
                                    if selected.is_some() && column.is_none_or(|c|c.height.is_none())
                                        && !observations.is_complete_geometry_source()
                                        && (!coarse_contact || f64::from(bounds[1])>ground_height+1e-6)
                                        && prism_query.intersects([f64::from(ix)*fine,f64::from(iy)*fine,f64::from(bounds[0])],
                                            [f64::from(ix+1)*fine,f64::from(iy+1)*fine,f64::from(bounds[1])]) {
                                        missing_slice_hit=true;
                                    }
                                    #[cfg(feature = "body-query-profile")]
                                    {
                                        let first=(f64::from(bounds[0])/fine+1e-5).floor() as i32;
                                        let last=(f64::from(bounds[1])/fine-1e-5).ceil() as i32;
                                        assigned_z_counts.dense_z_span=assigned_z_counts.dense_z_span
                                            .saturating_add((i64::from(last)-i64::from(first)).max(0) as u64);
                                    }
                                    for iz in (f64::from(bounds[0])/fine+1e-5).floor() as i32..(f64::from(bounds[1])/fine-1e-5).ceil() as i32 {
                                        #[cfg(feature = "body-query-profile")]
                                        { assigned_z_counts.actual_z_candidates=assigned_z_counts.actual_z_candidates.saturating_add(1); }
                                        // Both contact-run forms contain only real supported members
                                        // of this immutable observation owner. Preserve `found` so
                                        // an exempt fine contact cannot trigger coarse fallback.
                                        if column.is_some_and(|c|c.permits_contact(iz)) {
                                            found=true;
                                            continue;
                                        }
                                        if observations.get([ix,iy,iz]).is_none() {
                                            #[cfg(feature = "body-query-profile")]
                                            { assigned_z_counts.fine_get_absent=assigned_z_counts.fine_get_absent.saturating_add(1); }
                                            continue;
                                        }
                                        found = true;
                                        let contact=selected.is_none() && coarse_contact
                                            && fine_ground.is_some_and(|height|f64::from(iz+1)*fine<=height+1e-6);
                                        if contact {continue;}

                                        let hit=prism_query.intersects(
                                            [f64::from(ix)*fine,f64::from(iy)*fine,f64::from(iz)*fine],
                                            [f64::from(ix+1)*fine,f64::from(iy+1)*fine,f64::from(iz+1)*fine]);
                                        if hit && fine_hit.is_none() {fine_hit=Some(([ix,iy,iz],[f64::from(iz)*fine,f64::from(iz+1)*fine],fine_ground));}
                                        intersects |= hit;
                                    }
                                }
                            }
                            // 缺少该占据格的精细依据时保留粗格障碍，不把缺测当作自由。
                            if found { intersects || missing_slice_hit } else {  (!coarse_contact || f64::from(bounds[1])>ground_height+1e-6) && coarse_intersects() }
                        } else {  coarse_intersects() };
                        if intersects {
                            tracing::debug!(?body, x, y, z, ?bounds, ground_height, "机体与占据单元相交");
                            let (cell,z_bounds,source,ground_height)=match fine_hit {
                                Some((cell,bounds,height))=>(cell,bounds,OccupancySource::Fine,height),
                                None=>([x,y,z],bounds.map(f64::from),OccupancySource::CoarseFallback,Some(ground_height)),
                            };
                            return Err(reject(slot,BodyRejection::Occupancy{cell,source,z_bounds,ground_height}));
                        }
                    }
                }
            }
        }
        #[cfg(feature = "body-query-profile")]
        drop(pose_columns_profile);
        if let Some((a, b, _)) = fit_plane(&ground_points, centre.map_or(body,|c|c.support_xyz)) {
            let yaw = r[0][1].atan2(r[0][0]);
            let cross_slope = (-a * yaw.sin() + b * yaw.cos()).abs().atan().to_degrees();
            if cross_slope > model.max_cross_slope_deg + 1e-5 {
                tracing::debug!(?body, cross_slope, limit=model.max_cross_slope_deg, "实际机体支撑横坡超限");
                return Err(SurfaceError::UntraversableSegment);
            }
        }
        // Known terrain contradictions (including a measured drop) remain a
        // safety constraint. Absence of a layer returns Unknown and grants no
        // fabricated observation, but it is not a rejection.
        if self.body_ground_relation(body,quaternion)? == BodyGroundRelation::KnownIncompatible {
            return Err(SurfaceError::UntraversableSegment);
        }
        Ok(())
    }

    fn check_unassigned_body_column(&self,body:[f64;3],r:[[f64;3];3],polygon:&PolygonQuery,
        prism:&PrismQuery,x:i32,y:i32,slot:&mut Option<BodyRejection>)->Result<(),SurfaceError> {
        let size=f64::from(self.config().voxel_size);
        if let Some(observations)=&self.observations {
            let fine=f64::from(observations.size_m);
            let selected=self.fine_ground_columns(None,polygon,prism,body,r[2],x,y);
            let mut covered=std::collections::BTreeSet::new();
            for ix in (f64::from(x)*size/fine).floor() as i32..(f64::from(x+1)*size/fine).ceil() as i32 {
                for iy in (f64::from(y)*size/fine).floor() as i32..(f64::from(y+1)*size/fine).ceil() as i32 {
                    let levels=observations.column_levels(ix,iy).unwrap_or(&[]);
                    // 完整细源对粗格的覆盖与机体投影无关；先保留该粗格所有真实高度证据。
                    // 否则机体外的细占据会被遗漏，随后又被膨胀为相交的整块粗障碍。
                    for &z in levels {
                        if observations.get([ix,iy,z]).is_some() {
                            covered.insert((((f64::from(z)+0.5)*fine)/size).floor() as i32);
                        }
                    }
                    if !polygon.intersects(ix,iy,fine) {continue;}
                    let column=selected.as_ref().and_then(|rows|rows.iter().find(|row|row.xy==[ix,iy]));
                    for &z in levels {
                        let key=[ix,iy,z];
                        if column.is_some_and(|column|column.permits_contact(z))
                            || !observations.get(key).is_some() {continue;}
                        if prism.intersects(key.map(|v|f64::from(v)*fine),key.map(|v|(f64::from(v)+1.0)*fine)) {
                            return Err(reject(slot,BodyRejection::Occupancy{cell:key,source:OccupancySource::Fine,
                                z_bounds:[f64::from(z)*fine,(f64::from(z)+1.0)*fine],ground_height:column.and_then(|c|c.height)}));
                        }
                    }
                }
            }
            // Partial fine input cannot clear an already known coarse obstacle.
            for &z in self.data.occupied_columns.get(&(x,y)).into_iter().flatten() {
                if observations.is_complete_geometry_source() && covered.contains(&z) {continue;}
                let bounds=self.data.height_bounds.get(&(x,y,z)).map_or(
                    [f64::from(z)*size,(f64::from(z)+1.0)*size],|b|b.map(f64::from));
                if prism.intersects([f64::from(x)*size,f64::from(y)*size,bounds[0]],
                    [(f64::from(x)+1.0)*size,(f64::from(y)+1.0)*size,bounds[1]]) {
                    return Err(reject(slot,BodyRejection::Occupancy{cell:[x,y,z],source:OccupancySource::CoarseFallback,
                        z_bounds:bounds,ground_height:None}));
                }
            }
        } else {
            let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
            let reference=body[2]-model.body_to_support_z_m*r[2][2];
            let uncertainty=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
            let ground=self.data.physical_graph.surface_lookup.get(&(x,y)).into_iter().flatten()
                .map(|&z|self.ground_cell_position((x,y,z))[2])
                .filter(|&height|height-reference<=model.max_step_up_m+uncertainty+1e-6
                    && reference-height<=model.max_step_down_m+uncertainty+1e-6)
                .min_by(|a,b|(a-reference).abs().total_cmp(&(b-reference).abs()));
            for &z in self.data.occupied_columns.get(&(x,y)).into_iter().flatten() {
                let bounds=self.data.height_bounds.get(&(x,y,z)).map_or(
                    [f64::from(z)*size,(f64::from(z)+1.0)*size],|b|b.map(f64::from));
                if ground.is_some_and(|height|bounds[1]<=height+1e-6) {continue;}
                if prism.intersects([f64::from(x)*size,f64::from(y)*size,bounds[0]],
                    [(f64::from(x)+1.0)*size,(f64::from(y)+1.0)*size,bounds[1]]) {
                    return Err(reject(slot,BodyRejection::Occupancy{cell:[x,y,z],source:OccupancySource::CoarseFallback,
                        z_bounds:bounds,ground_height:None}));
                }
            }
        }
        Ok(())
    }

    /// 细体素可能跨越粗列的浮点边界；粗列只是索引，不能截掉实际机体接触。
    /// 只补查中心归属在粗包围框外、实际体积却与机体相交的已知细体素。
    fn check_fine_column_fringe(&self,body:[f64;3],r:[[f64;3];3],polygon:&PolygonQuery,prism:&PrismQuery,
        patch:&AHashMap<(i32,i32),VoxelKey>,min:[f64;3],max:[f64;3],bounds:([i32;2],[i32;2]),slot:&mut Option<BodyRejection>)
        ->Result<(),SurfaceError> {
        let Some(observations)=self.observations.as_ref() else {return Ok(());};
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let fine=f64::from(observations.size_m);let size=f64::from(self.config().voxel_size);
        let axis=|i:usize|->Vec<(i32,i32)> {
            let mut low=(min[i]/fine).floor() as i32;let mut high=(max[i]/fine).floor() as i32;
            // 与水平分离轴使用同一数值边界，只为实际可能接触的相邻细格保留闭边。
            if f64::from(low)*fine>=min[i]-1e-10 {low=low.saturating_sub(1);}
            if f64::from(high.saturating_add(1))*fine<=max[i]+1e-10 {high=high.saturating_add(1);}
            (low..=high).map(|key| {
                let point=((f64::from(key)+0.5)*fine) as f32;
                (key,(point*(1.0/self.config().voxel_size)).floor() as i32)
            }).collect()
        };
        let xs=axis(0);let ys=axis(1);
        let outer_y:Vec<_>=ys.iter().copied().filter(|&(_,y)|y<bounds.0[1]||y>bounds.1[1]).collect();
        let contact=model.max_step_up_m.max(model.max_step_down_m)+fine+model.support_uncertainty_m;
        let mut selected_column=None;
        let mut selected_rows=None;
        for (ix,x) in xs {
            let row=if x<bounds.0[0] || x>bounds.1[0] {&ys}else{&outer_y};
            for &(iy,y) in row {
                if !polygon.intersects(ix,iy,fine) {continue;}
                let Some(levels)=observations.column_levels(ix,iy) else {continue;};
                let first=((min[2]/fine).floor() as i32).saturating_sub(1);let last=(max[2]/fine).floor() as i32;
                let begin=levels.partition_point(|&iz|iz<first);
                if levels.get(begin).is_none_or(|&iz|iz>last) {continue;}
                if selected_column!=Some((x,y)) {
                    selected_rows=self.fine_ground_columns(patch.get(&(x,y)).copied(),polygon,prism,body,r[2],x,y);
                    selected_column=Some((x,y));
                }
                let column=selected_rows.as_ref().and_then(|rows|rows.iter().find(|c|c.xy==[ix,iy]));
                let ground=if selected_rows.is_some() {
                    column.and_then(|c|c.height)
                } else if let Some(&cell)=patch.get(&(x,y)) {
                    let plane=polygon.contact_plane(body,r[2],x,y,size,self.ground_cell_position(cell)[2],
                        -model.body_to_support_z_m/r[2][2]+contact+1e-6);
                    Some(self.footprint_ground_height(cell,polygon,prism,plane-model.body_to_support_z_m/r[2][2],contact,None)?)
                } else {self.fine_footprint_ground(polygon,body,r[2],x,y).map(|(_,height)|height)};
                for &iz in levels.iter().skip(begin).take_while(|&&iz|iz<=last) {
                    let key=[ix,iy,iz];
                    let contact=if selected_rows.is_some() {column.is_some_and(|c|c.permits_contact(iz))}
                        else {ground.is_some_and(|height|f64::from(iz+1)*fine<=height+1e-6)};
                    if contact
                        || !observations.get(key).is_some() {continue;}
                    if prism.intersects(key.map(|v|f64::from(v)*fine),key.map(|v|f64::from(v+1)*fine)) {
                        return Err(reject(slot,BodyRejection::Occupancy{cell:key,source:OccupancySource::FineFringe,
                            z_bounds:[f64::from(iz)*fine,f64::from(iz+1)*fine],ground_height:ground}));
                    }
                }
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
        let key=[xy[0].to_bits(),xy[1].to_bits(),yaw.to_bits(),
            previous_support[0].to_bits(),previous_support[1].to_bits(),previous_support[2].to_bits()];
        if let Some(result)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner()).projections.get(&key).cloned() {
            return result;
        }
        let result=self.project_body_pose(xy,yaw,previous_support);
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.projections.len()>=8192 {cache.projections.clear();}
        cache.projections.insert(key,result.clone());
        result
    }

    /// 从实际机体位姿推进到目标平面位置，仅应用真实地形的相对高差和法向变化。
    /// 保留起点的高度/倾斜偏差；返回的目标仍须与起点一起通过完整扫掠和观察检查。
    pub fn body_pose_after_motion(
        &self,
        from: ([f64; 3], [f64; 4]),
        xy: [f64; 2],
        yaw: f64,
    ) -> Result<([f64; 3], [f64; 4]), SurfaceError> {
        self.body_pose_after_motion_with_witness(from,xy,yaw,&mut None)
    }

    /// Preserve the actual rejected projected pose without replaying its geometry.
    pub fn body_pose_after_motion_with_witness(&self,from:([f64;3],[f64;4]),xy:[f64;2],yaw:f64,
        slot:&mut Option<BodyFailureWitness>)->Result<([f64;3],[f64;4]),SurfaceError> {
        *slot=None;
        #[cfg(feature = "body-query-profile")]
        let _motion_profile = query_profile::guard(QueryStage::MotionTotal);
        if !from.0.into_iter().chain(xy).chain([yaw]).all(f64::is_finite) {
            return Err(SurfaceError::InvalidPose);
        }
        self.validate_complete_source_resolution()?;
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let actual_rotation = rotation(from.1)?;
        let current_yaw = actual_rotation[0][1].atan2(actual_rotation[0][0]);
        let yaw_delta = (yaw-current_yaw+std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
            - std::f64::consts::PI;
        if xy == [from.0[0],from.0[1]] && yaw_delta.abs() < 1e-12 {
            #[cfg(feature = "body-query-profile")]
            let validation_profile = query_profile::guard(QueryStage::FinalBodyValidation);
            self.validate_body_pose_with_witness(from.0,from.1,slot)?;
            #[cfg(feature = "body-query-profile")]
            drop(validation_profile);
            return Ok(from);
        }
        #[cfg(feature = "body-query-profile")]
        let current_support_profile = query_profile::guard(QueryStage::CurrentSupportLookup);
        match self.locate_support_from_body(from.0,from.1,model.body_to_support_z_m) {
            Ok(_)=>(),
            Err(SurfaceError::MissingSupport)=>return self.estimated_body_motion(from,xy,yaw,slot),
            Err(error)=>return Err(error),
        };
        #[cfg(feature = "body-query-profile")]
        drop(current_support_profile);
        if actual_rotation[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        // 拟合查询在机体中心XY；名义接触平面在这里的高度是z-d/n_z，
        // 不能把位于旋转支撑点XY的z-d*n_z直接当作中心参考。
        let initial_height=from.0[2]-model.body_to_support_z_m/actual_rotation[2][2];
        let normal_tolerance=f64::from(self.config().step_threshold_m+self.config().voxel_size);
        #[cfg(feature = "body-query-profile")]
        let base_support_profile = query_profile::guard(QueryStage::BaseNearSupportLookup);
        let base_projection=match self.locate_near_support(
            [from.0[0],from.0[1],initial_height],normal_tolerance) {
            Ok(projected)=>projected,
            Err(SurfaceError::MissingSupport)=>return self.estimated_body_motion(from,xy,yaw,slot),
            Err(error)=>return Err(error),
        };
        #[cfg(feature = "body-query-profile")]
        drop(base_support_profile);
        #[cfg(feature = "body-query-profile")]
        let next_support_profile = query_profile::guard(QueryStage::NextNearSupportLookup);
        let projected=match self.locate_near_support(
            [xy[0],xy[1],base_projection.surface_xyz[2]],normal_tolerance) {
            Ok(projected)=>projected,
            Err(SurfaceError::MissingSupport)=>return self.estimated_body_motion(from,xy,yaw,slot),
            Err(error)=>return Err(error),
        };
        #[cfg(feature = "body-query-profile")]
        drop(next_support_profile);
        let (base_height,base_attitude)=match self.body_motion_contact_frame(base_projection,initial_height,current_yaw) {
            Ok(frame)=>frame,Err(SurfaceError::MissingSupport)=>return self.estimated_body_motion(from,xy,yaw,slot),
            Err(error)=>return Err(error),
        };
        let (next_height,next_attitude)=match self.body_motion_contact_frame(projected,initial_height,yaw) {
            Ok(frame)=>frame,Err(SurfaceError::MissingSupport)=>return self.estimated_body_motion(from,xy,yaw,slot),
            Err(error)=>return Err(error),
        };
        let base_rotation=rotation(base_attitude)?;
        let next_rotation=rotation(next_attitude)?;
        // 同一真实接触拟合场同时给出高度和法向，前足进入高地面时即可连续适应。
        // 原机体相对于地形的高度与倾斜偏差由差分传播保留。
        let centre_height_delta=next_height-base_height;
        // 同一地形帧上的恒等传播保留完整实际姿态，不进行矩阵往返舍入。
        let q=if base_rotation==next_rotation && yaw==current_yaw {
            from.1
        } else {
            // 将实际姿态在起点地面坐标中的偏差传到下一处地面，不能重置成拟合姿态。
            let mut columns = actual_rotation.map(|axis| {
                world(next_rotation,[0.0;3],base_rotation.map(|basis| dot(basis,axis)))
            });
            // 偏航约束前向轴的水平投影；绕世界竖轴修正会再次转走已搬运的法向。
            // 在保持机体法向的平面内重建正交基，避免转向细分积累倾斜误差。
            let normal=columns[2];
            let (sine,cosine)=yaw.sin_cos();
            let forward=[cosine*normal[2],sine*normal[2],-cosine*normal[0]-sine*normal[1]];
            let length=dot(forward,forward).sqrt();
            columns[0]=forward.map(|v|v/length);
            columns[1]=cross(normal,columns[0]);
            quaternion_from_columns(columns)
        };
        let offset = rotated_support_offset(q,model.body_to_support_z_m)?;
        let previous_offset=rotated_support_offset(from.1,model.body_to_support_z_m)?;
        // 拟合高度位于机体中心XY，真正标定参考位于旋转后的支撑点XY。
        // 先求两个接触修正的差，再叠加原Z，保持同一平面上的浮点恒等。
        let plane_at_offset=|r:[[f64;3];3],v:[f64;3]|-(r[2][0]*v[0]+r[2][1]*v[1])/r[2][2];
        let before_contact=plane_at_offset(base_rotation,previous_offset)-previous_offset[2];
        let after_contact=plane_at_offset(next_rotation,offset)-offset[2];
        let height_delta=centre_height_delta+(after_contact-before_contact);
        // 只改变朝向时没有地面平移，实际机体中心不应因标定偏移自行升降。
        let body_z=if xy==[from.0[0],from.0[1]] {from.0[2]} else {from.0[2]+height_delta};
        // Observed terrain provides the response, never a second connected-floor
        // admission. The actual swept body still checks known terrain, occupancy,
        // recent returns and forbidden geometry between the two poses.
        let body = [xy[0],xy[1],body_z];
        #[cfg(feature = "body-query-profile")]
        let validation_profile = query_profile::guard(QueryStage::FinalBodyValidation);
        {
            #[cfg(feature = "body-query-profile")]
            let _profile = query_profile::guard(QueryStage::ProjectedKnownTerrain);
            self.validate_known_terrain_motion(from,(body,q))?;
        }
        self.validate_body_pose_with_witness(body,q,slot)?;
        #[cfg(feature = "body-query-profile")]
        drop(validation_profile);
        Ok((body,q))
    }

    fn project_body_pose(&self,xy:[f64;2],yaw:f64,previous_support:[f64;3])
        -> Result<([f64;3],[f64;4]),SurfaceError> {
        if !xy.into_iter().chain([yaw]).all(f64::is_finite) {
            return Err(SurfaceError::InvalidPose);
        }
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let projection=self.sample_segment_support(previous_support,[xy[0],xy[1],previous_support[2]],1.0)?.support_xyz;
        if let Some((height,q))=self.measured_contact_frame(xy,projection[2],yaw) {
            let r=rotation(q)?;
            let offset=rotated_support_offset(q,model.body_to_support_z_m)?;
            // Keep the existing connectivity/boundary proof, but never replace
            // the measured plane with the legacy quantized support top afterward.
            self.sample_segment_support(previous_support,
                [xy[0]+offset[0],xy[1]+offset[1],projection[2]],1.0)?;
            let body=[xy[0],xy[1],height+model.body_to_support_z_m/r[2][2]];
            self.validate_body_pose(body,q)?;
            return Ok((body,q));
        }
        // 姿态在机体水平投影处估计一次，避免支撑偏移反复改变体素和拟合邻域。
        let q=self.body_orientation_at(projection,yaw)?;
        let offset=rotated_support_offset(q,model.body_to_support_z_m)?;
        let support=self.sample_segment_support(previous_support,[xy[0]+offset[0],xy[1]+offset[1],projection[2]],1.0)?.support_xyz;
        let p = self.body_position_from_support(support, q, model.body_to_support_z_m)?;
        self.validate_body_pose(p, q)?;
        Ok((p, q))
    }

    /// 逐段检查已知地形及完整包络扫掠；未知地面不否决运动，零平移仍检查旋转。
    pub fn validate_body_motion(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        boundary: &[[f64; 2]],
    ) -> Result<(), SurfaceError> {
        self.validate_body_motion_with_witness(from,to,boundary,&mut None)
    }

    /// Same swept samples; a cache hit restores the originally rejected sample.
    pub fn validate_body_motion_with_witness(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),
        boundary:&[[f64;2]],slot:&mut Option<BodyFailureWitness>)->Result<(),SurfaceError> {
        *slot=None;
        // 观察年龄与外部边界不属于纯几何键，仍沿各自的完整验证路径处理。
        if !boundary.is_empty() {
            #[cfg(feature = "body-query-profile")]
            query_profile::count(QueryCounter::MotionCacheBoundaryBypasses);
            return self.validate_body_motion_checked_with_witness(from,to,boundary,|_|Ok(()),slot);
        }
        let key=[from.0[0],from.0[1],from.0[2],from.1[0],from.1[1],from.1[2],from.1[3],
            to.0[0],to.0[1],to.0[2],to.1[0],to.1[1],to.1[2],to.1[3]].map(f64::to_bits);
        if let Some(result)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner()).motions.get(&key).cloned() {
            #[cfg(feature = "body-query-profile")]
            query_profile::count(QueryCounter::MotionCacheHits);
            return result.result(slot);
        }
        #[cfg(feature = "body-query-profile")]
        query_profile::count(QueryCounter::MotionCacheMisses);
        let result=self.validate_body_motion_checked_with_witness(from,to,boundary,|_|Ok(()),slot);
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.motions.len()>=8192 {
            #[cfg(feature = "body-query-profile")]
            query_profile::count(QueryCounter::MotionCacheCapacityClears);
            cache.motions.clear();
        }
        cache.motions.insert(key,CachedBodyResult::new(result.clone(),*slot));
        result
    }

    /// 当前实际占据的精细支撑位置；只用于本次控制检查的足下豁免。
    #[cfg(test)]
    pub fn body_observation_footprint(
        &self, pose: ([f64; 3], [f64; 4]),
    ) -> Result<std::collections::BTreeSet<[i32; 3]>, SurfaceError> {
        self.build_body_observation_footprint(pose)
    }

    // One geometry implementation for private Vec and the public set. Public
    // collection conversion remains inside the complete footprint profile.
    #[cfg(test)]
    fn build_body_observation_footprint<C: FromIterator<[i32; 3]>>(
        &self, pose: ([f64; 3], [f64; 4]),
    ) -> Result<C, SurfaceError> {
        #[cfg(feature = "body-query-profile")] let _footprint_profile=query_profile::guard(QueryStage::ObservationFootprint);
        let observations = self.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        if observations.size_m.to_bits() != self.source_voxel_size_m().to_bits() {
            return Err(SurfaceError::InvalidConfig("观察精度与几何来源不一致".into()));
        }
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let centre = {
            #[cfg(feature = "body-query-profile")] let _support_profile=query_profile::guard(QueryStage::ObservationFootprintSupport);
            self.locate_support_from_body(pose.0, pose.1, model.body_to_support_z_m)?
        };
        #[cfg(feature = "body-query-profile")] let shape_profile=query_profile::guard(QueryStage::ObservationFootprintShape);
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
        #[cfg(feature = "body-query-profile")] drop(shape_profile);
        let patch = {
            #[cfg(feature = "body-query-profile")] let _patch_profile=query_profile::guard(QueryStage::ObservationFootprintPatch);
            self.ground_patch(centre, radius)
        };
        #[cfg(feature = "body-query-profile")] let cells_profile=query_profile::guard(QueryStage::ObservationFootprintCells);
        let polygon_query=PolygonQuery::new(&polygon);
        let mut observation_lookup=crate::surface::observations::ObservationBlockLookup::new(observations);
        #[cfg(feature = "body-query-profile")] let mut height_counts=ObservationHeightCounts::default();
        let mut cells = Vec::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                if !polygon_query.intersects(x, y, size) { continue; }
                let column = (((x as f64 + 0.5) * size / planner_size).floor() as i32,
                    ((y as f64 + 0.5) * size / planner_size).floor() as i32);
                let ground = *patch.get(&column).ok_or(SurfaceError::MissingSupport)?;
                let height = self.ground_cell_position(ground)[2];
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
                #[cfg(feature = "body-query-profile")] {height_counts.columns+=1;}
                let z = (bottom..=top).rev().find(|&z| {
                    #[cfg(feature = "body-query-profile")] {height_counts.probes+=1;}
                    if observation_lookup.get([x,y,z]).is_none() {
                        #[cfg(feature = "body-query-profile")] {height_counts.absent+=1;}
                        return false;
                    }
                    let centre = (z as f64 + 0.5) * size;
                    let coarse_z = (centre / planner_size).floor() as i32;
                    self.data.occupied_columns.get(&column)
                        .is_some_and(|zs| zs.binary_search(&coarse_z).is_ok())
                }).unwrap_or(top);
                // Ascending x/y and one selected z per XY are strictly XYZ ordered.
                cells.push([x, y, z]);
            }
        }
        drop(observation_lookup);
        #[cfg(feature = "body-query-profile")] drop(height_counts);
        drop(polygon_query);
        #[cfg(feature = "body-query-profile")] {
            drop(cells_profile);
            query_profile::add(QueryCounter::ObservationFootprintCellsBuilt,cells.len() as u64);
        }
        Ok(cells.into_iter().collect())
    }

    /// 验证完整扫掠中新进入位置的实际观察；静态远端路线无需新鲜证据。
    #[cfg(test)]
    pub fn validate_observed_body_motion(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        occupied: &std::collections::BTreeSet<[i32; 3]>,
        now_ns: u64,
        max_age_ns: u64,
    ) -> Result<(), SurfaceError> {
        self.validate_observed_body_motion_in_boundary(from,to,occupied,now_ns,max_age_ns,&[])
    }

    /// 近场轨迹同时沿用原任务边界和实测观察，一次完成相同运动的几何核对。
    #[cfg(test)]
    pub fn validate_observed_body_motion_in_boundary(
        &self, from: ([f64;3],[f64;4]), to: ([f64;3],[f64;4]),
        occupied: &std::collections::BTreeSet<[i32;3]>, now_ns:u64, max_age_ns:u64,
        boundary:&[[f64;2]],
    ) -> Result<(),SurfaceError> {
        // Compatibility signature only: terrain age/hits are not motion permits.
        // Stream/pose health belongs to the runtime, not per-ground-cell evidence.
        let _=(occupied,now_ns,max_age_ns);
        self.validate_body_motion_with_witness(from,to,boundary,&mut None)
    }

    /// 本次近场检查复用一个精确足迹，仍逐次核对完整扫掠与当前检查时刻。
    /// 缓存仅保存足迹单元几何，不保存新鲜度、观察许可或失败结果。
    /// 返回值借用原视图、足下豁免和边界，不能跨观察源或机体视图复用许可。
    #[cfg(test)]
    pub fn observed_body_motion_checker<'a>(
        &'a self,
        occupied: &'a std::collections::BTreeSet<[i32; 3]>,
        max_age_ns: u64,
        boundary: &'a [[f64; 2]],
    ) -> impl FnMut(([f64; 3], [f64; 4]), ([f64; 3], [f64; 4]), u64)
        -> Result<(), SurfaceError> + 'a {
        let mut checker = self.observed_body_motion_checker_with_expiry(occupied, max_age_ns, boundary);
        move |from, to, checked_ns| checker(from, to, checked_ns).map(|_| ())
    }

    /// Compatibility adapter for callers migrating away from per-cell expiry.
    /// Success always returns None: there is no terrain observation time lease.
    /// Actual swept geometry is checked through the unified motion checker.
    #[cfg(test)]
    pub fn observed_body_motion_checker_with_expiry<'a>(
        &'a self,
        occupied: &'a std::collections::BTreeSet<[i32; 3]>,
        max_age_ns: u64,
        boundary: &'a [[f64; 2]],
    ) -> impl FnMut(([f64; 3], [f64; 4]), ([f64; 3], [f64; 4]), u64)
        -> Result<Option<u64>, SurfaceError> + 'a {
        let mut check=self.observed_body_motion_checker_with_expiry_and_witness(occupied,max_age_ns,boundary);
        move |from,to,checked_ns|check(from,to,checked_ns,&mut None)
    }

    /// Compatibility expiry signature; only geometry is checked and no lease is issued.
    #[cfg(test)]
    pub fn observed_body_motion_checker_with_expiry_and_witness<'a>(
        &'a self, occupied:&'a std::collections::BTreeSet<[i32;3]>, max_age_ns:u64,boundary:&'a [[f64;2]],
    )->impl FnMut(([f64;3],[f64;4]),([f64;3],[f64;4]),u64,&mut Option<BodyFailureWitness>)
        ->Result<Option<u64>,SurfaceError>+'a {
        let _=(occupied,max_age_ns);
        move |from,to,_checked_ns,slot| {
            self.validate_body_motion_with_witness(from,to,boundary,slot).map(|_|None)
        }
    }

    /// Geometry-only motion checking. This always executes the same actual pose,
    /// rotation, contact-event and boundary checks; it never grants a time lease.
    pub fn geometry_body_motion_checker_with_witness<'a>(&'a self,boundary:&'a [[f64;2]])
        ->impl FnMut(([f64;3],[f64;4]),([f64;3],[f64;4]),&mut Option<BodyFailureWitness>)
            ->Result<(),SurfaceError>+'a {
        move |from,to,slot|self.validate_body_motion_with_witness(from,to,boundary,slot)
    }

    // 原共同占据带证明保持不变；新离散踏面证明不能扩大它的高度区间。
    fn known_common_band_samples(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]))->Option<Vec<f64>> {
        #[cfg(feature = "body-query-profile")]
        let _profile = query_profile::guard(QueryStage::KnownCommonBand);
        let observations=self.observations.as_ref()?;
        if observations.size_m>=self.config().voxel_size {return None;}
        let size=f64::from(observations.size_m);let coarse=f64::from(self.config().voxel_size);
        let model=self.body_envelope()?;
        let occupied=|key:[i32;3]|observations.get(key).is_some();
        let band=|x:i32,y:i32,height:f64|->Option<(i32,i32)> {
            let z=(height/size).floor() as i32;
            let seed=if occupied([x,y,z]) {z} else {let previous=z.checked_sub(1)?;
                if occupied([x,y,previous]) {previous}else{return None;}};
            let mut lower=seed;let mut top=seed;
            while lower>i32::MIN && occupied([x,y,lower-1]) {lower-=1;}
            while top<i32::MAX && occupied([x,y,top+1]) {top+=1;}
            let above=top.checked_add(1)?;
            // 清除自体不等于原上邻为空，不能据此制造顶面并放宽阶差。
            if observations.get([x,y,above]).is_some() {return None;}
            let key=(((f64::from(x)+0.5)*size/coarse).floor() as i32,
                ((f64::from(y)+0.5)*size/coarse).floor() as i32);
            let complete=self.complete_observations_cover_column(key.0,key.1);
            if observations.is_complete_geometry_source() && !complete {return None;}
            if !complete && !observations.has_complete_block([x,y,above]) {
                // 部分源只能沿原已知露顶证明，不能把没有提交的块当作完整空白。

                if !self.data.physical_graph.surface_lookup.get(&key).is_some_and(|levels|levels.iter().any(|&z|
                    (self.ground_cell_position((key.0,key.1,z))[2]-f64::from(above)*size).abs()<=1e-6)) {return None;}
            }
            Some((lower,top))
        };
        let permitted=|a:(i32,i32),b:(i32,i32)| {
            let rise=(f64::from(b.1)-f64::from(a.1))*size;
            if rise==0.0 {return true;}
            let limit=if rise>0.0 {model.max_step_up_m}else{model.max_step_down_m};
            rise.abs()+2.0*model.support_uncertainty_m<=limit+1e-6
                && rise.abs()<=f64::from(self.config().step_threshold_m)+1e-6
        };
        let offset=model.body_to_support_z_m;
        let start=self.support_position_from_body(from.0,from.1,offset).ok()?;
        let end=self.support_position_from_body(to.0,to.1,offset).ok()?;
        let a=band((start[0]/size).floor() as i32,(start[1]/size).floor() as i32,start[2])?;
        let b=band((end[0]/size).floor() as i32,(end[1]/size).floor() as i32,end[2])?;
        let mut low=a.0.max(b.0);let mut high=a.1.min(b.1);
        let mut minimum=start[2].min(end[2]);let mut maximum=start[2].max(end[2]);
        // 真实参考可比露顶高一个源体素，与locate_support的量化口径一致。
        // 仍对整段的共同占据带只计一次，不能逐列累加或跨越不相交层。
        if low>high || minimum<f64::from(low)*size-1e-6 || maximum>(f64::from(high)+2.0)*size+1e-6 {return None;}
        let samples=self.body_support_grid_fractions(from,to,size).ok()?;
        let mut previous:Option<(i32,i32,(i32,i32))>=None;
        for &t in &samples {
            let (p,q)=interpolated_body_pose(from,to,t);
            let support=self.support_position_from_body(p,q,offset).ok()?;
            let x=(support[0]/size).floor() as i32;let y=(support[1]/size).floor() as i32;
            let current=band(x,y,support[2])?;
            low=low.max(current.0);high=high.min(current.1);
            minimum=minimum.min(support[2]);maximum=maximum.max(support[2]);
            if let Some((px,py,previous_band))=previous {
                if (i64::from(x)-i64::from(px)).abs()>1 || (i64::from(y)-i64::from(py)).abs()>1 {return None;}
                if !permitted(previous_band,current) {return None;}
                if x!=px && y!=py {
                    for (sx,sy) in [(px,y),(x,py)] {
                        let side=band(sx,sy,support[2])?;
                        if !permitted(previous_band,side) || !permitted(side,current) {return None;}
                        low=low.max(side.0);high=high.min(side.1);
                    }
                }
            }
            // 所有实际参考及对角侧列共同落在已观测带内，不能逐格换层跨空隙。
            if low>high || minimum<f64::from(low)*size-1e-6 || maximum>(f64::from(high)+2.0)*size+1e-6 {return None;}
            previous=Some((x,y,current));
        }
        Some(samples)
    }

    /// 粗剖面跨入下一块地形时，按原始细体素的真实宽度核验已声明的踏面能力。
    /// 只补证支撑参考的连接；调用者仍检查完整机体、接触事件、边界和观察时效。
    /// 不删占据、不生成地面，不把相同粗列里的独立楼层连接起来。
    fn known_tread_geometry_samples(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]))->Option<Vec<f64>> {
        #[cfg(feature = "body-query-profile")]
        let _profile = query_profile::guard(QueryStage::KnownTreadGeometry);
        let observations=self.observations.as_ref()?;
        if !observations.is_complete_geometry_source()
            || observations.size_m.to_bits()!=self.source_voxel_size_m().to_bits() {return None;}
        let model=self.body_envelope()?;
        let size=f64::from(observations.size_m);
        // 深度按源体素的有证据区间计算，而不是把粗格中心间距当作完整踏面。
        // 与配置中的粗剖面同用f32除法；提升成f64再ceil会把20cm/1cm误算为21格。
        let depth=(self.config().min_step_tread_depth_m/observations.size_m).ceil();
        // 仅限制源坐标可表示范围；粗格剖面的128格上限不能套到细格数量上。
        if !depth.is_finite() || depth<1.0 || depth>=i32::MAX as f32 {return None;}
        let depth=depth as i32;
        let occupied=|key:[i32;3]|observations.get(key).is_some();
        let exposed=|key:[i32;3]| {
            occupied(key) && key[2].checked_add(1).is_some_and(|above|
                !observations.get([key[0],key[1],above]).is_some())
        };
        let select=|pose:([f64;3],[f64;4])|->Option<[i32;3]> {
            let support=self.locate_support_from_body(pose.0,pose.1,model.body_to_support_z_m).ok()?;
            let reference=self.support_position_from_body(pose.0,pose.1,model.body_to_support_z_m).ok()?;
            let xy=[(reference[0]/size).floor() as i32,(reference[1]/size).floor() as i32];
            // 粗顶可能来自同格另一半；限定已选支撑带，在当前细列取实际最高顶。
            // 按有序列索引二分，不扫描别的XY或任意楼层，也不以清理结果降低顶面。
            let lower=self.observed_support_lower_bound(support.cell);
            let levels=observations.column_levels(xy[0],xy[1])?;
            let end=levels.partition_point(|&z|(f64::from(z)+1.0)*size<=support.surface_xyz[2]+1e-6);
            let z=*levels.get(end.checked_sub(1)?)?;
            if f64::from(z)*size<lower-1e-6 {return None;}
            let key=[xy[0],xy[1],z];
            exposed(key).then_some(key)
        };
        let tread=|key:[i32;3],direction:[i32;2]|->bool {
            (0..depth).all(|step| {
                let Some(x)=direction[0].checked_mul(step).and_then(|v|key[0].checked_add(v)) else{return false;};
                let Some(y)=direction[1].checked_mul(step).and_then(|v|key[1].checked_add(v)) else{return false;};
                exposed([x,y,key[2]])
            })
        };
        // 单次查询内只对有高差的唯一有向边扫描踏面；不新增共享缓存或许可寿命。
        let mut checked_steps=std::collections::BTreeSet::new();
        let mut edge=|a:[i32;3],b:[i32;3]|->bool {
            let dx=b[0].abs_diff(a[0]);let dy=b[1].abs_diff(a[1]);
            if dx+dy!=1 {return false;}
            let rise=(f64::from(b[2])-f64::from(a[2]))*size;
            if rise==0.0 || checked_steps.contains(&(a,b)) {return true;}
            let limit=if rise>0.0 {model.max_step_up_m}else{model.max_step_down_m};
            if rise.abs()+2.0*model.support_uncertainty_m>limit+1e-6
                || rise.abs()>f64::from(self.config().step_threshold_m)+1e-6 {return false;}
            let direction=[(b[0]-a[0]).signum(),(b[1]-a[1]).signum()];
            if !tread(a,[-direction[0],-direction[1]]) || !tread(b,direction) {return false;}
            checked_steps.insert((a,b));true
        };
        // 复用原路径网格事件；平段只做选顶，不按采样数乘踏深虚构几何否决门槛。
        let samples=self.body_support_grid_fractions(from,to,size).ok()?;
        let mut previous:Option<[i32;3]>=None;
        for &t in &samples {
            let pose=interpolated_body_pose(from,to,t);
            let current=select(pose)?;
            if let Some(a)=previous {
                if a==current {continue;}
                let dx=current[0].abs_diff(a[0]);let dy=current[1].abs_diff(a[1]);
                if dx>1 || dy>1 || dx+dy==0 {return None;}
                if dx+dy==1 {
                    if !edge(a,current) {return None;}
                } else {
                    // 对角必须两侧均有同层的真实连接，不能切过坑或独立层。
                    for xy in [[a[0],current[1]],[current[0],a[1]]] {
                        // 只允许端点支撑层，不经第三高度拆分一次过高的对角阶差。
                        let side=[a[2],current[2]].into_iter().map(|z|[xy[0],xy[1],z])
                            .any(|side|exposed(side)&&edge(a,side)&&edge(side,current));
                        if !side {return None;}
                    }
                }
            }
            previous=Some(current);
        }
        Some(samples)
    }

    /// Reject contradictions in observed terrain, rather than asking every
    /// crossed column to prove support. Missing columns and tread samples remain
    /// unknown; actual occupied geometry is checked independently by the sweep.
    fn validate_known_terrain_motion(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]))
        ->Result<(),SurfaceError> {
        #[cfg(feature = "body-query-profile")]
        let _profile = query_profile::guard(QueryStage::KnownTerrainTotal);
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        // Exact known-band/tread geometry can settle a transition without
        // inventing discrete steps from a thick contact interval or coarse XY.
        // Failure of either observation query is not a rejection: the sparse
        // known-only pass below still allows missing terrain.
        if self.known_common_band_samples(from,to).is_some()
            || self.known_tread_geometry_samples(from,to).is_some() {return Ok(());}
        #[cfg(feature = "body-query-profile")]
        let _fallback_profile = query_profile::guard(QueryStage::KnownOnlyFallback);
        let size=f64::from(self.observations.as_ref().map_or(self.config().voxel_size,|o|o.size_m));
        let samples=self.body_support_grid_fractions(from,to,size)?.into_iter().map(|t| {
            let pose=interpolated_body_pose(from,to,t);
            self.support_position_from_body(pose.0,pose.1,model.body_to_support_z_m)
        }).collect::<Result<Vec<_>,_>>()?;
        self.validate_known_reference_samples(samples,size,model.max_step_up_m,model.max_step_down_m,
            model.support_uncertainty_m)
    }

    /// Check only known step, drop, slope and tread contradictions along a
    /// calibrated support-reference segment. Missing columns remain unknown.
    /// This does not check body volume, recent returns or forbidden geometry.
    /// Without a body model it uses the already configured planar step limits.
    pub fn validate_known_reference_motion(&self,from:[f64;3],to:[f64;3])->Result<(),SurfaceError> {
        let size=f64::from(self.observations.as_ref().map_or(self.config().voxel_size,|o|o.size_m));
        let fractions=translation::reference_grid_fractions(from,to,size)?;
        let samples=fractions.into_iter().map(|t|std::array::from_fn(|axis|
            from[axis]+(to[axis]-from[axis])*t));
        let (up,down,uncertainty)=self.body_envelope().map_or(
            (f64::from(self.config().step_threshold_m),f64::from(self.config().step_threshold_m),0.0),
            |model|(model.max_step_up_m,model.max_step_down_m,model.support_uncertainty_m));
        self.validate_known_reference_samples(samples,size,up,down,uncertainty)
    }

    fn validate_known_reference_samples(&self,samples:impl IntoIterator<Item=[f64;3]>,size:f64,
        step_up:f64,step_down:f64,uncertainty:f64)->Result<(),SurfaceError> {
        let depth=(f64::from(self.config().min_step_tread_depth_m)/size-1e-6).ceil().max(1.0) as usize;
        let sample=|xy:[i32;2],reference:f64|self.known_terrain_transition_top(
            xy.map(|v|(f64::from(v)+0.5)*size),reference);
        let tread=|xy:[i32;2],height:f64,direction:[i32;2]| {
            (1..depth).all(|step| {
                let point=[xy[0]+direction[0]*step as i32,xy[1]+direction[1]*step as i32];
                sample(point,height).is_none_or(|z|(z-height).abs()<=1e-6)
            })
        };
        let edge=|a:([i32;2],f64),b:([i32;2],f64)| {
            let rise=b.1-a.1;
            if rise.abs()<=1e-6 {return true;}
            let dx=b.0[0]-a.0[0];let dy=b.0[1]-a.0[1];
            let run=f64::from(dx).hypot(f64::from(dy))*size;
            if run>0.0 && rise.abs()<=run*f64::from(self.config().max_slope_deg).to_radians().tan()+1e-6 {
                return true;
            }
            let limit=if rise>0.0 {step_up}else{step_down};
            if rise.abs()+2.0*uncertainty>limit+1e-6
                || rise.abs()>f64::from(self.config().step_threshold_m)+1e-6 {return false;}
            if dx==0&&dy==0 {return false;}
            let direction=[dx.signum(),dy.signum()];
            tread(a.0,a.1,[-direction[0],-direction[1]]) && tread(b.0,b.1,direction)
        };
        let mut previous=None;
        for support in samples {
            let xy=[(support[0]/size).floor() as i32,(support[1]/size).floor() as i32];
            let Some(height)=self.known_terrain_transition_top([support[0],support[1]],support[2]) else {
                previous=None;continue;
            };
            let current=(xy,height);
            if let Some(a)=previous {
                if !edge(a,current) {return Err(SurfaceError::UntraversableSegment);}
                if a.0[0]!=xy[0] && a.0[1]!=xy[1] {
                    for side in [[a.0[0],xy[1]],[xy[0],a.0[1]]] {
                        if let Some(z)=sample(side,a.1) {
                            if !edge(a,(side,z)) || !edge((side,z),current) {
                                return Err(SurfaceError::UntraversableSegment);
                            }
                        }
                    }
                }
            }
            previous=Some(current);
        }
        Ok(())
    }

    #[cfg(test)]
    fn validate_body_motion_checked(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),boundary:&[[f64;2]],
        check:impl FnMut(([f64;3],[f64;4]))->Result<(),SurfaceError>)->Result<(),SurfaceError> {
        self.validate_body_motion_checked_with_witness(from,to,boundary,check,&mut None)
    }

    fn validate_body_motion_checked_with_witness(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        boundary: &[[f64; 2]],
        mut check_observations: impl FnMut(([f64; 3], [f64; 4])) -> Result<(), SurfaceError>,
        slot:&mut Option<BodyFailureWitness>,
    ) -> Result<(), SurfaceError> {
        *slot=None;
        validate_boundary(boundary)?;
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        self.validate_body_pose_with_witness(from.0, from.1,slot)?;
        self.validate_body_pose_with_witness(to.0, to.1,slot)?;
        check_observations(from)?;
        check_observations(to)?;
        let a = self.support_position_from_body(from.0, from.1, model.body_to_support_z_m)?;
        let b = self.support_position_from_body(to.0, to.1, model.body_to_support_z_m)?;
        if !segment_inside_boundary([a[0], a[1]], [b[0], b[1]], boundary) {
            return Err(SurfaceError::InvalidBoundary);
        }

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
        // 采样密度按输入机体的真实平移和三维旋转包络确定。
        // 支撑格只证明地形连通，不能把机体重新抬到该格的层顶。
        let distance = (from.0[0] - to.0[0]).hypot(from.0[1] - to.0[1]).hypot(from.0[2] - to.0[2]);
        let rotation_radius = model.radius().hypot(
            model.z_bounds_m.into_iter().map(f64::abs).fold(model.body_to_support_z_m, f64::max),
        );
        let count = ((distance + rotation_radius * angle)
            / (f64::from(self.source_voxel_size_m()) * 0.5))
            .max(angle / 0.05)
            .ceil()
            .max(1.0) as usize;
        if count > 4096 {
            return Err(SurfaceError::InvalidConfig(format!(
                "机体扫掠需要{count}次采样，超过4096次工作上限；未证明几何不可通行"
            )));
        }
        {
            #[cfg(feature = "body-query-profile")]
            let _profile = query_profile::guard(QueryStage::SweptKnownTerrain);
            self.validate_known_terrain_motion(from,to)?;
        }
        for i in 1..count {
            let t = i as f64 / count as f64;
            let q = norm(std::array::from_fn(|j| qa[j] * (1.0 - t) + qb[j] * t));
            let p = std::array::from_fn(|j| from.0[j] * (1.0 - t) + to.0[j] * t);
            self.validate_body_pose_with_witness(p, q,slot)?;
            check_observations((p, q))?;
        }
        // 按真实占据接触事件补查，并保持调用方的实际位置与姿态插值。
        self.validate_body_contact_events(from,to,|t| {
            let p = std::array::from_fn(|j| if from.0[j]==to.0[j] {from.0[j]} else {
                from.0[j]*(1.0-t)+to.0[j]*t
            });
            let q=norm(std::array::from_fn(|j|qa[j]*(1.0-t)+qb[j]*t));
            self.validate_body_pose_with_witness(p,q,slot)?;
            check_observations((p,q))
        })
    }

    /// 从同一可达支撑邻域估计坡面姿态，位置仍由真实支撑提供。
    pub fn body_orientation_at(
        &self,
        support: [f64; 3],
        yaw: f64,
    ) -> Result<[f64; 4], SurfaceError> {
        self.body_envelope().ok_or_else(|| SurfaceError::InvalidConfig("body envelope is unavailable".into()))?;
        let centre = self.locate_support(support)?;
        self.body_orientation_on_contact(centre,support[2],yaw)
    }

    fn body_orientation_on_contact(
        &self,
        centre: SurfaceLocation,
        height: f64,
        yaw: f64,
    ) -> Result<[f64; 4], SurfaceError> {
        self.body_contact_frame(centre,height,yaw).map(|(_,q)|q)
    }

    /// 无精细观察时保留原粗格中心及其真实连续高度区间。
    fn ground_contact_samples(&self,cell:VoxelKey)->Arc<Vec<([f64;3],f64)>> {
        if let Some(points)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner())
            .contact_samples.get(&cell).cloned() {return points;}
        let coarse=self.ground_cell_position(cell);
        let low=if self.ground_evidence==GroundEvidence::OccupiedVolume {coarse[2]}else{
            (self.observed_support_lower_bound(cell)+f64::from(self.source_voxel_size_m())).min(coarse[2])};
        let points=Arc::new(vec![(coarse,low)]);
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.contact_samples.len()>=8192 {cache.contact_samples.clear();}
        cache.contact_samples.insert(cell,Arc::clone(&points));
        points
    }

    /// 原始细列的连续占据带；私有机体遮罩不切开真实带，也不制造新露顶。
    fn fine_contact_bands(&self,x:i32,y:i32)->Arc<Vec<FineContactBand>> {
        if let Some(bands)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner())
            .fine_bands.get(&(x,y)).cloned() {return bands;}
        let bands=self.read_fine_contact_bands(x,y);
        let bands=Arc::new(bands);
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.fine_bands.len()>=8192 {cache.fine_bands.clear();}
        cache.fine_bands.insert((x,y),Arc::clone(&bands));bands
    }

    fn read_fine_contact_bands(&self,x:i32,y:i32)->Vec<FineContactBand> {
        let mut bands=Vec::new();
        if let Some(observations)=&self.observations {
            let size=f64::from(observations.size_m);
            let mut run:Option<(i32,i32)>=None;
            for &z in observations.column_levels(x,y).unwrap_or(&[]) {
                if !observations.ground_member([x,y,z]) && !observations.get([x,y,z]).is_some() {continue;}
                if let Some((bottom,top))=run {
                    if top.checked_add(1)==Some(z) {run=Some((bottom,z));continue;}
                    let layer=GroundLayer::from_voxel_run([bottom,top],size,self.ground_evidence)
                        .expect("ordered source run and positive resolution");
                    bands.push(FineContactBand{fit:layer.contact_interval,run:[bottom,top]});
                }
                run=Some((z,z));
            }
            if let Some((bottom,top))=run {
                let layer=GroundLayer::from_voxel_run([bottom,top],size,self.ground_evidence)
                        .expect("ordered source run and positive resolution");
                    bands.push(FineContactBand{fit:layer.contact_interval,run:[bottom,top]});
            }
        }
        if self.ground_evidence==GroundEvidence::MeasuredSurface {
            if let Some(observations)=&self.observations {
                let quantum=f64::from(observations.size_m);
                let records:Vec<_>=observations.ground_layers([x,y]).collect();
                if !records.is_empty() {
                    // A measured physical layer replaces only its associated
                    // compatibility band in the contact response, never occupancy.
                    bands.retain(|band|!records.iter().any(|record|record.members.iter()
                        .any(|z|band.run[0]<=*z && *z<=band.run[1])));
                    let xy=[(f64::from(x)+0.5)*quantum,(f64::from(y)+0.5)*quantum];
                    for record in records {
                        if let Some(z)=record.height_at(xy) {
                            bands.push(FineContactBand {fit:[z,z],
                                run:[record.members[0],*record.members.last().expect("validated nonempty layer")]});
                        }
                    }
                    bands.sort_by(|a,b|a.fit[1].total_cmp(&b.fit[1]));
                }
            }
        }
        bands
    }

    fn fine_contact_domain(&self,origin:[f64;3],radius:f64)
        ->Result<Vec<([f64;3],f64,f64)>,SurfaceError> {
        let observations=self.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let size=f64::from(observations.size_m);
        let enumeration=radius+std::f64::consts::SQRT_2*size;
        let low:[i32;2]=std::array::from_fn(|i|((origin[i]-enumeration)/size).floor() as i32);
        let high:[i32;2]=std::array::from_fn(|i|((origin[i]+enumeration)/size).floor() as i32);
        let nx=i64::from(high[0])-i64::from(low[0])+1;
        let ny=i64::from(high[1])-i64::from(low[1])+1;
        // 仅限制局部稠密临时表的容量；特殊大域继续原稀疏算法，不改变许可或拟合域。
        let Some(count)=nx.checked_mul(ny).filter(|n|nx>0&&ny>0&&*n<=16_384) else {
            return self.fine_contact_domain_sparse(origin,radius);
        };
        #[cfg(feature = "body-query-profile")]
        query_profile::count(QueryCounter::DenseDomainCalls);
        let stride=ny as usize;let count=count as usize;
        let mut in_domain=vec![false;count];
        let mut bands=vec![None;count];
        let mut selected=vec![None;count];
        // 子阶段不移动生产临时值：表/队列初始化及保留缓冲清理仍只计入外层收集。
        #[cfg(feature = "body-query-profile")]
        let (mut cache_present_columns,mut cache_absent_columns,mut cache_single_band_columns)=(0_u64,0_u64,0_u64);
        #[cfg(feature = "body-query-profile")]
        let cache_profile=query_profile::guard(QueryStage::DenseBandCacheRead);
        // 同一不可变来源的带缓存一次批量读取，避免每个实测细列分别取锁。
        {
            #[cfg(feature = "body-query-profile")]
            let acquire_profile=query_profile::guard(QueryStage::DenseBandCacheAcquire);
            let cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
            #[cfg(feature = "body-query-profile")]
            drop(acquire_profile);
            for x in low[0]..=high[0] {for y in low[1]..=high[1] {
                let dx=(f64::from(x)+0.5)*size-origin[0];let dy=(f64::from(y)+0.5)*size-origin[1];
                if dx*dx+dy*dy>enumeration*enumeration {continue;}
                let index=(i64::from(x)-i64::from(low[0])) as usize*stride+(i64::from(y)-i64::from(low[1])) as usize;
                in_domain[index]=true;
                if let Some(values)=cache.fine_bands.get(&(x,y)) {
                    // 单带的选择与参考高度无关，直接写入原有选择表，省去临时 Arc 增减。
                    // 空带与多带仍保留原缓存读取/选择路径，不跨观察拥有方复用结果。
                    if values.len()==1 {selected[index]=Some(values[0].fit);}
                    else {bands[index]=Some(Arc::clone(values));}
                    #[cfg(feature = "body-query-profile")]
                    {
                        cache_present_columns+=1;
                        if values.len()==1 {cache_single_band_columns+=1;}
                    }
                } else {
                    #[cfg(feature = "body-query-profile")]
                    {cache_absent_columns+=1;}
                }
            }}
        }
        #[cfg(feature = "body-query-profile")]
        {
            drop(cache_profile);
            query_profile::add(QueryCounter::DenseAllocatedSlots,count as u64);
            query_profile::add(QueryCounter::DenseCachePresentColumns,cache_present_columns);
            query_profile::add(QueryCounter::DenseCacheAbsentColumns,cache_absent_columns);
            query_profile::add(QueryCounter::DenseCacheSingleBandColumns,cache_single_band_columns);
        }
        let distance=|band:[f64;2]|(band[0]-origin[2]).max(origin[2]-band[1]).max(0.0);
        let mut seed:Option<(usize,(i32,i32),[f64;4])>=None;
        #[cfg(feature = "body-query-profile")]
        let mut selected_columns=0_u64;
        #[cfg(feature = "body-query-profile")]
        let selection_profile=query_profile::guard(QueryStage::DenseBandSelectionSeed);
        for (index,inside) in in_domain.iter().copied().enumerate() {
            if !inside {continue;}
            let x=low[0]+(index/stride) as i32;let y=low[1]+(index%stride) as i32;
            let band=if let Some(band)=selected[index] {band} else {
                let values=bands[index].take().unwrap_or_else(||self.fine_contact_bands(x,y));
                let Some(&band)=values.iter().map(|entry|&entry.fit).min_by(|a,b|distance(**a).total_cmp(&distance(**b))
                    .then_with(||(a[1]-origin[2]).abs().total_cmp(&(b[1]-origin[2]).abs()))
                    .then_with(||a[1].total_cmp(&b[1]))) else {continue;};
                band
            };
            selected[index]=Some(band);let rise=band[1]-origin[2];
            #[cfg(feature = "body-query-profile")]
            {selected_columns+=1;}
            if rise>model.max_step_up_m+size+1e-6 || -rise>model.max_step_down_m+size+1e-6 {continue;}
            let dx=(f64::from(x)+0.5)*size-origin[0];let dy=(f64::from(y)+0.5)*size-origin[1];
            let rank=[dx*dx+dy*dy,distance(band),rise.abs(),band[1]];
            let earlier=seed.as_ref().is_none_or(|&(_,old,previous)| {
                rank.iter().zip(previous).map(|(a,b)|a.total_cmp(&b)).find(|v|!v.is_eq())
                    .unwrap_or_else(||(x,y).cmp(&old)).is_lt()
            });
            if earlier {seed=Some((index,(x,y),rank));}
        }
        #[cfg(feature = "body-query-profile")]
        {
            drop(selection_profile);
            query_profile::add(QueryCounter::DenseSelectedColumns,selected_columns);
        }
        let seed=seed.ok_or(SurfaceError::MissingSupport)?.0;
        let mut queue=VecDeque::from([seed]);let mut visited=vec![false;count];visited[seed]=true;
        #[cfg(feature = "body-query-profile")]
        query_profile::add(QueryCounter::DenseVisitedTableSlots,count as u64);
        #[cfg(feature = "body-query-profile")]
        let mut visited_columns=0_u64;
        #[cfg(feature = "body-query-profile")]
        let connectivity_profile=query_profile::guard(QueryStage::DenseConnectivityBfs);
        while let Some(index)=queue.pop_front() {
            #[cfg(feature = "body-query-profile")]
            {visited_columns+=1;}
            let row=index/stride;let column=index%stride;
            let band=selected[index].expect("只有已选真实带进入队列");
            for dx in -1_isize..=1 {for dy in -1_isize..=1 {
                if dx==0&&dy==0 {continue;}
                let rows=match dx {-1=>row,0=>usize::MAX,_=>nx as usize-1-row};
                let columns=match dy {-1=>column,0=>usize::MAX,_=>stride-1-column};
                // 到矩形边界的步数恰好保留原 checked_add/边界检查允许的格。
                // 每步仍为 (row+dx*k)*stride+column+dy*k，不会绕到邻行；
                // 局部表最多 16_384 格，有符号索引在 i32 极值附近也不做世界坐标加法。
                let offset=dx*stride as isize+dy;let mut next_index=index as isize;
                for _ in 0..rows.min(columns) {
                    next_index+=offset;let next_index=next_index as usize;
                    if !in_domain[next_index] {break;}
                    let Some(next)=selected[next_index] else {continue;};
                    if !visited[next_index] {
                        let rise=next[1]-band[1];
                        if rise<=model.max_step_up_m+size+1e-6 && -rise<=model.max_step_down_m+size+1e-6
                            {visited[next_index]=true;queue.push_back(next_index);}
                    }
                    // 第一真实列始终终止射线；已访问或高度不相容也不能跳过。
                    break;
                }
            }}
        }
        #[cfg(feature = "body-query-profile")]
        {
            drop(connectivity_profile);
            query_profile::add(QueryCounter::DenseVisitedColumns,visited_columns);
        }
        // 只改变私有输出顺序：仍仅发出原 BFS 访问过的真实列，点与权重算式不变。
        // 网格按 X 再 Y 排列，让 body_contact_frame 保留的排序直接接收有序输入。
        #[cfg(feature = "body-query-profile")]
        let emission_profile=query_profile::guard(QueryStage::DenseOrderedEmission);
        let mut points=Vec::new();
        for (index,seen) in visited.into_iter().enumerate() {
            if !seen {continue;}
            let x=low[0]+(index/stride) as i32;let y=low[1]+(index%stride) as i32;
            let band=selected[index].expect("只有已选真实带被访问");
            let point=[(f64::from(x)+0.5)*size,(f64::from(y)+0.5)*size,band[1]];
            let distance2=(point[0]-origin[0]).powi(2)+(point[1]-origin[1]).powi(2);
            let weight=(1.0-distance2/(radius*radius)).max(0.0).powi(2);
            if weight>0.0 {points.push((point,band[0],weight));}
        }
        #[cfg(feature = "body-query-profile")]
        drop(emission_profile);
        Ok(points)
    }

    /// 从真实细带独立构建接触响应域；粗图不决定拟合输入，仍核验全部实际机体许可。
    fn fine_contact_domain_sparse(&self,origin:[f64;3],radius:f64)
        ->Result<Vec<([f64;3],f64,f64)>,SurfaceError> {
        #[cfg(feature = "body-query-profile")]
        query_profile::count(QueryCounter::SparseDomainCalls);
        let observations=self.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let size=f64::from(observations.size_m);
        let enumeration=radius+std::f64::consts::SQRT_2*size;
        let low:[i32;2]=std::array::from_fn(|i|((origin[i]-enumeration)/size).floor() as i32);
        let high:[i32;2]=std::array::from_fn(|i|((origin[i]+enumeration)/size).floor() as i32);
        let distance=|band:[f64;2]|(band[0]-origin[2]).max(origin[2]-band[1]).max(0.0);
        let mut selected=AHashMap::<(i32,i32),[f64;2]>::new();
        let mut seed:Option<((i32,i32),[f64;4])>=None;
        for x in low[0]..=high[0] {for y in low[1]..=high[1] {
            let dx=(f64::from(x)+0.5)*size-origin[0];let dy=(f64::from(y)+0.5)*size-origin[1];
            let distance2=dx*dx+dy*dy;
            if distance2>enumeration*enumeration {continue;}
            let bands=self.fine_contact_bands(x,y);
            // 先选完整区间最近的带，不能先删高顶厚带再退到远处薄层。
            let Some(&band)=bands.iter().map(|entry|&entry.fit).min_by(|a,b|distance(**a).total_cmp(&distance(**b))
                .then_with(||(a[1]-origin[2]).abs().total_cmp(&(b[1]-origin[2]).abs()))
                .then_with(||a[1].total_cmp(&b[1]))) else {continue;};
            selected.insert((x,y),band);
            let rise=band[1]-origin[2];
            if rise>model.max_step_up_m+size+1e-6 || -rise>model.max_step_down_m+size+1e-6 {continue;}
            // 先关联当前实际XY附近的接触分量，不能让足迹外的高度偶合孤岛抢走种子。
            let rank=[distance2,distance(band),rise.abs(),band[1]];
            let earlier=seed.as_ref().is_none_or(|&(old,previous)| {
                rank.iter().zip(previous).map(|(a,b)|a.total_cmp(&b)).find(|v|!v.is_eq())
                    .unwrap_or_else(||(x,y).cmp(&old)).is_lt()
            });
            if earlier {seed=Some(((x,y),rank));}
        }}
        let seed=seed.ok_or(SurfaceError::MissingSupport)?.0;
        let mut queue=VecDeque::from([seed]);let mut visited=AHashSet::from_iter([seed]);
        let mut points=Vec::new();
        while let Some((x,y))=queue.pop_front() {
            let band=selected[&(x,y)];
            let point=[(f64::from(x)+0.5)*size,(f64::from(y)+0.5)*size,band[1]];
            let distance2=(point[0]-origin[0]).powi(2)+(point[1]-origin[1]).powi(2);
            let weight=(1.0-distance2/(radius*radius)).max(0.0).powi(2);
            if weight>0.0 {points.push((point,band[0],weight));}
            // 响应估计连接每个方向最近的真实采样列，不要求观测恰好填满细格。
            // 第一真实列即终止探测：不能越过不相容墙带；空格仍没有支撑许可。
            for dx in -1..=1 {for dy in -1..=1 {
                if dx==0 && dy==0 {continue;}
                let mut key=(x,y);
                loop {
                    let (Some(nx),Some(ny))=(key.0.checked_add(dx),key.1.checked_add(dy)) else {break;};
                    key=(nx,ny);
                    if nx<low[0] || nx>high[0] || ny<low[1] || ny>high[1] {break;}
                    let px=(f64::from(nx)+0.5)*size-origin[0];let py=(f64::from(ny)+0.5)*size-origin[1];
                    if px*px+py*py>enumeration*enumeration {break;}
                    let Some(next)=selected.get(&key) else {continue;};
                    let rise=next[1]-band[1];
                    if rise<=model.max_step_up_m+size+1e-6 && -rise<=model.max_step_down_m+size+1e-6
                        && visited.insert(key) {queue.push_back(key);}
                    break;
                }
            }}
        }
        Ok(points)
    }

    // A missing terrain response keeps the calibrated current plane as a local
    // reference. It does not change query_ground, source layers or observed state.
    fn estimated_body_motion(&self,from:([f64;3],[f64;4]),xy:[f64;2],yaw:f64,
        slot:&mut Option<BodyFailureWitness>)->Result<([f64;3],[f64;4]),SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let actual=rotation(from.1)?;
        if actual[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        let reference=from.0[2]-model.body_to_support_z_m/actual[2][2];
        let base=self.query_ground([from.0[0],from.0[1]],reference);
        let height=base.height.unwrap_or(reference);
        let uncertainty=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        if height-reference>model.max_step_up_m+uncertainty+1e-6
            || reference-height>model.max_step_down_m+uncertainty+1e-6 {
            return Err(SurfaceError::UntraversableSegment);
        }
        let next=self.query_ground(xy,height);
        let next_height=next.height.unwrap_or(height);
        let delta=next_height-height;
        if delta>model.max_step_up_m+uncertainty+1e-6 || -delta>model.max_step_down_m+uncertainty+1e-6 {
            return Err(SurfaceError::UntraversableSegment);
        }
        let frame=|normal:[f64;3],yaw:f64| {
            let forward=normalize([yaw.cos(),yaw.sin(),-(normal[0]*yaw.cos()+normal[1]*yaw.sin())/normal[2]]);
            [forward,cross(normal,forward),normal]
        };
        let normal=base.normal.unwrap_or(actual[2]);
        let base_rotation=frame(normal,actual[0][1].atan2(actual[0][0]));
        let next_rotation=frame(next.normal.unwrap_or(normal),yaw);
        let columns=actual.map(|axis|world(next_rotation,[0.0;3],base_rotation.map(|basis|dot(basis,axis))));
        let q=if base_rotation==next_rotation {from.1} else {quaternion_from_columns(columns)};
        let next_pose=([xy[0],xy[1],from.0[2]+delta],q);
        {
            #[cfg(feature = "body-query-profile")]
            let _profile = query_profile::guard(QueryStage::ProjectedKnownTerrain);
            self.validate_known_terrain_motion(from,next_pose)?;
        }
        self.validate_body_pose_with_witness(next_pose.0,next_pose.1,slot)?;
        Ok(next_pose)
    }

    /// Classify only a known height mismatch at the nominal contact position.
    /// This is not a footprint-support test: missing terrain remains Unknown.
    pub fn body_ground_relation(&self,body:[f64;3],quaternion:[f64;4])->Result<BodyGroundRelation,SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let support=self.support_position_from_body(body,quaternion,model.body_to_support_z_m)?;
        let size=f64::from(self.source_voxel_size_m());
        if support.iter().any(|v|!v.is_finite() || (v/size).abs()>=f64::from(i32::MAX)*0.5) {
            return Err(SurfaceError::InvalidPose);
        }
        let Some(height)=self.known_terrain_height([support[0],support[1]],support[2]) else {
            return Ok(BodyGroundRelation::Unknown);
        };
        let delta=height-support[2];
        let rounding=size+model.support_uncertainty_m+1e-6;
        Ok(if delta>model.max_step_up_m+rounding || -delta>model.max_step_down_m+rounding {
            BodyGroundRelation::KnownIncompatible
        } else {BodyGroundRelation::KnownCompatible})
    }

    /// Source-local exposed boundaries for step height and tread depth only.
    /// This is deliberately separate from the representative ground height and
    /// contact fit: a thick riser and its thin tread share an exposed top even
    /// when their legacy interval means differ. Missing fine samples must not
    /// inherit another half-column's coarse top.
    fn known_terrain_transition_top(&self,xy:[f64;2],reference:f64)->Option<f64> {
        let Some(observations)=&self.observations else {
            let size=f64::from(self.config().voxel_size);
            let column=xy.map(|v|(v/size).floor() as i32);
            return self.data.physical_graph.surface_lookup.get(&(column[0],column[1]))
                .into_iter().flatten().map(|&z|self.ground_cell_position((column[0],column[1],z))[2])
                .min_by(|a,b|(a-reference).abs().total_cmp(&(b-reference).abs()));
        };
        let fine=f64::from(observations.size_m);
        let column=xy.map(|v|(v/fine).floor() as i32);
        if self.ground_evidence==GroundEvidence::MeasuredSurface {
            if let Some(layer)=observations.ground_layers(column).min_by(|a,b|
                (a.height_at(xy).unwrap_or(f64::INFINITY)-reference).abs()
                    .total_cmp(&(b.height_at(xy).unwrap_or(f64::INFINITY)-reference).abs())) {
                return layer.height_at(xy);
            }
        }
        // Planar controllers and planning owners share immutable tomograms.
        // Read this bounded source column directly: disposable body caches may
        // be locked or retired by unrelated planning work on the same snapshot.
        let distance=|band:&FineContactBand|
            (band.fit[0]-reference).max(reference-band.fit[1]).max(0.0);
        self.read_fine_contact_bands(column[0],column[1]).iter().min_by(|a,b|
            distance(a).total_cmp(&distance(b)).then_with(||
                (a.fit[1]-reference).abs().total_cmp(&(b.fit[1]-reference).abs())))
            .map(|band|(f64::from(band.run[1])+1.0)*fine)
    }

    fn known_terrain_height(&self,xy:[f64;2],reference:f64)->Option<f64> {
        let direct=self.direct_ground_query(xy,reference).height;
        if self.observations.as_ref().is_some_and(|o|o.is_complete_geometry_source()) {return direct;}
        let coarse=f64::from(self.config().voxel_size);
        let column=xy.map(|v|(v/coarse).floor() as i32);
        self.data.physical_graph.surface_lookup.get(&(column[0],column[1]))
            .into_iter().flatten().map(|&z|self.ground_cell_position((column[0],column[1],z))[2])
            .chain(direct).min_by(|a,b|(a-reference).abs().total_cmp(&(b-reference).abs()))
    }

    fn direct_ground_query(&self,xy:[f64;2],reference:f64)->GroundQuery {
        if let Some(observations)=&self.observations {
            let fine=f64::from(observations.size_m);let column=xy.map(|v|(v/fine).floor() as i32);
            if self.ground_evidence==GroundEvidence::MeasuredSurface {
                if let Some(layer)=observations.ground_layers(column).min_by(|a,b|
                    (a.height_at(xy).unwrap_or(f64::INFINITY)-reference).abs()
                        .total_cmp(&(b.height_at(xy).unwrap_or(f64::INFINITY)-reference).abs())) {
                    let scatter=layer.statistics.moments().sample_scatter().map_or(0.0,|s| {
                        let n=layer.normal;
                        (n[0]*n[0]*s[0]+2.0*n[0]*n[1]*s[1]+2.0*n[0]*n[2]*s[2]
                            +n[1]*n[1]*s[3]+2.0*n[1]*n[2]*s[4]+n[2]*n[2]*s[5]).max(0.0).sqrt()
                    });
                    return GroundQuery {state:GroundQueryState::Observed,height:layer.height_at(xy),normal:Some(layer.normal),
                        sources:vec![layer.anchor],radius_m:0.0,max_residual_m:Some(scatter)};
                }
            }
            let bands=self.fine_contact_bands(column[0],column[1]);
            if let Some(band)=bands.iter().min_by(|a,b|
                (a.fit[0]+(a.fit[1]-a.fit[0])*0.5-reference).abs()
                    .total_cmp(&(b.fit[0]+(b.fit[1]-b.fit[0])*0.5-reference).abs())) {
                let layer=GroundLayer::from_voxel_run(band.run,fine,self.ground_evidence).expect("valid source run");
                return GroundQuery {state:GroundQueryState::LegacyGeometry,height:Some(layer.representative_z),
                    normal:Some([0.0,0.0,1.0]),sources:vec![[column[0],column[1],band.run[0]]],radius_m:0.0,
                    max_residual_m:Some((layer.contact_interval[1]-layer.contact_interval[0])*0.5)};
            }
            if observations.is_complete_geometry_source() {return GroundQuery::unknown();}
        }
        let size=f64::from(self.config().voxel_size);
        let x=(xy[0]/size).floor() as i32;let y=(xy[1]/size).floor() as i32;
        let Some(levels)=self.data.physical_graph.surface_lookup.get(&(x,y)) else {return GroundQuery::unknown();};
        let Some(&z)=levels.iter().min_by(|&&a,&&b|
            (self.ground_cell_position((x,y,a))[2]-reference).abs().total_cmp(&(self.ground_cell_position((x,y,b))[2]-reference).abs()))
            else {return GroundQuery::unknown();};
        let top=self.ground_cell_position((x,y,z))[2];let quantum=f64::from(self.source_voxel_size_m());
        let bottom=self.observed_support_lower_bound((x,y,z)).min(top-quantum);
        let Some(layer)=GroundLayer::from_legacy_band([bottom,top],quantum,self.ground_evidence) else {return GroundQuery::unknown();};
        GroundQuery {state:GroundQueryState::LegacyGeometry,height:Some(layer.representative_z),normal:Some([0.0,0.0,1.0]),
            sources:vec![[x,y,z]],radius_m:0.0,max_residual_m:Some((layer.contact_interval[1]-layer.contact_interval[0])*0.5)}
    }

    /// Shared terrain query. Missing samples are estimated only inside a bounded,
    /// connected same-plane neighbourhood; unknown never means occupied or free.
    /// Interpolation is lazy and cached per missing source column in this snapshot.
    pub fn query_ground(&self,xy:[f64;2],reference:f64)->GroundQuery {
        if !xy.into_iter().chain([reference]).all(f64::is_finite) {return GroundQuery::unknown();}
        let direct=self.direct_ground_query(xy,reference);
        if direct.state!=GroundQueryState::Unknown {return direct;}
        let Some(model)=self.body_envelope() else {return GroundQuery::unknown();};
        let grid=f64::from(self.observations.as_ref().map_or(self.config().voxel_size,|o|o.size_m));
        if xy.iter().any(|v|(v/grid).abs()>f64::from(i32::MAX)/2.0) {return GroundQuery::unknown();}
        let column=xy.map(|v|(v/grid).floor() as i32);let key=(column,reference.to_bits());
        if let Some(estimate)=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner()).ground_estimates.get(&key).cloned() {
            return estimate.map_or_else(GroundQuery::unknown,|e|e.at(xy));
        }
        let center=column.map(|v|(f64::from(v)+0.5)*grid);
        let radius=model.radius()+grid;
        let steps=(radius/grid).ceil() as i32;
        let quantum=f64::from(self.source_voxel_size_m());
        let mut samples=Vec::new();
        for dx in -1..=1 {for dy in -1..=1 {
            if dx==0&&dy==0 {continue;}
            for step in 1..=steps {
                let point=[center[0]+f64::from(dx*step)*grid,center[1]+f64::from(dy*step)*grid];
                if (point[0]-center[0]).hypot(point[1]-center[1])>radius+1e-9 {break;}
                let source=self.direct_ground_query(point,reference);
                if let Some(height)=source.height {
                    let cell=self.nearest_column_cell(point, height,model.max_step_up_m.max(model.max_step_down_m)+quantum);
                    if let Some(cell)=cell {samples.push((point,source,cell));}
                    break; // never look past a known different surface or wall
                }
                let ix=(point[0]/grid).floor() as i32;let iy=(point[1]/grid).floor() as i32;
                let occupied=self.observations.as_ref().map_or_else(||self.data.occupied_columns.contains_key(&(ix,iy)),
                    |o|o.column_levels(ix,iy).is_some());
                if occupied {break;}
            }
        }}
        let estimate=(|| {
            let first=samples.first()?;let normal=first.1.normal?;let z=first.1.height?;
            let origin=[first.0[0],first.0[1],z];
            let at=|point:[f64;2]|origin[2]-(normal[0]*(point[0]-origin[0])+normal[1]*(point[1]-origin[1]))/normal[2];
            if (at(center)-reference).abs()>model.max_step_up_m.max(model.max_step_down_m)+quantum {return None;}
            let surface_xyz=if first.1.state==GroundQueryState::Observed {
                let mut point=self.cell_position(first.2);
                point[2]=at([point[0],point[1]]);point
            } else {self.ground_cell_position_near(first.2,Some(z))};
            let location=SurfaceLocation{cell:first.2,support_xyz:origin,surface_xyz};
            let connected=self.ground_patch(location,2.0*radius+grid);
            let mut residual=0.0_f64;
            for (point,source,cell) in &samples {
                if connected.get(&(cell.0,cell.1))!=Some(cell) {return None;}
                let error=(source.height?-at(*point)).abs();
                // Source scatter is descriptive, not a robot step allowance.
                // Legacy quantization does not permit averaging an observed step.
                let tolerance=if source.state==GroundQueryState::Observed&&first.1.state==GroundQueryState::Observed {
                    2.0*(source.max_residual_m?+first.1.max_residual_m?)+1e-8
                } else {quantum*0.25+1e-8};
                if error>tolerance {return None;}
                residual=residual.max(error).max(source.max_residual_m?);
            }
            let boundary=hull(samples.iter().map(|s|s.0).collect());
            if boundary.len()<3||!inside_boundary(center,&boundary) {return None;}
            let sources=samples.iter().flat_map(|s|s.1.sources.iter().copied()).collect();
            Some(GroundEstimate{origin,normal,sources,hull:boundary,radius,residual})
        })();
        let result=estimate.as_ref().map_or_else(GroundQuery::unknown,|e|e.at(xy));
        let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
        if cache.ground_estimates.len()>=8192 {cache.ground_estimates.clear();}
        cache.ground_estimates.insert(key,estimate);result
    }

    // The capture owner already associated and fitted this physical layer.
    // Consumers query it directly rather than averaging it with a step or floor.
    fn measured_contact_frame(&self,xy:[f64;2],reference:f64,yaw:f64)->Option<(f64,[f64;4])> {
        if self.ground_evidence!=GroundEvidence::MeasuredSurface {return None;}
        let observations=self.observations.as_ref()?;
        let size=f64::from(observations.size_m);
        let column=xy.map(|v|(v/size).floor() as i32);
        let layer=observations.ground_layers(column).min_by(|a,b|
            (a.height_at(xy).unwrap_or(f64::INFINITY)-reference).abs()
                .total_cmp(&(b.height_at(xy).unwrap_or(f64::INFINITY)-reference).abs()))?;
        let normal=layer.normal;
        let forward=normalize([yaw.cos(),yaw.sin(),
            -(normal[0]*yaw.cos()+normal[1]*yaw.sin())/normal[2]]);
        let left=cross(normal,forward);
        Some((layer.height_at(xy)?,quaternion_from_columns([forward,left,normal])))
    }

    /// 在连续足迹邻域上共同拟合接触高度和法向；连续厚带保留实测参考允许的区间。
    fn body_contact_frame(&self,centre:SurfaceLocation,height:f64,yaw:f64)
        -> Result<(f64,[f64;4]),SurfaceError> {
        self.body_contact_frame_with_mode(centre,height,yaw,ContactPlaneMode::StaticInterval)
    }

    /// 动态输运只使用同一物理层的固定参考场，实际高度仅参与层关联。
    /// LegacyContactInterval（旧式接触区间）从不可变源的连通高度族读取固定参考。
    /// 该参考是几何近似，不是实测均值或物理真值；原区间仍决定动态拟合。
    fn body_motion_contact_frame(&self,centre:SurfaceLocation,height:f64,yaw:f64)
        -> Result<(f64,[f64;4]),SurfaceError> {
        self.body_contact_frame_with_mode(centre,height,yaw,ContactPlaneMode::FixedMotionReference)
    }

    fn body_contact_frame_with_mode(&self,centre:SurfaceLocation,height:f64,yaw:f64,mode:ContactPlaneMode)
        -> Result<(f64,[f64;4]),SurfaceError> {
        #[cfg(feature = "body-query-profile")]
        let _frame_profile = query_profile::guard(QueryStage::ContactFrame);
        let model=self.body_envelope()
            .ok_or_else(||SurfaceError::InvalidConfig("body envelope is unavailable".into()))?;
        let support=[centre.support_xyz[0],centre.support_xyz[1],height];
        if let Some(frame)=self.measured_contact_frame([support[0],support[1]],height,yaw) {return Ok(frame);}
        let key=(centre.cell,support.map(f64::to_bits),mode);
        #[cfg(feature = "body-query-profile")]
        let cache_profile = query_profile::guard(QueryStage::PlaneCacheLookup);
        let cached=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner()).planes.get(&key).copied();
        #[cfg(feature = "body-query-profile")]
        drop(cache_profile);
        let (a,b,z)=if let Some(plane)=cached {
            #[cfg(feature = "body-query-profile")]
            query_profile::count(QueryCounter::PlaneCacheHits);
            plane
        } else {
            #[cfg(feature = "body-query-profile")]
            query_profile::count(QueryCounter::PlaneCacheMisses);
            #[cfg(feature = "body-query-profile")]
            let domain_profile = query_profile::guard(QueryStage::ContactDomainCollection);
            // 足迹外包半径加实际接触样本间距；有精细观察时不随规划格改变。
            // 缺少精细观察时只保留粗格中心，不能用已丢失的细XY缩窄量化覆盖。
            let spacing=self.observations.as_ref().map_or(self.config().voxel_size,|o|o.size_m);
            let radius=model.radius()+f64::from(spacing);
            let mut points=if self.observations.is_some() {
                self.fine_contact_domain(support,radius)?
            } else {
                // 无原始细观察时保留粗支撑域；只为实际候选覆盖扩大枚举半径。
                let enumeration=radius+std::f64::consts::SQRT_2*f64::from(self.config().voxel_size);
                let patch=self.ground_patch_at_height(centre,enumeration,height);
                let mut points=Vec::new();
                for &cell in patch.values() {
                    for &(point,low) in self.ground_contact_samples(cell).iter() {
                        let distance2=(point[0]-support[0]).powi(2)+(point[1]-support[1]).powi(2);
                        let weight=(1.0-distance2/(radius*radius)).max(0.0).powi(2);
                        if weight>0.0 {points.push((point,low,weight));}
                    }
                }
                points
            };
            #[cfg(feature = "body-query-profile")]
            drop(domain_profile);
            #[cfg(feature = "body-query-profile")]
            query_profile::add(QueryCounter::ContactPoints, u64::try_from(points.len()).unwrap_or(u64::MAX));
            #[cfg(feature = "body-query-profile")]
            let sort_profile = query_profile::guard(QueryStage::ContactPointSort);
            points.sort_by(|a,b|a.0[0].total_cmp(&b.0[0]).then_with(||a.0[1].total_cmp(&b.0[1])));
            #[cfg(feature = "body-query-profile")]
            drop(sort_profile);
            #[cfg(feature = "body-query-profile")]
            let fit_profile = query_profile::guard(QueryStage::ContactPlaneFit);
            let plane=match mode {
                ContactPlaneMode::StaticInterval=>fit_contact_plane(&points,support),
                ContactPlaneMode::FixedMotionReference=> {
                    let thick=points.iter().any(|&(point,low,_)|point[2]!=low);
                    let reference=if let Some(observations)=self.observations.as_ref().filter(|_|thick) {
                        let field=observations.legacy_reference_field(self.ground_evidence,model.radius());
                        let samples:Option<Vec<_>>=points.iter().map(|&(point,low,weight)|
                            field.height(point,low).map(|height|([point[0]-support[0],point[1]-support[1],height],weight))).collect();
                        fit_plane_weighted(samples.ok_or(SurfaceError::MissingSupport)?.into_iter(),[0.0;3])
                            .ok_or(SurfaceError::MissingSupport)?.2
                    } else {
                        // 缺少原始细列时仍保留粗几何近似；薄面原高度不受固定参考影响。
                        let mass=points.iter().map(|point|point.2).sum::<f64>();
                        let origin=points.first().map_or(0.0,|&(point,low,_)|(low+point[2])*0.5);
                        origin+points.iter().map(|&(point,low,weight)|
                            ((low+point[2])*0.5-origin)*weight).sum::<f64>()/mass
                    };
                    dynamic_interval::fit_dynamic_contact_plane(&points,[support[0],support[1],reference])
                },
            }.ok_or(SurfaceError::MissingSupport)?;
            #[cfg(feature = "body-query-profile")]
            drop(fit_profile);
            let mut cache=self.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
            if cache.planes.len()>=8192 {cache.planes.clear();}
            cache.planes.insert(key,plane);plane
        };
        #[cfg(feature = "body-query-profile")]
        let _algebra_profile = query_profile::guard(QueryStage::ContactFrameAlgebra);
        let normal=normalize([-a,-b,1.0]);
        let forward=normalize([yaw.cos(),yaw.sin(),a*yaw.cos()+b*yaw.sin()]);
        let left=cross(normal,forward);
        Ok((z,quaternion_from_columns([forward,left,normal])))
    }

}
/// 共同水平区间最小化 Σw·dist(z,I)² + alpha·(z-h)²；alpha=rho·Σw。
/// rho=U/(U+S)，U=Σw·区间宽²/12，S=Σw·水平距离²，是几何正则偏好，
/// 不是已校准的传感器方差。区间残差允许非零；实际机体净空仍由完整几何检查决定。
/// 非共同区间和零区间宽度保留原拟合，秩检查也不变。
fn fit_contact_plane(points:&[([f64;3],f64,f64)],origin:[f64;3])->Option<(f64,f64,f64)> {
    let original=original_fit_contact_plane(points,origin)?;
    let low=points.iter().map(|p|p.1).fold(f64::NEG_INFINITY,f64::max);
    let high=points.iter().map(|p|p.0[2]).fold(f64::INFINITY,f64::min);
    if low>high {return Some(original);}
    let h=origin[2];
    if low<=h && h<=high {return Some((0.0,0.0,h));}
    let mut mass=0.0;let mut uncertainty=0.0;let mut span=0.0;
    for &(p,low,w) in points {
        mass+=w;uncertainty+=w*(p[2]-low).powi(2)/12.0;
        span+=w*((p[0]-origin[0]).powi(2)+(p[1]-origin[1]).powi(2));
    }
    if uncertainty==0.0 {return Some(original);}
    let anchor=uncertainty/(uncertainty+span)*mass;
    // Common intervals imply only upper OR lower endpoints can be active.
    // In displacement coordinates, alpha*d = sum(w*max(t-d,0)).
    let above=h>high;
    let mut events:Vec<_>=points.iter().filter_map(|&(p,l,w)| {
        let t=if above {h-p[2]} else {l-h};
        (t>0.0).then_some((t,w))
    }).collect();
    events.sort_unstable_by(|a,b|b.0.total_cmp(&a.0));
    let mut weight=0.0;let mut moment=0.0;
    for (i,&(t,w)) in events.iter().enumerate() {
        weight+=w;moment+=w*t;
        let displacement=moment/(anchor+weight);
        if i+1==events.len() || displacement>=events[i+1].0 {
            return Some((0.0,0.0,if above {h-displacement} else {h+displacement}));
        }
    }
    unreachable!("reference outside nonempty common interval has an active endpoint")
}


fn original_fit_contact_plane(points:&[([f64;3],f64,f64)],origin:[f64;3])->Option<(f64,f64,f64)> {
    if points.len()<3 {return None;}
    let rows:Vec<_>=points.iter().map(|&(p,low,w)|([p[0]-origin[0],p[1]-origin[1],1.0],low,p[2],w)).collect();
    let mut inverse=[[0.0;6];3];
    for &(v,_,_,w) in &rows {for i in 0..3 {for j in 0..3 {inverse[i][j]+=w*v[i]*v[j];}}}
    for i in 0..3 {inverse[i][i+3]=1.0;}
    for column in 0..3 {
        let pivot=(column..3).max_by(|&a,&b|inverse[a][column].abs().total_cmp(&inverse[b][column].abs()))?;
        if inverse[pivot][column].abs()<1e-10 {return None;}
        inverse.swap(column,pivot);
        let scale=inverse[column][column];for j in 0..6 {inverse[column][j]/=scale;}
        for i in 0..3 {if i!=column {
            let factor=inverse[i][column];for j in 0..6 {inverse[i][j]-=factor*inverse[column][j];}
        }}
    }
    // 区间拟合可能有多个同样可行的斜面，不能由每拍变化的参考高度选择不同法向。
    // 共同水平接触保留原参考；确需斜面时，从仅由固定观测决定的高度开始求解。
    let low=rows.iter().map(|row|row.1).fold(f64::NEG_INFINITY,f64::max);
    let high=rows.iter().map(|row|row.2).fold(f64::INFINITY,f64::min);
    if low<=high {return Some((0.0,0.0,origin[2].clamp(low,high)));}
    let weight=rows.iter().map(|row|row.3).sum::<f64>();
    let height=rows.iter().map(|row|(row.1+row.2)*0.5*row.3).sum::<f64>()/weight;
    let mut plane=[0.0,0.0,height];
    // 这是数值求解迭代界，不改变任何地面、净空、坡度或阶差许可。
    for _ in 0..128 {
        let project=|v:[f64;3],low:f64,high:f64|dot(v,plane).clamp(low,high);
        let first=rows[0];let height_origin=project(first.0,first.1,first.2);
        let mut rhs=[0.0;3];
        for &(v,low,high,w) in &rows {
            let residual=project(v,low,high)-height_origin;
            for i in 0..3 {rhs[i]+=w*v[i]*residual;}
        }
        let mut next=std::array::from_fn(|i|(0..3).map(|j|inverse[i][j+3]*rhs[j]).sum::<f64>());
        next[2]+=height_origin;
        let converged=plane.iter().zip(next).all(|(a,b)|(a-b).abs()<1e-12);
        plane=next;
        if converged {break;}
    }
    Some((plane[0],plane[1],plane[2]))
}

fn fit_plane(points: &[[f64; 3]], origin: [f64; 3]) -> Option<(f64, f64, f64)> {
    fit_plane_weighted(points.iter().copied().map(|point|(point,1.0)),origin)
}
fn fit_plane_weighted(points: impl Iterator<Item=([f64;3],f64)>, origin: [f64;3]) -> Option<(f64,f64,f64)> {
    // 用实际样本高度作数值原点，使恒定平面的右端项精确为零。
    // 名义支撑高度与地面的常量偏差不能经消元变成伪斜率。
    let mut height_origin=None;
    let mut count=0;
    let mut matrix = [[0.0; 4]; 3];
    for (p,weight) in points {
        let height_origin=*height_origin.get_or_insert(p[2]);
        count+=1;
        let v = [p[0] - origin[0], p[1] - origin[1], 1.0];
        for i in 0..3 {
            for j in 0..3 {
                matrix[i][j] += weight * v[i] * v[j];
            }
            matrix[i][3] += weight * v[i] * (p[2] - height_origin);
        }
    }
    if count<3 { return None; }
    let height_origin=height_origin?;
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
        height_origin + matrix[2][3] - matrix[0][3] * origin[0] - matrix[1][3] * origin[1],
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
    #[cfg(feature = "body-query-profile")]
    mod cache_profile_counter_tests { include!("body_cache_profile_counter_tests.rs"); }
    mod recent_query_tests { include!("body_recent_query_tests.rs"); }
    mod assigned_z_equivalence_tests { include!("body_assigned_z_equivalence_tests.rs"); }
    mod live_geometry_tests { include!("body_live_geometry_tests.rs"); }
    include!("body_observation_vec_tests.rs");
    include!("body_motion_reference_tests.rs");
    use super::*;
    #[test]
    fn oversized_motion_is_a_resource_error_without_a_collision_witness() {
        let map = scene(&[], false);
        let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let to = ([300.05, 0.05, 0.5], from.1);
        map.validate_body_pose(from.0, from.1).unwrap();
        map.validate_body_pose(to.0, to.1).unwrap();
        let mut witness = None;
        let result = map.validate_body_motion_with_witness(from, to, &[], &mut witness);
        assert!(matches!(result, Err(SurfaceError::InvalidConfig(ref reason)) if reason.contains("4096")), "{result:?}");
        assert!(witness.is_none(), "资源不足不能生成碰撞证据");
    }

    #[test]
    fn voxel_projection_accounts_for_corners_and_attitude_without_yaw_inflation() {
        let size=0.05;
        for yaw in [0.0_f64,0.3,0.8,1.5] {
            let r=rotation([0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()]).unwrap();
            assert!((projected_voxel_radius(r,size)-size/std::f64::consts::SQRT_2).abs()<1e-12);
        }
        let half=std::f64::consts::FRAC_PI_8;
        let r=rotation([half.sin(),0.0,0.0,half.cos()]).unwrap();
        assert!((projected_voxel_radius(r,size)-size*3.0_f64.sqrt()/2.0).abs()<1e-12);
    }
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
        make(&[0.175]).validate_body_pose(p,q).unwrap();
        assert!(make(&[0.175]).query_ground([p[0],p[1]],0.0).height.unwrap()>0.15,
            "the actual high contact is retained; no second nominal-support veto is imposed within declared step plus quantization tolerance");
    }

    #[test]
    fn reported_contact_height_does_not_change_the_same_observed_plane() {
        let points:Vec<_>=(-5..=5).flat_map(|x|(-5..=5).map(move|y| {
            let x=f64::from(x)*0.1;
            ([x,f64::from(y)*0.1,0.2*x+0.04],0.2*x-0.04,1.0)
        })).collect();
        for height in [-0.08,0.08] {
            let first=fit_contact_plane(&points,[0.0,0.0,height]).unwrap();
            let repeated=fit_contact_plane(&points,[0.0,0.0,first.2]).unwrap();
            assert!((first.0-repeated.0).abs()<1e-10 && (first.1-repeated.1).abs()<1e-10
                && (first.2-repeated.2).abs()<1e-10,
                "同一观测、同一XY按返回高度再查不能改变地面：{first:?} -> {repeated:?}");
        }
    }

    #[test]
    fn body_attitude_respects_contact_band_uncertainty_and_real_step_tops() {
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
        let step = make(false).body_orientation_at(support, 0.0).unwrap();
        assert!(step[1].abs() > 0.01, "实际高踏面不得被压成平地");
        assert!(band[0].abs()<1e-6 && band[1].abs()<1e-6,
            "实际共同平面位于连续观测带内时，不能强制贴合各列不同的最高点: {band:?}");
    }
    #[test]
    fn ground_attitude_changes_continuously_across_voxel_boundaries() {
        let base=scene(&[],false);
        let points:Vec<_>=(-20..20).flat_map(|x|(-15..15).map(move |y| {
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,if x>=0 {0.125} else {0.025})
        })).collect();
        let map=SurfaceSnapshot::from_points(&points,base.config().clone(),0.05).unwrap()
            .with_body_envelope(base.body_envelope().unwrap().clone()).unwrap();
        for index in (-4..0).chain(1..4) {
            let boundary=f64::from(index)*f64::from(map.config().voxel_size);
            let height=if index>0 {0.15} else {0.05};
            let a=map.body_orientation_at([boundary-1e-6,0.05,height],0.0).unwrap();
            let b=map.body_orientation_at([boundary+1e-6,0.05,height],0.0).unwrap();
            let difference=a.iter().zip(b).map(|(a,b)|(a-b).abs()).fold(0.0,f64::max);
            assert!(difference<1e-4,"微米跨格不能改变有限姿态: x={boundary} a={a:?} b={b:?}");
        }
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
    fn body_collision_retains_persistent_obstacles_independent_of_hit_count_or_age() {
        for kind in 0..5 {
            let mut extra = vec![(0.45,0.05,0.65)];
            if kind == 4 {extra.extend([(0.55,0.05,0.65),(0.45,0.15,0.65)]);}
            let base = scene(&extra,false);
            let mut observations = observation_grid(|values| {
                values.insert([4,0,6],SurfaceObservation {
                    observed_ns:if kind == 2 {10_000_000_000} else {1},
                    hits:if kind == 3 {3} else {1},
                });
                if kind == 4 {
                    for key in [[5,0,6],[4,1,6]] {
                        values.insert(key,SurfaceObservation {observed_ns:1,hits:1});
                    }
                }
            }).as_ref().clone();
            if kind != 0 {observations.set_source_timestamp_ns(10_000_000_000);}
            let map = base.with_observations(Some(Arc::new(observations)));
            assert_eq!(map.validate_body_pose([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]).is_ok(),false,
                "persistent occupancy does not expire by hit count or floor age: {kind}");
        }
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
            assert!(bad.validate_observed_body_motion(from, to, &occupied, 10_100_000_000, 1_000_000_000).is_ok(), "old ground metadata is not a live motion lease: {kind}");
        }
        // Ground history is geometric input; current stream health is independent.
        assert!(map.validate_observed_body_motion(from, to, &occupied, 12_000_000_000, 1_000_000_000).is_ok());
        // 当前实际占据区域仍需通过几何检查，但可以从旧足下证据开始停稳。
        map.validate_observed_body_motion(from, from, &occupied, 12_000_000_000, 1_000_000_000).unwrap();
    }

    fn assert_observation_checker_matches(
        map: &SurfaceSnapshot,
        occupied: &std::collections::BTreeSet<[i32; 3]>,
        max_age_ns: u64,
        boundary: &[[f64; 2]],
        queries: &[(([f64; 3], [f64; 4]), ([f64; 3], [f64; 4]), u64)],
    ) {
        let mut cached = map.observed_body_motion_checker(occupied, max_age_ns, boundary);
        let mut with_expiry = map.observed_body_motion_checker_with_expiry(occupied, max_age_ns, boundary);
        for &(from, to, checked_ns) in queries {
            let expected = map.validate_observed_body_motion_in_boundary(
                from, to, occupied, checked_ns, max_age_ns, boundary);
            assert_eq!(cached(from, to, checked_ns), expected,
                "足迹复用必须保留完整结果、首个错误及时间字段: {from:?} -> {to:?}, {checked_ns}");
            assert_eq!(with_expiry(from, to, checked_ns).map(|_| ()), expected,
                "到期凭据必须保留独立原始检查的完整结果和首个错误: {from:?} -> {to:?}, {checked_ns}");
        }
    }

    #[test]
    fn legacy_expiry_has_no_ground_lease_and_still_checks_intermediate_obstacles() {
        let map=scene(&[],false).with_observations(Some(observation_grid(|cells| {
            for key in [[-4,0,0],[10,0,0],[18,0,0]] {cells.get_mut(&key).unwrap().observed_ns=1;}
        })));
        let from=([0.05,0.05,0.5],[0.,0.,0.,1.]);let to=([1.85,0.05,0.5],from.1);
        let occupied=map.body_observation_footprint(from).unwrap();
        let mut checker=map.observed_body_motion_checker_with_expiry(&occupied,1_000_000_000,&[]);
        for now in [10_100_000_000,12_000_000_000,u64::MAX] {
            assert_eq!(checker(from,to,now),Ok(None));assert_eq!(checker(to,to,now),Ok(None));
        }
        let blocked=map.with_recent_obstacle_points(1,100,0,&[],[(0,[1.05,0.05,0.55])]).unwrap();
        blocked.validate_body_pose(from.0,from.1).unwrap();blocked.validate_body_pose(to.0,to.1).unwrap();
        let mut checker=blocked.observed_body_motion_checker_with_expiry(&occupied,1,&[]);
        assert_eq!(checker(from,to,u64::MAX),Err(SurfaceError::InsufficientClearance));
    }

    #[test]
    fn legacy_expiry_boundaries_do_not_veto_ground_but_new_obstacles_invalidate_geometry() {
        let map=scene(&[],false).with_observations(Some(observation_grid(|_|{})));
        let pose=([0.05,0.05,0.5],[0.,0.,0.,1.]);let none=Default::default();
        let mut checker=map.observed_body_motion_checker_with_expiry(&none,1_000_000_000,&[]);
        for now in [10_100_000_000,11_000_000_000,11_000_000_001,10_100_000_000] {
            assert_eq!(checker(pose,pose,now),Ok(None));
        }
        let obstacle=map.with_recent_obstacle_points(1,100,0,&[],[(0,[0.05,0.05,0.55])]).unwrap();
        let mut changed=obstacle.observed_body_motion_checker_with_expiry(&none,1_000_000_000,&[]);
        assert_eq!(changed(pose,pose,11_000_000_001),Err(SurfaceError::InsufficientClearance));
        assert_eq!(checker(pose,pose,11_000_000_001),Ok(None),"immutable prior view remains unchanged");
    }

    #[test]
    fn legacy_expiry_preserves_collision_failure_and_does_not_leak_to_clear_pose() {
        let map=scene(&[],false).with_observations(Some(observation_grid(|cells| {
            cells.get_mut(&[-4,0,0]).unwrap().observed_ns=1;
            cells.get_mut(&[8,0,0]).unwrap().hits=0;cells.remove(&[9,0,0]);
        }))).with_recent_obstacle_points(1,100,0,&[],[(0,[0.85,0.05,0.55])]).unwrap();
        let from=([0.05,0.05,0.5],[0.,0.,0.,1.]);let to=([0.45,0.05,0.5],from.1);
        let later=([1.65,0.05,0.5],from.1);let none=Default::default();
        let mut checker=map.observed_body_motion_checker_with_expiry(&none,1,&[]);
        assert_eq!(checker(from,from,u64::MAX),Ok(None));
        assert_eq!(checker(from,to,u64::MAX),Err(SurfaceError::InsufficientClearance));
        assert_eq!(checker(from,to,u64::MAX),Err(SurfaceError::InsufficientClearance));
        assert_eq!(checker(later,later,u64::MAX),Ok(None));
    }

    #[test]
    fn observed_motion_expiry_without_nonexempt_cells_still_checks_geometry_and_source() {
        let base = scene(&[], false);
        let map = base.with_observations(Some(observation_grid(|cells| {
            for evidence in cells.values_mut() { *evidence = SurfaceObservation { observed_ns: 1, hits: 0 }; }
        })));
        let pose = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let occupied = map.body_observation_footprint(pose).unwrap();
        let mut checker = map.observed_body_motion_checker_with_expiry(&occupied, 1_000_000_000, &[]);
        assert_eq!(checker(pose, pose, 12_000_000_000), Ok(None));
        assert_eq!(checker(pose, pose, 0), Ok(None));
        let invalid = (pose.0, [0.0; 4]);
        assert_eq!(checker(invalid, pose, 12_000_000_000).map(|_| ()),
            map.validate_observed_body_motion_in_boundary(invalid, pose, &occupied,
                12_000_000_000, 1_000_000_000, &[]));
        assert!(checker(invalid, pose, 12_000_000_000).is_err());
        let boundary = [[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]];
        let mut bounded = map.observed_body_motion_checker_with_expiry(&occupied, 1_000_000_000, &boundary);
        assert_eq!(bounded(pose, pose, 12_000_000_000), Err(SurfaceError::InvalidBoundary));
        let mut missing = base.observed_body_motion_checker_with_expiry(&occupied, 1_000_000_000, &[]);
        assert_eq!(missing(pose, pose, 12_000_000_000), Ok(None));
    }

    #[test]
    fn legacy_expiry_clock_extremes_never_change_geometry_or_create_a_time_certificate() {
        let base=scene(&[],false);let pose=([0.05,0.05,0.5],[0.,0.,0.,1.]);let none=Default::default();
        for (observed,age,now) in [(u64::MAX-100,99,u64::MAX-1),(u64::MAX-100,100,u64::MAX),
            (u64::MAX-100,101,u64::MAX),(u64::MAX,1,u64::MAX),(0,u64::MAX,0)] {
            let map=base.with_observations(Some(observation_grid(|cells| {
                for evidence in cells.values_mut() {evidence.observed_ns=observed;}
            })));
            assert_eq!(map.observed_body_motion_checker_with_expiry(&none,age,&[])(pose,pose,now),Ok(None));
            let blocked=map.with_recent_obstacle_points(1,100,0,&[],[(0,[0.05,0.05,0.55])]).unwrap();
            assert_eq!(blocked.observed_body_motion_checker_with_expiry(&none,age,&[])(pose,pose,now),Err(SurfaceError::InsufficientClearance));
        }
    }

    // Frozen direct-lookup specialization and borrowed-lookup full-footprint comparator.
    fn measured_footprint_block_probe<const BORROWED:bool>(
        map: &SurfaceSnapshot, pose: ([f64; 3], [f64; 4]),
    ) -> Result<std::collections::BTreeSet<[i32; 3]>, SurfaceError> {
        #[cfg(feature = "body-query-profile")] let _footprint_profile=query_profile::guard(QueryStage::ObservationFootprint);
        let observations = map.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        if observations.size_m.to_bits() != map.source_voxel_size_m().to_bits() {
            return Err(SurfaceError::InvalidConfig("观察精度与几何来源不一致".into()));
        }
        let mut block_probe=crate::surface::observations::ObservationBlockLookup::new(observations);
        let model = map.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let centre = {
            #[cfg(feature = "body-query-profile")] let _support_profile=query_profile::guard(QueryStage::ObservationFootprintSupport);
            map.locate_support_from_body(pose.0, pose.1, model.body_to_support_z_m)?
        };
        #[cfg(feature = "body-query-profile")] let shape_profile=query_profile::guard(QueryStage::ObservationFootprintShape);
        let r = rotation(pose.1)?;
        let vertices: Vec<_> = model.footprint_xy_m.iter()
            .flat_map(|p| model.z_bounds_m.map(|z| world(r, pose.0, [p[0], p[1], z])))
            .collect();
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let size = f64::from(observations.size_m);
        let planner_size = f64::from(map.config().voxel_size);
        let min: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min) / size).floor() as i32);
        let max: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::NEG_INFINITY, f64::max) / size).floor() as i32);
        if (i64::from(max[0]) - i64::from(min[0]) + 1) * (i64::from(max[1]) - i64::from(min[1]) + 1) > 16_384 {
            return Err(SurfaceError::InvalidConfig("近场精细支撑查询超过预算".into()));
        }
        let radius = model.radius() + model.z_bounds_m[0].abs().max(model.z_bounds_m[1].abs())
            + model.body_to_support_z_m + planner_size * 2.0;
        #[cfg(feature = "body-query-profile")] drop(shape_profile);
        let patch = {
            #[cfg(feature = "body-query-profile")] let _patch_profile=query_profile::guard(QueryStage::ObservationFootprintPatch);
            map.ground_patch(centre, radius)
        };
        #[cfg(feature = "body-query-profile")] let cells_profile=query_profile::guard(QueryStage::ObservationFootprintCells);
        let polygon_query=PolygonQuery::new(&polygon);
        #[cfg(feature = "body-query-profile")] let mut height_counts=ObservationHeightCounts::default();
        let mut cells = std::collections::BTreeSet::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                if !polygon_query.intersects(x, y, size) { continue; }
                let column = (((x as f64 + 0.5) * size / planner_size).floor() as i32,
                    ((y as f64 + 0.5) * size / planner_size).floor() as i32);
                let ground = *patch.get(&column).ok_or(SurfaceError::MissingSupport)?;
                let height = map.cell_position(ground)[2];
                // 粗格跨坡面/阶边时，低侧真实细格可以属于同列较低的占据粗格。
                // 查询范围由一格内允许的坡差/阶差限制，并要求候选确在规范物理占据内。
                // 始终先选实际最高回波再查时效，旧顶部不能被新低面替代。
                let top = ((height / size).round() - 1.0) as i32;
                let height_span = (planner_size * std::f64::consts::SQRT_2
                    * f64::from(map.config().max_slope_deg).to_radians().tan())
                    .max(model.max_step_up_m).max(model.max_step_down_m)
                    + size + model.support_uncertainty_m;
                let count = (height_span / size).ceil().min(128.0) as i32;
                let bottom = top.saturating_sub(count);
                #[cfg(feature = "body-query-profile")] {height_counts.columns+=1;}
                let z = (bottom..=top).rev().find(|&z| {
                    #[cfg(feature = "body-query-profile")] {height_counts.probes+=1;}
                    if (if BORROWED{block_probe.get([x,y,z])}else{observations.get([x,y,z])}).is_none() {
                        #[cfg(feature = "body-query-profile")] {height_counts.absent+=1;}
                        return false;
                    }
                    let centre = (z as f64 + 0.5) * size;
                    let coarse_z = (centre / planner_size).floor() as i32;
                    map.data.occupied_columns.get(&column)
                        .is_some_and(|zs| zs.binary_search(&coarse_z).is_ok())
                }).unwrap_or(top);
                cells.insert([x, y, z]);
            }
        }
        #[cfg(feature = "body-query-profile")] drop(height_counts);
        drop(polygon_query);
        #[cfg(feature = "body-query-profile")] {
            drop(cells_profile);
            query_profile::add(QueryCounter::ObservationFootprintCellsBuilt,cells.len() as u64);
        }
        Ok(cells)
    }

    #[test]
    fn borrowed_block_footprint_preserves_a_genuinely_derived_current_view() {
        use std::collections::BTreeMap;
        let mut groups=BTreeMap::<_,BTreeMap<_,_>>::new();
        for x in -30_i32..30 {for y in -30_i32..30 {
            if x==4&&y==0 {continue;}
            let key=[x,y,0];groups.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation{observed_ns:10_000_000_000,hits:2});
        }}
        let mut observations=SurfaceObservations::new(0.1,32).unwrap();
        for (block,values) in groups{observations.replace_block(block,values).unwrap();}
        let map=scene(&[],true).with_observations(Some(Arc::new(observations)));
        let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        map.validate_body_pose(pose.0,pose.1).unwrap();
        let view=map.with_current_body_support(pose.0,pose.1).unwrap();
        let expected=measured_footprint_block_probe::<false>(&view,pose);
        let actual=view.body_observation_footprint(pose);
        assert_eq!(actual,expected);
        assert_eq!(actual,Err(SurfaceError::MissingSupport),"legacy footprint diagnostic can remain unknown without vetoing geometry");
        assert!(view.observations.as_ref().unwrap().get([4,0,0]).is_none(),"view must not fabricate evidence");
        let occupied=std::collections::BTreeSet::new();
        let raw=view.validate_observed_body_motion(pose,pose,&occupied,10_100_000_000,1_000_000_000);
        assert_eq!(raw,Ok(()),"footprint diagnostics do not reinstate a per-cell age gate");
        let mut checker=view.observed_body_motion_checker(&occupied,1_000_000_000,&[]);
        assert_eq!(checker(pose,pose,10_100_000_000),raw);
    }

    #[test]
    fn full_footprint_block_probe_matches_direct_queries() {
        let mut successes=0;let mut failures=0;
        for size in [0.05,0.01] {for hole in [false,true] {
            let map=polygon_footprint_fixture(size,hole);
            for x in [-0.1,0.05,2.0] {for yaw in [0.0_f64,0.37,-1.1] {
                let pose=([x,-0.05,0.5],[0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()]);
                let old=map.body_observation_footprint(pose);
                assert_eq!(measured_footprint_block_probe::<false>(&map,pose),old);
                assert_eq!(measured_footprint_block_probe::<true>(&map,pose),old);
                if old.is_ok(){successes+=1}else{failures+=1}
                if let Ok(view)=map.with_current_body_support(pose.0,pose.1) {
                    assert_eq!(measured_footprint_block_probe::<true>(&view,pose),view.body_observation_footprint(pose));
                }
            }}
            let empty=map.with_observations(Some(Arc::new(SurfaceObservations::new(size,32).unwrap())));
            let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
            assert_eq!(measured_footprint_block_probe::<true>(&empty,pose),empty.body_observation_footprint(pose));
            let missing=map.with_observations(None);
            assert_eq!(measured_footprint_block_probe::<true>(&missing,pose),missing.body_observation_footprint(pose));
        }}
        assert!(successes>=12&&failures>0);
    }

    #[test]
    #[ignore="matched complete raw-footprint direct versus borrowed accessor comparison"]
    fn benchmark_full_footprint_block_probe() {
        use std::{hint::black_box,time::Instant};
        for size in [0.05,0.01] {
            let source=polygon_footprint_fixture(size,false);
            for rounded in [false,true] {
                let mut envelope=model();if rounded{envelope.footprint_xy_m=(0..24).map(|i|{let a=i as f64*std::f64::consts::TAU/24.0;[0.3*a.cos(),0.16*a.sin()]}).collect();}
                let map=source.with_body_envelope(envelope).unwrap();
                for yaw in [0.0_f64,0.37] {
                    let pose=([0.05,0.05,0.5],[0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()]);
                    let expected=map.body_observation_footprint(pose).unwrap();
                    assert_eq!(measured_footprint_block_probe::<true>(&map,pose).unwrap(),expected);
                    let mut direct_ms=Vec::new();let mut borrowed_ms=Vec::new();
                    for round in 0..6 {for borrowed in if round%2==0{[false,true]}else{[true,false]} {
                        let started=Instant::now();for _ in 0..30 {
                            let map=black_box(&map);let pose=black_box(pose);
                            let actual=if borrowed{measured_footprint_block_probe::<true>(map,pose)}else{measured_footprint_block_probe::<false>(map,pose)}.unwrap();
                            assert_eq!(black_box(&actual),&expected);
                        }
                        let ms=started.elapsed().as_secs_f64()*1e3;if borrowed{borrowed_ms.push(ms)}else{direct_ms.push(ms)}
                    }}
                    eprintln!("FULL_FOOTPRINT_BLOCK_PROBE size={size} rounded={rounded} yaw={yaw} cells={} loops=30 warmed_by_parity=true includes=full_raw_footprint_equality_retirement direct_ms={direct_ms:?} borrowed_ms={borrowed_ms:?}",expected.len());
                }
            }
        }
    }

    // Test-only level-index work-count prototype; production selection remains unchanged.
    fn measured_footprint_level_selection<const INDEXED:bool,const TOP_FIRST:bool>(
        map: &SurfaceSnapshot, counters: &mut [u64;3], pose: ([f64; 3], [f64; 4]),
    ) -> Result<std::collections::BTreeSet<[i32; 3]>, SurfaceError> {
        #[cfg(feature = "body-query-profile")] let _footprint_profile=query_profile::guard(QueryStage::ObservationFootprint);
        let observations = map.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        if observations.size_m.to_bits() != map.source_voxel_size_m().to_bits() {
            return Err(SurfaceError::InvalidConfig("观察精度与几何来源不一致".into()));
        }
        let model = map.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let centre = {
            #[cfg(feature = "body-query-profile")] let _support_profile=query_profile::guard(QueryStage::ObservationFootprintSupport);
            map.locate_support_from_body(pose.0, pose.1, model.body_to_support_z_m)?
        };
        #[cfg(feature = "body-query-profile")] let shape_profile=query_profile::guard(QueryStage::ObservationFootprintShape);
        let r = rotation(pose.1)?;
        let vertices: Vec<_> = model.footprint_xy_m.iter()
            .flat_map(|p| model.z_bounds_m.map(|z| world(r, pose.0, [p[0], p[1], z])))
            .collect();
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let size = f64::from(observations.size_m);
        let planner_size = f64::from(map.config().voxel_size);
        let min: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min) / size).floor() as i32);
        let max: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::NEG_INFINITY, f64::max) / size).floor() as i32);
        if (i64::from(max[0]) - i64::from(min[0]) + 1) * (i64::from(max[1]) - i64::from(min[1]) + 1) > 16_384 {
            return Err(SurfaceError::InvalidConfig("近场精细支撑查询超过预算".into()));
        }
        let radius = model.radius() + model.z_bounds_m[0].abs().max(model.z_bounds_m[1].abs())
            + model.body_to_support_z_m + planner_size * 2.0;
        #[cfg(feature = "body-query-profile")] drop(shape_profile);
        let patch = {
            #[cfg(feature = "body-query-profile")] let _patch_profile=query_profile::guard(QueryStage::ObservationFootprintPatch);
            map.ground_patch(centre, radius)
        };
        #[cfg(feature = "body-query-profile")] let cells_profile=query_profile::guard(QueryStage::ObservationFootprintCells);
        let polygon_query=PolygonQuery::new(&polygon);
        let mut cells = std::collections::BTreeSet::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                if !polygon_query.intersects(x, y, size) { continue; }
                let column = (((x as f64 + 0.5) * size / planner_size).floor() as i32,
                    ((y as f64 + 0.5) * size / planner_size).floor() as i32);
                let ground = *patch.get(&column).ok_or(SurfaceError::MissingSupport)?;
                let height = map.cell_position(ground)[2];
                // 粗格跨坡面/阶边时，低侧真实细格可以属于同列较低的占据粗格。
                // 查询范围由一格内允许的坡差/阶差限制，并要求候选确在规范物理占据内。
                // 始终先选实际最高回波再查时效，旧顶部不能被新低面替代。
                let top = ((height / size).round() - 1.0) as i32;
                let height_span = (planner_size * std::f64::consts::SQRT_2
                    * f64::from(map.config().max_slope_deg).to_radians().tan())
                    .max(model.max_step_up_m).max(model.max_step_down_m)
                    + size + model.support_uncertainty_m;
                let count = (height_span / size).ceil().min(128.0) as i32;
                let bottom = top.saturating_sub(count);
                counters[0]+=1;
                let mut accepts=|z:i32| {
                    counters[1]+=1;
                    if observations.get([x,y,z]).is_none() { counters[2]+=1;return false; }
                    let centre = (z as f64 + 0.5) * size;
                    let coarse_z = (centre / planner_size).floor() as i32;
                    map.data.occupied_columns.get(&column)
                        .is_some_and(|zs| zs.binary_search(&coarse_z).is_ok())
                };
                let z=if TOP_FIRST && accepts(top) {Some(top)}else if INDEXED {
                    let levels=observations.column_levels(x,y).unwrap_or(&[]);
                    let end=levels.partition_point(|&z|if TOP_FIRST{z<top}else{z<=top});
                    levels[..end].iter().rev().copied().take_while(|&z|z>=bottom)
                        .find(|&z|accepts(z))
                }else{(bottom..=top).rev().find(|&z|accepts(z))}.unwrap_or(top);
                cells.insert([x, y, z]);
            }
        }
        drop(polygon_query);
        #[cfg(feature = "body-query-profile")] {
            drop(cells_profile);
            query_profile::add(QueryCounter::ObservationFootprintCellsBuilt,cells.len() as u64);
        }
        Ok(cells)
    }
    fn level_selection_fixture(size:f32,geometry_z:i32,observed_layers:&[i32])->SurfaceSnapshot {
        use std::collections::BTreeMap;
        let n=(0.7/f64::from(size)).ceil() as i32;
        let points:Vec<_>=(-n..n).flat_map(|x|(-n..n).map(move|y|
            ((x as f32+0.5)*size,(y as f32+0.5)*size,(geometry_z as f32+0.5)*size))).collect();
        let base=scene(&[],false);
        let map=SurfaceSnapshot::from_points(&points,base.config().clone(),size).unwrap().with_body_envelope(model()).unwrap();
        let mut blocks=BTreeMap::<_,BTreeMap<_,_>>::new();
        for x in -n..n {for y in -n..n {for &z in observed_layers {
            let key=[x,y,z]; blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation{observed_ns:10_000_000_000,hits:2});
        }}}
        let mut observations=SurfaceObservations::new(size,32).unwrap();
        for (block,values) in blocks {observations.replace_block(block,values).unwrap();}
        observations.set_source_timestamp_ns(10_000_000_000);
        map.with_observations(Some(Arc::new(observations)))
    }

    #[test]
    #[cfg(feature = "body-query-profile")]
    fn observation_height_profile_matches_counted_original_scan() {
        for layers in [vec![0],vec![],vec![-1,20]] {
            let map=level_selection_fixture(0.01,0,&layers);
            let pose=([0.05,0.05,0.41],[0.0,0.0,0.0,1.0]);
            let mut counts=[0;3];
            let expected=measured_footprint_level_selection::<false,false>(&map,&mut counts,pose);
            query_profile::begin();let actual=map.body_observation_footprint(pose);let record=query_profile::end();
            assert_eq!(actual,expected);assert!(actual.is_ok());
            assert!(record.counters[QueryCounter::ObservationBlockRootResolutions as usize]>0);
            assert!(record.counters[QueryCounter::ObservationBlockRootResolutions as usize]<=counts[1]);
            for (counter,count) in [QueryCounter::ObservationHeightColumns,QueryCounter::ObservationHeightProbes,QueryCounter::ObservationHeightAbsent].into_iter().zip(counts) {
                assert_eq!(record.counters[counter as usize],count);
            }
        }
    }

    #[test]
    fn indexed_level_selection_prototype_matches_full_integer_scan() {
        let cases=[(0,vec![0]),(0,vec![9]),(0,(0..=9).collect()),(0,vec![-1,0,9,20,31,32]),
            (0,vec![]),(0,vec![-1,20]),(-10,vec![-10]),(-10,vec![-11,-10,-1,0])];
        let mut successes=0;
        for (geometry_z,layers) in cases {
            let map=level_selection_fixture(0.01,geometry_z,&layers);
            for x in [-0.1,0.05] {for yaw in [0.0_f64,0.37] {
                let body_z=0.4+(geometry_z as f64+1.0)*0.01;
                let pose=([x,0.05,body_z],[0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()]);
                let mut old_counts=[0;3];let mut new_counts=[0;3];
                let old=measured_footprint_level_selection::<false,false>(&map,&mut old_counts,pose);
                let new=measured_footprint_level_selection::<true,false>(&map,&mut new_counts,pose);
                let hybrid=measured_footprint_level_selection::<true,true>(&map,&mut [0;3],pose);
                assert_eq!(old,map.body_observation_footprint(pose));assert_eq!(new,old);assert_eq!(hybrid,old);
                if old.is_ok(){successes+=1;assert_eq!(old_counts[0],new_counts[0]);}
                eprintln!("LEVEL_SELECTION geometry_z={geometry_z} layers={layers:?} pose={pose:?} old_column_probe_absent={old_counts:?} indexed_column_probe_absent={new_counts:?} success={}",old.is_ok());
            }}
        }
        assert!(successes>=24,"must exercise successful XYZ output, not only errors");
    }

    #[test]
    #[ignore="static work-count and alternating prototype microbenchmark, no live throughput claim"]
    fn benchmark_indexed_level_selection_prototype() {
        use std::{hint::black_box,time::Instant};
        for (label,geometry_z,layers) in [("low_fine_floor",0,vec![0]),("above_top_only",0,vec![9]),
            ("dense_levels",0,(0..=9).collect()),("no_evidence",0,vec![]),
            ("incompatible_layers",0,vec![-1,20]),("negative_low_floor",-10,vec![-10])] {
            let map=level_selection_fixture(0.01,geometry_z,&layers);
            let pose=([0.05,0.05,0.4+(geometry_z as f64+1.0)*0.01],[0.0,0.0,0.0,1.0]);
            let mut counts=[0;3];let expected=map.body_observation_footprint(pose).unwrap();
            assert_eq!(measured_footprint_level_selection::<true,false>(&map,&mut counts,pose).unwrap(),expected);
            let mut old_ms=Vec::new();let mut new_ms=Vec::new();let mut hybrid_ms=Vec::new();
            for round in 0..6 {for variant in if round%2==0{[0,1,2]}else{[2,1,0]} {
                let mut counts=[0;3];let started=Instant::now();
                for _ in 0..30 {
                    let map=black_box(&map);let pose=black_box(pose);
                    let actual=match variant {1=>measured_footprint_level_selection::<true,false>(map,&mut counts,pose),
                        2=>measured_footprint_level_selection::<true,true>(map,&mut counts,pose),
                        _=>measured_footprint_level_selection::<false,false>(map,&mut counts,pose)}.unwrap();
                    assert_eq!(black_box(&actual),&expected);
                }
                let ms=started.elapsed().as_secs_f64()*1e3;
                match variant {1=>new_ms.push(ms),2=>hybrid_ms.push(ms),_=>old_ms.push(ms)}
            }}
            eprintln!("LEVEL_SELECTION_MICRO label={label} cells={} loops=30 includes=counting_output_equality_retirement warmed_by_parity=true old_ms={old_ms:?} indexed_ms={new_ms:?} hybrid_ms={hybrid_ms:?}",expected.len());
        }
    }

    // Frozen pre-hoist method; only the receiver and function name differ.
    fn reference_observation_footprint_rebuilding_polygon(
        map: &SurfaceSnapshot, pose: ([f64; 3], [f64; 4]),
    ) -> Result<std::collections::BTreeSet<[i32; 3]>, SurfaceError> {
        #[cfg(feature = "body-query-profile")] let _footprint_profile=query_profile::guard(QueryStage::ObservationFootprint);
        let observations = map.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
        if observations.size_m.to_bits() != map.source_voxel_size_m().to_bits() {
            return Err(SurfaceError::InvalidConfig("观察精度与几何来源不一致".into()));
        }
        let model = map.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let centre = {
            #[cfg(feature = "body-query-profile")] let _support_profile=query_profile::guard(QueryStage::ObservationFootprintSupport);
            map.locate_support_from_body(pose.0, pose.1, model.body_to_support_z_m)?
        };
        #[cfg(feature = "body-query-profile")] let shape_profile=query_profile::guard(QueryStage::ObservationFootprintShape);
        let r = rotation(pose.1)?;
        let vertices: Vec<_> = model.footprint_xy_m.iter()
            .flat_map(|p| model.z_bounds_m.map(|z| world(r, pose.0, [p[0], p[1], z])))
            .collect();
        let polygon = hull(vertices.iter().map(|p| [p[0], p[1]]).collect());
        let size = f64::from(observations.size_m);
        let planner_size = f64::from(map.config().voxel_size);
        let min: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min) / size).floor() as i32);
        let max: [i32; 2] = std::array::from_fn(|i| (vertices.iter().map(|p| p[i]).fold(f64::NEG_INFINITY, f64::max) / size).floor() as i32);
        if (i64::from(max[0]) - i64::from(min[0]) + 1) * (i64::from(max[1]) - i64::from(min[1]) + 1) > 16_384 {
            return Err(SurfaceError::InvalidConfig("近场精细支撑查询超过预算".into()));
        }
        let radius = model.radius() + model.z_bounds_m[0].abs().max(model.z_bounds_m[1].abs())
            + model.body_to_support_z_m + planner_size * 2.0;
        #[cfg(feature = "body-query-profile")] drop(shape_profile);
        let patch = {
            #[cfg(feature = "body-query-profile")] let _patch_profile=query_profile::guard(QueryStage::ObservationFootprintPatch);
            map.ground_patch(centre, radius)
        };
        #[cfg(feature = "body-query-profile")] let cells_profile=query_profile::guard(QueryStage::ObservationFootprintCells);
        let mut cells = std::collections::BTreeSet::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                if !polygon_intersects_cell(&polygon, x, y, size) { continue; }
                let column = (((x as f64 + 0.5) * size / planner_size).floor() as i32,
                    ((y as f64 + 0.5) * size / planner_size).floor() as i32);
                let ground = *patch.get(&column).ok_or(SurfaceError::MissingSupport)?;
                let height = map.cell_position(ground)[2];
                // 粗格跨坡面/阶边时，低侧真实细格可以属于同列较低的占据粗格。
                // 查询范围由一格内允许的坡差/阶差限制，并要求候选确在规范物理占据内。
                // 始终先选实际最高回波再查时效，旧顶部不能被新低面替代。
                let top = ((height / size).round() - 1.0) as i32;
                let height_span = (planner_size * std::f64::consts::SQRT_2
                    * f64::from(map.config().max_slope_deg).to_radians().tan())
                    .max(model.max_step_up_m).max(model.max_step_down_m)
                    + size + model.support_uncertainty_m;
                let count = (height_span / size).ceil().min(128.0) as i32;
                let bottom = top.saturating_sub(count);
                let z = (bottom..=top).rev().find(|&z| {
                    if observations.get([x,y,z]).is_none() { return false; }
                    let centre = (z as f64 + 0.5) * size;
                    let coarse_z = (centre / planner_size).floor() as i32;
                    map.data.occupied_columns.get(&column)
                        .is_some_and(|zs| zs.binary_search(&coarse_z).is_ok())
                }).unwrap_or(top);
                cells.insert([x, y, z]);
            }
        }
        #[cfg(feature = "body-query-profile")] {
            drop(cells_profile);
            query_profile::add(QueryCounter::ObservationFootprintCellsBuilt,cells.len() as u64);
        }
        Ok(cells)
    }


    fn polygon_footprint_fixture(size:f32,hole:bool)->SurfaceSnapshot {
        use std::collections::BTreeMap;
        let n=(1.2/f64::from(size)).ceil() as i32;
        let z=(0.1/f64::from(size)).round() as i32-1;
        let keys:Vec<_>=(-n..n).flat_map(|x|(-n..n).map(move|y|[x,y,z]))
            .filter(|key|!hole || !(key[0]==0&&key[1]==0)).collect();
        let points:Vec<_>=keys.iter().map(|key|((key[0]as f32+0.5)*size,(key[1]as f32+0.5)*size,(key[2]as f32+0.5)*size)).collect();
        let base=scene(&[],false);
        let map=SurfaceSnapshot::from_points(&points,base.config().clone(),size).unwrap().with_body_envelope(model()).unwrap();
        let mut blocks=BTreeMap::<_,BTreeMap<_,_>>::new();
        for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,
            SurfaceObservation{observed_ns:10_000_000_000,hits:2});}
        let mut observations=SurfaceObservations::new(size,32).unwrap();
        for (block,values) in blocks {observations.replace_block(block,values).unwrap();}
        observations.set_source_timestamp_ns(10_000_000_000);observations.mark_complete_geometry_source();
        map.with_observations(Some(Arc::new(observations)))
    }

    #[test]
    fn observation_polygon_once_matches_frozen_full_footprint_and_error_order() {
        let mut success=0;let mut failures=0;
        for size in [0.1,0.025,0.01] {
            for hole in [false,true] {
                let source=polygon_footprint_fixture(size,hole);
                for rounded in [false,true] {
                    let mut envelope=model();
                    if rounded {envelope.footprint_xy_m=(0..24).map(|i| {
                        let angle=i as f64*std::f64::consts::TAU/24.0;[0.3*angle.cos(),0.16*angle.sin()]
                    }).collect();}
                    let map=source.with_body_envelope(envelope).unwrap();
                    for x in [-0.1,0.05,f64::from_bits(0.1_f64.to_bits()-1),0.1,f64::from_bits(0.1_f64.to_bits()+1),2.0] {
                        for (yaw,pitch) in [(0.0_f64,0.0_f64),(0.37,0.0),(std::f64::consts::FRAC_PI_4,0.05),(-1.1,-0.04)] {
                            let q=[-(yaw/2.0).sin()*(pitch/2.0).sin(),(yaw/2.0).cos()*(pitch/2.0).sin(),
                                (yaw/2.0).sin()*(pitch/2.0).cos(),(yaw/2.0).cos()*(pitch/2.0).cos()];
                            let pose=([x,-0.05,0.5],q);
                            let old=reference_observation_footprint_rebuilding_polygon(&map,pose);
                            let new=map.body_observation_footprint(pose);
                            assert_eq!(new,old,"size={size} hole={hole} rounded={rounded} pose={pose:?}");
                            if new.is_ok(){success+=1;}else{failures+=1;}
                        }
                    }
                    let current=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
                    if let Ok(view)=map.with_current_body_support(current.0,current.1) {
                        assert_eq!(view.body_observation_footprint(current),reference_observation_footprint_rebuilding_polygon(&view,current));
                    }
                    for pose in [([0.05,0.05,0.5],[0.0;4]),([f64::NAN,0.0,0.5],[0.0,0.0,0.0,1.0])] {
                        assert_eq!(map.body_observation_footprint(pose),reference_observation_footprint_rebuilding_polygon(&map,pose));
                    }
                }
            }
        }
        assert!(success>100 && failures>0,"matrix must exercise successful complete cell sets and errors");
        let map=polygon_footprint_fixture(0.01,false);let mut huge=model();
        huge.footprint_xy_m=vec![[-2.0,-2.0],[2.0,-2.0],[2.0,2.0],[-2.0,2.0]];
        let huge=map.with_body_envelope(huge).unwrap();let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let old=reference_observation_footprint_rebuilding_polygon(&huge,pose);
        assert!(matches!(old,Err(SurfaceError::InvalidConfig(_))));
        assert_eq!(huge.body_observation_footprint(pose),old);
        let wrong=map.with_observations(Some(Arc::new(SurfaceObservations::new(0.02,32).unwrap())));
        assert_eq!(wrong.body_observation_footprint(pose),reference_observation_footprint_rebuilding_polygon(&wrong,pose));
    }

    #[test]
    #[ignore="matched warmed raw-footprint microbenchmark; no live control claim"]
    fn benchmark_observation_polygon_once() {
        use std::{hint::black_box,time::Instant};
        for size in [0.05,0.01] {
            let source=polygon_footprint_fixture(size,false);
            for rounded in [false,true] {
                let mut envelope=model();
                if rounded {envelope.footprint_xy_m=(0..24).map(|i| {let a=i as f64*std::f64::consts::TAU/24.0;[0.3*a.cos(),0.16*a.sin()]}).collect();}
                let map=source.with_body_envelope(envelope).unwrap();
                for yaw in [0.0_f64,0.37] {
                    let pose=([0.05,0.05,0.5],[0.0,0.0,(yaw/2.0).sin(),(yaw/2.0).cos()]);
                    let expected=reference_observation_footprint_rebuilding_polygon(&map,pose).unwrap();
                    assert_eq!(map.body_observation_footprint(pose).unwrap(),expected);
                    let mut old_ms=Vec::new();let mut new_ms=Vec::new();
                    for round in 0..6 {
                        for hoisted in if round%2==0 {[false,true]}else{[true,false]} {
                            let started=Instant::now();
                            for _ in 0..30 {
                                let map=black_box(&map);let pose=black_box(pose);
                                let cells=if hoisted {map.body_observation_footprint(pose)}else{reference_observation_footprint_rebuilding_polygon(map,pose)}.unwrap();
                                assert_eq!(black_box(&cells),&expected);
                            }
                            let elapsed=started.elapsed().as_secs_f64()*1e3;
                            if hoisted {new_ms.push(elapsed);}else{old_ms.push(elapsed);}
                        }
                    }
                    eprintln!("FOOTPRINT_POLYGON size={size} rounded={rounded} yaw={yaw} cells={} loops_per_round=30 warmed_by_parity_check=true alternating=true includes=full_raw_footprint_output_equality_and_retirement old_ms={old_ms:?} hoisted_ms={new_ms:?}",expected.len());
                }
            }
        }
    }

    #[test]
    fn ordered_observation_difference_preserves_full_xyz_order_and_failure_dependencies() {
        use std::collections::BTreeSet;
        for count in [0,1,7,63,257] {
            let cells:BTreeSet<_>=(0..count).map(|i|[i%17-8,i/17-8,i%3-1]).collect();
            for mode in 0..5 {
                let occupied:BTreeSet<_>=cells.iter().enumerate().filter_map(|(i,&cell)|match mode {
                    0=>None,1=>Some(cell),2=>(i%2==0).then_some(cell),
                    3=>Some([cell[0],cell[1],cell[2]+7]),_=>Some([cell[0]+100,cell[1],cell[2]]),
                }).collect();
                let old:Vec<_>=cells.iter().filter(|cell|!occupied.contains(*cell)).copied().collect();
                let new:Vec<_>=cells.difference(&occupied).copied().collect();
                assert_eq!(new,old,"fullXYZ ordered difference mode={mode} count={count}");
            }
        }
        let base=scene(&[],false);let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let original=base.with_observations(Some(observation_grid(|_|{})));
        let cells:Vec<_>=original.body_observation_footprint(pose).unwrap().into_iter().collect();
        assert!(cells.len()>=3);
        let map=base.with_observations(Some(observation_grid(|values| {
            values.get_mut(&cells[0]).unwrap().hits=1;
            values.get_mut(&cells[1]).unwrap().observed_ns=0;
            values.get_mut(&cells[2]).unwrap().observed_ns=0;
        })));
        for occupied in [BTreeSet::from([cells[1]]),BTreeSet::from([cells[0],cells[1]])] {
            let expected=map.validate_observed_body_motion_in_boundary(pose,pose,&occupied,10_500_000_000,1_000_000_000,&[]);
            let mut checker=map.observed_body_motion_checker_with_expiry(&occupied,1_000_000_000,&[]);
            assert_eq!(checker(pose,pose,10_500_000_000).map(|_|()),expected);
            assert_eq!(expected,Ok(()),"set difference is diagnostic, not a ground permission");
        }
        let wrong_height:BTreeSet<_>=cells.iter().map(|cell|[cell[0],cell[1],cell[2]+1]).collect();
        let mut checker=original.observed_body_motion_checker_with_expiry(&wrong_height,1_000_000_000,&[]);
        let expected=original.validate_observed_body_motion_in_boundary(pose,pose,&wrong_height,12_000_000_000,1_000_000_000,&[]);
        assert_eq!(checker(pose,pose,12_000_000_000).map(|_|()),expected);
        assert_eq!(expected,Ok(()));
        let map=base.with_observations(Some(observation_grid(|values| {
            values.get_mut(&cells[0]).unwrap().observed_ns=500;
            values.get_mut(&cells[1]).unwrap().observed_ns=0;
            values.get_mut(&cells[2]).unwrap().observed_ns=1000;
        })));
        let occupied=BTreeSet::from([cells[1]]);
        let max_age=u64::MAX-1000;
        let mut checker=map.observed_body_motion_checker_with_expiry(&occupied,max_age,&[]);
        assert_eq!(checker(pose,pose,10_500_000_000).unwrap(),None);
        assert!(map.validate_observed_body_motion_in_boundary(pose,pose,&occupied,10_500_000_000,max_age,&[]).is_ok());
        let occupied=BTreeSet::from([cells[0],cells[1]]);
        let mut checker=map.observed_body_motion_checker_with_expiry(&occupied,max_age,&[]);
        assert_eq!(checker(pose,pose,10_500_000_000).unwrap(),None,"exactMAX and overflow stay unbounded");
    }

    #[test]
    #[ignore="matched static evidence-iteration microbenchmark; no motion or throughput qualification"]
    fn benchmark_ordered_observation_difference() {
        use std::{collections::{BTreeMap,BTreeSet},hint::black_box,time::{Instant,SystemTime,UNIX_EPOCH}};
        fn check<'a>(cells:impl Iterator<Item=&'a [i32;3]>,observations:&SurfaceObservations,checked_ns:u64)->Result<Option<u64>,SurfaceError> {
            let mut minimum=None;
            for &cell in cells {
                let evidence=observations.get(cell).unwrap_or(SurfaceObservation{observed_ns:0,hits:0});
                let reason=if evidence.observed_ns==0 {Some("缺少真实观察")}
                    else if evidence.hits<2 {Some("独立采样命中不足")}
                    else if evidence.observed_ns>checked_ns.saturating_add(50_000_000) {Some("采样时间在未来")}
                    else if checked_ns.saturating_sub(evidence.observed_ns)>1_000_000_000 {Some("观察已过期")}
                    else {None};
                if let Some(reason)=reason {return Err(SurfaceError::NearFieldObservation{cell,reason,observed_ns:evidence.observed_ns,checked_ns});}
                if let Some(expiry)=evidence.observed_ns.checked_add(1_000_000_000).filter(|&expiry|expiry<u64::MAX) {
                    minimum=Some(minimum.map_or(expiry,|old:u64|old.min(expiry)));
                }
            }
            Ok(minimum)
        }
        let cells:BTreeSet<[i32;3]>=(-15..=15).flat_map(|x|(-15..=15).map(move|y|[x,y,9])).collect();
        let sample_ns=SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as u64;
        let mut blocks=BTreeMap::<_,BTreeMap<_,_>>::new();
        for &cell in &cells {blocks.entry(cell.map(|v|v.div_euclid(32))).or_default()
            .insert(cell,SurfaceObservation{observed_ns:sample_ns,hits:2});}
        let mut observations=SurfaceObservations::new(0.01,32).unwrap();
        for (block,values) in blocks {observations.replace_block(block,values).unwrap();}
        for (case,occupied) in [
            ("all_exempt",cells.clone()),
            ("near_complete_overlap",cells.iter().copied().filter(|cell|cell[0]<15).collect()),
            ("half_overlap",cells.iter().copied().filter(|cell|cell[0]<0).collect()),
            ("empty_exemption",BTreeSet::new()),
            ("large_disjoint_exemption",cells.iter().map(|cell|[cell[0]+100,cell[1],cell[2]]).collect()),
            ("interleaved_wrong_height",cells.iter().map(|cell|[cell[0],cell[1],cell[2]+1]).collect()),
            ("interleaved_low_overlap",cells.iter().enumerate().map(|(i,cell)|[cell[0],cell[1],cell[2]+i32::from(i%31!=0)]).collect()),
        ] {
            let expected=check(cells.iter().filter(|cell|!occupied.contains(*cell)),&observations,sample_ns).unwrap();
            assert_eq!(check(cells.difference(&occupied),&observations,sample_ns).unwrap(),expected);
            let mut old_ms=Vec::new();let mut new_ms=Vec::new();
            for round in 0..6 {
                for difference in if round%2==0 {[false,true]}else{[true,false]} {
                    let started=Instant::now();
                    for _ in 0..100 {
                        let (cells,occupied,observations)=black_box((&cells,&occupied,&observations));
                        let result=if difference {check(cells.difference(occupied),observations,sample_ns)}
                            else {check(cells.iter().filter(|cell|!occupied.contains(*cell)),observations,sample_ns)};
                        assert_eq!(black_box(result).unwrap(),expected);
                    }
                    let elapsed=started.elapsed().as_secs_f64()*1e3;
                    if difference {new_ms.push(elapsed);}else{old_ms.push(elapsed);}
                }
            }
            eprintln!("ORDERED_OBSERVATION case={case} cells={} exempt={} nonexempt={} loops_per_round=100 first_round=initial_after_parity_check later_rounds=alternating_order static_captured_anchor_ns={sample_ns} not_live_motion=true old_ms={old_ms:?} difference_ms={new_ms:?}",cells.len(),occupied.len(),cells.difference(&occupied).count());
        }
    }

    #[cfg(feature = "body-query-profile")]
    #[test]
    fn observed_profile_keeps_footprint_cache_and_geometry_only_compatibility() {
        let map=scene(&[],false).with_observations(Some(observation_grid(|_|{})));
        let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let occupied=map.body_observation_footprint(pose).unwrap();
        let mut cache=ObservationFootprintCache::new(&map);
        query_profile::begin();
        assert_eq!(cache.cells(pose).unwrap(),occupied.iter().copied().collect::<Vec<_>>().as_slice());
        assert_eq!(cache.cells(pose).unwrap(),occupied.iter().copied().collect::<Vec<_>>().as_slice());
        let geometry=query_profile::end();
        assert_eq!(geometry.counters[QueryCounter::ObservationFootprintCacheMisses as usize],1);
        assert_eq!(geometry.counters[QueryCounter::ObservationFootprintCacheHits as usize],1);
        assert_eq!(geometry.counters[QueryCounter::ObservationFootprintCellsBuilt as usize],occupied.len() as u64);
        assert_eq!(geometry.calls[QueryStage::ObservationFootprint as usize],1);
        fn assert_no_observation_lease(record: &query_profile::Record) {
            for counter in [QueryCounter::ObservationInputCells,
                QueryCounter::ObservationVisitedCells, QueryCounter::ObservationCheckedCells] {
                assert_eq!(record.counters[counter as usize],0);
            }
            assert_eq!(record.calls[QueryStage::ObservationEvidence as usize],0);
        }
        let none=std::collections::BTreeSet::new();
        for legacy_occupied in [&occupied,&none] {
            for (max_age_ns,checked_ns) in [(0,0),(1_000_000_000,12_000_000_000)] {
                // Cold independent geometry: neither the old exemption set nor
                // a future/expired observation time grants or rejects clearance.
                let cold=map.fork_for_planning();
                let mut checker=cold.observed_body_motion_checker_with_expiry(legacy_occupied,max_age_ns,&[]);
                query_profile::begin();
                let result=checker(pose,pose,checked_ns);
                let geometry_only=query_profile::end();
                assert_eq!(result,Ok(None),"compatibility age is not a support permission or lease");
                assert_no_observation_lease(&geometry_only);
            }
        }
        // Real occupied geometry still rejects, even with the legacy occupied
        // set supplied and an unbounded checked time. Preserve the real witness.
        let blocked=scene(&[(0.55,0.05,0.55)],false);
        let mut checker=blocked.observed_body_motion_checker_with_expiry_and_witness(&occupied,0,&[]);
        let mut witness=None;
        query_profile::begin();
        let result=checker(pose,pose,u64::MAX,&mut witness);
        let collision=query_profile::end();
        assert_eq!(result,Err(SurfaceError::InsufficientClearance));
        assert_eq!(witness.map(|rejected|rejected.pose),Some(pose));
        assert!(matches!(witness,Some(BodyFailureWitness {
            rejection:BodyRejection::Occupancy {source:OccupancySource::CoarseFallback,..},..
        })));
        assert_no_observation_lease(&collision);
    }

    #[test]
    fn observation_footprint_cache_is_exact_single_entry_and_borrows_hits() {
        let map = scene(&[], false).with_observations(Some(observation_grid(|_| {})));
        let pose = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let mut cache = ObservationFootprintCache::new(&map);
        let first = cache.cells(pose).unwrap().first().unwrap() as *const [i32; 3];
        assert_eq!(cache.builds, 1);
        let repeated = cache.cells(pose).unwrap().first().unwrap() as *const [i32; 3];
        assert_eq!(first, repeated, "命中必须借用原集合中的同一元素，不能克隆足迹");
        assert_eq!(cache.builds, 1);
        let words = [pose.0[0], pose.0[1], pose.0[2], pose.1[0], pose.1[1], pose.1[2], pose.1[3]];
        for index in 0..7 {
            let mut changed = words;
            changed[index] = if changed[index] == 0.0 { -0.0 }
                else { f64::from_bits(changed[index].to_bits() + 1) };
            let other = ([changed[0], changed[1], changed[2]],
                [changed[3], changed[4], changed[5], changed[6]]);
            assert_eq!(cache.cells(other).unwrap(), map.body_observation_footprint(other).unwrap().into_iter().collect::<Vec<_>>().as_slice());
            assert_eq!(cache.builds, 2 + index * 2, "全部七个输入位模式均属于键");
            cache.cells(other).unwrap();
            assert_eq!(cache.builds, 2 + index * 2);
            cache.cells(pose).unwrap();
            assert_eq!(cache.builds, 3 + index * 2, "另一位姿必须淘汰唯一旧项");
        }
        let missing = ([100.0, 100.0, 0.5], pose.1);
        let before = cache.builds;
        assert!(cache.cells(missing).is_err());
        assert!(cache.cells(missing).is_err());
        assert_eq!(cache.builds, before + 2, "失败不能进入缓存");
        cache.cells(pose).unwrap();
        assert_eq!(cache.builds, before + 2, "失败不替换最后一个成功足迹");
    }

    #[test]
    fn observation_footprint_cache_reuses_adjacent_engine_samples() {
        let map = scene(&[], false).with_observations(Some(observation_grid(|_| {})));
        let poses: Vec<_> = (0..8).map(|index| (
            [0.051 + f64::from(index) * 0.005, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0],
        )).collect();
        let mut cache = ObservationFootprintCache::new(&map);
        let mut queries = 0;
        for pair in poses.windows(2) {
            map.validate_body_motion_checked(pair[0], pair[1], &[], |pose| {
                queries += 1;
                assert_eq!(cache.cells(pose)?, map.body_observation_footprint(pose)?.into_iter().collect::<Vec<_>>().as_slice());
                Ok(())
            }).unwrap();
        }
        // 只计原采样顺序下的几何构建量；不作为耗时或观察许可的资格证明。
        eprintln!("observation footprint geometry work: queries={queries} builds={} hits={}",
            cache.builds, queries - cache.builds);
        assert!(queries >= 2 * (poses.len() - 1));
        assert!(cache.builds < queries, "邻段共享的真实端点应复用同一个精确足迹");
    }

    #[test]
    fn observed_motion_checker_rechecks_time_and_preserves_evidence_error_fields() {
        let base = scene(&[], false);
        let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let to = ([0.45, 0.05, 0.5], from.1);
        let good = base.with_observations(Some(observation_grid(|_| {})));
        let occupied = good.body_observation_footprint(from).unwrap();
        let mut checker = good.observed_body_motion_checker(&occupied, 1_000_000_000, &[]);
        checker(to, to, 10_100_000_000).unwrap();
        let expired = checker(to, to, 12_000_000_000);
        assert_eq!(expired, good.validate_observed_body_motion_in_boundary(
            to, to, &occupied, 12_000_000_000, 1_000_000_000, &[]));
        assert_eq!(expired,Ok(()),"historical floor age does not invalidate geometry");
        checker(to, to, 10_100_000_000).unwrap();
        checker(from, from, 12_000_000_000).unwrap();
        for kind in 0..8 {
            let map = base.with_observations(Some(observation_grid(|cells| {
                let evidence = cells.get_mut(&[8, 0, 0]).unwrap();
                match kind {
                    0 => {},
                    1 => evidence.hits = 0,
                    2 => evidence.hits = 1,
                    3 => evidence.observed_ns = 0,
                    4 => evidence.observed_ns = 1,
                    5 => evidence.observed_ns = 11_000_000_000,
                    6 => { cells.remove(&[8, 0, 0]); },
                    _ => {
                        evidence.observed_ns = 1;
                        cells.insert([8, 0, -1], SurfaceObservation { observed_ns: 10_000_000_000, hits: 3 });
                    },
                }
            })));
            assert_observation_checker_matches(&map, &occupied, 1_000_000_000, &[], &[
                (to, to, 10_100_000_000), (to, to, 10_100_000_001),
                (from, to, 10_100_000_000), (to, from, 12_000_000_000),
                (from, from, 12_000_000_000), (to, to, u64::MAX),
            ]);
        }
        // 未来容差和年龄边界均在同一足迹命中时重新计算。
        let none = Default::default();
        assert_observation_checker_matches(&good, &none, 1_000_000_000, &[], &[
            (to, to, 9_949_999_999), (to, to, 9_950_000_000),
            (to, to, 11_000_000_000), (to, to, 11_000_000_001),
        ]);
    }

    #[test]
    fn observed_motion_checker_preserves_missing_source_zero_time_and_first_error_order() {
        let base = scene(&[], false);
        let good = base.with_observations(Some(observation_grid(|_| {})));
        let wrong_resolution = base.with_observations(Some(Arc::new(SurfaceObservations::new(0.05, 32).unwrap())));
        let pose = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let invalid = (pose.0, [0.0; 4]);
        let occupied = good.body_observation_footprint(pose).unwrap();
        for map in [&base, &good, &wrong_resolution] {
            for max_age in [0, 1_000_000_000] {
                for boundary in [vec![], vec![[0.0, 0.0], [1.0, 0.0]]] {
                    assert_observation_checker_matches(map, &occupied, max_age, &boundary, &[
                        (pose, pose, 0), (invalid, pose, 0),
                        (pose, pose, 10_100_000_000), (invalid, pose, 10_100_000_000),
                        (pose, invalid, 10_100_000_000),
                    ]);
                }
            }
        }
        let stale = base.with_observations(Some(observation_grid(|cells| {
            cells.get_mut(&[-4, 0, 0]).unwrap().observed_ns = 1;
            cells.get_mut(&[8, 0, 0]).unwrap().hits = 1;
        })));
        let none = Default::default();
        let to = ([0.45, 0.05, 0.5], pose.1);
        for boundary in [vec![], vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]]] {
            assert_observation_checker_matches(&stale, &none, 1_000_000_000, &boundary, &[
                (pose, to, 10_100_000_000), (to, pose, 10_100_000_000),
                (pose, to, 12_000_000_000),
            ]);
        }
    }

    #[test]
    fn observed_motion_checker_keeps_geometry_rotation_boundary_and_view_contexts() {
        let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let to = ([0.45, 0.05, 0.5], from.1);
        let turn = (from.0, [0.0, 0.0, std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2]);
        let map = scene(&[], false).with_observations(Some(observation_grid(|_| {})));
        let occupied = map.body_observation_footprint(from).unwrap();
        let view = map.with_current_body_support(from.0, from.1).unwrap();
        let fork = map.fork_for_planning();
        let obstacle = scene(&[(0.85, 0.05, 0.4)], false).with_observations(Some(observation_grid(|_| {})));
        let blocked = map.with_virtual_obstacles([-4.0, -4.0], 0.1, 80, 80, &vec![true; 6400]).unwrap();
        for context in [&map, &view, &fork, &obstacle, &blocked] {
            for boundary in [vec![], vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]]] {
                assert_observation_checker_matches(context, &occupied, 1_000_000_000, &boundary, &[
                    (from, to, 10_100_000_000), (to, to, 10_100_000_000),
                    (from, turn, 10_100_000_000), (turn, to, 10_100_000_000),
                    (to, from, 12_000_000_000), (from, from, 12_000_000_000),
                ]);
            }
        }
        // 相同输入位姿的不同观察源不能共享几何选择或许可。
        let missing = scene(&[], false).with_observations(Some(observation_grid(|cells| { cells.remove(&[8,0,0]); })));
        let mut fresh_checker = map.observed_body_motion_checker(&occupied, 1_000_000_000, &[]);
        let mut missing_checker = missing.observed_body_motion_checker(&occupied, 1_000_000_000, &[]);
        fresh_checker(to, to, 10_100_000_000).unwrap();
        assert!(missing_checker(to, to, 10_100_000_000).is_ok());
        fresh_checker(to, to, 10_100_000_000).unwrap();
    }

    #[test]
    fn observed_body_motion_checks_intermediate_sweep_and_new_rotation_space() {
        let map=scene(&[],false).with_observations(Some(observation_grid(|cells| {
            cells.get_mut(&[10,0,0]).unwrap().observed_ns=1;
            cells.get_mut(&[0,4,0]).unwrap().observed_ns=1;
        })));
        let from=([0.05,0.05,0.5],[0.,0.,0.,1.]);let to=([1.85,0.05,0.5],from.1);
        let occupied=map.body_observation_footprint(from).unwrap();
        map.validate_observed_body_motion(from,to,&occupied,12_000_000_000,1).unwrap();
        let middle=map.with_recent_obstacle_points(1,100,0,&[],[(0,[1.05,0.05,0.55])]).unwrap();
        middle.validate_body_pose(from.0,from.1).unwrap();middle.validate_body_pose(to.0,to.1).unwrap();
        assert!(middle.validate_observed_body_motion(from,to,&occupied,12_000_000_000,1).is_err());
        let turn=(from.0,[0.,0.,std::f64::consts::FRAC_1_SQRT_2,std::f64::consts::FRAC_1_SQRT_2]);
        let side=map.with_recent_obstacle_points(1,100,0,&[],[(0,[0.05,0.45,0.55])]).unwrap();
        side.validate_body_pose(from.0,from.1).unwrap();
        assert!(side.validate_observed_body_motion(from,turn,&occupied,12_000_000_000,1).is_err());
    }
    #[test]
    fn observations_do_not_cross_vertical_layers() {
        let upper: Vec<_> = (-30..30).flat_map(|x| (-30..30).map(move |y| ((x as f32+0.5)*0.1, (y as f32+0.5)*0.1, 0.825))).collect();
        let map = scene(&upper, false).with_observations(Some(observation_grid(|_| {})));
        let from = ([0.05, 0.05, 1.3], [0.0, 0.0, 0.0, 1.0]);
        let to = ([0.45, 0.05, 1.3], from.1);
        let occupied = map.body_observation_footprint(from).unwrap();
        map.validate_body_motion(from, to, &[]).unwrap();
        let lower=map.locate_support([0.05,0.05,0.1]).unwrap();
        let upper=map.locate_support([0.05,0.05,0.9]).unwrap();
        assert_ne!(lower.cell,upper.cell);assert!(upper.surface_xyz[2]-lower.surface_xyz[2]>0.7);
        assert!(map.validate_observed_body_motion(from, to, &occupied, 10_100_000_000, 1_000_000_000).is_ok());
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
        assert_observation_checker_matches(&map, &occupied, 1_000_000_000, &[], &[
            (from, to, 10_100_000_000), (to, to, 10_100_000_000),
            (to, to, 12_000_000_000), (to, from, 10_100_000_000),
        ]);
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
        let mut blocks: std::collections::BTreeMap<_, std::collections::BTreeMap<_, _>> = Default::default();
        let fine = 0.025_f32;
        for &(x,y,z) in &points {
            let key = [x,y,z].map(|v| (f64::from(v) / f64::from(fine)).floor() as i32);
            blocks.entry(key.map(|v| v.div_euclid(32))).or_default().insert(key,
                SurfaceObservation { observed_ns: 10_000_000_000, hits: 2 });
        }
        let mut observations = SurfaceObservations::new(fine, 32).unwrap();
        for (key, block) in blocks { observations.replace_block(key, block).unwrap(); }
        let observed = map.with_observations(Some(Arc::new(observations)));
        let occupied = observed.body_observation_footprint(poses[25]).unwrap();
        let queries: Vec<_> = poses[25..=65].windows(2).step_by(5).flat_map(|pair| [
            (pair[0], pair[1], 10_100_000_000), (pair[1], pair[1], 10_100_000_000),
            (pair[1], pair[0], 12_000_000_000),
        ]).collect();
        assert_observation_checker_matches(&observed, &occupied, 1_000_000_000, &[], &queries);
        let floating=([-0.8,0.0,0.55],[0.0,0.0,0.0,1.0]);
        map.validate_body_pose(floating.0,floating.1).unwrap();
        assert!(map.query_ground([floating.0[0],floating.0[1]],0.025).height.unwrap()<floating.0[2]-0.4,
            "clearance preserves the actual floating pose rather than snapping it to terrain");
        let missing:Vec<_>=points.into_iter().filter(|p|!(p.0>0.0&&p.0<0.15)).collect();
        let hole=SurfaceSnapshot::from_points(&missing,cfg,0.025).unwrap().with_body_envelope(envelope).unwrap();
        let middle=poses[45];hole.validate_body_pose(middle.0,middle.1).unwrap();
        assert!(hole.query_ground([0.075,0.0],0.025).height.is_none(),"a separating strip is not an enclosing same-plane blind disk");
    }
    fn assert_witness_cold_and_warm(map:&SurfaceSnapshot,pose:([f64;3],[f64;4]),expected:Option<BodyFailureWitness>) {
        let cold=map.fork_for_planning();
        assert!(cold.body_query_cache.lock().unwrap().poses.is_empty());
        let error=expected.unwrap().rejection.error();let mut slot=None;
        assert_eq!(cold.validate_body_pose_with_witness(pose.0,pose.1,&mut slot),Err(error.clone()));
        assert_eq!(slot,expected,"cold source witness");
        assert_eq!(cold.validate_body_pose(pose.0,pose.1),Err(error.clone()));
        assert_eq!(cold.validate_body_pose_with_witness(pose.0,pose.1,&mut slot),Err(error));
        assert_eq!(slot,expected,"warm witness after legacy caller");
        eprintln!("COLD_WARM_MATCH {slot:?}");
    }

    #[test]
    fn body_witness_unknown_ground_is_not_collision_and_real_collision_cache_is_exact() {
        let q=[0.,0.,0.,1.];let body=[0.05,0.05,0.5];
        let points:Vec<_>=(-30..30).flat_map(|x|(-30..30).filter_map(move|y|
            (x!=1||y!=0).then_some(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.025)))).collect();
        let map=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.1).unwrap().with_body_envelope(model()).unwrap();
        let mut witness=None;
        for _ in 0..2 {map.validate_body_pose_with_witness(body,q,&mut witness).unwrap();assert!(witness.is_none());}
        assert_eq!(map.body_query_cache.lock().unwrap().poses.len(),1);
        let outside=[20.05,0.05,0.5];
        map.validate_body_pose_with_witness(outside,q,&mut witness).unwrap();assert!(witness.is_none());
        assert_eq!(map.query_ground([outside[0],outside[1]],0.1).state,GroundQueryState::Unknown);
        let blocked=map.with_recent_obstacle_points(1,100,0,&[],[(0,[0.05,0.05,0.55])]).unwrap();
        assert!(blocked.validate_body_pose_with_witness(body,q,&mut witness).is_err());
        assert_witness_cold_and_warm(&blocked,(body,q),witness);
        map.validate_body_pose_with_witness(body,q,&mut witness).unwrap();assert!(witness.is_none());
    }

    fn witnessed_observations(points:&[(f32,f32,f32)],size:f32)->Arc<SurfaceObservations> {
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(size)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
        let mut grid=SurfaceObservations::new(size,32).unwrap();
        for (key,values) in blocks {grid.replace_block(key,values).unwrap();}
        Arc::new(grid)
    }

    #[test]
    fn body_witness_partial_support_is_not_collision_free() {
        let mut points:Vec<_>=(-30..30).flat_map(|x|(-30..30).filter_map(move |y|
            if x==4&&y==0 {None}else{Some(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.0875))})).collect();
        points.push((-0.45,0.1875,0.55));
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.55,-0.12],[0.55,-0.12],[0.55,0.12],[-0.55,0.12]];
        let mut observed=witnessed_observations(&points,0.025).as_ref().clone();
        observed.mark_complete_geometry_source(); // both representations use these exact complete points
        let map=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.025).unwrap()
            .with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observed)));
        let pose=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);let mut slot=None;
        map.validate_body_pose_with_witness(pose.0,pose.1,&mut slot).unwrap();
        assert!(slot.is_none(),"a nearby nonintersecting obstacle and missing floor do not prove collision");
        let blocked=map.with_recent_obstacle_points(1,100,0,&[],[(0,[-0.45,0.15,0.55])]).unwrap();
        assert_eq!(blocked.validate_body_pose_with_witness(pose.0,pose.1,&mut slot),Err(SurfaceError::InsufficientClearance));
        assert_witness_cold_and_warm(&blocked,pose,slot);
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{..}));
    }

    #[test]
    fn sparse_high_observation_cannot_replace_ground_or_seed_contact_response() {
        let q=[0.0,0.0,0.0,1.0];let pose=([0.05,0.05,0.5],q);
        let coarse=scene(&[(0.55,0.05,0.55)],false);
        let fine=coarse.with_observations(Some(witnessed_observations(&[(0.55,0.05,0.55)],0.05)));
        let observations=fine.observations.as_ref().unwrap();
        assert!(!observations.is_complete_geometry_source());
        assert_eq!(observations.column_levels(11,1),Some(&[11][..]));
        assert_eq!(observations.get([11,1,1]),None,"原地面仅存在于保留的粗点云，不伪造细地面");
        let centre=fine.locate_support_from_body(pose.0,q,0.4).unwrap();
        let patch=fine.ground_patch(centre,1.0);let ground=patch[&(5,0)];
        assert_eq!(ground,(5,0,0));
        assert!((fine.cell_position(ground)[2]-0.1).abs()<1e-7);
        let support=[pose.0[0],pose.0[1],pose.0[2]-0.4];
        let domain=fine.fine_contact_domain(support,0.7);
        let frame=fine.body_contact_frame(centre,support[2],0.0);
        let response=fine.body_pose_after_motion(pose,[0.051,0.05],0.0);
        eprintln!("SPARSE_SUPPORT ground={ground:?} bounds=[{},{}] fine_levels={:?} fine_bands={:?} domain={domain:?} frame={frame:?} response={response:?}",
            fine.observed_support_lower_bound(ground),fine.cell_position(ground)[2],observations.column_levels(11,1),fine.fine_contact_bands(11,1));
        assert_eq!(domain,Err(SurfaceError::MissingSupport));
        assert_eq!(frame,Err(SurfaceError::MissingSupport));
        assert_eq!(response,Err(SurfaceError::InsufficientClearance),"missing response cannot conceal the actual high obstacle");
        let mut witness=None;
        assert_eq!(fine.validate_body_pose_with_witness(pose.0,q,&mut witness),Err(SurfaceError::InsufficientClearance));
        eprintln!("SPARSE_COLLISION {witness:?}");
        assert!(matches!(witness.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::Fine,cell:[11,1,11],..}));
        assert!(observations.get([11,1,11]).is_some());
    }

    mod shared_ground_tests {include!("body_shared_ground_tests.rs");}
    mod fine_band_cache_tests {include!("body_fine_band_cache_tests.rs");}
    mod fine_geometry_tests {include!("body_fine_geometry_tests.rs");}

    #[test]
    fn body_witness_occupancy_and_motion_cache_preserve_rejected_sample() {
        let q=[0.0,0.0,0.0,1.0];let pose=([0.05,0.05,0.5],q);
        let map=scene(&[(0.55,0.05,0.55)],false);let mut slot=None;
        assert_eq!(map.validate_body_pose(pose.0,q),Err(SurfaceError::InsufficientClearance));
        assert_eq!(map.validate_body_pose_with_witness(pose.0,q,&mut slot),Err(SurfaceError::InsufficientClearance));
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::CoarseFallback,..}));
        assert_witness_cold_and_warm(&map,pose,slot);
        let fine=map.with_observations(Some(witnessed_observations(&[(0.55,0.05,0.55)],0.05)));
        assert_eq!(fine.validate_body_pose_with_witness(pose.0,q,&mut slot),Err(SurfaceError::InsufficientClearance));
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::Fine,..}));
        assert_witness_cold_and_warm(&fine,pose,slot);
        let map=ground_band_sweep_scene(0.525);
        let from=([-0.25,0.0295,0.375],q);let to=([0.06,0.3395,0.375],q);
        map.validate_body_pose(from.0,q).unwrap();map.validate_body_pose(to.0,q).unwrap();
        assert_eq!(map.validate_body_motion(from,to,&[]),Err(SurfaceError::InsufficientClearance));
        assert_eq!(map.validate_body_motion_with_witness(from,to,&[],&mut slot),Err(SurfaceError::InsufficientClearance));
        let witness=slot.unwrap();assert_ne!(witness.pose,from);assert_ne!(witness.pose,to);
        assert_eq!(witness.pose.0[2],0.375);
        let cold=map.fork_for_planning();
        let mut cold_slot=None;
        assert_eq!(cold.validate_body_motion_with_witness(from,to,&[],&mut cold_slot),Err(SurfaceError::InsufficientClearance));
        assert_eq!(cold_slot,slot);
        assert_eq!(map.body_query_cache.lock().unwrap().motions.len(),1);
        assert_eq!(map.validate_body_motion_with_witness(from,from,&[],&mut slot),Ok(()));
        assert!(slot.is_none());
    }

    #[test]
    fn body_witness_fine_fringe_keeps_the_actual_outside_coarse_column() {
        let mut points:Vec<_>=(-30..30).flat_map(|x|(-30..30).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.075))).collect();
        points.push((0.075,0.025,0.525));
        let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.0;
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(model()).unwrap()
            .with_observations(Some(witnessed_observations(&[(0.075,0.025,0.525)],0.05)));
        let pose=([f64::from(0.1_f32)+0.55,0.05,0.5],[0.0,0.0,0.0,1.0]);let mut slot=None;
        assert_eq!(map.validate_body_pose_with_witness(pose.0,pose.1,&mut slot),Err(SurfaceError::InsufficientClearance));
        eprintln!("FINE_FRINGE {slot:?}");
        assert_witness_cold_and_warm(&map,pose,slot);
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::FineFringe,cell:[1,0,10],..}));
    }

    #[test]
    fn body_witness_support_fallback_success_discards_initial_failure() {
        let map=scene(&[],true);let body=[0.05,0.05,0.5];let q=[0.0,0.0,0.0,1.0];let mut slot=None;
        map.validate_body_pose_with_witness(body,q,&mut slot).unwrap();
        assert!(slot.is_none(),"sparse ground does not produce a collision witness");
        let view=map.body_query_with_current_support_and_witness(body,q,&mut slot).unwrap();
        assert!(slot.is_none());
        view.validate_body_motion_with_witness((body,q),(body,q),&[],&mut slot).unwrap();
        assert!(slot.is_none());
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
    fn planning_fork_shares_payloads_but_isolates_every_surface_cache_and_rebase() {
        let pose = ([0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let base = scene(&[], true)
            .with_virtual_obstacles([-4.0, -4.0], 0.1, 80, 80, &vec![false; 6400]).unwrap();
        let view = base.with_current_body_support(pose.0, pose.1).unwrap();
        // Fork construction and all private cache access must work while every
        // controller-side mutex is held, without threads, sleeps or timing claims.
        let _query = view.body_query_cache.lock().unwrap();
        let _body = view.data.body_cache.lock().unwrap();
        let _routing = view.data.routing_cache.lock().unwrap();
        let _frontier = view.data.frontier_cache.lock().unwrap();
        let fork = view.fork_for_planning();
        for (source, private) in [(&view, &fork)] {
            assert!(!source.same_geometry(private));
            assert!(!source.same_body_query_view(private));
            assert!(source.same_body_query_view(&source.clone()));
            assert!(!Arc::ptr_eq(&source.body_query_cache, &private.body_query_cache));
            assert!(Arc::ptr_eq(&source.data.graph, &private.data.graph));
            assert!(Arc::ptr_eq(&source.data.physical_graph, &private.data.physical_graph));
            assert!(Arc::ptr_eq(&source.data.regions, &private.data.regions));
            assert!(Arc::ptr_eq(&source.data.occupied_columns, &private.data.occupied_columns));
            assert!(Arc::ptr_eq(&source.data.height_bounds, &private.data.height_bounds));
            assert!(Arc::ptr_eq(&source.data.virtual_blocked_cells, &private.data.virtual_blocked_cells));
            assert!(Arc::ptr_eq(source.data.virtual_obstacles.as_ref().unwrap(), private.data.virtual_obstacles.as_ref().unwrap()));
            assert!(Arc::ptr_eq(source.data.body_envelope.as_ref().unwrap(), private.data.body_envelope.as_ref().unwrap()));
            assert_eq!(source.ground_evidence, private.ground_evidence);
            assert_eq!(source.data.occupied_voxels, private.data.occupied_voxels);
            assert!(private.body_query_cache.try_lock().is_ok());
            assert!(private.data.body_cache.try_lock().is_ok());
            assert!(private.data.routing_cache.try_lock().is_ok());
            assert!(private.data.frontier_cache.try_lock().is_ok());
        }
        fork.validate_body_pose(pose.0, pose.1).unwrap();
        fork.with_current_body_support(pose.0, pose.1).unwrap();
        super::super::centering::distances(&fork);
    }

    #[test]
    fn planning_fork_preserves_query_only_and_occupied_volume_flags() {
        let base = scene(&[], false).with_ground_evidence(GroundEvidence::OccupiedVolume);
        let mut config = base.config().clone();
        config.wall_clearance_m = 0.0;
        let source = SurfaceSnapshot::from_points(&[(0.05,0.05,0.025)], config, 0.1).unwrap()
            .with_ground_evidence(GroundEvidence::OccupiedVolume);
        let query = source.clone();
        let fork = query.fork_for_planning();
        assert_eq!(fork.ground_evidence(), GroundEvidence::OccupiedVolume);
        assert!(Arc::ptr_eq(&query.data.graph, &fork.data.graph));
        assert_eq!(query.locate_support([0.05,0.05,0.1]), fork.locate_support([0.05,0.05,0.1]));
    }

    #[test]
    fn planning_fork_preserves_body_observation_obstacle_and_boundary_decisions() {
        let from = ([0.05, 0.05, 0.5], [0.0, 0.0, 0.0, 1.0]);
        let to = ([0.45, 0.05, 0.5], from.1);
        for extra in [vec![], vec![(0.45, 0.05, 0.4)]] {
            let base = scene(&extra, false);
            for (_observation_kind, observations) in [None, Some(observation_grid(|_| {})), Some(observation_grid(|cells| {cells.remove(&[8,0,0]);}))].into_iter().enumerate() {
                let map = base.with_observations(observations);
                let fork = map.fork_for_planning();
                assert_eq!(map.validate_body_pose(from.0, from.1), fork.validate_body_pose(from.0, from.1));
                assert_eq!(map.locate_support([0.05,0.05,0.1]), fork.locate_support([0.05,0.05,0.1]));
                let occupied = base.with_observations(Some(observation_grid(|_| {})))
                    .body_observation_footprint(from).unwrap();
                for now in [10_100_000_000, 12_000_000_000] {
                    for boundary in [vec![], vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]]] {
                        let expected = map.validate_observed_body_motion_in_boundary(from,to,&occupied,now,1_000_000_000,&boundary);
                        assert_eq!(expected, fork.validate_observed_body_motion_in_boundary(from,to,&occupied,now,1_000_000_000,&boundary));
                        if extra.is_empty() {
                            if boundary.is_empty() {
                                assert!(expected.is_ok(), "fresh observed motion must pass");
                            } else {
                                assert!(expected.is_err(), "external boundary still rejects irrespective of evidence age");
                            }
                        }
                    }
                }
                let blocked = map.with_virtual_obstacles([-4.0,-4.0],0.1,80,80,&vec![true;6400]).unwrap();
                assert_eq!(blocked.validate_body_pose(from.0,from.1), blocked.fork_for_planning().validate_body_pose(from.0,from.1));
            }
        }
    }

    fn ground_band_sweep_scene(obstacle_height: f32) -> SurfaceSnapshot {
        let mut points: Vec<_> = (-20..20).flat_map(|x| (-6..6).flat_map(move |y| {
            [-0.025, 0.025].into_iter().map(move |z| {
                ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z)
            })
        })).collect();
        points.push((0.05, 0.05, obstacle_height));
        let mut config = scene(&[], false).config().clone();
        config.robot_height = 0.1;
        config.wall_clearance_m = 0.0;
        let mut envelope = model();
        envelope.footprint_xy_m = vec![[-0.1,-0.08],[0.1,-0.08],[0.1,0.08],[-0.1,0.08]];
        SurfaceSnapshot::from_points(&points, config, 0.05).unwrap()
            .with_body_envelope(envelope).unwrap()
    }

    #[test]
    fn fine_observed_rigid_low_obstacle_keeps_original_sweep_rejection() {
        // Same observations and model as ground_band_sweep_scene(0.275), now
        // supplying their fine source so the extra-band contact path is exercised.
        let mut points:Vec<_>=(-20..20).flat_map(|x|(-6..6).flat_map(move|y|
            [-0.025,0.025].into_iter().map(move|z|
                ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,z)))).collect();
        points.push((0.05,0.05,0.275));
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
                .insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
        let mut observed=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observed.replace_block(key,values).unwrap();}
        observed.mark_complete_geometry_source();
        let map=ground_band_sweep_scene(0.275).with_observations(Some(Arc::new(observed)));
        let q=[0.0,0.0,0.0,1.0];let from=([-0.65,0.05,0.375],q);let to=([0.65,0.05,0.375],q);
        map.validate_body_pose(from.0,q).unwrap();map.validate_body_pose(to.0,q).unwrap();
        let next=map.body_pose_after_motion(from,[to.0[0],to.0[1]],0.0).unwrap();
        assert!((next.0[2]-from.0[2]).abs()<1e-10);
        let mut slot=None;
        assert_eq!(map.validate_body_pose_with_witness([0.05,0.05,0.375],q,&mut slot),Err(SurfaceError::InsufficientClearance));
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::Fine|OccupancySource::FineFringe,..}));
        eprintln!("FINE_RIGID_OBSTACLE witness={slot:?}");
        assert_eq!(map.validate_body_motion(from,to,&[]),Err(SurfaceError::InsufficientClearance));
        assert_eq!(map.validate_body_motion(from,next,&[]),Err(SurfaceError::InsufficientClearance));
    }

    // 单层真实细采样令拟合在固定XY处唯一，不用厚带参考变化解释姿态搬运误差。
    fn biased_sloped_contact_fixture() -> (SurfaceSnapshot,([f64;3],[f64;4])) {
        let fine=0.05_f32;
        let points:Vec<_>=(-35..35).flat_map(|x|(-35..35).map(move|y| {
            let px=(x as f32+0.5)*fine;let py=(y as f32+0.5)*fine;
            (px,py,0.12*px+0.06*py)
        })).collect();
        let mut config=scene(&[],false).config().clone();config.wall_clearance_m=0.0;config.robot_height=0.1;
        let mut model=model();model.footprint_xy_m=vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]];
        model.max_cross_slope_deg=25.0;
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(fine)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
        let mut observations=SurfaceObservations::new(fine,32).unwrap();
        for(key,values)in blocks{observations.replace_block(key,values).unwrap();}observations.mark_complete_geometry_source();
        let map=SurfaceSnapshot::from_points(&points,config,fine).unwrap().with_body_envelope(model).unwrap()
            .with_observations(Some(Arc::new(observations)));
        let centre=map.locate_near_support([0.025,0.025,0.0],0.3).unwrap();
        let(height,ground)=map.body_contact_frame(centre,0.0,0.35).unwrap();
        let(sr,cr)=(0.10_f64*0.5).sin_cos();let(sp,cp)=(-0.07_f64*0.5).sin_cos();
        let bias=rotation([sr*cp,cr*sp,-sr*sp,cr*cp]).unwrap();let r=rotation(ground).unwrap();
        let actual=quaternion_from_columns(bias.map(|axis|world(r,[0.0;3],axis)));
        let body=[0.025,0.025,height+0.4/rotation(actual).unwrap()[2][2]];
        map.validate_body_pose(body,actual).unwrap();(map,(body,actual))
    }
    fn relative_up_on_ground(map:&SurfaceSnapshot,pose:([f64;3],[f64;4]))->[f64;3] {
        let actual=rotation(pose.1).unwrap();let yaw=actual[0][1].atan2(actual[0][0]);
        let height=pose.0[2]-map.body_envelope().unwrap().body_to_support_z_m/actual[2][2];
        let centre=map.locate_near_support([pose.0[0],pose.0[1],height],0.3).unwrap();
        let(_,ground)=map.body_contact_frame(centre,height,yaw).unwrap();
        rotation(ground).unwrap().map(|axis|dot(axis,actual[2]))
    }
    fn follow_closed_test_arc(map:&SurfaceSnapshot,from:([f64;3],[f64;4]),angle:f64,steps:usize)
        ->([f64;3],[f64;4]) {
        let r=rotation(from.1).unwrap();let yaw=r[0][1].atan2(r[0][0]);let phase=yaw-std::f64::consts::FRAC_PI_2;
        let radius=0.2;let center=[from.0[0]-radius*phase.cos(),from.0[1]-radius*phase.sin()];
        let mut current=from;
        for i in 1..=steps {
            let turn=angle*i as f64/steps as f64;
            let xy=if i==steps && angle.abs()==std::f64::consts::TAU {[from.0[0],from.0[1]]}
                else {[center[0]+radius*(phase+turn).cos(),center[1]+radius*(phase+turn).sin()]};
            let next=map.body_pose_after_motion(current,xy,yaw+turn).unwrap();
            map.validate_body_motion(current,next,&[]).unwrap();current=next;
        }
        current
    }

    #[test]
    fn relative_terrain_full_turn_preserves_transported_tilt_bias() {
        let(map,from)=biased_sloped_contact_fixture();let r=rotation(from.1).unwrap();let yaw=r[0][1].atan2(r[0][0]);
        let reference=relative_up_on_ground(&map,from);assert!(reference[2]<0.999,"测试必须具有非零真实倾斜偏差");
        for sign in [-1.0,1.0] {
            let mut current=from;let mut worst=0.0_f64;
            for step in 1..=180 {
                let target_yaw=yaw+sign*std::f64::consts::TAU*step as f64/180.0;
                let next=map.body_pose_after_motion(current,[from.0[0],from.0[1]],target_yaw).unwrap();
                map.validate_body_motion(current,next,&[]).unwrap();assert_eq!(next.0,from.0,"原地转向不能升降或平移");
                let actual=relative_up_on_ground(&map,next);
                worst=worst.max(actual.iter().zip(reference).map(|(a,b)|(a-b).abs()).fold(0.0,f64::max));
                current=next;
            }
            let closure=rotation(current.1).unwrap()[2].iter().zip(r[2]).map(|(a,b)|(a-b).abs()).fold(0.0,f64::max);
            eprintln!("斜面原地完整转向 sign={sign}, 相对法向最大偏差={worst:e}, 闭合误差={closure:e}");
            assert!(worst<1e-10&&closure<1e-10,"只修正偏航不能额外转动已搬运法向");
        }
    }
    #[test]
    fn relative_terrain_closed_path_preserves_attitude_and_height() {
        let(map,from)=biased_sloped_contact_fixture();
        let end=follow_closed_test_arc(&map,from,std::f64::consts::TAU,180);
        let a=rotation(from.1).unwrap();let b=rotation(end.1).unwrap();
        let attitude=a.iter().flatten().zip(b.iter().flatten()).map(|(a,b)|(a-b).abs()).fold(0.0,f64::max);
        eprintln!("斜面闭合路径 姿态误差={attitude:e}, 高度误差={:e}",end.0[2]-from.0[2]);
        assert!(attitude<1e-9&&(end.0[2]-from.0[2]).abs()<1e-9,"闭合地形和偏航不能累积倾斜/高度偏差");
    }
    #[test]
    fn relative_terrain_transport_is_independent_of_turn_segmentation() {
        let(map,from)=biased_sloped_contact_fixture();
        let direct=follow_closed_test_arc(&map,from,std::f64::consts::FRAC_PI_2,1);
        for steps in [3,18,90] {
            let divided=follow_closed_test_arc(&map,from,std::f64::consts::FRAC_PI_2,steps);
            let a=rotation(direct.1).unwrap();let b=rotation(divided.1).unwrap();
            let attitude=a.iter().flatten().zip(b.iter().flatten()).map(|(a,b)|(a-b).abs()).fold(0.0,f64::max);
            eprintln!("斜面转弯分段 n={steps}, 姿态差={attitude:e}, 高度差={:e}",divided.0[2]-direct.0[2]);
            assert!(attitude<1e-9&&(divided.0[2]-direct.0[2]).abs()<1e-9,"同一地形与完整端点不应依赖弯道细分数");
        }
    }

    #[test]
    fn relative_terrain_normals_do_not_require_motion_from_support_to_body_center() {
        let points:Vec<_>=(-25..25).flat_map(|x|(-15..15).map(move|y| {
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,if x==0 {0.125}else{0.025})
        })).collect();
        let map=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.05).unwrap()
            .with_body_envelope(model()).unwrap();
        let from=([0.02,0.05,0.45],[0.0,0.075_f64.sin(),0.0,0.075_f64.cos()]);
        map.validate_body_pose(from.0,from.1).unwrap();
        let support=map.locate_support_from_body(from.0,from.1,0.4).unwrap();
        assert!(map.sample_segment_support(support.support_xyz,[from.0[0],from.0[1],support.surface_xyz[2]],1.0).is_err(),
            "夹在足下的窄阶边不代表机体正沿标定偏移行走");
        let target=([from.0[0]+1e-6,from.0[1],from.0[2]],from.1);
        map.validate_body_motion(from,target,&[]).unwrap();
        let projected=map.body_pose_after_motion(from,[target.0[0],target.0[1]],0.0).unwrap();
        map.validate_body_motion(from,projected,&[]).unwrap();
    }

    #[test]
    fn relative_terrain_motion_preserves_actual_height_tilt_and_zero_motion() {
        let map=ground_band_sweep_scene(2.025);
        let from=([-0.65,0.05,0.375],[0.01,0.02,0.15,1.0]);
        map.validate_body_pose(from.0,from.1).unwrap();
        let before=rotation(from.1).unwrap();let yaw=before[0][1].atan2(before[0][0]);
        assert_eq!(map.body_pose_after_motion(from,[from.0[0],from.0[1]],yaw).unwrap(),from);
        let next=map.body_pose_after_motion(from,[0.65,0.05],yaw+0.25).unwrap();
        let after=rotation(next.1).unwrap();
        assert!((next.0[2]-from.0[2]).abs()<1e-10,"同一厚层不能凭空抬高实际机体");
        assert!((after[2][2]-before[2][2]).abs()<1e-10,"实际倾斜偏差必须保留");
        assert!((after[0][1].atan2(after[0][0])-yaw-0.25).abs()<1e-10);
        map.validate_body_motion(from,next,&[]).unwrap();
    }

    #[test]
    fn relative_terrain_motion_cannot_gain_clearance_by_resetting_height() {
        let map=ground_band_sweep_scene(0.275);
        let from=([-0.65,0.05,0.375],[0.0,0.0,0.0,1.0]);
        let next=map.body_pose_after_motion(from,[0.65,0.05],0.0).unwrap();
        assert!((next.0[2]-from.0[2]).abs()<1e-10);
        assert_eq!(map.validate_body_motion(from,next,&[]),Err(SurfaceError::InsufficientClearance));
    }

    #[test]
    fn support_sampling_preserves_tiny_negative_endpoints_at_a_grid_boundary() {
        let map=scene(&[],false);
        let from=[-0.2,-1e-24,0.1];let to=[-0.25,-1e-73,0.1];
        map.validate_segment(from,to).expect("两端都在零线负侧，末点不能因消减误入正侧");
        let end=map.sample_segment_support(from,to,1.0).unwrap();
        assert_eq!(end.support_xyz[0],to[0]);
        assert_eq!(end.support_xyz[1],to[1],"真实端点必须保留，不能靠截断小数解决");
        map.validate_body_motion(([-0.2,0.0,0.5],[-1e-24,0.0,0.0,1.0]),
            ([-0.25,0.0,0.5],[-1e-73,0.0,0.0,1.0]),&[]).unwrap();
    }

    #[test]
    fn relative_terrain_motion_keeps_flat_normals_exact_without_erasing_real_tilt() {
        let map=scene(&[],false);
        let mut from=([0.55,0.0,0.5],[0.0,0.0,0.0,1.0]);
        for index in 1..=30 {
            let next=map.body_pose_after_motion(from,[0.55-index as f64*0.05,0.0],0.0).unwrap();
            assert_eq!(next.1,[0.0,0.0,0.0,1.0],"真实平面不能拟合出跨零格界的伪倾斜");
            map.validate_body_motion(from,next,&[]).unwrap();
            from=next;
        }
        let tiny=([0.55,0.0,0.5],[1e-18,-2e-18,0.0,1.0]);
        let r=rotation(tiny.1).unwrap();let yaw=r[0][1].atan2(r[0][0]);
        let moved=map.body_pose_after_motion(tiny,[0.45,0.0],yaw).unwrap();
        assert!((moved.1[0]-tiny.1[0]).abs()<1e-30 && (moved.1[1]-tiny.1[1]).abs()<1e-30,
            "不能靠清零真实小姿态消除数值问题");
    }

    #[test]
    fn unchanged_terrain_frame_preserves_actual_quaternion_and_height_exactly() {
        let map=ground_band_sweep_scene(2.025);
        for q in [[0.00394335813731047,-0.0017848593550173339,-0.005558448676189815,0.9999751836180726],
            [1e-18,-2e-18,0.0,1.0]] {
            let from=([-0.65,0.05,0.375],q);
            let r=rotation(q).unwrap();let yaw=r[0][1].atan2(r[0][0]);
            let next=map.body_pose_after_motion(from,[0.65,0.05],yaw).unwrap();
            assert_eq!(next.1,q,"地形帧和偏航完全相同，不能经矩阵往返制造姿态变化");
            assert_eq!(next.0[2],from.0[2],"相同旋转偏移不能经世界高度加减制造升降");
        }
        let from=([-0.65,0.05,0.375],[0.0,0.0,0.0,1.0]);
        let next=map.body_pose_after_motion(from,[0.65,0.05],1e-18).unwrap();
        assert!(next.1[2]>0.0,"真实微小偏航仍必须保留");
    }

    #[test]
    fn relative_terrain_motion_has_no_attitude_jump_for_a_micrometre_translation() {
        let points:Vec<_>=(-50..50).flat_map(|x|(-30..30).map(move|y| {
            let px=(x as f32+0.5)*0.05;(px,(y as f32+0.5)*0.05,px*0.13+0.021)
        })).collect();
        let map=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.05).unwrap()
            .with_body_envelope(model()).unwrap();
        for index in -10..10 {
            let x=index as f64*0.1+0.035;
            let surface=map.locate_near_support([x,0.05,x*0.13+0.021],0.1).unwrap();
            let support=[x,0.05,surface.surface_xyz[2]];
            let q=map.body_orientation_at(support,0.0).unwrap();
            let from=(map.body_position_from_support(support,q,0.4).unwrap(),q);
            map.validate_body_pose(from.0,from.1).unwrap();
            let before=rotation(from.1).unwrap();
            let mut differences=Vec::new();
            for distance in [1e-6,1e-7] {
                let next=map.body_pose_after_motion(from,[from.0[0]+distance,from.0[1]],0.0).unwrap();
                let after=rotation(next.1).unwrap();
                let difference=(0..3).flat_map(|row|(0..3).map(move|col|
                    (before[row][col]-after[row][col]).abs())).fold(0.0,f64::max);
                assert!(difference<distance,"微米平移不能突然改变倾斜：{from:?} -> {next:?}");
                differences.push(difference);
            }
            assert!(differences[1]<differences[0]*0.2+1e-14,
                "位移缩小十倍时姿态变化应连续趋零：{differences:?}");
        }
    }

    #[test]
    fn exposed_ground_at_sparse_block_boundary_keeps_real_flat_motion() {
        for iz in [-33,-1,31] {
            let fine=f64::from(0.05_f32);
            let points:Vec<_>=(-30..30).flat_map(|x|(-20..20).map(move|y|
                (((x as f64+0.5)*fine) as f32,((y as f64+0.5)*fine) as f32,((iz as f64+0.5)*fine) as f32))).collect();
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/fine).floor() as i32);
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }
            let mut observed=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {observed.replace_block(key,values).unwrap();}
            assert!(!observed.has_complete_block([0,0,iz+1]),"稀疏索引不为全空上方保留块");
            let map=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.05).unwrap()
                .with_body_envelope(model()).unwrap().with_observations(Some(Arc::new(observed)));
            let from=([-0.55,0.05,(iz as f64+1.0)*fine+0.4],[0.0,0.0,0.0,1.0]);
            let next=map.body_pose_after_motion(from,[0.55,0.05],0.0).unwrap();
            assert_eq!(next.0[2],from.0[2]);assert_eq!(next.1,from.1);
            map.validate_body_motion(from,next,&[]).unwrap();
        }
    }

    #[test]
    fn relative_terrain_cannot_turn_a_wall_or_missing_footprint_into_ground() {
        for missing in [false,true] {
            let mut points=Vec::new();
            for x in -40..40 {for y in -20..20 {
                if missing && (0..4).contains(&x) {continue;}
                points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,-0.025));
                if !missing && (0..4).contains(&x) {
                    for z in 0..20 {points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));}
                }
            }}
            let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
            let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(model()).unwrap();
            let from=([-0.65,0.025,0.4],[0.0,0.0,0.0,1.0]);
            map.validate_body_pose(from.0,from.1).unwrap();
            if missing {
                map.body_pose_after_motion(from,[-0.1,0.025],0.0).unwrap();
                map.body_pose_after_motion(from,[0.1,0.025],0.0).unwrap();
                assert_eq!(map.query_ground([0.1,0.025],0.0).state,GroundQueryState::Unknown,
                    "carried pose reference never fills the separating strip");
            } else {
                assert!(map.body_pose_after_motion(from,[-0.1,0.025],0.0).is_err(),"actual wall still intersects the body");
            }
        }
    }

    #[test]
    fn relative_terrain_uses_exposed_fine_surface_without_internal_or_other_floor_drift() {
        let make=|fill:bool,upper:bool,blocked:bool| {
            let mut points=Vec::new();
            for x in -40..40 {for y in -20..20 {
                let px=(x as f32+0.5)*0.05;let py=(y as f32+0.5)*0.05;
                points.push((px,py,0.025));
                if fill {for z in if x<0 {vec![-0.025,-0.075,-0.125]}else{vec![-0.025]} {points.push((px,py,z));}}
                if upper {points.push((px,py,0.825));}
            }}
            if blocked {
                for x in -1..=0 {for y in -1..=0 {for z in 4..16 {
                    points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));
                }}}
            }
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }
            let mut observed=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {observed.replace_block(key,values).unwrap();}
            SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.05).unwrap()
                .with_body_envelope(model()).unwrap().with_observations(Some(Arc::new(observed)))
        };
        let from=([-0.65,0.05,0.45],[0.01,0.02,0.15,1.0]);
        let r=rotation(from.1).unwrap();let yaw=r[0][1].atan2(r[0][0]);
        for evidence in [GroundEvidence::OccupiedVolume,GroundEvidence::MeasuredSurface] {
            let mut results=Vec::new();
            for (fill,upper) in [(false,false),(true,false),(true,true)] {
                let map=make(fill,upper,false).with_ground_evidence(evidence);
                let result=map.body_pose_after_motion(from,[0.65,0.05],yaw).unwrap();
                assert_eq!(result.1,from.1,"两端固定参考面相同时保留原完整四元数");
                // 两侧原区间共同支持同一水平参考，向下填充不应再制造5厘米高度漂移。
                assert!((result.0[2]-from.0[2]).abs()<1e-12,
                    "固定区域应利用共同可行高度，不把厚度分布变化当作地面升降");
                map.validate_body_motion(from,result,&[]).unwrap();
                let returned=map.body_pose_after_motion(result,[from.0[0],from.0[1]],yaw).unwrap();
                assert!((returned.0[2]-from.0[2]).abs()<1e-10,"几何近似不能累积往返漂移");
                results.push(result);
            }
            assert_eq!(results[1],results[2],"独立上层不得混入当前已关联带");
            if evidence==GroundEvidence::OccupiedVolume {assert_eq!(results[0],results[1]);}
        }
        let blocked=make(true,false,true).with_ground_evidence(GroundEvidence::MeasuredSurface);
        let to=make(true,false,false).body_pose_after_motion(from,[0.65,0.05],yaw).unwrap();
        blocked.validate_body_pose(from.0,from.1).unwrap();
        blocked.validate_body_pose(to.0,to.1).unwrap();
        assert!(blocked.validate_body_motion(from,to,&[]).is_err(),"固定厚带近似不能穿透真实中间障碍");
    }

    #[test]
    fn current_body_clearing_does_not_turn_a_flat_observed_band_into_a_terrain_pit() {
        let mut points=Vec::new();
        for x in -40..40 {for y in -30..30 {for z in -1..3 {
            points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));
        }}}
        // 当前真实机体内的回波触发私有清除；完整地面仍有相同真实露顶。
        points.push((0.025,0.025,0.325));
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
            .with_observations(Some(Arc::new(observations)));
        let from=([0.0,0.0,0.4],[0.0,0.0,0.0,1.0]);
        assert!(map.validate_body_pose(from.0,from.1).is_err());
        assert!(map.with_current_body_support(from.0,from.1).is_err(),
            "a current-body wrapper must not erase an intersecting obstacle");
        assert!(map.observations.as_ref().unwrap().get([0,0,2]).is_some());
        assert!(map.observations.as_ref().unwrap().get([0,0,6]).is_some());
        assert!(map.validate_body_pose(from.0,from.1).is_err());
    }

    #[test]
    fn ground_contact_plane_selects_the_continuous_band_not_a_separate_high_echo() {
        let mut points=Vec::new();
        for x in -30..30 {for y in -20..20 {for z in -11..=-8 {
            points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));
        }}}
        // 同一粗列中-7层真实缺失；-6层高回波不能替换包含实际接触的低带。
        points.push((-0.175,-0.075,-0.275));
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        assert!(observations.get([-4,-2,-7]).is_none());
        assert!(observations.get([-4,-2,-6]).is_some());
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.z_bounds_m=[-0.448,0.057];
        envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;envelope.max_cross_slope_deg=18.0;
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
            .with_observations(Some(Arc::new(observations)));
        let from=([0.0,0.0,-0.044],[0.0,0.0,0.0,1.0]);
        map.validate_body_pose(from.0,from.1).unwrap();
        let next=map.body_pose_after_motion(from,[-0.13,0.0],0.0).unwrap();
        assert_eq!(next.0[2],from.0[2],"低真实厚带包含当前平面，不应被分离高回波强制抬升");
        assert_eq!(next.1,from.1,"不能因选错细带制造地形倾斜");
        map.validate_body_motion(from,next,&[]).unwrap();
        assert_eq!(map.validate_body_pose([from.0[0],from.0[1],from.0[2]-0.01],from.1),
            Err(SurfaceError::InsufficientClearance),"分离高回波仍参与原完整机体碰撞/接触检查");
    }

    #[test]
    fn relative_terrain_height_follows_the_plane_at_the_rotated_support_point() {
        for slope in [0.25_f64,-0.25] {
            // 网格与源分辨率相容，使真实露顶精确位于已知z=slope*(x-0.1)平面。
            let points:Vec<_>=(-40..40).flat_map(|x|(-20..20).map(move|y| {
                let px=(x as f64+0.5)*0.2;
                (px as f32,((y as f64+0.5)*0.2) as f32,(slope*(px-0.1).max(0.0)-0.025) as f32)
            })).collect();
            let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=0.2;cfg.min_step_tread_depth_m=0.4;cfg.wall_clearance_m=0.0;
            let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(model()).unwrap();
            let angle=slope.atan();let q=[0.0,(-angle*0.5).sin(),0.0,(-angle*0.5).cos()];
            for bias in [0.0,0.013] {
                let flat=([-1.5,0.05,0.4+bias],[0.0,0.0,0.0,1.0]);
                // 已知法向距离d对应机体中心的竖直高度d/cos(theta)。
                let height=slope*1.4+0.4*(1.0+slope*slope).sqrt()+bias;
                let inclined=([1.5,0.05,height],q);
                let up=map.body_pose_after_motion(flat,[1.5,0.05],0.0).unwrap();
                assert!((up.0[2]-height).abs()<1e-5,"中心高度须使用旋转支撑点的平面高度：{up:?} / {height}");
                let down=map.body_pose_after_motion(inclined,[-1.5,0.05],0.0).unwrap();
                assert!((down.0[2]-flat.0[2]).abs()<1e-5,"返回平地保留实际高度偏差：{down:?}");
                let turn=map.body_pose_after_motion(inclined,[inclined.0[0],inclined.0[1]],0.2).unwrap();
                assert_eq!(turn.0[2],inclined.0[2],"同一位置转向不能自行改变中心高度");
            }
        }
    }

    #[test]
    fn relative_terrain_motion_adapts_to_unambiguous_ground_before_the_center_crosses() {
        let mut points=Vec::new();
        for x in -40..40 {for y in -20..20 {
            let z=if x<0 {-1}else{1};
            points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));
        }}
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
        envelope.max_cross_slope_deg=18.0;
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let from=([-0.5,0.025,0.4],[0.0,0.0,0.0,1.0]);
        map.validate_body_pose(from.0,from.1).unwrap();
        assert_eq!(map.body_pose_after_motion(from,[from.0[0],from.0[1]],0.0).unwrap(),from);
        let mut current=from;
        for i in 1..=100 {
            let x=-0.5+i as f64*0.01;
            let next=map.body_pose_after_motion(current,[x,0.025],0.0).unwrap();
            map.validate_body_motion(current,next,&[]).unwrap();
            assert!((next.0[2]-current.0[2]).abs()<0.01,"真实露顶变化须连续适应，不能在中心跨格时突然抬升");
            if i==35 {assert!(next.0[2]>from.0[2]+0.005,"前足已覆盖高地面时须在中心跨格前抬升");}
            current=next;
        }
        assert!((current.0[2]-from.0[2]-0.1).abs()<0.01,"保持实际初始偏差，随露顶真实升高0.1米");
    }

    #[test]
    fn contact_response_follows_declared_footprint_and_source_not_planning_resolution() {
        let mut points=Vec::new();
        for x in -40..40 {for y in -30..30 {
            points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,if x<3 {-0.025}else{0.075}));
        }}
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        let observations=Arc::new(observations);
        let sample=|planning:f32,half:[f64;2],xy:[f64;2]| {
            let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=planning;cfg.wall_clearance_m=0.0;cfg.robot_height=0.1;
            let mut envelope=model();let [x,y]=half;
            envelope.footprint_xy_m=vec![[-x,-y],[x,-y],[x,y],[-x,y]];
            let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
                .with_observations(Some(Arc::clone(&observations)));
            let centre=map.locate_support([xy[0],xy[1],0.0]).unwrap();
            map.body_contact_frame(centre,0.0,0.0).unwrap()
        };
        for xy in [[-0.05,0.025],[-0.000001,0.099999],[-0.099999,-0.000001]] {
            let a=sample(0.05,[0.3,0.15],xy);let b=sample(0.1,[0.3,0.15],xy);
            assert!((a.0-b.0).abs()<1e-7 && a.1.iter().zip(b.1).all(|(x,y)|(x-y).abs()<1e-7),
                "相同足迹及源观测不能因规划格变化而采用另一响应核: {xy:?}, {a:?} / {b:?}");
        }
        let small=sample(0.1,[0.1,0.1],[-0.05,0.025]);
        assert_eq!(small,(0.0,[0.0,0.0,0.0,1.0]),"小足迹的一源体素覆盖域尚未接触高台");
        let large=sample(0.1,[0.3,0.15],[-0.05,0.025]);
        assert!(large.0>0.0 && large.1[1]<0.0,"声明足迹已覆盖高台时应产生真实提前适应");
    }

    #[test]
    fn contact_response_uses_retained_sample_spacing_without_fine_observations() {
        let points:Vec<_>=(-20..20).flat_map(|x|(-15..15).map(move |y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,if x<0 {-0.0125}else{0.0875}))).collect();
        let make=|source| {
            let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=0.1;cfg.wall_clearance_m=0.0;cfg.robot_height=0.1;
            SurfaceSnapshot::from_points(&points,cfg,source).unwrap().with_body_envelope(model()).unwrap()
        };
        let fine_source=make(0.025);let other_source=make(0.05);
        // 未保留精细观察时，两个来源精度均只留下同一粗列中心及真实高度区间。
        // 已不存在的精细XY不能继续缩窄地形响应核。
        for x in -6..6 {for y in -5..5 {
            let p=[(f64::from(x)+0.5)*0.1,(f64::from(y)+0.5)*0.1,if x<0 {0.0}else{0.1}];
            let a=fine_source.locate_support(p).unwrap();let b=other_source.locate_support(p).unwrap();
            assert_eq!(fine_source.ground_contact_samples(a.cell),other_source.ground_contact_samples(b.cell));
        }}
        for xy in [[-0.15,0.05],[-0.10,0.05],[-0.025,0.075]] {
            let p=[xy[0],xy[1],0.0];
            let a=fine_source.body_contact_frame(fine_source.locate_support(p).unwrap(),0.0,0.0).unwrap();
            let b=other_source.body_contact_frame(other_source.locate_support(p).unwrap(),0.0,0.0).unwrap();
            assert!((a.0-b.0).abs()<1e-12 && a.1.iter().zip(b.1).all(|(a,b)|(a-b).abs()<1e-12),
                "真实拟合输入相同，源分辨率元数据不应改变已聚合后的响应: {xy:?} {a:?} {b:?}");
        }
    }

    #[test]
    fn fine_contact_domain_uses_selected_bands_and_directed_connected_component() {
        let make=|levels:Vec<([i32;2],Vec<i32>)>| {
            let mut points=Vec::new();
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for (xy,zs) in levels {for z in zs {
                let key=[xy[0],xy[1],z];
                points.push(((f64::from(xy[0])+0.5) as f32*0.05,(f64::from(xy[1])+0.5) as f32*0.05,(f64::from(z)+0.5) as f32*0.05));
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }}
            let mut observations=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
            let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.0;cfg.robot_height=0.1;
            let mut envelope=model();envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.1;
            SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
                .with_observations(Some(Arc::new(observations)))
        };
        let stairs=make((-10..10).flat_map(|x|(-10..10).map(move |y|([x,y],vec![if x<0 {-1}else{3},19]))).collect());
        let up=stairs.fine_contact_domain([-0.075,0.025,0.0],0.3).unwrap();
        assert!(up.iter().any(|p|p.0[0]>0.0),"真实四细格上阶仍在声明上阶加量化范围内");
        assert!(up.iter().all(|p|p.0[2]<0.3),"独立上一层不得混入当前带");
        let down=stairs.fine_contact_domain([0.075,0.025,0.2],0.3).unwrap();
        assert!(down.iter().all(|p|p.0[0]>0.0),"相同高差超过声明下阶范围时不能借上阶许可连接");
        assert!(stairs.fine_contact_domain([0.075,0.025,1.0],0.3).unwrap().iter().all(|p|p.0[2]>0.9),
            "上层实际参考不能借近列探测混入下层地面");
        let diagonal=make((-2..=1).map(|i|([i,i],vec![-1])).collect());
        assert_eq!(diagonal.fine_contact_domain([0.0,0.0,0.0],0.3).unwrap().len(),4,
            "八方向必须包括真实对角邻列，不能复用粗图的四方向常量");
        let line=make((-10..10).map(|y|([0,y],vec![-1])).collect());
        let line_points=line.fine_contact_domain([0.025,0.025,0.0],0.3).unwrap();
        assert!(fit_contact_plane(&line_points,[0.025,0.025,0.0]).is_none(),
            "没有其它真实列时仍无法拟合二维地面，不能用虚构点补秩");
        line.validate_body_pose([0.025,0.025,0.4],[0.0,0.0,0.0,1.0]).unwrap();
        assert_eq!(line.query_ground([0.125,0.025],0.0).state,GroundQueryState::Unknown,
            "an isolated line cannot interpolate a two-dimensional terrain plane");
        let wall=make((-10..10).flat_map(|x|(-10..10).map(move |y|
            ([x,y],if x==0 {(-1..15).collect()}else{vec![-1]}))).collect());
        let domain=wall.fine_contact_domain([-0.1,0.025,0.0],0.3).unwrap();
        assert!(domain.iter().all(|p|p.0[0]<0.0),"高墙顶不能连接墙内侧种子与另一侧同高地面");
        let high_band=make((-10..10).flat_map(|x|(-10..10).map(move |y| {
            let mut zs=vec![-3];zs.extend(-1..8);([x,y],zs)
        })).collect());
        assert_eq!(high_band.fine_contact_domain([0.0,0.0,0.0],0.3),Err(SurfaceError::MissingSupport),
            "最近厚带没有近参考暴露顶时不能退而选更远的低薄层制造种子");
        let equal=make((-10..10).flat_map(|x|(-10..10).map(move |y|([x,y],vec![-3,1]))).collect());
        assert!(equal.fine_contact_domain([0.0,0.0,0.0],0.3).unwrap().iter().all(|p|p.0[2]<0.0),
            "完整区间距离和顶距相同时必须稳定选较低带");
    }

    #[test]
    fn fine_contact_seed_cannot_jump_from_supported_body_to_a_disconnected_outer_echo() {
        let mut keys:Vec<[i32;3]>=(-12..12).flat_map(|x|(-10..4).map(move |y|[x,y,-1])).collect();
        // 机体脚下为完整平地；横向间隔一列后，足迹外有两个高度样本形成的小孤岛。
        keys.extend([[4,5,-1],[4,5,0]]);
        let points:Vec<_>=keys.iter().map(|k|((k[0] as f32+0.5)*0.05,(k[1] as f32+0.5)*0.05,(k[2] as f32+0.5)*0.05)).collect();
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});}
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
        let coarse=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let map=coarse.with_observations(Some(Arc::new(observations)));
        let from=([0.0,0.0,0.41],[0.0,0.0,0.0,1.0]);
        map.validate_body_pose(from.0,from.1).expect("当前完整真实足迹和平地接触合法");
        coarse.body_pose_after_motion(from,[0.001,0.0],0.0).expect("原粗响应可完成真实平地微动");
        let next=map.body_pose_after_motion(from,[0.001,0.0],0.0)
            .expect("外部孤岛恰好包含参考高度，不应夺走当前完整地面的响应种子");
        // 固定几何场可包含外部观测造成的微小形状偏差；静态区间根仍须保持，
        // 动态输运则必须通过完整扫掠并能严格回到同一实际站姿。
        super::scalar_response_tests::assert_static_interval_and_reversible_motion(&map,from,next);
        eprintln!("SCALAR_FLAT_SEED height_delta_m={:?}",next.0[2]-from.0[2]);
    }

    fn sparse_contact_scene(keys:Vec<[i32;3]>,mut envelope:SurfaceBodyEnvelope)->SurfaceSnapshot {
        let fine=f64::from(0.05_f32);
        let points:Vec<_>=keys.iter().map(|key|((f64::from(key[0])+0.5)*fine,(f64::from(key[1])+0.5)*fine,(f64::from(key[2])+0.5)*fine))
            .map(|(x,y,z)|(x as f32,y as f32,z as f32)).collect();
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for key in keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});}
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        cfg.step_threshold_m=envelope.max_step_up_m.max(envelope.max_step_down_m) as f32;
        envelope.footprint_xy_m=vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]];
        envelope.z_bounds_m=[-0.15,0.15];envelope.body_to_support_z_m=0.35;
        SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
            .with_observations(Some(Arc::new(observations)))
    }


    /// 私有域输出现在是字典序；实际拟合输入必须逐位保持原 BFS 经排序后的结果。
    mod dense_contact_ordering {
        use super::*;

        type ContactPoint=([f64;3],f64,f64);

        // 冻结变更前的稠密算法，不调用新的域构建器或共享遍历/发点实现。
        fn reference_bfs(map:&SurfaceSnapshot,origin:[f64;3],radius:f64)
            ->Result<Vec<([f64;3],f64,f64)>,SurfaceError> {
            let observations=map.observations.as_ref().ok_or(SurfaceError::MissingSupport)?;
            let model=map.body_envelope().ok_or(SurfaceError::InvalidPose)?;
            let size=f64::from(observations.size_m);
            let enumeration=radius+std::f64::consts::SQRT_2*size;
            let low:[i32;2]=std::array::from_fn(|i|((origin[i]-enumeration)/size).floor() as i32);
            let high:[i32;2]=std::array::from_fn(|i|((origin[i]+enumeration)/size).floor() as i32);
            let nx=i64::from(high[0])-i64::from(low[0])+1;
            let ny=i64::from(high[1])-i64::from(low[1])+1;
            // 仅限制局部稠密临时表的容量；特殊大域继续原稀疏算法，不改变许可或拟合域。
            let Some(count)=nx.checked_mul(ny).filter(|n|nx>0&&ny>0&&*n<=16_384) else {
                return map.fine_contact_domain_sparse(origin,radius);
            };
            let stride=ny as usize;let count=count as usize;
            let mut in_domain=vec![false;count];
            let mut bands=vec![None;count];
            // 同一不可变来源的带缓存一次批量读取，避免每个实测细列分别取锁。
            {
                let cache=map.body_query_cache.lock().unwrap_or_else(|e|e.into_inner());
                for x in low[0]..=high[0] {for y in low[1]..=high[1] {
                    let dx=(f64::from(x)+0.5)*size-origin[0];let dy=(f64::from(y)+0.5)*size-origin[1];
                    if dx*dx+dy*dy>enumeration*enumeration {continue;}
                    let index=(i64::from(x)-i64::from(low[0])) as usize*stride+(i64::from(y)-i64::from(low[1])) as usize;
                    in_domain[index]=true;bands[index]=cache.fine_bands.get(&(x,y)).cloned();
                }}
            }
            let distance=|band:[f64;2]|(band[0]-origin[2]).max(origin[2]-band[1]).max(0.0);
            let mut selected=vec![None;count];let mut seed:Option<(usize,(i32,i32),[f64;4])>=None;
            for (index,inside) in in_domain.iter().copied().enumerate() {
                if !inside {continue;}
                let x=low[0]+(index/stride) as i32;let y=low[1]+(index%stride) as i32;
                let values=bands[index].take().unwrap_or_else(||map.fine_contact_bands(x,y));
                let Some(&band)=values.iter().map(|entry|&entry.fit).min_by(|a,b|distance(**a).total_cmp(&distance(**b))
                    .then_with(||(a[1]-origin[2]).abs().total_cmp(&(b[1]-origin[2]).abs()))
                    .then_with(||a[1].total_cmp(&b[1]))) else {continue;};
                selected[index]=Some(band);let rise=band[1]-origin[2];
                if rise>model.max_step_up_m+size+1e-6 || -rise>model.max_step_down_m+size+1e-6 {continue;}
                let dx=(f64::from(x)+0.5)*size-origin[0];let dy=(f64::from(y)+0.5)*size-origin[1];
                let rank=[dx*dx+dy*dy,distance(band),rise.abs(),band[1]];
                let earlier=seed.as_ref().is_none_or(|&(_,old,previous)| {
                    rank.iter().zip(previous).map(|(a,b)|a.total_cmp(&b)).find(|v|!v.is_eq())
                        .unwrap_or_else(||(x,y).cmp(&old)).is_lt()
                });
                if earlier {seed=Some((index,(x,y),rank));}
            }
            let seed=seed.ok_or(SurfaceError::MissingSupport)?.0;
            let mut queue=VecDeque::from([seed]);let mut visited=vec![false;count];visited[seed]=true;
            let mut points=Vec::new();
            while let Some(index)=queue.pop_front() {
                let x=low[0]+(index/stride) as i32;let y=low[1]+(index%stride) as i32;
                let band=selected[index].expect("只有已选真实带进入队列");
                let point=[(f64::from(x)+0.5)*size,(f64::from(y)+0.5)*size,band[1]];
                let distance2=(point[0]-origin[0]).powi(2)+(point[1]-origin[1]).powi(2);
                let weight=(1.0-distance2/(radius*radius)).max(0.0).powi(2);
                if weight>0.0 {points.push((point,band[0],weight));}
                for dx in -1..=1 {for dy in -1..=1 {
                    if dx==0&&dy==0 {continue;}
                    let(mut nx,mut ny)=(x,y);
                    loop {
                        let(Some(next_x),Some(next_y))=(nx.checked_add(dx),ny.checked_add(dy)) else {break;};
                        nx=next_x;ny=next_y;
                        if nx<low[0]||nx>high[0]||ny<low[1]||ny>high[1] {break;}
                        let next_index=(i64::from(nx)-i64::from(low[0])) as usize*stride+(i64::from(ny)-i64::from(low[1])) as usize;
                        if !in_domain[next_index] {break;}
                        let Some(next)=selected[next_index] else {continue;};
                        let rise=next[1]-band[1];
                        if rise<=model.max_step_up_m+size+1e-6 && -rise<=model.max_step_down_m+size+1e-6
                            && !visited[next_index] {visited[next_index]=true;queue.push_back(next_index);}
                        break;
                    }
                }}
            }
            Ok(points)
        }
        fn sort(points:&mut [ContactPoint]) {
            points.sort_by(|a,b|a.0[0].total_cmp(&b.0[0]).then_with(||a.0[1].total_cmp(&b.0[1])));
        }

        fn point_bits(point:&ContactPoint)->[u64;5] {
            [point.0[0],point.0[1],point.0[2],point.1,point.2].map(f64::to_bits)
        }

        fn plane_bits(plane:Option<(f64,f64,f64)>)->Option<[u64;3]> {
            plane.map(|(a,b,z)|[a,b,z].map(f64::to_bits))
        }

        fn compare(map:&SurfaceSnapshot,keys:&std::collections::BTreeSet<[i32;3]>,
            origin:[f64;3],radius:f64)->Result<Vec<ContactPoint>,SurfaceError> {
            let expected=reference_bfs(map,origin,radius);
            let actual=map.fine_contact_domain(origin,radius);
            match (expected,actual) {
                (Ok(mut expected),Ok(mut actual))=> {
                    assert_eq!(expected.len(),actual.len(),"point count: {origin:?}, {radius}");
                    assert!(actual.windows(2).all(|pair|pair[0].0[0].total_cmp(&pair[1].0[0])
                        .then_with(||pair[0].0[1].total_cmp(&pair[1].0[1])).is_lt()),
                        "dense output must be strictly XY ordered, without duplicate columns");
                    sort(&mut expected);sort(&mut actual);
                    assert_eq!(expected.iter().map(point_bits).collect::<Vec<_>>(),
                        actual.iter().map(point_bits).collect::<Vec<_>>(),"exact input: {origin:?}, {radius}");
                    assert_eq!(plane_bits(fit_contact_plane(&expected,origin)),
                        plane_bits(fit_contact_plane(&actual,origin)),"exact fit: {origin:?}, {radius}");
                    let size=f64::from(map.observations.as_ref().unwrap().size_m);
                    for &(point,low,weight) in &actual {
                        let x=(point[0]/size).floor() as i32;let y=(point[1]/size).floor() as i32;
                        let top=(point[2]/size).round() as i32-1;
                        assert!(keys.contains(&[x,y,top]),"emitted top must be observed");
                        assert!(!keys.contains(&[x,y,top+1]),"emitted top must terminate a real band");
                        let mut bottom=top;
                        while keys.contains(&[x,y,bottom-1]) {bottom-=1;}
                        let expected_low=if map.ground_evidence==GroundEvidence::OccupiedVolume {top}else{bottom};
                        assert_eq!(point.map(f64::to_bits),[(f64::from(x)+0.5)*size,
                            (f64::from(y)+0.5)*size,(f64::from(top)+1.0)*size].map(f64::to_bits));
                        assert_eq!(low.to_bits(),((f64::from(expected_low)+1.0)*size).to_bits());
                        let distance2=(point[0]-origin[0]).powi(2)+(point[1]-origin[1]).powi(2);
                        assert_eq!(weight.to_bits(),(1.0-distance2/(radius*radius)).max(0.0).powi(2).to_bits());
                        assert!(weight>0.0,"no zero-weight or synthetic support");
                    }
                    Ok(actual)
                },
                (Err(expected),Err(actual))=> {assert_eq!(expected,actual);Err(actual)},
                (expected,actual)=>panic!("domain result changed at {origin:?}, {radius}: {expected:?} / {actual:?}"),
            }
        }

        fn make(keys:Vec<[i32;3]>)->(SurfaceSnapshot,std::collections::BTreeSet<[i32;3]>) {
            let mut envelope=model();envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.1;
            let observed=keys.iter().copied().collect();
            (sparse_contact_scene(keys,envelope),observed)
        }

        fn result_bits(result:Result<Vec<ContactPoint>,SurfaceError>,origin:[f64;3])
            ->Result<(Vec<[u64;5]>,Option<[u64;3]>),SurfaceError> {
            result.map(|mut points| {
                sort(&mut points);
                (points.iter().map(point_bits).collect(),plane_bits(fit_contact_plane(&points,origin)))
            })
        }

        #[test]
        fn cold_and_mixed_cached_bands_match_both_references_after_cache_clear() {
            let keys=(-8_i32..=8).flat_map(|x|(-8_i32..=8).flat_map(move |y| {
                let levels:Vec<i32>=match (x+2*y).rem_euclid(6) {
                    0=>vec![],1=>vec![-1],2=>(-3..=-1).collect(),3=>vec![-1,3],
                    4=>vec![-4,0,4],_=>vec![if x<0 {-1}else{3}],
                };
                levels.into_iter().map(move |z|[x,y,z])
            })).collect();
            let (base,_)=make(keys);
            for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
                for height in [-0.1,0.0,0.1,4.0] {for cache_mode in 0..3 {
                    let map=base.fork_for_planning().with_ground_evidence(evidence);
                    let reference=map.fork_for_planning();let sparse=map.fork_for_planning();
                    assert!(map.body_query_cache.lock().unwrap().fine_bands.is_empty());
                    if cache_mode>0 {
                        // 混合命中包含空带、单带与多带；未预热的列仍走原缺失路径。
                        for x in -8..=8 {for y in -8..=8 {
                            if (x+y)%2==0 {map.fine_contact_bands(x,y);}
                        }}
                    }
                    if cache_mode==2 {
                        // 远处列确实没有观察。填满已有缓存，让域内第一次缺失清空缓存；
                        // 已抄入选择表的单带与已借持 Arc 的多带仍须给出相同结果。
                        let mut cache=map.body_query_cache.lock().unwrap();
                        let empty=Arc::new(Vec::new());
                        for index in cache.fine_bands.len()..8192 {
                            cache.fine_bands.insert((1_000_000+index as i32,0),Arc::clone(&empty));
                        }
                    }
                    let origin=[0.025,0.025,height];let radius=0.39;
                    // 被测实现先执行，参考实现用独立缓存，不能先预热候选路径。
                    let actual=result_bits(map.fine_contact_domain(origin,radius),origin);
                    if cache_mode==2 {assert!(map.body_query_cache.lock().unwrap().fine_bands.len()<8192);}
                    assert_eq!(actual,result_bits(reference_bfs(&reference,origin,radius),origin));
                    assert_eq!(actual,result_bits(sparse.fine_contact_domain_sparse(origin,radius),origin));
                    assert_eq!(actual,result_bits(map.fine_contact_domain(origin,radius),origin));
                }}
            }
        }

        #[test]
        fn rectangular_and_single_axis_domains_match_at_i32_coordinate_edges() {
            let fine=f64::from(0.05_f32);let radius=0.3;
            let enumeration=radius+std::f64::consts::SQRT_2*fine;
            let (base,_)=make(vec![[0,0,-1]]);
            for (ax,sx) in [(i32::MIN,1),(i32::MAX,-1)] {
                for (ay,sy) in [(i32::MIN,1),(i32::MAX,-1)] {
                    let keys:Vec<_>=(0..=16).flat_map(|x|(0..=16).map(move |y|[ax+sx*x,ay+sy*y,-1])).collect();
                    let mut blocks=std::collections::BTreeMap::<[i32;3],
                        std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
                    for &key in &keys {blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
                        .insert(key,SurfaceObservation{observed_ns:0,hits:0});}
                    let mut observations=SurfaceObservations::new(0.05,32).unwrap();
                    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
                    // 精确细观察直接挂接，不借助 f32 粗点云表达 i32 极值列。
                    let map=base.with_observations(Some(Arc::new(observations)));
                    for shape in 0..3 {
                        let mut origin=[(f64::from(ax)+0.5+f64::from(sx)*3.0)*fine,
                            (f64::from(ay)+0.5+f64::from(sy)*5.0)*fine,0.0];
                        if shape==1 {origin[0]=(f64::from(ax)+0.5-f64::from(sx)*(enumeration/fine-0.25))*fine;}
                        if shape==2 {origin[1]=(f64::from(ay)+0.5-f64::from(sy)*(enumeration/fine-0.25))*fine;}
                        let low:[i32;2]=std::array::from_fn(|i|((origin[i]-enumeration)/fine).floor() as i32);
                        let high:[i32;2]=std::array::from_fn(|i|((origin[i]+enumeration)/fine).floor() as i32);
                        if shape==1 {assert_eq!(low[0],high[0]);assert!(low[1]<high[1]);}
                        if shape==2 {assert_eq!(low[1],high[1]);assert!(low[0]<high[0]);}
                        let candidate=map.fork_for_planning();
                        #[cfg(feature="body-query-profile")]
                        query_profile::begin();
                        let actual=candidate.fine_contact_domain(origin,radius);
                        #[cfg(feature="body-query-profile")]
                        {
                            let record=query_profile::end();
                            let expected=keys.iter().filter(|key| {
                                let dx=(f64::from(key[0])+0.5)*fine-origin[0];
                                let dy=(f64::from(key[1])+0.5)*fine-origin[1];
                                dx*dx+dy*dy<=enumeration*enumeration
                            }).count();
                            // 单轴域虽可没有正权重点，也须确实走遍其相连真实列。
                            assert!(expected>1);
                            assert_eq!(record.counters[QueryCounter::DenseVisitedColumns as usize],expected as u64);
                        }
                        let actual=result_bits(actual,origin);
                        assert!(actual.is_ok());
                        assert_eq!(actual,result_bits(reference_bfs(&map.fork_for_planning(),origin,radius),origin));
                        assert_eq!(actual,result_bits(map.fork_for_planning().fine_contact_domain_sparse(origin,radius),origin));
                        assert_eq!(actual,result_bits(candidate.fine_contact_domain(origin,radius),origin));
                    }
                }
            }
        }

        #[test]
        fn exact_sorted_points_and_fits_match_frozen_dense_bfs_matrix() {
            let grid=|levels:fn(i32,i32)->Vec<i32>|->Vec<[i32;3]> {
                (-10_i32..=10).flat_map(|x|(-10_i32..=10).flat_map(move |y|
                    levels(x,y).into_iter().map(move |z|[x,y,z]))).collect()
            };
            let scenes=[
                ("flat",grid(|_,_|vec![-1])),
                ("quantized slope",grid(|x,y|vec![(x+2*y).div_euclid(4)-1])),
                ("gaps",grid(|x,y|if (x+y).rem_euclid(5)==0 || (x==1&&y<5) {vec![]}else{vec![-1]})),
                ("directed steps",grid(|x,_|vec![if x<0 {-1}else{3}])),
                ("blocking wall",grid(|x,_|if x==0 {(-1..15).collect()}else{vec![-1]})),
                ("separate layers",grid(|_,_|vec![-1,19])),
                ("overlapping height intervals",grid(|x,y|((x+y).rem_euclid(3)-4..=(x-y).rem_euclid(3)).collect())),
                ("equal band distances",grid(|_,_|vec![-3,1])),
                ("nearest high band cannot seed",grid(|_,_|std::iter::once(-3).chain(-1..8).collect())),
                ("disconnected islands",vec![[-4,-4,-1],[-4,-3,-1],[-3,-4,-1],[4,1,-1],[4,2,-1],[5,1,-1]]),
                ("diagonal rank loss",(-10..=10).map(|i|[i,i,-1]).collect()),
                ("single column rank loss",(-10..=10).map(|y|[0,y,-1]).collect()),
            ];
            let fine=f64::from(0.05_f32);
            let edge=2.0_f64.sqrt()*fine*0.5;
            let radii=[fine*0.25,f64::from_bits(edge.to_bits()-1),edge,
                f64::from_bits(edge.to_bits()+1),0.3,0.55];
            let origins=[[0.0,0.0,0.0],[-1.5*fine,0.5*fine,0.0],[1.5*fine,0.5*fine,4.0*fine],
                [-3.5*fine,-3.5*fine,0.0],[-0.173,-0.117,-0.1],[0.0,0.0,20.0*fine],[0.0,0.0,2.0]];
            let (mut successful,mut rejected,mut fitted,mut degenerate)=(0,0,0,0);
            for (name,keys) in scenes {
                let (base,observed)=make(keys);
                for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
                    let map=base.clone().with_ground_evidence(evidence);
                    for origin in origins {for radius in radii {
                        let result=compare(&map,&observed,origin,radius);
                        // Both variants share the unchanged snapshot; the reference runs first and warms its cache.
                        let repeated=compare(&map,&observed,origin,radius);
                        assert_eq!(result.as_ref().map(|v|v.iter().map(point_bits).collect::<Vec<_>>()),
                            repeated.as_ref().map(|v|v.iter().map(point_bits).collect::<Vec<_>>()),"{name}, {evidence:?}");
                        match result {
                            Ok(points)=> {successful+=1;if fit_contact_plane(&points,origin).is_some() {fitted+=1;}else{degenerate+=1;}},
                            Err(SurfaceError::MissingSupport)=>rejected+=1,
                            Err(error)=>panic!("unexpected {name} error: {error:?}"),
                        }
                    }}
                }
            }
            assert!(successful>0 && rejected>0 && fitted>0 && degenerate>0);
        }

        #[test]
        fn exact_membership_direction_ties_and_missing_support_are_retained() {
            let fine=f64::from(0.05_f32);
            let stairs=(-6..=6).flat_map(|x|(-6..=6).map(move |y|[x,y,if x<0 {-1}else{3}])).collect();
            let (map,keys)=make(stairs);
            let up=compare(&map,&keys,[-1.5*fine,0.5*fine,0.0],0.3).unwrap();
            assert!(up.iter().any(|p|p.0[0]>0.0));
            let down=compare(&map,&keys,[1.5*fine,0.5*fine,4.0*fine],0.3).unwrap();
            assert!(down.iter().all(|p|p.0[0]>0.0));
            let (islands,keys)=make(vec![[-4,-4,-1],[-4,-3,-1],[-3,-4,-1],[4,1,-1],[4,2,-1],[5,1,-1]]);
            let points=compare(&islands,&keys,[-3.5*fine,-3.5*fine,0.0],0.6).unwrap();
            assert_eq!(points.len(),3);assert!(points.iter().all(|p|p.0[0]<0.0));
            let tied=(-3..=3).flat_map(|x|(-3..=3).flat_map(move |y|[-3,1].map(|z|[x,y,z]))).collect();
            let (ties,keys)=make(tied);
            assert!(compare(&ties,&keys,[0.0;3],0.3).unwrap().iter().all(|p|p.0[2]<0.0));
            assert_eq!(compare(&ties,&keys,[0.0,0.0,4.0],0.3),Err(SurfaceError::MissingSupport));
            let missing=ties.with_observations(None);
            assert_eq!(reference_bfs(&missing,[0.0;3],0.3),Err(SurfaceError::MissingSupport));
            assert_eq!(missing.fine_contact_domain([0.0;3],0.3),Err(SurfaceError::MissingSupport));
            let no_model=SurfaceSnapshot::from_points(&[(0.025,0.025,-0.025)],ties.config().clone(),0.05)
                .unwrap().with_observations(ties.observations.clone());
            assert_eq!(reference_bfs(&no_model,[0.0;3],0.3),Err(SurfaceError::InvalidPose));
            assert_eq!(no_model.fine_contact_domain([0.0;3],0.3),Err(SurfaceError::InvalidPose));
        }

        #[test]
        fn visited_incompatible_column_still_stops_ray_before_far_layer() {
            let fine=f64::from(0.05_f32);
            for stride in [1,2] {
                // The low seed reaches +0.2 m, but cannot reach the far +0.4 m.
                // From +0.2 m, the already-visited low column blocks the ray even
                // though descending to it is incompatible. Empty columns may be
                // scanned through; visited real columns may never be skipped.
                let (map,keys)=make(vec![[-stride,0,7],[0,0,-1],[stride,0,3]]);
                let points=compare(&map,&keys,[0.5*fine,0.5*fine,0.0],0.3).unwrap();
                assert_eq!(points.len(),2);
                assert!(points.iter().all(|point|point.0[0]>=0.5*fine));
            }
        }

        #[cfg(feature="body-query-profile")]
        #[test]
        fn dense_payload_counter_counts_visited_table_only_after_seed() {
            let keys=(-2..=2).flat_map(|x|(-2..=2).map(move |y|[x,y,-1])).collect();
            let (map,_)=make(keys);
            query_profile::begin();
            let absent=map.fine_contact_domain([0.0,0.0,4.0],0.3);
            let record=query_profile::end();
            assert_eq!(absent,Err(SurfaceError::MissingSupport));
            assert!(record.counters[QueryCounter::DenseAllocatedSlots as usize]>0);
            assert_eq!(record.counters[QueryCounter::DenseVisitedTableSlots as usize],0);
            query_profile::begin();
            let present=map.fine_contact_domain([0.0;3],0.3);
            let record=query_profile::end();
            assert!(present.is_ok());
            assert_eq!(record.counters[QueryCounter::DenseVisitedTableSlots as usize],
                record.counters[QueryCounter::DenseAllocatedSlots as usize]);
            assert!(record.counters[QueryCounter::DenseVisitedColumns as usize]>0);
        }

        #[test]
        fn masked_current_view_never_emits_inferred_support() {
            let mut keys:Vec<_>=(-10_i32..=10).flat_map(|x|(-10_i32..=10)
                .filter(move |y|x.abs()>=2 || y.abs()>=2).map(move |y|[x,y,-1])).collect();
            keys.push([0,0,6]); // The intersecting return remains a real collision, regardless of the gap.
            let (base,keys)=make(keys);
            for evidence in [GroundEvidence::MeasuredSurface,GroundEvidence::OccupiedVolume] {
                let map=base.clone().with_ground_evidence(evidence);
                assert!(map.with_current_body_support([0.0,0.0,0.35],[0.0,0.0,0.0,1.0]).is_err());
                let view=map.clone();
                for origin in [[0.0;3],[-0.075,-0.025,0.0],[0.1,0.075,0.0]] {
                    let original=compare(&map,&keys,origin,0.3).unwrap();
                    let masked=compare(&view,&keys,origin,0.3).unwrap();
                    assert_eq!(original.iter().map(point_bits).collect::<Vec<_>>(),
                        masked.iter().map(point_bits).collect::<Vec<_>>());
                }
            }
        }

        #[test]
        fn dense_limit_keeps_isolated_seed_and_sparse_in_dense_membership() {
            let fine=f64::from(0.05_f32);let origin=[0.5*fine,0.5*fine,0.0];
            let radius=(63.5-std::f64::consts::SQRT_2)*fine;
            let enumeration=radius+std::f64::consts::SQRT_2*fine;
            let low=((origin[0]-enumeration)/fine).floor() as i32;
            let high=((origin[0]+enumeration)/fine).floor() as i32;
            assert_eq!((high-low+1).pow(2),16_384,"exercise the unchanged dense capacity boundary");
            for keys in [vec![[0,0,-1]],vec![[0,0,-1],[-4,0,-1],[0,4,-1],[4,4,-1]],
                (-8..=8).flat_map(|x|(-8..=8).filter(move |y|(x+y)%3!=0).map(move |y|[x,y,-1])).collect()] {
                let count=keys.len();let (map,observed)=make(keys);
                assert_eq!(compare(&map,&observed,origin,radius).unwrap().len(),count);
            }
        }

        #[test]
        #[ignore = "bounded opt-in dense emission benchmark; reports collection, sort and fit separately"]
        fn dense_order_reachable_fraction_microbenchmark() {
            let fine=f64::from(0.05_f32);let origin=[0.5*fine,0.5*fine,0.0];
            let radius=(63.5-std::f64::consts::SQRT_2)*fine;
            for half in [0,2,8,32,63] {
                let keys=(-half..=half).flat_map(|x|(-half..=half).map(move |y|[x,y,-1])).collect();
                let (base,_)=make(keys);let old=base.fork_for_planning();let new=base.fork_for_planning();
                for pass in 0..3 {
                    // Independent caches; alternate collection order rather than always favoring one variant.
                    let (mut expected,old_domain,mut actual,new_domain)=if pass%2==0 {
                        let start=std::time::Instant::now();let expected=reference_bfs(&old,origin,radius).unwrap();
                        let old_domain=start.elapsed();
                        let start=std::time::Instant::now();let actual=new.fine_contact_domain(origin,radius).unwrap();
                        (expected,old_domain,actual,start.elapsed())
                    } else {
                        let start=std::time::Instant::now();let actual=new.fine_contact_domain(origin,radius).unwrap();
                        let new_domain=start.elapsed();
                        let start=std::time::Instant::now();let expected=reference_bfs(&old,origin,radius).unwrap();
                        (expected,start.elapsed(),actual,new_domain)
                    };
                    let start=std::time::Instant::now();sort(&mut expected);let old_sort=start.elapsed();
                    let start=std::time::Instant::now();sort(&mut actual);let new_sort=start.elapsed();
                    assert_eq!(expected.iter().map(point_bits).collect::<Vec<_>>(),actual.iter().map(point_bits).collect::<Vec<_>>());
                    let start=std::time::Instant::now();let expected_plane=fit_contact_plane(&expected,origin);let old_fit=start.elapsed();
                    let start=std::time::Instant::now();let actual_plane=fit_contact_plane(&actual,origin);let new_fit=start.elapsed();
                    assert_eq!(plane_bits(expected_plane),plane_bits(actual_plane));
                    eprintln!("half={half} pass={pass} points={} dense_slots=16384 old_domain={old_domain:?} new_domain={new_domain:?} old_sort={old_sort:?} new_sort={new_sort:?} old_fit={old_fit:?} new_fit={new_fit:?}",actual.len());
                }
            }
        }

        #[test]
        fn radius_edges_keep_empty_degenerate_and_sparse_fallback_results() {
            let fine=f64::from(0.05_f32);
            let (map,keys)=make((-2..=2).flat_map(|x|(-2..=2).map(move |y|[x,y,-1])).collect());
            let origin=[0.5*fine,0.5*fine,0.0];
            assert!(compare(&map,&keys,origin,0.0).unwrap().is_empty());
            assert_eq!(compare(&map,&keys,origin,fine*0.25).unwrap().len(),1);
            for radius in [f64::from_bits(fine.to_bits()-1),fine,f64::from_bits(fine.to_bits()+1)] {
                compare(&map,&keys,origin,radius).unwrap();
            }
            // This exceeds the existing dense-table limit; preserve the untouched sparse order too.
            let radius=4.0;
            let actual=map.fine_contact_domain(origin,radius).unwrap();
            let sparse=map.fine_contact_domain_sparse(origin,radius).unwrap();
            assert_eq!(actual.iter().map(point_bits).collect::<Vec<_>>(),
                sparse.iter().map(point_bits).collect::<Vec<_>>());
            assert_eq!(plane_bits(fit_contact_plane(&actual,origin)),plane_bits(fit_contact_plane(&sparse,origin)));
        }
    }

    #[test]
    fn saved_boundary_sampled_flat_ground_keeps_legal_motion_across_empty_rows() {
        // 原探索保存PCD原点附近真实列；保留单精度量化产生的全部空行，不移动或补采样。
        let columns=[-12,-10,-9,-8,-7,-5,-4,-2,-1,1,2,3,4,6,8,9,11,12];
        let keys=columns.into_iter().flat_map(|x|columns.into_iter().flat_map(move |y|[-7,12].map(|z|[x,y,z]))).collect();
        let mut envelope=model();envelope.max_step_up_m=0.0;envelope.max_step_down_m=0.0;
        let map=sparse_contact_scene(keys,envelope);
        let from=([0.0,0.0,0.0],[0.0,0.0,0.0,1.0]);
        for direction in [[1.0,0.0],[0.0,1.0],[1.0,1.0]] {for distance in [0.05,0.25,0.30,0.325,0.35,0.375,0.40] {
            let xy=direction.map(|v|v*distance);let expected=([xy[0],xy[1],0.0],from.1);
            map.validate_body_pose(expected.0,expected.1).unwrap();
            map.validate_body_motion(from,expected,&[]).unwrap();
            let actual=map.body_pose_after_motion(from,xy,0.0).expect("原合法平地不能因细拟合分量退化而被关闭");
            assert_eq!(actual,expected,"常值地面始终保留实际高度和四元数");
        }}
    }

    #[test]
    fn sparse_quantized_ramp_response_remains_continuous_at_old_rank_boundary() {
        let fine=f64::from(0.05_f32);let grade=0.25_f64;
        let keys=(-10..12).filter(|x|![0,5,7].contains(x)).flat_map(|x|(-6..6).filter(|&y|y!=0).map(move |y| {
            let px=(f64::from(x)+0.5)*fine;
            let z=((-0.3+grade*px-fine*0.5)/fine).floor() as i32;[x,y,z]
        })).collect();
        let mut envelope=model();envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
        let map=sparse_contact_scene(keys,envelope);
        let radius=map.body_envelope().unwrap().radius()+fine;
        let plane=|x:f64| {
            let origin=[x,0.0,-0.3+grade*x];
            let mut points=map.fine_contact_domain(origin,radius).unwrap();
            points.sort_by(|a,b|a.0[0].total_cmp(&b.0[0]).then_with(||a.0[1].total_cmp(&b.0[1])));
            fit_contact_plane(&points,origin).expect("邻近真实同层样本应保留坡面的可观测方向")
        };
        let angle=-grade.atan();let q=[0.0,(angle*0.5).sin(),0.0,(angle*0.5).cos()];
        for delta in [1e-3,1e-5,1e-7] {
            let left=5.5*fine-delta;let right=5.5*fine+delta;let a=plane(left);let b=plane(right);
            assert!(a.0>0.1 && b.0>0.1,"不能在旧单列分量边界突然压平坡面: {a:?} {b:?}");
            assert!((b.2-a.2).abs()<2.0*delta && (b.0-a.0).abs()+(b.1-a.1).abs()<10.0*delta,
                "真实微动变小时高度和姿态变化须同步趋零: {delta} {a:?} {b:?}");
            let from=([left,0.0,-0.3+grade*left+0.35*(1.0+grade*grade).sqrt()],q);
            map.validate_body_pose(from.0,from.1).unwrap();
            let next=map.body_pose_after_motion(from,[right,0.0],0.0).unwrap();
            map.validate_body_motion(from,next,&[]).unwrap();
            assert!((next.0[2]-from.0[2]).abs()<2.0*delta && next.1.iter().zip(from.1).all(|(a,b)|(a-b).abs()<10.0*delta));
        }
    }

    #[test]
    fn relative_terrain_whole_footprint_descends_a_real_thick_step_and_closes() {
        let q=f64::from(0.05_f32);let mut points=Vec::new();
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for x in -40_i32..40 {for y in -20_i32..20 {
            let top=if x<0 {0}else{-3};
            for z in top-2..top {
                let key=[x,y,z];points.push((((f64::from(x)+0.5)*q)as f32,((f64::from(y)+0.5)*q)as f32,((f64::from(z)+0.5)*q)as f32));
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }
        }}
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}observations.mark_complete_geometry_source();
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=3.0*q;envelope.max_step_down_m=3.0*q;envelope.max_cross_slope_deg=18.0;
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)));
        let start=([-0.6,0.025,0.4],[0.0,0.0,0.0,1.0]);let mut current=start;
        for i in (1..=120).chain((0..120).rev()) {
            let x=-0.6+f64::from(i)*0.01;let next=map.body_pose_after_motion(current,[x,0.025],0.0).unwrap();
            map.validate_body_motion(current,next,&[]).unwrap_or_else(|e|panic!("厚阶原扫掠x={x}：{e}；{current:?}->{next:?}"));
            if i==120 {assert!((next.0[2]-(0.4-3.0*q)).abs()<1e-8,"全部足迹已进入低台，不能悬停在高台参考：{next:?}");}
            current=next;
        }
        assert!((current.0[2]-start.0[2]).abs()<1e-8);
        assert!(current.1.iter().zip(start.1).all(|(a,b)|(a-b).abs()<1e-8));
    }

    #[test]
    fn relative_terrain_motion_continuously_follows_three_observed_thin_steps() {
        let mut points=Vec::new();
        for x in -40..60 {for y in -20..20 {
            let step=i32::from(x>=0)+i32::from(x>=8)+i32::from(x>=16);
            let top=step*2;
            let first=if [0,8,16].contains(&x) {top-2}else{top-1};
            for z in first..top {points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));}
        }}
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
        envelope.max_cross_slope_deg=18.0;
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
            .with_observations(Some(Arc::new(observations)));
        let start=([-0.6,0.025,0.4],[0.0,0.0,0.0,1.0]);let mut current=start;
        let mut checkpoints=Vec::new();
        for round in 0..3 {
            for i in (1..=220).chain((0..220).rev()) {
                let x=-0.6+f64::from(i)*0.01;
                let next=map.body_pose_after_motion(current,[x,0.025],0.0).unwrap_or_else(|e|panic!("三阶相对推进round={round}, x={x}: {e}; from={current:?}"));
                map.validate_body_motion(current,next,&[]).unwrap_or_else(|e|panic!("三阶实际扫掠round={round}, x={x}: {e}; {current:?} -> {next:?}"));
                assert!((next.0[2]-current.0[2]).abs()<0.02,"连续小步不能突然跨越整阶高度");
                if i==220 || i==0 {checkpoints.push((round,i,next));}
                current=next;
            }
        }
        for (round,index,pose) in &checkpoints {
            let expected=if *index==220 {0.7}else{0.4};
            assert!((pose.0[2]-expected).abs()<1e-7,"立面观测区间不能造成往返高度累积：{checkpoints:?}");
            assert!(pose.1[0].abs()<1e-7 && pose.1[1].abs()<1e-7,"真实水平两端不能累计倾斜：round={round}, {pose:?}");
        }
    }

    #[test]
    fn relative_terrain_motion_still_follows_a_real_ramp() {
        let points:Vec<_>=(-50..50).flat_map(|x|(-30..30).map(move|y| {
            let px=(x as f32+0.5)*0.05;(px,(y as f32+0.5)*0.05,px*0.13+0.021)
        })).collect();
        let map=SurfaceSnapshot::from_points(&points,scene(&[],false).config().clone(),0.05).unwrap()
            .with_body_envelope(model()).unwrap();
        let support=map.locate_support([-0.55,0.05,-0.05]).unwrap().support_xyz;
        let q=map.body_orientation_at(support,0.0).unwrap();
        let mut from=(map.body_position_from_support(support,q,0.4).unwrap(),q);
        from.0[2]-=0.02;
        map.validate_body_pose(from.0,from.1).unwrap();
        let next=map.body_pose_after_motion(from,[0.55,0.05],0.0).unwrap();
        assert!(next.0[2]-from.0[2]>0.07,"真实上坡仍须抬升");
        assert!(next.0[2]-from.0[2]<0.18,"不能叠加格顶的绝对高度偏差");
        map.validate_body_motion(from,next,&[]).unwrap();
    }

    #[test]
    fn contact_connectivity_uses_only_the_fine_ground_inside_the_footprint() {
        let make = |missing_inside: bool| {
            let mut points = Vec::new();
            for x in -20..20 { for y in -16..16 {
                if missing_inside && (0..2).contains(&x) && y==2 { continue; }
                for z in [-0.025,0.025,0.075] {
                    points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,z));
                }
            }}
            // 高处回波位于同一粗列的足迹外半格；外侧台阶让它进入粗图接触邻域。
            for x in [0.025,0.075] {
                for z in [0.125,0.175,0.225,0.325,0.375] {points.push((x,0.175,z));}
                for y in [0.225,0.275] {for z in [0.125,0.175,0.225] {points.push((x,y,z));}}
            }
            let mut config = scene(&[],false).config().clone();
            config.robot_height=0.1;config.wall_clearance_m=0.0;config.step_threshold_m=0.15;
            let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.12],[0.3,-0.12],[0.3,0.12],[-0.3,0.12]];
            envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }
            let mut observations=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
            SurfaceSnapshot::from_points(&points,config,0.05).unwrap().with_body_envelope(envelope).unwrap()
                .with_observations(Some(Arc::new(observations)))
        };
        let body=[0.05,0.0,0.35];let q=[0.0,0.0,0.0,1.0];
        let map=make(false);
        map.validate_body_pose(body,q).expect("足迹外的高表面不能替换足迹内的真实低层接触");
        let coarse=map.with_observations(None);
        assert!(!Arc::ptr_eq(&map.body_query_cache,&coarse.body_query_cache),"removing fine evidence retires its exact contact cache");
        assert!(coarse.observations.is_none());
        assert!(map.locate_support_from_body(body,q,0.4).is_ok());
        assert!(map.validate_body_pose([0.05,0.1,0.35],q).is_err(),"进入真实高处回波后仍应拒绝");
        assert!(make(true).validate_body_pose(body,q).is_err(),"缺少足迹内精细地面时不能借足迹外的低层补足");
    }

    #[test]
    fn current_body_view_preserves_legal_ground_band_instead_of_digging_a_pit() {
        let mut config = scene(&[], false).config().clone();
        config.robot_height = 0.1;
        config.wall_clearance_m = 0.0;
        config.step_threshold_m = 0.15;
        let mut envelope = model();
        envelope.z_bounds_m = [-0.448, 0.057];
        envelope.max_step_up_m = 0.15;
        envelope.max_step_down_m = 0.15;
        let mut points: Vec<_> = (-20..20).flat_map(|x| (-10..10).flat_map(move |y| {
            [-0.025,0.025,0.075,0.125,0.175].into_iter().map(move |z| {
                ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, z)
            })
        })).collect();
        // 足迹内上方回波触发原有当前占据清除；周边实际地面不能随之降低。
        points.push((0.15,0.05,0.325));
        let map = SurfaceSnapshot::from_points(&points, config, 0.05).unwrap()
            .with_body_envelope(envelope).unwrap();
        let q = [0.0,0.0,0.0,1.0];
        let body = [0.05,0.05,0.4];
        assert!(map.validate_body_pose(body,q).is_err());
        let before = map.locate_support_from_body(body,q,0.4).unwrap();
        assert!(map.with_current_body_support(body,q).is_err(),"intersecting source echo is retained");
        let view = map.clone();
        let after = view.locate_support_from_body(body,q,0.4).unwrap();
        assert!((before.surface_xyz[2]-after.surface_xyz[2]).abs()<1e-6,
            "已在完整机体接触范围内的真实厚层上界必须保留");
        assert!((after.surface_xyz[2]-0.2).abs()<1e-6);
        assert!(view.validate_body_motion((body,q),([-0.85,0.05,0.4],q),&[]).is_err());
    }

    #[test]
    fn fine_inner_ground_remains_support_when_outer_half_is_a_wall() {
        let mut points:Vec<_>=(-20..20).flat_map(|x|(-20..20).map(move |y|
            ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,-0.025))).collect();
        for x in [0,1] {for z in 0..12 {points.push(((x as f32+0.5)*0.05,-0.075,(z as f32+0.5)*0.05));}}
        for x in [0,1] {points.push(((x as f32+0.5)*0.05,-0.025,0.625));}
        let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.0;cfg.robot_height=0.1;
        let envelope=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.15,-0.1],[0.15,-0.1],[0.15,0.1],[-0.15,0.1]],
            z_bounds_m:[-0.4,0.1],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let base=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let observed=|missing:bool| {
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
                if missing && key==[0,-1,-1] {continue;}
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation {observed_ns:0,hits:0});
            }
            let mut grid=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {grid.replace_block(key,values).unwrap();}
            base.with_observations(Some(Arc::new(grid)))
        };
        let p=[0.025,0.08,0.4];let q=[0.0,0.0,0.0,1.0];
        assert!(base.validate_body_pose(p,q).is_err(),"粗图自身不能证明墙边内半格可站立");
        let map=observed(false);
        map.validate_body_pose(p,q).expect("真实投影只覆盖内半列的连续地面");
        let missing=observed(true);
        assert!(missing.observations.as_ref().unwrap().has_complete_block([0,-1,-1]));
        assert!(missing.observations.as_ref().unwrap().get([0,-1,-1]).is_none());
        assert!(missing.validate_body_pose(p,q).is_err(),"块存在不能替代实际细地面证据");
        let mut grid=map.observations.as_ref().unwrap().as_ref().clone();
        grid.replace_block([0,-1,0],std::collections::BTreeMap::new()).unwrap();
        let unknown=base.with_observations(Some(Arc::new(grid)));
        let polygon=vec![[p[0]-0.15,p[1]-0.1],[p[0]+0.15,p[1]-0.1],
            [p[0]+0.15,p[1]+0.1],[p[0]-0.15,p[1]+0.1]];
        assert!(unknown.fine_footprint_ground(&PolygonQuery::new(&polygon),p,[0.0,0.0,1.0],0,-1).is_none(),
            "缺失上方完整块不能截断连续高墙并制造层顶");
        assert!(map.validate_body_pose([p[0],0.049,p[2]],q).is_err(),"踏入外侧高墙时不能把墙截成地面");
    }

    #[test]
    fn fine_ground_height_and_contact_plane_use_the_same_spatial_column() {
        let mut points=Vec::new();
        for x in -30..30 {for y in -20..20 {for z in [0.025,0.075] {
            points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,z));
        }}}
        // 较高地面只在支撑平面较高的半列，不能与另一半列的最低支撑值配对。
        for x in [0,1] {for z in [0.125,0.175] {points.push(((x as f32+0.5)*0.05,0.075,z));}}
        let mut cfg=scene(&[],false).config().clone();cfg.robot_height=0.1;cfg.wall_clearance_m=0.0;
        let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
        envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
        envelope.max_cross_slope_deg=18.0;
        let base=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let make=|missing:bool| {
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
                if missing && key[0]==0 && key[1]==0 {continue;}
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }
            let mut observations=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
            base.with_observations(Some(Arc::new(observations)))
        };
        let angle=0.1_f64;let q=[(angle*0.5).sin(),0.0,0.0,(angle*0.5).cos()];
        let body=[0.05,-0.06,0.4/angle.cos()-0.002-0.06*angle.tan()];
        assert!(base.validate_body_pose(body,q).is_err(),"只有粗高度时不能假设高点位于安全半列");
        let map=make(false);
        map.validate_body_pose(body,q).expect("每个真实细列的地面都满足该列原接触范围");
        assert_eq!(map.validate_body_pose([body[0],body[1],body[2]-0.004],q),Err(SurfaceError::InsufficientClearance),
            "同一真实高细列超过接触范围仍应拒绝");
        assert!(make(true).validate_body_pose(body,q).is_err(),"细列缺失不能用整格高度补出联合许可");
        let mut slot=None;
        let rejected=[body[0],body[1],body[2]-0.004];
        assert_eq!(map.validate_body_pose_with_witness(rejected,q,&mut slot),Err(SurfaceError::InsufficientClearance));
        let witness=slot.unwrap();
        assert_witness_cold_and_warm(&map,(rejected,q),slot);
        assert!(matches!(witness.rejection,BodyRejection::Occupancy{..}),"actual fine volume intersection: {witness:?}");
        map.validate_body_pose_with_witness(body,q,&mut slot).unwrap();assert!(slot.is_none());
        assert_eq!(base.validate_body_pose_with_witness(body,q,&mut slot),Err(SurfaceError::InsufficientClearance));
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{source:OccupancySource::CoarseFallback,..}));
        assert_witness_cold_and_warm(&base,(body,q),slot);
        let missing=make(true);
        assert_eq!(missing.validate_body_pose_with_witness(body,q,&mut slot),Err(SurfaceError::InsufficientClearance));
        assert!(matches!(slot.unwrap().rejection,BodyRejection::Occupancy{..}));
        assert_witness_cold_and_warm(&missing,(body,q),slot);

    }

    #[test]
    fn partial_footprint_contact_uses_actual_overlap_not_outside_grid_centre() {
        let mut points:Vec<_>=(-20..20).flat_map(|x|(-20..20).flat_map(move |y|
            [0.025,0.075].map(|z|((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,z)))).collect();
        points.push((0.025,-0.025,0.175));
        let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.0;
        let envelope=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.2,-0.1],[0.2,-0.1],[0.2,0.1],[-0.2,0.1]],
            z_bounds_m:[-0.4,0.1],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap();
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points {
            let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation {observed_ns:0,hits:0});
        }
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();
        for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
        let map=map.with_observations(Some(Arc::new(observations)));
        let tilt=0.1_f64;let q=[(tilt*0.5).sin(),0.0,0.0,(tilt*0.5).cos()];
        let height=0.2-(0.15+f64::from(0.05_f32))-0.0005+0.4/tilt.cos()-tilt.tan()*(-0.05-0.075);
        let body=[0.025,0.075,height];
        map.validate_body_pose(body,q).expect("粗列中心在机体投影外，实际相交区域仍在原接触行程内");
        assert_eq!(map.validate_body_pose([body[0],body[1],body[2]-0.002],q),Err(SurfaceError::InsufficientClearance),
            "真实交叠区域超过同一行程仍须拒绝");
    }

    #[test]
    fn fine_contact_connection_checks_intermediate_peak_not_only_equal_end_tops() {
        let make=|top:i32| {
            let mut points=Vec::new();
            for x in -30..30 {for y in -20..20 {
                let mut levels=vec![-1];
                if (0..2).contains(&x) {levels.extend(0..top);}
                for z in levels {points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,(z as f32+0.5)*0.05));}
            }}
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});
            }
            let mut observed=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {observed.replace_block(key,values).unwrap();}
            let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.0;cfg.robot_height=0.1;
            let mut envelope=model();envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
            SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(envelope).unwrap()
                .with_observations(Some(Arc::new(observed)))
        };
        let from=([-0.0001,0.025,0.4],[0.0,0.0,0.0,1.0]);let to=([0.1001,0.025,0.4],from.1);
        make(2).validate_body_motion(from,to,&[]).expect("同一已观测厚带内的10厘米中间顶受原15厘米能力约束");
        let high=make(4);
        high.validate_body_pose(from.0,from.1).unwrap();high.validate_body_pose(to.0,to.1).unwrap();
        assert!(high.validate_body_motion(from,to,&[]).is_err(),
            "两端等高也不能跳过中间超过原15厘米能力的20厘米顶");
    }

    #[test]
    fn fine_same_level_support_does_not_inherit_another_half_columns_step() {
        let make=|raise_crossed_half:bool,missing_fine:u8| {
            let mut points:Vec<_>=(-30..30).flat_map(|x|(-20..20).map(move |y|
                ((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,-0.025))).collect();
            for x in if raise_crossed_half {vec![0,1]}else{vec![1]} {
                for z in [0.025,0.075,0.125] {points.push(((x as f32+0.5)*0.05,0.025,z));}
            }
            let mut cfg=scene(&[],false).config().clone();cfg.wall_clearance_m=0.0;
            let map=SurfaceSnapshot::from_points(&points,cfg,0.05).unwrap().with_body_envelope(model()).unwrap();
            let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
            for &(x,y,z) in &points {
                let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);
                if (missing_fine==1 && key[0]==0 && key[1]==0)
                    || (missing_fine==2 && key[0]==-1 && key[1]==0) {continue;}
                blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation {observed_ns:0,hits:0});
            }
            let mut grid=SurfaceObservations::new(0.05,32).unwrap();
            for (key,values) in blocks {grid.replace_block(key,values).unwrap();}
            map.with_observations(Some(Arc::new(grid)))
        };
        let from=([0.025,-0.0001,0.4],[0.0,0.0,0.0,1.0]);
        let to=([0.025,0.0001,0.4],from.1);
        let map=make(false,0);
        map.validate_body_pose(from.0,from.1).unwrap();map.validate_body_pose(to.0,to.1).unwrap();
        let a=map.locate_support_from_body(from.0,from.1,0.4).unwrap();
        let b=map.locate_support_from_body(to.0,to.1,0.4).unwrap();
        assert_eq!(map.validate_segment(a.support_xyz,b.support_xyz),Err(SurfaceError::UntraversableSegment),
            "粗格把另一半列的高回波压到了真实经过的低平地面");
        map.validate_body_motion(from,to,&[]).expect("同一真实精细平层上的微小运动应连续");
        let adapted=map.body_pose_after_motion(from,[to.0[0],to.0[1]],0.0).unwrap();
        map.validate_body_motion(from,adapted,&[]).unwrap();
        assert!((adapted.0[2]-from.0[2]).abs()<(to.0[1]-from.0[1]).abs(),
            "足迹内其它真实高地面允许连续适应，但不能继承中心粗格的突然台阶");
        let smaller=map.body_pose_after_motion(from,[to.0[0],from.0[1]+(to.0[1]-from.0[1])*0.1],0.0).unwrap();
        assert!((smaller.0[2]-from.0[2]).abs()<(adapted.0[2]-from.0[2]).abs()*0.2+1e-12,
            "露顶高度传播须随实际位移连续趋零");
        let missing=make(false,1);
        missing.validate_body_motion(from,to,&[]).unwrap();
        assert!(missing.known_common_band_samples(from,to).is_none(),"geometry clearance does not fabricate a missing fine-contact certificate");
        let raised=make(true,0);
        raised.validate_body_motion(from,to,&[]).expect("真实厚带的0.15米升高仍在原0.2米上阶能力内");
        assert!(raised.validate_body_motion(to,from,&[]).is_err(),"同一0.15米下降超过原0.1米下阶能力");
        let mut lower_capability=model();lower_capability.max_step_up_m=0.1;
        assert!(raised.with_body_envelope(lower_capability).unwrap().validate_body_motion(from,to,&[]).is_err(),
            "精细厚带证明不能越过原上下阶能力");
        // 实际标定参考略高于真实露顶仍在原源体素量化范围内；粗半列抬高不能令微动失效。
        for elevation in [0.006,0.049] {
            let a=([from.0[0],from.0[1],from.0[2]+elevation],from.1);
            let b=([to.0[0],to.0[1],to.0[2]+elevation],to.1);
            map.validate_body_motion(a,b,&[]).expect("同一细带露顶的原量化误差须跨微动连续");
            let adapted=map.body_pose_after_motion(a,[b.0[0],b.0[1]],0.0).unwrap();
            map.validate_body_motion(a,adapted,&[]).unwrap();
            make(false,1).validate_body_motion(a,b,&[]).unwrap();
            assert!(make(false,1).known_common_band_samples(a,b).is_none());
        }
        let too_high=([from.0[0],from.0[1],from.0[2]+0.051],from.1);
        let too_high_end=([to.0[0],to.0[1],to.0[2]+0.051],to.1);
        map.validate_body_motion(too_high,too_high_end,&[]).unwrap();
        assert!(map.known_common_band_samples(too_high,too_high_end).is_none(),
            "no contact certificate is granted to the floating reference; actual clearance is separate");
        let diagonal_from=([-0.0001,-0.0001,0.4],from.1);
        let diagonal_to=([0.0001,0.0001,0.4],from.1);
        map.validate_body_motion(diagonal_from,diagonal_to,&[]).unwrap();
        make(false,2).validate_body_motion(diagonal_from,diagonal_to,&[]).unwrap();
        assert!(make(false,2).known_common_band_samples(diagonal_from,diagonal_to).is_none(),
            "diagonal missing contact remains diagnostic unknown, not a synthetic observation");
    }

    #[test]
    fn translated_body_sweep_catches_a_short_corner_contact_between_regular_samples() {
        let map=ground_band_sweep_scene(0.525);
        let q=[0.0,0.0,0.0,1.0];
        let from=([-0.25,0.0295,0.375],q);
        let to=([0.06,0.3395,0.375],q);
        map.validate_body_pose(from.0,q).unwrap();
        map.validate_body_pose(to.0,q).unwrap();
        let t=0.15/0.31;
        let contact=[from.0[0]*(1.0-t)+to.0[0]*t,from.0[1]*(1.0-t)+to.0[1]*t,0.375];
        assert_eq!(map.validate_body_pose(contact,q),Err(SurfaceError::InsufficientClearance));
        assert_eq!(map.validate_body_motion(from,to,&[]),Err(SurfaceError::InsufficientClearance),
            "不能漏过不到一毫米的真实角点碰撞区间");
    }

    #[test]
    fn slightly_rotating_body_sweep_keeps_short_corner_contacts() {
        let map=ground_band_sweep_scene(0.525);
        for angle in [-0.0011_f64,0.0011] {
            let from=([-0.25,0.0295,0.375],[0.0,0.0,0.0,1.0]);
            let to=([0.06,0.3395,0.375],[0.0,0.0,(angle*0.5).sin(),(angle*0.5).cos()]);
            map.validate_body_pose(from.0,from.1).unwrap();
            map.validate_body_pose(to.0,to.1).unwrap();
            // 用独立的局部细步扫描找真实见证，微旋转会移动纯平移的接触时刻。
            let witness=(0..=300).find(|&index| {
                let t=0.47+index as f64*0.0001;
                let p=std::array::from_fn(|i|from.0[i]*(1.0-t)+to.0[i]*t);
                let q=std::array::from_fn(|i|from.1[i]*(1.0-t)+to.1[i]*t);
                map.validate_body_pose(p,q)==Err(SurfaceError::InsufficientClearance)
            });
            assert!(witness.is_some(),"此场景应含微旋转后的真实短碰撞");
            assert_eq!(map.validate_body_motion(from,to,&[]),Err(SurfaceError::InsufficientClearance),
                "真实微小姿态变化不能绕过连续接触核验");
        }
    }

    #[test]
    fn actual_body_sweep_cannot_raise_the_body_over_a_low_obstacle() {
        let map = ground_band_sweep_scene(0.275);
        let q = [0.0, 0.0, 0.0, 1.0];
        let from = ([-0.65, 0.05, 0.375], q);
        let to = ([0.65, 0.05, 0.375], q);
        map.validate_body_pose(from.0, q).unwrap();
        map.validate_body_pose(to.0, q).unwrap();
        assert_eq!(map.validate_body_pose([0.05,0.05,0.375], q), Err(SurfaceError::InsufficientClearance));
        assert_eq!(map.validate_body_motion(from, to, &[]), Err(SurfaceError::InsufficientClearance),
            "不能把实际机体抬到地面厚层顶部后越过真实低障碍");
    }

    #[test]
    fn actual_body_sweep_does_not_raise_the_body_into_a_low_ceiling() {
        let map = ground_band_sweep_scene(0.675);
        let q = [0.0, 0.0, 0.0, 1.0];
        let from = ([-0.65, 0.05, 0.375], q);
        let to = ([0.65, 0.05, 0.375], q);
        for step in 0..=130 {
            map.validate_body_pose([-0.65 + step as f64 * 0.01,0.05,0.375], q).unwrap();
        }
        map.validate_body_motion_checked(from, to, &[], |(body, _)| {
            assert!((body[2]-0.375).abs()<1e-10, "观察检查必须使用实际机体轨迹");
            Ok(())
        }).unwrap();
    }

    #[test]
    fn exact_geometry_cache_keeps_boundary_and_new_obstacle_checks() {
        let clear=scene(&[],false);
        let from=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let to=([0.25,0.05,0.5],from.1);
        for _ in 0..2 {
            clear.validate_body_motion(from,to,&[]).unwrap();
            clear.body_pose_on_surface([0.05,0.05],0.0,[0.05,0.05,0.1]).unwrap();
        }
        assert_eq!(clear.body_query_cache.lock().unwrap().motions.len(),1);
        assert_eq!(clear.body_query_cache.lock().unwrap().projections.len(),1);
        let outside=[[-1.0,-1.0],[-0.5,-1.0],[-0.5,-0.5],[-1.0,-0.5]];
        assert!(clear.validate_body_motion(from,to,&outside).is_err());
        let changed=scene(&[(0.55,0.05,0.55)],false);
        assert!(changed.validate_body_motion(from,to,&[]).is_err());
        assert!(changed.body_pose_on_surface([0.05,0.05],0.0,[0.05,0.05,0.1]).is_err());
        // 换观察、模型或实际占据视图均使用各自缓存，不能沿用旧净空许可。
        let observed=clear.with_observations(Some(Arc::new(super::SurfaceObservations::new(0.05,32).unwrap())));
        assert!(!Arc::ptr_eq(&clear.body_query_cache,&observed.body_query_cache));
        let modeled=clear.with_body_envelope(model()).unwrap();
        assert!(!Arc::ptr_eq(&clear.body_query_cache,&modeled.body_query_cache));
        let unchanged=clear.with_current_body_support(from.0,from.1).unwrap();
        assert!(Arc::ptr_eq(&clear.body_query_cache,&unchanged.body_query_cache));
        let blind=scene(&[],true);
        let current=blind.with_current_body_support(from.0,from.1).unwrap();
        assert!(Arc::ptr_eq(&blind.body_query_cache,&current.body_query_cache),"no inferred support view is needed when only a footprint sample is absent");
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
        scene(&[], true).validate_body_pose(p, q).unwrap(); // unknown floor is not occupied
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
        let lower = [1.05,1.05,0.025];
        let upper = [4.05,1.05,0.175];
        let to_upper = s.goal_travel_costs(&[upper],&[]).unwrap();
        let to_lower = s.goal_travel_costs(&[lower],&[]).unwrap();
        let forward = s.frontier_travel_costs(lower,&[upper],&[]).unwrap()[0];
        let a=s.locate_support(lower).unwrap();let b=s.locate_support(upper).unwrap();
        let offset=|p:[f64;3],q:[f64;3]|(p[0]-q[0]).hypot(p[1]-q[1]).hypot(p[2]-q[2]);
        let graph=&s.data.graph;
        let start_id=graph.cells.id(a.cell).unwrap();let goal_id=graph.cells.id(b.cell).unwrap();
        let costs=nuwax_explore_routing::shortest_costs(graph.cells.slot_capacity(),start_id as usize,None,
            |id,out|out.extend(graph.cells.neighbors(id as CellId).iter().filter(|edge|edge.cost.is_finite())
                .map(|edge|(edge.dest as usize,f64::from(edge.cost)))));
        let weighted=to_upper.cost(a.cell).unwrap();
        assert!((weighted-costs[goal_id as usize]).abs()<1e-7,
            "目标场应等于正向允许边的实际加权代价");
        assert!(weighted+offset(lower,a.surface_xyz)+offset(upper,b.surface_xyz)+1e-5>=forward,
            "含台阶及软墙距的代价不得小于原几何距离");
        assert!(to_lower.cost(s.locate_support(upper).unwrap().cell).is_none(),
            "反向距离场不能把不允许下降的台阶变成双向边");
    }
    #[test]
    fn current_body_query_never_clears_occupied_cells_and_keeps_environment_immutable() {
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
        assert!(base.with_current_body_support(pose.0,pose.1).is_err());
        let view=base.clone();
        assert!(view.validate_body_pose(pose.0,pose.1).is_err());
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
    fn missing_fine_observations_do_not_erase_known_body_obstacles() {
        let base = scene(&[(0.25, 0.05, 0.65)], false);
        let position = [0.05, 0.05, 0.5];
        let orientation = [0.0, 0.0, 0.0, 1.0];
        assert!(base.validate_body_pose(position, orientation).is_err());
        let empty = Arc::new(SurfaceObservations::new(0.05, 32).unwrap());
        assert!(base.with_observations(Some(empty))
            .validate_body_pose(position, orientation).is_err());
    }

    #[test]
    fn fine_current_body_query_preserves_near_and_external_obstacles() {
        // 相同粗列同时容纳自体回波和外部障碍；整体高度平移不能改变结论。
        for (offset, obstacle) in [-200, 0, 200].into_iter()
            .flat_map(|offset| [false, true].map(move |obstacle| (offset, obstacle))) {
            let mut values = std::collections::BTreeMap::<[i32; 3], SurfaceObservation>::new();
            for x in -40..40 { for y in -30..30 {
                values.insert([x, y, offset], SurfaceObservation { observed_ns: 10, hits: 2 });
            }}
            for z in 2..=6 {
                values.insert([10, 0, offset + z], SurfaceObservation { observed_ns: 10, hits: 2 });
            }
            if obstacle {
                values.insert([11, 0, offset + 8], SurfaceObservation { observed_ns: 10, hits: 2 });
            }
            let size = 0.05_f32;
            let points: Vec<_> = values.keys().map(|k| (
                (k[0] as f32 + 0.5) * size, (k[1] as f32 + 0.5) * size,
                (k[2] as f32 + 0.5) * size,
            )).collect();
            let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
            for (key, value) in values {
                blocks.entry(key.map(|v| v.div_euclid(32))).or_default().insert(key, value);
            }
            let mut observations = SurfaceObservations::new(size, 32).unwrap();
            for (key, values) in blocks { observations.replace_block(key, values).unwrap(); }
            let observations = Arc::new(observations);
            let mut envelope = model();
            envelope.footprint_xy_m = vec![[-0.53, -0.14], [0.53, -0.14],
                [0.53, 0.14], [-0.53, 0.14]];
            envelope.z_bounds_m = [-0.448, 0.057];
            let base = SurfaceSnapshot::from_points(&points, scene(&[], false).config().clone(), size)
                .unwrap().with_body_envelope(envelope).unwrap()
                .with_observations(Some(Arc::clone(&observations)));
            let position = [0.0, 0.0, f64::from(offset) * f64::from(size) + 0.44];
            let orientation = [0.0, 0.0, 0.0, 1.0];
            assert!(base.validate_body_pose(position, orientation).is_err());
            assert!(base.with_current_body_support(position,orientation).is_err());
            let view=base.clone();
            assert!(view.validate_body_pose(position,orientation).is_err());
            let target = [0.15, position[1], position[2]];
            if obstacle {
                assert!(view.validate_body_pose(target, orientation).is_err(),
                    "不能清除同一粗列中的外部障碍");
            } else {
                assert!(view.validate_body_motion((position,orientation),(target,orientation),&[]).is_err(),
                    "occupied returns cannot be assumed to be the robot itself");
            }
            assert!(base.validate_body_pose(position, orientation).is_err(), "原环境不能被清除");
            assert!(Arc::ptr_eq(view.observations().unwrap(), &observations));
            assert_eq!(observations.get([10, 0, offset + 6]).unwrap().observed_ns, 10);
        }
    }

    #[test]
    fn tilted_body_shadow_does_not_turn_external_echoes_into_high_ground() {
        for offset in [-100, 0, 100] {
            for inside in [false, true] {
                let mut values = std::collections::BTreeMap::<[i32; 3], SurfaceObservation>::new();
                for x in -30..30 { for y in -30..30 {
                    values.insert([x, y, offset], SurfaceObservation { observed_ns: 10, hits: 2 });
                    values.insert([x, y, offset + 1], SurfaceObservation { observed_ns: 10, hits: 2 });
                }}
                for z in 2..=5 {
                    values.insert([-1, if inside { -4 } else { -5 }, offset + z],
                        SurfaceObservation { observed_ns: 10, hits: 2 });
                }
                let fine = 0.05_f32;
                let points: Vec<_> = values.keys().map(|k| (
                    (k[0] as f32 + 0.5)*fine, (k[1] as f32 + 0.5)*fine,
                    (k[2] as f32 + 0.5)*fine)).collect();
                let mut blocks = std::collections::BTreeMap::<_, std::collections::BTreeMap<_, _>>::new();
                for (key, value) in values {
                    blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key, value);
                }
                let mut observations = SurfaceObservations::new(fine, 32).unwrap();
                for (key, values) in blocks { observations.replace_block(key, values).unwrap(); }
                let mut envelope = model();
                envelope.footprint_xy_m = vec![[-0.24,-0.164],[0.34,-0.164],[0.34,0.164],[-0.24,0.164]];
                envelope.z_bounds_m = [-0.448,0.057];
                envelope.max_step_up_m = 0.15;
                envelope.max_step_down_m = 0.15;
                envelope.max_cross_slope_deg = 18.0;
                let observations=Arc::new(observations);
                for mls in [0.05,0.1] {
                let mut cfg=scene(&[],false).config().clone();cfg.voxel_size=mls;
                let map=SurfaceSnapshot::from_points(&points,cfg,fine).unwrap()
                    .with_body_envelope(envelope.clone()).unwrap().with_observations(Some(observations.clone()));
                let position = [0.027,-0.019,0.4534 + f64::from(offset)*f64::from(fine)];
                let orientation = [-0.0188553,0.0069169,0.0021580,0.9997960];
                let result=map.validate_body_pose(position,orientation);
                assert_eq!(result.is_err(),inside,"MLS={mls}, offset={offset}, inside={inside}: {result:?}");
                if !inside {map.with_current_body_support(position,orientation).unwrap()
                    .validate_body_pose(position,orientation).unwrap();}
                }
            }
        }
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
            base.validate_body_pose(pose.0,pose.1).unwrap();
            let view = base.with_current_body_support(pose.0, pose.1)
                .unwrap_or_else(|error| panic!("当前足迹盲区不能阻止起步，朝向{yaw}: {error}"));
            view.validate_body_pose(pose.0, pose.1).unwrap();
            let target = ([pose.0[0] + yaw.cos() * 0.8, pose.0[1] + yaw.sin() * 0.8, pose.0[2]], pose.1);
            view.validate_body_motion(pose, target, &[]).unwrap();
            base.validate_body_pose(pose.0,pose.1).unwrap();
            assert!(view.same_body_query_view(&base),"no floor was added to depart");
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
        assert_eq!(view.body_observation_footprint(start),Err(SurfaceError::MissingSupport));
        let occupied=std::collections::BTreeSet::new();
        view.validate_observed_body_motion(start,target,&occupied,10_100_000_000,1_000_000_000).unwrap();
        view.validate_observed_body_motion(start,target,&occupied,12_000_000_000,1_000_000_000).unwrap();
        assert_observation_checker_matches(&view, &occupied, 1_000_000_000, &[], &[
            (start, target, 10_100_000_000), (target, target, 10_100_000_000),
            (target, target, 12_000_000_000), (start, start, 12_000_000_000),
        ]);
        let advanced = view.with_current_body_support(target.0,target.1).unwrap();
        assert_observation_checker_matches(&advanced, &occupied, 1_000_000_000, &[], &[
            (start, target, 10_100_000_000), (target, target, 10_100_000_000),
        ]);
        advanced.validate_body_pose(start.0,start.1).unwrap(); // geometric clearance is not inferred support
        base.validate_body_pose(start.0,start.1).unwrap();
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
        view.validate_body_motion(pose,([0.025,0.0,0.5],pose.1),&[]).unwrap();
        // Geometry checking no longer grants or rejects per-footprint support.
        assert!(base.data.occupied_columns.get(&(5,0)).is_none(),"unknown footprint is not filled in the map");
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
        let map=SurfaceSnapshot::from_points(&points,config.clone(),0.05).unwrap().with_body_envelope(envelope.clone()).unwrap();
        let q=[0.0,0.0,0.0,1.0];
        map.validate_body_pose([0.55,0.2,0.45],q).unwrap();
        let path=map.plan([0.55,0.2,0.05],[5.45,0.2,0.05]).unwrap();
        for pair in path.windows(2) {
            map.validate_body_motion(([pair[0][0],pair[0][1],0.45],q),([pair[1][0],pair[1][1],0.45],q),&[]).unwrap();
        }
        let turn=[0.0,0.0,std::f64::consts::FRAC_1_SQRT_2,std::f64::consts::FRAC_1_SQRT_2];
        map.validate_body_motion(([1.0,0.2,0.45],q),([1.0,0.2,0.45],turn),&[]).unwrap();
        let mut walls=points;
        for x in 0..60 {for y in [-0.025,0.425] {for z in 0..12 {
            walls.push(((x as f32+0.5)*0.1,y,(z as f32+0.5)*0.05));
        }}}
        let enclosed=SurfaceSnapshot::from_points(&walls,config,0.05).unwrap().with_body_envelope(envelope).unwrap();
        enclosed.validate_body_pose([1.0,0.2,0.45],q).unwrap();
        assert!(enclosed.validate_body_motion(([1.0,0.2,0.45],q),([1.0,0.2,0.45],turn),&[]).is_err(),"actual side walls reject rotation");
    }

}

#[cfg(test)]
mod projection_equivalence {
    use super::*;
pub(super) fn reference_polygon_intersects_cell(polygon: &[[f64; 2]], x: i32, y: i32, size: f64) -> bool {
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
fn reference_prism_intersects_voxel(
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

    #[test]
    fn precomputed_projection_matches_vertex_reference_for_tilted_convex_bodies() {
        let model=SurfaceBodyEnvelope {
            footprint_xy_m:(0..24).map(|i| {let a=f64::from(i)*std::f64::consts::TAU/24.0;[0.3*a.cos(),0.16*a.sin()]}).collect(),
            z_bounds_m:[-0.4,0.06],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0,
        };
        let mut seed=7_u64;
        let mut random=|| {seed=seed.wrapping_mul(6364136223846793005).wrapping_add(1);(seed>>32) as f64/u32::MAX as f64*2.0-1.0};
        for _ in 0..80 {
            let q=[random(),random(),random(),random()];let r=rotation(q).unwrap();
            let origin=[random(),random(),random()];let query=PrismQuery::new(&model,r,origin);
            let polygon=hull(model.footprint_xy_m.iter().map(|p|{let w=world(r,origin,[p[0],p[1],0.0]);[w[0],w[1]]}).collect());
            let projected=PolygonQuery::new(&polygon);
            for _ in 0..80 {
                let min=[origin[0]+random()*0.5,origin[1]+random()*0.5,origin[2]+random()*0.5];
                let max=min.map(|v|v+0.05);
                assert_eq!(query.intersects(min,max),reference_prism_intersects_voxel(&model,r,origin,min,max));
                let x=(min[0]/0.05).floor() as i32;let y=(min[1]/0.05).floor() as i32;
                assert_eq!(projected.intersects(x,y,0.05),reference_polygon_intersects_cell(&polygon,x,y,0.05));
            }
        }
    }
}

// 缓存只属于同一不可变表面及当前足迹视图；输入保留浮点精确位，不扩大几何容差。
#[derive(Clone,Copy,PartialEq,Eq,Hash)]
enum ContactPlaneMode { StaticInterval, FixedMotionReference }

#[derive(Default)]
pub(super) struct BodyQueryCache {
    pub(super) supports: AHashMap<[u64;8],Result<SurfaceLocation,SurfaceError>>,
    measured_ground_patches: GroundPatchCache,
    lower_bounds: AHashMap<VoxelKey,f64>,
    planes: AHashMap<(VoxelKey,[u64;3],ContactPlaneMode),(f64,f64,f64)>,
    contact_samples: AHashMap<VoxelKey,Arc<Vec<([f64;3],f64)>>>,
    fine_bands: AHashMap<(i32,i32),Arc<Vec<FineContactBand>>>,
    complete_columns: AHashMap<(i32,i32),bool>,
    ground_estimates: AHashMap<([i32;2],u64),Option<GroundEstimate>>,
    poses: AHashMap<[u64;7],CachedBodyResult<(),BodyRejection>>,
    projections: AHashMap<[u64;6],Result<([f64;3],[f64;4]),SurfaceError>>,
    motions: AHashMap<[u64;14],CachedBodyResult<(),BodyFailureWitness>>,
}

#[cfg(test)]
#[path = "body_passage_tests.rs"]
mod passage_tests;

#[cfg(test)]
#[path = "contact_domain_tests.rs"]
mod contact_domain_tests;

#[cfg(test)]
#[path = "body_terrain_contract_tests.rs"]
mod terrain_contract_tests;

#[cfg(test)]
mod witness_cache_sizes {
    use super::*;
#[test]
fn body_witness_cache_value_sizes() {
    use std::mem::size_of;
    fn report<K,V>(name:&str,_:&AHashMap<K,V>) {
        eprintln!("CACHE_VALUE {name} key={} value={} entry={} max_values={}",size_of::<K>(),size_of::<V>(),size_of::<(K,V)>(),8192*size_of::<V>());
    }
    let c=BodyQueryCache::default();
    eprintln!("SURFACE_ERROR {}",size_of::<SurfaceError>());
    report("supports",&c.supports);
    report("poses",&c.poses);report("projections",&c.projections);report("motions",&c.motions);
}
}

#[cfg(test)]
#[path = "body_scalar_response_tests.rs"]
mod scalar_response_tests;
