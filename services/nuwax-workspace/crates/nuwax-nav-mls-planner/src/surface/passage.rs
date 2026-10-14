//! 从局部真实细占据提出连续通道中线；候选不会获得任何机体运动许可。
use super::*;

/// 穿越方向附近的几何搜索提示，所有点的Z仅复制查询参考高度。
/// 调用方须重新生成真实地形站姿，并复核接近、通过及离开的完整运动。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceBodyPassageCandidate {
    /// 局部障碍纵向跨度中点与横向可站立区间的中点。
    pub center_xyz: [f64; 3],
    /// 机体外包半径已完全退到局部障碍前方的参考点。
    pub entry_xyz: [f64; 3],
    /// 机体外包半径已完全越过局部障碍后方的参考点。
    pub exit_xyz: [f64; 3],
    /// 查询提供的穿越朝向，弧度；不表示已识别门框法向。
    pub heading: f64,
    /// 局部占据在横向投影后两侧障碍间的净宽，并非运动许可。
    pub clear_width_m: f64,
}

impl SurfaceSnapshot {
    /// 在给定朝向的局部矩形内寻找被真实占据夹住的横向空隙。
    /// `half_extent_xy_m`依次为纵向、横向查询半径，最多查询65536个细列。
    /// 仅投影与参考机体竖向范围相交的真实占据；倾斜地形和非轴向投影可能漏候选。
    /// 无细观察、无足迹或无夹持空隙时返回空集；空集/预算错误不能解释为无路径。
    /// 本查询不证明支撑、观察质量、虚拟边界或任何运动可行性。
    pub fn body_passage_candidates(
        &self, body_xyz: [f64; 3], heading: f64, half_extent_xy_m: [f64; 2],
    ) -> Result<Vec<SurfaceBodyPassageCandidate>, SurfaceError> {
        if !body_xyz.iter().chain(half_extent_xy_m.iter()).all(|v|v.is_finite())
            || !heading.is_finite() || half_extent_xy_m.iter().any(|v|*v<=0.0) {
            return Err(SurfaceError::InvalidPose);
        }
        let (Some(observations),Some(envelope))=(&self.observations,self.body_envelope()) else {return Ok(vec![]);};
        let fine=f64::from(observations.size_m);
        if observations.size_m.to_bits()!=self.source_voxel_size_m().to_bits() {
            return Err(SurfaceError::InvalidConfig("通道候选精细观察分辨率与几何源不一致".into()));
        }
        let (sin,cos)=heading.sin_cos();let longitudinal=[cos,sin];let lateral=[-sin,cos];
        let hx=cos.abs()*half_extent_xy_m[0]+sin.abs()*half_extent_xy_m[1];
        let hy=sin.abs()*half_extent_xy_m[0]+cos.abs()*half_extent_xy_m[1];
        let bounds=[((body_xyz[0]-hx)/fine).floor(),((body_xyz[0]+hx)/fine).ceil(),
            ((body_xyz[1]-hy)/fine).floor(),((body_xyz[1]+hy)/fine).ceil()];
        if bounds.iter().any(|v|*v<i32::MIN as f64||*v>i32::MAX as f64)
            || (bounds[1]-bounds[0])*(bounds[3]-bounds[2])>65_536.0 {
            return Err(SurfaceError::InvalidConfig("局部通道候选查询超过65536细列预算".into()));
        }
        // 每个细格在任意查询朝向的精确投影半径；合并投影会保守缩小斜门候选。
        let projection_radius=fine*(sin.abs()+cos.abs())*0.5;
        let lower_z=body_xyz[2]+envelope.z_bounds_m[0];
        let upper_z=body_xyz[2]+envelope.z_bounds_m[1];
        let mut obstacles=Vec::<([f64;2],[f64;2])>::new();
        for ix in bounds[0] as i32..bounds[1] as i32 {
            for iy in bounds[2] as i32..bounds[3] as i32 {
                let Some(levels)=observations.column_levels(ix,iy) else {continue;};
                let first=levels.partition_point(|iz|(f64::from(*iz)+1.0)*fine<lower_z);
                if levels.get(first).is_none_or(|iz|f64::from(*iz)*fine>upper_z) {continue;}
                let delta=[(f64::from(ix)+0.5)*fine-body_xyz[0],(f64::from(iy)+0.5)*fine-body_xyz[1]];
                let x=delta[0]*longitudinal[0]+delta[1]*longitudinal[1];
                let y=delta[0]*lateral[0]+delta[1]*lateral[1];
                if x-projection_radius>half_extent_xy_m[0]||x+projection_radius< -half_extent_xy_m[0]
                    ||y-projection_radius>half_extent_xy_m[1]||y+projection_radius< -half_extent_xy_m[1] {continue;}
                obstacles.push(([y-projection_radius,y+projection_radius],[x-projection_radius,x+projection_radius]));
            }
        }
        obstacles.sort_by(|a,b|a.0[0].total_cmp(&b.0[0]));
        let mut bands=Vec::<([f64;2],[f64;2])>::new();
        for (y,x) in obstacles {
            if let Some(last)=bands.last_mut().filter(|last|y[0]<=last.0[1]+1e-10) {
                last.0[1]=last.0[1].max(y[1]);last.1[0]=last.1[0].min(x[0]);last.1[1]=last.1[1].max(x[1]);
            } else {bands.push((y,x));}
        }
        let min_y=envelope.footprint_xy_m.iter().map(|p|p[1]).fold(f64::INFINITY,f64::min);
        let max_y=envelope.footprint_xy_m.iter().map(|p|p[1]).fold(f64::NEG_INFINITY,f64::max);
        let reserve=envelope.radius()+fine;
        let point=|x:f64,y:f64|[body_xyz[0]+x*cos-y*sin,body_xyz[1]+x*sin+y*cos,body_xyz[2]];
        let mut candidates=Vec::new();
        for pair in bands.windows(2) {
            let (left,right)=(pair[0],pair[1]);let width=right.0[0]-left.0[1];
            let low=left.0[1]-min_y;let high=right.0[0]-max_y;
            if high-low<=1e-9 {continue;}
            let y=(low+high)*0.5;
            if y.abs()>half_extent_xy_m[1] {continue;}
            let x0=left.1[0].min(right.1[0]);let x1=left.1[1].max(right.1[1]);
            candidates.push(SurfaceBodyPassageCandidate {center_xyz:point((x0+x1)*0.5,y),
                entry_xyz:point(x0-reserve,y),exit_xyz:point(x1+reserve,y),heading,clear_width_m:width});
        }
        candidates.sort_by(|a,b|{
            let distance=|p:&SurfaceBodyPassageCandidate|(p.center_xyz[0]-body_xyz[0]).hypot(p.center_xyz[1]-body_xyz[1]);
            distance(a).total_cmp(&distance(b))
        });
        Ok(candidates)
    }
}
