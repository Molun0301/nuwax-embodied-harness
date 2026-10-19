//! 台阶后转角的请求内参考候选；是否可执行仍由原投影和完整扫掠决定。
use super::{BodyGroundState, BodyPose, LocalPlanError, ScanConfig, SurfaceBodyQueryView, SurfaceSnapshot};

pub(super) fn guide(map: &SurfaceSnapshot, query: &SurfaceBodyQueryView,
    start_ground: &BodyGroundState, start: BodyPose, reference: &[[f64; 3]],
    config: &ScanConfig, arrival: bool) -> Result<Option<Vec<[f64; 3]>>, LocalPlanError> {
    let model = map.body_envelope().ok_or("台阶转角候选缺少机体尺寸")?;
    let radius = model.radius();
    let quantum = f64::from(map.source_voxel_size_m());
    let slope = f64::from(map.config().max_slope_deg).to_radians().tan();
    'candidate: for i in 1..reference.len().saturating_sub(1) {
        super::cancellation::checkpoint()?;
        let (a, b, c) = (reference[i-1], reference[i], reference[i+1]);
        let incoming = [b[0]-a[0], b[1]-a[1]];
        let outgoing = [c[0]-b[0], c[1]-b[1]];
        let before = incoming[0].hypot(incoming[1]);
        let after = outgoing[0].hypot(outgoing[1]);
        // 平滑坡面和源量化的单格起伏不触发；这里只尝试紧接转弯的参考高差。
        if before <= 1e-9 || after <= 1e-9 || (b[2]-a[2]).abs() <= before*slope+quantum+1e-6 {
            continue;
        }
        let incoming = incoming.map(|v|v/before);
        let outgoing = outgoing.map(|v|v/after);
        let turn = (incoming[0]*outgoing[1]-incoming[1]*outgoing[0])
            .atan2(incoming[0]*outgoing[0]+incoming[1]*outgoing[1]).abs();
        if turn <= 0.03 || turn >= std::f64::consts::PI-0.03 { continue; }
        let entry_xy = [a[0]-outgoing[0]*radius, a[1]-outgoing[1]*radius];
        let landing_xy = [b[0]-outgoing[0]*radius+incoming[0]*radius,
            b[1]-outgoing[1]*radius+incoming[1]*radius];
        let mut guide_xy: Vec<_> = reference[..i.saturating_sub(1).max(1)].iter()
            .map(|p|[p[0],p[1]]).collect();
        guide_xy.extend([entry_xy, landing_xy]);
        if !arrival && i+2 == reference.len() {
            // 窗口恰好截在阶边时沿原出向穿过同一交接截面；终端准入另外复核。
            let remaining = ((c[0]-landing_xy[0])*outgoing[0]+(c[1]-landing_xy[1])*outgoing[1]).max(0.0);
            let end_xy = [landing_xy[0]+outgoing[0]*(remaining+1e-6),
                landing_xy[1]+outgoing[1]*(remaining+1e-6)];
            guide_xy.push(end_xy);
        } else {
            guide_xy.extend(reference[i+1..].iter().map(|p|[p[0],p[1]]));
        }
        let corridor = (config.reference_corridor_m*0.9).min(config.horizon_m*0.3);
        let inside = guide_xy.iter().all(|p| reference.windows(2).any(|w| {
            let dx=w[1][0]-w[0][0]; let dy=w[1][1]-w[0][1]; let length=dx*dx+dy*dy;
            let t=if length>0.0 {(((p[0]-w[0][0])*dx+(p[1]-w[0][1])*dy)/length).clamp(0.0,1.0)} else {0.0};
            (p[0]-w[0][0]-t*dx).hypot(p[1]-w[0][1]-t*dy)<=corridor
        }));
        if !inside { continue; }
        // XY仍是原候选；从真实起点推进同一拥有方的分支状态，不能由参考Z重种站姿。
        // 拥有方以完整姿态转换的实际支撑XY选列；未知只携带先前h。
        let mut pose=start;
        let mut ground=start_ground.clone();
        let mut guide=Vec::with_capacity(guide_xy.len());
        for xy in guide_xy {
            super::cancellation::checkpoint()?;
            let dx=xy[0]-pose.0[0];let dy=xy[1]-pose.0[1];
            let heading=if dx.hypot(dy)>1e-9 {dy.atan2(dx)} else {super::yaw(pose.1)};
            let Some(projected)=super::admit_candidate(super::project_endpoint(query,&ground,pose,xy,heading))?
                else { continue 'candidate; };
            pose=projected.pose();ground=projected.ground().clone();
            guide.push([xy[0],xy[1],ground.height()]);
        }
        // 这里只取得候选高度；完整project_bound仍重新投影并逐段扫掠/检查任务边界。
        return Ok(Some(guide));
    }
    super::cancellation::checkpoint()?;
    Ok(None)
}

#[cfg(test)]
#[path = "step_turn_native_tests.rs"]
mod native_tests;
