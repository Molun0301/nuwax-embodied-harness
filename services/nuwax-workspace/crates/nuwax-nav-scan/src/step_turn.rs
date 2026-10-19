//! 台阶后转角的请求内参考候选；是否可执行仍由原投影和完整扫掠决定。
use super::{BodyGroundState, BodyPose, LocalPlanError, ScanConfig, SurfaceBodyQueryView, SurfaceSnapshot};

pub(super) fn guide(map:&SurfaceSnapshot,query:&SurfaceBodyQueryView,start_ground:&BodyGroundState,
    start:BodyPose,reference:&[[f64;3]],config:&ScanConfig,arrival:bool)->Result<Option<Vec<[f64;3]>>,LocalPlanError> {
    guide_state(map,query,&super::MotionState::body(start_ground.clone()),start,reference,config,arrival,&[])
}
pub(super) fn guide_state<const N:usize>(map: &SurfaceSnapshot, query: &SurfaceBodyQueryView,
    start_ground: &super::MotionState<'_>, start: BodyPose, reference: &[[f64; N]],
    config: &ScanConfig, arrival: bool,boundary:&[[f64;2]]) -> Result<Option<Vec<[f64; N]>>, LocalPlanError> {
    let model = map.body_envelope().ok_or("台阶转角候选缺少机体尺寸")?;
    let radius = model.radius();
    let quantum = f64::from(map.source_voxel_size_m());
    let slope = f64::from(map.config().max_slope_deg).to_radians().tan();
    // Proposal heights come from the independent navigation walk. Unknown
    // remains None; an absent step proposal never rejects the planning request.
    let heights=if let Some(mut navigation)=start_ground.navigation {
        let mut heights=Vec::with_capacity(reference.len());
        for point in reference {
            super::cancellation::checkpoint()?;
            let result=navigation.walk_xy([point[0],point[1]],boundary,None).map_err(|e|LocalPlanError::from(e.to_string()));
            let Some(segment)=super::admit_candidate(result)? else {return Ok(None);};
            navigation=segment.end(); heights.push(navigation.planned_ground_z());
        }
        Some(heights)
    } else {None};
    'candidate: for i in 1..reference.len().saturating_sub(1) {
        super::cancellation::checkpoint()?;
        let (a, b, c) = (reference[i-1], reference[i], reference[i+1]);
        let incoming = [b[0]-a[0], b[1]-a[1]];
        let outgoing = [c[0]-b[0], c[1]-b[1]];
        let before = incoming[0].hypot(incoming[1]);
        let after = outgoing[0].hypot(outgoing[1]);
        // 平滑坡面和源量化的单格起伏不触发；这里只尝试紧接转弯的参考高差。
        let height_delta=if let Some(heights)=&heights {
            let (Some(a),Some(b))=(heights[i-1],heights[i]) else {continue;}; b-a
        } else {b[2]-a[2]};
        if before <= 1e-9 || after <= 1e-9 || height_delta.abs() <= before*slope+quantum+1e-6 {
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
        if start_ground.navigation.is_some() {
            // Genuine XY proposal. The selected physical candidate below walks
            // these actual subedges afresh and performs the full H3 sweep.
            return Ok(Some(guide_xy.into_iter().map(|xy|std::array::from_fn(|axis|xy[axis])).collect()));
        }
        let mut pose=start;
        let mut ground=start_ground.physical.clone();
        let mut guide=Vec::with_capacity(guide_xy.len());
        for xy in guide_xy {
            super::cancellation::checkpoint()?;
            let dx=xy[0]-pose.0[0];let dy=xy[1]-pose.0[1];
            let heading=if dx.hypot(dy)>1e-9 {dy.atan2(dx)} else {super::yaw(pose.1)};
            let Some(projected)=super::admit_candidate(super::project_endpoint(query,&ground,pose,xy,heading))?
                else { continue 'candidate; };
            pose=projected.pose();ground=projected.ground().clone();
            guide.push(std::array::from_fn(|axis|if axis<2 {xy[axis]} else {ground.height()}));
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
