//! 台阶后转角的请求内参考候选；是否可执行仍由原投影和完整扫掠决定。
use super::{LocalPlanError, ScanConfig, SurfaceSnapshot, Tomogram};

pub(super) fn guide(map: &SurfaceSnapshot, tomogram: &Tomogram, reference: &[[f64; 3]],
    config: &ScanConfig, arrival: bool, boundary: &[[f64; 2]]) -> Result<Option<Vec<[f64; 3]>>, LocalPlanError> {
    let model = map.body_envelope().ok_or("台阶转角候选缺少机体尺寸")?;
    let radius = model.radius();
    let quantum = f64::from(map.source_voxel_size_m());
    let slope = f64::from(map.config().max_slope_deg).to_radians().tan();
    for i in 1..reference.len().saturating_sub(1) {
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
        let Ok(entry) = tomogram.project_reference_motion(a, entry_xy, boundary) else { continue; };
        let Ok(landing) = tomogram.project_reference_motion(entry, landing_xy, boundary) else { continue; };
        let mut guide = reference[..i.saturating_sub(1).max(1)].to_vec();
        guide.extend([entry, landing]);
        if !arrival && i+2 == reference.len() {
            // 窗口恰好截在阶边时沿原出向穿过同一交接截面；终端准入另外复核。
            let remaining = ((c[0]-landing[0])*outgoing[0]+(c[1]-landing[1])*outgoing[1]).max(0.0);
            let end_xy = [landing[0]+outgoing[0]*(remaining+1e-6),
                landing[1]+outgoing[1]*(remaining+1e-6)];
            let Ok(end) = tomogram.project_reference_motion(landing, end_xy, boundary) else { continue; };
            guide.push(end);
        } else {
            guide.extend_from_slice(&reference[i+1..]);
        }
        let corridor = (config.reference_corridor_m*0.9).min(config.horizon_m*0.3);
        let inside = guide.iter().all(|p| reference.windows(2).any(|w| {
            let dx=w[1][0]-w[0][0]; let dy=w[1][1]-w[0][1]; let length=dx*dx+dy*dy;
            let t=if length>0.0 {(((p[0]-w[0][0])*dx+(p[1]-w[0][1])*dy)/length).clamp(0.0,1.0)} else {0.0};
            (p[0]-w[0][0]-t*dx).hypot(p[1]-w[0][1]-t*dy)<=corridor
        }));
        if inside { return Ok(Some(guide)); }
    }
    super::cancellation::checkpoint()?;
    Ok(None)
}
