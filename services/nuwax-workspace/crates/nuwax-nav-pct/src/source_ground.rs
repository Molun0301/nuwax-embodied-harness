//! 借用正式源地表记录；PCT 只保存本列固定参考和原源内的小键。
use crate::{Tomogram, tomogram::GroundCandidate};
use nuwax_nav_mls_planner::{SurfaceSnapshot, surface::{GroundEvidence, SourceGroundRecord}};
use nuwax_types::ground_layer::{GroundApproximation, GroundLayer};

/// None 保留旧占据带回退；Some 即使为空也不由 raw 占据再造地面。
pub(crate) fn candidates(source: &SurfaceSnapshot, column: [i32;2], resolution: f64,
    bands: &[[f64;2]]) -> Option<Vec<GroundCandidate>> {
    if source.ground_evidence()!=GroundEvidence::MeasuredSurface { return None; }
    // 虚拟边界已清掉本列占据；元数据不能将其恢复成可导航地面。
    if bands.is_empty() { return Some(Vec::new()); }
    let observations=source.observations()?;
    let quantum=f64::from(observations.voxel_size_m());
    if !observations.is_complete_geometry_source() || resolution.to_bits()!=quantum.to_bits()
        || quantum.to_bits()!=f64::from(source.source_voxel_size_m()).to_bits() {
        return None;
    }
    let mut records=observations.ground_records(column).peekable();
    if records.peek().is_none() {
        return observations.classified_empty_ground(column).then(Vec::new);
    }
    let xy=column.map(|v|(f64::from(v)+0.5)*quantum);
    let mut native=false;
    let mut result=Vec::new();
    for record in records {
        native|=record.is_heightfield();
        let Some(height)=record.height_at(xy) else { continue; };
        // 只排除当前已选地表的真实成员。bounds 中的洞和其它回波仍为
        // 实际占据；这个参考 ceiling 不是对机体接触的碰撞豁免。
        let ceiling=observations.column_levels(column[0],column[1]).unwrap_or(&[]).iter()
            .filter(|&&z|!record.contains_member([column[0],column[1],z]))
            .map(|&z|f64::from(z)*quantum)
            .filter(|&bottom|bottom+quantum>height+1e-9)
            .min_by(f64::total_cmp).unwrap_or(f64::INFINITY);
        result.push(GroundCandidate { height, bounds:record.occupied_bounds(quantum), ceiling,
            approximation:if record.is_heightfield() {GroundApproximation::ObservedHeightfield}
                else {GroundApproximation::ObservedLayer},
            source_ground:Some(record.key()) });
    }
    // 原生地表列中的无记录回波（例如低横杆）仅保留为实体障碍。
    // 无原生地表时维持旧 LegacyPlane 对独立占据带的兼容回退。
    if !native {
        for (i,&band) in bands.iter().enumerate() {
            let contains_measured=observations.ground_records(column).any(|record|
                record.members().any(|z| {
                    let center=(f64::from(z)+0.5)*quantum;
                    center>=band[0] && center<=band[1]
                }));
            if !contains_measured {
                if let Some(layer)=GroundLayer::from_legacy_band(band,quantum,source.ground_evidence()) {
                    result.push(GroundCandidate::legacy(layer,
                        bands.get(i+1).map_or(f64::INFINITY,|next|next[0])));
                }
            }
        }
    }
    result.sort_by(|a,b|a.height.total_cmp(&b.height));
    Some(result)
}

impl Tomogram {
    /// 借用单元已选的原始记录，不按当前机体高度重新选择地表。
    pub(crate) fn source_ground_record(&self, id: usize) -> Option<SourceGroundRecord<'_>> {
        let cell=self.cells.get(id)?;
        let key=cell.source_ground?;
        let anchor=key.anchor();
        if [anchor[0],anchor[1]]!=cell.column { return None; }
        self.source.as_ref()?.observations()?.ground_record(key)
    }
}
