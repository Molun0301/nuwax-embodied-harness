//! 导航与探索共用的封图根差分、精细观测和规划列更新。

use nuwax_map_chunks::{Manifest, Receiver};
use std::sync::Arc;

/// 按几何修订替换观测，返回相对兼容旧根的完整变化列；首帧及切域返回未知范围。
pub fn observations_with_changes(
    reader: &mut Receiver,
    root: &Manifest,
    previous: Option<(&Manifest, &Arc<crate::SurfaceObservations>)>,
) -> Result<(Arc<crate::SurfaceObservations>, Option<Vec<[i32; 2]>>), String> {
    use crate::{SurfaceObservation, SurfaceObservations};
    use std::collections::BTreeMap;
    let previous = previous.filter(|(old, _)| old.domain == root.domain
        && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits());
    if let Some((_, grid)) = previous.filter(|(old, _)| *old == root) {
        return Ok((Arc::clone(grid), Some(Vec::new())));
    }
    let known = previous.is_some();
    // 未变几何沿用原观测；证据字段不参与导航与探索的计算。
    let mut mutated = false;
    let mut changed = Vec::new();
    let mut grid = match previous {
        Some((_, grid)) => grid.as_ref().clone(),
        None => SurfaceObservations::new(root.voxel_size_m, 32).map_err(|e| e.to_string())?,
    };
    let old: BTreeMap<_, _> = previous.into_iter().flat_map(|(r, _)| &r.blocks).map(|b| (b.key, b)).collect();
    let new: BTreeMap<_, _> = root.blocks.iter().map(|b| (b.key, b)).collect();
    // 含类型化地面的根才需按竖直块组重建；变化对象只解码一次，零缓存时也复用。
    let typed = root.version >= 5 || previous.is_some_and(|(old,_)|old.version >= 5);
    let mut records_by_block = Vec::new();
    let removed: Vec<_> = old.keys().filter(|key| !new.contains_key(*key)).copied().collect();
    for key in &removed {
        mutated = true;
        changed.extend(grid.replace_block_with_geometry_changes(*key, BTreeMap::new())
            .map_err(|e| e.to_string())?);
        changed.extend(grid.replace_ground_layers(*key, Arc::new(BTreeMap::new()))
            .map_err(|e| e.to_string())?);
    }
    for reference in &root.blocks {
        if old.get(&reference.key).is_some_and(|old| old.geometry_revision == reference.geometry_revision) { continue; }
        let (block, records) = reader.ground_parts(reference)?;
        let mut values = BTreeMap::new();
        for (&local, evidence) in block.iter() {
            values.insert(nuwax_map_chunks::global_key(reference.key, local)?,
                SurfaceObservation { observed_ns: evidence.observed_ns, hits: evidence.hits });
        }
        mutated = true;
        changed.extend(grid.replace_block_with_ground_changes(reference.key, values, records.clone())
            .map_err(|e| e.to_string())?);
        if typed { records_by_block.push((reference.key, records)); }
    }
    if typed {
        if let Some((_, previous)) = previous {
            changed.extend(grid.replace_heightfield_blocks(previous, &records_by_block, &removed));
        } else {
            let mut groups: BTreeMap<_, Vec<_>> = BTreeMap::new();
            for (key, records) in records_by_block { groups.entry([key[0], key[1]]).or_default().push((key, records)); }
            for (xy, records) in groups { changed.extend(grid.replace_heightfield_columns(xy, &records)); }
        }
    }

    // 同一提交根覆盖完整几何源，缺少占据块不等于缺少已提交的几何源。
    grid.mark_complete_geometry_source();
    changed.sort_unstable();
    changed.dedup();
    let grid = match previous {
        Some((_, previous)) if !mutated && previous.is_complete_geometry_source() => Arc::clone(previous),
        _ => Arc::new(grid),
    };
    Ok((grid, known.then_some(changed)))
}

/// 使用已经更新的观测列，不为规划列再次读取块；原输入点统计取变化几何块清单。
pub fn column_updates(
    root: &Manifest,
    previous: Option<&Manifest>,
    planner_size: f32,
    observations: &crate::SurfaceObservations,
    changes: Option<&[[i32; 2]]>,
) -> (Vec<crate::SurfaceColumnUpdate>, usize) {
    use std::collections::BTreeMap;
    let previous = previous.filter(|old|
        old.domain == root.domain && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits());
    let old: BTreeMap<_, _> = previous.into_iter().flat_map(|root| &root.blocks).map(|b| (b.key, b)).collect();
    let input_points = root.blocks.iter().filter(|b|
        old.get(&b.key).is_none_or(|old| old.geometry_revision != b.geometry_revision))
        .map(|b| b.points as usize).sum();
    (observations.planner_column_updates(changes, planner_size), input_points)
}
