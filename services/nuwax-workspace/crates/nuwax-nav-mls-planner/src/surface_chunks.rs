//! 导航与探索共用的封图根差分、精细观测和规划列更新。

use nuwax_map_chunks::{Manifest, Receiver};
use std::sync::Arc;

/// 按块修订替换观测，返回相对兼容旧根的完整变化列；首帧及切域返回未知范围。
pub fn observations_with_changes(
    reader: &mut Receiver,
    root: &Manifest,
    previous: Option<(&Manifest, &Arc<crate::SurfaceObservations>)>,
) -> Result<(Arc<crate::SurfaceObservations>, Option<Vec<[i32; 2]>>), String> {
    use crate::{SurfaceObservation, SurfaceObservations};
    use std::collections::{BTreeMap, BTreeSet};
    let previous = previous.filter(|(old, _)| old.domain == root.domain
        && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits());
    if let Some((_, grid)) = previous.filter(|(old, _)| *old == root) {
        return Ok((Arc::clone(grid), Some(Vec::new())));
    }
    let known = previous.is_some();
    // 几何未变不代表观测证据和分层统计未变。
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
    let mut native_groups = BTreeSet::new();
    let mut records_by_block = BTreeMap::new();
    for key in old.keys().filter(|key| !new.contains_key(*key)) {
        mutated = true;
        if typed { native_groups.insert([key[0],key[1]]); }
        changed.extend(grid.replace_block_with_geometry_changes(*key, BTreeMap::new())
            .map_err(|e| e.to_string())?);
        // 最后一个占据块删除时，也移除对应实测地面层。
        changed.extend(grid.replace_ground_layers(*key, Arc::new(BTreeMap::new()))
            .map_err(|e| e.to_string())?);
    }
    for reference in &root.blocks {
        if old.get(&reference.key).is_some_and(|old| old.block_revision == reference.block_revision) { continue; }
        let (block, records) = reader.ground_parts(reference)?;
        let mut values = BTreeMap::new();
        for (&local, evidence) in block.iter() {
            if evidence.observed_ns > root.source_timestamp_ns { return Err("精细观察晚于提交根".into()); }
            values.insert(nuwax_map_chunks::global_key(reference.key, local)?,
                SurfaceObservation { observed_ns: evidence.observed_ns, hits: evidence.hits });
        }
        mutated = true;
        changed.extend(grid.replace_block_with_geometry_changes(reference.key, values)
            .map_err(|e| e.to_string())?);
        if typed {
            native_groups.insert([reference.key[0],reference.key[1]]);
            records_by_block.insert(reference.key,records);
        }
    }
    for xy in native_groups {
        let mut group = Vec::new();
        for (&key, reference) in new.range([xy[0],xy[1],i32::MIN]..=[xy[0],xy[1],i32::MAX]) {
            let records = match records_by_block.remove(&key) {
                Some(records) => records,
                None => reader.ground_records(reference)?,
            };
            group.push((key,records));
        }
        changed.extend(grid.replace_heightfield_columns(xy,&group));
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

/// 只展开几何变化块涉及的完整规划列；上下叠层和跨块列共同聚合。
pub fn column_updates(
    reader: &mut Receiver,
    root: &Manifest,
    previous: Option<&Manifest>,
    planner_size: f32,
) -> Result<(Vec<crate::SurfaceColumnUpdate>, usize), String> {
    use std::collections::{BTreeMap, BTreeSet};
    if !planner_size.is_finite() || planner_size < root.voxel_size_m {
        return Err("规划精度小于精细源精度".into());
    }
    let compatible = previous.filter(|old| {
        old.domain == root.domain && old.voxel_size_m.to_bits() == root.voxel_size_m.to_bits()
    });
    let old: BTreeMap<_, _> = compatible
        .into_iter()
        .flat_map(|r| &r.blocks)
        .map(|b| (b.key, b))
        .collect();
    let new: BTreeMap<_, _> = root.blocks.iter().map(|b| (b.key, b)).collect();
    let mut dirty = BTreeSet::new();
    for key in old.keys().chain(new.keys()) {
        if old.get(key).map(|b| b.geometry_revision) == new.get(key).map(|b| b.geometry_revision) {
            continue;
        }
        let (min, max) = planning_columns(*key, root.voxel_size_m, planner_size)?;
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                dirty.insert([x, y]);
            }
        }
        if dirty.len() > 2_000_000 {
            return Err("脏列超过规划工作集预算".into());
        }
    }
    let mut columns: BTreeMap<[i32; 2], BTreeMap<i32, [f32; 2]>> = BTreeMap::new();
    let mut read_points = 0;
    for reference in &root.blocks {
        let (min, max) = planning_columns(reference.key, root.voxel_size_m, planner_size)?;
        if !dirty
            .range([min[0], i32::MIN]..=[max[0], i32::MAX])
            .any(|c| c[1] >= min[1] && c[1] <= max[1])
        {
            continue;
        }
        let block = reader.block(reference)?;
        read_points += block.len();
        for (&local, evidence) in block.iter() {
            if evidence.observed_ns > root.source_timestamp_ns {
                return Err("精细证据晚于提交根".into());
            }
            let key = nuwax_map_chunks::global_key(reference.key, local)?;
            let xyz = key.map(|v| ((f64::from(v) + 0.5) * f64::from(root.voxel_size_m)) as f32);
            let cell = xyz.map(|v| (v / planner_size).floor() as i32);
            let column = [cell[0], cell[1]];
            if !dirty.contains(&column) {
                continue;
            }
            // 与完整点输入使用相同的精细高度量化，避免浮点边界出现两套语义。
            let fine = (f64::from(xyz[2]) / f64::from(root.voxel_size_m)).floor();
            let low = (fine * f64::from(root.voxel_size_m)) as f32;
            let high = ((fine + 1.0) * f64::from(root.voxel_size_m)) as f32;
            columns
                .entry(column)
                .or_default()
                .entry(cell[2])
                .and_modify(|b| {
                    b[0] = b[0].min(low);
                    b[1] = b[1].max(high);
                })
                .or_insert([low, high]);
        }
    }
    let updates = dirty
        .into_iter()
        .map(|column| crate::SurfaceColumnUpdate {
            column,
            cells: columns
                .remove(&column)
                .unwrap_or_default()
                .into_iter()
                .collect(),
        })
        .collect();
    Ok((updates, read_points))
}

fn planning_columns(
    key: [i32; 3],
    source: f32,
    planner: f32,
) -> Result<([i32; 2], [i32; 2]), String> {
    let first = nuwax_map_chunks::global_key(key, 0)?;
    let last = nuwax_map_chunks::global_key(key, 32767)?;
    let convert = |v| {
        let point = ((f64::from(v) + 0.5) * f64::from(source)) as f32;
        let column = (point / planner).floor();
        if !column.is_finite() || column.abs() >= i32::MAX as f32 * 0.5 {
            return Err("精细块规划坐标越界".to_owned());
        }
        Ok(column as i32)
    };
    Ok((
        [convert(first[0])?, convert(first[1])?],
        [convert(last[0])?, convert(last[1])?],
    ))
}
