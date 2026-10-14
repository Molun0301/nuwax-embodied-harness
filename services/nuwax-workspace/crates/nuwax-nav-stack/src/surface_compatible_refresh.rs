//! A deliberately narrow no-replan proof for added ground on a flat reference.
//! No collision checks, terrain fitting, map scan or replacement route is needed.
use super::*;

pub(super) fn adds_only_flat_ground(
    previous: &SurfaceSnapshot,
    current: &SurfaceSnapshot,
    path: &Path3D,
    fine_size: f32,
    fine: &[[i32; 2]],
    planner_size: f32,
    coarse: &[[i32; 2]],
    mut remaining: usize,
) -> bool {
    let Some(model) = current.body_envelope() else {
        return false;
    };
    let Some((old, new)) = previous.observations().zip(current.observations()) else {
        return false;
    };
    if fine.is_empty()
        || !old.is_complete_geometry_source()
        || !new.is_complete_geometry_source()
        || previous.body_envelope() != current.body_envelope()
        || fine_size.to_bits() != current.source_voxel_size_m().to_bits()
        || fine_size.to_bits() != previous.source_voxel_size_m().to_bits()
        || !fine_size.is_finite()
        || fine_size <= 0.0
        || !planner_size.is_finite()
        || planner_size <= 0.0
    {
        return false;
    }
    // Count repeated indexed-column/member work too. Excess remains the original
    // conservative refresh, without growing the owner's advisory work limit.
    let mut spend = |count: usize| {
        if count > remaining {
            false
        } else {
            remaining -= count;
            true
        }
    };
    let size = f64::from(fine_size);
    // The actual initial body can pitch/roll independently of horizontal ground.
    // Only the subsequent yaw-free PCT guide defines this flat-reference proof.
    if path.poses.len() < 2 {
        return false;
    }
    let body_z = path.poses[1].pose.position.z;
    let support_z = body_z - model.body_to_support_z_m;
    if !support_z.is_finite() || (support_z / size).abs() >= f64::from(i32::MAX) * 0.5 {
        return false;
    }
    let mut reference_level = None;
    for (index, pose) in path.poses.iter().enumerate() {
        let q = pose.pose.orientation;
        if !spend(1)
            || (index > 0 && (q.x != 0.0 || q.y != 0.0))
            || (index > 0 && pose.pose.position.z != body_z)
            || (index == 0 && (pose.pose.position.z - body_z).abs() > size + 1e-6)
        {
            return false;
        }
        let column =
            [pose.pose.position.x, pose.pose.position.y].map(|v| (v / size).floor() as i32);
        if let Some(levels) = old.column_levels(column[0], column[1]) {
            if !spend(levels.len()) || levels.len() != 1 {
                return false;
            }
            let z = levels[0];
            // Existing quantized source compatibility, not a step-height allowance.
            if ((f64::from(z) + 1.0) * size - support_z).abs() > size + 1e-6 {
                return false;
            }
            if reference_level.is_some_and(|level| level != z) {
                return false;
            }
            reference_level = Some(z);
        }
    }
    // A wholly unknown flat route has only its carried contact coordinate. A
    // different height remains a real geometry change and requests refinement.
    let level = reference_level.unwrap_or_else(|| (support_z / size).round() as i32 - 1);
    if reference_level.is_none() && ((f64::from(level) + 1.0) * size - support_z).abs() > 1e-6 {
        return false;
    }
    for column in fine {
        let Some(levels) = new.column_levels(column[0], column[1]) else {
            return false;
        };
        if !spend(levels.len()) || levels != [level] {
            return false;
        }
        let old_levels = old.column_levels(column[0], column[1]);
        if old_levels.is_some_and(|levels| levels != [level]) {
            return false;
        }
        // An already classified plane changing, disappearing, splitting or gaining
        // members is not the unknown-to-known case and cannot use this proof.
        if old.ground_layers(*column).next().is_some() {
            return false;
        }
        let mut layers = new.ground_layers(*column);
        if let Some(layer) = layers.next() {
            if !spend(layer.members.len())
                || layer.members != [level]
                || layers.next().is_some()
                || layer.normal != [0.0, 0.0, 1.0]
            {
                return false;
            }
            let Some(z) = layer.height_at(column.map(|v| (f64::from(v) + 0.5) * size)) else {
                return false;
            };
            if z < f64::from(level) * size - 1e-6 || z > (f64::from(level) + 1.0) * size + 1e-6 {
                return false;
            }
        } else if old_levels.is_some() {
            // Unchanged occupancy without newly classified ground is not a proved
            // new observation; keep malformed/incomplete delta reports conservative.
            return false;
        }
    }
    // Complete fine geometry owns the coarse derivation. Require every changed
    // coarse column to be represented by those proved compatible fine columns.
    for column in coarse {
        let mut found = false;
        for fine_column in fine {
            if !spend(1) {
                return false;
            }
            let mapped = fine_column
                .map(|v| (((f64::from(v) + 0.5) * size) / f64::from(planner_size)).floor() as i32);
            if mapped == *column {
                found = true;
                break;
            }
        }
        if !found {
            return false;
        }
    }
    true
}
