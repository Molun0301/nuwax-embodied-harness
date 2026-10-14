// Frozen before the copy-only trial on 2026-10-06. Do not route this through
// production compose: both the virtual caller and borrowed wrapper compare to it.
// Raster primitives intentionally remain shared; this trial does not change them.
use super::*;

pub(crate) fn compose_effective_map(
    raw: &MapState,
    loaded: &LoadedForbiddenZones,
) -> Result<MapState, String> {
    let expected = raw.width as usize * raw.height as usize;
    if raw.width == 0
        || raw.height == 0
        || !raw.resolution.is_finite()
        || raw.resolution <= 0.0
        || raw.data.len() != expected
    {
        return Err("raw occupancy layout is invalid".to_string());
    }
    let mut effective = raw.clone();
    if matches!(loaded.status, ZoneLoadStatus::FailClosed(_)) {
        effective.data.fill(100);
        return Ok(effective);
    }
    let enabled: Vec<&ForbiddenZone> = loaded.zones.iter().filter(|zone| zone.enabled).collect();
    if enabled.is_empty() {
        return Ok(effective);
    }

    let cell_count = effective.data.len();
    let mut forbidden = vec![false; cell_count];
    let outside_zones: Vec<_> = enabled
        .iter()
        .copied()
        .filter(|zone| zone.mode == ZoneMode::Outside)
        .collect();
    if !outside_zones.is_empty() {
        for row in 0..raw.height {
            for col in 0..raw.width {
                let rect = cell_rect(raw, col, row);
                let allowed = outside_zones
                    .iter()
                    .all(|zone| cell_fully_inside_closed_shape(rect, zone, raw.resolution as f64));
                if !allowed {
                    forbidden[cell_index(raw, col, row)] = true;
                }
            }
        }
    }

    for zone in enabled {
        if zone.mode == ZoneMode::Outside {
            continue;
        }
        for row in 0..raw.height {
            for col in 0..raw.width {
                let rect = cell_rect(raw, col, row);
                if cell_forbidden_by_inside_zone(rect, zone, raw.resolution as f64) {
                    forbidden[cell_index(raw, col, row)] = true;
                }
            }
        }
    }

    for (index, is_forbidden) in forbidden.into_iter().enumerate() {
        if is_forbidden {
            effective.data[index] = 100;
        }
    }
    Ok(effective)
}
