//! 缓存等价测试的原全量重建对照。
use super::*;

pub(super) fn full_tomography_input(
    map: &SurfaceSnapshot,
    body_to_ground: f64,
    boundary: &[[f64; 2]],
) -> Result<(Config, Vec<Cell>, Vec<Band>), String> {
    let (resolution, columns) = map.navigation_source_columns(None);
    let config = pct_config(map, body_to_ground, boundary, resolution);
    let mut points = Vec::new();
    // 完整精细源带独立地面记录时，地面取记录的连续高度，成员体素不再作为占据采样。
    // 体素中心按源精度量化，缓坡或略倾斜的地图会被切成逐格台阶，两层体素厚的地面也会挡住机体。
    let ground = map.observations().filter(|source| {
        map.ground_evidence() == nuwax_nav_mls_planner::surface::GroundEvidence::MeasuredSurface
            && source.is_complete_geometry_source()
            && source.voxel_size_m().to_bits() == map.source_voxel_size_m().to_bits()
    });
    if let Some(measured) = map
        .navigation_measured_points()
        .filter(|_| ground.is_none())
    {
        points.extend(measured.iter().map(|p| p.map(|v| v as f32)));
    } else {
        // 没有地面记录的列（旧存档或已分类为空）仅恢复已有源体素中心。
        for (column, bands) in columns {
            let xy = column.map(|v| (f64::from(v) + 0.5) * resolution);
            let mut members = BTreeSet::new();
            for record in ground
                .into_iter()
                .flat_map(|source| source.ground_records(column))
            {
                members.extend(record.members());
                if let Some(height) = record.height_at(xy) {
                    points.push([xy[0] as f32, xy[1] as f32, height as f32]);
                }
            }
            for [low, high] in bands {
                let count = ((high - low) / config.point_resolution).round().max(1.0) as usize;
                for index in 0..count {
                    let z = low + (index as f64 + 0.5) * (high - low) / count as f64;
                    if members.contains(&((z / resolution).floor() as i32)) {
                        continue;
                    }
                    points.push([xy[0] as f32, xy[1] as f32, z as f32]);
                }
            }
        }
    }
    let (recent_ground, recent_obstacles) = map.navigation_recent_samples();
    points.extend(recent_ground.iter().map(|p| p.map(|v| v as f32)));
    let slice_dh = config.robot_height.max(config.point_resolution);
    let min_z = points
        .iter()
        .map(|p| f64::from(p[2]))
        .fold(f64::INFINITY, f64::min);
    let slice_h0 = if min_z.is_finite() {
        min_z + slice_dh
    } else {
        0.0
    };
    let mut tomogram = nuwax_pct_tomography::build(
        &points,
        &nuwax_pct_tomography::TomographyConfig {
            resolution,
            point_resolution: config.point_resolution,
            slice_h0,
            slice_dh,
        },
    )?;
    // 近期非地面只补实体占据，不能把动态物体顶面变成全局地形。
    let half = config.point_resolution * 0.5;
    tomogram.bands.extend(recent_obstacles.iter().map(|p| Band {
        x: (p[0] / resolution).floor() as i32,
        y: (p[1] / resolution).floor() as i32,
        low: p[2] - half,
        high: p[2] + half,
    }));
    tomogram.bands.extend(
        map.navigation_forbidden_columns(resolution)
            .into_iter()
            .map(|column| Band {
                x: column[0],
                y: column[1],
                low: f64::NEG_INFINITY,
                high: f64::INFINITY,
            }),
    );
    Ok((config, tomogram.cells, tomogram.bands))
}
