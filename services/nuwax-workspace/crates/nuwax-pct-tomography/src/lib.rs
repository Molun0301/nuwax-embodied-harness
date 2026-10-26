//! PCT（点云层析）几何内核；固定上游来源及本地差异见 crate 根目录 NOTICE。
//!
//! 对每个切片，高于切片的最低采样为顶面，不高于切片的最高采样为地面。
//! 仅输出实测地面，同列完全相同的几何切片合并；不同净空全部交原生桥判断。
//! 地图格使用原生桥的 `floor(x / resolution)`，格中心位于 `(x + 0.5) * resolution`。
//! 本模块不计算通行代价、台阶、坡度和膨胀。
//!
//! 2026-10-26：从 PCT_planner 35cd73f 的 tomographyKernel 移植为 Rust/rayon。
use nuwax_nav_pct_native::{Band, Cell};
use rayon::prelude::*;

#[cfg(test)]
mod tests;

#[derive(Clone, Debug)]
pub struct TomographyConfig {
    /// 平面格边长，单位米。
    pub resolution: f64,
    /// 第一层切片的地图高度，单位米。
    pub slice_h0: f64,
    /// 相邻切片高度差，单位米。
    pub slice_dh: f64,
    /// 占据采样合并精度，单位米，与原生桥相同。
    pub point_resolution: f64,
}

#[derive(Clone, Debug)]
pub struct Tomogram {
    pub resolution: f64,
    pub cells: Vec<Cell>,
    pub bands: Vec<Band>,
}

#[derive(Clone, Copy)]
struct Sample {
    x: i32,
    y: i32,
    z: f32,
}

/// 上游切片按浮点模板 U（输入为 f32）保存；保留最终转换位置。
struct Slices {
    h0: f64,
    dh: f64,
    count: usize,
}

impl Slices {
    fn new(max_z: f32, cfg: &TomographyConfig) -> Result<Self, String> {
        let last = ((f64::from(max_z) - cfg.slice_h0) / cfg.slice_dh)
            .ceil()
            .max(0.0);
        // 高度索引必须可由 f64 精确表达；检查后才转为无符号整数。
        let max_index = ((1_u64 << 53) - 2).min((usize::MAX - 1) as u64) as f64;
        if !last.is_finite() || last > max_index {
            return Err("层析切片数量超出可表示范围，请检查切片间隔与高度".into());
        }
        let mut slices = Self {
            h0: cfg.slice_h0,
            dh: cfg.slice_dh,
            count: last as usize + 1,
        };
        // 修正推导末层时的 f64 舍入；不改变实际切片高度与比较规则。
        if slices.height(slices.count - 1) < max_z {
            slices.count += 1;
        }
        Ok(slices)
    }

    fn height(&self, index: usize) -> f32 {
        (self.h0 + index as f64 * self.dh) as f32
    }

    fn first_at_or_above(&self, z: f32) -> usize {
        let (mut low, mut high) = (0, self.count);
        while low < high {
            let middle = low + (high - low) / 2;
            if self.height(middle) < z {
                low = middle + 1;
            } else {
                high = middle;
            }
        }
        low
    }
}

/// 对整张静态地图构建几何层析；无效点（任一坐标非有限）沿用上游过滤语义。
pub fn build(static_points_map: &[[f32; 3]], cfg: &TomographyConfig) -> Result<Tomogram, String> {
    build_selected(static_points_map, cfg, None)
}

/// 使用地图坐标下的闭合正方形筛选源点，格索引始终与整图一致。
pub fn build_region(
    static_points_map: &[[f32; 3]],
    cfg: &TomographyConfig,
    center: [f64; 2],
    half_extent_m: f64,
) -> Result<Tomogram, String> {
    if !center.iter().all(|value| value.is_finite())
        || !half_extent_m.is_finite()
        || half_extent_m < 0.0
    {
        return Err("层析区域中心必须有限，半边长必须为有限非负值".into());
    }
    build_selected(static_points_map, cfg, Some((center, half_extent_m)))
}

fn validate(cfg: &TomographyConfig) -> Result<(), String> {
    if !cfg.resolution.is_finite() || cfg.resolution <= 0.0 {
        return Err("层析格边长必须为有限正值".into());
    }
    if !cfg.slice_h0.is_finite() || !cfg.slice_dh.is_finite() || cfg.slice_dh <= 0.0 {
        return Err("层析首层高度必须有限，切片间隔必须为有限正值".into());
    }
    if !cfg.point_resolution.is_finite() || cfg.point_resolution <= 0.0 {
        return Err("层析占据采样精度必须为有限正值".into());
    }
    Ok(())
}

fn coordinate(value: f32, resolution: f64) -> Result<i32, String> {
    let index = (f64::from(value) / resolution).floor();
    if !index.is_finite() || index < f64::from(i32::MIN) || index > f64::from(i32::MAX) {
        return Err("层析点坐标超出 i32 地图格范围".into());
    }
    Ok(index as i32)
}

fn build_selected(
    points: &[[f32; 3]],
    cfg: &TomographyConfig,
    region: Option<([f64; 2], f64)>,
) -> Result<Tomogram, String> {
    validate(cfg)?;
    let mut samples = match region {
        None => Vec::with_capacity(points.len()),
        Some(_) => Vec::new(),
    };
    let mut max_z = f32::NEG_INFINITY;
    for point in points {
        if !point.iter().all(|value| value.is_finite()) {
            continue;
        }
        if let Some((center, extent)) = region {
            if (f64::from(point[0]) - center[0]).abs() > extent
                || (f64::from(point[1]) - center[1]).abs() > extent
            {
                continue;
            }
        }
        samples.push(Sample {
            x: coordinate(point[0], cfg.resolution)?,
            y: coordinate(point[1], cfg.resolution)?,
            z: point[2],
        });
        max_z = max_z.max(point[2]);
    }
    if samples.is_empty() {
        return Ok(Tomogram {
            resolution: cfg.resolution,
            cells: Vec::new(),
            bands: Vec::new(),
        });
    }
    let slices = Slices::new(max_z, cfg)?;
    samples.par_sort_unstable_by(|left, right| {
        (left.x, left.y)
            .cmp(&(right.x, right.y))
            .then_with(|| left.z.total_cmp(&right.z))
    });
    let mut columns = Vec::new();
    let mut first = 0;
    for end in 1..=samples.len() {
        if end < samples.len()
            && (samples[end].x, samples[end].y) == (samples[first].x, samples[first].y)
        {
            continue;
        }
        columns.push(first..end);
        first = end;
    }
    let geometry: Vec<_> = columns
        .par_iter()
        .map(|range| column_geometry(&samples[range.clone()], &slices, cfg.point_resolution))
        .collect();
    let cell_count = geometry.iter().map(|(cells, _)| cells.len()).sum();
    let band_count = geometry.iter().map(|(_, bands)| bands.len()).sum();
    let mut result = Tomogram {
        resolution: cfg.resolution,
        cells: Vec::with_capacity(cell_count),
        bands: Vec::with_capacity(band_count),
    };
    for (cells, bands) in geometry {
        result.cells.extend(cells);
        result.bands.extend(bands);
    }
    Ok(result)
}

fn column_geometry(
    samples: &[Sample],
    slices: &Slices,
    point_resolution: f64,
) -> (Vec<Cell>, Vec<Band>) {
    let (x, y) = (samples[0].x, samples[0].y);
    let mut cells = Vec::new();
    let mut bands = Vec::new();
    let mut band_first = 0;
    for index in 0..samples.len() {
        let z = samples[index].z;
        let next = samples.get(index + 1).map(|point| point.z);
        if next == Some(z) {
            continue;
        }
        // 排序后的每个高度区间只需判断是否包含切片，等价于逐切片最大/最小值。
        // 未命中的窄区间不生成地面；命中的重复切片仅输出一次。
        let first_slice = slices.first_at_or_above(z);
        if first_slice < slices.count
            && next.is_none_or(|ceiling| slices.height(first_slice) < ceiling)
        {
            cells.push(Cell {
                x,
                y,
                ground: f64::from(z),
                ceiling: next.map_or(f64::INFINITY, f64::from),
            });
        }
        // 与既有桥的 add_points 相同：相邻采样间隔不大于精度的1.5倍即连续。
        if next.is_none_or(|height| f64::from(height) - f64::from(z) > point_resolution * 1.5) {
            bands.push(Band {
                x,
                y,
                low: f64::from(samples[band_first].z),
                high: f64::from(z),
            });
            band_first = index + 1;
        }
    }
    (cells, bands)
}
