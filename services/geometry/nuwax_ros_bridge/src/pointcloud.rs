use std::collections::HashMap;

const VOXEL_SIZE: f32 = 0.05;

pub struct PointCloudProcessor {
    max_distance: f32,
    min_distance: f32,
    voxel_size: f32,
}

impl PointCloudProcessor {
    pub fn new(min_distance: f32, max_distance: f32, voxel_size: f32) -> Self {
        Self { max_distance, min_distance, voxel_size }
    }

    pub fn default_go2() -> Self {
        Self::new(0.15, 5.0, VOXEL_SIZE)
    }

    /// NaN 过滤 + 距离裁剪 + 体素降采样。
    /// 输入：原始 PointCloud2 data (XYZI 格式, point_step=16)
    /// 输出：过滤后的 data + 有效点数
    pub fn process(&self, data: &[u8], point_step: u32) -> (Vec<u8>, u32) {
        let ps = point_step as usize;
        if ps < 12 || data.len() < ps {
            return (Vec::new(), 0);
        }
        let point_count = data.len() / ps;

        // Pass 1: NaN filter + distance clip
        let mut valid_points: Vec<[f32; 3]> = Vec::with_capacity(point_count / 2);
        for i in 0..point_count {
            let offset = i * ps;
            let x = f32::from_le_bytes(data[offset..offset+4].try_into().unwrap());
            let y = f32::from_le_bytes(data[offset+4..offset+8].try_into().unwrap());
            let z = f32::from_le_bytes(data[offset+8..offset+12].try_into().unwrap());

            if x.is_nan() || y.is_nan() || z.is_nan() {
                continue;
            }
            let dist_sq = x * x + y * y + z * z;
            let min_sq = self.min_distance * self.min_distance;
            let max_sq = self.max_distance * self.max_distance;
            if dist_sq < min_sq || dist_sq > max_sq {
                continue;
            }
            valid_points.push([x, y, z]);
        }

        // Pass 2: voxel downsample using hash grid
        let downsampled = voxel_downsample(&valid_points, self.voxel_size);

        // Serialize to PointCloud2 XYZ format (point_step=12)
        let out_step = 12u32;
        let mut output = Vec::with_capacity(downsampled.len() * out_step as usize);
        for p in &downsampled {
            output.extend_from_slice(&p[0].to_le_bytes());
            output.extend_from_slice(&p[1].to_le_bytes());
            output.extend_from_slice(&p[2].to_le_bytes());
        }

        (output, downsampled.len() as u32)
    }
}

fn voxel_downsample(points: &[[f32; 3]], voxel_size: f32) -> Vec<[f32; 3]> {
    if points.is_empty() || voxel_size <= 0.0 {
        return points.to_vec();
    }
    let inv = 1.0 / voxel_size;
    let mut grid: HashMap<(i32, i32, i32), (f64, f64, f64, u32)> = HashMap::new();

    for p in points {
        let key = (
            (p[0] * inv).floor() as i32,
            (p[1] * inv).floor() as i32,
            (p[2] * inv).floor() as i32,
        );
        let entry = grid.entry(key).or_insert((0.0, 0.0, 0.0, 0));
        entry.0 += p[0] as f64;
        entry.1 += p[1] as f64;
        entry.2 += p[2] as f64;
        entry.3 += 1;
    }

    grid.values()
        .map(|(sx, sy, sz, n)| {
            let n = *n as f64;
            [(*sx / n) as f32, (*sy / n) as f32, (*sz / n) as f32]
        })
        .collect()
}
