//! 当前扫描的滚动稀疏块；旧帧缓冲只复用存储，不复用占据或来源年龄。
use super::rays::project_updates;
use nuwax_mapping::OccupancyGrid;
use nuwax_slam_2d_runtime::input::NavigationRay;
use std::collections::BTreeMap;

const SIDE: usize = 32;
type Block = Box<[i8; SIDE * SIDE]>;

#[derive(Default)]
pub(super) struct LocalObservation {
    blocks: BTreeMap<(usize, usize), Block>,
}
impl LocalObservation {
    pub(super) fn compose(
        &mut self,
        physical: &OccupancyGrid,
        rays: &[NavigationRay],
        pose: [f64; 3],
    ) -> Result<OccupancyGrid, String> {
        let mut previous = std::mem::take(&mut self.blocks);
        let mut current = BTreeMap::new();
        project_updates(physical.width, physical.height, physical.resolution, physical.origin, rays, pose, |index, value| {
            let x = index % physical.width as usize;
            let y = index / physical.width as usize;
            let block = current.entry((x / SIDE, y / SIDE)).or_insert_with(|| {
                let mut block = previous.remove(&(x / SIDE, y / SIDE)).unwrap_or_else(|| Box::new([-1; SIDE * SIDE]));
                block.fill(-1);
                block
            });
            block[(y % SIDE) * SIDE + x % SIDE] = value;
        })?;
        // 完整协议仍需要一个全图输出；只有本帧明确观测过的位置覆盖物理图。
        let mut output = physical.clone();
        for (&(bx, by), block) in &current {
            for y in 0..SIDE.min(physical.height as usize - by * SIDE) {
                let width = SIDE.min(physical.width as usize - bx * SIDE);
                let start = (by * SIDE + y) * physical.width as usize + bx * SIDE;
                for x in 0..width {
                    let value = block[y * SIDE + x];
                    if value >= 0 { output.data[start + x] = value; }
                }
            }
        }
        self.blocks = current;
        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::rays::trace_rays;
    fn ray(from: [f64; 2], to: [f64; 2], hit: bool, trace_free: bool) -> NavigationRay {
        NavigationRay { origin_base_m: from, endpoint_base_m: to, hit, trace_free, source_timestamp_ns: 1 }
    }
    #[test]
    fn rolling_blocks_match_full_scan_across_layouts_and_never_reuse_old_evidence() {
        let mut cache = LocalObservation::default();
        for layout in 0..4 {
            let mut physical = OccupancyGrid::new(127 + layout * 7, 99, 0.05 + f64::from(layout) * 0.01);
            physical.origin = [-2.37 + f64::from(layout) * 0.2, -1.18, 0.0];
            for (i, value) in physical.data.iter_mut().enumerate() { *value = if i % 7 == 0 { 100 } else if i % 3 == 0 { 0 } else { -1 }; }
            for frame in 0..24 {
                let pose = [frame as f64 * 0.17 - 1.0, 0.37, frame as f64 * 0.13];
                let rays: Vec<_> = (0..180).map(|i| {
                    let angle = i as f64 * std::f64::consts::TAU / 180.0;
                    ray([0.19, -0.12], [angle.cos() * 2.3, angle.sin() * 2.3], i % 3 != 0, i % 5 != 0)
                }).collect();
                let mut reference = physical.clone();
                trace_rays(&mut reference, &rays, pose).unwrap();
                let result = cache.compose(&physical, &rays, pose).unwrap();
                assert_eq!(reference.data, result.data, "布局{layout}帧{frame}");
                assert_eq!(result.origin, physical.origin);
            }
            // 同一窗口没有本帧射线时恢复真实物理图，旧自由清除和障碍都不得续期。
            assert_eq!(cache.compose(&physical, &[], [0.0; 3]).unwrap().data, physical.data);
            assert!(cache.blocks.is_empty());
        }
    }
    #[test]
    fn invalid_frame_drops_partial_blocks_and_physical_revision_is_authoritative() {
        let mut cache = LocalObservation::default();
        let mut physical = OccupancyGrid::new(100, 100, 0.1);
        let good = ray([0.0, 0.0], [2.0, 0.0], false, true);
        cache.compose(&physical, &[good.clone()], [1.0, 1.0, 0.0]).unwrap();
        let invalid = ray([0.0, 0.0], [f64::NAN, 1.0], true, true);
        assert!(cache.compose(&physical, &[good, invalid], [1.0, 1.0, 0.0]).is_err());
        assert!(cache.blocks.is_empty());
        physical.data[1010] = 100;
        physical.origin = [-4.0, -3.0, 0.0];
        assert_eq!(cache.compose(&physical, &[], [0.0; 3]).unwrap().data, physical.data);
    }
    #[test]
    fn fixed_scan_block_count_does_not_grow_with_global_map() {
        let mut counts = Vec::new();
        for side in [256, 1024, 2048] {
            let physical = OccupancyGrid::new(side, side, 0.05);
            let mut cache = LocalObservation::default();
            let rays: Vec<_> = (0..360).map(|i| { let a = i as f64 * std::f64::consts::TAU / 360.0; ray([0.0, 0.0], [a.cos() * 1.6, a.sin() * 1.6], true, true) }).collect();
            let mut reference_times = Vec::new();
            let mut local_times = Vec::new();
            for _ in 0..11 {
                let start = std::time::Instant::now();
                let mut reference = physical.clone();
                trace_rays(&mut reference, &rays, [3.0, 3.0, 0.0]).unwrap();
                reference_times.push(start.elapsed().as_micros());
                let start = std::time::Instant::now();
                let result = cache.compose(&physical, &rays, [3.0, 3.0, 0.0]).unwrap();
                local_times.push(start.elapsed().as_micros());
                assert_eq!(result.data, reference.data);
            }
            reference_times.sort_unstable(); local_times.sort_unstable();
            println!("global_cells={} local_blocks={} full_output_us={} local_complete_output_us={}", physical.data.len(), cache.blocks.len(), reference_times[5], local_times[5]);
            counts.push(cache.blocks.len());
        }
        assert!(counts[0] > 0 && counts.iter().all(|c| *c == counts[0]));
    }
}
