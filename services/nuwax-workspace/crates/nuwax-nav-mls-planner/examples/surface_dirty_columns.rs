//! Dirty-block column batches: unchanged replacements must not copy persistent-map pages.
use nuwax_nav_mls_planner::{
    mls_planner::Config, SurfaceBuildKind, SurfaceColumnUpdate, SurfaceMapBuilder,
};
use std::{
    alloc::{GlobalAlloc, Layout, System},
    sync::atomic::{AtomicUsize, Ordering},
    time::Instant,
};
struct CountingAllocator;
static ALLOCATIONS: AtomicUsize = AtomicUsize::new(0);
static BYTES: AtomicUsize = AtomicUsize::new(0);
unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        BYTES.fetch_add(layout.size(), Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        BYTES.fetch_add(size, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, size) }
    }
}
#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;
fn config() -> Config {
    Config {
        world_frame: "map".into(),
        voxel_size: 0.1,
        robot_height: 0.6,
        surface_closing_radius: 0.0,
        node_spacing_m: 0.3,
        wall_clearance_m: 0.15,
        wall_buffer_m: 0.3,
        wall_buffer_weight: 2.0,
        step_threshold_m: 0.2,
        max_slope_deg: 25.0,
        min_step_tread_depth_m: 0.2,
        step_penalty_weight: 1.0,
        goal_tolerance: 0.1,
        viz_publish_hz: 0.0,
    }
}
fn main() {
    for side in [64, 256] {
        let mut builder = SurfaceMapBuilder::new(config());
        let points: Vec<_> = (0..side)
            .flat_map(|x| {
                (0..side).map(move |y| ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.0125))
            })
            .collect();
        let first = builder.build(&points, 0.025).unwrap().snapshot;
        let mut updates: Vec<_> = (0..32)
            .flat_map(|x| {
                (0..32).map(move |y| SurfaceColumnUpdate {
                    column: [x, y],
                    cells: vec![(0, [0.0, 0.025])],
                })
            })
            .collect();
        for mode in ["unchanged", "one_changed"] {
            for iteration in 0..20 {
                updates[0].cells = if mode == "one_changed" && iteration % 2 == 0 {
                    vec![(-2, [-0.2, -0.175]), (0, [0.0, 0.025])]
                } else {
                    vec![(0, [0.0, 0.025])]
                };
                ALLOCATIONS.store(0, Ordering::Relaxed);
                BYTES.store(0, Ordering::Relaxed);
                let start = Instant::now();
                let built = builder
                    .build_column_updates(&updates, 0.025, false, 1024)
                    .unwrap();
                let us = start.elapsed().as_secs_f64() * 1e6;
                let allocations = ALLOCATIONS.load(Ordering::Relaxed);
                let bytes = BYTES.load(Ordering::Relaxed);
                assert_eq!(
                    built.stats.changed_columns,
                    usize::from(mode == "one_changed")
                );
                assert_eq!(built.stats.changed_surface_cells, 0);
                assert_eq!(
                    built.stats.kind,
                    if mode == "unchanged" {
                        SurfaceBuildKind::Reused
                    } else {
                        SurfaceBuildKind::Incremental
                    }
                );
                println!(
                    "{}",
                    serde_json::json!({"side":side,"mode":mode,"iteration":iteration,"us":us,"allocations":allocations,"bytes":bytes})
                );
            }
        }
        assert_eq!(first.cell_keys().count(), points.len());
    }
}
