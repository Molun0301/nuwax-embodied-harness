use super::*;
use crate::pointcloud::voxel_key;
#[test]
fn an_isolated_ray_does_not_invent_observed_neighbours() {
    let mut filter = DynamicCloudFilter::default();
    let mut map = VoxelIndex::from([(40, 0, 0), (120, 0, 0), (40, 40, 0)]);
    let original = map.clone();
    let points = vec![Point3::new(4.025, 0.025, 0.025); 20];
    for stamp in [1, 1, 0, 2, 3, 4, 5] {
        filter.observe_with_ground(&mut map, 0.05, [0.025; 3], stamp, &points);
    }
    assert_eq!(
        map, original,
        "no 27-cell observed neighbourhood, no invented void"
    );
}
#[test]
fn accumulated_static_room_keeps_actual_observed_geometry() {
    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    let mut filter = DynamicCloudFilter::default();
    let mut map = VoxelIndex::new();
    let mut truth = VoxelIndex::new();
    for frame in 1..=30 {
        let (points, _) = corridor(None, (frame % 3) as f64);
        filter.observe_with_ground(&mut map, 0.05, ORIGIN, frame * 100_000_000, &points);
        let keys = points
            .iter()
            .map(|p| voxel_key([p.x, p.y, p.z], 0.05).unwrap())
            .collect::<VoxelIndex>();
        truth.extend(keys.iter().copied());
        map.extend(keys);
    }
    assert!(truth.iter().all(|k| map.contains(k)));
}
