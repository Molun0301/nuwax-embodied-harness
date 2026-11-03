//! 三层满样本输入使用真实块结构；BL6必须完整保存，不替换编码返回值。
pub(super) fn stage(fine: &mut super::FineCloud, stamp: u64) {
    use nuwax_map_chunks::{ColumnGroundRecords, HeightfieldSurface, LayerSample, LayerSource};
    use nuwax_map_chunks::heightfield_observations::HeightfieldColumn;
    let q = fine.writer.voxel_size_m();
    let mut columns = Vec::new();
    for x in 320..352 {
        for y in 320..352 {
            let mut surfaces = Vec::new();
            for z in [0, 10, 20] {
                let column = HeightfieldColumn::from_counts([x,y], q,
                    [(z, [1,0,0,0,0,0,0,0])].into()).unwrap().unwrap();
                surfaces.push(HeightfieldSurface { column, samples: (0..32).map(|index| LayerSample {
                    source: std::sync::Arc::new(LayerSource::Lidar { backend: 1, runtime_generation: 1, sensor_frame_id: "test".into() }),
                    point: [(f64::from(x)+0.5)*f64::from(q),(f64::from(y)+0.5)*f64::from(q),
                        (f64::from(z)+0.0625)*f64::from(q)],
                    source_sequence: 1, source_timestamp_ns: stamp, point_index: index,
                }).collect() });
                fine.writer.observe([[x,y,z]], stamp).unwrap();
            }
            columns.push(([x,y], ColumnGroundRecords { classified_empty:false,  heightfields:surfaces }));
        }
    }
    fine.writer.replace_ground_columns(columns.iter().map(|(xy, column)|(*xy,column))).unwrap();
}
