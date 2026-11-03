fn assert_accounting(w: &Writer) {
    let bytes: usize = w.dirty.values().map(|d| d.layers.bytes()).sum();
    assert_eq!(w.dirty_layer_bytes(), bytes);
    assert_eq!(w.resident_bytes(), w.source_watermark_bytes + bytes + w.dirty_points * 64 + w.reader.cached_bytes()
        + w.pending.values().map(|p| p.bytes.len()).sum::<usize>());
}
use super::*;

fn layers(samples: usize) -> LayerObservations {
    [(0, LayerObservation {
        members: vec![0], plane_origin: [0.0; 3], normal: [0.0, 0.0, 1.0],
        samples: (0..samples).map(|i| LayerSample {
            source: Arc::new(LayerSource::Lidar { backend: 1, runtime_generation: 1, sensor_frame_id: "test".into() }),
            point: [0.0; 3], source_sequence: 1, source_timestamp_ns: 1, point_index: i as u32,
        }).collect(),
    })].into()
}
fn writer() -> Writer {
    Writer::new(Repository::temporary().unwrap(), 0.125, 64 * 1024 * 1024).unwrap()
}







// 高度场记录在脏块上原地替换、删除后，逐块字节统计仍等于整表重算。
#[test]
fn heightfield_in_place_edits_keep_block_bytes_equal_to_full_recount() {
    use heightfield_observations::HeightfieldColumn;
    let surface = |xy: [i32; 2], zs: &[i32], samples: u32| HeightfieldSurface {
        column: HeightfieldColumn::from_counts(xy, 0.125, zs.iter().map(|&z| (z, [1, 0, 0, 0, 0, 0, 0, 0])).collect()).unwrap().unwrap(),
        samples: (0..samples).map(|i| LayerSample { source: Arc::new(LayerSource::Lidar { backend: 1, runtime_generation: 1,
            sensor_frame_id: format!("sensor-{}", i % 2) }), point: [0.0; 3], source_sequence: 1, source_timestamp_ns: 1, point_index: i }).collect(),
    };
    let column = |s: HeightfieldSurface| ColumnGroundRecords { classified_empty: false,  heightfields: vec![s] };
    let check = |w: &Writer| {
        for dirty in w.dirty.values() { assert_eq!(dirty.layer_bytes, dirty.layers.bytes()); }
        assert_eq!(w.dirty_layer_bytes(), w.dirty.values().map(|d| d.layers.bytes()).sum::<usize>());
    };
    let mut w = writer();
    w.observe((0..4).flat_map(|x| (0..3).map(move |z| [x, 0, z])), 1).unwrap();
    let first: Vec<_> = (0..4).map(|x| ([x, 0], column(surface([x, 0], &[0, 1], 3)))).collect();
    w.replace_ground_columns(first.iter().map(|(xy, c)| (*xy, c))).unwrap();
    check(&w);
    let second: Vec<_> = (0..2).map(|x| ([x, 0], column(surface([x, 0], &[1, 2], 5)))).collect();
    w.replace_ground_columns(second.iter().map(|(xy, c)| (*xy, c))).unwrap();
    check(&w);
    w.remove([[2, 0, 1], [0, 0, 2]]).unwrap();
    check(&w);
}

// 自动模式：即将压力刷写时先扩大预算（至少翻倍、不超过上限），到上限后照原方式刷写，
// 警告每个写入器只记一次；固定模式的刷写边界见上一个用例，不受影响。
#[test]
fn automatic_cache_grows_instead_of_flushing_until_its_limit_and_warns_once() {
    const MIB: usize = 1024 * 1024;
    let mut w = Writer::new(Repository::temporary().unwrap(), 0.125, 4 * MIB).unwrap();
    assert!(w.set_cache_policy(CachePolicy::Auto { max_bytes: 4 * MIB - 128 }).is_err(), "上限不能低于当前预算");
    w.set_cache_policy(CachePolicy::Auto { max_bytes: 4 * MIB + 2 * 128 }).unwrap();
    // 每次观测一个新块：前三个块靠扩大预算放下，不刷写。
    for i in 0..3 { w.observe([[i * 32, 0, 0]], 1).unwrap(); }
    assert_eq!(w.encoded_blocks, 0);
    assert_eq!(w.cache_budget_bytes(), 4 * MIB + 2 * 128);
    assert_eq!(w.cache_limit_warning(), None);
    // 第四个块超过上限：刷写全部脏块，记下所需与上限。
    w.observe([[96, 0, 0]], 1).unwrap();
    assert_eq!(w.encoded_blocks, 3);
    let first = Some((4 * MIB + 3 * 128, 4 * MIB + 2 * 128));
    assert_eq!(w.cache_limit_warning(), first);
    for i in 4..12 { w.observe([[i * 32, 0, 0]], 1).unwrap(); }
    assert!(w.encoded_blocks > 3, "到上限后继续按原方式刷写");
    assert_eq!(w.cache_warnings, 1);
    assert_eq!(w.cache_limit_warning(), first);
    assert_eq!(w.cache_budget_bytes(), 4 * MIB + 2 * 128, "预算只增不减");
    assert_accounting(&w);
    // 上限足够时按翻倍扩大。
    let mut roomy = Writer::new(Repository::temporary().unwrap(), 0.125, 4 * MIB).unwrap();
    roomy.set_cache_policy(CachePolicy::Auto { max_bytes: 64 * MIB }).unwrap();
    for i in 0..3 { roomy.observe([[i * 32, 0, 0]], 1).unwrap(); }
    assert_eq!((roomy.encoded_blocks, roomy.cache_budget_bytes()), (0, 8 * MIB));
}
