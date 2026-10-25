use super::*;

fn layers(samples: usize) -> LayerObservations {
    [(0, LayerObservation {
        members: vec![0], plane_origin: [0.0; 3], normal: [0.0, 0.0, 1.0],
        samples: (0..samples).map(|i| LayerSample {
            source: Arc::new(LayerSource::Legacy),
            point: [0.0; 3], source_sequence: 1, source_timestamp_ns: 1, point_index: i as u32,
        }).collect(),
    })].into()
}
fn writer() -> Writer {
    Writer::new(Repository::temporary().unwrap(), 0.125, 64 * 1024 * 1024).unwrap()
}
fn assert_accounting(w: &Writer) {
    let bytes: usize = w.dirty.values().flat_map(|d| d.layers.iter())
        .map(|(_, record)| match record {
            GroundRecordRef::LegacyPlane(layer) => terrain_observations::layer_bytes(layer),
            GroundRecordRef::HeightfieldV1(_) | GroundRecordRef::ClassifiedEmptyColumn => panic!("original accounting fixture is Legacy"),
        }).sum();
    assert_eq!(w.dirty_layer_bytes(), bytes);
    assert_eq!(w.resident_bytes(), w.source_watermark_bytes + bytes + w.dirty_points * 64 + w.reader.cached_bytes()
        + w.pending.values().map(|p| p.bytes.len()).sum::<usize>());
}

#[test]
fn partial_layer_deletion_releases_member_and_original_sample_budget() {
    let mut w=writer();
    w.observe([[0,0,0],[0,0,1]],1).unwrap();
    let mut observations=layers(2);
    let layer=observations.get_mut(&0).unwrap();
    layer.members.push(1024);
    layer.samples[0].point=[0.01,0.01,0.124];
    layer.samples[1].point=[0.01,0.01,0.126];
    w.replace_layer_observations([0,0,0],observations).unwrap();
    assert_accounting(&w);
    let before=w.dirty_layer_bytes();
    w.remove([[0,0,0]]).unwrap();
    assert_accounting(&w);
    assert_eq!(w.dirty_layer_bytes(),before-2-64);
    w.remove([[0,0,1]]).unwrap();
    assert_accounting(&w);
    assert_eq!(w.dirty_layer_bytes(),0);
}
#[test]
fn dirty_layer_accounting_tracks_mutations_and_failed_flush() {
    let mut w = writer();
    w.observe([[0, 0, 0], [32, 0, 0]], 1).unwrap();
    assert_accounting(&w);
    w.replace_layer_observations([0, 0, 0], layers(1)).unwrap();
    w.replace_layer_observations([1, 0, 0], layers(2)).unwrap();
    assert_accounting(&w);
    let bytes = w.resident_bytes();
    w.replace_layer_observations([0, 0, 0], layers(1)).unwrap();
    w.observe([[0, 0, 0]], 1).unwrap();
    assert_eq!(w.resident_bytes(), bytes);
    let mut invalid = layers(3);
    invalid.get_mut(&0).unwrap().members.push(1);
    assert!(w.replace_layer_observations([0, 0, 0], invalid).is_err());
    assert_eq!(w.resident_bytes(), bytes);
    w.replace_layer_observations([0, 0, 0], layers(4)).unwrap();
    assert_accounting(&w);
    w.replace_layer_observations([0, 0, 0], layers(1)).unwrap();
    assert_accounting(&w);
    w.repository.fail_put_after(Some(1));
    assert!(w.flush().is_err());
    assert_accounting(&w);
    let pending_ids = w.pending_ids();
    w.repository.fail_put_after(None);
    w.flush().unwrap();
    assert_accounting(&w);
    assert!(w.dirty.is_empty());
    assert!(pending_ids.iter().all(|id| w.retained_objects().contains(id)));
    w.observe([[0, 0, 0], [32, 0, 0]], 1).unwrap();
    assert_accounting(&w);
    assert_eq!(w.dirty_layer_bytes(), 194 + 258);
    assert!(w.dirty.values().all(|d| !d.changed));
    w.replace_layer_observations([0, 0, 0], LayerObservations::new()).unwrap();
    assert_accounting(&w);
    w.remove([[32, 0, 0]]).unwrap();
    assert_accounting(&w);
    assert_eq!(w.dirty_layer_bytes(), 0);
    w.flush().unwrap();
    assert_eq!(w.blocks.len(), 1);
    w.observe([[0, 0, 0]], 1).unwrap();
    w.replace_layer_observations([0, 0, 0], layers(3)).unwrap();
    w.reset_pending();
    assert_accounting(&w);
    assert_eq!(w.dirty_layer_bytes(), 0);
}
#[test]
fn dirty_layer_accounting_keeps_exact_pressure_boundary() {
    let mut w = writer();
    w.observe([[0, 0, 0]], 1).unwrap();
    w.replace_layer_observations([0, 0, 0], layers(1)).unwrap();
    // Equality must not flush; one unit below must flush exactly as before.
    w.dirty_point_budget = 1 + 32768 + 194 / 64;
    w.observe([[32, 0, 0]], 1).unwrap();
    assert_eq!(w.encoded_blocks, 0);
    assert_accounting(&w);
    w.observe([[64, 0, 0]], 1).unwrap();
    assert_eq!(w.encoded_blocks, 2);
    assert_eq!(w.dirty.len(), 1);
    assert_accounting(&w);
    assert_eq!(w.dirty_layer_bytes(), 0);
}
#[test]
#[ignore = "release-only synthetic dirty-block bookkeeping diagnostic"]
fn dirty_layer_accounting_cost() {
    for n in [1024_i32, 8192] {
        for populated in [false, true] {
            let mut w = writer();
            w.observe((0..n).map(|i| [i * 32, 0, 0]), 1).unwrap();
            if populated {
                for i in 0..n { w.replace_layer_observations([i, 0, 0], layers(1)).unwrap(); }
            }
            let start = std::time::Instant::now();
            w.observe((n..2*n).map(|i| [i * 32, 0, 0]), 1).unwrap();
            let elapsed = start.elapsed();
            assert_eq!(w.dirty.len(), 2 * n as usize);
            assert_eq!(w.encoded_blocks, 0);
            assert_eq!(w.dirty_points, 2 * n as usize);
            assert_eq!(w.dirty_layer_bytes(), if populated { n as usize * 194 } else { 0 });
            assert_accounting(&w);
            println!("blocks={n} populated={populated} elapsed_us={:.3}", elapsed.as_secs_f64() * 1e6);
        }
    }
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
    let column = |s: HeightfieldSurface| ColumnGroundRecords { classified_empty: false, legacy: vec![], heightfields: vec![s] };
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
