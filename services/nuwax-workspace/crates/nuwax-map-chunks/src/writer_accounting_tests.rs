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
            GroundRecordRef::HeightfieldV1(_) => panic!("original accounting fixture is Legacy"),
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
