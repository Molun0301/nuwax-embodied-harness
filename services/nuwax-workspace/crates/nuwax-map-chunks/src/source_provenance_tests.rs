use super::*;
use std::io::{Cursor, Read, Seek, SeekFrom};

fn domain() -> Domain {
    Domain {
        session: "source-provenance".into(),
        generation: 1,
        frame_generation: 0,
        producer_generation: 1,
    }
}
fn lidar() -> Arc<LayerSource> {
    Arc::new(LayerSource::Lidar {
        backend: 2,
        runtime_generation: i64::MAX,
        sensor_frame_id: "lidar".into(),
    })
}
fn auxiliary(sensor: &str, clock: &str, runtime: u64, session: u64) -> Arc<LayerSource> {
    Arc::new(LayerSource::Auxiliary {
        sensor_id: sensor.into(),
        clock_id: clock.into(),
        runtime_generation: runtime,
        session_generation: session,
    })
}
fn sample(source: Arc<LayerSource>, sequence: u64, timestamp: u64, index: u32) -> LayerSample {
    LayerSample {
        source,
        point: [0.03125, 0.0625, 0.09375],
        source_sequence: sequence,
        source_timestamp_ns: timestamp,
        point_index: index,
    }
}
fn layer(samples: Vec<LayerSample>) -> LayerObservation {
    LayerObservation {
        members: vec![0],
        plane_origin: [0.0625, 0.0625, 0.09375],
        normal: [0.0, 0.0, 1.0],
        samples,
    }
}
fn block(timestamp: u64) -> Block {
    [(
        0,
        Evidence {
            observed_ns: timestamp,
            hits: 1,
        },
    )]
    .into()
}
fn writer() -> Writer {
    Writer::new(Repository::temporary().unwrap(), 0.125, 4 * 1024 * 1024).unwrap()
}
fn install(w: &mut Writer, observed: LayerObservation) {
    w.observe([[0, 0, 0]], observed.latest_timestamp_ns())
        .unwrap();
    w.replace_layer_observations([0, 0, 0], [(0, observed)].into())
        .unwrap();
}
fn raw_layer(magic: &[u8; 8], value: &serde_json::Value) -> Vec<u8> {
    let json = serde_json::to_vec(&vec![(0_u16, value)]).unwrap();
    let mut bytes = encode_block(&block(20)).unwrap();
    bytes[..8].copy_from_slice(magic);
    bytes[12..16].copy_from_slice(&(json.len() as u32).to_le_bytes());
    bytes.extend(json);
    bytes
}

#[test]
fn typed_groups_roundtrip_share_sources_and_preserve_original_numeric_samples() {
    let source = auxiliary("front-depth", "device-A", u64::MAX, u64::MAX);
    let observed = layer(vec![
        sample(lidar(), 7, 20, 3),
        sample(source.clone(), 7, 20, 3),
        sample(source, 0, 20, 4),
        sample(Arc::new(LayerSource::Legacy), 7, 20, 3),
    ]);
    let layers = [(0, observed.clone())].into();
    let bytes = encode_layer_block(&block(20), &layers).unwrap();
    assert!(bytes.starts_with(b"NUWAXBL3"));
    let json: serde_json::Value = serde_json::from_slice(&bytes[28..]).unwrap();
    let wire = &json[0][1];
    assert_eq!(wire["sample_sources"].as_array().unwrap().len(), 2);
    for (wire_sample, original) in wire["samples"]
        .as_array()
        .unwrap()
        .iter()
        .zip(&observed.samples)
    {
        assert_eq!(wire_sample.as_object().unwrap().len(), 4);
        assert!(wire_sample.get("source").is_none());
        assert_eq!(
            wire_sample["point"],
            serde_json::to_value(original.point).unwrap()
        );
        assert_eq!(wire_sample["source_sequence"], original.source_sequence);
        assert_eq!(
            wire_sample["source_timestamp_ns"],
            original.source_timestamp_ns
        );
        assert_eq!(wire_sample["point_index"], original.point_index);
    }
    let decoded = decode_layer_observations(&bytes).unwrap();
    assert_eq!(decoded, layers);
    assert!(Arc::ptr_eq(
        &decoded[&0].samples[1].source,
        &decoded[&0].samples[2].source
    ));
    assert!(decoded[&0].samples[3].source.is_legacy());
    assert_eq!(block_summary(&bytes).unwrap(), (1, 20));
}

#[test]
fn every_real_namespace_dimension_separates_identical_sample_ids() {
    let sources = vec![
        lidar(),
        Arc::new(LayerSource::Lidar {
            backend: 1,
            runtime_generation: i64::MAX,
            sensor_frame_id: "lidar".into(),
        }),
        Arc::new(LayerSource::Lidar {
            backend: 2,
            runtime_generation: 1,
            sensor_frame_id: "lidar".into(),
        }),
        Arc::new(LayerSource::Lidar {
            backend: 2,
            runtime_generation: i64::MAX,
            sensor_frame_id: "other-lidar".into(),
        }),
        auxiliary("front", "device-A", 1, 1),
        auxiliary("rear", "device-A", 1, 1),
        auxiliary("front", "device-B", 1, 1),
        auxiliary("front", "device-A", 2, 1),
        auxiliary("front", "device-A", 1, 2),
    ];
    let mut observed = layer(
        sources
            .into_iter()
            .map(|source| sample(source, 9, 20, 2))
            .collect(),
    );
    assert!(encode_layer_block(&block(20), &[(0, observed.clone())].into()).is_ok());
    observed.samples.push(observed.samples[0].clone());
    assert!(encode_layer_block(&block(20), &[(0, observed)].into()).is_err());
}

#[test]
fn malformed_source_groups_are_rejected_without_accepting_inline_claims() {
    let observed = layer(vec![
        sample(lidar(), 1, 20, 0),
        sample(auxiliary("depth", "clock", 1, 1), 0, 20, 1),
    ]);
    let valid = serde_json::to_value(observed).unwrap();
    for bad in 0..8 {
        let mut value = valid.clone();
        match bad {
            0 => value["sample_sources"][1]["sample_positions"] = serde_json::json!([0]),
            1 => value["sample_sources"][0]["sample_positions"] = serde_json::json!([0, 0]),
            2 => value["sample_sources"][0]["sample_positions"] = serde_json::json!([2]),
            3 => value["sample_sources"][0]["sample_positions"] = serde_json::json!([]),
            4 => value["sample_sources"][0]["source"] = serde_json::json!({"kind":"legacy"}),
            5 => {
                value["sample_sources"][1]["source"] = value["sample_sources"][0]["source"].clone()
            }
            6 => value["sample_sources"][0]["source"]["runtime_generation"] = serde_json::json!(-1),
            _ => value["samples"][0]["source"] = serde_json::json!({"kind":"legacy"}),
        }
        assert!(
            decode_layer_observations(&raw_layer(b"NUWAXBL3", &value)).is_err(),
            "case {bad}"
        );
    }
    assert!(decode_layer_observations(&raw_layer(b"NUWAXBL2", &valid)).is_err());
}

#[test]
fn source_watermarks_survive_ar4_restore_without_cross_clock_cutoffs() {
    let source = auxiliary("depth", "device-clock", 4, 8);
    let same_numeric_ids = auxiliary("rear-depth", "device-B", 4, 8);
    let mut w = writer();
    install(
        &mut w,
        layer(vec![
            sample(source.clone(), 0, 2, 0),
            sample(lidar(), 9, 20, 0),
            sample(same_numeric_ids.clone(), 9, 20, 0),
        ]),
    );
    w.record_source_watermark(source.clone(), 2, 0).unwrap();
    w.record_source_watermark(lidar(), 20, 9).unwrap();
    w.record_source_watermark(same_numeric_ids, 20, 9).unwrap();
    let root = w.snapshot(domain(), 1, 20).unwrap();
    assert_eq!(root.version, 4);
    assert_eq!(root.source_watermarks.len(), 3);
    let (mut file, _) = archive_file(&root, w.repository()).unwrap();
    // Opt-in interoperability evidence, never an artifact inside the repository.
    if let Some(path) = std::env::var_os("NEH_PROVENANCE_ARCHIVE_OUT") {
        let mut bytes = Vec::new();
        file.read_to_end(&mut bytes).unwrap();
        std::fs::write(path, bytes).unwrap();
        file.seek(SeekFrom::Start(0)).unwrap();
    }
    let mut magic = [0; 8];
    file.read_exact(&mut magic).unwrap();
    assert_eq!(&magic, b"NUWAXAR4");
    file.seek(SeekFrom::Start(0)).unwrap();
    let repository = Repository::temporary().unwrap();
    let restored_root = ArchiveReader::open(file)
        .unwrap()
        .import(&repository)
        .unwrap();
    let mut restored = Writer::new(repository.clone(), 0.125, 4 * 1024 * 1024).unwrap();
    let reads = repository.counts().0;
    restored.restore_archived(&restored_root).unwrap();
    assert_eq!(repository.counts().0 - reads, 1);
    assert_eq!(restored.source_watermark(&source), Some((2, 0)));
    assert_eq!(restored.source_watermark(&lidar()), Some((20, 9)));
    assert!(!restored.has_unpublished_changes());
    restored
        .record_source_watermark(source.clone(), 2, 0)
        .unwrap();
    assert!(!restored.has_unpublished_changes());
    restored
        .record_source_watermark(source.clone(), 2, 1)
        .unwrap();
    restored
        .record_source_watermark(source.clone(), 3, 0)
        .unwrap();
    assert!(restored
        .record_source_watermark(source.clone(), 2, u64::MAX)
        .is_err());
    let reset = auxiliary("depth", "device-clock", 5, 8);
    assert_eq!(restored.source_watermark(&reset), None);
    restored
        .record_source_watermark(reset.clone(), 1, 0)
        .unwrap();
    assert_eq!(restored.source_watermark(&reset), Some((1, 0)));
    let second = restored.snapshot(domain(), 2, 20).unwrap();
    assert_eq!(second.blocks, root.blocks);
    assert_eq!(second.geometry_revision, root.geometry_revision);
    assert_eq!(second.source_watermarks.len(), 4);
    assert!(!restored.has_unpublished_changes());
}

#[test]
fn old_ar3_and_missing_watermark_field_restore_only_legacy_sources() {
    let mut w = writer();
    let observed = layer(vec![sample(Arc::new(LayerSource::Legacy), 1, 20, 0)]);
    install(&mut w, observed.clone());
    let mut old = (*w.snapshot(domain(), 1, 20).unwrap()).clone();
    old.version = 3;
    let bytes = old.encode().unwrap();
    assert!(!serde_json::from_slice::<serde_json::Value>(&bytes)
        .unwrap()
        .as_object()
        .unwrap()
        .contains_key("source_watermarks"));
    let (file, _) = archive_file(&old, w.repository()).unwrap();
    let repository = Repository::temporary().unwrap();
    let archived = ArchiveReader::open(file)
        .unwrap()
        .import(&repository)
        .unwrap();
    let mut restored = Writer::new(repository, 0.125, 4 * 1024 * 1024).unwrap();
    restored.restore_archived(&archived).unwrap();
    assert_eq!(restored.source_watermark(&lidar()), None);
    assert_eq!(
        restored.layer_observations([0, 0, 0]).unwrap()[&0],
        observed
    );
    assert_eq!(restored.snapshot(domain(), 2, 20).unwrap().version, 4);
    for version in [2, 3, 4] {
        old.version = version;
        assert!(Manifest::decode(&old.encode().unwrap())
            .unwrap()
            .source_watermarks
            .is_empty());
    }
}

#[test]
fn typed_sources_require_explicit_root_and_archive_versions() {
    let mut w = writer();
    install(&mut w, layer(vec![sample(lidar(), 1, 20, 0)]));
    let root = w.snapshot(domain(), 1, 20).unwrap();
    let mut old = (*root).clone();
    old.version = 3;
    assert!(archive_file(&old, w.repository()).is_err());
    assert!(w
        .repository()
        .copy_snapshot(&old, &Repository::temporary().unwrap())
        .is_err());
    let mut restored = Writer::new(w.repository().clone(), 0.125, 4 * 1024 * 1024).unwrap();
    assert!(restored.restore(&old).is_err());
    let mut recv = Receiver::new(Repository::temporary().unwrap());
    assert!(recv
        .receive(
            &Transfer {
                root: old.encode().unwrap(),
                included: BTreeMap::new()
            },
            &domain(),
            |_, b| w.repository().read_object(b.object, MAX_BLOCK_BYTES)
        )
        .is_err());
    old.source_watermarks.push(SourceWatermark {
        source: lidar(),
        source_timestamp_ns: 20,
        source_sequence: 1,
    });
    assert!(old.encode().is_err());
    let (mut file, _) = archive_file(&root, w.repository()).unwrap();
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).unwrap();
    bytes[..8].copy_from_slice(b"NUWAXAR3");
    assert!(ArchiveReader::open(Cursor::new(bytes)).is_err());
}

#[test]
fn invalid_or_legacy_watermark_claims_do_not_change_writer() {
    let mut w = writer();
    assert!(w
        .record_source_watermark(Arc::new(LayerSource::Legacy), 1, 1)
        .is_err());
    assert!(w.record_source_watermark(lidar(), 1, 0).is_err());
    assert!(w
        .record_source_watermark(auxiliary("depth", "clock", 1, 1), 0, 0)
        .is_err());
    assert!(w
        .record_source_watermark(auxiliary("depth", " ", 1, 1), 1, 0)
        .is_err());
    assert!(!w.has_unpublished_changes());
    let mut root = (*w.snapshot(domain(), 1, 1).unwrap()).clone();
    let mark = SourceWatermark {
        source: lidar(),
        source_timestamp_ns: 1,
        source_sequence: 1,
    };
    root.source_watermarks = vec![mark.clone(), mark];
    assert!(root.validate().is_err());
}

#[test]
fn restore_decodes_uncacheable_layer_block_once() {
    let mut w = writer();
    w.observe(
        (0..32768_u16).map(|local| global_key([0, 0, 0], local).unwrap()),
        20,
    )
    .unwrap();
    w.replace_layer_observations(
        [0, 0, 0],
        [(0, layer(vec![sample(lidar(), 1, 20, 0)]))].into(),
    )
    .unwrap();
    let root = w.snapshot(domain(), 1, 20).unwrap();
    let repository = w.repository().clone();
    let mut restored = Writer::new(repository.clone(), 0.125, 4 * 1024 * 1024).unwrap();
    let reads = repository.counts().0;
    restored.restore(&root).unwrap();
    assert_eq!(repository.counts().0 - reads, 1);
}

#[test]
fn source_descriptor_accounting_counts_shared_allocations_once_per_layer() {
    let source = auxiliary(&"d".repeat(1024), "clock", 1, 1);
    let observed = layer(vec![
        sample(source.clone(), 0, 20, 0),
        sample(source.clone(), 0, 20, 1),
    ]);
    let baseline = 128 + 2 + 2 * 64;
    assert_eq!(
        terrain_observations::layer_bytes(&observed),
        baseline + source.resident_bytes()
    );
    let mut separately_allocated = observed.clone();
    separately_allocated.samples[1].source = Arc::new((*source).clone());
    assert_eq!(
        terrain_observations::layer_bytes(&separately_allocated),
        baseline
            + source.resident_bytes()
            + separately_allocated.samples[1].source.resident_bytes()
    );
    let mut w = writer();
    install(&mut w, observed);
    let before = w.resident_bytes();
    w.record_source_watermark(source.clone(), 20, 0).unwrap();
    assert!(w.resident_bytes() >= before + source.resident_bytes());
    let root = w.snapshot(domain(), 1, 20).unwrap();
    let mut reader = RegionReader::new(w.repository().clone(), 4 * 1024 * 1024);
    let decoded = reader.layers(&root.blocks[0]).unwrap();
    assert_eq!(
        reader.cached_bytes(),
        64 + 128 + terrain_observations::layer_bytes(&decoded[&0])
    );
}
