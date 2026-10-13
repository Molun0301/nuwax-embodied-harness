use super::*;
use std::io::{Read, Seek, SeekFrom};
fn domain() -> Domain {
    Domain {
        session: "observed".into(),
        generation: 1,
        frame_generation: 1,
        producer_generation: 1,
    }
}
fn layer(z: f64, stamp: u64) -> LayerObservation {
    LayerObservation {
        members: vec![0],
        plane_origin: [0.0625, 0.0625, z],
        normal: [0.0, 0.0, 1.0],
        samples: vec![LayerSample {
            point: [0.04, 0.06, z],
            source_sequence: stamp,
            source_timestamp_ns: stamp,
            point_index: 0,
        }],
    }
}
fn writer() -> Writer {
    Writer::new(Repository::temporary().unwrap(), 0.125, 4 * 1024 * 1024).unwrap()
}
fn update(w: &mut Writer, layer: LayerObservation) {
    w.observe([[0, 0, 0]], layer.latest_timestamp_ns()).unwrap();
    w.replace_layer_observations([0, 0, 0], [(0, layer)].into())
        .unwrap();
}
#[test]
fn observed_block_roundtrip_and_legacy_has_no_fabricated_samples() {
    let block = [(
        0,
        Evidence {
            observed_ns: 1,
            hits: 1,
        },
    )]
    .into();
    let layers = [(0, layer(0.08, 1))].into();
    let bytes = encode_layer_block(&block, &layers).unwrap();
    assert!(bytes.starts_with(b"NUWAXBL2"));
    assert_eq!(decode_block(&bytes).unwrap(), block);
    assert_eq!(decode_layer_observations(&bytes).unwrap(), layers);
    assert_eq!(block_summary(&bytes).unwrap(), (1, 1));
    assert!(decode_layer_observations(&encode_block(&block).unwrap())
        .unwrap()
        .is_empty());
}
#[test]
fn layer_surface_change_and_metadata_change_have_separate_revisions() {
    let mut w = writer();
    update(&mut w, layer(0.08, 1));
    let first = w.snapshot(domain(), 1, 1).unwrap();
    assert_eq!(first.version, 3);
    let mut diagnostic = layer(0.08, 2);
    diagnostic.samples[0].point = [0.05, 0.06, 0.081];
    update(&mut w, diagnostic);
    let second = w.snapshot(domain(), 2, 2).unwrap();
    assert_eq!(second.geometry_revision, 1);
    assert_eq!(second.blocks[0].geometry_revision, 1);
    assert_eq!(second.blocks[0].block_revision, 2);
    update(&mut w, layer(0.09, 3));
    let third = w.snapshot(domain(), 3, 3).unwrap();
    assert_eq!(third.geometry_revision, 3);
    assert_eq!(third.blocks[0].geometry_revision, 3);
    assert_eq!(third.blocks[0].points, first.blocks[0].points);
    w.replace_layer_observations([0, 0, 0], LayerObservations::new())
        .unwrap();
    let fourth = w.snapshot(domain(), 4, 3).unwrap();
    assert_eq!(fourth.geometry_revision, 4);
}
#[test]
fn ar3_restore_preserves_bounded_returns_and_old_root_is_not_upgraded_to_statistics() {
    let mut w = writer();
    update(&mut w, layer(0.08, 1));
    let root = w.snapshot(domain(), 1, 1).unwrap();
    let (mut file, _) = archive_file(&root, w.repository()).unwrap();
    let mut magic = [0; 8];
    file.read_exact(&mut magic).unwrap();
    assert_eq!(&magic, b"NUWAXAR3");
    file.seek(SeekFrom::Start(0)).unwrap();
    let repository = Repository::temporary().unwrap();
    let mut archive = ArchiveReader::open(file).unwrap();
    let archived = archive.import(&repository).unwrap();
    let mut restored = Writer::new(repository, 0.125, 4 * 1024 * 1024).unwrap();
    restored.restore_archived(&archived).unwrap();
    assert_eq!(
        restored.layer_observations([0, 0, 0]).unwrap()[&0],
        layer(0.08, 1)
    );
    restored.observe([[0, 0, 0]], 2).unwrap();
    let next = restored.snapshot(domain(), 2, 2).unwrap();
    assert_eq!(next.geometry_revision, 1);
    let old_block = encode_block(
        &[(
            0,
            Evidence {
                observed_ns: 0,
                hits: 0,
            },
        )]
        .into(),
    )
    .unwrap();
    assert!(decode_layer_observations(&old_block).unwrap().is_empty());
}
#[test]
fn receiver_checks_observed_geometry_progression_and_spatial_binding() {
    let mut w = writer();
    update(&mut w, layer(0.08, 1));
    let first = w.snapshot(domain(), 1, 1).unwrap();
    let mut recv = Receiver::new(Repository::temporary().unwrap());
    let mut receive = |root: &Manifest| {
        recv.receive(
            &Transfer {
                root: root.encode().unwrap(),
                included: BTreeMap::new(),
            },
            &domain(),
            |_, b| w.repository().read_object(b.object, MAX_BLOCK_BYTES),
        )
    };
    receive(&first).unwrap();
    drop(receive);
    update(&mut w, layer(0.09, 2));
    let second = w.snapshot(domain(), 2, 2).unwrap();
    let mut forged = (*second).clone();
    forged.geometry_revision = 1;
    forged.blocks[0].geometry_revision = 1;
    assert!(recv
        .receive(
            &Transfer {
                root: forged.encode().unwrap(),
                included: BTreeMap::new()
            },
            &domain(),
            |_, b| w.repository().read_object(b.object, MAX_BLOCK_BYTES)
        )
        .is_err());
    let mut wrong = (*second).clone();
    wrong.blocks[0].key = [1, 0, 0];
    assert!(recv
        .receive(
            &Transfer {
                root: wrong.encode().unwrap(),
                included: BTreeMap::new()
            },
            &domain(),
            |_, b| w.repository().read_object(b.object, MAX_BLOCK_BYTES)
        )
        .is_err());
    let mut wrong = (*second).clone();
    wrong.version = 2;
    assert!(recv
        .receive(
            &Transfer {
                root: wrong.encode().unwrap(),
                included: BTreeMap::new()
            },
            &domain(),
            |_, b| w.repository().read_object(b.object, MAX_BLOCK_BYTES)
        )
        .is_err());
    recv.receive(
        &Transfer {
            root: second.encode().unwrap(),
            included: BTreeMap::new(),
        },
        &domain(),
        |_, b| w.repository().read_object(b.object, MAX_BLOCK_BYTES),
    )
    .unwrap();
    assert_eq!(recv.layers(&second.blocks[0]).unwrap()[&0], layer(0.09, 2));
}
#[test]
fn invalid_layer_population_cannot_change_writer_or_mix_columns() {
    let mut w = writer();
    update(&mut w, layer(0.08, 1));
    for bad in 0..5 {
        let mut l = layer(0.08, 1);
        match bad {
            0 => l.samples.push(l.samples[0].clone()),
            1 => l.members.push(1),
            2 => l.normal = [f64::NAN, 0.0, 0.0],
            3 => l.samples[0].point[0] = f64::INFINITY,
            _ => l.samples[0].source_timestamp_ns = 2,
        }
        assert!(w
            .replace_layer_observations([0, 0, 0], [(0, l)].into())
            .is_err());
        assert_eq!(w.layer_observations([0, 0, 0]).unwrap()[&0], layer(0.08, 1));
    }
    w.remove([[0, 0, 0]]).unwrap();
    assert!(w.layer_observations([0, 0, 0]).unwrap().is_empty());
}
