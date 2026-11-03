use super::*;
use std::io::{Read, Seek, SeekFrom};


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

fn writer() -> Writer {
    Writer::new(Repository::temporary().unwrap(), 0.125, 4 * 1024 * 1024).unwrap()
}

#[test]
fn source_watermarks_survive_ar4_restore_without_cross_clock_cutoffs() {
    let source = auxiliary("depth", "device-clock", 4, 8);
    let same_numeric_ids = auxiliary("rear-depth", "device-B", 4, 8);
    let mut w = writer();
    w.observe([[0, 0, 0]], 20).unwrap();
    w.record_source_watermark(source.clone(), 2, 0).unwrap();
    w.record_source_watermark(lidar(), 20, 9).unwrap();
    w.record_source_watermark(same_numeric_ids, 20, 9).unwrap();
    let root = w.snapshot(domain(), 1, 20).unwrap();
    assert_eq!(root.version, 4);
    assert_eq!(root.source_watermarks.len(), 3);
    let (mut file, _) = archive_file(&root, w.repository()).unwrap();
    // 按需写出互操作证据，产物不进入仓库。
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
fn invalid_watermark_claims_do_not_change_writer() {
    let mut w = writer();
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
