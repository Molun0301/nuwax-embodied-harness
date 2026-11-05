use super::*;
use crate::{
    mapper::tests::room_frame,
    mapper::{Mapper, MapperConfig},
};
struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}
fn directory() -> Directory {
    let value = Directory(
        std::env::temp_dir().join(format!("geometry-map-store-{}", uuid::Uuid::new_v4())),
    );
    fs::create_dir(&value.0).unwrap();
    value
}
#[tokio::test]
async fn map_directory_round_trip_and_atomic_replacement_follow_source_name() {
    let directory = directory();
    let store = MapStore::new(directory.0.clone());
    let mapper = Mapper::start(
        31,
        MapperConfig {
            map_name: "源图".into(),
            ..Default::default()
        },
    )
    .unwrap();
    mapper.feed(room_frame(31, 1_000_000_000));
    let first = mapper.snapshot().await.unwrap();
    assert_eq!(first.planar_resume_unavailable_reason, None);
    assert!(!store.save("源图", None, first.clone()).unwrap());
    assert_eq!(
        store.planar_resume_unavailable_reason("源图").unwrap(),
        None
    );
    let descriptor: serde_json::Value =
        serde_json::from_slice(&fs::read(directory.0.join("源图/geometry/map.json")).unwrap())
            .unwrap();
    assert!(descriptor.get("planar_resume_unavailable_reason").is_none());
    fs::write(directory.0.join("源图/meta.json"), "网关元信息").unwrap();
    assert_eq!(
        store.save("源图", None, first.clone()).unwrap_err().code(),
        tonic::Code::AlreadyExists
    );
    assert!(store.save("源图", Some("源图"), first.clone()).unwrap());
    assert_eq!(
        fs::read_to_string(directory.0.join("源图/meta.json")).unwrap(),
        "网关元信息"
    );
    assert!(!store.save("另存图", Some("源图"), first.clone()).unwrap());
    assert!(store.exists("源图") && store.exists("另存图"));
    let loaded = store.load("源图").unwrap();
    assert_eq!(loaded.planar_resume_unavailable_reason, None);
    assert_eq!(loaded.occupancy, first.occupancy);
    assert_eq!(loaded.backend_checkpoint, first.backend_checkpoint);
    assert_eq!(loaded.cloud.display, first.cloud.display);
    assert!(loaded.cloud.fine_archive.is_some());
    assert!(loaded.cloud.dense.is_some());
    assert_eq!(loaded.scans.len(), 1);
    let restored = Mapper::load(32, MapperConfig::default(), loaded, true).unwrap();
    assert!(restored.surface_map().await.unwrap().is_some());
    assert_eq!(
        restored
            .snapshot()
            .await
            .unwrap()
            .planar_resume_unavailable_reason,
        None
    );
    restored.shutdown().await.unwrap();
    mapper.shutdown().await.unwrap();
    assert!(fs::read_dir(directory.0.join("源图"))
        .unwrap()
        .all(|entry| !entry
            .unwrap()
            .file_name()
            .to_string_lossy()
            .starts_with(".geometry-")));
}
#[tokio::test]
async fn missing_and_corrupt_map_files_have_contract_status_codes() {
    let directory = directory();
    let store = MapStore::new(directory.0.clone());
    assert_eq!(
        store.load("缺失").err().unwrap().code(),
        tonic::Code::NotFound
    );
    let mapper = Mapper::start(33, MapperConfig::default()).unwrap();
    mapper.feed(room_frame(33, 1_000_000_000));
    let snapshot = mapper.snapshot().await.unwrap();
    assert!(
        mapper.error_rx().borrow().is_none(),
        "{:?}",
        *mapper.error_rx().borrow()
    );
    store.save("损坏图", None, snapshot).unwrap();
    fs::write(
        directory.0.join("损坏图/geometry/occupancy.pb"),
        [255, 255, 255],
    )
    .unwrap();
    let status = store.load("损坏图").err().unwrap();
    assert_eq!(status.code(), tonic::Code::DataLoss);
    assert!(status.message().contains("occupancy.pb"));
    mapper.shutdown().await.unwrap();
}

#[tokio::test]
async fn fault_snapshot_preserves_real_map_and_planar_resume_reason_across_reload() {
    const REASON: &str = "无完整二维 SLAM 图，不能用于二维续建";
    let directory = directory();
    let store = MapStore::new(directory.0.clone());
    let mapper = Mapper::start(34, MapperConfig::default()).unwrap();
    mapper.feed(room_frame(34, 1_000_000_000));
    let mut snapshot = mapper.snapshot().await.unwrap();
    assert!(snapshot.occupancy.is_some());
    assert!(!snapshot.cloud.display.is_empty());
    assert!(snapshot.planar_checkpoint.is_none());
    store.save("故障图", None, snapshot.clone()).unwrap();
    fs::write(directory.0.join("故障图/meta.json"), "网关元信息").unwrap();
    snapshot.planar_resume_unavailable_reason = Some(REASON.into());
    assert_eq!(
        store
            .save("故障图", None, snapshot.clone())
            .unwrap_err()
            .code(),
        tonic::Code::AlreadyExists
    );
    assert_eq!(
        store.planar_resume_unavailable_reason("故障图").unwrap(),
        None
    );
    assert!(store
        .save("故障图", Some("故障图"), snapshot.clone())
        .unwrap());
    assert_eq!(
        fs::read_to_string(directory.0.join("故障图/meta.json")).unwrap(),
        "网关元信息"
    );
    assert_eq!(
        store
            .planar_resume_unavailable_reason("故障图")
            .unwrap()
            .as_deref(),
        Some(REASON)
    );
    let descriptor: serde_json::Value =
        serde_json::from_slice(&fs::read(directory.0.join("故障图/geometry/map.json")).unwrap())
            .unwrap();
    assert_eq!(descriptor["planar_resume_unavailable_reason"], REASON);
    assert_eq!(descriptor["planar"], false);
    assert!(!directory.0.join("故障图/geometry/planar.bin").exists());
    let loaded = store.load("故障图").unwrap();
    assert_eq!(
        loaded.planar_resume_unavailable_reason.as_deref(),
        Some(REASON)
    );
    assert_eq!(loaded.occupancy, snapshot.occupancy);
    assert_eq!(loaded.cloud.display, snapshot.cloud.display);
    let restored = Mapper::load(35, MapperConfig::default(), loaded, true).unwrap();
    let recovered = restored.snapshot().await.unwrap();
    assert_eq!(
        recovered.planar_resume_unavailable_reason.as_deref(),
        Some(REASON)
    );
    assert_eq!(recovered.occupancy, snapshot.occupancy);
    assert_eq!(recovered.cloud.display, snapshot.cloud.display);
    store.save("另存故障图", Some("故障图"), recovered).unwrap();
    let prepared = Mapper::prepare_load(
        36,
        MapperConfig::default(),
        store.load("另存故障图").unwrap(),
        true,
    )
    .unwrap();
    let prepared_snapshot = prepared.snapshot().await.unwrap();
    assert_eq!(
        prepared_snapshot
            .planar_resume_unavailable_reason
            .as_deref(),
        Some(REASON)
    );
    assert_eq!(prepared_snapshot.occupancy, snapshot.occupancy);
    assert_eq!(prepared_snapshot.cloud.display, snapshot.cloud.display);
    prepared.shutdown().await.unwrap();
    restored.shutdown().await.unwrap();
    mapper.shutdown().await.unwrap();
    assert!(fs::read_dir(directory.0.join("故障图"))
        .unwrap()
        .all(|entry| !entry
            .unwrap()
            .file_name()
            .to_string_lossy()
            .starts_with(".geometry-")));
}
