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
    assert!(!store.save("源图", None, first.clone()).unwrap());
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
    assert_eq!(loaded.occupancy, first.occupancy);
    assert_eq!(loaded.backend_checkpoint, first.backend_checkpoint);
    assert_eq!(loaded.cloud.display, first.cloud.display);
    assert!(loaded.cloud.fine_archive.is_some());
    assert!(loaded.cloud.dense.is_some());
    assert_eq!(loaded.scans.len(), 1);
    let restored = Mapper::load(32, MapperConfig::default(), loaded, true).unwrap();
    assert!(restored.surface_map().await.unwrap().is_some());
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
