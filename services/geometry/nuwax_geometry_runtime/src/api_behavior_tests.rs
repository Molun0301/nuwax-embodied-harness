use super::*;
use crate::mapper::{Mapper, MapperConfig};
use std::time::Duration;

#[tokio::test]
async fn subscribe_map_updates_is_event_driven_and_carries_atomic_effective_and_physical_grids() {
    let mapper = Mapper::start(
        810,
        MapperConfig {
            map_name: "双栅格地图".into(),
            ..Default::default()
        },
    )
    .unwrap();
    let directory = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target/B2导航输出")
        .join(uuid::Uuid::new_v4().to_string());
    std::fs::create_dir_all(&directory).unwrap();
    let zones = directory.join("禁行区.json");
    std::fs::write(&zones, r#"{"zones":[{"id":"测试区域","shape":"rectangle","mode":"inside","vertices":[{"x":0.1,"y":0.1},{"x":0.9,"y":0.9}]}]}"#).unwrap();
    mapper.set_forbidden_zones(zones).await.unwrap();
    // SubscribeMap生产入口使用同一个present，只从建图拥有方接收真实修订。
    let mut stream = present(mapper.map_rx());
    let original = vec![0, 0, 255, 100, 0, 255];
    let grid = OccupancyGrid {
        map_name: "双栅格地图".into(),
        meta: Some(OccupancyGridMeta {
            frame_id: "map".into(),
            width: 3,
            height: 2,
            resolution: 1.0,
            timestamp_ns: 1_000_000_000,
            ..Default::default()
        }),
        data: original.clone(),
    };
    mapper.feed_grid(810, grid.clone());
    mapper.snapshot().await.unwrap();
    let first = tokio::time::timeout(Duration::from_secs(2), stream.next())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert_eq!(first.raw_data, original);
    assert_eq!(first.data[0], 100);
    assert_ne!(first.data, first.raw_data);
    assert_eq!(first.meta, grid.meta);
    assert_eq!(first.data, mapper.occupancy(true).unwrap().data);
    assert_eq!(first.raw_data, mapper.occupancy(false).unwrap().data);
    let mut next = grid;
    next.meta.as_mut().unwrap().timestamp_ns += 1_000_000_000;
    next.data[4] = 100;
    mapper.feed_grid(810, next);
    mapper.snapshot().await.unwrap();
    let second = tokio::time::timeout(Duration::from_secs(2), stream.next())
        .await
        .unwrap()
        .unwrap()
        .unwrap();
    assert_eq!(second.raw_data[4], 100);
    assert_eq!(second.data[4], 100);
    assert_eq!(second.raw_data[2], 255);
    assert_eq!(second.data[2], 255);
    assert_eq!(second.meta.as_ref().unwrap().timestamp_ns, 2_000_000_000);
    mapper.shutdown().await.unwrap();
    std::fs::remove_dir_all(directory).unwrap();
}
