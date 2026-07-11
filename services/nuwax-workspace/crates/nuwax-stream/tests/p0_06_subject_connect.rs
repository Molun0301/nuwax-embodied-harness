//! P0-06 Subject::connect 合流测试
//!
//! rework22 验证把多个 source Subject 合并到一个新 Subject 后,
//! 订阅新 Subject 能同时收到所有 source 的消息。

use futures::StreamExt;

#[tokio::test]
async fn p0_06_subject_connect_merges_two_sources() {
    let s1 = nuwax_stream::Subject::<u32>::new(16);
    let s2 = nuwax_stream::Subject::<u32>::new(16);
    // 给每个 source 保留一个 active receiver 防止 push 报 NoSubscribers
    let mut rx1 = s1.subscribe_internal();
    let mut rx2 = s2.subscribe_internal();
    let _keep1 = tokio::spawn(async move {
        while rx1.recv().await.is_ok() {}
    });
    let _keep2 = tokio::spawn(async move {
        while rx2.recv().await.is_ok() {}
    });

    let merged = nuwax_stream::Subject::connect(vec![s1.clone(), s2.clone()], 64);
    let mut merged_rx = merged.subscribe();

    // 给 spawn 的合流任务一点点时间注册内部订阅
    tokio::time::sleep(std::time::Duration::from_millis(50)).await;

    s1.push(1).unwrap();
    s2.push(2).unwrap();

    let mut got = vec![];
    for _ in 0..2 {
        let v = tokio::time::timeout(std::time::Duration::from_millis(500), merged_rx.next())
            .await
            .ok()
            .and_then(|r| r)
            .and_then(|inner| inner.ok());
        if let Some(v) = v {
            got.push(v);
        }
    }
    got.sort();
    assert_eq!(got, vec![1, 2]);
}

#[tokio::test]
async fn p0_06_subject_connect_single_source_passthrough() {
    let s = nuwax_stream::Subject::<&'static str>::new(8);
    // connect 内部会 subscribe_internal,无需外部预留 receiver
    let merged = nuwax_stream::Subject::connect(vec![s.clone()], 16);

    // merged 自身也需一个活跃 receiver 才能让 connect 内部 forward 不报 NoSubscribers
    let mut merged_rx = merged.subscribe();
    tokio::time::sleep(std::time::Duration::from_millis(50)).await;

    s.push("hello").unwrap();

    let v = tokio::time::timeout(std::time::Duration::from_millis(500), merged_rx.next())
        .await
        .ok()
        .and_then(|r| r)
        .and_then(|inner| inner.ok());
    assert_eq!(v, Some("hello"));
}

#[tokio::test]
async fn p0_06_subject_clone_produces_same_broadcast() {
    // Subject<T>::Clone 必须共享同一组订阅者(底层 broadcast::Sender::clone)
    let s1 = nuwax_stream::Subject::<u32>::new(8);
    let s2 = s1.clone();
    let mut rx = s1.subscribe();
    s2.push(7).unwrap();
    let v = rx.next().await.unwrap().unwrap();
    assert_eq!(v, 7);
}

