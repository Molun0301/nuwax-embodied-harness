//! P0-06 Out<T> broadcast 模式测试
//!
//! rework22 验证两个独立订阅者都能收到同一条 broadcast 消息。

#[tokio::test]
async fn p0_06_out_broadcast_two_subscribers_both_receive() {
    let (out, mut rx1) = nuwax_core::Out::<u32>::broadcast(16);
    let mut rx2 = rx1.resubscribe();
    out.send(42).unwrap();
    assert_eq!(rx1.recv().await.unwrap(), 42);
    assert_eq!(rx2.recv().await.unwrap(), 42);
}

#[tokio::test]
async fn p0_06_out_broadcast_sender_gone_errors() {
    let (out, rx) = nuwax_core::Out::<u32>::broadcast(4);
    drop(rx);
    let err = out.send(1).unwrap_err();
    assert!(matches!(err, nuwax_core::OutError::ReceiverGone));
}

#[tokio::test]
async fn p0_06_out_single_sender_gone_errors() {
    let (out, inp) = nuwax_core::channel::<u32>(1);
    drop(inp);
    let err = out.send(1).unwrap_err();
    assert!(matches!(err, nuwax_core::OutError::ReceiverGone));
}
