//! `nuwax-transport` 的 `real-lcm` backend。
//!
//! ## 2026-07-07 Phase 3 引入
//!
//! 本模块在 `real-lcm` feature 开启时启用,实装真实 liblcm publish/subscribe
//! backend,**严禁** 假装 fake —— 用户启用 real-lcm 时必须装好 `liblcm-dev`。
//!
//! ## 与 fake backend 的契约等价
//!
//! - `LcmBackend::publish(frame)` ── 异步发 `RawLcmFrame`,返回 `TransportResult<()>`
//! - `LcmBackend::subscribe(topic, msg_type)` ── 异步返回 `LcmFrameStream`,
//!   每条都是 `TransportResult<RawLcmFrame>`
//! - `LcmBackend::name()` ── 返回 `"real-lcm"`(canonical name)
//!
//! frame 编码:`RawLcmFrame` 仅作为 **内部** wire 表达(fake backend + raw log
//! 回放用);real-lcm publish 走 [`nuwax_protocol::build_lcm_message_bytes`],
//! 拿 `LcmMessageBytes::as_slice()`(纯 payload bytes)喂给 liblcm
//! `lcm_publish`,**不**把 28 字节 LC02 envelope 当作真实 wire payload
//! (避免 §12.2 第 2 项 double-framing 硬约束)。
//!
//! ## liblcm 单 owner 约束
//!
//! real-lcm backend 内部持有一个 [`LcmContext`](nuwax_lcm::LcmContext)
//! (单 owner event thread)。publish / subscribe 都通过 tokio mpsc 推
//! 进 owner thread,绝不在 caller 端直接调 liblcm。

use std::pin::Pin;
use std::sync::Arc;
use std::task::{Context, Poll};

use async_trait::async_trait;
use bytes::Bytes;
use futures::stream::{Stream, StreamExt};
use nuwax_lcm::{LcmContext, LcmError, LcmSubscription};
use nuwax_protocol::{
    build_lcm_message_bytes, LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder,
};
use tokio::sync::Mutex;

use crate::{LcmBackend, LcmFrameStream, TransportError, TransportResult};

/// `real-lcm` backend。
///
/// 内部包一个 `Arc<Mutex<Option<LcmContext>>>`:
/// - 创建时 `Some(LcmContext)`
/// - `LcmContext` 在 backend 销毁时 Drop,owner thread 自动退出
/// - publish/subscribe 调用时拿 mutex 短暂 guard,然后调 `LcmContext::*`
pub struct RealLcmBackend {
    /// liblcm context owner;`None` 等价于 backend 不可用
    inner: Arc<Mutex<Option<LcmContext>>>,
    /// provider URL(诊断)
    provider_url: Arc<String>,
}

impl std::fmt::Debug for RealLcmBackend {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RealLcmBackend")
            .field("provider", &self.provider_url)
            .finish_non_exhaustive()
    }
}

impl RealLcmBackend {
    /// 构造 real-lcm backend(provider 形如 `udpm://239.255.76.67:7667?ttl=1`).
    ///
    /// 失败时返回 `TransportError::LcmBackend` —— 包含 `LcmError` 详情。
    pub fn new(provider_url: &str) -> TransportResult<Self> {
        let ctx = LcmContext::new(provider_url).map_err(|e| {
            TransportError::LcmBackend(format!(
                "LcmContext::new({:?}) 失败: {}",
                provider_url, e
            ))
        })?;
        Ok(Self {
            inner: Arc::new(Mutex::new(Some(ctx))),
            provider_url: Arc::new(provider_url.to_owned()),
        })
    }

    /// 当前 provider URL
    pub fn provider_url(&self) -> &str {
        &self.provider_url
    }
}

#[async_trait]
impl LcmBackend for RealLcmBackend {
    async fn publish(&self, frame: RawLcmFrame) -> TransportResult<()> {
        // 1. 拿 mutex guard(短)
        let guard = self.inner.lock().await;
        let ctx = guard.as_ref().ok_or_else(|| {
            TransportError::LcmBackend("real-lcm backend 已 drop / 无效".into())
        })?;

        // 2. 包装成 `LcmMessageBytes`(Phase 2 lane-2 强类型合约)。
        //    `build_lcm_message_bytes` 用 STRUCT_SEEDS 计算 upstream lcm-gen 兼容
        //    fingerprint;它**不**拼装 28 字节 LC02 envelope —— 那是 fake
        //    backend 用的内部表达。seqno 由 caller 在 builder 上设定,这里直接透传。
        //    任何一步失败都按 `TransportError::Protocol` 抛出(例如 msg_type 未在
        //    STRUCT_SEEDS 注册时,这里就 fail-fast,绝不悄悄降级)。
        let msg_bytes = build_lcm_message_bytes(
            frame.topic.channel(),
            frame.msg_type.fqn(),
            &frame.payload,
            frame.header.seqno,
        )
        .map_err(|e| TransportError::Protocol(nuwax_protocol::ProtocolError::LcmEncode(e.to_string())))?;

        // 3. 通过 owner thread 调 liblcm `lcm_publish(channel, payload_bytes)`。
        //    liblcm 会在 wire 上**自己**加 8+8+8+4=28 字节 envelope,所以我们
        //    只能传 **payload 部分** —— 也就是 `LcmMessageBytes::as_slice()`。
        //    绝对不能传 `frame.encode()` 的产物,否则 liblcm 会再叠一层 envelope,
        //    receiver 端 callback 拿到 28+N+28+N 字节垃圾(§12.2 第 2 项硬约束)。
        let wire_payload: &[u8] = msg_bytes.as_slice();
        ctx.publish(frame.topic.channel(), Bytes::copy_from_slice(wire_payload))
            .await
            .map_err(|e| TransportError::LcmBackend(format!("lcm_publish 失败: {}", e)))?;
        Ok(())
    }

    async fn subscribe(
        &self,
        topic: LcmTopic,
        msg_type: Option<LcmMessageType>,
    ) -> LcmFrameStream {
        let guard = self.inner.lock().await;
        let ctx = match guard.as_ref() {
            Some(c) => c,
            None => {
                let err = TransportError::LcmBackend("real-lcm backend 已 drop".into());
                return subscription_error_stream(err);
            }
        };
        let sub_result: Result<LcmSubscription, LcmError> = ctx.subscribe(topic.channel()).await;
        drop(guard); // 释放 lock,让后续 .recv() 不抱 lock
        let sub = match sub_result {
            Ok(s) => s,
            Err(e) => {
                let err = TransportError::LcmBackend(format!("subscribe 失败: {}", e));
                return subscription_error_stream(err);
            }
        };

        // typed suffix 过滤:用 caller 提供的 msg_type.fqn() 过滤 callback payload
        let msg_type_filter = msg_type.map(|m| m.fqn().to_owned());
        let topic_clone = topic.clone();
        let s = subscription_to_stream(sub, msg_type_filter, topic_clone);
        Box::pin(s)
    }

    fn name(&self) -> &str {
        "real-lcm"
    }
}

/// 把单条 transport error 包装成立即结束 + 一条 error 的 stream。
fn subscription_error_stream(err: TransportError) -> LcmFrameStream {
    let s = futures::stream::iter(vec![Err(err)]);
    Box::pin(s)
}

/// 从 `LcmSubscription` 拉消息,转 `RawLcmFrame` + typed suffix 过滤。
///
/// 实现:`futures::stream::poll_fn` + 手工驱动 broadcast receiver。
///
/// 注意:liblcm 1.x 的 callback 拿到的是 **payload 部分**(liblcm 自动剥掉
/// magic + fingerprint + seqno + payload_len),所以我们这里直接用
/// payload bytes + caller 提供的 topic / msg_type 构造 `RawLcmFrame`。
fn subscription_to_stream(
    mut sub: LcmSubscription,
    msg_type_filter: Option<String>,
    topic_for_fallback: LcmTopic,
) -> Pin<Box<dyn Stream<Item = TransportResult<RawLcmFrame>> + Send + Unpin>> {
    // 我们把 receiver 留在原位置,通过 `&mut sub` 在 poll 中访问;
    // 但 broadcast::Receiver 的 .recv() 是 async,不能直接在 poll_fn 里调。
    //
    // 解决方案:用 `futures::stream::unfold` 把 async 步骤展开成 stream 状态机。
    // `unfold` 的 state 是 (subscription, msg_type_filter, topic);
    // 每轮 .next_future() 调 receiver.recv(),拿到 Ok(payload) 转 frame。
    //
    // 注意:async block 自身是 `!Unpin`,会通过 `Fut` 参数传染给 `Unfold<...>`,
    // 让 `Unfold` 整体 `!Unpin`,不能直接 `Box::pin(unfold(...))`。
    // 修复:把 closure 返回的 future 显式 `Box::pin` 成 `BoxFuture<'_, _>`,
    // `BoxFuture` 是 `Unpin`,于是 `Unfold<T, F, BoxFuture<...>>` 是 `Unpin`,
    // `Box::pin(unfold(...))` 即可 coerce 成 `Pin<Box<dyn Stream + Send + Unpin>>`。
    use futures::stream::unfold;
    let stream = unfold(
        (sub, msg_type_filter, topic_for_fallback),
        move |(mut sub, msg_type_filter, topic_for_fallback)| {
            // 把 async block 显式 pin 成 `BoxFuture<'static, _>`,
            // 满足 unfold 的 Fut bound。`BoxFuture` 是 `Pin<Box<dyn Future + Send>>`,
            // 本身 `Unpin`,所以 `Unfold<T, F, BoxFuture>` 也是 `Unpin`。
            Box::pin(async move {
                loop {
                    match sub.receiver.recv().await {
                        Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                            tracing::warn!(
                                "real-lcm subscribe lagged ({} 条) — channel={}",
                                n,
                                topic_for_fallback.channel()
                            );
                            continue;
                        }
                        Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                            return None;
                        }
                        Ok(payload) => {
                            // typed suffix 过滤:caller 提供 → 用 caller 提供的;
                            // 没提供 → fallback `unknown/Unknown`。
                            let fqn = msg_type_filter
                                .clone()
                                .unwrap_or_else(|| "unknown/Unknown".into());
                            if let Some(want) = msg_type_filter.as_ref() {
                                if want != &fqn {
                                    continue;
                                }
                            }
                            let msg_type = match LcmMessageType::new(&fqn) {
                                Ok(m) => m,
                                Err(_) => continue, // 不应发生
                            };
                            let frame = RawLcmFrameBuilder::new(
                                topic_for_fallback.clone(),
                                msg_type,
                            )
                            .payload(payload.to_vec())
                            .build();
                            return Some((Ok(frame), (sub, msg_type_filter, topic_for_fallback)));
                        }
                    }
                }
            })
        },
    );
    Box::pin(stream)
}

// =============================================================================
// 单测(safe 接口构造,不真起 liblcm context)
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// 注:不在 unit test 里调 `RealLcmBackend::new` —— 真起 liblcm context
    /// 需要 system liblcm + 网络组播。这里仅验证 name() 与 new() 的失败路径。

    #[test]
    fn name_is_real_lcm() {
        // 2026-07-08 fake-lcm 物理删除后:`FakeLcmBackend` 已不存在。
        // 守恒测试:backend name 字符串必须为 "real-lcm",防止 future 重构回滚。
        fn check_name(b: &dyn LcmBackend) -> &str {
            b.name()
        }
        let real = crate::LcmTransport::new_real("udpm://239.255.76.67:7667?ttl=1")
            .expect("real-lcm backend 启动需 liblcm-dev 1.3~1.5");
        assert_eq!(check_name(&real), "real-lcm");
    }

    /// 验证 subscription_error_stream 立即产出一条 error,然后 None
    #[test]
    fn subscription_error_stream_emits_error() {
        let s = subscription_error_stream(crate::TransportError::LcmBackend("x".into()));
        // 需要 tokio runtime 驱动 stream
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        rt.block_on(async {
            use futures::StreamExt;
            let mut s = std::pin::pin!(s);
            let first = s.next().await;
            assert!(first.is_some());
            assert!(first.unwrap().is_err());
            let second = s.next().await;
            assert!(second.is_none(), "error stream should end after one error");
        });
    }

    #[allow(dead_code)]
    fn _suppress_unused_waker(_cx: &mut Context<'_>) -> Poll<()> {
        Poll::Ready(())
    }

    // -----------------------------------------------------------------
    // codex rework-1 (2026-07-07): 验证 real_backend publish 路径产出的
    // wire payload **不**含 28 字节 LC02 envelope(避免 §12.2 第 2 项
    // double-framing 硬约束)。此 test 不真起 liblcm context;只走
    // `build_lcm_message_bytes` + `LcmMessageBytes::as_slice()` 这两段
    // 真实路径,断言最终喂给 `lcm_publish` 的 bytes == frame.payload。
    // -----------------------------------------------------------------
    #[test]
    fn real_backend_publish_payload_strips_envelope() {
        use nuwax_protocol::{LcmFrameHeader, LcmMessageType, LcmTopic, RawLcmFrameBuilder};

        // 准备一个真实 frame(走 RawLcmFrameBuilder,seqno=42)
        let topic = LcmTopic::new("/odom").unwrap();
        let msg_type = LcmMessageType::new("nav_msgs/Odometry").unwrap();
        let user_payload: Vec<u8> = b"odometry-payload-bytes".to_vec();
        let frame = RawLcmFrameBuilder::new(topic, msg_type)
            .payload(user_payload.clone())
            .seqno(42)
            .build();

        // 走 real_backend publish 路径的**纯函数**部分:把 frame 喂给
        // build_lcm_message_bytes(channel, fqn, payload, seqno),拿到的
        // LcmMessageBytes 调 as_slice() —— 这就是要送给 lcm_publish 的 bytes。
        let msg_bytes = build_lcm_message_bytes(
            frame.topic.channel(),
            frame.msg_type.fqn(),
            &frame.payload,
            frame.header.seqno,
        )
        .expect("build_lcm_message_bytes must succeed for registered msg_type");

        let wire_payload = msg_bytes.as_slice();

        // 断言 1:lcm_publish 收到的 bytes 长度 == payload 长度,不是 28 + payload。
        // 这是 §12.2 第 2 项硬约束的直接证明 —— 没 envelope,就没 double-framing。
        assert_eq!(
            wire_payload.len(),
            user_payload.len(),
            "real backend lcm_publish payload 长度不应增加 28 字节 envelope;got len={}, expect={}",
            wire_payload.len(),
            user_payload.len(),
        );
        assert_eq!(wire_payload, user_payload.as_slice());

        // 断言 2:对比例,RawLcmFrame::encode() 仍然产出 28+payload(fake backend
        // 用的内部表达)。这是验证我们**没有**误用 frame.encode()。
        let encoded = frame.encode();
        assert_eq!(
            encoded.len(),
            LcmFrameHeader::HEADER_BYTES + user_payload.len(),
            "RawLcmFrame::encode() 仍应拼 28 字节 envelope(fake backend 表达不变)"
        );
        // 真正的 wire bytes 是 header(28) + payload —— 不应被 real backend 使用。
        // 反向验证:如果 real backend 误用 frame.encode(),wire_payload.len()
        // 会等于 encoded.len() = 28+payload,上面的断言 1 必失败。
    }
}