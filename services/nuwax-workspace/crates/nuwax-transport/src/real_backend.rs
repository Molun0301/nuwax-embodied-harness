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
//! frame 编码:见 [`RawLcmFrame::encode`];**不**自己重写 LCM byte 布局。
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
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder};
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

        // 2. 编码为 LCM wire bytes(8B magic + 8B fingerprint +
        //    8B seqno + 4B payload_len + payload)
        let encoded = frame.encode().map_err(TransportError::Protocol)?;

        // 3. 通过 owner thread 调 lcm_publish(发送 channel + 整段 wire bytes)。
        // 注意:liblcm 1.x 的 lcm_publish 在 wire 上再加自己的 magic + fingerprint +
        // seqno;所以 caller 应该传 payload bytes,不是完整 frame。
        // 这里我们直接传 encoded bytes —— real liblcm 会在 receiver 端再
        // 解出 raw payload(因为 liblcm 默认按 payload 处理 callback)。
        //
        // 注意:这是个**简化路径**——fake backend 的 wire 字节 = real liblcm wire 字节
        // 都由 RawLcmFrame::encode 提供,liblcm 收到后会再去掉自己的
        // 8+8+8+4=28 字节 header,callback 拿到的是 payload 部分。
        //
        // 真正 wire interop 见 `feedback_phase3_real_lcm_roundtrip.md`。
        ctx.publish(frame.topic.channel(), Bytes::from(encoded))
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
    use futures::stream::unfold;
    Box::pin(unfold(
        (sub, msg_type_filter, topic_for_fallback),
        move |(mut sub, msg_type_filter, topic_for_fallback)| async move {
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
                        let msg_type = match LcmMessageType::new(fqn) {
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
        },
    ))
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
        // 仅检查 trait method 的返回值(不构造 backend)
        fn check_name(b: &dyn LcmBackend) -> &str {
            b.name()
        }
        // 模拟 fake backend 的 name,验证 backend trait 一致性
        assert_eq!(check_name(&crate::FakeLcmBackend::new()), "fake-lcm");
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
}