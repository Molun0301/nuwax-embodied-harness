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

use async_trait::async_trait;
use bytes::Bytes;
use futures::stream::Stream;
use nuwax_lcm::{LcmContext, LcmError, LcmReceivedMessage, LcmSubscription, PublishError};
use nuwax_protocol::{
    build_lcm_message_bytes, LcmMessageType, LcmTopic, RawLcmFrame, RawLcmFrameBuilder,
};

use crate::{LcmBackend, LcmFrameStream, TransportError, TransportResult};

/// An explicitly owned real-lcm subscription.
///
/// The handle is intentionally not [`Clone`]: one generation owner is
/// responsible for receive, acknowledged unsubscribe, and final drop.
/// Dropping without calling [`Self::unsubscribe`] retains the lower-level
/// fire-and-forget fallback behavior.
///
/// ```compile_fail
/// use nuwax_transport::OwnedLcmSubscription;
/// fn requires_clone<T: Clone>() {}
/// requires_clone::<OwnedLcmSubscription>();
/// ```
pub struct OwnedLcmSubscription {
    // Retain the owner context until acknowledged unsubscribe completes. The
    // transport/backend may be dropped before this generation owner shuts down.
    _context: Arc<LcmContext>,
    subscription: Option<LcmSubscription>,
    msg_type_filter: Option<LcmMessageType>,
    topic: LcmTopic,
}

impl std::fmt::Debug for OwnedLcmSubscription {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OwnedLcmSubscription")
            .field("topic", &self.topic)
            .field("msg_type_filter", &self.msg_type_filter)
            .field("active", &self.subscription.is_some())
            .finish()
    }
}

impl OwnedLcmSubscription {
    pub(crate) async fn subscribe(
        context: Arc<LcmContext>,
        topic: LcmTopic,
        msg_type_filter: Option<LcmMessageType>,
    ) -> TransportResult<Self> {
        let subscription_pattern = subscription_pattern(&topic, msg_type_filter.as_ref());
        let subscription = context
            .subscribe(&subscription_pattern)
            .await
            .map_err(|error| TransportError::LcmBackend(format!("subscribe 失败: {error}")))?;
        Ok(Self {
            _context: context,
            subscription: Some(subscription),
            msg_type_filter,
            topic,
        })
    }

    /// Receive and decode the next callback using the subscription's fixed
    /// topic and optional typed-suffix filter.
    ///
    /// Lagged broadcast entries are reported through tracing and skipped.
    pub async fn recv(&mut self) -> TransportResult<RawLcmFrame> {
        let subscription = self
            .subscription
            .as_mut()
            .ok_or(TransportError::ChannelClosed)?;
        loop {
            match subscription.receiver.recv().await {
                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    tracing::warn!(
                        "owned real-lcm subscribe lagged ({} 条) — channel={}",
                        n,
                        self.topic.channel()
                    );
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    return Err(TransportError::ChannelClosed);
                }
                Ok(message) => {
                    return decode_subscription_message(
                        message,
                        &self.topic,
                        self.msg_type_filter.as_ref(),
                    );
                }
            }
        }
    }

    /// 读取已有积压中的最新帧及跳过数量；仅供明确允许合并的状态/点云消费者使用。
    /// 消费范围在首帧到达后截取，持续生产不能让读取一直等待“队列清空”。
    pub async fn recv_latest(&mut self) -> TransportResult<(RawLcmFrame, u64)> {
        let first = self.recv().await?;
        let subscription = self.subscription.as_mut().ok_or(TransportError::ChannelClosed)?;
        let queued = subscription.receiver.len();
        let mut latest = None;
        let mut skipped = 0_u64;
        for _ in 0..queued {
            match subscription.receiver.try_recv() {
                Ok(message) => {
                    latest = Some(message);
                    skipped = skipped.saturating_add(1);
                }
                Err(tokio::sync::broadcast::error::TryRecvError::Lagged(count)) => {
                    skipped = skipped.saturating_add(count);
                }
                Err(tokio::sync::broadcast::error::TryRecvError::Empty) => break,
                Err(tokio::sync::broadcast::error::TryRecvError::Closed) => {
                    return Err(TransportError::ChannelClosed);
                }
            }
        }
        let frame = match latest {
            Some(message) => decode_subscription_message(message, &self.topic, self.msg_type_filter.as_ref())?,
            None => first,
        };
        Ok((frame, skipped))
    }

    /// Wait until the liblcm owner thread has processed this unsubscribe.
    ///
    /// This consumes the handle so queued callback data cannot be observed by
    /// the former owner after the acknowledgement.
    pub async fn unsubscribe(mut self) -> TransportResult<()> {
        let Some(mut subscription) = self.subscription.take() else {
            return Ok(());
        };
        subscription
            .unsubscribe()
            .await
            .map_err(|error| TransportError::LcmBackend(format!("unsubscribe 失败: {error}")))
    }
}

/// `real-lcm` backend。
///
/// The backend owns one `Arc<LcmContext>` shared by typed transport users and
/// raw protocols such as LCM RPC. The final `Arc` drop shuts down and joins the
/// single owner thread.
pub struct RealLcmBackend {
    /// Shared process-wide liblcm context owner.
    context: Arc<LcmContext>,
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
            TransportError::LcmBackend(format!("LcmContext::new({provider_url:?}) 失败: {e}"))
        })?;
        Ok(Self {
            context: Arc::new(ctx),
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
        let (typed_channel, payload) = encode_publish_frame(frame)
            .map_err(|error| TransportError::NotDispatched(Box::new(error)))?;
        self.context
            .publish_with_outcome(&typed_channel, payload)
            .await
            .map_err(single_publish_error)
    }

    async fn publish_batch(&self, frames: Vec<RawLcmFrame>) -> TransportResult<()> {
        let messages = frames
            .into_iter()
            .map(encode_publish_frame)
            .collect::<TransportResult<Vec<_>>>()?;
        self.context
            .publish_batch(messages)
            .await
            .map_err(|error| TransportError::LcmBackend(format!("lcm_publish batch 失败: {error}")))
    }

    async fn subscribe(&self, topic: LcmTopic, msg_type: Option<LcmMessageType>) -> LcmFrameStream {
        let subscription_pattern = subscription_pattern(&topic, msg_type.as_ref());
        let sub_result: Result<LcmSubscription, LcmError> =
            self.context.subscribe(&subscription_pattern).await;
        let sub = match sub_result {
            Ok(s) => s,
            Err(e) => {
                let err = TransportError::LcmBackend(format!("subscribe 失败: {e}"));
                return subscription_error_stream(err);
            }
        };

        // typed suffix 过滤:用 caller 提供的 msg_type.fqn() 过滤 callback payload
        let topic_clone = topic.clone();
        let s = subscription_to_stream(sub, msg_type, topic_clone);
        Box::pin(s)
    }

    fn name(&self) -> &str {
        "real-lcm"
    }

    fn shared_lcm_context(&self) -> Option<Arc<LcmContext>> {
        Some(Arc::clone(&self.context))
    }
}

fn single_publish_error(error: PublishError) -> TransportError {
    match error {
        PublishError::NotDispatched(cause) => TransportError::NotDispatched(Box::new(
            TransportError::LcmBackend(format!("lcm_publish 失败: {cause}")),
        )),
        PublishError::EffectUnconfirmed(cause) =>
            TransportError::LcmBackend(format!("lcm_publish 失败: {cause}")),
    }
}

fn encode_publish_frame(frame: RawLcmFrame) -> TransportResult<(String, Bytes)> {
    // 只补充生成消息的类型指纹；传输包头仍由原 liblcm 唯一生成。
    let message = build_lcm_message_bytes(
        frame.topic.channel(),
        frame.msg_type.fqn(),
        &frame.payload,
        u64::from(frame.header.seqno),
    )
    .map_err(|error| {
        TransportError::Protocol(nuwax_protocol::ProtocolError::LcmEncode(error.to_string()))
    })?;
    Ok((
        frame.topic.fqn(frame.msg_type.fqn()),
        Bytes::copy_from_slice(message.as_slice()),
    ))
}

fn subscription_pattern(topic: &LcmTopic, msg_type: Option<&LcmMessageType>) -> String {
    msg_type.map_or_else(|| topic.channel().to_owned(), |kind| topic.fqn(kind.fqn()))
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
    sub: LcmSubscription,
    msg_type_filter: Option<LcmMessageType>,
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
                        Ok(message) => {
                            let result = decode_subscription_message(
                                message,
                                &topic_for_fallback,
                                msg_type_filter.as_ref(),
                            );
                            return Some((result, (sub, msg_type_filter, topic_for_fallback)));
                        }
                    }
                }
            })
        },
    );
    Box::pin(stream)
}

fn decode_subscription_message(
    message: LcmReceivedMessage,
    expected_topic: &LcmTopic,
    expected_type: Option<&LcmMessageType>,
) -> TransportResult<RawLcmFrame> {
    let (actual_topic, actual_suffix) = LcmTopic::parse_subscription_pattern(&message.channel)
        .map_err(|e| TransportError::Topic(format!("invalid callback channel: {e}")))?;
    if actual_topic != *expected_topic {
        return Err(TransportError::Topic(format!(
            "callback channel mismatch: got {}, expected {}",
            actual_topic.channel(),
            expected_topic.channel()
        )));
    }

    let actual_type = actual_suffix
        .as_deref()
        .map(LcmMessageType::new)
        .transpose()
        .map_err(|e| TransportError::MessageType(e.to_string()))?;
    let message_type = match (expected_type, actual_type) {
        (Some(expected), Some(actual)) if expected == &actual => expected.clone(),
        (Some(expected), Some(actual)) => {
            return Err(TransportError::MessageType(format!(
                "callback type mismatch: got {}, expected {}",
                actual.fqn(),
                expected.fqn()
            )));
        }
        (Some(expected), None) => {
            return Err(TransportError::MessageType(format!(
                "typed subscription {} received untyped channel {}",
                expected.fqn(),
                message.channel
            )));
        }
        (None, Some(actual)) => actual,
        (None, None) => LcmMessageType::new("unknown.Unknown")
            .expect("static fallback message type must be valid"),
    };

    if message.payload.len() < 8 {
        return Err(TransportError::Protocol(
            nuwax_protocol::ProtocolError::Incomplete {
                need: 8,
                got: message.payload.len(),
            },
        ));
    }
    let expected_fingerprint = message_type.fingerprint();
    if expected_fingerprint == [0; 8] {
        return Err(TransportError::MessageType(format!(
            "no locked LCM schema fingerprint for {}",
            message_type.fqn()
        )));
    }
    if message.payload[..8] != expected_fingerprint {
        return Err(TransportError::Protocol(
            nuwax_protocol::ProtocolError::LcmDecode(format!(
                "fingerprint mismatch on {} for {}",
                message.channel,
                message_type.fqn()
            )),
        ));
    }

    Ok(
        RawLcmFrameBuilder::new(expected_topic.clone(), message_type)
            .payload(message.payload[8..].to_vec())
            .build(),
    )
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
    fn single_publish_mapping_uses_stage_not_shared_owner_down_variant() {
        let unsent = single_publish_error(PublishError::NotDispatched(
            LcmError::OwnerThreadDown { detail: "same diagnostic".into() },
        ));
        let unknown = single_publish_error(PublishError::EffectUnconfirmed(
            LcmError::OwnerThreadDown { detail: "same diagnostic".into() },
        ));
        assert!(unsent.is_definitely_not_dispatched());
        assert!(!unknown.is_definitely_not_dispatched());
        assert!(!single_publish_error(PublishError::EffectUnconfirmed(
            LcmError::Publish { channel: "/test/native-error".into() },
        )).is_definitely_not_dispatched());
    }

    #[test]
    fn name_is_real_lcm() {
        // 守恒测试：backend name 字符串必须为 "real-lcm"。
        fn check_name(b: &dyn LcmBackend) -> &str {
            b.name()
        }
        let real = crate::LcmTransport::new_real("udpm://239.255.76.67:7667?ttl=1")
            .expect("real-lcm backend 启动需 liblcm-dev 1.3~1.5");
        assert_eq!(check_name(&real), "real-lcm");
    }

    #[tokio::test]
    async fn real_lcm_batch_delivers_ordered_imu_before_cloud() {
        use nuwax_codec::lcm::schema;
        let port = std::net::UdpSocket::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let provider = format!("udpm://239.255.76.67:{port}?ttl=0");
        let transport = crate::LcmTransport::new_real(&provider).unwrap();
        let receiver = LcmContext::new(&provider).unwrap();
        let mut subscription = receiver.subscribe("/test/ordered-input/.*").await.unwrap();
        let imu_topic = LcmTopic::new("/test/ordered-input/imu").unwrap();
        let imu_type = LcmMessageType::new("sensor_msgs.Imu").unwrap();
        let cloud_topic = LcmTopic::new("/test/ordered-input/cloud").unwrap();
        let cloud_type = LcmMessageType::new("sensor_msgs.PointCloud2").unwrap();
        let mut frames = Vec::new();
        for index in 0..20 {
            let imu = schema::Imu {
                header: schema::Header {
                    seq: index,
                    stamp_sec: 1,
                    stamp_nsec: index * 1_000_000,
                    frame_id: "sensor".into(),
                },
                linear_acceleration: [0.0, 0.0, 9.81],
                ..schema::Imu::default()
            };
            frames.push(
                RawLcmFrameBuilder::new(imu_topic.clone(), imu_type.clone())
                    .payload(schema::encode_imu_body(&imu))
                    .build(),
            );
        }
        let cloud = schema::PointCloud2 {
            header: schema::Header {
                seq: 20,
                frame_id: "sensor".into(),
                ..schema::Header::default()
            },
            ..schema::PointCloud2::default()
        };
        frames.push(
            RawLcmFrameBuilder::new(cloud_topic.clone(), cloud_type.clone())
                .payload(schema::encode_point_cloud2_body(&cloud).unwrap())
                .build(),
        );
        let started = std::time::Instant::now();
        transport.publish_batch(frames).await.unwrap();
        let elapsed = started.elapsed();
        eprintln!("真实组播21条输入批量发布耗时：{elapsed:?}");
        assert!(
            elapsed < std::time::Duration::from_millis(500),
            "传输转发不可退回逐条确认：{elapsed:?}"
        );
        for index in 0..=20 {
            let message = tokio::time::timeout(
                std::time::Duration::from_secs(2),
                subscription.receiver.recv(),
            )
            .await
            .expect("有序批次接收超时")
            .unwrap();
            if index < 20 {
                assert_eq!(message.channel, imu_topic.fqn(imu_type.fqn()));
                let imu = schema::decode_imu_body(&message.payload[8..]).unwrap();
                assert_eq!(imu.header.seq, index);
            } else {
                assert_eq!(message.channel, cloud_topic.fqn(cloud_type.fqn()));
                assert_eq!(
                    &message.payload[8..],
                    schema::encode_point_cloud2_body(&cloud).unwrap()
                );
            }
        }
        receiver.shutdown().await.unwrap();
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

    // -----------------------------------------------------------------
    // codex rework-1 (2026-07-07): 验证 real_backend publish 路径产出的
    // wire payload **不**含 envelope(避免 §12.2 第 2 项 double-framing
    // 硬约束)。此 test 不真起 liblcm context;只走
    // `build_lcm_message_bytes` + `LcmMessageBytes::as_slice()` 这两段
    // 真实路径,断言最终喂给 `lcm_publish` 的 bytes == frame.payload。
    //
    // 2026-07-09 B3 修正:envelope 由 28 字节缩为 8 字节 preamble,但 real
    // backend publish 路径仍只送 payload 部分(double-framing 硬约束不变)。
    // -----------------------------------------------------------------
    #[test]
    fn real_backend_publish_payload_contains_schema_fingerprint_without_transport_envelope() {
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
        // 2026-07-09 B3 修正:`frame.header.seqno` 由 u64 收紧为 u32,
        // build_lcm_message_bytes 仍接 u64(进程内 metadata),这里显式 cast。
        let msg_bytes = build_lcm_message_bytes(
            frame.topic.channel(),
            frame.msg_type.fqn(),
            &frame.payload,
            u64::from(frame.header.seqno),
        )
        .expect("build_lcm_message_bytes must succeed for registered msg_type");

        let wire_payload = msg_bytes.as_slice();

        // lcm_publish receives the generated fingerprint and body, but not the
        // LC02 transport preamble (liblcm adds that itself).
        assert_eq!(
            wire_payload.len(),
            8 + user_payload.len(),
            "real backend lcm_publish payload must be fingerprint + body; got len={}, expect={}",
            wire_payload.len(),
            8 + user_payload.len(),
        );
        assert_eq!(&wire_payload[..8], &msg_bytes.fingerprint);
        assert_eq!(&wire_payload[8..], user_payload.as_slice());

        // 断言 2:对比例,RawLcmFrame::encode() 仍产出 preamble+payload(2026-07-09
        // B3 修正后是 8 字节 preamble + payload)。这是验证我们**没有**误用
        // frame.encode() 走 real backend publish 路径。
        let encoded = frame.encode();
        assert_eq!(
            encoded.len(),
            LcmFrameHeader::HEADER_BYTES + user_payload.len(),
            "RawLcmFrame::encode() 应拼 8 字节 preamble(magic BE 4 + seqno BE 4)"
        );
        assert_eq!(
            LcmFrameHeader::HEADER_BYTES,
            8,
            "preamble 必须为 8 字节(B3 修正:由 28 字节缩为 8 字节)"
        );
        assert_ne!(wire_payload, encoded.as_slice());
    }

    #[test]
    fn callback_validation_uses_actual_dot_typed_channel_and_fingerprint() {
        let topic = LcmTopic::new("/pose").unwrap();
        let msg_type = LcmMessageType::new("geometry_msgs/PoseStamped").unwrap();
        let mut encoded = msg_type.fingerprint().to_vec();
        encoded.extend_from_slice(b"generated-body");
        let frame = decode_subscription_message(
            LcmReceivedMessage {
                channel: "/pose#geometry_msgs.PoseStamped".into(),
                payload: Bytes::from(encoded),
            },
            &topic,
            Some(&msg_type),
        )
        .unwrap();
        assert_eq!(frame.msg_type.fqn(), "geometry_msgs.PoseStamped");
        assert_eq!(frame.payload, b"generated-body");
    }

    #[test]
    fn callback_validation_rejects_actual_type_or_fingerprint_mismatch() {
        let topic = LcmTopic::new("/pose").unwrap();
        let msg_type = LcmMessageType::new("geometry_msgs.PoseStamped").unwrap();
        let wrong_type = decode_subscription_message(
            LcmReceivedMessage {
                channel: "/pose#nav_msgs.Odometry".into(),
                payload: Bytes::from_static(&[0; 8]),
            },
            &topic,
            Some(&msg_type),
        );
        assert!(matches!(wrong_type, Err(TransportError::MessageType(_))));

        let wrong_fingerprint = decode_subscription_message(
            LcmReceivedMessage {
                channel: "/pose#geometry_msgs.PoseStamped".into(),
                payload: Bytes::from_static(&[0; 8]),
            },
            &topic,
            Some(&msg_type),
        );
        assert!(matches!(
            wrong_fingerprint,
            Err(TransportError::Protocol(_))
        ));
    }
}
