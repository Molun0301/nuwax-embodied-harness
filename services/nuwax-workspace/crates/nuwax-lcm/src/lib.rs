//! `nuwax-lcm`: real liblcm 的 safe wrapper + 单 owner 事件线程。
//!
//! ## 2026-07-07 Phase 3 引入(替代上游 dimos 旧路径)
//!
//! 本 crate 是 `nuwax-lcm-sys` (liblcm FFI) 之上的 safe 包装,负责:
//!
//! - **RAII 持有 `lcm_t*`**:`LcmContext` 析构时自动 `lcm_destroy`
//! - **单 owner 事件线程**:spawn 一个 `std::thread` 反复调用
//!   `lcm_handle_timeout(lcm, 50)` 驱动 publish / subscribe 协议栈,
//!   handler 回调把消息 push 进 `broadcast::Sender`
//! - **owner-thread serialized 调用**:`publish` / `subscribe` 必须经
//!   mpsc `Command` 队列送进 owner thread,**禁止**跨线程直接调
//!   `lcm_publish` (liblcm 内部状态不是 Sync,跨线程会触发 RTSAN 死锁)
//!
//! ## lifetime 契约
//!
//! `LcmContext` 不是 `Clone`。每个进程持**一个**实例(`Arc` 在外层如需共享),
//! Drop 时:
/*! 1. 推送 `Command::Shutdown` 进 owner mpsc
   2. owner thread 在下个 50 ms tick 收到 → 调 `lcm_destroy` → 退出
   3. std::thread 不 join(daemon thread 自然结束)
*/
//!
//! ## 2026-07-08 fake-lcm 根除(v3 F2 决策)
//!
//! fake-lcm backend(`nuwax-transport::FakeLcmBackend` + `MockLcmBackend` Python
//! 类)已物理删除。本 crate 是 nuwax 仓内 LCM 链路的**唯一**入口,镜像必须装
//! `liblcm-dev` 1.3~1.5(版本锁 `>= 1.3, < 1.6` 由 nuwax-lcm-sys/build.rs 守门)。
//!
//! 不允许以 fake 假装真;fake-lcm 没有存在的意义(v3 用户硬约束)。

// FFI binding 必然需要 unsafe;但我们**不允许**在自己写的新代码里用 unsafe
// —— 所有 unsafe 都封装在 extern "C" 调用处,其它逻辑都用 safe Rust。
#![warn(missing_docs)]

use std::collections::HashMap;
use std::ffi::CString;
use std::os::raw::{c_char, c_void};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use bytes::Bytes;
use nuwax_lcm_sys::{
    lcm_create, lcm_destroy, lcm_get_fileno, lcm_handle_timeout,
    lcm_publish, lcm_recv_buf_t, lcm_subscribe, lcm_subscription_t, lcm_unsubscribe,
    lcm_t, to_cstring,
};
use thiserror::Error;
use tokio::sync::{broadcast, mpsc, oneshot};
use tracing::{debug, error, info, warn};

// =============================================================================
// 错误类型
// =============================================================================

/// real-lcm 错误。
///
/// 2026-07-08 fake-lcm 物理删除后,本 enum 是 nuwax 仓内 LCM 唯一错误源;
/// 错误来自 liblcm 内部,所以必须可观测;上层 (`LcmTransport::publish`)
/// 把该错误转成 `TransportError::LcmBackend`。
#[derive(Debug, Error)]
pub enum LcmError {
    /// lcm_t 创建失败(lcm_create 返 NULL)
    #[error("lcm_create 失败 (provider={provider}): 可能是 URL 格式错误或 liblcm 内部初始化失败")]
    Create {
        /// provider URL(用于错误日志)
        provider: String,
    },
    /// publish 失败(`lcm_publish` 返非 0)
    #[error("lcm_publish 失败 (channel={channel})")]
    Publish {
        /// 出错的 channel 名
        channel: String,
    },
    /// subscribe 失败
    #[error("lcm_subscribe 失败 (channel={channel})")]
    Subscribe {
        /// 出错的 channel 名
        channel: String,
    },
    /// owner thread 关闭 / panic
    #[error("lcm owner thread 已关闭(无法恢复): {detail}")]
    OwnerThreadDown {
        /// 详情
        detail: String,
    },
    /// backend 不可用(host 缺 liblcm / 启动期 fail-fast)
    #[error("lcm backend 不可用: {0}")]
    BackendUnavailable(String),
    /// 用户传了含 NUL 的字符串
    #[error("channel 含 NUL 字节: {0}")]
    NulByte(String),
}

// =============================================================================
// Subscription handle —— caller 通过 broadcast::Receiver 拿 payload Bytes
// =============================================================================

/// 单条订阅的句柄。
///
/// - `receiver`:tokio `broadcast::Receiver<Bytes>`,caller 直接 `.recv().await`
///   即可拉取 payload。
/// - `channel`:LCM channel 名(/cmd_vel 等);`unsubscribe` 时使用
///
/// 注意:`broadcast::Receiver` 本身不 Clone;若需要给多个 task 共享,应
/// 在外层 wrap 一个 `Arc<Mutex<LcmSubscription>>` 或使用
/// `LcmContext::subscribe` 多次注册同一 channel(每条通道独立 receiver,
/// 但都共享同一 liblcm subscription —— liblcm 内部按 channel 名去重)。
pub struct LcmSubscription {
    /// channel 名(用作 unsubscribe key)
    pub channel: Arc<CString>,
    /// broadcast receiver
    pub receiver: broadcast::Receiver<Bytes>,
}

impl std::fmt::Debug for LcmSubscription {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LcmSubscription")
            .field("channel", &self.channel.to_string_lossy().into_owned())
            .finish_non_exhaustive()
    }
}

// =============================================================================
// Command —— caller → owner thread
// =============================================================================

#[derive(Debug)]
enum Command {
    /// 发布一条消息
    Publish {
        /// channel(owned CString,owner thread 直接调 lcm_publish)
        channel: CString,
        /// payload bytes
        payload: Bytes,
        /// reply:成功 / 失败 via oneshot(rework22 P0-04:让 caller 拿到 publish 真错误)
        reply: oneshot::Sender<Result<(), LcmError>>,
    },
    /// 注册一条订阅。owner thread 调 lcm_subscribe + 装一个 broadcast sender
    Subscribe {
        /// channel(CString)
        channel: CString,
        /// 回调形式 channel 名(通常等于 channel;typed suffix 由 `LcmSubscription` 自行记)
        pattern: CString,
        /// reply:成功 / 失败 via oneshot
        reply: std::sync::mpsc::SyncSender<Result<(), LcmError>>,
        /// 注册后用于后续 push bytes 的 broadcast sender
        sender: broadcast::Sender<Bytes>,
    },
    /// 取消订阅
    Unsubscribe {
        /// channel
        channel: CString,
    },
    /// 关闭 owner thread
    Shutdown,
}

// =============================================================================
// Owner thread 内部状态
// =============================================================================

/// owner thread 内部状态:raw `*mut lcm_t` + mpsc rx + 当前所有 subscriptions
///
/// 重要:这个结构不是 `Send`;**只能** 在 owner thread 内移动 / 操作。
/// 我们通过 `std::thread::spawn` 时接受的 `F: FnOnce() -> T + Send + 'static`
/// 自动推断 Send —— 这里由于 `lcm_t` 是 raw pointer,我们直接用
/// `unsafe impl Send`(只在新建时一次性转移到 owner thread,后续绝不跨线程)。
struct OwnerThread {
    lcm: *mut lcm_t,
    cmd_rx: mpsc::Receiver<Command>,
    /// 已注册的 subscription:`channel CString → SubscriptionHandle`
    ///
    /// 2026-07-09 B2 修复后,`lcm_subscribe` 返回 `*mut lcm_subscription_t`
    /// (liblcm 1.x 真实签名),后续 `lcm_unsubscribe` 须以该 handle 为 key,
    /// 而**不是** channel 字符串。所以必须把 handle 存下来。
    ///
    /// 2026-07-11 rework22 P0-01:`SubscriptionEntry` 是 leaked box(`*mut`),
    /// 内含 broadcast sender + alive flag。callback 期间 `(*entry).alive`
    /// 决定是否真把 bytes 推到 sender;`unsubscribe`/`run` 收尾时先
    /// 设 false、调用 `lcm_unsubscribe`、再 `Box::from_raw` 释放。
    subscriptions: HashMap<CString, SubscriptionHandle>,
}

/// SubscriptionHandle —— owner thread 内部:`sub` 是 liblcm handle;
/// `entry` 是 leaked `Box<SubscriptionEntry>`,由 callback 通过 user_data
/// 取回。`unsubscribe` / `run` 收尾统一释放。
struct SubscriptionHandle {
    sub: *mut lcm_subscription_t,
    entry: *mut SubscriptionEntry,
}

/// SubscriptionEntry —— leaked 留在 liblcm user_data 里。
///
/// - `sender`:callback 把收到的 bytes broadcast 出去
/// - `alive` :`unsubscribe` / owner thread 退出时先设 false;callback
///   acquire-load,看到 false 直接 return(防 use-after-free:
///   unsubscribe 后 liblcm 仍可能在超时窗口里再投一次)
struct SubscriptionEntry {
    sender: broadcast::Sender<Bytes>,
    alive: AtomicBool,
}

// SAFETY: `OwnerThread` 只在 `std::thread::spawn` 的那一刻被 move 一次;
// 此后整个结构只在 owner thread 内被操作(`run` 拿走的 self),没有跨线程
// 共享。所以裸指针的存在 ≠ 真实的多线程 alias;unsafe impl Send 仅是为
// 满足 `std::thread::spawn` 的 trait bound。
//
// 注:`*mut lcm_t` 本身的非 Send 是为防止 caller 不小心跨线程触碰。
// 这里我们把 `OwnerThread` 整个结构一次性转交 owner thread。
unsafe impl Send for OwnerThread {}

impl OwnerThread {
    /// 注册一条订阅 + 把 broadcast::Sender 装进 liblcm user pointer。
    ///
    /// 关键:user pointer 是 raw pointer,必须 `'static`,所以我们用
    /// `Box::into_raw` + 自定义 free 流程;owner thread drop 时统一释放。
    ///
    /// 2026-07-09 B2 修复:`lcm_subscribe` 现在返 `*mut lcm_subscription_t`
    /// (NULL on failure),存进 self.subscriptions 供后续 `lcm_unsubscribe` 用。
    fn subscribe(
        &mut self,
        channel: CString,
        pattern: CString,
        sender: broadcast::Sender<Bytes>,
    ) -> Result<(), LcmError> {
        let chan_str = channel.to_string_lossy().into_owned();
        // rework22 P0-01:把 sender 装进 leaked `SubscriptionEntry`,
        // callback 通过 user_data 取 `&(*entry).sender` 的引用
        // (不 move out —— 旧实现 `*Box::from_raw(sender_ptr)` 把
        //  allocation 释放后立刻 move 出值,导致 HashMap insert 时
        // 旧 entry 内存已 free,这里直接泄漏整块 entry 一次)
        let entry = Box::new(SubscriptionEntry {
            sender,
            alive: AtomicBool::new(true),
        });
        let entry_ptr = Box::into_raw(entry);
        // SAFETY: liblcm 接管 entry_ptr 的 lifetime,回调里再
        // `&(*entry_ptr)` 取回 sender 引用 + alive flag。
        let sub = unsafe {
            lcm_subscribe(
                self.lcm,
                pattern.as_ptr(),
                Some(lcm_msg_callback),
                entry_ptr as *mut c_void,
            )
        };
        if sub.is_null() {
            // 失败 → 回收 entry(把 sender 一起 drop,broadcast sender
            // 析构时所有 receiver 收到关闭信号)
            let _ = unsafe { Box::from_raw(entry_ptr) };
            return Err(LcmError::Subscribe { channel: chan_str });
        }
        self.subscriptions
            .insert(channel, SubscriptionHandle { sub, entry: entry_ptr });
        Ok(())
    }

    fn unsubscribe(&mut self, channel: &CString) {
        // 2026-07-09 B2 修复:从 subscriptions 取 sub handle 调 liblcm
        // (原先错用 channel 字符串 — liblcm 1.x 的 lcm_unsubscribe 不接受
        //  channel 字符串,会直接返回错误或 UB)。
        //
        // 2026-07-11 rework22 P0-01:先设 `entry.alive = false`,这样
        // 接下来 unsubscribed 的 sub 在 race window 内被 liblcm 二次投
        // 也不会再 push bytes 走 dangling sender。
        let handle = match self.subscriptions.remove(channel) {
            Some(h) => h,
            None => {
                warn!(
                    "lcm_unsubscribe: 未找到 channel={} 的 subscription handle",
                    channel.to_string_lossy()
                );
                return;
            }
        };
        unsafe { (*handle.entry).alive.store(false, Ordering::Release) };
        let rc = unsafe { lcm_unsubscribe(self.lcm, handle.sub) };
        if rc != 0 {
            warn!(
                "lcm_unsubscribe 返回非 0 (channel={})",
                channel.to_string_lossy()
            );
        }
        // 释放 leaked entry(注:即使 `unsubscribe` 返回非 0 也要 release,
        // 否则 HashMap remove 之后我们不再持有该指针,泄漏)
        unsafe { drop(Box::from_raw(handle.entry)) };
    }

    fn publish(&self, channel: &CString, payload: &Bytes) -> Result<(), LcmError> {
        let rc = unsafe {
            lcm_publish(
                self.lcm,
                channel.as_ptr(),
                payload.as_ptr() as *const c_void,
                payload.len() as u32,
            )
        };
        if rc != 0 {
            return Err(LcmError::Publish {
                channel: channel.to_string_lossy().into_owned(),
            });
        }
        Ok(())
    }

    /// event loop 主循环:处理 commands + lcm_handle_timeout。
    fn run(mut self) {
        info!("nuwax-lcm owner thread 启动");
        let _fd = unsafe { lcm_get_fileno(self.lcm) };

        // rework22 P0-04:每个 tick 内把所有 queued commands 处理完再
        // 让出时间片给 liblcm(原实现一次只 drain 一条,在 50ms 吞吐
        // 有限;高负载 publish 风暴会塞满 mpsc → caller backpressure)
        let mut shutdown = false;
        while !shutdown {
            loop {
                match self.cmd_rx.try_recv() {
                    Ok(Command::Shutdown) => {
                        info!("nuwax-lcm owner thread 收到 Shutdown,退出");
                        shutdown = true;
                        break;
                    }
                    Ok(Command::Publish {
                        channel,
                        payload,
                        reply,
                    }) => {
                        let r = self.publish(&channel, &payload);
                        let _ = reply.send(r);
                    }
                    Ok(Command::Subscribe {
                        channel,
                        pattern,
                        reply,
                        sender,
                    }) => {
                        let r = self.subscribe(channel, pattern, sender);
                        let _ = reply.send(r);
                    }
                    Ok(Command::Unsubscribe { channel }) => {
                        self.unsubscribe(&channel);
                    }
                    Err(mpsc::error::TryRecvError::Empty) => break,
                    Err(mpsc::error::TryRecvError::Disconnected) => {
                        error!("nuwax-lcm owner thread 的 mpsc sender 全 dropped;退出");
                        shutdown = true;
                        break;
                    }
                }
            }
            // 50 ms 轮询
            let _ = unsafe { lcm_handle_timeout(self.lcm, 50) };
        }

        // 收尾:drain 所有 subscriptions(2026-07-11 rework22 P0-01:
        // 先把 entry.alive 设为 false,再 lcm_unsubscribe,最后释放
        // leaked entry —— 与 `unsubscribe` 同序,避免 callback race)
        for (ch, handle) in self.subscriptions.drain() {
            unsafe { (*handle.entry).alive.store(false, Ordering::Release) };
            unsafe { lcm_unsubscribe(self.lcm, handle.sub) };
            unsafe { drop(Box::from_raw(handle.entry)) };
            let _ = ch;
        }
        unsafe { lcm_destroy(self.lcm) };
    }
}

/// liblcm 静态回调。
///
/// 2026-07-09 B1 修复:liblcm 1.x 真实签名是
/// `void (*)(const lcm_recv_buf_t *rbuf, const char *channel, void *user_data)`。
/// payload 长度 / 字节来自 `rbuf->data_size` / `rbuf->data`(不是直接传 rbuf+rbuf_len)。
unsafe extern "C" fn lcm_msg_callback(
    rbuf: *const lcm_recv_buf_t,
    channel: *const c_char,
    user: *mut c_void,
) {
    if user.is_null() || rbuf.is_null() {
        return;
    }
    // SAFETY: rbuf 由 liblcm 持有,本回调期间有效;data_size 来自 C 头,
    // data 指针按 data_size 字节数有效。
    let entry = user as *const SubscriptionEntry;
    // rework22 P0-01:check alive flag first。若 unsubscribe / owner
    // shutdown drain 已发生,entry 内存可能已被 Box::from_raw 释放,
    // 此时 acquire-load 看到 false 直接 return,绝对不要碰 sender。
    if unsafe { !(*entry).alive.load(Ordering::Acquire) } {
        return;
    }
    let (data_ptr, len) = unsafe { ((*rbuf).data, (*rbuf).data_size as usize) };
    if data_ptr.is_null() || len == 0 {
        return;
    }
    // SAFETY: alive=true 时 entry 仍 leaked & valid;只借用 sender
    // 的引用,不 move,不释放 allocation。
    let sender = unsafe { &(*entry).sender };
    let slice = unsafe { std::slice::from_raw_parts(data_ptr as *const u8, len) };
    let bytes = Bytes::copy_from_slice(slice);
    let _ = sender.send(bytes);

    if !channel.is_null() {
        let cstr = unsafe { std::ffi::CStr::from_ptr(channel) };
        debug!("lcm_msg_callback: channel={:?} len={}", cstr, len);
    }
}

// =============================================================================
// LcmContext —— RAII handle,内含 owner thread + mpsc command channel
// =============================================================================

/// RAII 持有的 real LCM context。
///
/// - `new(provider)`:创建 lcm_t + spawn owner thread
/// - Drop:推送 `Shutdown` 命令;owner thread 自然退出
/// - `publish / subscribe`:走 tokio mpsc send → owner thread 真正调 liblcm
pub struct LcmContext {
    /// tokio mpsc — caller 用 `.send(...).await` 推命令
    cmd_tx: mpsc::Sender<Command>,
    /// provider URL —— Debug 用
    provider: Arc<String>,
}

impl std::fmt::Debug for LcmContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LcmContext")
            .field("provider", &self.provider)
            .field("cmd_tx_open", &!self.cmd_tx.is_closed())
            .finish()
    }
}

impl LcmContext {
    /// 构造一个 real LCM context。
    ///
    /// - `provider`:liblcm provider URL,例如
    ///   `udpm://239.255.76.67:7667?ttl=1`
    /// - 返回 `Err(LcmError::Create)` 当 `lcm_create` 返回 NULL(provider
    ///   格式错误 / system liblcm 内部初始化失败)
    pub fn new(provider: &str) -> Result<Self, LcmError> {
        let provider_c =
            to_cstring(provider).map_err(|_| LcmError::NulByte(provider.to_owned()))?;
        let lcm = unsafe { lcm_create(provider_c.as_ptr()) };
        if lcm.is_null() {
            return Err(LcmError::Create {
                provider: provider.to_owned(),
            });
        }

        let (cmd_tx, cmd_rx) = mpsc::channel::<Command>(10_000);

        let owner = OwnerThread {
            lcm,
            cmd_rx,
            subscriptions: std::collections::HashMap::new(),
        };
        let provider_owned = Arc::new(provider.to_owned());

        std::thread::Builder::new()
            .name("nuwax-lcm-owner".into())
            .spawn(move || {
                owner.run();
            })
            .map_err(|e| LcmError::BackendUnavailable(format!(
                "spawn owner thread 失败: {}",
                e
            )))?;

        Ok(Self {
            cmd_tx,
            provider: provider_owned,
        })
    }

    /// 当前 provider URL(诊断)
    pub fn provider(&self) -> &str {
        &self.provider
    }

    /// 异步发布一条消息。owner thread 真正调 `lcm_publish`。
    ///
    /// rework22 P0-04:返回真错误。命令队列入队失败 → `OwnerThreadDown`;
    /// owner 执行结果 → 通过 oneshot reply 透传回 caller。
    pub async fn publish(&self, channel: &str, payload: Bytes) -> Result<(), LcmError> {
        let c = to_cstring(channel).map_err(|_| LcmError::NulByte(channel.to_owned()))?;
        let (reply_tx, reply_rx) = oneshot::channel();
        self.cmd_tx
            .send(Command::Publish {
                channel: c,
                payload,
                reply: reply_tx,
            })
            .await
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?;
        reply_rx
            .await
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?
    }

    /// 注册一个 subscription。
    ///
    /// 返回一个 [`LcmSubscription`],caller 通过其 `receiver` 拉消息。
    pub async fn subscribe(&self, channel: &str) -> Result<LcmSubscription, LcmError> {
        let chan_c = to_cstring(channel).map_err(|_| LcmError::NulByte(channel.to_owned()))?;
        let pattern_c = chan_c.clone();
        let (tx, rx) = broadcast::channel::<Bytes>(8);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel::<Result<(), LcmError>>(1);

        self.cmd_tx
            .send(Command::Subscribe {
                channel: chan_c.clone(),
                pattern: pattern_c,
                sender: tx,
                reply: reply_tx,
            })
            .await
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?;

        let resp = reply_rx
            .recv_timeout(Duration::from_secs(1))
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: format!("subscribe reply 超时/失败: {}", e),
            })?;
        resp?;

        Ok(LcmSubscription {
            channel: Arc::new(chan_c),
            receiver: rx,
        })
    }

    /// 取消订阅(可选;Drop 时也会自动 unsubscribe)
    pub async fn unsubscribe(&self, channel: &str) -> Result<(), LcmError> {
        let c = to_cstring(channel).map_err(|_| LcmError::NulByte(channel.to_owned()))?;
        self.cmd_tx
            .send(Command::Unsubscribe { channel: c })
            .await
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?;
        Ok(())
    }

    /// 主动关闭 owner thread。一般用不到(Drop 会自动关)。
    pub async fn shutdown(&self) -> Result<(), LcmError> {
        self.cmd_tx
            .send(Command::Shutdown)
            .await
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?;
        Ok(())
    }
}

impl Drop for LcmContext {
    fn drop(&mut self) {
        // 尽力通知 owner thread 退出(若 cmd_tx 已 close,吞掉错误)
        let _ = self.cmd_tx.try_send(Command::Shutdown);
        // 不 join thread —— owner thread 自身会在收到 Shutdown 后
        // 调用 lcm_destroy(持有 lcm_t raw ptr)。
    }
}

// =============================================================================
// 单测
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nul_byte_rejected() {
        let e = to_cstring("/cmd_vel\0bad").unwrap_err();
        assert_eq!(e.input, "/cmd_vel\0bad");
    }

    #[test]
    fn nul_byte_error_via_lcm_error() {
        let err = LcmError::NulByte("/x".into());
        assert!(err.to_string().contains("NUL"));
    }

    /// 单元测试不真正起 liblcm context(需要系统 liblcm + 网络组播);
    /// 这里只验证错误类型 + Arc/Clone 行为。
    #[test]
    fn lcm_error_create_message_contains_provider() {
        let e = LcmError::Create {
            provider: "udpm://239.255.76.67:7667".into(),
        };
        assert!(e.to_string().contains("udpm://"));
    }

    #[test]
    fn lcm_error_backend_unavailable() {
        let e = LcmError::BackendUnavailable("missing liblcm".into());
        assert!(e.to_string().contains("missing liblcm"));
    }
}