//! `nuwax-lcm`: real liblcm 的 safe wrapper + 单 owner 事件线程。
//!
//! ## 2026-07-07 Phase 3 引入(替代上游 dimos 旧路径)
//!
//! 本 crate 是 `nuwax-lcm-sys` (liblcm FFI) 之上的 safe 包装,负责:
//!
//! - **RAII 持有 `lcm_t*`**:`LcmContext` 析构时自动 `lcm_destroy`
//! - **单 owner 事件线程**: 同时等待通信描述符与命令唤醒管道，
//!   收到实际数据后以非阻塞 `lcm_handle_timeout(lcm, 0)` 驱动接收，
//!   handler 回调把消息 push 进 `broadcast::Sender`
//! - **owner-thread serialized 调用**:`publish` / `subscribe` 必须经
//!   mpsc `Command` 队列送进 owner thread,**禁止**跨线程直接调
//!   `lcm_publish` (liblcm 内部状态不是 Sync,跨线程会触发 RTSAN 死锁)
//!
//! ## lifetime 契约
//!
//! `LcmContext` 不是 `Clone`。每个进程持**一个**实例(`Arc` 在外层如需共享),
//! Drop 时:
/*! 1. 通过无界 control channel 推送 `Command::Shutdown`
   2. owner thread 调 `lcm_destroy` 并释放 callback storage
   3. 析构方 join owner thread，确保进程内不残留 LCM 生命周期
*/
//!
//! 本 crate 是 nuwax 仓内 LCM 链路的唯一入口。镜像必须安装
//! `liblcm-dev` 1.3~1.5（版本锁 `>= 1.3, < 1.6` 由 build.rs 守门）。

// FFI binding 必然需要 unsafe;但我们**不允许**在自己写的新代码里用 unsafe
// —— 所有 unsafe 都封装在 extern "C" 调用处,其它逻辑都用 safe Rust。
#![warn(missing_docs)]

use std::collections::HashMap;
use std::ffi::CString;
use std::io::{self, Read, Write};
use std::os::fd::AsRawFd;
use std::os::raw::{c_char, c_void};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use bytes::Bytes;
use nuwax_lcm_sys::{
    lcm_create, lcm_destroy, lcm_get_fileno, lcm_handle_timeout, lcm_publish, lcm_recv_buf_t,
    lcm_subscribe, lcm_subscription_set_queue_capacity, lcm_subscription_t, lcm_t, lcm_unsubscribe,
    to_cstring,
};
use thiserror::Error;
use tokio::sync::{broadcast, mpsc, oneshot};
use tracing::{debug, error, info, warn};

/// Capacity used by the caller-to-owner command queue.
pub const LCM_COMMAND_QUEUE_CAPACITY: usize = 10_000;
/// Capacity used by both liblcm and the Rust subscription broadcast queue.
pub const LCM_SUBSCRIPTION_QUEUE_CAPACITY: usize = 10_000;

const MAX_COMMANDS_PER_TICK: usize = 64;
const MAX_RECEIVES_PER_TICK: usize = 64;

/// 仅通知命令已经入队；命令内容和顺序仍由原有队列拥有。
#[derive(Debug)]
struct CommandWakeup {
    reader: UnixStream,
    writer: UnixStream,
}

impl CommandWakeup {
    fn new() -> io::Result<Self> {
        let (reader, writer) = UnixStream::pair()?;
        reader.set_nonblocking(true)?;
        writer.set_nonblocking(true)?;
        Ok(Self { reader, writer })
    }

    fn notify(&self) -> io::Result<()> {
        let mut writer = &self.writer;
        loop {
            match writer.write(&[1]) {
                Ok(_) => return Ok(()),
                // 管道已满意味着通知仍可读，不需要为每条命令保存独立通知。
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => return Ok(()),
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(error) => return Err(error),
            }
        }
    }

    fn consume(&self) -> io::Result<()> {
        let mut reader = &self.reader;
        let mut notifications = [0_u8; 256];
        loop {
            match reader.read(&mut notifications) {
                Ok(0) => {
                    return Err(io::Error::new(
                        io::ErrorKind::BrokenPipe,
                        "命令唤醒管道已关闭",
                    ))
                }
                Ok(_) => return Ok(()),
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => return Ok(()),
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(error) => return Err(error),
            }
        }
    }

    fn wake_owner(&self) -> Result<(), LcmError> {
        self.notify().map_err(|error| LcmError::OwnerThreadDown {
            detail: format!("命令已提交但无法唤醒通信拥有线程：{error}"),
        })
    }
}

// =============================================================================
// 错误类型
// =============================================================================

/// real-lcm 错误。
///
/// 本 enum 是 nuwax 仓内 LCM 唯一错误源；
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
    /// 批量发布中某条消息失败；其后的消息未发送。
    #[error("LCM batch publish failed at sample {index}: {source}")]
    PublishBatch {
        /// 失败消息在批次内的零基序号。
        index: usize,
        /// 底层发布错误。
        source: Box<LcmError>,
    },
    /// subscribe 失败
    #[error("lcm_subscribe 失败 (channel={channel})")]
    Subscribe {
        /// 出错的 channel 名
        channel: String,
    },
    /// unsubscribe 失败
    #[error("lcm_unsubscribe 失败 (channel={channel})")]
    Unsubscribe {
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

/// Failed single publication, classified at the ownership-transfer source.
///
/// Neither a completed local publish nor an unconfirmed effect proves that a
/// remote execution lease has stopped. Batch publication keeps its own contract.
#[derive(Debug, Error)]
pub enum PublishError {
    /// This command was not transferred to the owner queue.
    #[error("{0}")]
    NotDispatched(#[source] LcmError),
    /// The command may have executed; its external effect is not disproved.
    #[error("{0}")]
    EffectUnconfirmed(#[source] LcmError),
}

impl PublishError {
    /// Return the original error for callers retaining the legacy API contract.
    pub fn into_error(self) -> LcmError {
        match self {
            Self::NotDispatched(error) | Self::EffectUnconfirmed(error) => error,
        }
    }
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
    subscription_id: Option<u64>,
    /// channel 名(用作 unsubscribe key)
    pub channel: Arc<CString>,
    /// broadcast receiver
    pub receiver: broadcast::Receiver<LcmReceivedMessage>,
    control_tx: std::sync::mpsc::Sender<Command>,
    wakeup: Arc<CommandWakeup>,
}

/// Message delivered by liblcm, including the actual callback channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LcmReceivedMessage {
    /// Actual channel passed to the C callback.
    pub channel: String,
    /// Complete generated LCM message bytes.
    pub payload: Bytes,
}

impl std::fmt::Debug for LcmSubscription {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LcmSubscription")
            .field("channel", &self.channel.to_string_lossy().into_owned())
            .finish_non_exhaustive()
    }
}

impl LcmSubscription {
    /// Explicitly unsubscribe and wait until the owner has called `lcm_unsubscribe`.
    pub async fn unsubscribe(&mut self) -> Result<(), LcmError> {
        let Some(subscription_id) = self.subscription_id.take() else {
            return Ok(());
        };
        let (reply_tx, reply_rx) = oneshot::channel();
        self.control_tx
            .send(Command::Unsubscribe {
                subscription_id,
                reply: Some(reply_tx),
            })
            .map_err(|error| LcmError::OwnerThreadDown {
                detail: error.to_string(),
            })?;
        self.wakeup.wake_owner()?;
        reply_rx.await.map_err(|error| LcmError::OwnerThreadDown {
            detail: error.to_string(),
        })?
    }
}

impl Drop for LcmSubscription {
    fn drop(&mut self) {
        if let Some(subscription_id) = self.subscription_id.take() {
            let _ = self.control_tx.send(Command::Unsubscribe {
                subscription_id,
                reply: None,
            });
            let _ = self.wakeup.notify();
        }
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
    /// 同一批消息按给定顺序发布，整批只确认一次。
    PublishBatch {
        messages: Vec<(CString, Bytes)>,
        reply: oneshot::Sender<Result<(), LcmError>>,
    },
    /// 注册一条订阅。owner thread 调 lcm_subscribe + 装一个 broadcast sender
    Subscribe {
        /// channel(CString)
        channel: CString,
        /// 回调形式 channel 名(通常等于 channel;typed suffix 由 `LcmSubscription` 自行记)
        pattern: CString,
        /// reply:成功 / 失败 via oneshot
        reply: oneshot::Sender<Result<u64, LcmError>>,
        /// 注册后用于后续 push bytes 的 broadcast sender
        sender: broadcast::Sender<LcmReceivedMessage>,
    },
    /// 取消订阅
    Unsubscribe {
        /// Owner-local subscription identifier.
        subscription_id: u64,
        /// Optional processing acknowledgement.
        reply: Option<oneshot::Sender<Result<(), LcmError>>>,
    },
    /// Cancel every subscription registered for one channel.
    UnsubscribeChannel {
        /// Channel.
        channel: CString,
        /// Processing acknowledgement.
        reply: oneshot::Sender<Result<(), LcmError>>,
    },
    /// 关闭 owner thread
    Shutdown {
        /// Optional completion acknowledgement used by explicit shutdown.
        reply: Option<oneshot::Sender<()>>,
    },
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
    /// Non-droppable lifecycle/control commands, independent of publish backpressure.
    control_rx: std::sync::mpsc::Receiver<Command>,
    wakeup: Arc<CommandWakeup>,
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
    subscriptions: HashMap<u64, SubscriptionHandle>,
    next_subscription_id: u64,
    retired_entries: Vec<*mut SubscriptionEntry>,
}

/// SubscriptionHandle —— owner thread 内部:`sub` 是 liblcm handle;
/// `entry` 是 leaked `Box<SubscriptionEntry>`,由 callback 通过 user_data
/// 取回。`unsubscribe` / `run` 收尾统一释放。
struct SubscriptionHandle {
    channel: CString,
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
    sender: broadcast::Sender<LcmReceivedMessage>,
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
        sender: broadcast::Sender<LcmReceivedMessage>,
    ) -> Result<u64, LcmError> {
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
        let capacity_rc = unsafe {
            lcm_subscription_set_queue_capacity(sub, LCM_SUBSCRIPTION_QUEUE_CAPACITY as i32)
        };
        if capacity_rc != 0 {
            unsafe { (*entry_ptr).alive.store(false, Ordering::Release) };
            let unsubscribe_rc = unsafe { lcm_unsubscribe(self.lcm, sub) };
            if unsubscribe_rc == 0 {
                unsafe { drop(Box::from_raw(entry_ptr)) };
            } else {
                self.retired_entries.push(entry_ptr);
            }
            return Err(LcmError::Subscribe { channel: chan_str });
        }

        let subscription_id = self.next_subscription_id;
        self.next_subscription_id = self.next_subscription_id.wrapping_add(1);
        self.subscriptions.insert(
            subscription_id,
            SubscriptionHandle {
                channel,
                sub,
                entry: entry_ptr,
            },
        );
        Ok(subscription_id)
    }

    fn unsubscribe(&mut self, subscription_id: u64) -> Result<(), LcmError> {
        // 2026-07-09 B2 修复:从 subscriptions 取 sub handle 调 liblcm
        // (原先错用 channel 字符串 — liblcm 1.x 的 lcm_unsubscribe 不接受
        //  channel 字符串,会直接返回错误或 UB)。
        //
        // 2026-07-11 rework22 P0-01:先设 `entry.alive = false`,这样
        // 接下来 unsubscribed 的 sub 在 race window 内被 liblcm 二次投
        // 也不会再 push bytes 走 dangling sender。
        let handle = match self.subscriptions.remove(&subscription_id) {
            Some(h) => h,
            None => {
                warn!(
                    "lcm_unsubscribe: 未找到 id={} 的 subscription handle",
                    subscription_id
                );
                return Ok(());
            }
        };
        unsafe { (*handle.entry).alive.store(false, Ordering::Release) };
        let rc = unsafe { lcm_unsubscribe(self.lcm, handle.sub) };
        if rc != 0 {
            let channel = handle.channel.to_string_lossy().into_owned();
            self.retired_entries.push(handle.entry);
            return Err(LcmError::Unsubscribe { channel });
        }
        // liblcm guarantees that, after a successful `lcm_unsubscribe`, the
        // handler is no longer valid and will not be invoked again. Since all
        // calls are serialized on this owner thread, callback userdata can be
        // reclaimed here instead of growing until context shutdown.
        unsafe { drop(Box::from_raw(handle.entry)) };
        Ok(())
    }

    fn unsubscribe_channel(&mut self, channel: &CString) -> Result<(), LcmError> {
        let ids: Vec<u64> = self
            .subscriptions
            .iter()
            .filter_map(|(id, handle)| (handle.channel == *channel).then_some(*id))
            .collect();
        let mut first_error = None;
        for id in ids {
            if let Err(error) = self.unsubscribe(id) {
                first_error.get_or_insert(error);
            }
        }
        match first_error {
            Some(error) => Err(error),
            None => Ok(()),
        }
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

    fn handle_command(&mut self, command: Command) -> (bool, Option<oneshot::Sender<()>>) {
        match command {
            Command::Shutdown { reply } => {
                info!("nuwax-lcm owner thread 收到 Shutdown,退出");
                (true, reply)
            }
            Command::Publish {
                channel,
                payload,
                reply,
            } => {
                let result = self.publish(&channel, &payload);
                let _ = reply.send(result);
                (false, None)
            }
            Command::PublishBatch { messages, reply } => {
                let result =
                    messages
                        .iter()
                        .enumerate()
                        .try_for_each(|(index, (channel, payload))| {
                            self.publish(channel, payload).map_err(|source| {
                                LcmError::PublishBatch {
                                    index,
                                    source: Box::new(source),
                                }
                            })
                        });
                let _ = reply.send(result);
                (false, None)
            }
            Command::Subscribe {
                channel,
                pattern,
                reply,
                sender,
            } => {
                let result = self.subscribe(channel, pattern, sender);
                let _ = reply.send(result);
                (false, None)
            }
            Command::Unsubscribe {
                subscription_id,
                reply,
            } => {
                let result = self.unsubscribe(subscription_id);
                if let Some(reply) = reply {
                    let _ = reply.send(result);
                }
                (false, None)
            }
            Command::UnsubscribeChannel { channel, reply } => {
                let result = self.unsubscribe_channel(&channel);
                let _ = reply.send(result);
                (false, None)
            }
        }
    }

    /// 空闲时只等待实际数据或命令通知，不以固定收包超时拖延发布。
    fn wait_for_activity(&self) -> io::Result<()> {
        // 描述符由当前拥有线程读取并使用，其他线程只写私有唤醒管道。
        let lcm_fd = unsafe { lcm_get_fileno(self.lcm) };
        if lcm_fd < 0 {
            return Err(io::Error::other("LCM 未提供可等待的接收描述符"));
        }
        let mut descriptors = [
            libc::pollfd {
                fd: lcm_fd,
                events: libc::POLLIN,
                revents: 0,
            },
            libc::pollfd {
                fd: self.wakeup.reader.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            },
        ];
        loop {
            // 描述符在本次等待期间均由 self 保活；仅系统调用读取此定长数组。
            let result = unsafe {
                libc::poll(
                    descriptors.as_mut_ptr(),
                    descriptors.len() as libc::nfds_t,
                    -1,
                )
            };
            if result < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(error);
            }
            if descriptors
                .iter()
                .any(|fd| fd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0)
            {
                return Err(io::Error::other("通信或命令唤醒描述符失效"));
            }
            if descriptors[1].revents & libc::POLLIN != 0 {
                self.wakeup.consume()?;
            }
            return Ok(());
        }
    }

    /// 保持原有控制、命令和接收配额，在同一线程完成所有通信操作。
    fn run(mut self) {
        info!("nuwax-lcm owner thread 启动");
        let _fd = unsafe { lcm_get_fileno(self.lcm) };

        // rework22 P0-04:每个 tick 内把所有 queued commands 处理完再
        // 让出时间片给 liblcm(原实现一次只 drain 一条,在 50ms 吞吐
        // 有限;高负载 publish 风暴会塞满 mpsc → caller backpressure)
        let mut shutdown = false;
        let mut shutdown_reply = None;
        while !shutdown {
            let mut control_count = 0;
            while control_count < MAX_COMMANDS_PER_TICK {
                match self.control_rx.try_recv() {
                    Ok(command) => {
                        let (should_shutdown, reply) = self.handle_command(command);
                        if should_shutdown {
                            shutdown = true;
                            shutdown_reply = reply;
                            break;
                        }
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => break,
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        error!("nuwax-lcm owner thread 的 control sender 全 dropped;退出");
                        shutdown = true;
                        break;
                    }
                }
                control_count += 1;
            }
            if shutdown {
                break;
            }

            let mut command_count = 0;
            while command_count < MAX_COMMANDS_PER_TICK {
                match self.cmd_rx.try_recv() {
                    Ok(command) => {
                        let (should_shutdown, reply) = self.handle_command(command);
                        if should_shutdown {
                            shutdown = true;
                            shutdown_reply = reply;
                            break;
                        }
                    }
                    Err(mpsc::error::TryRecvError::Empty) => break,
                    Err(mpsc::error::TryRecvError::Disconnected) => {
                        debug!("nuwax-lcm owner thread 的 publish sender 已关闭");
                        break;
                    }
                }
                command_count += 1;
            }
            if shutdown {
                break;
            }

            // Both sides are bounded. A command storm cannot starve receive,
            // and an inbound storm cannot starve publish/subscribe commands.
            let mut receive_count = 0;
            for _ in 0..MAX_RECEIVES_PER_TICK {
                let handled = unsafe { lcm_handle_timeout(self.lcm, 0) };
                if handled <= 0 {
                    break;
                }
                receive_count += 1;
            }
            if command_count == 0 && control_count == 0 && receive_count == 0 {
                if let Err(error) = self.wait_for_activity() {
                    error!(%error, "LCM 拥有线程事件等待失败，退出并释放资源");
                    break;
                }
            }
        }

        // 收尾:drain 所有 subscriptions(2026-07-11 rework22 P0-01:
        // 先把 entry.alive 设为 false,再 lcm_unsubscribe,最后释放
        // leaked entry —— 与 `unsubscribe` 同序,避免 callback race)
        for (_, handle) in self.subscriptions.drain() {
            unsafe { (*handle.entry).alive.store(false, Ordering::Release) };
            let rc = unsafe { lcm_unsubscribe(self.lcm, handle.sub) };
            if rc == 0 {
                unsafe { drop(Box::from_raw(handle.entry)) };
            } else {
                self.retired_entries.push(handle.entry);
            }
        }
        unsafe { lcm_destroy(self.lcm) };
        for entry in self.retired_entries.drain(..) {
            unsafe { drop(Box::from_raw(entry)) };
        }
        if let Some(reply) = shutdown_reply {
            let _ = reply.send(());
        }
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
    if user.is_null() || rbuf.is_null() || channel.is_null() {
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
    if data_ptr.is_null() && len != 0 {
        return;
    }
    // SAFETY: alive=true 时 entry 仍 leaked & valid;只借用 sender
    // 的引用,不 move,不释放 allocation。
    let sender = unsafe { &(*entry).sender };
    let bytes = if len == 0 {
        Bytes::new()
    } else {
        let slice = unsafe { std::slice::from_raw_parts(data_ptr as *const u8, len) };
        Bytes::copy_from_slice(slice)
    };
    let cstr = unsafe { std::ffi::CStr::from_ptr(channel) };
    let actual_channel = cstr.to_string_lossy().into_owned();
    let _ = sender.send(LcmReceivedMessage {
        channel: actual_channel,
        payload: bytes,
    });
    debug!("lcm_msg_callback: channel={:?} len={}", cstr, len);
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
    /// Unbounded lifecycle channel so unsubscribe/shutdown cannot be dropped by data backpressure.
    control_tx: std::sync::mpsc::Sender<Command>,
    wakeup: Arc<CommandWakeup>,
    /// provider URL —— Debug 用
    provider: Arc<String>,
    owner_thread: Mutex<Option<JoinHandle<()>>>,
    owner_alive: Arc<AtomicBool>,
}

impl std::fmt::Debug for LcmContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LcmContext")
            .field("provider", &self.provider)
            .field("cmd_tx_open", &!self.cmd_tx.is_closed())
            .field("owner_alive", &self.owner_alive.load(Ordering::Acquire))
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
        let wakeup = Arc::new(CommandWakeup::new().map_err(|error| {
            LcmError::BackendUnavailable(format!("创建通信命令唤醒管道失败：{error}"))
        })?);
        let provider_c =
            to_cstring(provider).map_err(|_| LcmError::NulByte(provider.to_owned()))?;
        let lcm = unsafe { lcm_create(provider_c.as_ptr()) };
        if lcm.is_null() {
            return Err(LcmError::Create {
                provider: provider.to_owned(),
            });
        }

        let (cmd_tx, cmd_rx) = mpsc::channel::<Command>(LCM_COMMAND_QUEUE_CAPACITY);
        let (control_tx, control_rx) = std::sync::mpsc::channel::<Command>();

        let owner = OwnerThread {
            lcm,
            cmd_rx,
            control_rx,
            wakeup: Arc::clone(&wakeup),
            subscriptions: std::collections::HashMap::new(),
            next_subscription_id: 1,
            retired_entries: Vec::new(),
        };
        let provider_owned = Arc::new(provider.to_owned());
        let owner_alive = Arc::new(AtomicBool::new(true));
        let thread_alive = Arc::clone(&owner_alive);

        let owner_thread = std::thread::Builder::new()
            .name("nuwax-lcm-owner".into())
            .spawn(move || {
                owner.run();
                thread_alive.store(false, Ordering::Release);
            })
            .map_err(|e| {
                // 创建线程失败时闭包未运行，原生上下文仍由构造方负责释放。
                unsafe { lcm_destroy(lcm) };
                LcmError::BackendUnavailable(format!("spawn owner thread 失败: {e}"))
            })?;

        Ok(Self {
            cmd_tx,
            control_tx,
            wakeup,
            provider: provider_owned,
            owner_thread: Mutex::new(Some(owner_thread)),
            owner_alive,
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
        self.publish_with_outcome(channel, payload)
            .await
            .map_err(PublishError::into_error)
    }

    /// Publish once while preserving whether this command reached the queue.
    ///
    /// A fatal wake error still returns immediately after enqueue. It does not
    /// retract the command, wait indefinitely for a reply, or send another copy.
    pub async fn publish_with_outcome(
        &self,
        channel: &str,
        payload: Bytes,
    ) -> Result<(), PublishError> {
        let c = to_cstring(channel)
            .map_err(|_| PublishError::NotDispatched(LcmError::NulByte(channel.to_owned())))?;
        let (reply_tx, reply_rx) = oneshot::channel();
        self.cmd_tx
            .send(Command::Publish {
                channel: c,
                payload,
                reply: reply_tx,
            })
            .await
            .map_err(|error| PublishError::NotDispatched(LcmError::OwnerThreadDown {
                detail: error.to_string(),
            }))?;
        self.wakeup.wake_owner().map_err(PublishError::EffectUnconfirmed)?;
        reply_rx
            .await
            .map_err(|error| PublishError::EffectUnconfirmed(LcmError::OwnerThreadDown {
                detail: error.to_string(),
            }))?
            .map_err(PublishError::EffectUnconfirmed)
    }

    /// 将一个有序批次交给原有线程连续发布，完成整批后返回一次确认。
    ///
    /// 中途失败立即停止；已发布的前缀不会撤回。确认仅表示本地发布完成。
    pub async fn publish_batch(&self, messages: Vec<(String, Bytes)>) -> Result<(), LcmError> {
        if messages.is_empty() {
            return Ok(());
        }
        let messages = messages
            .into_iter()
            .map(|(channel, payload)| {
                to_cstring(&channel)
                    .map(|channel| (channel, payload))
                    .map_err(|_| LcmError::NulByte(channel))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let (reply_tx, reply_rx) = oneshot::channel();
        self.cmd_tx
            .send(Command::PublishBatch {
                messages,
                reply: reply_tx,
            })
            .await
            .map_err(|error| LcmError::OwnerThreadDown {
                detail: error.to_string(),
            })?;
        self.wakeup.wake_owner()?;
        reply_rx.await.map_err(|error| LcmError::OwnerThreadDown {
            detail: error.to_string(),
        })?
    }

    /// 注册一个 subscription。
    ///
    /// 返回一个 [`LcmSubscription`],caller 通过其 `receiver` 拉消息。
    pub async fn subscribe(&self, channel: &str) -> Result<LcmSubscription, LcmError> {
        let chan_c = to_cstring(channel).map_err(|_| LcmError::NulByte(channel.to_owned()))?;
        let pattern_c = chan_c.clone();
        let (tx, rx) = broadcast::channel::<LcmReceivedMessage>(LCM_SUBSCRIPTION_QUEUE_CAPACITY);
        let (reply_tx, reply_rx) = oneshot::channel();

        self.control_tx
            .send(Command::Subscribe {
                channel: chan_c.clone(),
                pattern: pattern_c,
                sender: tx,
                reply: reply_tx,
            })
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?;

        self.wakeup.wake_owner()?;
        let resp = reply_rx.await.map_err(|e| LcmError::OwnerThreadDown {
            detail: format!("subscribe reply 失败: {e}"),
        })?;
        let subscription_id = resp?;

        Ok(LcmSubscription {
            subscription_id: Some(subscription_id),
            channel: Arc::new(chan_c),
            receiver: rx,
            control_tx: self.control_tx.clone(),
            wakeup: Arc::clone(&self.wakeup),
        })
    }

    /// 取消订阅(可选;Drop 时也会自动 unsubscribe)
    pub async fn unsubscribe(&self, channel: &str) -> Result<(), LcmError> {
        let c = to_cstring(channel).map_err(|_| LcmError::NulByte(channel.to_owned()))?;
        let (reply_tx, reply_rx) = oneshot::channel();
        self.control_tx
            .send(Command::UnsubscribeChannel {
                channel: c,
                reply: reply_tx,
            })
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?;
        self.wakeup.wake_owner()?;
        reply_rx.await.map_err(|e| LcmError::OwnerThreadDown {
            detail: e.to_string(),
        })?
    }

    /// 主动关闭 owner thread。一般用不到(Drop 会自动关)。
    pub async fn shutdown(&self) -> Result<(), LcmError> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.control_tx
            .send(Command::Shutdown {
                reply: Some(reply_tx),
            })
            .map_err(|e| LcmError::OwnerThreadDown {
                detail: e.to_string(),
            })?;
        self.wakeup.wake_owner()?;
        reply_rx.await.map_err(|e| LcmError::OwnerThreadDown {
            detail: e.to_string(),
        })?;
        self.join_owner_thread()
    }

    fn join_owner_thread(&self) -> Result<(), LcmError> {
        let owner = self
            .owner_thread
            .lock()
            .map_err(|_| LcmError::OwnerThreadDown {
                detail: "owner join mutex poisoned".into(),
            })?
            .take();
        if let Some(owner) = owner {
            owner.join().map_err(|_| LcmError::OwnerThreadDown {
                detail: "owner thread panicked".into(),
            })?;
        }
        Ok(())
    }
}

impl Drop for LcmContext {
    fn drop(&mut self) {
        let _ = self.control_tx.send(Command::Shutdown { reply: None });
        let _ = self.wakeup.notify();
        if let Ok(owner) = self.owner_thread.get_mut() {
            if let Some(owner) = owner.take() {
                let _ = owner.join();
            }
        }
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

    #[test]
    fn context_drop_joins_owner_thread() {
        let context = LcmContext::new("memq://").expect("create in-memory LCM context");
        let alive = Arc::clone(&context.owner_alive);
        assert!(alive.load(Ordering::Acquire));
        drop(context);
        assert!(!alive.load(Ordering::Acquire));
    }

    #[tokio::test]
    async fn measurement_publication_does_not_wait_per_imu_sample() {
        let context = LcmContext::new("memq://").expect("真实 liblcm 内存传输");
        let started = std::time::Instant::now();
        context
            .publish_batch(
                (0_u64..21)
                    .map(|index| {
                        (
                            "/test/measurement".to_owned(),
                            Bytes::copy_from_slice(&index.to_le_bytes()),
                        )
                    })
                    .collect(),
            )
            .await
            .unwrap();
        let elapsed = started.elapsed();
        eprintln!("21 条输入消息发布耗时：{elapsed:?}");
        assert!(
            elapsed < std::time::Duration::from_millis(500),
            "同帧输入不应累计逐条等待：{elapsed:?}"
        );
        context.publish_batch(Vec::new()).await.unwrap();
        context.shutdown().await.unwrap();
    }

    #[tokio::test]
    async fn idle_owner_publish_wakes_without_waiting_for_receive_timeout() {
        let context = LcmContext::new("memq://").expect("真实 liblcm 内存传输");
        let mut elapsed_samples = Vec::new();
        for _ in 0..12 {
            std::thread::sleep(std::time::Duration::from_millis(5));
            let started = std::time::Instant::now();
            context
                .publish("/test/idle-wakeup", Bytes::from_static(b"probe"))
                .await
                .unwrap();
            elapsed_samples.push(started.elapsed());
        }
        elapsed_samples.sort();
        let median = elapsed_samples[elapsed_samples.len() / 2];
        eprintln!("空闲拥有线程发布中位延迟：{median:?}；样本：{elapsed_samples:?}");
        context.shutdown().await.unwrap();
        assert!(
            median < std::time::Duration::from_millis(20),
            "发布命令不应等待收包轮询超时：{median:?}"
        );
    }

    #[tokio::test]
    async fn wakeup_preserves_batch_order_subscription_cleanup_and_idle_shutdown() {
        let context = LcmContext::new("memq://").expect("真实 liblcm 内存传输");
        let wakeup = Arc::downgrade(&context.wakeup);
        std::thread::sleep(std::time::Duration::from_millis(5));
        let mut subscription = context.subscribe("/test/wakeup-order").await.unwrap();
        let retired = context.subscribe("/test/wakeup-retired").await.unwrap();
        drop(retired);
        context
            .publish_batch(
                (0_u64..8)
                    .map(|index| {
                        (
                            "/test/wakeup-order".to_owned(),
                            Bytes::copy_from_slice(&index.to_le_bytes()),
                        )
                    })
                    .collect(),
            )
            .await
            .unwrap();
        for index in 0_u64..8 {
            let message = tokio::time::timeout(
                std::time::Duration::from_secs(1),
                subscription.receiver.recv(),
            )
            .await
            .unwrap()
            .unwrap();
            assert_eq!(message.payload.as_ref(), &index.to_le_bytes());
        }
        subscription.unsubscribe().await.unwrap();
        context
            .publish(
                "/test/wakeup-order",
                Bytes::from_static(b"after-unsubscribe"),
            )
            .await
            .unwrap();
        assert!(subscription.receiver.is_closed());
        context.unsubscribe("/test/wakeup-retired").await.unwrap();
        std::thread::sleep(std::time::Duration::from_millis(5));
        context.shutdown().await.unwrap();
        drop(subscription);
        drop(context);
        assert!(
            wakeup.upgrade().is_none(),
            "拥有方和订阅释放后唤醒描述符一并回收"
        );
    }

    #[tokio::test]
    async fn shutdown_wakes_owner_even_when_publish_queue_capacity_is_reserved() {
        let context = LcmContext::new("memq://").expect("真实 liblcm 内存传输");
        let permits = context
            .cmd_tx
            .reserve_many(LCM_COMMAND_QUEUE_CAPACITY)
            .await
            .unwrap();
        std::thread::sleep(std::time::Duration::from_millis(5));
        tokio::time::timeout(std::time::Duration::from_secs(1), context.shutdown())
            .await
            .expect("关闭不等待发布队列容量")
            .unwrap();
        assert!(!context.owner_alive.load(Ordering::Acquire));
        drop(permits);
    }

    #[cfg(feature = "link-liblcm")]
    #[test]
    fn successful_unsubscribe_reclaims_callback_entry_immediately() {
        let provider = to_cstring("memq://").expect("provider CString");
        let lcm = unsafe { lcm_create(provider.as_ptr()) };
        assert!(!lcm.is_null(), "create memq provider");

        let (_cmd_tx, cmd_rx) = mpsc::channel(1);
        let (_control_tx, control_rx) = std::sync::mpsc::channel();
        let mut owner = OwnerThread {
            lcm,
            cmd_rx,
            control_rx,
            wakeup: Arc::new(CommandWakeup::new().expect("命令唤醒管道")),
            subscriptions: HashMap::new(),
            next_subscription_id: 1,
            retired_entries: Vec::new(),
        };
        let channel = to_cstring("/nuwax/churn").expect("channel CString");
        let (sender, _receiver) = broadcast::channel(1);
        let subscription_id = owner
            .subscribe(channel.clone(), channel, sender)
            .expect("subscribe");

        owner.unsubscribe(subscription_id).expect("unsubscribe");
        let retired_after_success = owner.retired_entries.len();

        unsafe { lcm_destroy(owner.lcm) };
        for entry in owner.retired_entries.drain(..) {
            unsafe { drop(Box::from_raw(entry)) };
        }

        assert_eq!(
            retired_after_success, 0,
            "successful lcm_unsubscribe guarantees handler quiescence, so userdata must not accumulate"
        );
    }

    #[cfg(feature = "link-liblcm")]
    #[test]
    fn repeated_subscribe_unsubscribe_churn_does_not_retain_userdata() {
        let provider = to_cstring("memq://").expect("provider CString");
        let lcm = unsafe { lcm_create(provider.as_ptr()) };
        assert!(!lcm.is_null(), "create memq provider");

        let (_cmd_tx, cmd_rx) = mpsc::channel(1);
        let (_control_tx, control_rx) = std::sync::mpsc::channel();
        let mut owner = OwnerThread {
            lcm,
            cmd_rx,
            control_rx,
            wakeup: Arc::new(CommandWakeup::new().expect("命令唤醒管道")),
            subscriptions: HashMap::new(),
            next_subscription_id: 1,
            retired_entries: Vec::new(),
        };

        for sequence in 0..10_000_u64 {
            let channel = to_cstring(&format!("/nuwax/churn/{sequence}")).expect("channel CString");
            let (sender, _receiver) = broadcast::channel(1);
            let subscription_id = owner
                .subscribe(channel.clone(), channel, sender)
                .expect("subscribe");
            owner.unsubscribe(subscription_id).expect("unsubscribe");
            assert!(owner.subscriptions.is_empty());
            assert!(owner.retired_entries.is_empty());
        }

        unsafe { lcm_destroy(owner.lcm) };
    }

    #[cfg(feature = "link-liblcm")]
    #[test]
    fn callback_delivery_unsubscribe_churn_is_asan_clean() {
        let provider = to_cstring("memq://").expect("provider CString");
        let lcm = unsafe { lcm_create(provider.as_ptr()) };
        assert!(!lcm.is_null(), "create memq provider");

        let (_cmd_tx, cmd_rx) = mpsc::channel(1);
        let (_control_tx, control_rx) = std::sync::mpsc::channel();
        let mut owner = OwnerThread {
            lcm,
            cmd_rx,
            control_rx,
            wakeup: Arc::new(CommandWakeup::new().expect("命令唤醒管道")),
            subscriptions: HashMap::new(),
            next_subscription_id: 1,
            retired_entries: Vec::new(),
        };

        for sequence in 0..5_000_u64 {
            let channel =
                to_cstring(&format!("/nuwax/callback-churn/{sequence}")).expect("channel CString");
            let (sender, mut receiver) = broadcast::channel(1);
            let subscription_id = owner
                .subscribe(channel.clone(), channel.clone(), sender)
                .expect("subscribe");
            owner
                .publish(&channel, &Bytes::copy_from_slice(&sequence.to_be_bytes()))
                .expect("publish before unsubscribe");
            assert!(unsafe { lcm_handle_timeout(owner.lcm, 0) } > 0);
            let delivered = receiver.try_recv().expect("callback delivery");
            assert_eq!(delivered.payload.as_ref(), sequence.to_be_bytes());

            owner.unsubscribe(subscription_id).expect("unsubscribe");
            owner
                .publish(&channel, &Bytes::from_static(b"after-unsubscribe"))
                .expect("publish after unsubscribe");
            let _ = unsafe { lcm_handle_timeout(owner.lcm, 0) };
            assert!(matches!(
                receiver.try_recv(),
                Err(broadcast::error::TryRecvError::Empty)
                    | Err(broadcast::error::TryRecvError::Closed)
            ));
            assert!(owner.retired_entries.is_empty());
        }

        unsafe { lcm_destroy(owner.lcm) };
    }
}

// Tests-only publish-stage controls; appended to baseline 322e996b.
// Original draft and all actual qualification results are retained outside the repo.
// No production changes or network setup.
// T1-T5 exercise the actual LcmContext publisher with a manually owned queue.
// T6 exercises actual OwnerThread::handle_command and same-context liblcm memq.
// This is not a UDP, remote-receiver, or robot qualification.

#[cfg(test)]
mod publish_error_outcomes_review {
    use super::*;
    use std::future::{poll_fn, Future};
    use std::net::Shutdown;
    use std::pin::Pin;
    use std::task::Poll;

    fn manual_context() -> (
        LcmContext,
        mpsc::Receiver<Command>,
        std::sync::mpsc::Receiver<Command>,
    ) {
        let (cmd_tx, cmd_rx) = mpsc::channel(1);
        let (control_tx, control_rx) = std::sync::mpsc::channel();
        (
            LcmContext {
                cmd_tx,
                control_tx,
                wakeup: Arc::new(CommandWakeup::new().expect("isolated test wakeup")),
                provider: Arc::new("test-manual-command-owner".to_owned()),
                owner_thread: Mutex::new(None),
                // No real owner thread is claimed by this test fixture.
                owner_alive: Arc::new(AtomicBool::new(false)),
            },
            cmd_rx,
            control_rx,
        )
    }

    async fn completion_with_deadline<F: Future>(future: F) -> F::Output {
        tokio::time::timeout(std::time::Duration::from_secs(5), future)
            .await
            .expect("isolated LCM publication did not complete within fixture deadline")
    }

    async fn poll_once_pending<F: Future>(mut future: Pin<&mut F>) {
        poll_fn(move |cx| {
            assert!(future.as_mut().poll(cx).is_pending());
            Poll::Ready(())
        })
        .await;
    }

    #[tokio::test]
    async fn nul_rejection_does_not_enqueue() {
        let (context, mut commands, _control) = manual_context();
        let result = completion_with_deadline(context.publish("/test/nul\0suffix", Bytes::from_static(b"goal"))).await;
        let queue_is_empty = matches!(commands.try_recv(), Err(mpsc::error::TryRecvError::Empty));
        assert!(matches!(result, Err(LcmError::NulByte(_))));
        assert!(queue_is_empty, "CString rejection precedes command ownership transfer");
    }

    #[tokio::test]
    async fn closed_command_queue_does_not_enqueue() {
        let (context, commands, _control) = manual_context();
        drop(commands);
        let result = completion_with_deadline(context.publish("/test/closed", Bytes::from_static(b"goal"))).await;
        assert!(matches!(result, Err(LcmError::OwnerThreadDown { .. })));
        // No receiver existed for the send, so this attempt cannot be executed.
    }

    #[tokio::test]
    async fn fatal_wakeup_error_leaves_the_command_queued() {
        let (context, mut commands, _control) = manual_context();
        // Artificially inject a fatal wake failure into this isolated fixture.
        context.wakeup.writer.shutdown(Shutdown::Write).expect("half-close test writer");
        let result = completion_with_deadline(context.publish("/test/wake", Bytes::from_static(b"goal"))).await;
        let queued = commands.try_recv().expect("send succeeded before wake failed");
        let (channel, payload, caller_stopped_observing) = match queued {
            Command::Publish { channel, payload, reply } => {
                (channel, payload, reply.is_closed())
            }
            _ => panic!("expected the original single publish command"),
        };
        assert!(matches!(result, Err(LcmError::OwnerThreadDown { .. })));
        assert_eq!(channel.to_bytes(), b"/test/wake");
        assert_eq!(payload.as_ref(), b"goal");
        assert!(caller_stopped_observing, "current wake-error ? dropped the original reply receiver");
    }

    #[tokio::test]
    async fn reply_loss_after_enqueue_is_not_send_rejection() {
        let (context, mut commands, _control) = manual_context();
        let mut publication = Box::pin(context.publish("/test/reply-lost", Bytes::from_static(b"goal")));
        poll_once_pending(publication.as_mut()).await;
        let queued = commands.try_recv().expect("actual publish call enqueued its command");
        assert!(matches!(&queued, Command::Publish { .. }));
        // This fixture deliberately discards a not-yet-executed command.
        // Other histories cannot be inferred from the shared error variant.
        drop(queued);
        assert!(matches!(completion_with_deadline(publication).await, Err(LcmError::OwnerThreadDown { .. })));
    }

    #[tokio::test]
    async fn reply_already_delivered_survives_queue_close() {
        let (context, mut commands, _control) = manual_context();
        let mut publication = Box::pin(context.publish("/test/replied", Bytes::from_static(b"goal")));
        poll_once_pending(publication.as_mut()).await;
        let queued = commands.try_recv().expect("original publication command");
        match queued {
            Command::Publish { reply, .. } => reply.send(Ok(())).expect("live reply receiver"),
            _ => panic!("expected publish command"),
        }
        drop(commands);
        assert!(completion_with_deadline(publication).await.is_ok(), "queue/owner termination cannot erase an already sent reply");
    }

    // Keep cleanup owned even if an assertion panics after native setup.
    #[cfg(feature = "link-liblcm")]
    struct ManualNativeOwner {
        context: LcmContext,
        owner: Option<OwnerThread>,
    }

    #[cfg(feature = "link-liblcm")]
    impl ManualNativeOwner {
        fn new() -> Self {
            let (context, cmd_rx, control_rx) = manual_context();
            let provider = to_cstring("memq://").expect("literal provider");
            let lcm = unsafe { lcm_create(provider.as_ptr()) };
            assert!(!lcm.is_null(), "same-context memq must be available");
            let owner = OwnerThread {
                lcm,
                cmd_rx,
                control_rx,
                wakeup: Arc::clone(&context.wakeup),
                subscriptions: std::collections::HashMap::new(),
                next_subscription_id: 1,
                retired_entries: Vec::new(),
            };
            Self { context, owner: Some(owner) }
        }
    }

    #[cfg(feature = "link-liblcm")]
    impl Drop for ManualNativeOwner {
        fn drop(&mut self) {
            if let Some(owner) = self.owner.take() {
                // The original owner loop consumes its control Shutdown first,
                // unsubscribes/destroys native state, then drops queued commands.
                let _ = self.context.control_tx.send(Command::Shutdown { reply: None });
                owner.run();
            }
        }
    }

    #[cfg(feature = "link-liblcm")]
    #[tokio::test]
    async fn queued_command_can_publish_after_fatal_wakeup_error() {
        let mut fixture = ManualNativeOwner::new();
        let channel = to_cstring("/test/real-owner-wake-error").expect("literal channel");
        let (sender, mut callback) = broadcast::channel(1);
        fixture.owner.as_mut().unwrap()
            .subscribe(channel.clone(), channel, sender)
            .expect("same-context real subscription");
        fixture.context.wakeup.writer.shutdown(Shutdown::Write)
            .expect("inject fatal wake error only in isolated test fixture");

        let caller_result = completion_with_deadline(fixture.context.publish(
            "/test/real-owner-wake-error", Bytes::from_static(b"original-goal-body")
        )).await;
        let owner = fixture.owner.as_mut().unwrap();
        let command = owner.cmd_rx.try_recv().expect("original queued command survives caller Err");
        let abandoned_reply = matches!(&command, Command::Publish { reply, .. } if reply.is_closed());
        let (stopped, _) = owner.handle_command(command);
        let callbacks_handled = unsafe { lcm_handle_timeout(owner.lcm, 0) };
        let delivery = callback.try_recv();
        drop(fixture); // Native ownership cleaned before reporting assertions.

        assert!(matches!(caller_result, Err(LcmError::OwnerThreadDown { .. })));
        assert!(abandoned_reply);
        assert!(!stopped);
        assert!(callbacks_handled > 0);
        assert_eq!(delivery.expect("real memq callback").payload.as_ref(), b"original-goal-body");
        // This proves execution through real liblcm after an injected wake Err.
        // It does NOT prove a UDP packet or a physical navigation command.
    }

    #[tokio::test]
    async fn typed_nul_error_is_not_dispatched_and_queue_stays_empty() {
        let (context, mut commands, _control) = manual_context();
        let result = completion_with_deadline(context.publish_with_outcome(
            "/test/typed-nul\0suffix", Bytes::from_static(b"goal"),
        )).await;
        assert!(matches!(result, Err(PublishError::NotDispatched(LcmError::NulByte(_)))));
        assert!(matches!(commands.try_recv(), Err(mpsc::error::TryRecvError::Empty)));
    }

    #[tokio::test]
    async fn typed_closed_queue_error_is_not_dispatched() {
        let (context, commands, _control) = manual_context();
        drop(commands);
        let result = completion_with_deadline(context.publish_with_outcome(
            "/test/typed-closed", Bytes::from_static(b"goal"),
        )).await;
        assert!(matches!(result, Err(PublishError::NotDispatched(LcmError::OwnerThreadDown { .. }))));
    }

    #[tokio::test]
    async fn typed_wake_error_is_unconfirmed_with_original_command_queued() {
        let (context, mut commands, _control) = manual_context();
        context.wakeup.writer.shutdown(Shutdown::Write).expect("inject wake failure");
        let result = completion_with_deadline(context.publish_with_outcome(
            "/test/typed-wake", Bytes::from_static(b"original-goal"),
        )).await;
        let queued = commands.try_recv().expect("the original command was transferred");
        assert!(matches!(result, Err(PublishError::EffectUnconfirmed(LcmError::OwnerThreadDown { .. }))));
        match queued {
            Command::Publish { channel, payload, reply } => {
                assert_eq!(channel.to_bytes(), b"/test/typed-wake");
                assert_eq!(payload.as_ref(), b"original-goal");
                assert!(reply.is_closed());
            }
            _ => panic!("expected one original Publish command"),
        }
        assert!(matches!(commands.try_recv(), Err(mpsc::error::TryRecvError::Empty)));
    }

    #[tokio::test]
    async fn typed_reply_loss_is_unconfirmed_after_actual_enqueue() {
        let (context, mut commands, _control) = manual_context();
        let mut publication = Box::pin(context.publish_with_outcome(
            "/test/typed-reply-loss", Bytes::from_static(b"goal"),
        ));
        poll_once_pending(publication.as_mut()).await;
        let queued = commands.try_recv().expect("actual publisher enqueued");
        assert!(matches!(&queued, Command::Publish { .. }));
        drop(queued); // Explicitly unexecuted here; the API cannot infer that history.
        assert!(matches!(completion_with_deadline(publication).await,
            Err(PublishError::EffectUnconfirmed(LcmError::OwnerThreadDown { .. }))));
    }

    #[tokio::test]
    async fn typed_owner_error_reply_is_unconfirmed_not_a_send_rejection() {
        let (context, mut commands, _control) = manual_context();
        let mut publication = Box::pin(context.publish_with_outcome(
            "/test/typed-owner-error", Bytes::from_static(b"goal"),
        ));
        poll_once_pending(publication.as_mut()).await;
        match commands.try_recv().expect("original queued command") {
            Command::Publish { reply, .. } => reply.send(Err(LcmError::Publish {
                channel: "/test/typed-owner-error".to_owned(),
            })).expect("live receiver"),
            _ => panic!("expected Publish"),
        }
        // The reply is injected, not evidence of a naturally failing native call.
        assert!(matches!(completion_with_deadline(publication).await,
            Err(PublishError::EffectUnconfirmed(LcmError::Publish { .. }))));
    }

    #[tokio::test]
    async fn typed_delivered_ok_survives_queue_close() {
        let (context, mut commands, _control) = manual_context();
        let mut publication = Box::pin(context.publish_with_outcome(
            "/test/typed-replied", Bytes::from_static(b"goal"),
        ));
        poll_once_pending(publication.as_mut()).await;
        match commands.try_recv().expect("original queued command") {
            Command::Publish { reply, .. } => reply.send(Ok(())).expect("live receiver"),
            _ => panic!("expected Publish"),
        }
        drop(commands);
        assert!(completion_with_deadline(publication).await.is_ok());
    }

    #[cfg(feature = "link-liblcm")]
    #[tokio::test]
    async fn typed_wake_unknown_can_still_publish_through_original_native_owner() {
        let mut fixture = ManualNativeOwner::new();
        let channel = to_cstring("/test/typed-native-wake").expect("literal channel");
        let (sender, mut callback) = broadcast::channel(1);
        fixture.owner.as_mut().unwrap().subscribe(channel.clone(), channel, sender)
            .expect("same-context subscription");
        fixture.context.wakeup.writer.shutdown(Shutdown::Write).expect("inject wake failure");
        let result = completion_with_deadline(fixture.context.publish_with_outcome(
            "/test/typed-native-wake", Bytes::from_static(b"original-typed-body"),
        )).await;
        let owner = fixture.owner.as_mut().unwrap();
        let command = owner.cmd_rx.try_recv().expect("the original command is still owned");
        let (stopped, _) = owner.handle_command(command);
        // SAFETY: the fixture exclusively owns this same native context until Drop.
        let handled = unsafe { lcm_handle_timeout(owner.lcm, 0) };
        let delivery = callback.try_recv();
        drop(fixture); // Existing fixture runs original Shutdown/destroy before assertions.
        assert!(matches!(result, Err(PublishError::EffectUnconfirmed(_))));
        assert!(!stopped);
        assert!(handled > 0);
        let delivery = delivery.expect("actual same-context native delivery");
        assert_eq!(delivery.channel, "/test/typed-native-wake");
        assert_eq!(delivery.payload.as_ref(), b"original-typed-body");
        // Manual original handle_command is not natural wake recovery or UDP proof.
    }
}
