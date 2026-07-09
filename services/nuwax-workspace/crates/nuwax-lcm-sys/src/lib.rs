//! `nuwax-lcm-sys`:liblcm 1.x 的最小 extern "C" FFI 绑定。
//!
//! ## 2026-07-07 Phase 3 引入说明
//!
//! 本 crate 是 **Phase 3 (real-LCM transport)** 的硬基础设施,在本仓库的
//! `services/nuwax-workspace/crates/nuwax-lcm-sys/` 下**首次**实装,禁止再
//! 引用上游 dimos 任何旧路径(旧路径已被 `scripts/vendor_dimos_crates.sh`
//! vendor 化为本仓 `vendor/` 同步副本,本仓内代码也走相对 `../nuwax-lcm-sys`)。
//!
//! ## FFI 范围(最小集合)
//!
//! 只绑定我们真正使用的 liblcm 1.4+ 函数 —— 高级特性(`lcm_eventlog_*` /
//! `lcm_*_memq` 等)不暴露,后续如需可增量补。
//!
//! | 函数                       | 用途                              |
//! | -------------------------- | --------------------------------- |
//! | [`lcm_create`]             | 构造 lcm_t(provider URL)          |
//! | [`lcm_destroy`]            | 释放 lcm_t                        |
//! | [`lcm_publish`]            | 发送 payload 到 channel           |
//! | [`lcm_subscribe`]          | 注册 typed subscription          |
//! | [`lcm_unsubscribe`]        | 取消订阅                          |
//! | [`lcm_handle_timeout`]     | 在 event loop 中处理一次消息     |
//! | [`lcm_get_fileno`]         | 取得 file descriptor (select/poll)|
//!
//! ## Send/Sync
//!
//! `lcm_t` 内部含 UDP socket + recv thread 状态,**不是** Sync。`nuwax-lcm`
//! crate 通过 single-owner event thread + mpsc command channel 解决并发问题,
//! 本 crate 不做任何 Send/Sync impl,只暴露 `*mut lcm_t` raw pointer。
//!
//! ## lifetime 契约
//!
//! `lcm_t*` 的 lifetime 由 `Box<LcmContext>` 在 `nuwax-lcm::LcmContext` 内
//! 持有,Drop 时调 `lcm_destroy`;**禁止**把 raw pointer 跨线程 / 跨
//! `LcmContext` 实例传递。

#![deny(unsafe_code)]
#![warn(missing_docs)]

use std::os::raw::{c_char, c_int, c_uint, c_void};

// =============================================================================
// Opaque 类型
// =============================================================================

/// Opaque liblcm context。实际定义在 `<lcm/lcm_coretypes.h>`,外部不可见。
#[repr(C)]
pub struct lcm_t {
    /// 私有字段占位:不引用字段(防止 layout 假设)。
    _private: [u8; 0],
}

/// Opaque liblcm subscription handle。liblcm 1.x 在 `lcm_subscribe` 返
/// 内部的 subscription 对象;`lcm_t` 拥有它的 lifetime。
#[repr(C)]
pub struct lcm_subscription_t {
    /// 私有字段占位
    _private: [u8; 0],
}

/// liblcm 1.x 真实 C 结构(字段顺序来自 `<lcm/lcm.h>` 第 62-80 行)。
///
/// ```c
/// struct _lcm_recv_buf_t {
///     void     *data;        // 接收到的 payload 字节
///     uint32_t  data_size;   // payload 长度
///     int64_t   recv_utime;  // 接收时间戳(微秒)
///     lcm_t    *lcm;         // 拥有此 buffer 的 lcm_t
/// };
/// ```
///
/// 必须用 `#[repr(C)]` 保证布局与 C 一致;`data` 与 `lcm` 是 raw pointer
/// (liblcm 内部用 `g_realloc` 复用,生命周期由 lcm_t 持有)。
#[repr(C)]
pub struct lcm_recv_buf_t {
    /// 接收到的 payload 字节 buffer
    pub data: *mut c_void,
    /// payload 长度(字节)
    pub data_size: u32,
    /// 接收时间戳(微秒自 epoch)
    pub recv_utime: i64,
    /// 拥有此 buffer 的 lcm_t(为方便 callback 访问 lcm 内部状态)
    pub lcm: *mut lcm_t,
}

/// LCM 消息回调签名 (liblcm 原生约定,见 `<lcm/lcm.h>` 第 91 行)。
///
/// 形参:
/// - `rbuf`    — receive buffer(含 payload + timestamp + 拥有方 lcm)
/// - `channel` — NUL-terminated C string,channel 名
/// - `user`    — 注册订阅时传入的 user pointer
#[allow(non_camel_case_types)]
pub type lcm_msg_handler_t = Option<
    unsafe extern "C" fn(
        rbuf: *const lcm_recv_buf_t,
        channel: *const c_char,
        user: *mut c_void,
    ),
>;

// =============================================================================
// FFI 绑定
// =============================================================================

extern "C" {
    /// 创建 lcm_t(provider URL,例如 `udpm://239.255.76.67:7667?ttl=1`)。
    ///
    /// 返回 NULL 即失败;调用方需立刻转 `LcmError::Create(...)`。
    pub fn lcm_create(provider: *const c_char) -> *mut lcm_t;

    /// 释放 lcm_t 及其内部资源。**不允许**对已 destroy 的指针再次调用。
    pub fn lcm_destroy(lcm: *mut lcm_t);

    /// 发布一条消息。
    ///
    /// - `channel` — NUL-terminated C string,例如 `c"/cmd_vel".as_ptr()`
    /// - `data`    — payload 字节 buffer
    /// - `datalen` — payload 长度
    ///
    /// 返回 0 = 成功;非 0 = liblcm 内部错误。
    pub fn lcm_publish(
        lcm: *mut lcm_t,
        channel: *const c_char,
        data: *const c_void,
        datalen: c_uint,
    ) -> c_int;

    /// 注册一个 typed subscription,handler 在 event loop 内被回调。
    ///
    /// 返回指向内部 `lcm_subscription_t` 的指针(归 lcm_t 拥有,
    /// **不要** 手动 free);NULL = 失败(channel 名为 NULL / OOM)。
    /// 后续 `lcm_unsubscribe` 须以该指针为 key。
    pub fn lcm_subscribe(
        lcm: *mut lcm_t,
        channel: *const c_char,
        handler: lcm_msg_handler_t,
        user: *mut c_void,
    ) -> *mut lcm_subscription_t;

    /// 取消一个订阅。
    ///
    /// 形参:`sub` 必须由 `lcm_subscribe` 在同一 `lcm_t` 上返回。
    /// 返回 0 = 成功;非 0 = 找不到该 sub / 已取消。
    pub fn lcm_unsubscribe(lcm: *mut lcm_t, sub: *mut lcm_subscription_t) -> c_int;

    /// 设置 subscription 内部消息队列容量(消息堆积上限)。
    ///
    /// 2026-07-09 暴露:liblcm 1.5.2 头文件第 344 行存在;在高频 channel
    /// 上把 num_messages 调高可避免 producer 快于 consumer 时丢帧。
    /// liblcm 1.3 / 1.4 也兼容(API 自 1.3 引入,见 changelog)。
    pub fn lcm_subscription_set_queue_capacity(
        sub: *mut lcm_subscription_t,
        num_messages: c_int,
    ) -> c_int;

    /// 在 event loop 中处理一次消息,带超时。
    ///
    /// - `timeout_ms` — 阻塞等待时长(毫秒);0 表示非阻塞
    /// - 返回 0 表示 timeout;非 0 通常表示有消息被处理
    ///
    /// 调用方在循环里反复调用 `lcm_handle_timeout(lcm, 50)` 即可驱动
    /// 整个 pub/sub 协议栈。
    pub fn lcm_handle_timeout(lcm: *mut lcm_t, timeout_ms: c_int) -> c_int;

    /// 取得 lcm 内部 socket file descriptor,供 select/poll/epoll 集成。
    pub fn lcm_get_fileno(lcm: *mut lcm_t) -> c_int;
}

// =============================================================================
// 帮助 trait:把 &str 安全转成 C 字符串
// =============================================================================

/// `&str` → `CString` 转换 helper。
///
/// liblcm 的 channel 参数是 C 字符串(`*const c_char`),Rust 侧必须保证
/// NUL-terminated;`CString::new` 在含 NUL 字节时会 panic —— 我们选择显式
/// 返回 `Result` 把决定权交给上层。
pub fn to_cstring(s: &str) -> Result<std::ffi::CString, NulByteError> {
    std::ffi::CString::new(s).map_err(|_| NulByteError { input: s.to_owned() })
}

/// `&str` 含 NUL 字节
#[derive(Debug, thiserror::Error)]
#[error("string contains interior NUL byte: {input:?}")]
pub struct NulByteError {
    /// 原始输入
    pub input: String,
}

// =============================================================================
// 单测:FFI 链接正常性 + to_cstring
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn to_cstring_basic() {
        let c = to_cstring("/cmd_vel").expect("ok");
        assert_eq!(c.as_bytes_with_nul(), b"/cmd_vel\0");
    }

    #[test]
    fn to_cstring_rejects_nul() {
        // NUL 字节 → 错误
        let err = to_cstring("/cmd_vel\0bad").unwrap_err();
        assert_eq!(err.input, "/cmd_vel\0bad");
    }

    /// 在没装 liblcm 的 dev 镜像上,本 crate 编译本来就会 fail;
    /// 这里只验证 symbol **名字** 存在(取函数指针)。函数指针不为 null
    /// 即代表 extern 块正确链接了 libc / liblcm 符号(运行期调用的安全性
    /// 由 `nuwax-lcm` crate 负责)。
    ///
    /// 2026-07-08 rework21 修复:本测试裸取 liblcm 符号 `lcm_destroy` /
    /// `lcm_handle_timeout`,必须链 liblcm 才能编过。加 `cfg(feature = "link-liblcm")`
    /// 守护,跟 `build.rs` 的 `CARGO_FEATURE_LINK_LIBLCM` 门控对称——
    /// 默认 features 不开 → 测试不参与 → 默认 build / `--all-targets` 都过;
    /// 启用 `link-liblcm` → 编译时链 liblcm → 测试也编过 → 真正验证符号。
    #[cfg(feature = "link-liblcm")]
    #[test]
    fn ffi_symbols_linked() {
        // 取地址 ≠ null 即视为 extern "C" 链接通过
        let f: unsafe extern "C" fn(*mut lcm_t) = lcm_destroy;
        assert!(f as usize != 0, "lcm_destroy not linked");

        let p: unsafe extern "C" fn(*mut lcm_t, c_int) -> c_int = lcm_handle_timeout;
        assert!(p as usize != 0, "lcm_handle_timeout not linked");
    }
}