//! Transport-backed request/response RPC for nuwax.
//!
//! The wire contract follows `dimos.protocol.rpc.pubsubrpc`:
//! requests and responses use `/rpc/<name>/{req,res}`, request IDs correlate
//! concurrent calls, and handlers run outside the subscription task so nested
//! calls cannot block request delivery. Payloads use the Python pickle primitive
//! subset needed by the crate's JSON-value API; object-construction opcodes are
//! rejected during decoding.

#![deny(unsafe_code)]
#![warn(missing_docs)]

use async_trait::async_trait;
use nuwax_core::{ModuleError, ModuleResult};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex as StdMutex, MutexGuard as StdMutexGuard};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use thiserror::Error;
use tokio::sync::{oneshot, Mutex, RwLock};
use tokio::task::{JoinHandle, JoinSet};

mod pickle;

/// Default timeout used by [`RpcClient::new`], matching the Python runtime.
pub const DEFAULT_RPC_TIMEOUT: Duration = Duration::from_secs(120);

/// Maximum LCM channel length used by the original runtime.
pub const LCM_MAX_CHANNEL_NAME_LENGTH: usize = 63;

/// Default upper bound for simultaneously executing handlers.
///
/// This matches the original Python `PubSubRPCMixin` thread-pool size.
pub const DEFAULT_RPC_MAX_CONCURRENCY: usize = 50;

static RPC_ID_COUNTER: AtomicU64 = AtomicU64::new(1);

/// RPC errors.
#[derive(Debug, Error)]
pub enum RpcError {
    /// Request or response serialization failed.
    #[error("serialize failed: {0}")]
    Serialize(String),
    /// Request or response deserialization failed.
    #[error("deserialize failed: {0}")]
    Deserialize(String),
    /// The requested handler is not registered.
    #[error("method not found: {0}")]
    MethodNotFound(String),
    /// A response did not arrive before the deadline.
    #[error("rpc timeout after {timeout_ms} ms")]
    Timeout {
        /// Deadline expressed in milliseconds.
        timeout_ms: u64,
    },
    /// A feature-gated backend could not be constructed.
    #[error("rpc backend unavailable: {0}")]
    BackendUnavailable(String),
    /// A local handler returned an exception response.
    #[error("rpc handler error: {0}")]
    Handler(String),
    /// A remote handler returned serialized exception data.
    #[error("remote rpc exception: {0}")]
    RemoteException(Value),
    /// The request/response transport failed.
    #[error("rpc transport error: {0}")]
    Transport(String),
    /// A server task failed while stopping.
    #[error("rpc server task failed: {0}")]
    ServerTask(String),
    /// Server configuration is invalid.
    #[error("invalid rpc server configuration: {0}")]
    InvalidServerConfig(String),
    /// Module-layer error.
    #[error("module error: {0}")]
    Module(#[from] ModuleError),
}

/// RPC result.
pub type RpcResult<T> = Result<T, RpcError>;

/// RPC arguments as `(positional, keyword)`, matching Python `Args`.
pub type RpcArgs = (Vec<Value>, HashMap<String, Value>);

/// RPC request matching the Python `RPCReq` keys.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcRequest {
    /// RPC name.
    pub name: String,
    /// Positional and keyword arguments.
    pub args: RpcArgs,
    /// Correlation ID, or `None` for a fire-and-forget call.
    pub id: Option<f64>,
}

/// RPC response matching the Python `RPCRes` keys.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RpcResponse {
    /// Correlation ID copied from the request.
    pub id: Option<f64>,
    /// Handler result.
    pub res: Option<Value>,
    /// Serialized remote exception.
    pub exception: Option<Value>,
}

impl RpcResponse {
    /// Construct a successful response.
    #[must_use]
    pub fn ok(id: Option<f64>, res: Value) -> Self {
        Self {
            id,
            res: Some(res),
            exception: None,
        }
    }

    /// Construct an exception response.
    #[must_use]
    pub fn error(id: Option<f64>, exception: Value) -> Self {
        Self {
            id,
            res: None,
            exception: Some(exception),
        }
    }
}

/// Concrete request and response channels for one RPC name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RpcTopics {
    /// Request channel.
    pub request: String,
    /// Response channel.
    pub response: String,
}

/// Build `/rpc/<name>/{req,res}` channels.
///
/// Names that would exceed the LCM 63-byte channel limit use the same
/// SHA-1/base62 `short_id` algorithm as the Python implementation.
pub fn rpc_topics(name: &str) -> RpcResult<RpcTopics> {
    if name.is_empty() {
        return Err(RpcError::Transport("rpc name must not be empty".into()));
    }
    if name.contains('\0') {
        return Err(RpcError::Transport("rpc name contains NUL".into()));
    }

    let response = format!("/rpc/{name}/res");
    let effective_name = if response.len() > LCM_MAX_CHANNEL_NAME_LENGTH {
        short_id(name)
    } else {
        name.to_owned()
    };
    Ok(RpcTopics {
        request: format!("/rpc/{effective_name}/req"),
        response: format!("/rpc/{effective_name}/res"),
    })
}

fn short_id(input: &str) -> String {
    const ALPHABET: &[u8; 62] = b"0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    let digest = sha1(input.as_bytes());
    let mut prefix = [0_u8; 16];
    prefix.copy_from_slice(&digest[..16]);
    let mut number = u128::from_be_bytes(prefix);
    let mut reversed = Vec::new();
    while number > 0 || reversed.len() < 18 {
        reversed.push(ALPHABET[(number % 62) as usize]);
        number /= 62;
    }
    reversed.reverse();
    String::from_utf8(reversed.into_iter().take(18).collect()).expect("base62 alphabet is ASCII")
}

fn sha1(input: &[u8]) -> [u8; 20] {
    let bit_len = (input.len() as u64) * 8;
    let mut padded = input.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend_from_slice(&bit_len.to_be_bytes());

    let mut h0 = 0x6745_2301_u32;
    let mut h1 = 0xEFCD_AB89_u32;
    let mut h2 = 0x98BA_DCFE_u32;
    let mut h3 = 0x1032_5476_u32;
    let mut h4 = 0xC3D2_E1F0_u32;

    for chunk in padded.chunks_exact(64) {
        let mut words = [0_u32; 80];
        for (index, bytes) in chunk.chunks_exact(4).enumerate() {
            words[index] = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        }
        for index in 16..80 {
            words[index] =
                (words[index - 3] ^ words[index - 8] ^ words[index - 14] ^ words[index - 16])
                    .rotate_left(1);
        }

        let (mut a, mut b, mut c, mut d, mut e) = (h0, h1, h2, h3, h4);
        for (index, word) in words.iter().enumerate() {
            let (f, k) = match index {
                0..=19 => ((b & c) | ((!b) & d), 0x5A82_7999),
                20..=39 => (b ^ c ^ d, 0x6ED9_EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1B_BCDC),
                _ => (b ^ c ^ d, 0xCA62_C1D6),
            };
            let next = a
                .rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(*word);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = next;
        }
        h0 = h0.wrapping_add(a);
        h1 = h1.wrapping_add(b);
        h2 = h2.wrapping_add(c);
        h3 = h3.wrapping_add(d);
        h4 = h4.wrapping_add(e);
    }

    let mut digest = [0_u8; 20];
    for (target, word) in digest.chunks_exact_mut(4).zip([h0, h1, h2, h3, h4]) {
        target.copy_from_slice(&word.to_be_bytes());
    }
    digest
}

/// A live subscription for raw RPC payload bytes.
#[async_trait]
pub trait RpcSubscription: Send {
    /// Receive the next payload.
    async fn recv(&mut self) -> RpcResult<Vec<u8>>;

    /// Explicitly release the underlying subscription.
    ///
    /// Transports that can acknowledge unsubscribe should override this method.
    /// The default preserves compatibility with subscriptions whose drop is
    /// already synchronous.
    async fn unsubscribe(&mut self) -> RpcResult<()> {
        Ok(())
    }
}

/// Minimal byte-oriented pub/sub contract required by RPC.
///
/// The contract is intentionally channel + bytes: Python `PickleLCM` RPC does
/// not use a generated LCM schema, so forcing a typed message fingerprint at
/// this boundary would reject valid RPC payloads.
#[async_trait]
pub trait RpcTransport: Send + Sync + 'static {
    /// Publish one payload to a concrete channel.
    async fn publish(&self, channel: &str, payload: Vec<u8>) -> RpcResult<()>;
    /// Subscribe to a concrete channel.
    async fn subscribe(&self, channel: &str) -> RpcResult<Box<dyn RpcSubscription>>;
    /// Stable backend name used by diagnostics and server handles.
    fn backend_name(&self) -> &'static str;
}

/// Adapter from the safe single-owner [`nuwax_lcm::LcmContext`] to RPC bytes.
#[cfg(feature = "link-liblcm")]
#[derive(Clone)]
pub struct LcmRpcTransport {
    context: Arc<nuwax_lcm::LcmContext>,
}

#[cfg(feature = "link-liblcm")]
impl LcmRpcTransport {
    /// Wrap an existing process-wide LCM context.
    #[must_use]
    pub fn from_context(context: Arc<nuwax_lcm::LcmContext>) -> Self {
        Self { context }
    }

    /// Construct a real liblcm context from a provider URL.
    pub fn from_provider(provider: &str) -> RpcResult<Self> {
        let context = nuwax_lcm::LcmContext::new(provider)
            .map_err(|error| RpcError::BackendUnavailable(error.to_string()))?;
        Ok(Self::from_context(Arc::new(context)))
    }
}

#[cfg(feature = "link-liblcm")]
struct LcmRpcSubscription {
    subscription: nuwax_lcm::LcmSubscription,
}

#[cfg(feature = "link-liblcm")]
#[async_trait]
impl RpcSubscription for LcmRpcSubscription {
    async fn recv(&mut self) -> RpcResult<Vec<u8>> {
        loop {
            match self.subscription.receiver.recv().await {
                Ok(message) => return Ok(message.payload.to_vec()),
                Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    return Err(RpcError::Transport("LCM subscription closed".into()));
                }
            }
        }
    }

    async fn unsubscribe(&mut self) -> RpcResult<()> {
        self.subscription
            .unsubscribe()
            .await
            .map_err(|error| RpcError::Transport(error.to_string()))
    }
}

#[cfg(feature = "link-liblcm")]
#[async_trait]
impl RpcTransport for LcmRpcTransport {
    async fn publish(&self, channel: &str, payload: Vec<u8>) -> RpcResult<()> {
        self.context
            .publish(channel, payload.into())
            .await
            .map_err(|error| RpcError::Transport(error.to_string()))
    }

    async fn subscribe(&self, channel: &str) -> RpcResult<Box<dyn RpcSubscription>> {
        let subscription = self
            .context
            .subscribe(channel)
            .await
            .map_err(|error| RpcError::Transport(error.to_string()))?;
        Ok(Box::new(LcmRpcSubscription { subscription }))
    }

    fn backend_name(&self) -> &'static str {
        "real-lcm"
    }
}

type PendingResponse = oneshot::Sender<RpcResult<RpcResponse>>;
type PendingResponses = Arc<StdMutex<HashMap<u64, PendingResponse>>>;

fn lock_pending(
    pending: &StdMutex<HashMap<u64, PendingResponse>>,
) -> StdMutexGuard<'_, HashMap<u64, PendingResponse>> {
    pending
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

struct PendingCallGuard {
    pending: PendingResponses,
    key: u64,
}

impl PendingCallGuard {
    fn register(pending: &PendingResponses, key: u64, sender: PendingResponse) -> Self {
        lock_pending(pending).insert(key, sender);
        Self {
            pending: Arc::clone(pending),
            key,
        }
    }
}

impl Drop for PendingCallGuard {
    fn drop(&mut self) {
        lock_pending(&self.pending).remove(&self.key);
    }
}

/// A transport-backed client bound to one RPC name.
#[derive(Clone)]
pub struct RpcClient {
    inner: Arc<RpcClientInner>,
}

struct RpcClientInner {
    transport: Arc<dyn RpcTransport>,
    rpc_name: String,
    topics: RpcTopics,
    default_timeout: Duration,
    pending: PendingResponses,
    response_task: Mutex<Option<JoinHandle<()>>>,
}

impl Drop for RpcClientInner {
    fn drop(&mut self) {
        if let Some(task) = self.response_task.get_mut().take() {
            task.abort();
        }
    }
}

impl RpcClient {
    /// Construct a client with [`DEFAULT_RPC_TIMEOUT`].
    pub fn new(transport: Arc<dyn RpcTransport>, rpc_name: impl Into<String>) -> RpcResult<Self> {
        Self::with_timeout(transport, rpc_name, DEFAULT_RPC_TIMEOUT)
    }

    /// Construct a client with an explicit default timeout.
    pub fn with_timeout(
        transport: Arc<dyn RpcTransport>,
        rpc_name: impl Into<String>,
        default_timeout: Duration,
    ) -> RpcResult<Self> {
        let rpc_name = rpc_name.into();
        let topics = rpc_topics(&rpc_name)?;
        Ok(Self {
            inner: Arc::new(RpcClientInner {
                transport,
                rpc_name,
                topics,
                default_timeout,
                pending: Arc::new(StdMutex::new(HashMap::new())),
                response_task: Mutex::new(None),
            }),
        })
    }

    /// Bound RPC name.
    #[must_use]
    pub fn rpc_name(&self) -> &str {
        &self.inner.rpc_name
    }

    /// Number of correlated calls currently awaiting a response.
    ///
    /// This is intended for health diagnostics and cancellation verification.
    #[must_use]
    pub fn pending_call_count(&self) -> usize {
        lock_pending(&self.inner.pending).len()
    }

    /// Call the remote handler using this client's default timeout.
    pub async fn call_default(&self, args: RpcArgs) -> RpcResult<Value> {
        self.call(args, self.inner.default_timeout).await
    }

    /// 在本次调用范围内拥有响应订阅，不创建常驻分发任务；适合有界分块补取。
    pub async fn call_scoped(&self, args: RpcArgs, timeout: Duration) -> RpcResult<Value> {
        let mut subscription = self.inner.transport.subscribe(&self.inner.topics.response).await?;
        let id = next_correlation_id();
        let request = RpcRequest { name: self.inner.rpc_name.clone(), args, id: Some(id) };
        let result = tokio::time::timeout(timeout, async {
            self.inner.transport.publish(&self.inner.topics.request, pickle::encode_request(&request)?).await?;
            loop {
                let payload = subscription.recv().await?;
                let response = pickle::decode_response(&payload)?;
                if response.id.map(f64::to_bits) != Some(id.to_bits()) { continue; }
                if let Some(exception) = response.exception { return Err(RpcError::RemoteException(exception)); }
                return Ok(response.res.unwrap_or(Value::Null));
            }
        }).await;
        result.unwrap_or_else(|_| Err(RpcError::Timeout { timeout_ms: duration_millis(timeout) }))
    }

    /// Call the remote handler and wait for its correlated response.
    pub async fn call(&self, args: RpcArgs, timeout: Duration) -> RpcResult<Value> {
        self.ensure_response_task().await?;

        let id = next_correlation_id();
        let key = id.to_bits();
        let request = RpcRequest {
            name: self.inner.rpc_name.clone(),
            args,
            id: Some(id),
        };
        let payload = pickle::encode_request(&request)?;
        let (response_tx, response_rx) = oneshot::channel();
        let _pending_call = PendingCallGuard::register(&self.inner.pending, key, response_tx);

        self.inner
            .transport
            .publish(&self.inner.topics.request, payload)
            .await?;

        let response = match tokio::time::timeout(timeout, response_rx).await {
            Ok(Ok(result)) => result?,
            Ok(Err(_)) => {
                return Err(RpcError::Transport("response dispatcher stopped".into()));
            }
            Err(_) => {
                return Err(RpcError::Timeout {
                    timeout_ms: duration_millis(timeout),
                });
            }
        };

        if let Some(exception) = response.exception {
            return Err(RpcError::RemoteException(exception));
        }
        Ok(response.res.unwrap_or(Value::Null))
    }

    /// Publish a fire-and-forget request with no response correlation ID.
    pub async fn call_nowait(&self, args: RpcArgs) -> RpcResult<()> {
        let request = RpcRequest {
            name: self.inner.rpc_name.clone(),
            args,
            id: None,
        };
        let payload = pickle::encode_request(&request)?;
        self.inner
            .transport
            .publish(&self.inner.topics.request, payload)
            .await
    }

    async fn ensure_response_task(&self) -> RpcResult<()> {
        let mut task_guard = self.inner.response_task.lock().await;
        if task_guard.as_ref().is_some_and(|task| !task.is_finished()) {
            return Ok(());
        }
        if let Some(finished) = task_guard.take() {
            finished.abort();
        }

        let mut subscription = self
            .inner
            .transport
            .subscribe(&self.inner.topics.response)
            .await?;
        let pending = Arc::clone(&self.inner.pending);
        *task_guard = Some(tokio::spawn(async move {
            loop {
                let payload = match subscription.recv().await {
                    Ok(payload) => payload,
                    Err(error) => {
                        let detail = error.to_string();
                        let mut waiting = lock_pending(&pending);
                        for (_, sender) in waiting.drain() {
                            let _ = sender.send(Err(RpcError::Transport(detail.clone())));
                        }
                        return;
                    }
                };
                let response = match pickle::decode_response(&payload) {
                    Ok(response) => response,
                    Err(_) => continue,
                };
                let Some(id) = response.id else {
                    continue;
                };
                if let Some(sender) = lock_pending(&pending).remove(&id.to_bits()) {
                    let _ = sender.send(Ok(response));
                }
            }
        }));
        Ok(())
    }
}

fn next_correlation_id() -> f64 {
    const MAX_EXACT_F64_INTEGER: u64 = (1_u64 << 53) - 1;
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let counter = RPC_ID_COUNTER.fetch_add(1, Ordering::Relaxed);
    let mut material = [0_u8; 28];
    material[..16].copy_from_slice(&timestamp.to_be_bytes());
    material[16..20].copy_from_slice(&std::process::id().to_be_bytes());
    material[20..].copy_from_slice(&counter.to_be_bytes());
    let digest = sha1(&material);
    let mut prefix = [0_u8; 8];
    prefix.copy_from_slice(&digest[..8]);
    let id = u64::from_be_bytes(prefix) & MAX_EXACT_F64_INTEGER;
    id.max(1) as f64
}

fn duration_millis(duration: Duration) -> u64 {
    duration.as_millis().min(u128::from(u64::MAX)) as u64
}

/// Runtime limits for one RPC server subscription.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RpcServerConfig {
    /// Maximum number of handlers that may execute simultaneously.
    pub max_concurrent_requests: usize,
}

impl RpcServerConfig {
    fn validate(self) -> RpcResult<Self> {
        if self.max_concurrent_requests == 0 {
            return Err(RpcError::InvalidServerConfig(
                "max_concurrent_requests must be greater than zero".into(),
            ));
        }
        Ok(self)
    }
}

impl Default for RpcServerConfig {
    fn default() -> Self {
        Self {
            max_concurrent_requests: DEFAULT_RPC_MAX_CONCURRENCY,
        }
    }
}

/// Handle for a running RPC server subscription.
pub struct RpcServerHandle {
    /// RPC name served by this handle.
    pub service_name: String,
    /// Backend implementation name.
    pub backend: &'static str,
    shutdown: Option<oneshot::Sender<()>>,
    task: Option<JoinHandle<RpcResult<()>>>,
}

impl std::fmt::Debug for RpcServerHandle {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("RpcServerHandle")
            .field("service_name", &self.service_name)
            .field("backend", &self.backend)
            .field("running", &self.task.is_some())
            .finish()
    }
}

impl RpcServerHandle {
    fn running(
        service_name: impl Into<String>,
        backend: &'static str,
        shutdown: oneshot::Sender<()>,
        task: JoinHandle<RpcResult<()>>,
    ) -> Self {
        Self {
            service_name: service_name.into(),
            backend,
            shutdown: Some(shutdown),
            task: Some(task),
        }
    }

    /// Stop the subscription and cancel in-flight handler tasks.
    pub async fn stop(mut self) -> RpcResult<()> {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(task) = self.task.take() {
            task.await
                .map_err(|error| RpcError::ServerTask(error.to_string()))??;
        }
        Ok(())
    }

    /// Wait until the subscription fails or a handler response cannot be sent.
    ///
    /// Unlike [`Self::stop`], this does not request shutdown. Transport,
    /// serialization, publication, and handler-task failures are returned.
    pub async fn wait(mut self) -> RpcResult<()> {
        let shutdown = self.shutdown.take();
        let result = match self.task.take() {
            Some(task) => task
                .await
                .map_err(|error| RpcError::ServerTask(error.to_string()))?,
            None => Ok(()),
        };
        drop(shutdown);
        result
    }
}

impl Drop for RpcServerHandle {
    fn drop(&mut self) {
        if let Some(shutdown) = self.shutdown.take() {
            let _ = shutdown.send(());
        }
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }
}

/// Lifecycle handle for a module-style set of RPC methods.
///
/// The original runtime registers one LCM subscription per
/// `<ModuleClass>/<method>`. This group preserves that wire layout while making
/// startup rollback and acknowledged shutdown a single operation.
#[derive(Debug)]
pub struct RpcServerGroup {
    service_names: Vec<String>,
    backend: &'static str,
    handles: Vec<RpcServerHandle>,
}

impl RpcServerGroup {
    /// Registered RPC names in startup order.
    #[must_use]
    pub fn service_names(&self) -> &[String] {
        &self.service_names
    }

    /// Backend shared by every method subscription.
    #[must_use]
    pub fn backend(&self) -> &'static str {
        self.backend
    }

    /// Number of live method subscriptions in this group.
    #[must_use]
    pub fn len(&self) -> usize {
        self.handles.len()
    }

    /// Whether this group contains no method subscriptions.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.handles.is_empty()
    }

    /// Stop every method and wait for each transport unsubscribe acknowledgement.
    ///
    /// All handles are stopped even if one reports an error; the first error is
    /// returned after cleanup completes.
    pub async fn stop(self) -> RpcResult<()> {
        let mut first_error = None;
        for handle in self.handles {
            if let Err(error) = handle.stop().await {
                first_error.get_or_insert(error);
            }
        }
        match first_error {
            Some(error) => Err(error),
            None => Ok(()),
        }
    }
}

/// RPC handler.
#[async_trait]
pub trait RpcHandler: Send + Sync + 'static {
    /// Handle one request.
    async fn handle(&self, req: RpcRequest) -> RpcResponse;
}

/// Handler registry with local and transport-backed dispatch.
#[derive(Clone)]
pub struct RpcBroker {
    handlers: Arc<RwLock<HashMap<String, Arc<dyn RpcHandler>>>>,
}

impl RpcBroker {
    /// Construct an empty broker.
    #[must_use]
    pub fn new() -> Self {
        Self {
            handlers: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Register or replace one handler.
    pub async fn register(
        &self,
        name: impl Into<String>,
        handler: Box<dyn RpcHandler>,
    ) -> ModuleResult<()> {
        self.handlers
            .write()
            .await
            .insert(name.into(), Arc::from(handler));
        Ok(())
    }

    /// Dispatch a request in-process.
    pub async fn call(&self, req: RpcRequest) -> RpcResult<RpcResponse> {
        let handler = self.handlers.read().await.get(&req.name).cloned();
        match handler {
            Some(handler) => Ok(handler.handle(req).await),
            None => Err(RpcError::MethodNotFound(req.name)),
        }
    }

    /// Convenience in-process call using positional arguments only.
    pub async fn call_method(&self, method: &str, args: Vec<Value>) -> RpcResult<Value> {
        let response = self
            .call(RpcRequest {
                name: method.into(),
                args: (args, HashMap::new()),
                id: None,
            })
            .await?;
        if let Some(exception) = response.exception {
            let message = exception
                .get("message")
                .and_then(Value::as_str)
                .unwrap_or("handler returned exception")
                .to_owned();
            return Err(RpcError::Handler(message));
        }
        Ok(response.res.unwrap_or(Value::Null))
    }

    /// Serve one registered RPC name over a byte-oriented pub/sub transport.
    ///
    /// Call this once per registered RPC name, as Python `serve_rpc` does.
    /// The default concurrency bound matches the original Python implementation.
    pub async fn serve(
        &self,
        transport: Arc<dyn RpcTransport>,
        service_name: &str,
    ) -> RpcResult<RpcServerHandle> {
        self.serve_with_config(transport, service_name, RpcServerConfig::default())
            .await
    }

    /// Serve one RPC name with explicit handler concurrency limits.
    ///
    /// The registry lock is released before awaiting handlers. Requests above
    /// `max_concurrent_requests` remain in the transport queue until capacity is
    /// available, preserving nested-call support without unbounded task growth.
    pub async fn serve_with_config(
        &self,
        transport: Arc<dyn RpcTransport>,
        service_name: &str,
        config: RpcServerConfig,
    ) -> RpcResult<RpcServerHandle> {
        let config = config.validate()?;
        let handler = self
            .handlers
            .read()
            .await
            .get(service_name)
            .cloned()
            .ok_or_else(|| RpcError::MethodNotFound(service_name.to_owned()))?;
        let topics = rpc_topics(service_name)?;
        let mut subscription = transport.subscribe(&topics.request).await?;
        let backend = transport.backend_name();
        let service_name_owned = service_name.to_owned();
        let response_channel = topics.response;
        let (shutdown_tx, mut shutdown_rx) = oneshot::channel();

        let task = tokio::spawn(async move {
            run_rpc_server(
                &mut *subscription,
                handler,
                transport,
                service_name_owned,
                response_channel,
                config.max_concurrent_requests,
                &mut shutdown_rx,
            )
            .await
        });

        Ok(RpcServerHandle::running(
            service_name,
            backend,
            shutdown_tx,
            task,
        ))
    }

    /// Serve multiple registered names as one module lifecycle.
    ///
    /// Startup is transactional: if any subscription fails, all subscriptions
    /// started earlier in this call are stopped before the error is returned.
    pub async fn serve_many<I, S>(
        &self,
        transport: Arc<dyn RpcTransport>,
        service_names: I,
    ) -> RpcResult<RpcServerGroup>
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        let service_names: Vec<String> = service_names.into_iter().map(Into::into).collect();
        if service_names.is_empty() {
            return Err(RpcError::InvalidServerConfig(
                "at least one RPC service name is required".into(),
            ));
        }

        let mut unique_names = HashSet::with_capacity(service_names.len());
        for service_name in &service_names {
            if !unique_names.insert(service_name.as_str()) {
                return Err(RpcError::InvalidServerConfig(format!(
                    "duplicate RPC service name: {service_name}"
                )));
            }
        }

        let backend = transport.backend_name();
        let mut handles = Vec::with_capacity(service_names.len());
        for service_name in &service_names {
            match self.serve(Arc::clone(&transport), service_name).await {
                Ok(handle) => handles.push(handle),
                Err(error) => {
                    for handle in handles {
                        let _ = handle.stop().await;
                    }
                    return Err(error);
                }
            }
        }

        Ok(RpcServerGroup {
            service_names,
            backend,
            handles,
        })
    }
}

async fn run_rpc_server(
    subscription: &mut dyn RpcSubscription,
    handler: Arc<dyn RpcHandler>,
    transport: Arc<dyn RpcTransport>,
    service_name: String,
    response_channel: String,
    max_concurrent_requests: usize,
    shutdown: &mut oneshot::Receiver<()>,
) -> RpcResult<()> {
    let mut calls = JoinSet::new();
    let mut result = loop {
        if calls.len() >= max_concurrent_requests {
            tokio::select! {
                biased;
                completed = calls.join_next() => {
                    if let Err(error) = completed_handler(completed) {
                        break Err(error);
                    }
                }
                _ = &mut *shutdown => break Ok(()),
            }
            continue;
        }

        tokio::select! {
            biased;
            completed = calls.join_next(), if !calls.is_empty() => {
                if let Err(error) = completed_handler(completed) {
                    break Err(error);
                }
            }
            _ = &mut *shutdown => break Ok(()),
            payload = subscription.recv() => {
                let payload = match payload {
                    Ok(payload) => payload,
                    Err(error) => break Err(error),
                };
                let request = match pickle::decode_request(&payload) {
                    Ok(request) => request,
                    Err(_) => continue,
                };
                if request.name != service_name {
                    continue;
                }

                calls.spawn(execute_request(
                    Arc::clone(&handler),
                    Arc::clone(&transport),
                    response_channel.clone(),
                    request,
                ));
            }
        }
    };

    calls.abort_all();
    while let Some(completed) = calls.join_next().await {
        match completed {
            Ok(Err(error)) if result.is_ok() => result = Err(error),
            Err(error) if !error.is_cancelled() && result.is_ok() => {
                result = Err(RpcError::ServerTask(error.to_string()));
            }
            _ => {}
        }
    }
    if let Err(error) = subscription.unsubscribe().await {
        if result.is_ok() {
            result = Err(error);
        }
    }
    result
}

fn completed_handler(
    completed: Option<Result<RpcResult<()>, tokio::task::JoinError>>,
) -> RpcResult<()> {
    match completed {
        Some(Ok(result)) => result,
        Some(Err(error)) => Err(RpcError::ServerTask(error.to_string())),
        None => Err(RpcError::ServerTask(
            "handler task set ended unexpectedly".into(),
        )),
    }
}

async fn execute_request(
    handler: Arc<dyn RpcHandler>,
    transport: Arc<dyn RpcTransport>,
    response_channel: String,
    request: RpcRequest,
) -> RpcResult<()> {
    execute_request_with_encoder(
        handler,
        transport,
        response_channel,
        request,
        pickle::encode_response,
    )
    .await
}

async fn execute_request_with_encoder(
    handler: Arc<dyn RpcHandler>,
    transport: Arc<dyn RpcTransport>,
    response_channel: String,
    request: RpcRequest,
    encode_response: impl FnOnce(&RpcResponse) -> RpcResult<Vec<u8>>,
) -> RpcResult<()> {
    let request_id = request.id;
    let mut response = handler.handle(request).await;
    let Some(request_id) = request_id else {
        return Ok(());
    };
    response.id = Some(request_id);
    let payload = encode_response(&response)?;
    transport.publish(&response_channel, payload).await
}

impl Default for RpcBroker {
    fn default() -> Self {
        Self::new()
    }
}

/// Placeholder namespace retained for the future registration macro crate.
#[doc(hidden)]
pub mod nuwax_macros {}

#[cfg(test)]
mod tests {
    use super::*;

    struct EchoHandler;

    #[async_trait]
    impl RpcHandler for EchoHandler {
        async fn handle(&self, req: RpcRequest) -> RpcResponse {
            let positional = req.args.0.first().cloned().unwrap_or(Value::Null);
            RpcResponse::ok(
                req.id,
                serde_json::json!({"positional": positional, "keyword": req.args.1}),
            )
        }
    }

    struct ThrowHandler;

    #[async_trait]
    impl RpcHandler for ThrowHandler {
        async fn handle(&self, req: RpcRequest) -> RpcResponse {
            RpcResponse::error(
                req.id,
                serde_json::json!({"type": "ValueError", "message": "intentional failure"}),
            )
        }
    }

    struct NoopTransport;

    #[async_trait]
    impl RpcTransport for NoopTransport {
        async fn publish(&self, _channel: &str, _payload: Vec<u8>) -> RpcResult<()> {
            Ok(())
        }

        async fn subscribe(&self, _channel: &str) -> RpcResult<Box<dyn RpcSubscription>> {
            Err(RpcError::Transport("not used by this test".into()))
        }

        fn backend_name(&self) -> &'static str {
            "noop"
        }
    }

    #[tokio::test]
    async fn broker_register_and_call() {
        let broker = RpcBroker::new();
        broker
            .register("echo", Box::new(EchoHandler))
            .await
            .unwrap();
        let response = broker
            .call(RpcRequest {
                name: "echo".into(),
                args: (vec![serde_json::json!(1.0)], HashMap::new()),
                id: Some(42.0),
            })
            .await
            .unwrap();
        assert_eq!(response.id, Some(42.0));
        assert_eq!(response.res.unwrap()["positional"], serde_json::json!(1.0));
    }

    #[tokio::test]
    async fn broker_method_not_found() {
        let broker = RpcBroker::new();
        let error = broker
            .call(RpcRequest {
                name: "missing".into(),
                args: (vec![], HashMap::new()),
                id: None,
            })
            .await
            .unwrap_err();
        assert!(matches!(error, RpcError::MethodNotFound(_)));
    }

    #[tokio::test]
    async fn broker_exception_path() {
        let broker = RpcBroker::new();
        broker
            .register("throw", Box::new(ThrowHandler))
            .await
            .unwrap();
        let error = broker.call_method("throw", vec![]).await.unwrap_err();
        assert!(matches!(error, RpcError::Handler(_)));
    }

    #[tokio::test]
    async fn response_encoding_error_is_propagated() {
        let handler: Arc<dyn RpcHandler> = Arc::new(EchoHandler);
        let transport: Arc<dyn RpcTransport> = Arc::new(NoopTransport);
        let request = RpcRequest {
            name: "encode-error".into(),
            args: (vec![Value::Null], HashMap::new()),
            id: Some(1.0),
        };
        let error = execute_request_with_encoder(
            handler,
            transport,
            "/rpc/encode-error/res".into(),
            request,
            |_| Err(RpcError::Serialize("injected encoding failure".into())),
        )
        .await
        .unwrap_err();
        assert!(matches!(
            error,
            RpcError::Serialize(ref detail) if detail == "injected encoding failure"
        ));
    }

    #[test]
    fn request_uses_name_field() {
        let request = RpcRequest {
            name: "foo".into(),
            args: (vec![], HashMap::new()),
            id: None,
        };
        let json = serde_json::to_value(request).unwrap();
        assert!(json.get("name").is_some());
        assert!(json.get("method").is_none());
    }

    #[test]
    fn topics_match_python_short_and_long_names() {
        assert_eq!(
            rpc_topics("Hello/say").unwrap(),
            RpcTopics {
                request: "/rpc/Hello/say/req".into(),
                response: "/rpc/Hello/say/res".into(),
            }
        );
        let long = "GreatyLongComplexExampleClassNameForTestingStuff/create";
        assert_eq!(
            rpc_topics(long).unwrap().response,
            "/rpc/2cudPuFGMJdWxM5KZb/res"
        );
    }
}
