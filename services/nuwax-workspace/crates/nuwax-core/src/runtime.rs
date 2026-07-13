//! Production module ports, transport fanout, lifecycle ownership, and introspection.

use crate::{Module, ModuleError, ModuleResult};
use std::any::{type_name, Any, TypeId};
use std::collections::{BTreeMap, HashMap};
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use thiserror::Error;
use tokio::sync::{mpsc, Mutex as AsyncMutex};

type PortCallback<T> = Arc<dyn Fn(T) + Send + Sync + 'static>;

/// Port or transport operation error.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum PortError {
    /// The port or transport has already stopped.
    #[error("port or transport is stopped")]
    Stopped,
    /// A receiver disappeared while a value was being delivered.
    #[error("port receiver is gone")]
    ReceiverGone,
    /// A module registered the same port direction/name more than once.
    #[error("duplicate {direction:?} port named {name}")]
    DuplicatePort {
        /// Conflicting port name.
        name: String,
        /// Conflicting port direction.
        direction: PortDirection,
    },
    /// Auto-connect found the same logical name with incompatible Rust types.
    #[error("port {name} has conflicting types {left} and {right}")]
    NameTypeConflict {
        /// Logical port name.
        name: String,
        /// First registered type.
        left: String,
        /// Conflicting type.
        right: String,
    },
    /// A type-erased connection was attempted between different value types.
    #[error("cannot connect {output_type} output to {input_type} input")]
    TypeMismatch {
        /// Input value type.
        input_type: String,
        /// Output value type.
        output_type: String,
    },
    /// A concrete transport backend reported an error.
    #[error("transport {transport} failed: {message}")]
    Transport {
        /// Transport label or topic.
        transport: String,
        /// Backend error detail.
        message: String,
    },
}

/// Direction of a typed module port.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum PortDirection {
    /// Module input.
    Input,
    /// Module output.
    Output,
}

/// Introspection snapshot for one typed port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    /// Logical name used by auto-connect and remapping layers.
    pub name: String,
    /// Fully qualified Rust value type.
    pub type_name: String,
    /// Input or output direction.
    pub direction: PortDirection,
    /// Number of live local or transport bindings.
    pub connection_count: usize,
    /// Whether the port owner has stopped it.
    pub stopped: bool,
}

/// Owned, idempotent unsubscribe/stop action.
///
/// Dropping the handle executes the action, matching the Disposable ownership
/// contract used by the original stream layer.
pub struct StopHandle {
    action: Option<Box<dyn FnOnce() + Send + 'static>>,
}

impl StopHandle {
    fn new(action: impl FnOnce() + Send + 'static) -> Self {
        Self {
            action: Some(Box::new(action)),
        }
    }

    /// Execute the stop action immediately. Repeated calls are no-ops.
    pub fn stop(&mut self) {
        if let Some(action) = self.action.take() {
            action();
        }
    }

    /// Whether this handle still owns a live action.
    #[must_use]
    pub fn is_active(&self) -> bool {
        self.action.is_some()
    }
}

impl Drop for StopHandle {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Typed transport extension point used by local and cross-process streams.
///
/// Real LCM or socket backends can implement this trait without changing the
/// module graph. Subscriptions return an owned `StopHandle` and therefore cannot
/// become detached callbacks.
pub trait PortTransport<T>: Send + Sync
where
    T: Clone + Send + Sync + 'static,
{
    /// Human-readable transport label or topic.
    fn label(&self) -> &str;

    /// Publish a value and return the number of callbacks that accepted it.
    ///
    /// # Errors
    /// Returns a backend-specific `PortError` when publication fails.
    fn publish(&self, value: T) -> Result<usize, PortError>;

    /// Subscribe a callback and return its owned unsubscribe handle.
    ///
    /// # Errors
    /// Returns `PortError::Stopped` if the backend no longer accepts subscriptions.
    fn subscribe(&self, callback: PortCallback<T>) -> Result<StopHandle, PortError>;

    /// Stop the transport owner and release all subscribers.
    ///
    /// Shared transports should only be stopped by their lifecycle owner; port
    /// bindings merely unsubscribe themselves.
    ///
    /// # Errors
    /// Returns a backend-specific error if teardown fails.
    fn stop(&self) -> Result<(), PortError>;
}

struct LocalTransportInner<T> {
    label: String,
    next_id: AtomicU64,
    stopped: AtomicBool,
    subscribers: Mutex<HashMap<u64, PortCallback<T>>>,
}

/// In-process fanout transport implementing the same owned subscription contract
/// as production transport adapters.
#[derive(Clone)]
pub struct LocalTransport<T> {
    inner: Arc<LocalTransportInner<T>>,
}

impl<T> LocalTransport<T>
where
    T: Clone + Send + Sync + 'static,
{
    /// Create a local transport with an introspection label.
    #[must_use]
    pub fn new(label: impl Into<String>) -> Self {
        Self {
            inner: Arc::new(LocalTransportInner {
                label: label.into(),
                next_id: AtomicU64::new(1),
                stopped: AtomicBool::new(false),
                subscribers: Mutex::new(HashMap::new()),
            }),
        }
    }

    /// Current subscriber count.
    #[must_use]
    pub fn subscriber_count(&self) -> usize {
        lock_unpoisoned(&self.inner.subscribers).len()
    }
}

impl<T> PortTransport<T> for LocalTransport<T>
where
    T: Clone + Send + Sync + 'static,
{
    fn label(&self) -> &str {
        &self.inner.label
    }

    fn publish(&self, value: T) -> Result<usize, PortError> {
        if self.inner.stopped.load(Ordering::Acquire) {
            return Err(PortError::Stopped);
        }
        let callbacks: Vec<_> = lock_unpoisoned(&self.inner.subscribers)
            .values()
            .cloned()
            .collect();
        for callback in &callbacks {
            callback(value.clone());
        }
        Ok(callbacks.len())
    }

    fn subscribe(&self, callback: PortCallback<T>) -> Result<StopHandle, PortError> {
        if self.inner.stopped.load(Ordering::Acquire) {
            return Err(PortError::Stopped);
        }
        let id = self.inner.next_id.fetch_add(1, Ordering::Relaxed);
        lock_unpoisoned(&self.inner.subscribers).insert(id, callback);
        let inner = Arc::downgrade(&self.inner);
        Ok(StopHandle::new(move || {
            if let Some(inner) = inner.upgrade() {
                lock_unpoisoned(&inner.subscribers).remove(&id);
            }
        }))
    }

    fn stop(&self) -> Result<(), PortError> {
        if !self.inner.stopped.swap(true, Ordering::AcqRel) {
            lock_unpoisoned(&self.inner.subscribers).clear();
        }
        Ok(())
    }
}

/// Delivery counts from one output publication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PublishReport {
    /// Local callback deliveries.
    pub local_deliveries: usize,
    /// Number of attached transport backends invoked.
    pub transports: usize,
    /// Sum of callback deliveries reported by attached transports.
    pub transport_deliveries: usize,
}

struct OutputPortInner<T> {
    name: String,
    local: LocalTransport<T>,
    next_transport_id: AtomicU64,
    transports: Mutex<HashMap<u64, Arc<dyn PortTransport<T>>>>,
    stopped: AtomicBool,
}

/// Typed module output with simultaneous local and transport fanout.
#[derive(Clone)]
pub struct OutputPort<T> {
    inner: Arc<OutputPortInner<T>>,
}

impl<T> OutputPort<T>
where
    T: Clone + Send + Sync + 'static,
{
    /// Create an output port with the logical name used by auto-connect.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        let name = name.into();
        Self {
            inner: Arc::new(OutputPortInner {
                local: LocalTransport::new(format!("local:{name}")),
                name,
                next_transport_id: AtomicU64::new(1),
                transports: Mutex::new(HashMap::new()),
                stopped: AtomicBool::new(false),
            }),
        }
    }

    /// Logical port name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    /// Publish to local subscribers and every attached transport backend.
    ///
    /// # Errors
    /// Returns the first local or transport error. Backends invoked before the
    /// failure may already have accepted the value.
    pub fn publish(&self, value: T) -> Result<PublishReport, PortError> {
        if self.inner.stopped.load(Ordering::Acquire) {
            return Err(PortError::Stopped);
        }
        let local_deliveries = self.inner.local.publish(value.clone())?;
        let transports: Vec<_> = lock_unpoisoned(&self.inner.transports)
            .values()
            .cloned()
            .collect();
        let mut transport_deliveries = 0usize;
        for transport in &transports {
            transport_deliveries = transport_deliveries
                .checked_add(transport.publish(value.clone())?)
                .ok_or_else(|| PortError::Transport {
                    transport: transport.label().to_string(),
                    message: "delivery count overflow".to_string(),
                })?;
        }
        Ok(PublishReport {
            local_deliveries,
            transports: transports.len(),
            transport_deliveries,
        })
    }

    /// Subscribe a local callback.
    ///
    /// # Errors
    /// Returns `PortError::Stopped` after port teardown.
    pub fn subscribe(
        &self,
        callback: impl Fn(T) + Send + Sync + 'static,
    ) -> Result<StopHandle, PortError> {
        self.inner.local.subscribe(Arc::new(callback))
    }

    /// Attach a production transport fanout and return its owned detach handle.
    ///
    /// The shared transport itself is not stopped when this binding is detached.
    ///
    /// # Errors
    /// Returns `PortError::Stopped` after port teardown.
    pub fn attach_transport(
        &self,
        transport: Arc<dyn PortTransport<T>>,
    ) -> Result<StopHandle, PortError> {
        if self.inner.stopped.load(Ordering::Acquire) {
            return Err(PortError::Stopped);
        }
        let id = self.inner.next_transport_id.fetch_add(1, Ordering::Relaxed);
        lock_unpoisoned(&self.inner.transports).insert(id, transport);
        let inner = Arc::downgrade(&self.inner);
        Ok(StopHandle::new(move || {
            if let Some(inner) = inner.upgrade() {
                lock_unpoisoned(&inner.transports).remove(&id);
            }
        }))
    }

    /// Stop this port, detach transports, and release local subscribers.
    pub fn stop(&self) {
        if !self.inner.stopped.swap(true, Ordering::AcqRel) {
            let _ = self.inner.local.stop();
            lock_unpoisoned(&self.inner.transports).clear();
        }
    }

    /// Current introspection snapshot.
    #[must_use]
    pub fn info(&self) -> PortInfo {
        PortInfo {
            name: self.inner.name.clone(),
            type_name: type_name::<T>().to_string(),
            direction: PortDirection::Output,
            connection_count: self.inner.local.subscriber_count()
                + lock_unpoisoned(&self.inner.transports).len(),
            stopped: self.inner.stopped.load(Ordering::Acquire),
        }
    }
}

enum InboxValue<T> {
    Value(T),
    Stop,
}

struct InputPortInner<T> {
    name: String,
    sender: mpsc::UnboundedSender<InboxValue<T>>,
    receiver: AsyncMutex<mpsc::UnboundedReceiver<InboxValue<T>>>,
    connections: AtomicUsize,
    stopped: AtomicBool,
}

/// Typed module input with owned local or transport bindings.
#[derive(Clone)]
pub struct InputPort<T> {
    inner: Arc<InputPortInner<T>>,
}

impl<T> InputPort<T>
where
    T: Clone + Send + Sync + 'static,
{
    /// Create an input port with the logical name used by auto-connect.
    #[must_use]
    pub fn new(name: impl Into<String>) -> Self {
        let (sender, receiver) = mpsc::unbounded_channel();
        Self {
            inner: Arc::new(InputPortInner {
                name: name.into(),
                sender,
                receiver: AsyncMutex::new(receiver),
                connections: AtomicUsize::new(0),
                stopped: AtomicBool::new(false),
            }),
        }
    }

    /// Logical port name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    /// Receive the next value, or `None` after port teardown.
    pub async fn recv(&self) -> Option<T> {
        if self.inner.stopped.load(Ordering::Acquire) {
            return None;
        }
        let mut receiver = self.inner.receiver.lock().await;
        match receiver.recv().await {
            Some(InboxValue::Value(value)) => Some(value),
            Some(InboxValue::Stop) | None => None,
        }
    }

    fn callback(&self) -> PortCallback<T> {
        let inner = Arc::downgrade(&self.inner);
        Arc::new(move |value| {
            if let Some(inner) = inner.upgrade() {
                if !inner.stopped.load(Ordering::Acquire) {
                    let _ = inner.sender.send(InboxValue::Value(value));
                }
            }
        })
    }

    fn own_subscription(&self, mut upstream: StopHandle) -> StopHandle {
        self.inner.connections.fetch_add(1, Ordering::AcqRel);
        let inner = Arc::downgrade(&self.inner);
        StopHandle::new(move || {
            upstream.stop();
            if let Some(inner) = inner.upgrade() {
                decrement_saturating(&inner.connections);
            }
        })
    }

    /// Explicitly connect an output to this input and return an owned disconnect handle.
    ///
    /// Explicit connections may use different logical names; `autoconnect` is the
    /// name-and-type matching layer.
    ///
    /// # Errors
    /// Returns `PortError::Stopped` if either port has stopped.
    pub fn connect(&self, output: &OutputPort<T>) -> Result<StopHandle, PortError> {
        if self.inner.stopped.load(Ordering::Acquire) {
            return Err(PortError::Stopped);
        }
        let callback = self.callback();
        let subscription = output.inner.local.subscribe(callback)?;
        Ok(self.own_subscription(subscription))
    }

    /// Bind this input to a production transport and return an owned unsubscribe handle.
    ///
    /// # Errors
    /// Returns a transport error when subscription cannot be established.
    pub fn bind_transport(
        &self,
        transport: Arc<dyn PortTransport<T>>,
    ) -> Result<StopHandle, PortError> {
        if self.inner.stopped.load(Ordering::Acquire) {
            return Err(PortError::Stopped);
        }
        let subscription = transport.subscribe(self.callback())?;
        Ok(self.own_subscription(subscription))
    }

    /// Stop this port and wake a pending receiver.
    pub fn stop(&self) {
        if !self.inner.stopped.swap(true, Ordering::AcqRel) {
            let _ = self.inner.sender.send(InboxValue::Stop);
        }
    }

    /// Current introspection snapshot.
    #[must_use]
    pub fn info(&self) -> PortInfo {
        PortInfo {
            name: self.inner.name.clone(),
            type_name: type_name::<T>().to_string(),
            direction: PortDirection::Input,
            connection_count: self.inner.connections.load(Ordering::Acquire),
            stopped: self.inner.stopped.load(Ordering::Acquire),
        }
    }
}

/// Typed remote stream facade backed by a `PortTransport` implementation.
#[derive(Clone)]
pub struct RemoteStream<T>
where
    T: Clone + Send + Sync + 'static,
{
    name: String,
    transport: Arc<dyn PortTransport<T>>,
}

impl<T> RemoteStream<T>
where
    T: Clone + Send + Sync + 'static,
{
    /// Create a named remote stream over an existing transport owner.
    #[must_use]
    pub fn new(name: impl Into<String>, transport: Arc<dyn PortTransport<T>>) -> Self {
        Self {
            name: name.into(),
            transport,
        }
    }

    /// Remote logical stream name.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Publish directly to the remote transport.
    ///
    /// # Errors
    /// Propagates the backend publication error.
    pub fn publish(&self, value: T) -> Result<usize, PortError> {
        self.transport.publish(value)
    }

    /// Bind a local input to this remote stream.
    ///
    /// # Errors
    /// Propagates the backend subscription error.
    pub fn bind_input(&self, input: &InputPort<T>) -> Result<StopHandle, PortError> {
        input.bind_transport(Arc::clone(&self.transport))
    }

    /// Attach a local output to publish on this remote stream.
    ///
    /// # Errors
    /// Returns `PortError::Stopped` after output teardown.
    pub fn bind_output(&self, output: &OutputPort<T>) -> Result<StopHandle, PortError> {
        output.attach_transport(Arc::clone(&self.transport))
    }
}

trait ErasedInput: Send + Sync {
    fn info(&self) -> PortInfo;
    fn value_type_id(&self) -> TypeId;
    fn connect_erased(&self, output: &dyn ErasedOutput) -> Result<StopHandle, PortError>;
    fn stop(&self);
}

trait ErasedOutput: Send + Sync {
    fn info(&self) -> PortInfo;
    fn value_type_id(&self) -> TypeId;
    fn as_any(&self) -> &dyn Any;
    fn stop(&self);
}

impl<T> ErasedInput for InputPort<T>
where
    T: Clone + Send + Sync + 'static,
{
    fn info(&self) -> PortInfo {
        self.info()
    }

    fn value_type_id(&self) -> TypeId {
        TypeId::of::<T>()
    }

    fn connect_erased(&self, output: &dyn ErasedOutput) -> Result<StopHandle, PortError> {
        let Some(output) = output.as_any().downcast_ref::<OutputPort<T>>() else {
            return Err(PortError::TypeMismatch {
                input_type: type_name::<T>().to_string(),
                output_type: output.info().type_name,
            });
        };
        self.connect(output)
    }

    fn stop(&self) {
        self.stop();
    }
}

impl<T> ErasedOutput for OutputPort<T>
where
    T: Clone + Send + Sync + 'static,
{
    fn info(&self) -> PortInfo {
        self.info()
    }

    fn value_type_id(&self) -> TypeId {
        TypeId::of::<T>()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn stop(&self) {
        self.stop();
    }
}

/// Heterogeneous typed ports owned by one module.
#[derive(Default)]
pub struct ModulePorts {
    inputs: Vec<Arc<dyn ErasedInput>>,
    outputs: Vec<Arc<dyn ErasedOutput>>,
}

impl ModulePorts {
    /// Create an empty module port registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Register an input port clone.
    ///
    /// # Errors
    /// Returns `PortError::DuplicatePort` for a repeated input name.
    pub fn register_input<T>(&mut self, input: InputPort<T>) -> Result<(), PortError>
    where
        T: Clone + Send + Sync + 'static,
    {
        if self
            .inputs
            .iter()
            .any(|port| port.info().name == input.name())
        {
            return Err(PortError::DuplicatePort {
                name: input.name().to_string(),
                direction: PortDirection::Input,
            });
        }
        self.inputs.push(Arc::new(input));
        Ok(())
    }

    /// Register an output port clone.
    ///
    /// # Errors
    /// Returns `PortError::DuplicatePort` for a repeated output name.
    pub fn register_output<T>(&mut self, output: OutputPort<T>) -> Result<(), PortError>
    where
        T: Clone + Send + Sync + 'static,
    {
        if self
            .outputs
            .iter()
            .any(|port| port.info().name == output.name())
        {
            return Err(PortError::DuplicatePort {
                name: output.name().to_string(),
                direction: PortDirection::Output,
            });
        }
        self.outputs.push(Arc::new(output));
        Ok(())
    }

    /// Sorted port introspection snapshot.
    #[must_use]
    pub fn introspect(&self) -> Vec<PortInfo> {
        let mut ports: Vec<_> = self
            .inputs
            .iter()
            .map(|port| port.info())
            .chain(self.outputs.iter().map(|port| port.info()))
            .collect();
        ports.sort_by(|left, right| {
            (left.name.as_str(), left.direction).cmp(&(right.name.as_str(), right.direction))
        });
        ports
    }

    fn stop_all(&self) {
        for input in &self.inputs {
            input.stop();
        }
        for output in &self.outputs {
            output.stop();
        }
    }
}

/// Owned collection of auto-connected local port subscriptions.
pub struct ConnectionSet {
    handles: Vec<StopHandle>,
}

impl ConnectionSet {
    /// Number of live connections initially created by auto-connect.
    #[must_use]
    pub fn len(&self) -> usize {
        self.handles.len()
    }

    /// Whether no connections were created.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.handles.is_empty()
    }

    /// Disconnect every connection immediately.
    pub fn stop(&mut self) {
        for handle in &mut self.handles {
            handle.stop();
        }
        self.handles.clear();
    }
}

impl Drop for ConnectionSet {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Auto-connect all module inputs and outputs sharing the same `(name, Rust type)`.
///
/// Multiple outputs fan into every matching input. A reused name with different
/// types is rejected before any connection is created, matching the original
/// coordinator's conflict gate.
///
/// # Errors
/// Returns a name/type conflict or concrete connection error.
pub fn autoconnect(modules: &[&ModulePorts]) -> Result<ConnectionSet, PortError> {
    let mut name_types: BTreeMap<String, (TypeId, String)> = BTreeMap::new();
    for info in modules.iter().flat_map(|module| module.introspect()) {
        let value_type_id = modules
            .iter()
            .flat_map(|module| {
                module
                    .inputs
                    .iter()
                    .map(|port| (port.info(), port.value_type_id()))
                    .chain(
                        module
                            .outputs
                            .iter()
                            .map(|port| (port.info(), port.value_type_id())),
                    )
            })
            .find_map(|(candidate, type_id)| (candidate == info).then_some(type_id))
            .expect("introspected port must have a registered type id");
        if let Some((existing_id, existing_name)) = name_types.get(&info.name) {
            if *existing_id != value_type_id {
                return Err(PortError::NameTypeConflict {
                    name: info.name,
                    left: existing_name.clone(),
                    right: info.type_name,
                });
            }
        } else {
            name_types.insert(info.name, (value_type_id, info.type_name));
        }
    }

    let outputs: Vec<_> = modules
        .iter()
        .flat_map(|module| module.outputs.iter().cloned())
        .collect();
    let mut handles = Vec::new();
    for input in modules
        .iter()
        .flat_map(|module| module.inputs.iter().cloned())
    {
        let input_info = input.info();
        for output in &outputs {
            let output_info = output.info();
            if input_info.name == output_info.name
                && input.value_type_id() == output.value_type_id()
            {
                handles.push(input.connect_erased(output.as_ref())?);
            }
        }
    }
    Ok(ConnectionSet { handles })
}

/// Managed module lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleState {
    /// Constructed but not started.
    Created,
    /// Underlying `Module::start` is running.
    Starting,
    /// Started successfully.
    Running,
    /// Underlying `Module::stop` is running.
    Stopping,
    /// Stopped and all owned connections released.
    Stopped,
    /// Start or stop returned an error.
    Failed,
}

/// Introspection snapshot for a managed module.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModuleInfo {
    /// Runtime module name.
    pub name: String,
    /// Current lifecycle state.
    pub state: LifecycleState,
    /// Sorted input/output metadata.
    pub ports: Vec<PortInfo>,
}

/// Module owner that couples lifecycle, port teardown, and connection ownership.
pub struct ManagedModule<M: Module> {
    name: String,
    module: Arc<M>,
    ports: ModulePorts,
    state: Mutex<LifecycleState>,
    connections: Mutex<Vec<ConnectionSet>>,
    bindings: Mutex<Vec<StopHandle>>,
}

impl<M: Module> ManagedModule<M> {
    /// Construct a managed module from an implementation and its registered ports.
    #[must_use]
    pub fn new(name: impl Into<String>, module: M, ports: ModulePorts) -> Self {
        Self {
            name: name.into(),
            module: Arc::new(module),
            ports,
            state: Mutex::new(LifecycleState::Created),
            connections: Mutex::new(Vec::new()),
            bindings: Mutex::new(Vec::new()),
        }
    }

    /// Registered module ports.
    #[must_use]
    pub fn ports(&self) -> &ModulePorts {
        &self.ports
    }

    /// Underlying module implementation.
    #[must_use]
    pub fn inner(&self) -> &M {
        &self.module
    }

    /// Transfer an auto-connected set into this module's lifecycle ownership.
    ///
    /// # Errors
    /// Returns `ModuleError::AlreadyStopped` after teardown begins.
    pub fn own_connections(&self, connections: ConnectionSet) -> ModuleResult<()> {
        if matches!(
            *lock_unpoisoned(&self.state),
            LifecycleState::Stopping | LifecycleState::Stopped | LifecycleState::Failed
        ) {
            return Err(ModuleError::AlreadyStopped);
        }
        lock_unpoisoned(&self.connections).push(connections);
        Ok(())
    }

    /// Transfer a transport binding into this module's lifecycle ownership.
    ///
    /// # Errors
    /// Returns `ModuleError::AlreadyStopped` after teardown begins.
    pub fn own_binding(&self, binding: StopHandle) -> ModuleResult<()> {
        if matches!(
            *lock_unpoisoned(&self.state),
            LifecycleState::Stopping | LifecycleState::Stopped | LifecycleState::Failed
        ) {
            return Err(ModuleError::AlreadyStopped);
        }
        lock_unpoisoned(&self.bindings).push(binding);
        Ok(())
    }

    /// Start the underlying module exactly once.
    ///
    /// # Errors
    /// Propagates `Module::start` or rejects an invalid lifecycle transition.
    pub async fn start(&self) -> ModuleResult<()> {
        {
            let mut state = lock_unpoisoned(&self.state);
            if *state != LifecycleState::Created {
                return Err(ModuleError::Other(format!(
                    "module {} cannot start from {state:?}",
                    self.name
                )));
            }
            *state = LifecycleState::Starting;
        }
        match self.module.start().await {
            Ok(()) => {
                *lock_unpoisoned(&self.state) = LifecycleState::Running;
                Ok(())
            }
            Err(error) => {
                *lock_unpoisoned(&self.state) = LifecycleState::Failed;
                self.release_owned_resources();
                self.ports.stop_all();
                Err(error)
            }
        }
    }

    /// Stop the module and release every owned connection/binding and port.
    ///
    /// Resources are released even if `Module::stop` returns an error.
    ///
    /// # Errors
    /// Propagates `Module::stop` or returns `ModuleError::AlreadyStopped`.
    pub async fn stop(&self) -> ModuleResult<()> {
        {
            let mut state = lock_unpoisoned(&self.state);
            if matches!(*state, LifecycleState::Stopping | LifecycleState::Stopped) {
                return Err(ModuleError::AlreadyStopped);
            }
            if *state != LifecycleState::Running {
                return Err(ModuleError::Other(format!(
                    "module {} cannot stop from {state:?}",
                    self.name
                )));
            }
            *state = LifecycleState::Stopping;
        }

        let result = self.module.stop().await;
        self.release_owned_resources();
        self.ports.stop_all();
        *lock_unpoisoned(&self.state) = if result.is_ok() {
            LifecycleState::Stopped
        } else {
            LifecycleState::Failed
        };
        result
    }

    /// Current module and port introspection snapshot.
    #[must_use]
    pub fn introspect(&self) -> ModuleInfo {
        ModuleInfo {
            name: self.name.clone(),
            state: *lock_unpoisoned(&self.state),
            ports: self.ports.introspect(),
        }
    }

    fn release_owned_resources(&self) {
        lock_unpoisoned(&self.connections).clear();
        lock_unpoisoned(&self.bindings).clear();
    }
}

fn decrement_saturating(counter: &AtomicUsize) {
    let mut current = counter.load(Ordering::Acquire);
    while current > 0 {
        match counter.compare_exchange_weak(
            current,
            current - 1,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => return,
            Err(observed) => current = observed,
        }
    }
}

fn lock_unpoisoned<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}
