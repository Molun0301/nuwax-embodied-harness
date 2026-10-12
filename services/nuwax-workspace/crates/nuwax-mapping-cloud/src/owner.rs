//! 单任务 mapping-cloud 生命周期 owner。

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use thiserror::Error;
use tokio::sync::{watch, Mutex};
use tokio::task::JoinHandle;
use tokio::time::{timeout, timeout_at, Instant};

use crate::{
    adapt_assigned_wgs84_goal_intent, AssignedGeoreferenceV3, AssignedMapIdentityV3,
    AssignedMappingCloudGoalIntentV3, CloudError, CloudQueryIdentityV1, GoalAdapterError,
    GoogleMapsClient, LatLng, OsmClient,
};

use crate::goal_adapter::validate_assigned_goal_inputs;

/// Owner 使用的真实地图查询后端。
#[derive(Clone)]
pub enum MappingCloudProvider {
    /// Google Maps 文本地点查询，使用首个结果。
    GoogleTextSearch(Arc<GoogleMapsClient>),
    /// OpenStreetMap Nominatim 查询，使用首个结果。
    OsmNominatim(Arc<OsmClient>),
    /// In-memory provider for assigned lifecycle tests; absent from production.
    #[cfg(test)]
    Controlled {
        /// Result returned after release.
        result: Result<Option<LatLng>, String>,
        /// Number of started provider calls.
        calls: Arc<std::sync::atomic::AtomicUsize>,
        /// Signal that the provider future was polled.
        started: Arc<tokio::sync::Notify>,
        /// Hold the provider future until released or cancelled.
        release: Arc<tokio::sync::Notify>,
    },
}

impl MappingCloudProvider {
    fn identity(&self) -> &'static str {
        match self {
            Self::GoogleTextSearch(_) => "google-text-search",
            Self::OsmNominatim(_) => "osm-nominatim",
            #[cfg(test)]
            Self::Controlled { .. } => "assigned-controlled-test",
        }
    }

    async fn query(&self, text: &str) -> Result<Option<LatLng>, CloudError> {
        match self {
            Self::GoogleTextSearch(client) => client.text_search(text).await.map(|items| {
                items.into_iter().next().map(|item| LatLng {
                    lat: item.lat,
                    lng: item.lng,
                })
            }),
            Self::OsmNominatim(client) => client.nominatim_search(text).await.map(|items| {
                items.into_iter().next().map(|item| LatLng {
                    lat: item.lat,
                    lng: item.lng,
                })
            }),
            #[cfg(test)]
            Self::Controlled { result, calls, started, release } => {
                calls.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                started.notify_one();
                release.notified().await;
                result.clone().map_err(CloudError::Parse)
            }
        }
    }
}

/// 一个受监督 cloud 查询请求。
#[derive(Clone)]
pub struct MappingCloudRequest {
    /// 稳定请求及生产者身份。
    pub query_identity: CloudQueryIdentityV1,
    /// 发送给真实 provider 的非空查询文本。
    pub query_text: String,
    /// 查询结果必须绑定的 active-map 快照。
    pub active_map: AssignedMapIdentityV3,
    /// 已校验且由原 assigned asset ID 绑定的 immutable 地理参考。
    pub georeference: Arc<AssignedGeoreferenceV3>,
    /// 调用者允许的端到端时长。
    pub caller_timeout: Duration,
    /// provider 单次调用允许的时长。
    pub provider_timeout: Duration,
}

/// Bounded business fields retained by the existing lifecycle owner.
/// Query source time is deliberately excluded: the server generates it at each
/// admission attempt. Replays preserve the first accepted worker and terminal,
/// including that worker's original query time, rather than replacing it.
/// Georeference equality covers the assigned asset and immutable typed fields;
/// exact repository-byte stability remains the repository snapshot's fence.
#[derive(Clone, PartialEq)]
struct RetainedRequestIdentity {
    provider: &'static str,
    producer: String,
    producer_epoch: u64,
    query_text: String,
    active_map: AssignedMapIdentityV3,
    georeference: Arc<AssignedGeoreferenceV3>,
    caller_timeout: Duration,
    provider_timeout: Duration,
}

impl MappingCloudRequest {
    fn retained_identity(&self, provider: &MappingCloudProvider) -> RetainedRequestIdentity {
        RetainedRequestIdentity {
            provider: provider.identity(),
            producer: self.query_identity.producer.clone(),
            producer_epoch: self.query_identity.producer_epoch,
            query_text: self.query_text.clone(),
            active_map: self.active_map.clone(),
            georeference: Arc::clone(&self.georeference),
            caller_timeout: self.caller_timeout,
            provider_timeout: self.provider_timeout,
        }
    }
}

/// 每个已接受请求唯一发布的类型化终态。
#[derive(Debug, Clone, PartialEq)]
pub enum MappingCloudTerminal {
    /// 查询、坐标转换及 intent 校验全部成功。
    Succeeded(AssignedMappingCloudGoalIntentV3),
    /// provider 成功响应但没有匹配项。
    ProviderRejected,
    /// provider 在自身超时前返回失败。
    ProviderFailed(String),
    /// provider 自身 deadline 先到期。
    ProviderTimeout,
    /// 调用者端到端 deadline 先到期。
    CallerDeadline,
    /// 请求被显式取消或由 shutdown 取消。
    Cancelled,
    /// provider 结果无法适配为稳定 intent。
    AdaptationFailed(GoalAdapterError),
    /// 执行任务异常终止。
    FinalizerFailed(String),
}

/// `submit` 的 admission 结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MappingCloudSubmit {
    /// 新请求已接受并开始执行。
    Accepted {
        /// 当前 owner 生命周期 epoch。
        owner_epoch: u64,
        /// 原查询请求 ID。
        query_request_id: String,
    },
    /// same-ID、相同业务字段重放，未再次调用 provider。
    Replay {
        /// 当前 owner 生命周期 epoch。
        owner_epoch: u64,
        /// 原查询请求 ID。
        query_request_id: String,
        /// 原请求是否已经进入终态。
        terminal: bool,
    },
}

/// Owner admission 或查找失败。
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum MappingCloudOwnerError {
    /// Owner epoch 必须由启动者提供且非零。
    #[error("mapping-cloud owner epoch must be non-zero")]
    InvalidOwnerEpoch,
    /// 查询文本或 deadline 不满足生产准入。
    #[error("invalid mapping-cloud request: {0}")]
    InvalidRequest(&'static str),
    /// Owner 已开始 shutdown，不再接受请求。
    #[error("mapping-cloud owner is shutting down")]
    ShuttingDown,
    /// 单任务容量已占用。
    #[error("mapping-cloud owner is busy with request {active_request_id}")]
    Busy {
        /// 当前活跃请求 ID。
        active_request_id: String,
    },
    /// 相同请求 ID 携带了不同业务字段。
    #[error("mapping-cloud request ID conflicts with retained business fields")]
    RequestConflict,
    /// 请求 ID 不属于当前 owner。
    #[error("mapping-cloud request is not retained")]
    UnknownRequest,
}

/// shutdown 在 deadline 内的收口结果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MappingCloudShutdown {
    /// 所有 retained finalizer 已 join。
    Settled,
    /// deadline 到期；同一 owner 仍保留 finalizer，可继续 await。
    Pending {
        /// 尚在收尾的请求 ID。
        request_id: String,
        /// owner 生命周期 epoch。
        owner_epoch: u64,
    },
}

struct RetainedRequest {
    identity: RetainedRequestIdentity,
    terminal: Option<MappingCloudTerminal>,
    settled: bool,
    cancel: watch::Sender<bool>,
    finalizer: Arc<Mutex<JoinHandle<()>>>,
}

struct OwnerState {
    shutting_down: bool,
    active_request_id: Option<String>,
    requests: HashMap<String, RetainedRequest>,
}

struct Shared {
    owner_epoch: u64,
    state: Mutex<OwnerState>,
    changed: watch::Sender<u64>,
}

/// 只允许一个在途请求的 mapping-cloud supervised owner。
pub struct MappingCloudOwner {
    provider: MappingCloudProvider,
    shared: Arc<Shared>,
}

impl MappingCloudOwner {
    /// 启动一个新 owner 生命周期。
    pub fn start(
        owner_epoch: u64,
        provider: MappingCloudProvider,
    ) -> Result<Self, MappingCloudOwnerError> {
        if owner_epoch == 0 {
            return Err(MappingCloudOwnerError::InvalidOwnerEpoch);
        }
        let (changed, _) = watch::channel(0_u64);
        Ok(Self {
            provider,
            shared: Arc::new(Shared {
                owner_epoch,
                state: Mutex::new(OwnerState {
                    shutting_down: false,
                    active_request_id: None,
                    requests: HashMap::new(),
                }),
                changed,
            }),
        })
    }

    /// 接受新请求，或重放同一 ID 与业务字段的 retained 状态。
    pub async fn submit(
        &self,
        request: MappingCloudRequest,
    ) -> Result<MappingCloudSubmit, MappingCloudOwnerError> {
        validate_request(&request)?;
        let request_id = request.query_identity.query_request_id.clone();
        let identity = request.retained_identity(&self.provider);
        let mut state = self.shared.state.lock().await;
        if state.shutting_down {
            return Err(MappingCloudOwnerError::ShuttingDown);
        }
        if let Some(retained) = state.requests.get(&request_id) {
            if retained.identity != identity {
                return Err(MappingCloudOwnerError::RequestConflict);
            }
            return Ok(MappingCloudSubmit::Replay {
                owner_epoch: self.shared.owner_epoch,
                query_request_id: request_id,
                terminal: retained.terminal.is_some(),
            });
        }
        if let Some(active_request_id) = state.active_request_id.as_ref() {
            return Err(MappingCloudOwnerError::Busy {
                active_request_id: active_request_id.clone(),
            });
        }
        let admission = Instant::now();
        let caller_deadline = admission.checked_add(request.caller_timeout).ok_or(
            MappingCloudOwnerError::InvalidRequest("caller_timeout exceeds the instant range"),
        )?;
        let provider_deadline = admission.checked_add(request.provider_timeout).ok_or(
            MappingCloudOwnerError::InvalidRequest("provider_timeout exceeds the instant range"),
        )?;

        let (cancel, cancel_rx) = watch::channel(false);
        let worker = tokio::spawn(execute_request(
            self.provider.clone(),
            request,
            caller_deadline,
            provider_deadline,
            cancel_rx,
        ));
        let shared = Arc::clone(&self.shared);
        let task_request_id = request_id.clone();
        let task_identity = identity.clone();
        let finalizer = tokio::spawn(async move {
            let mut terminal = match worker.await {
                Ok(terminal) => terminal,
                Err(error) => MappingCloudTerminal::FinalizerFailed(error.to_string()),
            };
            let mut state = shared.state.lock().await;
            if let Some(retained) = state.requests.get_mut(&task_request_id) {
                if *retained.cancel.borrow() {
                    terminal = MappingCloudTerminal::Cancelled;
                } else if Instant::now() >= caller_deadline {
                    terminal = MappingCloudTerminal::CallerDeadline;
                }
                if retained.identity == task_identity && retained.terminal.is_none() {
                    retained.terminal = Some(terminal);
                }
            }
            if state.active_request_id.as_deref() == Some(task_request_id.as_str()) {
                state.active_request_id = None;
            }
            drop(state);
            shared
                .changed
                .send_modify(|version| *version = version.wrapping_add(1));
        });
        state.active_request_id = Some(request_id.clone());
        state.requests.insert(
            request_id.clone(),
            RetainedRequest {
                identity,
                terminal: None,
                settled: false,
                cancel,
                finalizer: Arc::new(Mutex::new(finalizer)),
            },
        );
        Ok(MappingCloudSubmit::Accepted {
            owner_epoch: self.shared.owner_epoch,
            query_request_id: request_id,
        })
    }

    /// 取消 retained 请求；重复取消保持幂等。
    pub async fn cancel(&self, request_id: &str) -> Result<(), MappingCloudOwnerError> {
        let state = self.shared.state.lock().await;
        let retained = state
            .requests
            .get(request_id)
            .ok_or(MappingCloudOwnerError::UnknownRequest)?;
        if retained.terminal.is_none() {
            let _ = retained.cancel.send(true);
        }
        Ok(())
    }

    /// 等待 retained 请求的唯一终态，并 join 同一 finalizer。
    pub async fn await_terminal(
        &self,
        request_id: &str,
    ) -> Result<MappingCloudTerminal, MappingCloudOwnerError> {
        let mut changed = self.shared.changed.subscribe();
        loop {
            let (terminal, finalizer, settled) = {
                let state = self.shared.state.lock().await;
                let retained = state
                    .requests
                    .get(request_id)
                    .ok_or(MappingCloudOwnerError::UnknownRequest)?;
                (
                    retained.terminal.clone(),
                    Arc::clone(&retained.finalizer),
                    retained.settled,
                )
            };
            if settled {
                return Ok(terminal.expect("settled request has a terminal"));
            }
            if let Some(terminal) = terminal {
                let mut finalizer = finalizer.lock().await;
                {
                    let state = self.shared.state.lock().await;
                    let retained = state
                        .requests
                        .get(request_id)
                        .ok_or(MappingCloudOwnerError::UnknownRequest)?;
                    if retained.settled {
                        return Ok(retained
                            .terminal
                            .clone()
                            .expect("settled request has a terminal"));
                    }
                }
                let _ = (&mut *finalizer).await;
                let mut state = self.shared.state.lock().await;
                if let Some(retained) = state.requests.get_mut(request_id) {
                    retained.settled = true;
                }
                drop(state);
                self.shared
                    .changed
                    .send_modify(|version| *version = version.wrapping_add(1));
                return Ok(terminal);
            }
            let _ = changed.changed().await;
        }
    }

    /// 停止 admission、取消 active 请求，并在 deadline 内收口全部 finalizer。
    pub async fn shutdown(&self, deadline: Duration) -> MappingCloudShutdown {
        let request_ids = {
            let mut state = self.shared.state.lock().await;
            state.shutting_down = true;
            if let Some(active_request_id) = state.active_request_id.as_ref() {
                if let Some(retained) = state.requests.get(active_request_id) {
                    let _ = retained.cancel.send(true);
                }
            }
            state.requests.keys().cloned().collect::<Vec<_>>()
        };
        let settle = async {
            for request_id in &request_ids {
                let _ = self.await_terminal(request_id).await;
            }
        };
        if timeout(deadline, settle).await.is_ok() {
            MappingCloudShutdown::Settled
        } else {
            let pending = {
                let state = self.shared.state.lock().await;
                request_ids
                    .into_iter()
                    .find(|id| state.requests.get(id).is_some_and(|entry| !entry.settled))
            };
            match pending {
                Some(request_id) => MappingCloudShutdown::Pending {
                    request_id,
                    owner_epoch: self.shared.owner_epoch,
                },
                None => MappingCloudShutdown::Settled,
            }
        }
    }
}

async fn execute_request(
    provider: MappingCloudProvider,
    request: MappingCloudRequest,
    caller_deadline: Instant,
    provider_deadline: Instant,
    mut cancel: watch::Receiver<bool>,
) -> MappingCloudTerminal {
    let provider_result = if provider_deadline < caller_deadline {
        let provider_call = timeout_at(provider_deadline, provider.query(&request.query_text));
        tokio::pin!(provider_call);
        tokio::select! {
            biased;
            _ = cancel.changed() => return MappingCloudTerminal::Cancelled,
            result = timeout_at(caller_deadline, &mut provider_call) => match result {
                Ok(Ok(result)) => result,
                Ok(Err(_)) => return MappingCloudTerminal::ProviderTimeout,
                Err(_) => return MappingCloudTerminal::CallerDeadline,
            },
        }
    } else {
        let provider_call = provider.query(&request.query_text);
        tokio::pin!(provider_call);
        tokio::select! {
            biased;
            _ = cancel.changed() => return MappingCloudTerminal::Cancelled,
            result = timeout_at(caller_deadline, &mut provider_call) => match result {
                Ok(result) => result,
                Err(_) => return MappingCloudTerminal::CallerDeadline,
            },
        }
    };

    if *cancel.borrow() {
        return MappingCloudTerminal::Cancelled;
    }
    let terminal = match provider_result {
        Ok(Some(position)) => match adapt_assigned_wgs84_goal_intent(
            &request.georeference,
            request.query_identity,
            request.active_map,
            position,
        ) {
            Ok(intent) => MappingCloudTerminal::Succeeded(intent),
            Err(error) => MappingCloudTerminal::AdaptationFailed(error),
        },
        Ok(None) => MappingCloudTerminal::ProviderRejected,
        Err(error) => MappingCloudTerminal::ProviderFailed(error.to_string()),
    };
    if *cancel.borrow() {
        MappingCloudTerminal::Cancelled
    } else if Instant::now() >= caller_deadline {
        MappingCloudTerminal::CallerDeadline
    } else {
        terminal
    }
}

fn validate_request(request: &MappingCloudRequest) -> Result<(), MappingCloudOwnerError> {
    if request.query_text.trim().is_empty() || request.query_text.len() > 4096 {
        return Err(MappingCloudOwnerError::InvalidRequest(
            "query_text must be non-empty and at most 4096 bytes",
        ));
    }
    if request.caller_timeout.is_zero() {
        return Err(MappingCloudOwnerError::InvalidRequest(
            "caller_timeout must be non-zero",
        ));
    }
    if request.provider_timeout.is_zero() {
        return Err(MappingCloudOwnerError::InvalidRequest(
            "provider_timeout must be non-zero",
        ));
    }
    validate_assigned_goal_inputs(
        &request.georeference,
        &request.query_identity,
        &request.active_map,
    )
    .map_err(|_| {
        MappingCloudOwnerError::InvalidRequest(
            "query, active-map, or georeference identity is invalid or mismatched",
        )
    })?;
    Ok(())
}


#[cfg(test)]
mod assigned_tests {
    use super::*;
    use crate::assigned_test_support::{anchor, request, ASSET_ID, RAW};
    use std::sync::atomic::{AtomicUsize, Ordering};
    use tokio::sync::Notify;

    struct Control {
        calls: Arc<AtomicUsize>,
        started: Arc<Notify>,
        release: Arc<Notify>,
    }

    fn controlled(result: Result<Option<LatLng>, String>) -> (MappingCloudProvider, Control) {
        let control = Control { calls: Arc::new(AtomicUsize::new(0)),
            started: Arc::new(Notify::new()), release: Arc::new(Notify::new()) };
        let provider = MappingCloudProvider::Controlled { result,
            calls: Arc::clone(&control.calls), started: Arc::clone(&control.started),
            release: Arc::clone(&control.release) };
        (provider, control)
    }

    async fn terminal(owner: &MappingCloudOwner) -> MappingCloudTerminal {
        timeout(Duration::from_secs(2), owner.await_terminal("assigned-request"))
            .await.expect("owner must settle").unwrap()
    }

    #[test]
    fn assigned_owner_retained_identity_binds_all_business_metadata() {
        let (provider, _) = controlled(Ok(Some(anchor())));
        let original = request().retained_identity(&provider);
        assert!(request().retained_identity(&provider) == original);
        for component in 0..21 {
            let mut changed = request();
            match component {
                0 => changed.query_identity.producer.push('2'),
                1 => changed.query_identity.producer_epoch += 1,
                2 => changed.query_text.push('2'),
                3 => changed.active_map.asset_id = "22222222222222222222222222222222".into(),
                4 => changed.active_map.activation_request_id.push('2'),
                5 => changed.active_map.checkpoint_id = "44444444444444444444444444444444".into(),
                6 => changed.active_map.map_name.push('2'),
                7 => changed.active_map.map_frame_id.push('2'),
                8 => changed.active_map.runtime_generation += 1,
                9 => changed.active_map.lifecycle_epoch += 1,
                10 => changed.active_map.mapping_session_id.push('2'),
                11 => changed.active_map.mapping_session_generation += 1,
                12 => changed.active_map.backend_kind = "raytracing".into(),
                13 => changed.active_map.backend_generation += 1,
                14 => changed.active_map.committed_occupancy_revision += 1,
                15 => changed.active_map.committed_occupancy_boot_nonce[0] ^= 1,
                16 => changed.active_map.committed_occupancy_commit_sequence += 1,
                17 => changed.active_map.committed_occupancy_source_timestamp_ns += 1,
                18 => changed.caller_timeout += Duration::from_millis(1),
                19 => changed.provider_timeout += Duration::from_millis(1),
                _ => {
                    let raw = std::str::from_utf8(RAW).unwrap()
                        .replace("\"map_origin_x_m\":10.0", "\"map_origin_x_m\":11.0");
                    changed.georeference = Arc::new(AssignedGeoreferenceV3::from_json(
                        raw.as_bytes(), ASSET_ID).unwrap());
                }
            }
            assert!(changed.retained_identity(&provider) != original, "component {component}");
        }
        let mut retry = request();
        retry.query_identity.source_timestamp_ns += 1;
        assert!(retry.retained_identity(&provider) == original);
    }

    #[tokio::test]
    async fn assigned_owner_replays_inflight_and_terminal_preserving_first_query_time() {
        let (provider, control) = controlled(Ok(Some(anchor())));
        let owner = MappingCloudOwner::start(9, provider).unwrap();
        assert_eq!(owner.submit(request()).await.unwrap(), MappingCloudSubmit::Accepted {
            owner_epoch: 9, query_request_id: "assigned-request".into() });
        control.started.notified().await;
        let mut retry = request();
        retry.query_identity.source_timestamp_ns = 200;
        assert_eq!(owner.submit(retry.clone()).await.unwrap(), MappingCloudSubmit::Replay {
            owner_epoch: 9, query_request_id: "assigned-request".into(), terminal: false });
        assert_eq!(control.calls.load(Ordering::SeqCst), 1);
        control.release.notify_one();
        let original = terminal(&owner).await;
        let MappingCloudTerminal::Succeeded(intent) = &original else { panic!("expected goal") };
        assert_eq!(intent.query().source_timestamp_ns, 100);
        assert_eq!(intent.active_map().committed_occupancy_source_timestamp_ns, 8);
        assert_eq!(owner.submit(retry).await.unwrap(), MappingCloudSubmit::Replay {
            owner_epoch: 9, query_request_id: "assigned-request".into(), terminal: true });
        assert_eq!(terminal(&owner).await, original);
        assert_eq!(control.calls.load(Ordering::SeqCst), 1);
        assert_eq!(owner.shutdown(Duration::from_secs(1)).await, MappingCloudShutdown::Settled);
    }

    #[tokio::test]
    async fn assigned_owner_retains_conflict_busy_and_cancel_semantics() {
        let (provider, control) = controlled(Ok(Some(anchor())));
        let owner = MappingCloudOwner::start(9, provider).unwrap();
        owner.submit(request()).await.unwrap();
        control.started.notified().await;
        let mut conflict = request();
        conflict.active_map.checkpoint_id = "44444444444444444444444444444444".into();
        assert_eq!(owner.submit(conflict.clone()).await.unwrap_err(), MappingCloudOwnerError::RequestConflict);
        let mut busy = request();
        busy.query_identity.query_request_id = "different-request".into();
        assert_eq!(owner.submit(busy.clone()).await.unwrap_err(), MappingCloudOwnerError::Busy {
            active_request_id: "assigned-request".into() });
        owner.cancel("assigned-request").await.unwrap();
        owner.cancel("assigned-request").await.unwrap();
        assert_eq!(terminal(&owner).await, MappingCloudTerminal::Cancelled);
        assert_eq!(owner.submit(conflict).await.unwrap_err(), MappingCloudOwnerError::RequestConflict);
        assert!(matches!(owner.submit(busy).await.unwrap(), MappingCloudSubmit::Accepted { .. }));
        owner.cancel("different-request").await.unwrap();
        assert_eq!(owner.await_terminal("different-request").await.unwrap(), MappingCloudTerminal::Cancelled);
        assert_eq!(owner.shutdown(Duration::from_secs(1)).await, MappingCloudShutdown::Settled);
    }

    #[tokio::test]
    async fn assigned_owner_deadlines_cancel_blocked_provider() {
        for caller_first in [false, true] {
            let (provider, control) = controlled(Ok(Some(anchor())));
            let owner = MappingCloudOwner::start(9, provider).unwrap();
            let mut request = request();
            if caller_first { request.caller_timeout = Duration::from_millis(10); }
            else { request.provider_timeout = Duration::from_millis(10); }
            owner.submit(request).await.unwrap();
            control.started.notified().await;
            let expected = if caller_first { MappingCloudTerminal::CallerDeadline }
                else { MappingCloudTerminal::ProviderTimeout };
            assert_eq!(terminal(&owner).await, expected);
            assert_eq!(control.calls.load(Ordering::SeqCst), 1);
            assert_eq!(owner.shutdown(Duration::from_secs(1)).await, MappingCloudShutdown::Settled);
        }
    }

    #[tokio::test]
    async fn assigned_owner_shutdown_before_and_during_query() {
        for active in [false, true] {
            let (provider, control) = controlled(Ok(Some(anchor())));
            let owner = MappingCloudOwner::start(9, provider).unwrap();
            if active {
                owner.submit(request()).await.unwrap();
                control.started.notified().await;
            }
            assert_eq!(owner.shutdown(Duration::from_secs(1)).await, MappingCloudShutdown::Settled);
            assert_eq!(owner.submit(request()).await.unwrap_err(), MappingCloudOwnerError::ShuttingDown);
            if active { assert_eq!(terminal(&owner).await, MappingCloudTerminal::Cancelled); }
            assert_eq!(control.calls.load(Ordering::SeqCst), usize::from(active));
        }
    }

    #[tokio::test]
    async fn assigned_owner_pending_shutdown_keeps_the_original_finalizer() {
        let (provider, control) = controlled(Ok(Some(anchor())));
        let owner = MappingCloudOwner::start(9, provider).unwrap();
        owner.submit(request()).await.unwrap();
        control.started.notified().await;
        let finalizer = {
            let state = owner.shared.state.lock().await;
            Arc::clone(&state.requests.get("assigned-request").unwrap().finalizer)
        };
        // Hold the existing join guard so shutdown must report a retained
        // pending finalizer, even after cancellation publishes its terminal.
        let join_guard = finalizer.lock().await;
        assert_eq!(owner.shutdown(Duration::from_millis(10)).await,
            MappingCloudShutdown::Pending { request_id: "assigned-request".into(), owner_epoch: 9 });
        {
            let state = owner.shared.state.lock().await;
            assert!(Arc::ptr_eq(&finalizer,
                &state.requests.get("assigned-request").unwrap().finalizer));
        }
        drop(join_guard);
        assert_eq!(terminal(&owner).await, MappingCloudTerminal::Cancelled);
        assert_eq!(owner.shutdown(Duration::from_secs(1)).await, MappingCloudShutdown::Settled);
        assert_eq!(control.calls.load(Ordering::SeqCst), 1);
    }

    #[tokio::test]
    async fn assigned_owner_retains_provider_failure_rejection_and_adaptation_failure() {
        for result in [Ok(None), Err("controlled failure".into()),
            Ok(Some(LatLng { lat: 90.0, lng: 121.0 }))] {
            let (provider, control) = controlled(result.clone());
            let owner = MappingCloudOwner::start(9, provider).unwrap();
            owner.submit(request()).await.unwrap();
            control.started.notified().await;
            control.release.notify_one();
            let terminal = terminal(&owner).await;
            match result {
                Ok(None) => assert_eq!(terminal, MappingCloudTerminal::ProviderRejected),
                Err(_) => assert!(matches!(terminal, MappingCloudTerminal::ProviderFailed(_))),
                Ok(Some(_)) => assert!(matches!(terminal, MappingCloudTerminal::AdaptationFailed(_))),
            }
            assert!(matches!(owner.submit(request()).await.unwrap(), MappingCloudSubmit::Replay { terminal: true, .. }));
            assert_eq!(control.calls.load(Ordering::SeqCst), 1);
            assert_eq!(owner.shutdown(Duration::from_secs(1)).await, MappingCloudShutdown::Settled);
        }
    }

    #[tokio::test]
    async fn assigned_owner_rejects_invalid_input_without_querying_provider() {
        let (provider, control) = controlled(Ok(Some(anchor())));
        let owner = MappingCloudOwner::start(9, provider).unwrap();
        for component in 0..7 {
            let mut request = request();
            match component {
                0 => request.active_map.committed_occupancy_boot_nonce = [0; 16],
                1 => request.active_map.committed_occupancy_commit_sequence = 0,
                2 => request.active_map.committed_occupancy_source_timestamp_ns = 0,
                3 => request.query_text = " ".into(),
                4 => request.query_text = "x".repeat(4097),
                5 => request.caller_timeout = Duration::ZERO,
                _ => request.provider_timeout = Duration::ZERO,
            }
            assert!(matches!(owner.submit(request).await, Err(MappingCloudOwnerError::InvalidRequest(_))));
        }
        assert_eq!(control.calls.load(Ordering::SeqCst), 0);
        assert_eq!(owner.cancel("unknown").await.unwrap_err(), MappingCloudOwnerError::UnknownRequest);
        assert_eq!(owner.await_terminal("unknown").await.unwrap_err(), MappingCloudOwnerError::UnknownRequest);
        assert_eq!(owner.shutdown(Duration::from_secs(1)).await, MappingCloudShutdown::Settled);
    }
}
