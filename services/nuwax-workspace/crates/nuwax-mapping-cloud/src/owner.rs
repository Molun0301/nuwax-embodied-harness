//! 单任务 mapping-cloud 生命周期 owner。

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use thiserror::Error;
use tokio::sync::{watch, Mutex};
use tokio::task::JoinHandle;
use tokio::time::{timeout, timeout_at, Instant};

use crate::{
    adapt_wgs84_goal_intent, ActiveMapIdentityV2, CloudError, CloudQueryIdentityV1,
    GoalAdapterError, GoogleMapsClient, LatLng, MappingCloudGoalIntentV2, OsmClient, Sha256Digest,
    VerifiedGeoreferenceV1,
};

const REQUEST_DIGEST_DOMAIN: &[u8] = b"nuwax.mapping-cloud.owner-request.v2\0";

/// Owner 使用的真实地图查询后端。
#[derive(Clone)]
pub enum MappingCloudProvider {
    /// Google Maps 文本地点查询，使用首个结果。
    GoogleTextSearch(Arc<GoogleMapsClient>),
    /// OpenStreetMap Nominatim 查询，使用首个结果。
    OsmNominatim(Arc<OsmClient>),
}

impl MappingCloudProvider {
    fn identity(&self) -> &'static str {
        match self {
            Self::GoogleTextSearch(_) => "google-text-search",
            Self::OsmNominatim(_) => "osm-nominatim",
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
        }
    }
}

/// 一个受监督 cloud 查询请求。
pub struct MappingCloudRequest {
    /// 稳定请求及生产者身份。
    pub query_identity: CloudQueryIdentityV1,
    /// 发送给真实 provider 的非空查询文本。
    pub query_text: String,
    /// 查询结果必须绑定的 active-map 快照。
    pub active_map: ActiveMapIdentityV2,
    /// 已校验且绑定原始资产字节的地理参考。
    pub georeference: Arc<VerifiedGeoreferenceV1>,
    /// 调用者允许的端到端时长。
    pub caller_timeout: Duration,
    /// provider 单次调用允许的时长。
    pub provider_timeout: Duration,
}

impl MappingCloudRequest {
    fn digest(&self, provider: &MappingCloudProvider) -> Sha256Digest {
        let mut bytes = Vec::with_capacity(512);
        bytes.extend_from_slice(REQUEST_DIGEST_DOMAIN);
        push_text(&mut bytes, provider.identity());
        push_text(&mut bytes, &self.query_identity.query_request_id);
        push_text(&mut bytes, &self.query_identity.producer);
        bytes.extend_from_slice(&self.query_identity.producer_epoch.to_be_bytes());
        bytes.extend_from_slice(&self.query_identity.source_timestamp_ns.to_be_bytes());
        push_text(&mut bytes, &self.query_text);
        push_text(&mut bytes, &self.active_map.map_name);
        push_text(&mut bytes, &self.active_map.map_frame_id);
        for digest in [
            self.active_map.asset_manifest_sha256,
            self.active_map.asset_bundle_sha256,
            self.active_map.georeference_sha256,
            self.active_map.activation_identity_sha256,
            self.georeference.raw_sha256(),
            self.georeference.canonical_sha256(),
        ] {
            bytes.extend_from_slice(digest.as_bytes());
        }
        for value in [
            self.active_map.runtime_generation,
            self.active_map.mapping_session_generation,
            self.active_map.committed_occupancy_revision,
        ] {
            bytes.extend_from_slice(&value.to_be_bytes());
        }
        bytes.extend_from_slice(&self.active_map.committed_occupancy_boot_nonce);
        bytes.extend_from_slice(&self.active_map.committed_occupancy_commit_sequence.to_be_bytes());
        bytes.extend_from_slice(&self.active_map.committed_occupancy_source_timestamp_ns.to_be_bytes());
        bytes.extend_from_slice(&self.caller_timeout.as_nanos().to_be_bytes());
        bytes.extend_from_slice(&self.provider_timeout.as_nanos().to_be_bytes());
        Sha256Digest::of_bytes(&bytes)
    }
}

/// 每个已接受请求唯一发布的类型化终态。
#[derive(Debug, Clone, PartialEq)]
pub enum MappingCloudTerminal {
    /// 查询、坐标转换及 intent 校验全部成功。
    Succeeded(MappingCloudGoalIntentV2),
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
        /// 请求摘要。
        request_digest: Sha256Digest,
    },
    /// same-ID、same-digest 重放，未再次调用 provider。
    Replay {
        /// 当前 owner 生命周期 epoch。
        owner_epoch: u64,
        /// 原请求摘要。
        request_digest: Sha256Digest,
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
    /// 相同请求 ID 携带了不同摘要。
    #[error("mapping-cloud request ID conflicts with retained digest")]
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
    digest: Sha256Digest,
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

    /// 接受新请求，或重放同一 ID 与摘要的 retained 状态。
    pub async fn submit(
        &self,
        request: MappingCloudRequest,
    ) -> Result<MappingCloudSubmit, MappingCloudOwnerError> {
        validate_request(&request)?;
        let request_id = request.query_identity.query_request_id.clone();
        let digest = request.digest(&self.provider);
        let mut state = self.shared.state.lock().await;
        if state.shutting_down {
            return Err(MappingCloudOwnerError::ShuttingDown);
        }
        if let Some(retained) = state.requests.get(&request_id) {
            if retained.digest != digest {
                return Err(MappingCloudOwnerError::RequestConflict);
            }
            return Ok(MappingCloudSubmit::Replay {
                owner_epoch: self.shared.owner_epoch,
                request_digest: digest,
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
                if retained.digest == digest && retained.terminal.is_none() {
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
            request_id,
            RetainedRequest {
                digest,
                terminal: None,
                settled: false,
                cancel,
                finalizer: Arc::new(Mutex::new(finalizer)),
            },
        );
        Ok(MappingCloudSubmit::Accepted {
            owner_epoch: self.shared.owner_epoch,
            request_digest: digest,
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
        Ok(Some(position)) => match adapt_wgs84_goal_intent(
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
    let contract = request.georeference.contract();
    let validation_position = LatLng {
        lat: contract.anchor_lat_deg,
        lng: contract.anchor_lng_deg,
    };
    adapt_wgs84_goal_intent(
        &request.georeference,
        request.query_identity.clone(),
        request.active_map.clone(),
        validation_position,
    )
    .map_err(|_| {
        MappingCloudOwnerError::InvalidRequest(
            "query, active-map, or georeference identity is invalid or mismatched",
        )
    })?;
    Ok(())
}

fn push_text(output: &mut Vec<u8>, value: &str) {
    let length = u32::try_from(value.len()).unwrap_or(u32::MAX);
    output.extend_from_slice(&length.to_be_bytes());
    output.extend_from_slice(value.as_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::GeoreferenceBindingV1;

    fn request() -> MappingCloudRequest {
        let raw = br#"{"schema_version":1,"map_name":"shanghai_lab","map_frame_id":"map/cloud","source_crs":"EPSG:4326","projected_crs":"EPSG:3857","axis_convention":"projected_x_east_y_north_z_up","anchor_lat_deg":31.2304,"anchor_lng_deg":121.4737,"anchor_easting_m":13522390.428774875,"anchor_northing_m":3662707.257050403,"map_origin_x_m":10.0,"map_origin_y_m":-4.0,"map_from_projected_yaw_rad":0.0,"altitude_policy":"map_plane_constant","map_plane_z_m":1.25}"#;
        let georeference = Arc::new(VerifiedGeoreferenceV1::from_json(raw,
            &GeoreferenceBindingV1::for_document(raw)).unwrap());
        MappingCloudRequest {
            query_identity: CloudQueryIdentityV1 {
                query_request_id: "receipt-request".into(), producer: "mapping-cloud-test".into(),
                producer_epoch: 1, source_timestamp_ns: 100,
            },
            query_text: "destination".into(),
            active_map: ActiveMapIdentityV2 {
                map_name: georeference.contract().map_name.clone(),
                map_frame_id: georeference.contract().map_frame_id.clone(),
                asset_manifest_sha256: Sha256Digest::of_bytes(b"manifest"),
                asset_bundle_sha256: Sha256Digest::of_bytes(b"bundle"),
                georeference_sha256: georeference.raw_sha256(),
                activation_identity_sha256: Sha256Digest::of_bytes(b"activation"),
                runtime_generation: 1, mapping_session_generation: 2,
                committed_occupancy_revision: 3, committed_occupancy_boot_nonce: [4; 16],
                committed_occupancy_commit_sequence: 5, committed_occupancy_source_timestamp_ns: 6,
            },
            georeference,
            caller_timeout: Duration::from_secs(2), provider_timeout: Duration::from_secs(1),
        }
    }

    #[test]
    fn owner_request_identity_binds_each_receipt_component() {
        let provider = MappingCloudProvider::OsmNominatim(Arc::new(OsmClient::new()));
        let baseline = request();
        validate_request(&baseline).unwrap();
        let original = baseline.digest(&provider);
        assert_eq!(request().digest(&provider), original);
        assert_eq!(REQUEST_DIGEST_DOMAIN, b"nuwax.mapping-cloud.owner-request.v2\0");
        for component in 0..3 {
            let mut changed = request();
            match component {
                0 => changed.active_map.committed_occupancy_boot_nonce[0] ^= 1,
                1 => changed.active_map.committed_occupancy_commit_sequence += 1,
                _ => changed.active_map.committed_occupancy_source_timestamp_ns += 1,
            }
            assert_ne!(changed.digest(&provider), original);
        }
    }

    #[test]
    fn owner_rejects_missing_receipt_before_provider_query() {
        let mut missing = request();
        missing.active_map.committed_occupancy_boot_nonce = [0; 16];
        assert!(validate_request(&missing).is_err());
        missing = request();
        missing.active_map.committed_occupancy_commit_sequence = 0;
        assert!(validate_request(&missing).is_err());
    }
}
