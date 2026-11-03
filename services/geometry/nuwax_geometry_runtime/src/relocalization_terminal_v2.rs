//! 重定位终态在内存中共享原始编码与已有请求身份。

use std::sync::Arc;
use prost::Message;
use crate::correction_coordinator::{
    CorrectionCandidate,
    RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES, RELOCALIZATION_MAX_TERMINAL_BYTES,
};
use crate::proto;
use crate::relocalization_identity::AssignedCloudRelocalizeRequestIdentityV2;

#[derive(Debug)]
struct AssignedTerminalDataV2 {
    request: AssignedCloudRelocalizeRequestIdentityV2,
    result: proto::RelocalizeResult,
    encoded: Vec<u8>,
}

#[derive(Clone, Debug)]
pub(crate) struct AssignedRelocalizationTerminalV2(Arc<AssignedTerminalDataV2>);
impl PartialEq for AssignedRelocalizationTerminalV2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0) || self.0.encoded == other.0.encoded
    }
}
impl Eq for AssignedRelocalizationTerminalV2 {}

impl AssignedRelocalizationTerminalV2 {
    pub(crate) fn new(request: AssignedCloudRelocalizeRequestIdentityV2, result: proto::RelocalizeResult) -> Self {
        let encoded = result.encode_to_vec();
        Self(Arc::new(AssignedTerminalDataV2 { request, result, encoded }))
    }

    pub(crate) fn from_encoded(encoded: Vec<u8>) -> Result<Self, String> {
        bounded(encoded.len(), RELOCALIZATION_MAX_TERMINAL_BYTES, "TERMINAL")?;
        let result = proto::RelocalizeResult::decode(encoded.as_slice())
            .map_err(|_| "RELOCALIZATION_ASSIGNED_TERMINAL_DECODE_INVALID")?;
        if result.schema_version != 2 {
            return Err("RELOCALIZATION_ASSIGNED_TERMINAL_SCHEMA_UNSUPPORTED".into());
        }
        let request = match (&result.assigned_cloud_v2, &result.assigned_planar_v2) {
            (Some(body), None) => {
                bounded(body.candidate_payload_v3.len(), RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES, "CANDIDATE")?;
                if !body.candidate_payload_v3.is_empty() {
                    CorrectionCandidate::decode_body(&body.candidate_payload_v3).map_err(|error| error.to_string())?;
                }
                body.request.clone()
            },
            (None, Some(body)) => body.request.clone(),
            _ => return Err("RELOCALIZATION_ASSIGNED_TERMINAL_BODY_ABSENT_OR_MIXED".into()),
        }.ok_or("RELOCALIZATION_ASSIGNED_TERMINAL_REQUEST_ABSENT")?;
        let request = AssignedCloudRelocalizeRequestIdentityV2::admit_retained(request)?;
        if result.request_id != request.request_id() {
            return Err("RELOCALIZATION_ASSIGNED_TERMINAL_REQUEST_ID_MISMATCH".into());
        }
        Ok(Self(Arc::new(AssignedTerminalDataV2 { request, result, encoded })))
    }

    pub(crate) fn request(&self) -> &AssignedCloudRelocalizeRequestIdentityV2 { &self.0.request }
    pub(crate) fn result(&self) -> &proto::RelocalizeResult { &self.0.result }
    pub(crate) fn encoded(&self) -> &[u8] { &self.0.encoded }
    pub(crate) fn to_wire(&self) -> proto::RelocalizeResult { self.0.result.clone() }
    pub(crate) fn body(&self) -> &proto::AssignedCloudRelocalizeResultV2 {
        self.0.result.assigned_cloud_v2.as_ref().expect("cloud terminal body")
    }

}

fn bounded(actual: usize, maximum: usize, field: &str) -> Result<(), String> {
    if actual > maximum { Err(format!("RELOCALIZATION_ASSIGNED_{field}_BOUND_EXCEEDED")) } else { Ok(()) }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn request() -> AssignedCloudRelocalizeRequestIdentityV2 {
        AssignedCloudRelocalizeRequestIdentityV2::admit(crate::relocalization_identity::tests::wire()).unwrap()
    }
    fn terminal() -> AssignedRelocalizationTerminalV2 {
        let request = request();
        let result = proto::RelocalizeResult { schema_version: 2, request_id: request.request_id().into(),
            assigned_cloud_v2: Some(proto::AssignedCloudRelocalizeResultV2 {
                request: Some(request.to_wire()), disposition: proto::RelocalizeDisposition::TargetUnavailable as i32,
                detail: "RELOCALIZATION_TARGET_UNAVAILABLE".into(), ..Default::default()
            }), ..Default::default() };
        AssignedRelocalizationTerminalV2::new(request, result)
    }
    #[test]
    fn capture_moves_original_encoded_allocation_and_retry_shares_owner() {
        let encoded = terminal().to_wire().encode_to_vec();
        let pointer = encoded.as_ptr();
        let terminal = AssignedRelocalizationTerminalV2::from_encoded(encoded).unwrap();
        assert_eq!(terminal.encoded().as_ptr(), pointer);
        let weak = Arc::downgrade(&terminal.0);
        let retries: Vec<_> = (0..64).map(|_| terminal.clone()).collect();
        assert!(retries.iter().all(|retry| Arc::ptr_eq(&retry.0, &terminal.0)));
        drop(terminal); assert!(weak.upgrade().is_some());
        drop(retries); assert!(weak.upgrade().is_none());
    }

}

#[cfg(test)]
#[path = "planar_relocalization_assigned_tests.rs"]
pub(crate) mod planar_tests;
