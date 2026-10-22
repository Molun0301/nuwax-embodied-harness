//! 重定位终态共享原始编码；构造消费已有身份，读取只在实际文件边界解码。

use std::sync::Arc;
use prost::Message;
use crate::correction_coordinator::{
    validate_relocalization_request_id, CorrectionCandidate,
    RELOCALIZATION_MAX_CANDIDATE_PREIMAGE_BYTES, RELOCALIZATION_MAX_TERMINAL_BYTES,
    RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES,
};
use crate::proto;
use crate::relocalization_identity::AssignedCloudRelocalizeRequestIdentityV2;

const DURABLE_DOMAIN: &[u8] = b"nuwax.geometry.relocalize.assigned-terminal.v2\0";
const FILE_PREFIX: &str = "assigned-v2.";
const FILE_SUFFIX: &str = ".terminal";

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

    pub(crate) fn encode_durable(&self) -> Result<Vec<u8>, String> {
        let request = self.request().to_wire().encode_to_vec();
        let total = DURABLE_DOMAIN.len().checked_add(8)
            .and_then(|n| n.checked_add(request.len()))
            .and_then(|n| n.checked_add(self.encoded().len()))
            .ok_or("RELOCALIZATION_ASSIGNED_DURABLE_LENGTH_OVERFLOW")?;
        bounded(total, RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES, "DURABLE_RECORD")?;
        let mut out = Vec::with_capacity(total);
        out.extend_from_slice(DURABLE_DOMAIN);
        push_segment(&mut out, &request)?;
        push_segment(&mut out, self.encoded())?;
        Ok(out)
    }

    pub(crate) fn decode_durable(bytes: &[u8]) -> Result<Self, String> {
        bounded(bytes.len(), RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES, "DURABLE_RECORD")?;
        let mut cursor = bytes.strip_prefix(DURABLE_DOMAIN)
            .ok_or("RELOCALIZATION_ASSIGNED_DURABLE_SCHEMA_UNSUPPORTED")?;
        let request_bytes = take_segment(&mut cursor)?;
        let terminal_bytes = take_segment(&mut cursor)?;
        if !cursor.is_empty() { return Err("RELOCALIZATION_ASSIGNED_DURABLE_TRAILING_BYTES".into()); }
        // 保留现有双段文件格式；请求身份由终态内的唯一类型化请求解码取得。
        proto::RelocalizeRequest::decode(request_bytes)
            .map_err(|_| "RELOCALIZATION_ASSIGNED_DURABLE_REQUEST_INVALID")?;
        Self::from_encoded(terminal_bytes.to_vec())
    }

    pub(crate) fn decode_durable_named(filename: &str, bytes: &[u8]) -> Result<Self, String> {
        let request_id = assigned_terminal_request_id_from_filename_v2(filename)?;
        let terminal = Self::decode_durable(bytes)?;
        if terminal.request().request_id() != request_id {
            return Err("RELOCALIZATION_ASSIGNED_DURABLE_FILENAME_REQUEST_CONFLICT".into());
        }
        Ok(terminal)
    }
}

pub(crate) fn assigned_terminal_filename_v2(request_id: &str) -> Result<String, String> {
    validate_relocalization_request_id(request_id)?;
    Ok(format!("{FILE_PREFIX}{request_id}{FILE_SUFFIX}"))
}
pub(crate) fn assigned_terminal_request_id_from_filename_v2(name: &str) -> Result<&str, String> {
    let id = name.strip_prefix(FILE_PREFIX).and_then(|s| s.strip_suffix(FILE_SUFFIX))
        .ok_or("RELOCALIZATION_ASSIGNED_DURABLE_FILENAME_INVALID")?;
    validate_relocalization_request_id(id)?;
    Ok(id)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct AssignedRelocalizationTerminalQueryV2 { request: AssignedCloudRelocalizeRequestIdentityV2 }
impl AssignedRelocalizationTerminalQueryV2 {
    pub(crate) fn admit(query: proto::GetRelocalizationTerminalRequest) -> Result<Self, String> {
        bounded(query.encoded_len(), RELOCALIZATION_MAX_TERMINAL_BYTES, "QUERY")?;
        if query.schema_version != 2 { return Err("RELOCALIZATION_ASSIGNED_QUERY_SCHEMA_INVALID".into()); }
        let request = AssignedCloudRelocalizeRequestIdentityV2::admit_retained(
            query.assigned_request_v2.ok_or("RELOCALIZATION_ASSIGNED_QUERY_REQUEST_ABSENT")?)?;
        if query.request_id != request.request_id() { return Err("RELOCALIZATION_ASSIGNED_QUERY_REQUEST_ID_MISMATCH".into()); }
        Ok(Self { request })
    }
    pub(crate) fn request(&self) -> &AssignedCloudRelocalizeRequestIdentityV2 { &self.request }
    #[cfg(test)]
    pub(crate) fn to_wire(&self) -> proto::GetRelocalizationTerminalRequest {
        proto::GetRelocalizationTerminalRequest { schema_version: 2, request_id: self.request.request_id().into(),
            assigned_request_v2: Some(self.request.to_wire()) }
    }
    pub(crate) fn response(&self, status: proto::RelocalizationTerminalStatus,
        terminal: Option<&AssignedRelocalizationTerminalV2>, ledger_observation: Option<proto::RelocalizationLedgerObservation>,
    ) -> proto::GetRelocalizationTerminalResponse {
        proto::GetRelocalizationTerminalResponse { schema_version: 2, request_id: self.request.request_id().into(),
            status: status as i32, terminal: terminal.map(AssignedRelocalizationTerminalV2::to_wire),
            ledger_observation, assigned_request_v2: Some(self.request.to_wire()) }
    }


}

fn bounded(actual: usize, maximum: usize, field: &str) -> Result<(), String> {
    if actual > maximum { Err(format!("RELOCALIZATION_ASSIGNED_{field}_BOUND_EXCEEDED")) } else { Ok(()) }
}
fn push_segment(out: &mut Vec<u8>, bytes: &[u8]) -> Result<(), String> {
    bounded(bytes.len(), RELOCALIZATION_MAX_TERMINAL_BYTES, "DURABLE_SEGMENT")?;
    let length = u32::try_from(bytes.len()).map_err(|_| "RELOCALIZATION_ASSIGNED_DURABLE_LENGTH_OVERFLOW")?;
    out.extend_from_slice(&length.to_be_bytes()); out.extend_from_slice(bytes); Ok(())
}
fn take_segment<'a>(cursor: &mut &'a [u8]) -> Result<&'a [u8], String> {
    let (length, rest) = cursor.split_at_checked(4).ok_or("RELOCALIZATION_ASSIGNED_DURABLE_TRUNCATED")?;
    let length = u32::from_be_bytes(length.try_into().map_err(|_| "RELOCALIZATION_ASSIGNED_DURABLE_TRUNCATED")?) as usize;
    bounded(length, RELOCALIZATION_MAX_TERMINAL_BYTES, "DURABLE_SEGMENT")?;
    let (value, rest) = rest.split_at_checked(length).ok_or("RELOCALIZATION_ASSIGNED_DURABLE_TRUNCATED")?;
    *cursor = rest; Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use proto::RelocalizationTerminalStatus as Status;
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
    #[test]
    fn durable_reopen_preserves_bytes_and_checks_filename() {
        let terminal = terminal(); let encoded = terminal.encode_durable().unwrap();
        let name = assigned_terminal_filename_v2(terminal.request().request_id()).unwrap();
        let reopened = AssignedRelocalizationTerminalV2::decode_durable_named(&name, &encoded).unwrap();
        assert_eq!(reopened.encoded(), terminal.encoded());
        assert_eq!(reopened.request(), terminal.request());
        assert!(AssignedRelocalizationTerminalV2::decode_durable_named("assigned-v2.other.terminal", &encoded).is_err());
    }
    #[test]
    fn durable_boundary_rejects_truncation_trailing_bytes_and_real_limits() {
        let encoded = terminal().encode_durable().unwrap();
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&encoded[..encoded.len()-1]).is_err());
        let mut trailing = encoded; trailing.push(0);
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&trailing).is_err());
        assert!(AssignedRelocalizationTerminalV2::from_encoded(vec![0; RELOCALIZATION_MAX_TERMINAL_BYTES+1]).is_err());
        assert!(AssignedRelocalizationTerminalV2::decode_durable(&vec![0; RELOCALIZATION_MAX_TERMINAL_RECORD_BYTES+1]).is_err());
    }
    #[test]
    fn query_does_not_create_missing_terminal_receipts() {
        let query = AssignedRelocalizationTerminalQueryV2 { request: request() };
        let admitted = AssignedRelocalizationTerminalQueryV2::admit(query.to_wire()).unwrap();
        for status in [Status::Pending, Status::NotFound, Status::IdentityConflict] {
            let response = admitted.response(status, None, None);
            assert!(response.terminal.is_none());
            assert_eq!(response.status,status as i32);
            assert_eq!(response.assigned_request_v2,Some(admitted.request().to_wire()));
        }
    }

}

#[cfg(test)]
#[path = "planar_relocalization_assigned_tests.rs"]
pub(crate) mod planar_tests;
