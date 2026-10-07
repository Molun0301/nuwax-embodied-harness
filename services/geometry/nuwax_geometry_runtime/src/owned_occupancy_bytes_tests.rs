//! Qualification of owned occupancy encoding only. The RPC fixtures below use
//! an explicitly injected in-process transport; they do not qualify real LCM.

use super::*;
use crate::lcm_runtime::{GeometryTestTaskOwner, MapState, OccupancyShmSnapshot};
use nuwax_transport::{LcmBackend, LcmFrameStream, LcmTransport, TransportResult};
use std::task::Poll;

// Frozen pre-change payload expression, independent of owned_occupancy_bytes.
fn borrowed_bytes_reference(data: &[i8]) -> Vec<u8> {
    data.iter().map(|&value| value as u8).collect()
}

// Frozen pre-change data RPC bodies, with only self -> service for free functions.
async fn effective_rpc_reference(service: &GeometryRuntimeService) -> Result<Response<OccupancyGridData>, Status> {
    let map = service.bridge.state.read().await.map.clone();
    validate_occupancy_grid(&map).map_err(OccupancyGridValidationError::into_status)?;
    if service.bridge.occupancy_shm.is_some() {
        await_occupancy_shm_revision(&service.bridge, map.revision).await?;
    }
    Ok(Response::new(OccupancyGridData {
        meta: Some(occupancy_grid_meta(&map)),
        data: map.data.iter().map(|&v| v as u8).collect(),
    }))
}

async fn raw_rpc_reference(service: &GeometryRuntimeService) -> Result<Response<OccupancyGridData>, Status> {
    let adapter = service.bridge.mapping_adapter.as_ref()
        .ok_or_else(|| Status::unavailable("mapping adapter is unavailable"))?;
    let raw_map = adapter.lock().raw_map_snapshot()
        .ok_or_else(|| Status::failed_precondition("physical occupancy grid is unavailable"))?;
    validate_occupancy_grid(&raw_map).map_err(OccupancyGridValidationError::into_status)?;
    Ok(Response::new(OccupancyGridData {
        meta: Some(occupancy_grid_meta(&raw_map)),
        data: raw_map.data.iter().map(|&value| value as u8).collect(),
    }))
}

#[test]
fn owned_conversion_retains_allocation_capacity_and_all_byte_patterns() {
    for (name, len, capacity) in [
        ("empty", 0, 0), ("allocated_empty", 0, 4096), ("single", 1, 1),
        ("all_bytes", 256, 256), ("spare_capacity", 256, 4096),
        ("one_mib", 1024 * 1024, 1024 * 1024),
        ("four_mib_spare", 4 * 1024 * 1024, 6 * 1024 * 1024),
        ("sixteen_mib", 16 * 1024 * 1024, 16 * 1024 * 1024),
    ] {
        let mut data = Vec::with_capacity(capacity);
        data.extend((0..len).map(|index| (index % 256) as u8 as i8));
        let expected = borrowed_bytes_reference(&data);
        let pointer = data.as_ptr().cast::<u8>();
        let actual_capacity = data.capacity();
        let output = owned_occupancy_bytes(std::hint::black_box(data));
        assert_eq!(output, expected, "bytes: {name}");
        assert_eq!(output.len(), len, "length: {name}");
        assert_eq!(output.as_ptr(), pointer, "allocation: {name}");
        assert_eq!(output.capacity(), actual_capacity, "capacity: {name}");
    }
}

struct UnitTransport;
#[async_trait::async_trait]
impl LcmBackend for UnitTransport {
    async fn publish(&self, _frame: RawLcmFrame) -> TransportResult<()> { Ok(()) }
    async fn subscribe(&self, _topic: LcmTopic, _kind: Option<LcmMessageType>) -> LcmFrameStream {
        Box::pin(futures::stream::empty())
    }
    fn name(&self) -> &str { "owned-occupancy-in-process-unit" }
}

struct UnitFixture {
    service: GeometryRuntimeService,
    handle: GeometryRuntimeHandle,
    tasks: GeometryTestTaskOwner,
}
impl UnitFixture {
    async fn new() -> Self {
        let (handle, tasks) = crate::lcm_runtime::build_handle_for_tests_with_unit_transport(
            false, Some(crate::mapping_backend::MappingBackendSpec::Voxel),
            LcmTransport::new(Arc::new(UnitTransport)),
        ).await.expect("explicit in-process occupancy fixture");
        assert_eq!(handle.transport_backend_name, "in-process-unit");
        assert!(!handle.lcm_backend_available);
        let service = GeometryRuntimeService::new(handle.clone());
        Self { service, handle, tasks }
    }
    fn start_mapping_owner(&self) {
        self.handle.mapping_adapter.as_ref().unwrap().lock()
            .start_session("owned-occupancy-unit", "test-map").expect("unit mapping owner");
    }
    async fn finish(self) {
        let mapping = self.handle.mapping_adapter.as_ref().unwrap().clone();
        let retirement = {
            let _lifecycle = self.handle.mapping_lifecycle_lock.lock().await;
            let mut adapter = mapping.lock();
            let session = adapter.snapshot();
            if session.active {
                adapter.stop_session_detached(&session.session_id).expect("unit stop").retirement
            } else { None }
        };
        finish_backend_retirement(&mapping, &self.handle.mapping_lifecycle_lock, retirement)
            .await.expect("unit mapping worker joined");
        drop(mapping);
        drop(self.service);
        drop(self.handle);
        self.tasks.finish().await.expect("unit fanout joined and fixture files removed");
    }
}

fn map(data: Vec<i8>, revision: u64) -> MapState {
    MapState { width: data.len() as u32, height: 1, resolution: 0.125,
        origin_x: -12.5, origin_y: 3.75, frame_id: "map".into(), revision,
        timestamp_ns: 123_456_789, data }
}
fn assert_map_equal(actual: &MapState, expected: &MapState) {
    assert_eq!(occupancy_grid_meta(actual), occupancy_grid_meta(expected));
    assert_eq!(actual.data, expected.data);
}
fn assert_same_error(actual: Status, expected: Status) {
    assert_eq!(actual.code(), expected.code());
    assert_eq!(actual.message(), expected.message());
}
fn descriptor(revision: u64, count: usize) -> OccupancyShmSnapshot {
    OccupancyShmSnapshot { key: "unit-descriptor-no-file-read".into(), file_size: 4096,
        payload_size: count as u64, revision, layout_version: 2, payload_schema_version: 1,
        generation: revision, active_slot: 0 }
}

#[tokio::test]
async fn unit_effective_rpc_preserves_all_bytes_metadata_and_source_snapshot() {
    let mut fixture = UnitFixture::new().await;
    // This case intentionally covers the existing no-SHM branch.
    fixture.service.bridge.occupancy_shm = None;
    let original = map((0..=255).map(|byte| byte as u8 as i8).collect(), 17);
    fixture.handle.state.write().await.map = original.clone();
    let source_pointer = fixture.handle.state.read().await.map.data.as_ptr() as usize;
    let expected = effective_rpc_reference(&fixture.service).await.unwrap().into_inner();
    let mut actual = GeometryRuntime::get_occupancy_grid_data(&fixture.service, Request::new(Empty {}))
        .await.unwrap().into_inner();
    assert_eq!(actual, expected);
    assert_eq!(actual.meta, Some(occupancy_grid_meta(&original)));
    assert_eq!(actual.data, (0..=255).collect::<Vec<u8>>());
    actual.data.fill(7);
    actual.meta.as_mut().unwrap().frame_id = "changed-by-reader".into();
    {
        let state = fixture.handle.state.read().await;
        assert_map_equal(&state.map, &original);
        assert_eq!(state.map.data.as_ptr() as usize, source_pointer);
    }
    assert_eq!(GeometryRuntime::get_occupancy_grid_data(&fixture.service, Request::new(Empty {}))
        .await.unwrap().into_inner(), expected);
    fixture.finish().await;
}

#[tokio::test]
async fn unit_raw_rpc_preserves_metadata_bytes_and_mapping_owner() {
    let fixture = UnitFixture::new().await;
    fixture.start_mapping_owner();
    let original = map((-1..=100).map(|value| value as i8).collect(), 7);
    let event = crate::lcm_runtime::test_owner_map_revision(
        &mut fixture.handle.mapping_adapter.as_ref().unwrap().lock(), &original);
    assert_eq!(event.revision, original.revision);
    // A different effective SharedState must not be substituted for raw storage.
    fixture.handle.state.write().await.map = map(vec![100, 0], 99);
    let expected = raw_rpc_reference(&fixture.service).await.unwrap().into_inner();
    let mut actual = GeometryRuntime::get_raw_occupancy_grid_data(&fixture.service, Request::new(Empty {}))
        .await.unwrap().into_inner();
    assert_eq!(actual, expected);
    assert_eq!(actual.meta, Some(occupancy_grid_meta(&original)));
    actual.data.fill(0);
    actual.meta.as_mut().unwrap().revision = 999;
    let retained = fixture.handle.mapping_adapter.as_ref().unwrap().lock().raw_map_snapshot().unwrap();
    assert_map_equal(&retained, &original);
    assert_eq!(GeometryRuntime::get_raw_occupancy_grid_data(&fixture.service, Request::new(Empty {}))
        .await.unwrap().into_inner(), expected);
    fixture.finish().await;
}

#[tokio::test]
async fn unit_malformed_grid_ordering_precedes_uncommitted_shm_wait() {
    let fixture = UnitFixture::new().await;
    let valid = map(vec![-1, 0, 42, 100], 7);
    let mut cases = Vec::new();
    let mut bad = valid.clone(); bad.revision = 0; bad.width = 0; bad.resolution = f32::NAN; bad.frame_id.clear(); bad.data.clear();
    cases.push((bad, tonic::Code::FailedPrecondition, "occupancy grid is not ready"));
    let mut bad = valid.clone(); bad.width = 0; bad.resolution = f32::NAN; bad.frame_id.clear();
    cases.push((bad, tonic::Code::Internal, "occupancy grid metadata is invalid: dimensions must be nonzero"));
    let mut bad = valid.clone(); bad.resolution = f32::NAN; bad.frame_id.clear(); bad.data.clear();
    cases.push((bad, tonic::Code::Internal, "occupancy grid metadata is invalid: resolution must be finite and positive"));
    let mut bad = valid.clone(); bad.frame_id = "  ".into(); bad.data.clear();
    cases.push((bad, tonic::Code::Internal, "occupancy grid metadata is invalid: frame_id must not be empty"));
    let mut bad = valid; bad.data.pop();
    cases.push((bad, tonic::Code::Internal, "occupancy grid payload size mismatch: expected=4 actual=3"));
    for (bad, code, message) in cases {
        fixture.handle.state.write().await.map = bad;
        let actual = GeometryRuntime::get_occupancy_grid_data(&fixture.service, Request::new(Empty {}));
        tokio::pin!(actual);
        let error = match futures::poll!(actual.as_mut()) {
            Poll::Ready(Err(error)) => error,
            _ => panic!("malformed data must fail before waiting for SHM"),
        };
        assert_eq!(error.code(), code);
        assert_eq!(error.message(), message);
        assert_same_error(error, effective_rpc_reference(&fixture.service).await.unwrap_err());
    }
    fixture.finish().await;
}

#[tokio::test]
async fn unit_raw_rpc_retains_missing_owner_error_ordering() {
    let mut fixture = UnitFixture::new().await;
    assert_same_error(
        GeometryRuntime::get_raw_occupancy_grid_data(&fixture.service, Request::new(Empty {})).await.unwrap_err(),
        raw_rpc_reference(&fixture.service).await.unwrap_err());
    fixture.service.bridge.mapping_adapter = None;
    let error = GeometryRuntime::get_raw_occupancy_grid_data(&fixture.service, Request::new(Empty {})).await.unwrap_err();
    assert_eq!(error.code(), tonic::Code::Unavailable);
    assert_eq!(error.message(), "mapping adapter is unavailable");
    assert_same_error(error, raw_rpc_reference(&fixture.service).await.unwrap_err());
    fixture.finish().await;
}

#[tokio::test]
async fn unit_effective_rpc_waits_for_captured_revision_and_retains_old_snapshot() {
    let fixture = UnitFixture::new().await;
    fixture.start_mapping_owner();
    let captured = map(vec![-1, 0, 42, 100], 7);
    let event = crate::lcm_runtime::test_owner_map_revision(
        &mut fixture.handle.mapping_adapter.as_ref().unwrap().lock(), &captured);
    fixture.handle.state.write().await.map = captured.clone();
    // Descriptor publication is injected only in this explicitly unit-scoped test.
    // The unchanged real-LCM tests cover the real fanout writer.
    *fixture.handle.occupancy_shm_current.write().await = Some(descriptor(6, 4));
    {
        let actual = GeometryRuntime::get_occupancy_grid_data(&fixture.service, Request::new(Empty {}));
        let reference = effective_rpc_reference(&fixture.service);
        tokio::pin!(actual, reference);
        assert!(futures::poll!(actual.as_mut()).is_pending());
        assert!(futures::poll!(reference.as_mut()).is_pending());
        let newer = map(vec![100, 100], 8);
        {
            let mut state = fixture.handle.state.try_write().expect("snapshot guard released before SHM wait");
            state.map = newer.clone();
        }
        // Merely replacing the shared map must not complete a revision-7 read.
        assert!(futures::poll!(actual.as_mut()).is_pending());
        *fixture.handle.occupancy_shm_current.write().await = Some(descriptor(7, 4));
        fixture.handle.map_revision_latest_tx.send_replace(Some(event));
        let result = actual.await.unwrap().into_inner();
        assert_eq!(result, reference.await.unwrap().into_inner());
        assert_eq!(result.meta, Some(occupancy_grid_meta(&captured)));
        assert_eq!(result.data, borrowed_bytes_reference(&captured.data));
        assert_map_equal(&fixture.handle.state.read().await.map, &newer);
    }
    fixture.finish().await;
}

#[tokio::test]
async fn unit_effective_rpc_preserves_visible_revision_without_descriptor_error() {
    let fixture = UnitFixture::new().await;
    fixture.start_mapping_owner();
    let captured = map(vec![-1, 0, 42, 100], 7);
    let event = crate::lcm_runtime::test_owner_map_revision(
        &mut fixture.handle.mapping_adapter.as_ref().unwrap().lock(), &captured);
    fixture.handle.state.write().await.map = captured;
    *fixture.handle.occupancy_shm_current.write().await = Some(descriptor(6, 4));
    fixture.handle.map_revision_latest_tx.send_replace(Some(event));
    let error = GeometryRuntime::get_occupancy_grid_data(&fixture.service, Request::new(Empty {})).await.unwrap_err();
    assert_eq!(error.code(), tonic::Code::Internal);
    assert_eq!(error.message(), "map revision 7 is visible without a matching occupancy SHM descriptor");
    assert_same_error(error, effective_rpc_reference(&fixture.service).await.unwrap_err());
    fixture.finish().await;
}
