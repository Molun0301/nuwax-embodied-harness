// Paired, explicitly in-process coverage for the real-LCM lifecycle tests.
// Included inside server::lifecycle_tests; all fixtures, repositories and task
// owners below are the existing concrete implementations. This is not LCM or
// gRPC socket coverage and must not replace the corresponding integration tests.
mod navigation_block_unit_transport_tests {
    use super::*;

    const SOURCE_NS: u64 = 1_700_000_000_123_456_789;

    async fn activate_fixture(fixture: &CanonicalResetFixture) {
        assert_eq!(fixture.handle.transport_backend_name, "in-process-unit");
        *fixture.handle.mapping_adapter.as_ref().unwrap().lock() =
            crate::mapping_adapter::MappingAdapter::new_with_backend_for_tests(
                crate::mapping_adapter::MappingAdapterConfig {
                    resolution: 0.1,
                    initial_safe_radius_meters: 0.0,
                },
                crate::mapping_backend::MappingBackendSpec::Voxel,
                "map",
                "odom",
            );
        let xyz = [[0.025_f32, 0.025, 0.025], [1.025_f32, 0.025, 0.025]]
            .into_iter()
            .flatten()
            .flat_map(f32::to_le_bytes)
            .collect();
        let mut request = cloud_activation_request(xyz);
        request
            .checkpoint
            .as_mut()
            .unwrap()
            .meta
            .as_mut()
            .unwrap()
            .timestamp_ns = SOURCE_NS;
        let activated = GeometryRuntime::activate_map(&fixture.service, Request::new(request))
            .await
            .expect("explicit in-process cloud activation")
            .into_inner();
        assert!(activated.success && activated.active);
    }

    fn block_case(
        fixture: &CanonicalResetFixture,
    ) -> (
        Arc<nuwax_map_chunks::SealedMap>,
        NavigationMapBlockRequest,
        Vec<u8>,
    ) {
        let fine = fixture
            .handle
            .cloud_map_writer
            .lock()
            .as_ref()
            .unwrap()
            .committed_fine_cloud()
            .unwrap();
        let root = fine.manifest();
        let reference = root.blocks.first().expect("activated fine block");
        let object = match reference.object {
            nuwax_map_chunks::ObjectIdentity::Assigned(id) => NavigationMapObjectIdentity {
                kind: 1,
                object_id: id.0.to_vec(),
                legacy_alias: Vec::new(),
            },
            nuwax_map_chunks::ObjectIdentity::Legacy(id) => NavigationMapObjectIdentity {
                kind: 2,
                object_id: Vec::new(),
                legacy_alias: id.0.to_vec(),
            },
        };
        let request = NavigationMapBlockRequest {
            schema_version: 3,
            root: Some(NavigationMapRootIdentity {
                revision: root.revision,
                domain: Some(NavigationMapDomain {
                    session: root.domain.session.clone(),
                    generation: root.domain.generation,
                    frame_generation: root.domain.frame_generation,
                    producer_generation: root.domain.producer_generation,
                }),
            }),
            block: Some(NavigationMapBlockRef {
                key: reference.key.to_vec(),
                object: Some(object),
                block_revision: reference.block_revision,
                geometry_revision: reference.geometry_revision,
                points: reference.points,
            }),
            offset: 0,
            max_bytes: 7,
        };
        let expected = fine
            .repository
            .read_object(reference.object, nuwax_map_chunks::MAX_BLOCK_BYTES)
            .expect("actual repository bytes, not a second encoder");
        assert!(expected.len() > 7);
        assert!(expected.len() <= nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES);
        (fine, request, expected)
    }

    async fn fetch(
        fixture: &CanonicalResetFixture,
        request: NavigationMapBlockRequest,
    ) -> NavigationMapBlockResponse {
        let response = GeometryRuntime::get_navigation_map_block(
            &fixture.service,
            Request::new(request.clone()),
        )
        .await
        .expect("actual in-process service block read")
        .into_inner();
        assert_eq!(response.schema_version, request.schema_version);
        assert_eq!(response.root, request.root);
        assert_eq!(response.block, request.block);
        assert_eq!(response.offset, request.offset);
        response
    }

    async fn assert_status(
        fixture: &CanonicalResetFixture,
        request: NavigationMapBlockRequest,
        code: tonic::Code,
        message: Option<&str>,
    ) {
        let error =
            GeometryRuntime::get_navigation_map_block(&fixture.service, Request::new(request))
                .await
                .expect_err("service rejects the precise invalid state");
        assert_eq!(error.code(), code, "unexpected error: {error}");
        if let Some(message) = message {
            assert_eq!(error.message(), message);
        }
    }

    // Repository::object_path is intentionally private. This helper mirrors
    // only its on-disk namespace so a test can remove/watch the exact object;
    // it is never used to implement or replace a service read.
    fn object_path(
        fine: &nuwax_map_chunks::SealedMap,
        request: &NavigationMapBlockRequest,
    ) -> std::path::PathBuf {
        let identity = request.block.as_ref().unwrap().object.as_ref().unwrap();
        let (prefix, bytes) = if identity.kind == 1 {
            ("id-", &identity.object_id)
        } else {
            ("", &identity.legacy_alias)
        };
        let hex = bytes
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<String>();
        fine.repository.path().join(format!("{prefix}{hex}"))
    }

    #[tokio::test]
    async fn unit_transport_navigation_block_wire_versions_and_short_final_part_match_repository() {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            activate_fixture(&fixture).await;
            let (_fine, request, expected) = block_case(&fixture);
            for schema_version in [2, 3] {
                let mut full = request.clone();
                full.schema_version = schema_version;
                full.max_bytes = if schema_version == 2 {
                    0
                } else {
                    nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES as u32
                };
                let response = fetch(&fixture, full).await;
                assert_eq!(response.data, expected);
                assert_eq!(
                    response.total_bytes,
                    if schema_version == 2 {
                        0
                    } else {
                        expected.len() as u32
                    }
                );
            }
            // Choose a small non-divisor, so this fixture necessarily exercises
            // a short final part rather than only full-width transfer parts.
            let part = (2_usize..=31)
                .find(|part| expected.len() % part != 0)
                .unwrap();
            let mut joined = Vec::new();
            let mut last_len = 0;
            while joined.len() < expected.len() {
                let offset = joined.len();
                let mut next = request.clone();
                next.offset = offset as u32;
                next.max_bytes = part as u32;
                let response = fetch(&fixture, next).await;
                assert_eq!(response.total_bytes as usize, expected.len());
                last_len = part.min(expected.len() - offset);
                assert_eq!(response.data, expected[offset..offset + last_len]);
                joined.extend_from_slice(&response.data);
            }
            assert!(last_len > 0 && last_len < part);
            assert_eq!(joined, expected);
            let mut final_byte = request;
            final_byte.offset = expected.len() as u32 - 1;
            final_byte.max_bytes = nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES as u32;
            let response = fetch(&fixture, final_byte).await;
            assert_eq!(response.total_bytes as usize, expected.len());
            assert_eq!(response.data, expected[expected.len() - 1..]);
        }))
        .await;
        fixture.finish().await;
        if let Err(panic) = outcome {
            std::panic::resume_unwind(panic);
        }
    }

    #[tokio::test]
    async fn unit_transport_navigation_block_range_errors_distinguish_admission_bounds_and_missing_object(
    ) {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            activate_fixture(&fixture).await;
            let (fine, request, expected) = block_case(&fixture);
            for schema_version in [0, 1, 4] {
                let mut invalid = request.clone();
                invalid.schema_version = schema_version;
                assert_status(
                    &fixture,
                    invalid,
                    tonic::Code::InvalidArgument,
                    Some("unsupported fine-map block protocol version"),
                )
                .await;
            }
            for (schema_version, offset, max_bytes) in [
                (2, 1, 0),
                (2, 0, 1),
                (3, 0, 0),
                (
                    3,
                    0,
                    nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES as u32 + 1,
                ),
                (3, nuwax_map_chunks::MAX_BLOCK_BYTES as u32, 7),
                (3, u32::MAX, 7),
            ] {
                let mut invalid = request.clone();
                invalid.schema_version = schema_version;
                invalid.offset = offset;
                invalid.max_bytes = max_bytes;
                assert_status(
                    &fixture,
                    invalid,
                    tonic::Code::InvalidArgument,
                    Some("invalid fine-map block range"),
                )
                .await;
            }
            for offset in [expected.len() as u32, expected.len() as u32 + 1] {
                let mut beyond = request.clone();
                beyond.offset = offset;
                assert_status(
                    &fixture,
                    beyond,
                    tonic::Code::InvalidArgument,
                    Some("fine-map block offset exceeds object"),
                )
                .await;
            }
            let mut expired = request.clone();
            expired.root.as_mut().unwrap().revision += 1;
            assert_status(
                &fixture,
                expired,
                tonic::Code::FailedPrecondition,
                Some("requested fine-map root expired; request the latest complete root"),
            )
            .await;
            let mut wrong_member = request.clone();
            wrong_member.block.as_mut().unwrap().block_revision += 1;
            assert_status(
                &fixture,
                wrong_member,
                tonic::Code::FailedPrecondition,
                Some("requested block identity/revisions differ from the retained root"),
            )
            .await;
            // Leave root and reference intact. Only a valid pinned membership
            // with an absent backing object reaches Repository's NotFound path.
            let path = object_path(&fine, &request);
            assert_eq!(std::fs::read(&path).unwrap(), expected);
            std::fs::remove_file(path).expect("remove only this fixture's object");
            for offset in [0, expected.len() as u32] {
                let mut missing = request.clone();
                missing.offset = offset;
                assert_status(&fixture, missing, tonic::Code::NotFound, None).await;
            }
            // Entry validation still wins over missing-file IO. Repository
            // opens an otherwise valid request before checking object length.
            let mut invalid_missing = request;
            invalid_missing.max_bytes = 0;
            assert_status(
                &fixture,
                invalid_missing,
                tonic::Code::InvalidArgument,
                Some("invalid fine-map block range"),
            )
            .await;
        }))
        .await;
        fixture.finish().await;
        if let Err(panic) = outcome {
            std::panic::resume_unwind(panic);
        }
    }

    #[tokio::test]
    async fn unit_transport_retained_navigation_block_survives_first_pose_frame_rehome() {
        let fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            activate_fixture(&fixture).await;
            let (fine, request, expected) = block_case(&fixture);
            let old_manifest = fine.manifest().as_ref().clone();
            let old_owner = Arc::downgrade(&fine);
            assert!(fixture
                .handle
                .mapping_adapter
                .as_ref()
                .unwrap()
                .lock()
                .map_odom_transform_snapshot()
                .is_none());
            // Do not let a caller-owned Arc make the old root artificially
            // pinnable through the writer's weak-reader list.
            drop(fine);
            commit_checkpoint_frame_test_pose(&fixture.handle, SOURCE_NS + 17_000_001).await;
            let current = fixture
                .handle
                .cloud_map_writer
                .lock()
                .as_ref()
                .unwrap()
                .committed_fine_cloud()
                .unwrap();
            let mut expected_manifest = old_manifest.clone();
            expected_manifest.domain.frame_generation += 1;
            assert_eq!(current.manifest().as_ref(), &expected_manifest);
            let retained = old_owner
                .upgrade()
                .expect("writer, not caller, retained the old root");
            assert!(!Arc::ptr_eq(&current, &retained));
            drop(retained);
            for schema_version in [2, 3] {
                let mut old = request.clone();
                old.schema_version = schema_version;
                old.max_bytes = if schema_version == 2 { 0 } else { 7 };
                if schema_version == 2 {
                    let response = fetch(&fixture, old).await;
                    assert_eq!(response.data, expected);
                    assert_eq!(response.total_bytes, 0);
                } else {
                    let mut joined = Vec::new();
                    while joined.len() < expected.len() {
                        old.offset = joined.len() as u32;
                        let response = fetch(&fixture, old.clone()).await;
                        assert_eq!(response.total_bytes as usize, expected.len());
                        let end = (joined.len() + 7).min(expected.len());
                        assert_eq!(response.data, expected[joined.len()..end]);
                        joined.extend_from_slice(&response.data);
                    }
                    assert_eq!(joined, expected);
                }
            }
            assert_eq!(
                fixture
                    .handle
                    .mapping_adapter
                    .as_ref()
                    .unwrap()
                    .lock()
                    .global_frame_generation(),
                expected_manifest.domain.frame_generation
            );
            assert_eq!(current.manifest().blocks, old_manifest.blocks);
        }))
        .await;
        fixture.finish().await;
        if let Err(panic) = outcome {
            std::panic::resume_unwind(panic);
        }
    }

    include!("navigation_block_cancel_unit_transport_tests.rs");
}
