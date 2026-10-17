#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn missing_rpc_is_typed_incomplete_and_recovery_commits_one_complete_root() {
    let repository = Repository::temporary().unwrap();
    let cells = BTreeSet::from([[0, 0, 0], [40, 0, 0]]);
    let first = install_revisioned(&repository, &cells, 0.05, 1, None);
    let second = install_revisioned(
        &repository,
        &BTreeSet::from([[0, 0, 1], [40, 0, 1]]),
        0.05,
        2,
        Some(&first),
    );
    // Real isolated memq RPC client has no block provider; no production endpoint.
    let mut reader = FineMapReader::new("memq://".into()).unwrap();
    let old = reader
        .receive(&transfer_message(&repository, &first))
        .unwrap();
    let mut partial = transfer_message(&repository, &second);
    let last_object = partial
        .fine
        .as_ref()
        .unwrap()
        .included
        .keys()
        .next_back()
        .copied()
        .unwrap();
    partial.fine.as_mut().unwrap().included.remove(&last_object);
    let (mut reader, error) = tokio::task::spawn_blocking(move || {
        let error = reader.receive(&partial).unwrap_err();
        (reader, error)
    })
    .await
    .unwrap();
    assert!(
        matches!(error, SurfaceMapBuildError::IncompleteFineSource(_)),
        "{error}"
    );
    assert!(Arc::ptr_eq(&reader.receiver.current().unwrap(), &old));
    let observations = reader.observations(&old, None).unwrap();
    assert!(observations.get([0, 0, 0]).is_some());
    assert!(
        observations.get([0, 0, 1]).is_none(),
        "partial candidate cannot enter old-root observations"
    );
    let recovered = reader
        .receive(&transfer_message(&repository, &second))
        .unwrap();
    assert_eq!(*recovered, second);
    assert!(Arc::ptr_eq(&reader.receiver.current().unwrap(), &recovered));
    let observations = reader.observations(&recovered, None).unwrap();
    assert!(observations.get([0, 0, 0]).is_none());
    assert!(observations.get([0, 0, 1]).is_some());
    assert!(observations.get([40, 0, 1]).is_some());
    // Holding the old immutable root remains honest after atomic installation.
    assert!(reader
        .observations(&old, None)
        .unwrap()
        .get([40, 0, 0])
        .is_some());
}

#[test]
fn corrupt_root_and_corrupt_block_are_invalid_not_retryable() {
    let repository = Repository::temporary().unwrap();
    let root = install_revisioned(&repository, &BTreeSet::from([[0, 0, 0]]), 0.05, 1, None);
    let mut message = transfer_message(&repository, &root);
    let mut reader = FineMapReader::new("memq://".into()).unwrap();
    message.fine.as_mut().unwrap().root[0] ^= 1;
    assert!(matches!(
        reader.receive(&message),
        Err(SurfaceMapBuildError::Invalid(_))
    ));
    assert!(reader.receiver.current().is_none());
    let mut message = transfer_message(&repository, &root);
    message
        .fine
        .as_mut()
        .unwrap()
        .included
        .values_mut()
        .next()
        .unwrap()[0] ^= 1;
    assert!(matches!(
        reader.receive(&message),
        Err(SurfaceMapBuildError::Invalid(_))
    ));
    assert!(reader.receiver.current().is_none());
}

#[test]
fn fetched_identity_and_body_corruption_never_become_incomplete() {
    let repository = Repository::temporary().unwrap();
    let root = install_revisioned(&repository, &BTreeSet::from([[0, 0, 0]]), 0.05, 1, None);
    let mut transfer = transfer_message(&repository, &root).fine.unwrap();
    transfer.included.clear();
    for wrong_scope in [false, true] {
        let mut receiver = Receiver::new(Repository::temporary().unwrap());
        let result =
            receive_with_fetch(&mut receiver, &transfer, &root.domain, |root, reference| {
                let request = block_request(root, reference);
                let mut response = request.clone();
                use base64::{engine::general_purpose::STANDARD, Engine as _};
                let data=if wrong_scope {
                    repository
                        .read_object(reference.object, MAX_BLOCK_BYTES)
                        .unwrap()
                } else {
                    vec![0_u8; 16]
                };
                response["total_bytes"] = serde_json::json!(data.len());
                response["data_base64"] = serde_json::json!(STANDARD.encode(data));
                if wrong_scope {
                    response["root"]["revision"] = serde_json::json!(999);
                }
                block_response(&request, &response).map_err(Into::into)
            });
        assert!(matches!(result, Err(SurfaceMapBuildError::Invalid(_))));
        assert!(receiver.current().is_none());
    }
}
