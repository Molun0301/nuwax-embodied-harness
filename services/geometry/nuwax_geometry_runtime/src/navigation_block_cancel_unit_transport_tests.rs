// This test covers cancellation after admission and before blocking IO can run.
// It does not claim a deterministic mid-read syscall cancellation boundary.
#[test]
fn unit_transport_cancelled_navigation_block_request_keeps_admitted_read_owned() {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .max_blocking_threads(1)
        .build()
        .unwrap();
    runtime.block_on(async {
        let mut fixture = CanonicalResetFixture::new().await;
        let outcome = futures::FutureExt::catch_unwind(std::panic::AssertUnwindSafe(async {
            // All fixture construction/activation and the positive read finish
            // before we occupy this runtime's only blocking worker.
            activate_fixture(&fixture).await;
            let (fine, request, expected) = block_case(&fixture);
            let baseline = fetch(&fixture, request.clone()).await;
            assert_eq!(baseline.data, expected[..7]);
            let (entered_tx, entered_rx) = tokio::sync::oneshot::channel();
            let (release_tx, release_rx) = std::sync::mpsc::channel::<()>();
            // Sender drop releases the worker even if any assertion unwinds.
            let blocker = tokio::task::spawn_blocking(move || {
                let _ = entered_tx.send(());
                let _ = release_rx.recv();
            });
            tokio::time::timeout(Duration::from_secs(1), entered_rx)
                .await
                .unwrap()
                .unwrap();
            let owners_before = Arc::strong_count(&fine);
            let mut cancelled = Box::pin(tokio::task::unconstrained(
                GeometryRuntime::get_navigation_map_block(
                    &fixture.service,
                    Request::new(request.clone()),
                ),
            ));
            assert!(futures::poll!(cancelled.as_mut()).is_pending());
            assert_eq!(
                Arc::strong_count(&fine),
                owners_before + 1,
                "the actual service pinned this exact root before yielding at its receiver"
            );
            // The current-thread runtime cannot run a second task between the
            // baseline count and this poll. The service does not yield between
            // its synchronous pin/register calls and awaiting the receiver.
            drop(cancelled);
            assert_eq!(
                Arc::strong_count(&fine),
                owners_before + 1,
                "dropping the request must not release the admitted read's root"
            );
            let mut stopping = Box::pin(fixture.task_owner.shutdown());
            assert!(
                tokio::time::timeout(Duration::from_millis(25), stopping.as_mut())
                    .await
                    .is_err(),
                "owner cannot settle while its blocking read cannot run"
            );
            assert_eq!(
                Arc::strong_count(&fine),
                owners_before + 1,
                "the read retains the exact root while shutdown is pending"
            );
            drop(release_tx);
            blocker.await.unwrap();
            let settled = tokio::time::timeout(Duration::from_secs(1), stopping)
                .await
                .unwrap();
            assert!(settled.producers_settled());
            settled
                .result
                .expect("cancelled-request read remains root owned through settlement");
            assert_eq!(
                Arc::strong_count(&fine),
                owners_before,
                "the read released its pin before shutdown reported settlement"
            );
            assert_status(
                &fixture,
                request,
                tonic::Code::FailedPrecondition,
                Some("Geometry task admission is closed: navigation-map-operation"),
            )
            .await;
        }))
        .await;
        fixture.finish().await;
        if let Err(panic) = outcome {
            std::panic::resume_unwind(panic);
        }
    });
}
