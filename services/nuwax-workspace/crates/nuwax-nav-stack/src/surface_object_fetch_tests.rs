// 新协议走真实内存LCM与原接收器；失败候选已经取得的对象必须留给下次复用。
use nuwax_rpc::{RpcBroker, RpcHandler, RpcRequest, RpcResponse};
use std::sync::{Mutex, atomic::{AtomicBool, Ordering}};

struct Objects {
    repository: Repository,
    calls: Arc<Mutex<BTreeMap<ObjectIdentity, usize>>>,
    fail: ObjectIdentity,
    unavailable: AtomicBool,
    pause: ObjectIdentity,
    arrived: Arc<tokio::sync::Notify>,
    release: Arc<tokio::sync::Notify>,
}
#[async_trait::async_trait]
impl RpcHandler for Objects {
    async fn handle(&self, request: RpcRequest) -> RpcResponse {
        use base64::{engine::general_purpose::STANDARD, Engine as _};
        let scope = &request.args.0[0];
        assert_eq!(scope.as_object().unwrap().len(), 2);
        let object = ObjectIdentity::Assigned(ObjectId(serde_json::from_value(scope["object_id"].clone()).unwrap()));
        let count = { let mut calls=self.calls.lock().unwrap(); let count=calls.entry(object).or_default(); *count+=1; *count };
        if object==self.pause && count==1 { self.arrived.notify_one(); self.release.notified().await; }
        if object == self.fail && self.unavailable.swap(false, Ordering::SeqCst) {
            return RpcResponse::error(request.id, serde_json::json!({"message":"地图块已被新修订替换或地图已切换"}));
        }
        let ObjectRangeRead::Data { bytes, total_bytes } = self.repository.read_object_range(object,
            MAX_BLOCK_BYTES, scope["offset"].as_u64().unwrap() as usize, MAX_BLOCK_TRANSFER_PART_BYTES).unwrap() else { panic!("分段偏移错误"); };
        RpcResponse::ok(request.id, serde_json::json!({"total_bytes":total_bytes,"data_base64":STANDARD.encode(bytes)}))
    }
}

#[tokio::test(flavor="multi_thread", worker_threads=2)]
async fn failed_candidate_keeps_downloaded_objects_for_a_newer_complete_root() {
    let source = Repository::temporary().unwrap();
    let mut writer = Writer::new(source.clone(), 0.05, 4*1024*1024).unwrap();
    let mut reader = FineMapReader::new("memq://".into()).unwrap();
    writer.observe([[0,0,0]], 1).unwrap();
    let domain = Domain {session:"持续建图".into()};
    let first = writer.snapshot(domain.clone(), 1, 1).unwrap();
    reader.receive(&transfer_message(&source, &first)).unwrap();
    writer.observe([[32,0,0],[64,0,0]], 2).unwrap();
    let next = writer.snapshot(domain.clone(), 2, 2).unwrap();
    let calls = Arc::new(Mutex::new(BTreeMap::new()));
    let transport = Arc::new(nuwax_rpc::LcmRpcTransport::from_provider("memq://").unwrap());
    let broker = RpcBroker::new();
    let arrived=Arc::new(tokio::sync::Notify::new());let release=Arc::new(tokio::sync::Notify::new());
    let method = "test/e5/object-fetch";
    broker.register(method, Box::new(Objects {repository:source.clone(), calls:calls.clone(),
        fail:next.blocks[2].object, unavailable:AtomicBool::new(true),
        pause:next.blocks[1].object, arrived:arrived.clone(), release:release.clone()})).await.unwrap();
    let server = broker.serve_many(transport.clone(), [method]).await.unwrap();
    reader.client = Some(nuwax_rpc::RpcClient::new(transport, method).unwrap());
    let message = {
        let root = &next;
        let mut message = transfer_message(&source, root);
        message.fine.as_mut().unwrap().included.clear();
        message
    };
    let reception = tokio::task::spawn_blocking(move || {
        let result = reader.receive(&message); (reader, result)
    });
    arrived.notified().await;
    // 根2的真实分段请求在途时，发布端连续提交三次；对象不再依赖旧根保留。
    let mut newest = first;
    for revision in 3..=5 { newest = writer.snapshot(domain.clone(), revision, revision).unwrap(); }
    release.notify_one();
    let (mut reader,result)=reception.await.unwrap();
    assert!(result.is_err());
    assert!(!result.unwrap_err().contains("请求的地图修订已不可用"));
    assert_eq!(reader.receiver.current().unwrap().revision, 1);
    let objects: Vec<_> = newest.blocks.iter().map(|b| b.object).collect();
    let message = {
        let mut message = transfer_message(&source, &newest);
        message.fine.as_mut().unwrap().included.clear(); message
    };
    let result = tokio::task::spawn_blocking(move || reader.receive(&message)).await.unwrap().unwrap();
    assert_eq!(result.revision, 5);
    let counts = calls.lock().unwrap();
    assert_eq!(counts.get(&objects[0]), None);
    assert_eq!(counts[&objects[1]], 1);
    assert_eq!(counts[&objects[2]], 2);
    drop(counts);
    server.stop().await.unwrap();
}

#[test]
fn object_reply_checks_only_total_and_decoded_part_length() {
    use base64::{engine::general_purpose::STANDARD, Engine as _};
    let request = serde_json::json!({"object_id":vec![1;16],"offset":0});
    let bytes = vec![7; MAX_BLOCK_TRANSFER_PART_BYTES];
    let compressed = nuwax_codec::lcm::map_block_part::compress(&bytes).unwrap();
    let reply = serde_json::json!({"total_bytes":bytes.len(),"data_lz4_base64":STANDARD.encode(compressed)});
    assert_eq!(block_response(&request,&reply).unwrap(),bytes);
    let reply = serde_json::json!({"total_bytes":4,"data_base64":"AAH+/w=="});
    assert_eq!(block_response(&request,&reply).unwrap(),[0,1,254,255]);
    for total in [3,5,MAX_BLOCK_BYTES+1] {
        let mut wrong=reply.clone();wrong["total_bytes"]=serde_json::json!(total);
        assert!(block_response(&request,&wrong).is_err());
    }
    let tail=serde_json::json!({"offset":MAX_BLOCK_BYTES-4});
    let mut reply=reply;reply["total_bytes"]=serde_json::json!(MAX_BLOCK_BYTES);
    assert_eq!(block_response(&tail,&reply).unwrap(),[0,1,254,255]);
}

#[tokio::test(flavor="multi_thread", worker_threads=2)]
async fn multipart_fetch_calls_each_part_once_and_timeout_abandons_candidate() {
    use std::sync::atomic::{AtomicUsize, Ordering};
    struct Parts { calls: Arc<Mutex<Vec<usize>>>, mode: Arc<AtomicUsize>, bytes: Arc<Vec<u8>> }
    #[async_trait::async_trait]
    impl RpcHandler for Parts {
        async fn handle(&self, request: RpcRequest) -> RpcResponse {
            use base64::{engine::general_purpose::STANDARD, Engine as _};
            let offset = request.args.0[0]["offset"].as_u64().unwrap() as usize;
            self.calls.lock().unwrap().push(offset);
            match self.mode.load(Ordering::SeqCst) {
                1 => tokio::time::sleep(Duration::from_millis(500)).await,
                2 => return RpcResponse::error(request.id,serde_json::json!({"message":"块不可用"})),
                _ => {},
            }
            let end=(offset+MAX_BLOCK_TRANSFER_PART_BYTES).min(self.bytes.len());
            RpcResponse::ok(request.id,serde_json::json!({"total_bytes":self.bytes.len(),
                "data_base64":STANDARD.encode(&self.bytes[offset..end])}))
        }
    }
    let repository = Repository::temporary().unwrap();
    let root=install(&repository,&BTreeSet::from([[0,0,0]]),0.05,1);
    let calls=Arc::new(Mutex::new(Vec::new()));
    let mode=Arc::new(AtomicUsize::new(0));
    let bytes=Arc::new(vec![37; MAX_BLOCK_TRANSFER_PART_BYTES*5+4]);
    let broker=RpcBroker::new();
    let transport=Arc::new(nuwax_rpc::LcmRpcTransport::from_provider("memq://").unwrap());
    let method="test/e5/parts";
    broker.register(method,Box::new(Parts {calls:calls.clone(),mode:mode.clone(),bytes:bytes.clone()})).await.unwrap();
    let server=broker.serve_many(transport.clone(),[method]).await.unwrap();
    let client=nuwax_rpc::RpcClient::new(transport,method).unwrap();
    assert_eq!(fetch_block_parts(&client,&root.blocks[0]).await.unwrap(),*bytes);
    let mut offsets=calls.lock().unwrap().clone();offsets.sort_unstable();
    assert_eq!(offsets,(0..bytes.len()).step_by(MAX_BLOCK_TRANSFER_PART_BYTES).collect::<Vec<_>>());
    for value in [1,2] {
        calls.lock().unwrap().clear();mode.store(value,Ordering::SeqCst);
        let started=Instant::now();
        assert!(fetch_block_parts(&client,&root.blocks[0]).await.is_err());
        assert_eq!(*calls.lock().unwrap(),vec![0],"超时和远程异常都不能段内重试");
        if value==1 { assert!(started.elapsed() >= Duration::from_millis(350)); }
    }
    server.stop().await.unwrap();
}
