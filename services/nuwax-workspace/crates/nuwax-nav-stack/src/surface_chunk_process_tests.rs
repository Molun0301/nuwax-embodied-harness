//! 真实进程和LCM补取：接收者只取得字节，不读取服务端的本地对象路径。
use super::*;
use nuwax_map_chunks::*;
use nuwax_rpc::{RpcBroker,RpcHandler,RpcRequest,RpcResponse};
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::{Child,Command,Stdio};
use std::sync::atomic::{AtomicUsize,Ordering};

const TEST_MULTICAST_PREFIX: &str = "udpm://239.254.76.68:";
const TEST_RPC_PREFIX: &str = "test/map-block/";

fn assert_test_endpoint(url: &str, name: &str) {
    let port = url.strip_prefix(TEST_MULTICAST_PREFIX).unwrap()
        .strip_suffix("?ttl=0").unwrap().parse::<u16>().unwrap();
    assert!((49_152..=u16::MAX).contains(&port));
    let id = name.strip_prefix(TEST_RPC_PREFIX).unwrap();
    assert_eq!(id.len(), 32);
    assert!(id.bytes().all(|byte| byte.is_ascii_hexdigit()));
    let topics = nuwax_rpc::rpc_topics(name).unwrap();
    assert_eq!(topics.request, format!("/rpc/{name}/req"));
    assert_eq!(topics.response, format!("/rpc/{name}/res"));
}

fn test_endpoint() -> (String, String) {
    let id = ObjectId::random().unwrap();
    let namespace = id.0.iter().map(|byte| format!("{byte:02x}")).collect::<String>();
    let name = format!("{TEST_RPC_PREFIX}{namespace}");
    let first = u16::from_le_bytes([id.0[0], id.0[1]]) as u32;
    for offset in 0..16_384 {
        let port = (49_152 + (first + offset) % 16_384) as u16;
        if let Ok(socket) = std::net::UdpSocket::bind((std::net::Ipv4Addr::LOCALHOST, port)) {
            // 仅探测测试动态端口；liblcm 接管前仍存在端口竞争，不能视作独占预留。
            drop(socket);
            let url = format!("{TEST_MULTICAST_PREFIX}{port}?ttl=0");
            assert_test_endpoint(&url, &name);
            return (url, name);
        }
    }
    panic!("没有可用的测试 LCM 动态端口");
}

struct BlockServer { root:PathBuf, calls:AtomicUsize }
#[async_trait::async_trait]
impl RpcHandler for BlockServer {
    async fn handle(&self,request:RpcRequest)->RpcResponse {
        let count=self.calls.fetch_add(1,Ordering::SeqCst)+1;
        std::fs::write(self.root.join("calls"),count.to_string()).unwrap();
        let mode=std::fs::read_to_string(self.root.join("mode")).unwrap_or_default();
        if mode=="delay" {std::fs::write(self.root.join("mode"),"good").unwrap();tokio::time::sleep(Duration::from_millis(650)).await;}
        let Some(scope) = request.args.0.first() else {
            return RpcResponse::error(request.id, serde_json::json!({"message":"缺少块请求"}));
        };
        let Some(revision) = scope.get("root").and_then(|root| root.get("revision"))
            .and_then(serde_json::Value::as_u64) else {
            return RpcResponse::error(request.id, serde_json::json!({"message":"缺少根修订"}));
        };
        if mode=="missing" {return RpcResponse::error(request.id,serde_json::json!({"message":"块已回收"}));}
        let manifest = std::fs::read(self.root.join(format!("manifest-{revision}.json")))
            .map_err(|e| e.to_string()).and_then(|bytes| Manifest::decode(&bytes));
        let Ok(manifest) = manifest else {
            return RpcResponse::error(request.id, serde_json::json!({"message":"没有此根"}));
        };
        let Some(reference) = manifest.blocks.iter().find(|block| {
            let expected=block_request(&manifest,block);
            ["schema_version","root","block"].iter().all(|field|expected[*field]==scope[*field])
        }) else {
            return RpcResponse::error(request.id, serde_json::json!({"message":"根与精确块引用不符"}));
        };
        let offset=scope["offset"].as_u64().unwrap() as usize;
        let maximum=scope["max_bytes"].as_u64().unwrap() as usize;
        assert!(maximum>0 && maximum<=MAX_BLOCK_TRANSFER_PART_BYTES);
        if mode=="missing-tail" && offset>0 {
            return RpcResponse::error(request.id,serde_json::json!({"message":"后续段已不可用"}));
        }
        if mode=="tail-delay" && offset>0 {
            std::fs::write(self.root.join("mode"),"good").unwrap();
            tokio::time::sleep(Duration::from_millis(650)).await;
        }
        let bytes = Repository::open(&self.root).and_then(|repository|
            repository.read_object_range(reference.object, MAX_BLOCK_BYTES, offset, maximum));
        let Ok(ObjectRangeRead::Data { mut bytes, total_bytes }) = bytes else {
            return RpcResponse::error(request.id, serde_json::json!({"message":"没有此块"}));
        };
        if mode=="corrupt" {bytes[0]^=1;}
        if mode=="corrupt-tail" && offset>0 {bytes[0]^=1;}
        if mode=="truncated-tail" && offset>0 {bytes.pop();}
        let mut response = scope.clone();
        response["total_bytes"] = serde_json::json!(total_bytes);
        response["data_base64"] = serde_json::json!(base64::Engine::encode(
            &base64::engine::general_purpose::STANDARD, &bytes));
        if mode=="changed-total" && offset>0 {response["total_bytes"]=serde_json::json!(total_bytes+1);}
        if mode=="repeated-part" && offset>0 {response["offset"]=serde_json::json!(0);}
        if mode=="wrong-root" {response["root"]["revision"] = serde_json::json!(revision + 1);}
        if mode=="wrong-block" {response["block"]["geometry_revision"] = serde_json::json!(revision + 1);}
        RpcResponse::ok(request.id, response)
    }
}

#[test]
#[ignore="由真实补取进程测试作为子进程调用"]
fn map_block_server_process() {
    let root=PathBuf::from(std::env::var("NUWAX_BLOCK_TEST_ROOT").expect("仅测试提供器"));
    let url=std::env::var("NUWAX_BLOCK_TEST_URL").unwrap();
    let name=std::env::var("NUWAX_BLOCK_TEST_RPC_NAME").unwrap();
    assert_test_endpoint(&url, &name);
    tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap().block_on(async {
        let transport=Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(&url).unwrap());
        let broker=RpcBroker::new();
        broker.register(&name,Box::new(BlockServer{root:root.clone(),calls:AtomicUsize::new(0)})).await.unwrap();
        let server=broker.serve(transport,&name).await.unwrap();
        std::fs::write(root.join("ready"),"ready").unwrap();
        let deadline=Instant::now()+Duration::from_secs(20);
        while !root.join("stop").exists() && Instant::now()<deadline {tokio::time::sleep(Duration::from_millis(10)).await;}
        server.stop().await.unwrap();
    });
}
struct ChildOwner(Child);
impl Drop for ChildOwner {fn drop(&mut self){if self.0.try_wait().ok().flatten().is_none(){let _=self.0.kill();}let _=self.0.wait();}}

fn message(repository:&Repository,revision:u64,variant:u16)->schema::NavigationMap3D {
    message_with_points(repository, revision, variant, 12000)
}
fn message_with_points(repository:&Repository,revision:u64,variant:u16,point_count:u16)->schema::NavigationMap3D {
    let stamp=1_700_000_000_000_000_000+revision;
    let domain=Domain{session:"remote-fine".into(),generation:1,frame_generation:1,producer_generation:17};
    let mut references=Vec::new();
    for x in 0..3 {
        let block=Block::from_iter((0..point_count).map(|id|(id,Evidence{observed_ns:stamp,hits:variant})));
        let bytes=encode_block(&block).unwrap();
        let object = ObjectIdentity::Assigned(ObjectId::random().unwrap());
        repository.put_object(object,&bytes).unwrap();
        references.push(BlockRef{key:[x,0,0],object,block_revision:revision,
            geometry_revision:1,points:block.len() as u32});
    }
    let root=Manifest{source_watermarks:Vec::new(),version:2,domain,revision,geometry_revision:1,
        source_timestamp_ns:stamp,voxel_size_m:0.05,blocks:references}.encode().unwrap();
    std::fs::write(repository.path().join(format!("manifest-{revision}.json")), &root).unwrap();
    schema::NavigationMap3D {
        schema_version:schema::NAVIGATION_MAP_3D_SCHEMA_VERSION,producer_runtime_generation:17,
        committed_map:schema::CommittedMapIdentity{session_id:"remote-fine".into(),session_generation:1,revision:revision as i64,commit_boot_nonce:[4;16],commit_sequence:revision as i64,source_timestamp_ns:stamp as i64,frame_id:"map".into()},
        map_frame_generation:1,cloud_revision:revision as i64,cloud_occupancy_revision:revision as i64,source_timestamp_ns:stamp as i64,source_voxel_size:0.05,
        commit_boot_nonce:[4;16],commit_sequence:revision as i64,cleared:false,virtual_obstacles:None,
        fine:Some(Transfer{root,included:BTreeMap::new()}),
        cloud:schema::PointCloud2{header:schema::Header{seq:revision as i32,stamp_sec:(stamp/1_000_000_000) as i32,stamp_nsec:(stamp%1_000_000_000) as i32,frame_id:"map".into()},height:1,width:0,fields:["x","y","z"].iter().enumerate().map(|(i,name)|schema::PointField{name:(*name).into(),offset:(i*4) as i32,datatype:7,count:1}).collect(),is_bigendian:false,point_step:12,row_step:0,data:Vec::new(),is_dense:true},
    }
}
async fn receive(reader:FineMapReader,message:schema::NavigationMap3D)->(FineMapReader,Result<Arc<Manifest>,SurfaceMapBuildError>) {
    tokio::task::spawn_blocking(move||{let mut reader=reader;let result=reader.receive(&message);(reader,result)}).await.unwrap()
}

#[tokio::test(flavor="multi_thread",worker_threads=2)]
async fn real_process_rpc_recovers_late_missing_and_corrupt_blocks_without_partial_install() {
    let repository=Repository::temporary().unwrap();let root=repository.path();
    let (url, rpc_name)=test_endpoint();
    std::fs::write(root.join("mode"),"delay").unwrap();
    let mut child=ChildOwner(Command::new(std::env::current_exe().unwrap()).args(["--exact","surface_chunks::rpc_process_tests::map_block_server_process","--ignored","--nocapture"])
        .env("NUWAX_BLOCK_TEST_ROOT",root).env("NUWAX_BLOCK_TEST_URL",&url).env("NUWAX_BLOCK_TEST_RPC_NAME",&rpc_name).stdout(Stdio::null()).stderr(Stdio::inherit()).spawn().unwrap());
    let deadline=Instant::now()+Duration::from_secs(3);
    while !root.join("ready").exists(){assert!(Instant::now()<deadline,"块服务未就绪");assert!(child.0.try_wait().unwrap().is_none());tokio::time::sleep(Duration::from_millis(10)).await;}
    let first=message(&repository,1,1);
    let mut reader=FineMapReader::new(url.clone()).unwrap();
    // 仍使用真实 liblcm、RpcClient 和补取协议，仅为此测试注入独占 RPC 名称。
    reader.client=Some(nuwax_rpc::RpcClient::new(
        Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(&url).unwrap()), rpc_name,
    ).unwrap());
    let (reader,result)=receive(reader,first.clone()).await;
    let old=result.unwrap();assert_eq!(old.revision,1);
    let calls=std::fs::read_to_string(root.join("calls")).unwrap().parse::<usize>().unwrap();assert!(calls>=2,"必须实际出现迟到重试");
    let current=reader.receiver.current().unwrap();
    std::fs::write(root.join("mode"),"corrupt").unwrap();
    let (reader,result)=receive(reader,message(&repository,2,2)).await;
    assert!(matches!(&result,Err(SurfaceMapBuildError::Invalid(_))));
    assert!(result.unwrap_err().to_string().contains("格式"));assert_eq!(reader.receiver.current().unwrap(),current);
    std::fs::write(root.join("mode"),"missing").unwrap();
    let (reader,result)=receive(reader,message(&repository,3,3)).await;
    // Remote exceptions have no typed missing-vs-corrupt code; never guess from text.
    assert!(matches!(result,Err(SurfaceMapBuildError::Invalid(_))));assert_eq!(reader.receiver.current().unwrap().revision,1);
    std::fs::write(root.join("mode"),"wrong-root").unwrap();
    let (reader,result)=receive(reader,message(&repository,4,4)).await;
    assert!(matches!(&result,Err(SurfaceMapBuildError::Invalid(_))));
    assert!(result.unwrap_err().to_string().contains("身份"));assert_eq!(reader.receiver.current().unwrap().revision,1);
    std::fs::write(root.join("mode"),"wrong-block").unwrap();
    let (reader,result)=receive(reader,message(&repository,5,5)).await;
    assert!(matches!(&result,Err(SurfaceMapBuildError::Invalid(_))));
    assert!(result.unwrap_err().to_string().contains("身份"));assert_eq!(reader.receiver.current().unwrap().revision,1);
    std::fs::write(root.join("mode"),"good").unwrap();
    let (mut reader,result)=receive(reader,message(&repository,6,6)).await;assert_eq!(result.unwrap().revision,6);
    // 慢读者持有的旧根仍可读，且读取的是消费者自己已验证的对象。
    let (old_columns,count)=reader.columns(&old,None,0.1).unwrap();assert!(!old_columns.is_empty());assert_eq!(count,36000);
    let (reader,result)=receive(reader,first).await;assert!(result.is_err());assert_eq!(reader.receiver.current().unwrap().revision,6);
    drop(reader);
    std::fs::write(root.join("stop"),"stop").unwrap();
    let deadline=Instant::now()+Duration::from_secs(3);
    loop {if let Some(status)=child.0.try_wait().unwrap(){assert!(status.success());break;}assert!(Instant::now()<deadline,"补取服务未结束");tokio::time::sleep(Duration::from_millis(10)).await;}
}

#[tokio::test(flavor="multi_thread", worker_threads=2)]
async fn multipart_rpc_preserves_complete_installation_and_original_deadlines() {
    let repository = Repository::temporary().unwrap();
    let root = repository.path();
    let (url, rpc_name) = test_endpoint();
    std::fs::write(root.join("mode"), "good").unwrap();
    let mut child = ChildOwner(Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "surface_chunks::rpc_process_tests::map_block_server_process", "--ignored", "--nocapture"])
        .env("NUWAX_BLOCK_TEST_ROOT", root).env("NUWAX_BLOCK_TEST_URL", &url)
        .env("NUWAX_BLOCK_TEST_RPC_NAME", &rpc_name)
        .stdout(Stdio::null()).stderr(Stdio::inherit()).spawn().unwrap());
    let deadline = Instant::now() + Duration::from_secs(3);
    while !root.join("ready").exists() {
        assert!(Instant::now() < deadline && child.0.try_wait().unwrap().is_none());
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let mut reader = FineMapReader::new(url.clone()).unwrap();
    reader.client = Some(nuwax_rpc::RpcClient::new(Arc::new(
        nuwax_rpc::LcmRpcTransport::from_provider(&url).unwrap()), &rpc_name).unwrap());
    let first = message_with_points(&repository, 1, 1, 32768);
    let manifest = Manifest::decode(&first.fine.as_ref().unwrap().root).unwrap();
    let block = &manifest.blocks[0];
    let expected = repository.read_object(block.object, MAX_BLOCK_BYTES).unwrap();
    assert_eq!(expected.len(), 393_232);
    assert!(expected.len() > MAX_BLOCK_TRANSFER_PART_BYTES
        && expected.len() < MAX_BLOCK_TRANSFER_PART_BYTES * 2);
    let started = Instant::now();
    let (mut reader, result) = receive(reader, first).await;
    let installed = result.unwrap();
    eprintln!("multipart bytes_per_block={} blocks={} elapsed_ms={:.3}",
        expected.len(), installed.blocks.len(), started.elapsed().as_secs_f64() * 1000.0);
    assert_eq!(reader.receiver.block(block).unwrap().len(), 32768);
    assert_eq!(encode_block(&reader.receiver.block(block).unwrap()).unwrap(), expected);
    let calls = std::fs::read_to_string(root.join("calls")).unwrap().parse::<usize>().unwrap();
    assert!(calls >= manifest.blocks.len() * 2, "every block must cross two real RPC parts");

    for (index, mode) in ["changed-total", "repeated-part", "truncated-tail", "corrupt-tail", "missing-tail"]
        .into_iter().enumerate() {
        std::fs::write(root.join("mode"), mode).unwrap();
        let candidate = message_with_points(&repository, index as u64 + 2, 2, 32768);
        let (next, result) = receive(reader, candidate).await;
        reader = next;
        assert!(matches!(result, Err(SurfaceMapBuildError::Invalid(_))), "mode={mode}: {result:?}");
        assert_eq!(reader.receiver.current().unwrap(), installed,
            "an invalid or missing later part must not install a partial root");
    }
    std::fs::write(root.join("mode"), "tail-delay").unwrap();
    let next = message_with_points(&repository, 8, 3, 32768);
    let (reader, result) = receive(reader, next).await;
    assert_eq!(result.unwrap().revision, 8, "a later part retries within the original shared deadline");
    assert_eq!(reader.receiver.current().unwrap().revision, 8);
    drop(reader);
    std::fs::write(root.join("stop"), "stop").unwrap();
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.0.try_wait().unwrap() { assert!(status.success()); break; }
        assert!(Instant::now() < deadline, "multipart server did not stop");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

#[tokio::test(flavor="multi_thread",worker_threads=2)]
#[ignore="需要本机真实大块路径NUWAX_REAL_BLOCK_FIXTURE，显式执行端到端传输诊断"]
async fn recorded_large_block_crosses_real_process_rpc_with_original_deadlines() {
    let bytes=std::fs::read(std::env::var("NUWAX_REAL_BLOCK_FIXTURE").unwrap()).unwrap();
    assert!(bytes.len()>3_000_000 && bytes.len()<=MAX_BLOCK_BYTES);
    let (points,latest)=block_summary(&bytes).unwrap();
    let repository=Repository::temporary().unwrap();
    let root=repository.path();
    let mut input=message(&repository,1,1);
    let mut manifest=Manifest::decode(&input.fine.as_ref().unwrap().root).unwrap();
    manifest.version=4;
    let mut watermarks=BTreeMap::<Arc<LayerSource>,(u64,u64)>::new();
    for layer in decode_layer_observations(&bytes).unwrap().values() {
        for sample in &layer.samples {
            if !sample.source.is_legacy() {
                let marker=(sample.source_timestamp_ns,sample.source_sequence);
                watermarks.entry(sample.source.clone()).and_modify(|old|*old=(*old).max(marker)).or_insert(marker);
            }
        }
    }
    manifest.source_watermarks=watermarks.into_iter().map(|(source,(source_timestamp_ns,source_sequence))|
        SourceWatermark {source,source_timestamp_ns,source_sequence}).collect();
    let object=ObjectIdentity::Assigned(ObjectId::random().unwrap());
    repository.put_object(object,&bytes).unwrap();
    manifest.source_timestamp_ns=latest.max(manifest.source_timestamp_ns);
    manifest.blocks=vec![BlockRef { key:[0,0,0],object,block_revision:1,
        geometry_revision:1,points }];
    let encoded=manifest.encode().unwrap();
    std::fs::write(root.join("manifest-1.json"),&encoded).unwrap();
    input.fine.as_mut().unwrap().root=encoded;
    input.source_timestamp_ns=manifest.source_timestamp_ns as i64;
    input.committed_map.source_timestamp_ns=input.source_timestamp_ns;
    let (url,rpc_name)=test_endpoint();
    std::fs::write(root.join("mode"),"good").unwrap();
    let mut child=ChildOwner(Command::new(std::env::current_exe().unwrap())
        .args(["--exact","surface_chunks::rpc_process_tests::map_block_server_process","--ignored","--nocapture"])
        .env("NUWAX_BLOCK_TEST_ROOT",root).env("NUWAX_BLOCK_TEST_URL",&url)
        .env("NUWAX_BLOCK_TEST_RPC_NAME",&rpc_name)
        .stdout(Stdio::null()).stderr(Stdio::inherit()).spawn().unwrap());
    let deadline=Instant::now()+Duration::from_secs(3);
    while !root.join("ready").exists() {
        assert!(Instant::now()<deadline && child.0.try_wait().unwrap().is_none());
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    let mut failed=Vec::new();
    for attempt in 0..5 {
        let mut reader=FineMapReader::new(url.clone()).unwrap();
        reader.client=Some(nuwax_rpc::RpcClient::new(Arc::new(
            nuwax_rpc::LcmRpcTransport::from_provider(&url).unwrap()),&rpc_name).unwrap());
        let started=Instant::now();
        let (mut reader,result)=receive(reader,input.clone()).await;
        eprintln!("large_block attempt={attempt} bytes={} elapsed_ms={:.3} result={:?}",
            bytes.len(),started.elapsed().as_secs_f64()*1000.0,result.as_ref().map(|m|m.revision));
        if let Err(error)=result {failed.push(error.to_string());}
        else {
            assert_eq!(reader.receiver.block(&manifest.blocks[0]).unwrap().len(),points as usize);
        }
        drop(reader);
    }
    for mode in ["changed-total","repeated-part"] {
        std::fs::write(root.join("mode"),mode).unwrap();
        let mut reader=FineMapReader::new(url.clone()).unwrap();
        reader.client=Some(nuwax_rpc::RpcClient::new(Arc::new(
            nuwax_rpc::LcmRpcTransport::from_provider(&url).unwrap()),&rpc_name).unwrap());
        let (reader,result)=receive(reader,input.clone()).await;
        assert!(matches!(result,Err(SurfaceMapBuildError::Invalid(_))),"mode={mode} result={result:?}");
        assert!(reader.receiver.current().is_none(),"分段不完整不能安装根");
    }
    std::fs::write(root.join("stop"),"stop").unwrap();
    let deadline=Instant::now()+Duration::from_secs(3);
    loop {
        if let Some(status)=child.0.try_wait().unwrap(){assert!(status.success());break;}
        assert!(Instant::now()<deadline);
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert!(failed.is_empty(),"实际大块仍有补取失败：{failed:?}");
}
