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
        let Some(reference) = manifest.blocks.iter().find(|block| block_request(&manifest, block) == *scope) else {
            return RpcResponse::error(request.id, serde_json::json!({"message":"根与精确块引用不符"}));
        };
        let bytes = Repository::open(&self.root).and_then(|repository|
            repository.read_object(reference.object, MAX_BLOCK_BYTES));
        let Ok(mut bytes) = bytes else {
            return RpcResponse::error(request.id, serde_json::json!({"message":"没有此块"}));
        };
        if mode=="corrupt" {bytes[0]^=1;}
        let mut response = scope.clone();
        response["data"] = serde_json::json!(bytes);
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
    let stamp=1_700_000_000_000_000_000+revision;
    let domain=Domain{session:"remote-fine".into(),generation:1,frame_generation:1,producer_generation:17};
    let mut references=Vec::new();
    for x in 0..3 {
        let block=Block::from_iter((0..12000).map(|id|(id,Evidence{observed_ns:stamp,hits:variant})));
        let bytes=encode_block(&block).unwrap();
        let object = ObjectIdentity::Assigned(ObjectId::random().unwrap());
        repository.put_object(object,&bytes).unwrap();
        references.push(BlockRef{key:[x,0,0],object,block_revision:revision,
            geometry_revision:1,points:block.len() as u32});
    }
    let root=Manifest{version:2,domain,revision,geometry_revision:1,
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
async fn receive(reader:FineMapReader,message:schema::NavigationMap3D)->(FineMapReader,Result<Arc<Manifest>,String>) {
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
    assert!(result.unwrap_err().contains("格式"));assert_eq!(reader.receiver.current().unwrap(),current);
    std::fs::write(root.join("mode"),"missing").unwrap();
    let (reader,result)=receive(reader,message(&repository,3,3)).await;
    assert!(result.is_err());assert_eq!(reader.receiver.current().unwrap().revision,1);
    std::fs::write(root.join("mode"),"wrong-root").unwrap();
    let (reader,result)=receive(reader,message(&repository,4,4)).await;
    assert!(result.unwrap_err().contains("身份"));assert_eq!(reader.receiver.current().unwrap().revision,1);
    std::fs::write(root.join("mode"),"wrong-block").unwrap();
    let (reader,result)=receive(reader,message(&repository,5,5)).await;
    assert!(result.unwrap_err().contains("身份"));assert_eq!(reader.receiver.current().unwrap().revision,1);
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
