//! 真实进程和LCM补取：接收者只取得字节，不读取服务端的本地对象路径。
use super::*;
use nuwax_map_chunks::*;
use nuwax_rpc::{RpcBroker,RpcHandler,RpcRequest,RpcResponse};
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::{Child,Command,Stdio};
use std::sync::atomic::{AtomicUsize,Ordering};

struct BlockServer { root:PathBuf, calls:AtomicUsize }
#[async_trait::async_trait]
impl RpcHandler for BlockServer {
    async fn handle(&self,request:RpcRequest)->RpcResponse {
        let count=self.calls.fetch_add(1,Ordering::SeqCst)+1;
        std::fs::write(self.root.join("calls"),count.to_string()).unwrap();
        let mode=std::fs::read_to_string(self.root.join("mode")).unwrap_or_default();
        if mode=="delay" {std::fs::write(self.root.join("mode"),"good").unwrap();tokio::time::sleep(Duration::from_millis(650)).await;}
        let Some(hash)=request.args.0.first().and_then(|v|v.get("sha256_hex")).and_then(|v|v.as_str()) else{return RpcResponse::error(request.id,serde_json::json!({"message":"缺少摘要"}));};
        if mode=="missing" {return RpcResponse::error(request.id,serde_json::json!({"message":"块已回收"}));}
        let Ok(mut bytes)=std::fs::read(self.root.join(hash)) else{return RpcResponse::error(request.id,serde_json::json!({"message":"没有此块"}));};
        if mode=="corrupt" {bytes[0]^=1;}
        RpcResponse::ok(request.id,serde_json::json!({"data_hex":bytes.iter().map(|v|format!("{v:02x}")).collect::<String>()}))
    }
}

#[test]
#[ignore="由真实补取进程测试作为子进程调用"]
fn map_block_server_process() {
    let root=PathBuf::from(std::env::var("NUWAX_BLOCK_TEST_ROOT").expect("仅测试提供器"));
    let url=std::env::var("NUWAX_BLOCK_TEST_URL").unwrap();
    tokio::runtime::Builder::new_multi_thread().worker_threads(2).enable_all().build().unwrap().block_on(async {
        let transport=Arc::new(nuwax_rpc::LcmRpcTransport::from_provider(&url).unwrap());
        let broker=RpcBroker::new();let name="GeometryRuntimeService/get_navigation_map_block";
        broker.register(name,Box::new(BlockServer{root:root.clone(),calls:AtomicUsize::new(0)})).await.unwrap();
        let server=broker.serve(transport,name).await.unwrap();
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
        let bytes=encode_block(&block).unwrap();let hash=digest(&bytes);repository.put_object(hash,&bytes).unwrap();
        references.push(BlockRef{key:[x,0,0],sha256:hash,geometry_sha256:block_geometry_digest(&block),points:block.len() as u32});
    }
    let root=Manifest{version:1,domain,revision,source_timestamp_ns:stamp,voxel_size_m:0.05,blocks:references}.encode().unwrap();
    schema::NavigationMap3D {
        schema_version:2,producer_runtime_generation:17,
        committed_map:schema::CommittedMapIdentity{session_id:"remote-fine".into(),session_generation:1,revision:revision as i64,payload_sha256:[3;32],frame_id:"map".into()},
        map_frame_generation:1,cloud_revision:revision as i64,source_timestamp_ns:stamp as i64,source_voxel_size:0.05,
        commit_boot_nonce:[4;16],commit_sequence:revision as i64,cleared:false,virtual_obstacles:None,
        fine:Some(Transfer{root_sha256:digest(&root),root,included:BTreeMap::new()}),
        cloud:schema::PointCloud2{header:schema::Header{seq:revision as i32,stamp_sec:(stamp/1_000_000_000) as i32,stamp_nsec:(stamp%1_000_000_000) as i32,frame_id:"map".into()},height:1,width:0,fields:["x","y","z"].iter().enumerate().map(|(i,name)|schema::PointField{name:(*name).into(),offset:(i*4) as i32,datatype:7,count:1}).collect(),is_bigendian:false,point_step:12,row_step:0,data:Vec::new(),is_dense:true},
    }
}
async fn receive(reader:FineMapReader,message:schema::NavigationMap3D)->(FineMapReader,Result<Arc<Manifest>,String>) {
    tokio::task::spawn_blocking(move||{let mut reader=reader;let result=reader.receive(&message);(reader,result)}).await.unwrap()
}

#[tokio::test(flavor="multi_thread",worker_threads=2)]
async fn real_process_rpc_recovers_late_missing_and_corrupt_blocks_without_partial_install() {
    let repository=Repository::temporary().unwrap();let root=repository.path();
    let socket=std::net::UdpSocket::bind("127.0.0.1:0").unwrap();let port=socket.local_addr().unwrap().port();drop(socket);
    let url=format!("udpm://239.255.76.67:{port}?ttl=0");
    std::fs::write(root.join("mode"),"delay").unwrap();
    let mut child=ChildOwner(Command::new(std::env::current_exe().unwrap()).args(["--exact","surface_chunks::rpc_process_tests::map_block_server_process","--ignored","--nocapture"])
        .env("NUWAX_BLOCK_TEST_ROOT",root).env("NUWAX_BLOCK_TEST_URL",&url).stdout(Stdio::null()).stderr(Stdio::inherit()).spawn().unwrap());
    let deadline=Instant::now()+Duration::from_secs(3);
    while !root.join("ready").exists(){assert!(Instant::now()<deadline,"块服务未就绪");assert!(child.0.try_wait().unwrap().is_none());tokio::time::sleep(Duration::from_millis(10)).await;}
    let first=message(&repository,1,1);
    let (reader,result)=receive(FineMapReader::new(url).unwrap(),first.clone()).await;
    let old=result.unwrap();assert_eq!(old.revision,1);
    let calls=std::fs::read_to_string(root.join("calls")).unwrap().parse::<usize>().unwrap();assert!(calls>=2,"必须实际出现迟到重试");
    let current=reader.receiver.current().unwrap();
    std::fs::write(root.join("mode"),"corrupt").unwrap();
    let (reader,result)=receive(reader,message(&repository,2,2)).await;
    assert!(result.unwrap_err().contains("摘要"));assert_eq!(reader.receiver.current().unwrap(),current);
    std::fs::write(root.join("mode"),"missing").unwrap();
    let (reader,result)=receive(reader,message(&repository,3,3)).await;
    assert!(result.is_err());assert_eq!(reader.receiver.current().unwrap().revision,1);
    std::fs::write(root.join("mode"),"good").unwrap();
    let (mut reader,result)=receive(reader,message(&repository,4,4)).await;assert_eq!(result.unwrap().revision,4);
    // 慢读者持有的旧根仍可读，且读取的是消费者自己已验证的对象。
    let (old_columns,count)=reader.columns(&old,None,0.1).unwrap();assert!(!old_columns.is_empty());assert_eq!(count,36000);
    let (reader,result)=receive(reader,first).await;assert!(result.is_err());assert_eq!(reader.receiver.current().unwrap().revision,4);
    drop(reader);
    std::fs::write(root.join("stop"),"stop").unwrap();
    let deadline=Instant::now()+Duration::from_secs(3);
    loop {if let Some(status)=child.0.try_wait().unwrap(){assert!(status.success());break;}assert!(Instant::now()<deadline,"补取服务未结束");tokio::time::sleep(Duration::from_millis(10)).await;}
}
