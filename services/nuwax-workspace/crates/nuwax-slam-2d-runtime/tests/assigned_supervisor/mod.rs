use super::*;
use crate::archive;
use nuwax_rpc::{RpcBroker, RpcHandler, RpcRequest, RpcResponse};
use std::sync::atomic::{AtomicUsize, Ordering as CountOrdering};

struct Directory(PathBuf);
impl Directory {
    fn new() -> Self {
        let p=std::env::temp_dir().join(format!("assigned-supervisor-{}",uuid::Uuid::new_v4().simple()));
        std::fs::create_dir(&p).unwrap();Self(p)
    }
}
impl Drop for Directory { fn drop(&mut self) { let _=std::fs::remove_dir_all(&self.0); } }
fn identity()->Identity { Identity {runtime_generation:1,owner_nonce:uuid::Uuid::new_v4().simple().to_string()} }
fn archive_file(dir:&Directory)->(PathBuf,archive::AssignedArtifact) {
    let paths=["graph.posegraph","graph.data","grid.i8"].map(|s|dir.0.join(s));
    for (p,b) in paths.iter().zip([b"abc".as_slice(),b"def",&[0,100,255]]) {std::fs::write(p,b).unwrap();}
    let metadata=archive::AssignedMetadata {
        checkpoint_id:"1".repeat(32),
        source:archive::Metadata {
            version:2,
            session:serde_json::from_str(r#"{"session_id":"s","map_name":"map","session_generation":1,"sensor_id":"laser","frame_id":"laser","map_frame_id":"map","odom_frame_id":"odom","body_frame_id":"base","clock_domain":"unix_ns","clock_id":"","calibration_ref":"c","odometry_source":"device","core_config":{"base_from_laser":[0,0,0]}}"#).unwrap(),
            estimate:Estimate {graph_revision:1,..Default::default()},source_unix_timestamp_ns:1,
            width:3,height:1,resolution:0.05,origin:[0.0,0.0],
            members:["graph.posegraph","graph.data","grid.i8"].map(|s|archive::AssignedMember {name:s.into(),size:3}),
        }
    };
    let path=dir.0.join("source.nws2d");
    let descriptor=archive::write_assigned(&path,&metadata,paths.each_ref().map(|p|p.as_path())).unwrap();(path,descriptor)
}
#[tokio::test]
async fn stage_and_open_keep_checkpoint_across_owner_locators() {
    let source=Directory::new();let target=Directory::new();let (path,descriptor)=archive_file(&source);
    let context=Arc::new(LcmContext::new("memq://").unwrap());let client=Client::new(context,identity(),target.0.clone()).unwrap();
    let artifact=client.stage_artifact(archive::open_regular(&path).unwrap(),descriptor.payload_size,
        ArtifactIdentity::Assigned {checkpoint_id:descriptor.checkpoint_id.clone()}).await.unwrap();
    assert_ne!(artifact.file_name,path.file_name().unwrap().to_str().unwrap());
    assert_eq!(artifact.identity,ArtifactIdentity::Assigned {checkpoint_id:descriptor.checkpoint_id});
    let mut opened=client.open_artifact(&artifact).await.unwrap();
    assert_eq!(archive::grid_structure(&mut opened).unwrap().1,vec![0,100,-1]);
    let mut changed=artifact.clone();changed.identity=ArtifactIdentity::Assigned {checkpoint_id:"2".repeat(32)};
    assert!(client.open_artifact(&changed).await.is_err());
    let count=std::fs::read_dir(&target.0).unwrap().count();
    assert!(client.stage_artifact(archive::open_regular(&path).unwrap(),artifact.payload_size,ArtifactIdentity::Legacy).await.is_err());
    assert_eq!(std::fs::read_dir(&target.0).unwrap().count(),count);
    client.closed.store(true,Ordering::Release);
    assert!(client.open_artifact(&artifact).await.is_err());
}
struct RetainedResponse { request:Request, response:Response, calls:Arc<AtomicUsize> }
#[async_trait::async_trait]
impl RpcHandler for RetainedResponse {
    async fn handle(&self,rpc:RpcRequest)->RpcResponse {
        let request:Request=serde_json::from_value(rpc.args.0[0].clone()).unwrap();
        assert_eq!(request,self.request);self.calls.fetch_add(1,CountOrdering::SeqCst);
        RpcResponse::ok(rpc.id,serde_json::to_value(&self.response).unwrap())
    }
}
#[tokio::test]
async fn pending_client_retries_original_sequence_and_retained_descriptor() {
    // Real Client::execute and memq RPC, with a retained-response fixture (not a native worker cache test).
    let dir=Directory::new();let context=Arc::new(LcmContext::new("memq://").unwrap());let id=identity();
    let client=Client::new(context.clone(),id.clone(),dir.0.clone()).unwrap();
    let operation=Operation::ExportGraph {session_id:"s".into(),session_generation:1,unix_offset_ns:0};
    let request=Request {version:VERSION,identity:id.clone(),command_sequence:42,operation:operation.clone()};
    let artifact=FileArtifact {file_name:"graph-frozen.nws2d".into(),payload_size:1024,
        identity:ArtifactIdentity::Assigned {checkpoint_id:"1".repeat(32)}};
    let response=Response {version:VERSION,identity:id.clone(),command_sequence:42,status:Status::default(),snapshot:None,artifact:Some(artifact.clone()),error:None};
    let calls=Arc::new(AtomicUsize::new(0));let broker=RpcBroker::new();
    broker.register(id.rpc_name(),Box::new(RetainedResponse {request:request.clone(),response,calls:calls.clone()})).await.unwrap();
    let server=broker.serve(Arc::new(LcmRpcTransport::from_context(context)),&id.rpc_name()).await.unwrap();
    {let mut commands=client.commands.lock().await;commands.sequence=42;commands.pending=Some(request);}
    let result=client.execute(operation).await.unwrap();
    assert_eq!(result.artifact,Some(artifact));assert_eq!(calls.load(CountOrdering::SeqCst),1);
    let commands=client.commands.lock().await;assert_eq!(commands.sequence,42);assert!(commands.pending.is_none());drop(commands);
    server.stop().await.unwrap();
}
