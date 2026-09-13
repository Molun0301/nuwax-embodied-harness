use nuwax_lcm::LcmContext;
use nuwax_rpc::{LcmRpcTransport, RpcBroker, RpcServerConfig};
use nuwax_slam_2d_runtime::{
    protocol::{Identity, WORKER_CONTRACT},
    worker::WorkerOwner,
};
use std::{path::PathBuf, sync::Arc};
#[tokio::main(flavor = "current_thread")]
async fn main() {
    if let Err(error) = run().await {
        eprintln!("二维工作进程失败：{error}");
        std::process::exit(1);
    }
}
async fn run() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args == ["--contract"] {
        println!("{WORKER_CONTRACT}");
        return Ok(());
    }
    if args.len() != 8 {
        return Err("需要--owner、--generation、--lcm与--directory".into());
    }
    let field = |name: &str| {
        args.chunks_exact(2)
            .find(|pair| pair[0] == name)
            .map(|pair| pair[1].clone())
            .ok_or_else(|| format!("缺少{name}"))
    };
    let identity = Identity {
        runtime_generation: field("--generation")?
            .parse::<u64>()
            .map_err(|e| e.to_string())?,
        owner_nonce: field("--owner")?,
    };
    identity.validate()?;
    let directory = PathBuf::from(field("--directory")?);
    if !directory.is_dir() {
        return Err("二维拥有方数据目录不存在".into());
    }
    let context = Arc::new(LcmContext::new(&field("--lcm")?).map_err(|e| e.to_string())?);
    let mut owner = WorkerOwner::start(identity.clone(), directory)?;
    let mut algorithm_exit = owner.exit_receiver();
    let broker = RpcBroker::new();
    broker
        .register(identity.rpc_name(), Box::new(owner.handle()))
        .await
        .map_err(|e| e.to_string())?;
    let mut scan = context
        .subscribe(&format!(
            "^{}[0-9a-f]{{16}}#nuwax_msgs\\.PlanarScan$",
            identity.scan_prefix()
        ))
        .await
        .map_err(|e| e.to_string())?;
    let server = broker
        .serve_with_config(
            Arc::new(LcmRpcTransport::from_context(context.clone())),
            &identity.rpc_name(),
            RpcServerConfig {
                max_concurrent_requests: 8,
            },
        )
        .await
        .map_err(|e| e.to_string())?;
    let mut term = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
        .map_err(|e| e.to_string())?;
    let mut interrupt = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::interrupt())
        .map_err(|e| e.to_string())?;
    loop {
        tokio::select! {
            _=term.recv()=>break,
            _=algorithm_exit.changed()=>return Err("二维算法线程已结束".into()),
            _=interrupt.recv()=>break,
            message=scan.receiver.recv()=>match message {
                Ok(message)=>match nuwax_codec::lcm::planar::decode(&message.payload){Ok(data)=>owner.handle().accept_scan(&message.channel,data),Err(error)=>eprintln!("二维扫描解码拒绝：{error}")},
                Err(tokio::sync::broadcast::error::RecvError::Lagged(_))=>{},
                Err(error)=>return Err(error.to_string()),
            }
        }
    }
    scan.unsubscribe().await.map_err(|e| e.to_string())?;
    server.stop().await.map_err(|e| e.to_string())?;
    owner.close_and_join()?;
    context.shutdown().await.map_err(|e| e.to_string())?;
    Ok(())
}
