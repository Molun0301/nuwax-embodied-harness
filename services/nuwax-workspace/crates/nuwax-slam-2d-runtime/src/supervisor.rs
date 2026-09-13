//! 子进程句柄由单一拥有方回收；LCM超时保留未决命令，禁止重发成新操作。
use crate::protocol::*;
use nix::{
    sys::signal::{killpg, Signal},
    unistd::Pid,
};
use nuwax_lcm::LcmContext;
use nuwax_rpc::{LcmRpcTransport, RpcClient};
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tokio::{
    process::{Child, Command},
    sync::Mutex,
};

struct Commands {
    sequence: u64,
    pending: Option<Request>,
}
#[derive(Clone)]
pub struct Client {
    rpc: RpcClient,
    context: Arc<LcmContext>,
    identity: Identity,
    commands: Arc<Mutex<Commands>>,
    closed: Arc<AtomicBool>,
    directory: PathBuf,
}
impl Client {
    fn new(
        context: Arc<LcmContext>,
        identity: Identity,
        directory: PathBuf,
    ) -> Result<Self, String> {
        identity.validate()?;
        let rpc = RpcClient::new(
            Arc::new(LcmRpcTransport::from_context(context.clone())),
            identity.rpc_name(),
        )
        .map_err(|e| e.to_string())?;
        Ok(Self {
            rpc,
            context,
            identity,
            commands: Arc::new(Mutex::new(Commands {
                sequence: 0,
                pending: None,
            })),
            directory,
            closed: Arc::new(AtomicBool::new(false)),
        })
    }
    async fn request(&self, request: &Request, timeout: Duration) -> Result<Response, String> {
        let value = self
            .rpc
            .call(
                (
                    vec![serde_json::to_value(request).map_err(|e| e.to_string())?],
                    HashMap::new(),
                ),
                timeout,
            )
            .await
            .map_err(|e| e.to_string())?;
        let response: Response = serde_json::from_value(value).map_err(|e| e.to_string())?;
        if response.identity != self.identity
            || response.version != VERSION
            || response.command_sequence != request.command_sequence
        {
            return Err("二维回执身份不匹配".into());
        }
        Ok(response)
    }
    pub fn identity(&self) -> &Identity {
        &self.identity
    }
    pub async fn status(&self) -> Result<Status, String> {
        let request = Request {
            version: VERSION,
            identity: self.identity.clone(),
            command_sequence: 0,
            operation: Operation::Status,
        };
        let response = self.request(&request, Duration::from_millis(500)).await?;
        match response.error {
            Some(error) => Err(error),
            None => Ok(response.status),
        }
    }
    pub async fn execute(&self, operation: Operation) -> Result<Response, String> {
        if self.closed.load(Ordering::Acquire)
            && !matches!(operation, Operation::Shutdown | Operation::Status)
        {
            return Err("二维拥有方已关闭新操作接纳".into());
        }
        if !operation.mutates() {
            return self
                .request(
                    &Request {
                        version: VERSION,
                        identity: self.identity.clone(),
                        command_sequence: 0,
                        operation,
                    },
                    Duration::from_secs(5),
                )
                .await;
        }
        let mut commands = self.commands.lock().await;
        if let Some(pending) = commands.pending.clone() {
            let result = self.request(&pending, Duration::from_secs(5)).await?;
            commands.pending = None;
            if pending.operation == operation {
                return Ok(result);
            }
            if let Some(error) = result.error {
                return Err(format!("前序二维控制失败：{error}"));
            }
        }
        commands.sequence = commands.sequence.checked_add(1).ok_or("二维控制序号耗尽")?;
        let request = Request {
            version: VERSION,
            identity: self.identity.clone(),
            command_sequence: commands.sequence,
            operation,
        };
        commands.pending = Some(request.clone());
        let response = self.request(&request, Duration::from_secs(5)).await?;
        commands.pending = None;
        Ok(response)
    }
    pub async fn publish_scan(
        &self,
        session_generation: u64,
        scan: &nuwax_codec::lcm::planar::PlanarScan,
    ) -> Result<(), String> {
        if self.closed.load(Ordering::Acquire) {
            return Err("二维拥有方已关闭扫描接纳".into());
        }
        if scan.runtime_generation != self.identity.runtime_generation as i64 {
            return Err("二维扫描运行代际不匹配".into());
        }
        let data = nuwax_codec::lcm::planar::encode(scan).map_err(|e| e.to_string())?;
        self.context
            .publish(&self.identity.scan_channel(session_generation), data.into())
            .await
            .map_err(|e| e.to_string())
    }
    pub async fn open_artifact(&self, artifact: &FileArtifact) -> Result<std::fs::File, String> {
        artifact.validate()?;
        if self.closed.load(Ordering::Acquire) {
            return Err("二维拥有方已关闭制品接纳".into());
        }
        let path = self.directory.join(&artifact.file_name);
        let artifact = artifact.clone();
        let closed = self.closed.clone();
        tokio::task::spawn_blocking(move || {
            use std::io::{Seek, SeekFrom};
            let mut file = crate::archive::open_regular(&path)?;
            let (size, sha) = crate::archive::hash_reader(&mut file)?;
            if size != artifact.payload_size
                || sha != artifact.sha256
                || closed.load(Ordering::Acquire)
            {
                return Err("二维制品身份不一致或拥有方已关闭".into());
            }
            file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
            Ok(file)
        })
        .await
        .map_err(|e| e.to_string())?
    }

    pub async fn stage_artifact(
        &self,
        mut source: std::fs::File,
        size: u64,
        sha256: [u8; 32],
    ) -> Result<FileArtifact, String> {
        let artifact = FileArtifact {
            file_name: format!("input-{}.nws2d", uuid::Uuid::new_v4().simple()),
            payload_size: size,
            sha256,
        };
        artifact.validate()?;
        if self.closed.load(Ordering::Acquire) {
            return Err("二维拥有方已关闭制品接纳".into());
        }
        let path = self.directory.join(&artifact.file_name);
        let closed = self.closed.clone();
        tokio::task::spawn_blocking(move || {
            use sha2::{Digest, Sha256};
            use std::io::{Read, Seek, SeekFrom, Write};
            use std::os::unix::fs::OpenOptionsExt;
            let mut output = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .mode(0o600)
                .open(&path)
                .map_err(|e| e.to_string())?;
            let result = (|| {
                source.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
                let mut buffer = vec![0; crate::archive::CHUNK_BYTES];
                let mut hash = Sha256::new();
                let mut total = 0u64;
                loop {
                    let n = source.read(&mut buffer).map_err(|e| e.to_string())?;
                    if n == 0 {
                        break;
                    }
                    total += n as u64;
                    if total > size {
                        return Err("二维输入制品增长".into());
                    }
                    hash.update(&buffer[..n]);
                    output.write_all(&buffer[..n]).map_err(|e| e.to_string())?;
                }
                if total != size
                    || <[u8; 32]>::from(hash.finalize()) != sha256
                    || closed.load(Ordering::Acquire)
                {
                    return Err("二维输入制品变化或拥有方已关闭".into());
                }
                output.sync_all().map_err(|e| e.to_string())?;
                Ok(artifact)
            })();
            if result.is_err() {
                let _ = std::fs::remove_file(path);
            }
            result
        })
        .await
        .map_err(|e| e.to_string())?
    }

    pub async fn read_snapshot(&self, snapshot: &Snapshot) -> Result<Vec<i8>, String> {
        if self.closed.load(Ordering::Acquire) {
            return Err("二维拥有方已关闭候选接纳".into());
        }
        if snapshot.width == 0
            || snapshot.height == 0
            || !snapshot.resolution.is_finite()
            || snapshot.resolution <= 0.0
            || snapshot.origin.iter().any(|v| !v.is_finite())
        {
            return Err("二维快照栅格几何无效".into());
        }
        if snapshot.file_name != format!("candidate-{:016x}.bin", snapshot.snapshot_id)
            || snapshot.payload_size != u64::from(snapshot.width) * u64::from(snapshot.height)
            || snapshot.payload_size > 16 * 1024 * 1024
        {
            return Err("二维快照文件身份或尺寸非法".into());
        }
        let path = self.directory.join(&snapshot.file_name);
        let expected = snapshot.clone();
        let closed = self.closed.clone();
        tokio::task::spawn_blocking(move || {
            use std::io::Read;
            use std::os::unix::fs::OpenOptionsExt;
            let file = std::fs::OpenOptions::new()
                .read(true)
                .custom_flags(nix::libc::O_NOFOLLOW)
                .open(path)
                .map_err(|e| e.to_string())?;
            let metadata = file.metadata().map_err(|e| e.to_string())?;
            if !metadata.is_file() || metadata.len() != expected.payload_size {
                return Err("二维快照文件长度已变化".into());
            }
            let mut data = Vec::new();
            file.take(expected.payload_size + 1)
                .read_to_end(&mut data)
                .map_err(|e| e.to_string())?;
            if data.len() as u64 != expected.payload_size
                || nuwax_codec::hash::sha256_bytes(&data) != expected.data_sha256
            {
                return Err("二维快照内容与回执不一致".into());
            }
            if data.iter().any(|v| !matches!(*v, 0 | 100 | 255)) {
                return Err("二维栅格含未知通行类别".into());
            }
            if closed.load(Ordering::Acquire) {
                return Err("二维候选读取期间拥有方已关闭".into());
            }
            Ok(data.into_iter().map(|v| v as i8).collect())
        })
        .await
        .map_err(|e| e.to_string())?
    }
}

pub struct ProcessOwner {
    child: Option<Child>,
    group: Option<Pid>,
    client: Client,
    directory: PathBuf,
    terminal: Option<Result<(), String>>,
}
impl ProcessOwner {
    pub async fn start(
        binary: &Path,
        context: Arc<LcmContext>,
        runtime_generation: u64,
        data_root: &Path,
    ) -> Result<Self, String> {
        let identity = Identity {
            runtime_generation,
            owner_nonce: uuid::Uuid::new_v4().simple().to_string(),
        };
        identity.validate()?;
        let directory = data_root.join(format!("slam2d-{}", identity.owner_nonce));
        std::fs::create_dir_all(data_root).map_err(|e| e.to_string())?;
        use std::os::unix::fs::DirBuilderExt;
        std::fs::DirBuilder::new()
            .mode(0o700)
            .create(&directory)
            .map_err(|e| e.to_string())?;
        let client = Client::new(context.clone(), identity.clone(), directory.clone())?;
        let mut command = Command::new(binary);
        // 完整发布包的动态库只注入算法子进程，避免核心库加载进Geometry父进程。
        if let Some(path) = std::env::var_os("NUWAX_GEOMETRY_SLAM2D_LIBRARY_PATH") {
            command.env("LD_LIBRARY_PATH", path);
        }
        command
            .arg("--owner")
            .arg(&identity.owner_nonce)
            .arg("--generation")
            .arg(runtime_generation.to_string())
            .arg("--lcm")
            .arg(context.provider())
            .arg("--directory")
            .arg(&directory);
        command.kill_on_drop(true);
        command.process_group(0);
        let child = match command.spawn() {
            Ok(child) => child,
            Err(error) => {
                let _ = std::fs::remove_dir(&directory);
                return Err(error.to_string());
            }
        };
        let group = child.id().map(|pid| Pid::from_raw(pid as i32));
        let mut owner = Self {
            child: Some(child),
            group,
            client,
            directory,
            terminal: None,
        };
        let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(status) = owner
                .child
                .as_mut()
                .unwrap()
                .try_wait()
                .map_err(|e| e.to_string())?
            {
                owner.child = None;
                owner.group = None;
                owner.terminal = Some(Err(format!("二维工作进程启动失败：{status}")));
                return Err(format!("二维工作进程启动失败：{status}"));
            }
            if owner.client.status().await.is_ok() {
                return Ok(owner);
            }
            if tokio::time::Instant::now() >= deadline {
                let _ = owner.terminate().await;
                return Err("二维工作进程控制接口启动超时".into());
            }
        }
    }
    pub fn client(&self) -> Client {
        self.client.clone()
    }
    pub fn process_id(&self) -> Option<u32> {
        self.child.as_ref().and_then(Child::id)
    }
    pub fn check_alive(&mut self) -> Result<(), String> {
        let child = self.child.as_mut().ok_or("二维工作进程已回收")?;
        if let Some(status) = child.try_wait().map_err(|e| e.to_string())? {
            self.child = None;
            self.group = None;
            let error = format!("二维工作进程已退出：{status}");
            self.terminal = Some(Err(error.clone()));
            self.client.closed.store(true, Ordering::Release);
            return Err(error);
        }
        Ok(())
    }
    async fn terminate(&mut self) -> Result<(), String> {
        if let Some(group) = self.group {
            let _ = killpg(group, Signal::SIGKILL);
        }
        if let Some(child) = self.child.as_mut() {
            child.wait().await.map_err(|e| e.to_string())?;
        }
        self.child = None;
        self.group = None;
        Ok(())
    }
    pub async fn shutdown(&mut self) -> Result<(), String> {
        self.client.closed.store(true, Ordering::Release);
        if self.child.is_none() {
            return self.terminal.clone().unwrap_or(Ok(()));
        }
        let result = self.shutdown_inner().await;
        if self.child.is_none() {
            self.terminal = Some(result.clone());
        }
        result
    }
    async fn shutdown_inner(&mut self) -> Result<(), String> {
        let reply = self.client.execute(Operation::Shutdown).await;
        let error = match reply {
            Ok(response) => response.error,
            Err(error) => Some(error),
        };
        if let Some(group) = self.group {
            let _ = killpg(group, Signal::SIGTERM);
        }
        if let Some(child) = self.child.as_mut() {
            match tokio::time::timeout(Duration::from_secs(5), child.wait()).await {
                Ok(Ok(status)) => {
                    self.child = None;
                    self.group = None;
                    if !status.success() {
                        return Err(format!("二维工作进程关闭失败：{status}"));
                    }
                }
                _ => {
                    self.terminate().await?;
                    return Err("二维工作进程未按期退出，已终止并回收进程组".into());
                }
            }
        }
        std::fs::remove_dir_all(&self.directory).map_err(|e| e.to_string())?;
        if let Some(error) = error {
            return Err(format!("二维控制收尾未确认，进程已回收：{error}"));
        }
        Ok(())
    }
}
impl Drop for ProcessOwner {
    fn drop(&mut self) {
        self.client.closed.store(true, Ordering::Release);
        // 异常丢弃仍终止整个进程组；正常拥有方须调用shutdown取得真实回收结果。
        if let Some(group) = self.group {
            let _ = killpg(group, Signal::SIGKILL);
        }
        if self.child.is_none() {
            let _ = std::fs::remove_dir_all(&self.directory);
        }
    }
}
