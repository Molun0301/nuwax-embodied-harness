use memmap2::MmapMut;
use std::fs::OpenOptions;
use std::io::Read;
use std::os::unix::net::UnixListener;
use std::path::{Path, PathBuf};
use std::sync::atomic::{fence, AtomicU64, Ordering};
use std::sync::Arc;
use tokio::sync::Notify;
use tracing::{info, error};

const SHM_MAGIC: u64 = 0x4E55_5741_5856_4944; // "NUWAXVID"
const SHM_VERSION: u32 = 1;
const HEADER_SIZE: usize = 64;
const MAX_FRAME_SIZE: usize = 512 * 1024; // 512KB max per JPEG frame
const BUFFER_SIZE: usize = HEADER_SIZE + MAX_FRAME_SIZE * 2;

#[repr(C)]
struct ShmHeader {
    magic: u64,
    version: u32,
    _pad0: u32,
    sequence: u64,
    frame_size: u32,
    width: u16,
    height: u16,
    timestamp_ns: u64,
    active_buffer: u32, // 0 or 1
    _pad1: [u8; 20],
}

pub struct VideoRelayHandle {
    pub shm_path: PathBuf,
    pub uds_path: PathBuf,
    pub sequence: Arc<AtomicU64>,
    pub notify: Arc<Notify>,
}

pub fn spawn_video_relay(
    shm_path: &Path,
    uds_path: &Path,
) -> std::io::Result<VideoRelayHandle> {
    let sequence = Arc::new(AtomicU64::new(0));
    let notify = Arc::new(Notify::new());

    let shm_path_owned = shm_path.to_path_buf();
    let uds_path_owned = uds_path.to_path_buf();
    let seq_clone = sequence.clone();
    let notify_clone = notify.clone();

    // PLACEHOLDER_SPAWN_THREAD
    std::thread::spawn(move || {
        if let Err(e) = video_relay_loop(
            &shm_path_owned, &uds_path_owned, seq_clone, notify_clone,
        ) {
            error!("视频中转线程退出: {}", e);
        }
    });

    Ok(VideoRelayHandle {
        shm_path: shm_path.to_path_buf(),
        uds_path: uds_path.to_path_buf(),
        sequence,
        notify,
    })
}

fn init_shm(path: &Path) -> std::io::Result<MmapMut> {
    let file = OpenOptions::new()
        .read(true).write(true).create(true).truncate(true)
        .open(path)?;
    file.set_len(BUFFER_SIZE as u64)?;
    let mut mmap = unsafe { MmapMut::map_mut(&file)? };
    // Write header
    let header_bytes = &mut mmap[..HEADER_SIZE];
    header_bytes[0..8].copy_from_slice(&SHM_MAGIC.to_le_bytes());
    header_bytes[8..12].copy_from_slice(&SHM_VERSION.to_le_bytes());
    // Rest is zero-initialized by truncate
    Ok(mmap)
}

fn video_relay_loop(
    shm_path: &Path,
    uds_path: &Path,
    sequence: Arc<AtomicU64>,
    notify: Arc<Notify>,
) -> std::io::Result<()> {
    let mut mmap = init_shm(shm_path)?;
    info!("视频共享内存已创建: {:?} ({} bytes)", shm_path, BUFFER_SIZE);

    // Remove stale socket
    let _ = std::fs::remove_file(uds_path);
    let listener = UnixListener::bind(uds_path)?;
    info!("视频 UDS 监听: {:?}", uds_path);

    for stream in listener.incoming() {
        match stream {
            Ok(mut conn) => {
                info!("视频推流器已连接");
                if let Err(e) = handle_video_connection(
                    &mut conn, &mut mmap, &sequence, &notify,
                ) {
                    info!("视频连接断开: {}", e);
                }
            }
            Err(e) => {
                error!("UDS accept 失败: {}", e);
            }
        }
    }
    Ok(())
}

fn handle_video_connection(
    conn: &mut std::os::unix::net::UnixStream,
    mmap: &mut MmapMut,
    sequence: &Arc<AtomicU64>,
    notify: &Arc<Notify>,
) -> std::io::Result<()> {
    let mut len_buf = [0u8; 4];
    loop {
        // Protocol: [4-byte LE length][JPEG data]
        conn.read_exact(&mut len_buf)?;
        let frame_len = u32::from_le_bytes(len_buf) as usize;
        if frame_len == 0 || frame_len > MAX_FRAME_SIZE {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("帧大小无效: {}", frame_len),
            ));
        }

        let seq = sequence.fetch_add(1, Ordering::Relaxed) + 1;
        let active_buf = (seq % 2) as u32;
        let buf_offset = HEADER_SIZE + (active_buf as usize) * MAX_FRAME_SIZE;

        // Read frame data directly into shared memory
        conn.read_exact(&mut mmap[buf_offset..buf_offset + frame_len])?;

        // Update header (offsets match ShmHeader struct layout)
        // 0-7: magic, 8-11: version, 12-15: pad, 16-23: sequence
        // 24-27: frame_size, 28-29: width, 30-31: height
        // 32-39: timestamp_ns, 40-43: active_buffer
        let now_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let header = &mut mmap[..HEADER_SIZE];
        header[24..28].copy_from_slice(&(frame_len as u32).to_le_bytes());
        header[32..40].copy_from_slice(&now_ns.to_le_bytes());
        header[40..44].copy_from_slice(&active_buf.to_le_bytes());
        // Release fence：确保帧数据与上面的 header 字段对读侧可见后，再发布 sequence。
        // 读侧用 sequence 做 seqlock 双读校验，依赖此处的发布顺序语义。
        fence(Ordering::Release);
        // Write sequence last (acts as release fence for readers)
        header[16..24].copy_from_slice(&seq.to_le_bytes());
        mmap.flush_range(0, HEADER_SIZE + frame_len)?;

        notify.notify_waiters();
    }
}