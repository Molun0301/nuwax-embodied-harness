// 开发取证入口：只读消息和原可靠块接口，不启动规划器、运动拥有方或独立平台。
mod input_capture {
    use super::*;
    use futures::StreamExt;
    use std::io::Write;
    use std::path::Path;

    fn archive_objects(
        root: &Manifest,
        source: &Repository,
        archive: &Repository,
        seen: &mut BTreeSet<ObjectIdentity>,
        bytes: &std::sync::atomic::AtomicU64,
    ) -> Result<usize, String> {
        let mut copied = 0;
        for reference in &root.blocks {
            if seen.contains(&reference.object) {
                continue;
            }
            let data = source.read_object(reference.object, MAX_BLOCK_BYTES)?;
            claim(bytes, data.len())?;
            archive.put_object(reference.object, &data)?;
            seen.insert(reference.object);
            copied += 1;
        }
        Ok(copied)
    }

    fn map_identity(value: &schema::NavigationMap3D) -> serde_json::Value {
        serde_json::json!({"session_id":value.session_id, "revision":value.revision,
            "frame_id":value.frame_id, "source_timestamp_ns":value.source_timestamp_ns})
    }
    fn claim(total: &std::sync::atomic::AtomicU64, amount: usize) -> Result<(), String> {
        total
            .fetch_update(
                std::sync::atomic::Ordering::Relaxed,
                std::sync::atomic::Ordering::Relaxed,
                |previous| {
                    previous
                        .checked_add(amount as u64)
                        .filter(|next| *next <= 512 * 1024 * 1024)
                },
            )
            .map(|_| ())
            .map_err(|_| "取证总预算512MiB已用尽".into())
    }

    fn wall_ns() -> u128 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    }
    fn save_bytes(
        directory: &Path,
        name: &str,
        data: &[u8],
        total: &std::sync::atomic::AtomicU64,
    ) -> Result<(), String> {
        claim(total, data.len())?;
        std::fs::write(directory.join(name), data).map_err(|error| error.to_string())
    }

    #[test]
    fn captured_root_objects_keep_original_bytes_and_can_be_received_again() {
        let source = Repository::temporary().unwrap();
        let archive = Repository::temporary().unwrap();
        let root = install(
            &source,
            &[[0, 0, 0], [20, 0, 1], [2, 8, 3]].into_iter().collect(),
            0.05,
            1,
        );
        let bytes = std::sync::atomic::AtomicU64::new(0);
        let mut seen = BTreeSet::new();
        assert_eq!(
            archive_objects(&root, &source, &archive, &mut seen, &bytes).unwrap(),
            root.blocks.len()
        );
        let initial = bytes.load(std::sync::atomic::Ordering::Relaxed);
        assert_eq!(
            archive_objects(&root, &source, &archive, &mut seen, &bytes).unwrap(),
            0
        );
        assert_eq!(initial, bytes.load(std::sync::atomic::Ordering::Relaxed));
        for reference in &root.blocks {
            assert_eq!(
                source
                    .read_object(reference.object, MAX_BLOCK_BYTES)
                    .unwrap(),
                archive
                    .read_object(reference.object, MAX_BLOCK_BYTES)
                    .unwrap()
            );
        }
        let mut receiver = Receiver::new(Repository::temporary().unwrap());
        let transfer = Transfer {
            root: root.encode().unwrap(),
            included: Default::default(),
        };
        let loaded = receiver
            .receive(&transfer, &root.domain, |_, reference| {
                archive.read_object(reference.object, MAX_BLOCK_BYTES)
            })
            .unwrap();
        assert_eq!(*loaded, root);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    #[ignore = "显式提供同一受管会话的LCM地址、主题、地图会话号，只读取证"]
    async fn capture_live_surface_messages_and_frozen_objects() {
        use nuwax_protocol::{LcmMessageType, LcmTopic};
        use nuwax_transport::LcmTransport;
        let required = |name: &str| {
            std::env::var(name).unwrap_or_else(|_| panic!("必须明确提供{name}，禁止猜测另一实例"))
        };
        let url = required("NUWAX_LCM_URL");
        let map_topic = required("NUWAX_SURFACE_CAPTURE_MAP_TOPIC");
        let pose_topic = required("NUWAX_SURFACE_CAPTURE_POSE_TOPIC");
        let directory = std::path::PathBuf::from(required("NUWAX_SURFACE_CAPTURE_DIRECTORY"));
        let session_id = required("NUWAX_SURFACE_CAPTURE_SESSION");
        let seconds: u64 = std::env::var("NUWAX_SURFACE_CAPTURE_SECONDS")
            .unwrap_or_else(|_| "120".into())
            .parse()
            .unwrap();
        assert!(seconds > 0 && seconds <= 600);
        std::fs::create_dir(&directory).expect("输出必须是尚不存在的新目录，父目录须预先存在");
        let transport = LcmTransport::new_real(&url).unwrap();
        let maps = transport
            .subscribe(
                LcmTopic::new(&map_topic).unwrap(),
                Some(LcmMessageType::new("nuwax_msgs.NavigationMap3D").unwrap()),
            )
            .await;
        let poses = transport
            .subscribe(
                LcmTopic::new(&pose_topic).unwrap(),
                Some(LcmMessageType::new("geometry_msgs.PoseStamped").unwrap()),
            )
            .await;
        let mut inputs = futures::stream::select(
            maps.map(|frame| ("map", frame)),
            poses.map(|frame| ("pose", frame)),
        );
        let total = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let (sender, mut queue) =
            tokio::sync::mpsc::channel::<(usize, Arc<schema::NavigationMap3D>)>(8);
        let fetch_directory = directory.clone();
        let fetch_total = total.clone();
        let fetch_url = url.clone();
        let worker = tokio::task::spawn_blocking(move || {
            let local = Repository::temporary().unwrap();
            let archive = Repository::open(fetch_directory.join("objects")).unwrap();
            let mut reader = FineMapReader {
                receiver: Receiver::with_cache_budget(local.clone(), 32 * 1024 * 1024),
                client: None,
                compress_parts: true,
                lcm_url: fetch_url,
            };
            let mut seen = BTreeSet::new();
            let mut complete_active = 0;
            let mut log = std::io::BufWriter::new(
                std::fs::File::create(fetch_directory.join("根对象核对.jsonl")).unwrap(),
            );
            while let Some((ordinal, message)) = queue.blocking_recv() {
                let started = Instant::now();
                let result = if message.cleared {
                    reader.receiver.clear();
                    Ok(serde_json::json!({"cleared":true,"blocks":0}))
                } else if message.fine.is_none() {
                    Ok(serde_json::json!({"inline_cloud_only":true,"blocks":0}))
                } else {
                    reader.receive(&message).map_err(|error|error.to_string()).and_then(|root| {
                    let copied=archive_objects(&root,&local,&archive,&mut seen,&fetch_total)?;
                    save_bytes(&fetch_directory,&format!("根_{ordinal:06}.bin"),&message.fine.as_ref().unwrap().root,&fetch_total)?;
                    Ok(serde_json::json!({"root":root.identity(),"blocks":root.blocks.len(),"copied":copied}))
                })
                };
                if result.is_ok() && !message.cleared {
                    complete_active += 1;
                }
                let row = serde_json::json!({"ordinal":ordinal,"revision":message.revision,
                    "committed_map":map_identity(&message),"elapsed_ms":started.elapsed().as_secs_f64()*1000.0,
                    "complete":result.is_ok(),"result":result.as_ref().ok(),"error":result.err()});
                writeln!(log, "{row}").unwrap();
                log.flush().unwrap();
            }
            complete_active
        });
        let mut order = std::io::BufWriter::new(
            std::fs::File::create(directory.join("原始接收顺序.jsonl")).unwrap(),
        );
        let deadline = tokio::time::Instant::now() + Duration::from_secs(seconds);
        let mut count = 0;
        let mut failure = None;
        let mut bound = false;
        let mut skipped_unbound_poses = 0;
        std::fs::write(directory.join("只读采集已就绪.json"),serde_json::to_vec(&serde_json::json!({
            "wall_ns":wall_ns(),"url":url,"session_id":session_id,"map_topic":map_topic,"pose_topic":pose_topic,
            "scope":"只读消息与原可靠块接口；读到流时刻不冒充物理采集或发布时刻"})).unwrap()).unwrap();
        while let Ok(Some((kind, frame))) = tokio::time::timeout_at(deadline, inputs.next()).await {
            let received = wall_ns();
            let frame = match frame {
                Ok(frame) => frame,
                Err(error) => {
                    failure = Some(error.to_string());
                    break;
                }
            };
            let message = if kind == "map" {
                let message = match schema::decode_navigation_map_3d_body(&frame.payload) {
                    Ok(value) => Arc::new(value),
                    Err(error) => {
                        failure = Some(error.to_string());
                        break;
                    }
                };
                if message.session_id != session_id {
                    failure = Some("出现不属于指定受管运行的表面生产者，停止取证".into());
                    break;
                }
                bound = true;
                Some(message)
            } else {
                None
            };
            if !bound {
                skipped_unbound_poses += 1;
                continue;
            }
            if let Err(error) = save_bytes(
                &directory,
                &format!("{kind}_{count:06}.body"),
                &frame.payload,
                &total,
            ) {
                failure = Some(error);
                break;
            }
            let mut row = serde_json::json!({"ordinal":count,"kind":kind,"capture_read_wall_ns":received,"bytes":frame.payload.len()});
            if let Some(message) = message {
                row["revision"] = serde_json::json!(message.revision);
                row["committed_map"] = map_identity(&message);
                row["cleared"] = serde_json::json!(message.cleared);
                row["objects_queued"] =
                    serde_json::json!(sender.try_send((count, message)).is_ok());
            }
            writeln!(order, "{row}").unwrap();
            order.flush().unwrap();
            count += 1;
        }
        drop(sender);
        let complete_active = worker.await.unwrap();
        std::fs::write(directory.join("采集结束.json"),serde_json::to_vec(&serde_json::json!({
            "wall_ns":wall_ns(),"records":count,"budget_claimed_bytes":total.load(std::sync::atomic::Ordering::Relaxed),
            "skipped_unbound_poses":skipped_unbound_poses,"complete_active_roots":complete_active,"error":failure})).unwrap()).unwrap();
        assert!(failure.is_none(), "取证未完整结束：{failure:?}");
        assert!(
            bound && complete_active > 0,
            "未取得属于指定运行的完整活跃表面输入"
        );
    }
}
