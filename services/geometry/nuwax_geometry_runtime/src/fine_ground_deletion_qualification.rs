//! 真实原生帧的动态删除与地面成员对账；仅测试，直接复用生产拥有方。
use super::*;
use nuwax_map_chunks::{block_key, global_key, local_key, Domain, Key, LayerObservation, LayerSample, LayerSource};
use std::collections::BTreeMap;
use std::io::Write;
use std::path::Path;

fn target_state(fine: &mut FineCloud, target: Key) -> serde_json::Value {
    let block = block_key(target);
    let local = local_key(target);
    let occupancy = fine.writer.region(target, target, 1).unwrap();
    let layers = fine.writer.layer_observations(block).unwrap();
    let column: Vec<_> = layers.iter()
        .filter(|(_, layer)| layer.members[0] % 1024 == local % 1024)
        .map(|(anchor, layer)| serde_json::json!({
            "anchor": anchor,
            "members_global": layer.members.iter().map(|member| global_key(block, *member).unwrap()).collect::<Vec<_>>(),
            "record": layer,
        })).collect();
    serde_json::json!({
        "occupied": !occupancy.is_empty(),
        "occupancy": occupancy.iter().map(|(key, evidence)| serde_json::json!({
            "key": key, "observed_ns": evidence.observed_ns, "hits": evidence.hits,
        })).collect::<Vec<_>>(),
        "claimed": layers.values().any(|layer| layer.members.contains(&local)),
        "column_layers": column,
    })
}

fn save_archive(fine: &FineCloud, sealed: &Arc<SealedMap>, path: &Path) {
    let (mut source, _) = nuwax_map_chunks::archive_file(sealed.manifest(), fine.writer.repository()).unwrap();
    let mut destination = std::fs::File::create(path).unwrap();
    std::io::copy(&mut source, &mut destination).unwrap();
}

#[test]
fn removing_one_ground_member_keeps_other_observed_members() {
    let mut fine = FineCloud::new(0.05).unwrap();
    let low = [0, 0, 0];
    let high = [0, 0, 1];
    fine.writer.observe([low, high], 10).unwrap();
    let source = Arc::new(LayerSource::Lidar {
        backend: 2, runtime_generation: 1, sensor_frame_id: "body".into(),
    });
    let layer = LayerObservation {
        members: vec![local_key(low), local_key(high)],
        plane_origin: [0.025, 0.025, 0.05], normal: [0.0, 0.0, 1.0],
        samples: vec![
            LayerSample { point: [0.025, 0.025, 0.049], source: source.clone(), source_sequence: 1, source_timestamp_ns: 10, point_index: 0 },
            LayerSample { point: [0.025, 0.025, 0.051], source, source_sequence: 1, source_timestamp_ns: 10, point_index: 1 },
        ],
    };
    fine.writer.replace_layer_observations([0, 0, 0], BTreeMap::from([(local_key(low), layer)])).unwrap();
    fine.writer.remove([low]).unwrap();
    assert_eq!(fine.writer.region(high, high, 1).unwrap().len(), 1, "没有删除证据的相邻占据仍在");
    let after = fine.writer.layer_observations([0, 0, 0]).unwrap();
    let remaining = after.values().find(|layer| layer.members.contains(&local_key(high)))
        .expect("删除一个成员不能顺带撤销同层未被删除的地面成员");
    assert_eq!(remaining.members, vec![local_key(high)], "被明确删除的格也不能保留旧地面声明");
    assert!(remaining.samples.iter().all(|sample| sample.point_index == 1), "删除格的旧样本不能继续进入剩余统计");
    fine.writer.remove([high]).unwrap();
    assert!(fine.writer.layer_observations([0, 0, 0]).unwrap().is_empty(), "最后成员删除后层必须退出");
}

#[derive(serde::Deserialize)]
struct RecordedFrame {
    sequence: u64,
    timestamp_ns: u64,
    correlated_odometry_source_sequence: u64,
    correlated_odometry_source_timestamp_ns: u64,
    source: LayerSource,
    domain: Domain,
    origin_map: Option<[f64; 3]>,
    origin_status: String,
    points: Vec<[f64; 3]>,
    indices: Vec<u32>,
    published_ground_indices: Vec<u32>,
    native_file: String,
    native_sha256: String,
}

#[test]
#[ignore = "需实际原生点、同源规范位姿、独立物理核实应保留的目标成员及独立输出目录"]
fn replay_native_fine_ground_deletions_with_production_origins() {
    replay_native_ground(ReplayCheck::RetainedMembers);
}

#[test]
#[ignore = "需完整真实前缀、同源规范位姿及独立核实的指定当前帧地面原索引"]
fn replay_native_ground_classification_with_production_origins() {
    let requested: [u64; 2] = serde_json::from_str(&std::env::var("NUWAX_FINE_REPLAY_EXPECTED_GROUND")
        .expect("指定真实源序号和原索引")).unwrap();
    replay_native_ground(ReplayCheck::CurrentGround {
        sequence: requested[0], index: u32::try_from(requested[1]).unwrap(),
    });
}

#[derive(Clone, Copy, Debug)]
enum ReplayCheck {
    RetainedMembers,
    CurrentGround { sequence: u64, index: u32 },
}

fn replay_native_ground(check: ReplayCheck) {
    let input = std::env::var("NUWAX_FINE_REPLAY_FILE").expect("真实生产累计输入路径");
    let output = std::env::var("NUWAX_FINE_REPLAY_REPORT_DIR").expect("独立诊断输出目录");
    let output = Path::new(&output);
    assert!(!output.exists(), "诊断结果不得覆盖已有红绿证据");
    std::fs::create_dir_all(output).unwrap();
    let target: Key = serde_json::from_str(&std::env::var("NUWAX_FINE_REPLAY_TARGET")
        .unwrap_or_else(|_| "[-8,-1,-7]".into())).unwrap();
    let companion: Option<Key> = std::env::var("NUWAX_FINE_REPLAY_COMPANION")
        .ok().map(|value| serde_json::from_str(&value).unwrap());
    if let Some(companion) = companion {
        assert!(matches!(check, ReplayCheck::RetainedMembers), "同层成员与当前索引分别具名验证");
        assert_eq!(&target[..2], &companion[..2], "同层对照必须来自同一实际细列");
        assert_eq!(block_key(target), block_key(companion), "本具名对照限定同块真实成员");
    }
    let maximum_sequence = std::env::var("NUWAX_FINE_REPLAY_MAX_SEQUENCE")
        .map(|value| value.parse::<u64>().unwrap()).unwrap_or(60);
    assert_eq!(std::env::var("NUWAX_GEOMETRY_LIO_BACKEND").unwrap(), "fastlio2", "与原生产动态近距配置一致");
    assert!(std::env::var("NUWAX_NAV_CAPABILITIES_JSON").unwrap_or_default().is_empty(), "本批生产未配置额外观察范围");
    let mut fine = FineCloud::new(0.05).unwrap();
    let mut records = std::fs::File::create(output.join("逐帧累计删除与重观察.jsonl")).unwrap();
    let mut first_claim = None;
    let mut first_loss = None;
    let mut first_capture_loss = None;
    let mut first_sequence = None;
    let mut losses = Vec::new();
    let mut capture_losses = Vec::new();
    let mut same_layer_splits = Vec::new();
    let mut companion_observed_frames = 0;
    let mut required_ground = None;
    let mut frame_count = 0_usize;
    let mut point_count = 0_usize;
    let mut previous = 0_u64;
    for line in std::fs::read_to_string(input).unwrap().lines() {
        let frame: RecordedFrame = serde_json::from_str(line).unwrap();
        if frame.sequence > maximum_sequence { break; }
        assert!(frame.sequence > previous, "保持原序号顺序，不重新编号");
        first_sequence.get_or_insert(frame.sequence);
        previous = frame.sequence;
        assert_eq!(frame.points.len(), frame.indices.len());
        assert_eq!(frame.origin_status, "exact_correlated_canonical_pose", "缺原点时不得用真值或插值补造");
        let origin = frame.origin_map.expect("当前验证前缀必须具有精确生产原点");
        let points: Vec<_> = frame.points.iter().copied().map(Point3::from).collect();
        let before = target_state(&mut fine, target);
        // 原生产顺序：动态射线删除 -> 写入当前占据 -> 同帧地面捕获。
        fine.accumulate_inner(&points, Some(origin), frame.timestamp_ns,None).unwrap();
        let mut removed: Vec<Key> = fine.removed_keys().iter().map(|key| [key.0, key.1, key.2]).collect();
        removed.sort_unstable();
        let after_accumulate = target_state(&mut fine, target);
        let current_source = Arc::new(frame.source.clone());
        let dynamic = &fine.dynamic;
        let classified = ground_observations::capture(&mut fine.writer, &points,
            frame.sequence, frame.timestamp_ns, current_source.clone(),
            |index| Some(frame.indices[index]), |point| dynamic.is_dynamic([point.x, point.y, point.z])).unwrap();
        if let ReplayCheck::CurrentGround { sequence, index } = check {
            if frame.sequence == sequence {
                let position = frame.indices.iter().position(|actual| *actual == index)
                    .expect("指定原索引必须存在于真实帧中");
                required_ground = Some(serde_json::json!({"sequence":sequence,"index":index,
                    "source_timestamp_ns":frame.timestamp_ns,"original_point":frame.points[position],
                    "published_ground":frame.published_ground_indices.contains(&index),
                    "replayed_ground":classified.contains(&index)}));
            }
        }
        fine.writer.record_source_watermark(current_source, frame.timestamp_ns, frame.sequence).unwrap();
        let after_capture = target_state(&mut fine, target);
        let companion_state = companion.map(|key| target_state(&mut fine, key));
        let same_layer = companion.map(|key| after_capture["column_layers"].as_array().unwrap().iter()
            .any(|layer| layer["members_global"].as_array().unwrap().contains(&serde_json::json!(target))
                && layer["members_global"].as_array().unwrap().contains(&serde_json::json!(key))));
        if after_capture["claimed"] == true && companion_state.as_ref().is_some_and(|state| state["claimed"] == true) {
            companion_observed_frames += 1;
            if same_layer == Some(false) {
                same_layer_splits.push(serde_json::json!({"sequence":frame.sequence,
                    "timestamp_ns":frame.timestamp_ns,"target":target,"companion":companion,
                    "after_capture":after_capture,"companion_state":companion_state}));
            }
        }
        let target_indices: Vec<_> = frame.points.iter().zip(&frame.indices).filter_map(|(point, index)| {
            let key = voxel_key(*point, 0.05).unwrap();
            ([key.0, key.1, key.2] == target).then_some(*index)
        }).collect();
        let lost_after_remove = before["claimed"] == true
            && after_accumulate["occupied"] == true && after_accumulate["claimed"] == false;
        if lost_after_remove {
            let event = serde_json::json!({"sequence":frame.sequence,"timestamp_ns":frame.timestamp_ns,
                "removed":removed,"target":target,"before":before,"after_accumulate":after_accumulate,
                "after_capture":after_capture});
            if first_loss.is_none() { first_loss = Some(frame.sequence); }
            losses.push(event);
        }
        let lost_after_capture = after_accumulate["claimed"] == true
            && after_capture["occupied"] == true && after_capture["claimed"] == false;
        if lost_after_capture {
            first_capture_loss.get_or_insert(frame.sequence);
            capture_losses.push(serde_json::json!({
                "sequence": frame.sequence, "timestamp_ns": frame.timestamp_ns,
                "target": target, "before": before, "after_accumulate": after_accumulate,
                "after_capture": after_capture, "removed": removed,
            }));
        }
        // 每帧走真实封存/读取边界；重放修订号与生产发布节流修订号分开标记。
        frame_count += 1;
        point_count += points.len();
        let sealed = fine.seal(frame.domain, frame_count as u64, frame.timestamp_ns).unwrap();
        fine.retain_committed(&sealed);
        if first_claim.is_none() && after_capture["claimed"] == true {
            first_claim = Some(frame.sequence);
            save_archive(&fine, &sealed, &output.join(format!("首次拥有目标成员_{}.bin", frame.sequence)));
        }
        if first_loss == Some(frame.sequence) {
            save_archive(&fine, &sealed, &output.join(format!("首次删除后重观察_{}.bin", frame.sequence)));
        }
        if first_capture_loss == Some(frame.sequence) {
            save_archive(&fine, &sealed, &output.join(format!("首次捕获后丢成员_{}.bin", frame.sequence)));
        }
        writeln!(records, "{}", serde_json::json!({
            "sequence":frame.sequence,"timestamp_ns":frame.timestamp_ns,
            "correlated_odometry_source_sequence":frame.correlated_odometry_source_sequence,
            "correlated_odometry_source_timestamp_ns":frame.correlated_odometry_source_timestamp_ns,
            "origin_map":origin,"native_file":frame.native_file,"native_sha256":frame.native_sha256,
            "points":points.len(),"target_indices":target_indices,
            "published_target_ground_indices":target_indices.iter().filter(|i|frame.published_ground_indices.contains(i)).collect::<Vec<_>>(),
            "replayed_target_ground_indices":target_indices.iter().filter(|i|classified.contains(i)).collect::<Vec<_>>(),
            "removed_keys":removed,"before":before,"after_accumulate":after_accumulate,"after_capture":after_capture,
            "companion_state":companion_state,"same_physical_layer":same_layer,
            "replay_snapshot_revision":sealed.manifest().revision,
            "snapshot_blocks":sealed.manifest().blocks.len(),"snapshot_points":sealed.manifest().points(),
        })).unwrap();
    }
    let summary = serde_json::json!({"frames":frame_count,"points":point_count,"target":target,
        "first_input_sequence":first_sequence,
        "first_claim_sequence":first_claim,"first_loss_sequence":first_loss,"losses":losses,
        "first_capture_loss_sequence":first_capture_loss,"capture_losses":capture_losses,
        "independently_verified_same_layer_companion":companion,
        "companion_observed_frames":companion_observed_frames,"same_layer_splits":same_layer_splits,
        "assertion":format!("{check:?}"),"required_current_ground":required_ground,
        "maximum_source_sequence":maximum_sequence,
        "limits":[format!("输入从实际源序号{}开始，未补造此前缺失前缀", first_sequence.unwrap_or(0)), "地图点来自公开f32载荷，不冒称内部原f64",
            "原点只用精确关联规范位姿与生产配置外参", "每帧封存用于对账，不冒称生产发布频率"]});
    std::fs::write(output.join("动态删除成员对账汇总.json"), serde_json::to_vec_pretty(&summary).unwrap()).unwrap();
    eprintln!("实际原点完整累计：{frame_count}帧/{point_count}点，检查={check:?}，首成员={first_claim:?}，首删除丢层={first_loss:?}，首捕获丢层={first_capture_loss:?}");
    match check {
        ReplayCheck::RetainedMembers => {
            assert!(first_claim.is_some(), "前缀中必须实际建立过目标地面成员");
            assert!(losses.is_empty(), "动态删除不能撤掉仍有占据证据的同层未删成员；已保存逐步红测");
            assert!(capture_losses.is_empty(), "已独立核实的同层真实地面不能因捕获顺序丢失旧成员；已保存逐步红测");
            if companion.is_some() {
                assert!(companion_observed_frames > 0, "指定同层双成员须真实出现，不能用空对照通过");
                assert!(same_layer_splits.is_empty(), "已独立核实同层且有完整样本覆盖的成员不能被过时拟合输出拆层");
            }
        }
        ReplayCheck::CurrentGround { .. } => {
            let observation = required_ground.expect("完整前缀必须覆盖指定真实帧，不能以空查询通过");
            assert_eq!(observation["replayed_ground"], true, "独立核实的指定当前原索引必须由完整生产累计正确分类");
        }
    }
}
