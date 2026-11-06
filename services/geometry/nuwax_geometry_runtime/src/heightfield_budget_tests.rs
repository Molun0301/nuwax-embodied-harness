//! 使用生产捕获入口复现固定面积、多层、完整精度回波的字节增长。
use super::*;
use nuwax_map_chunks::{Domain, GroundRecord, GroundRecords, Repository, MAX_BLOCK_BYTES};

#[test]
fn repeated_dense_observations_keep_ground_records_bounded() {
    let mut rows = Vec::new();
    for layer_count in [2, 3, 4] {
        let q = 0.05_f32;
        let mut writer =
            Writer::new(Repository::temporary().unwrap(), q, 64 * 1024 * 1024).unwrap();
        let source = Arc::new(LayerSource::Lidar {
            backend: 1,
            runtime_generation: 1793277429123456789,
            sensor_frame_id: "g1_imu_base_link".into(),
        });
        let domain = Domain {
            session: "dense-ground-budget".into(),
            };
        let mut plateau_bytes = None;
        let mut exceeded_old = false;
        for frame in 1..=160_u64 {
            let stamp = 1793277430786632777 + frame * 2_000_000_000;
            // 每个水平面面积约1.6×1.6米；每格每帧一个原始输入点，多层位于同一32³块。
            let mut returns = Vec::new();
            for z in (0..layer_count).map(|layer| layer * (32 / layer_count)) {
                for x in 0..32 {
                    for y in 0..32 {
                        let offset = ((frame * 17 + (x * 31 + y * 13) as u64) % 97) as f64 / 101.;
                        let point = [
                            (f64::from(x) + 0.04 + offset) * f64::from(q),
                            (f64::from(y) + 0.04 + offset * 0.973) * f64::from(q),
                            (f64::from(z) + 0.03141592653589793) * f64::from(q),
                        ];
                        returns.push(Return::from_key(
                            point,
                            [x, y, z],
                            returns.len() as u32,
                            false,
                            q,
                        ));
                    }
                }
            }
            writer
                .observe(returns.iter().map(|p| p.key), stamp)
                .unwrap();
            let indices = capture(
                &mut writer,
                &returns,
                &[],
                frame + 392,
                stamp,
                source.clone(),
            )
            .unwrap();
            assert_eq!(indices.len(), returns.len());
            if [1, 8, 16, 24, 32, 40, 64, 96, 128, 160].contains(&frame) {
                let root = writer.snapshot(domain.clone(), frame, stamp).unwrap();
                assert_eq!(root.blocks.len(), 1);
                let bytes = writer
                    .repository()
                    .read_object(root.blocks[0].object, MAX_BLOCK_BYTES)
                    .unwrap();
                let records = nuwax_map_chunks::decode_ground_records(&bytes).unwrap();
                let GroundRecords::Versioned(records) = records else {
                    panic!("必须保留已分类高度场");
                };
                let samples: usize = records
                    .values()
                    .map(|r| match r {
                        GroundRecord::HeightfieldV1(f) => {
                            assert!(f.samples.len() <= MAX_LAYER_SAMPLES);
                            f.samples.len()
                        }
                        _ => panic!("实测平面不得退化为空列"),
                    })
                    .sum();
                assert_eq!(records.len(), 1024 * layer_count as usize);
                let json = serde_json::to_vec(&records.iter().collect::<Vec<_>>()).unwrap();
                let old_bytes = 16 + root.blocks[0].points as usize * 12 + json.len();
                exceeded_old |= old_bytes > nuwax_map_chunks::MAX_V1_TO_V5_BLOCK_BYTES;
                if frame >= 32 {
                    assert_eq!(samples, 1024 * layer_count as usize * MAX_LAYER_SAMPLES);
                    if let Some(plateau) = plateau_bytes {
                        assert_eq!(bytes.len(), plateau);
                    }
                    plateau_bytes = Some(bytes.len());
                }
                rows.push(format!(
                    "{layer_count},{frame},{},{},{samples},{old_bytes},{}",
                    frame * 2,
                    records.len(),
                    bytes.len()
                ));
                assert_eq!(
                    nuwax_map_chunks::decode_block(&bytes).unwrap().len(),
                    1024 * layer_count as usize
                );
            }
        }
        assert!(exceeded_old, "必须用生产捕获结果复现原BL4超出4MiB");
    }
    let report = format!(
        "layers,frame,seconds,records,samples,bl4_bytes,bl6_bytes\n{}\n",
        rows.join("\n")
    );
    eprintln!("{report}");
    if let Some(path) = std::env::var_os("NEH_GROUND_BUDGET_CURVE") {
        std::fs::write(path, report).unwrap();
    }
}
