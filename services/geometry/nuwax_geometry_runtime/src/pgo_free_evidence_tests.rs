
use super::*;
use crate::dynamic_backend::OnlineDufo;

fn cell(key: [i32; 3], depth: u32) -> FreeCell {
    FreeCell { key, depth }
}

#[test]
fn bloom_is_fixed_eight_mib_and_never_misses_signed_f32_leaf_or_parent() {
    let mut bloom = ProcessedEvidence::new();
    let size = bloom.memory_bytes();
    assert_eq!(
        size,
        PROCESSED_EVIDENCE_BYTES + std::mem::size_of::<ProcessedEvidence>()
    );
    assert!(!bloom.may_intersect(&[cell([0; 3], 0)]));
    let r = super::super::online_params().resolution;
    // 0.15 的 f32 值位于双精度边界另一侧；-0.15 的符号方向也必须正确。
    for value in [
        0.,
        -0.,
        0.15,
        -0.15,
        0.3,
        -0.3,
        1.5,
        -1.5,
        f64::from(f32::from_bits(0.15f32.to_bits() - 1)),
        f64::from(f32::from_bits(0.15f32.to_bits() + 1)),
        f64::from(f32::from_bits((-0.15f32).to_bits() - 1)),
        f64::from(f32::from_bits((-0.15f32).to_bits() + 1)),
    ] {
        let point = Point3::new(value, -value, value / 2.);
        let key = [point.x, point.y, point.z].map(|v| (f64::from(v as f32) / r).floor() as i32);
        bloom.insert_point(point);
        assert_eq!(bloom.leaf_key(point), Some(key));
        assert!(bloom.may_intersect(&[cell(key, 0)]));
        assert!(bloom.may_intersect(&[cell(key.map(|v| v.div_euclid(2) * 2), 1)]));
    }
    assert_eq!(
        bloom.leaf_key(Point3::new(-0.15, 0.15, 0.)),
        Some([-2, 1, 0])
    );
    for x in -32..=32 {
        for y in -8..=8 {
            for z in -2..=2 {
                let key = [x, y, z];
                let point = Point3::from(key.map(|v| (f64::from(v) + 0.5) * r));
                bloom.insert_point(point);
                assert!(bloom.may_intersect(&[cell(key, 0)]), "已插入叶子不得漏检");
                assert!(bloom.may_intersect(&[cell(key.map(|v| v.div_euclid(2) * 2), 1)]));
            }
        }
    }
    assert!(bloom.may_intersect(&[cell([i32::MAX; 3], 1)]));
    assert!(bloom.may_intersect(&[cell([0; 3], 2)]));
    assert_eq!(bloom.memory_bytes(), size);
    bloom.insert_point(Point3::new(f64::INFINITY, 0., 0.));
    assert!(bloom.may_intersect(&[cell([12345; 3], 0)]));
    assert!(!bloom.may_intersect(&[]));
}

#[test]
fn journal_bounds_gaps_eviction_oversize_and_shared_clone_continuation() {
    let mut journal = FreeChangeJournal::default();
    assert_eq!(journal.since(0, 0), Some(vec![]));
    assert!(journal.since(0, 1).is_none(), "旧代码未知修订必须保守拒绝");
    for revision in 1..=65 {
        journal.record(revision, &[cell([revision as i32, 0, 0], 0)]);
    }
    assert_eq!(journal.entries.len(), FREE_CHANGE_MAX_REVISIONS);
    assert!(journal.since(0, 65).is_none());
    assert_eq!(journal.since(1, 65).unwrap().len(), 64);
    let mut fork = journal.clone();
    assert!(Arc::ptr_eq(&journal.entries[0].1, &fork.entries[0].1));
    fork.record(66, &[cell([66, 0, 0], 0)]);
    assert_eq!(journal.latest, 65);
    assert_eq!(fork.latest, 66);
    assert_eq!(journal.entries.len(), 64);
    let half = vec![cell([7; 3], 0); FREE_CHANGE_PAYLOAD_BYTES / CELL_BYTES / 2];
    journal.record(66, &half);
    journal.record(67, &half);
    assert_eq!(journal.payload_bytes, FREE_CHANGE_PAYLOAD_BYTES);
    assert!(journal.since(64, 67).is_none());
    assert_eq!(journal.since(65, 67).unwrap().len(), half.len() * 2);
    let oversized = vec![cell([8; 3], 0); FREE_CHANGE_PAYLOAD_BYTES / CELL_BYTES + 1];
    journal.record(68, &oversized);
    assert!(journal.entries.is_empty());
    assert_eq!(journal.payload_bytes, 0);
    assert!(journal.since(67, 68).is_none());
    journal.record(69, &[]);
    assert_eq!(journal.latest, 68);
    journal.record(69, &[cell([9; 3], 0)]);
    assert_eq!(journal.since(68, 69), Some(vec![cell([9; 3], 0)]));
    journal.record(72, &[cell([10; 3], 0)]);
    assert!(journal.since(69, 72).is_none());
    assert_eq!(journal.since(71, 72).unwrap().len(), 1);
    assert!(journal.since(73, 72).is_none());
}

#[test]
fn real_online_free_changes_distinguish_far_space_and_previously_retained_person() {
    use crate::dynamic_scene::tests::{corridor, ORIGIN};
    let (points, truth) = corridor(Some(0.), 0.);
    let mut online = OnlineDufo::default();
    let (_, kept) = online
        .observe(&points, ORIGIN, 1, nalgebra::Isometry3::identity())
        .unwrap();
    let people: Vec<_> = kept
        .into_iter()
        .filter(|&i| truth[i as usize])
        .map(|i| points[i as usize])
        .collect();
    assert!(!people.is_empty());
    let mut bloom = ProcessedEvidence::new();
    for point in &people {
        bloom.insert_point(*point);
    }
    let before = online.free_revision();
    let (empty, _) = corridor(None, 0.);
    for index in 0..32 {
        let far: Vec<_> = empty
            .iter()
            .map(|p| p + nalgebra::Vector3::new(100., 0., 0.))
            .collect();
        online
            .observe(
                &far,
                [ORIGIN[0] + 100., ORIGIN[1], ORIGIN[2]],
                index + 2,
                nalgebra::Isometry3::identity(),
            )
            .unwrap();
    }
    let (frozen, changes) = online.snapshot_since(before);
    let changes = changes.expect("短场自由变化必须有完整日志");
    assert!(!changes.is_empty());
    assert!(
        !bloom.may_intersect(&changes),
        "远处变化不能要求重放近处保留点"
    );
    let before = frozen.free_revision();
    for index in 0..33 {
        online
            .observe(&empty, ORIGIN, index + 34, nalgebra::Isometry3::identity())
            .unwrap();
    }
    let (newer, changes) = online.snapshot_since(before);
    let changes = changes.expect("相邻修订必须有完整日志");
    let frame = newer.terminal_frame(&people, &people).unwrap();
    assert!(
        people.iter().any(|p| frame.is_dynamic([p.x, p.y, p.z])),
        "人体离开后必须出现真实相关空闲证据"
    );
    assert!(
        bloom.may_intersect(&changes),
        "真实相关变化不得被布隆过滤器遗漏"
    );
    assert_eq!(
        frozen.free_revision(),
        before,
        "冻结树的独立后续修订不能被在线修改"
    );
    assert_eq!(frozen.snapshot_since(before).1, Some(vec![]));
    assert_eq!(OnlineDufo::default().snapshot_since(0).1, Some(vec![]));
}
