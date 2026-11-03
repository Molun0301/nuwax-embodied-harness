use super::*;

fn fixture() -> (Block, GroundRecords) {
    let block = [(0, Evidence { observed_ns: u64::MAX, hits: u16::MAX }),
        (1, Evidence { observed_ns: u64::MAX, hits: 1 })].into();
    let field = HeightfieldFragment {
        anchor: [i32::MIN, i32::MAX, -32], height_m: -0.0, occupied_bounds: [-1., 1.],
        members: vec![(0, [1., 0., 0., 0., 0., 0., 0., 0.])],
        samples: vec![LayerSample { source: Arc::new(LayerSource::Auxiliary {
            sensor_id: "雷达".into(), clock_id: "时钟".into(), runtime_generation: u64::MAX, session_generation: 1 }),
            point: [-0., f64::from_bits(1), f64::from_bits(0x3ff0000000000001)],
            source_sequence: u64::MAX, source_timestamp_ns: u64::MAX, point_index: u32::MAX }],
    };
    (block, GroundRecords::Versioned(Arc::new([(0, GroundRecord::HeightfieldV1(field)),
        (1, GroundRecord::ClassifiedEmptyColumn {})].into())))
}

#[test]
fn bl5_keeps_exact_bits_and_compact_bl4_records() {
    let (block, records) = fixture();
    let GroundRecords::Versioned(entries) = &records else { unreachable!() };
    let current = ground_binary::encode(&block, entries).unwrap();
    assert!(current.starts_with(b"NUWAXBL5"));
    let compact = serde_json::to_vec(&entries.iter().collect::<Vec<_>>()).unwrap();
    let GroundRecord::HeightfieldV1(field) = &entries[&0] else { unreachable!() };
    let mut variants = vec![current.clone()];
    for json in [compact] {
        let mut bytes = encode_block(&block).unwrap();
        bytes[..8].copy_from_slice(b"NUWAXBL4");
        bytes[12..16].copy_from_slice(&(json.len() as u32).to_le_bytes());
        bytes.extend(json); variants.push(bytes);
    }
    for bytes in variants {
        let (actual_block, actual) = decode_block_parts(&bytes).unwrap();
        assert_eq!(actual_block, block); assert_eq!(actual, records);
        let GroundRecordRef::HeightfieldV1(decoded) = actual.get(0).unwrap() else { unreachable!() };
        assert_eq!(decoded.height_m.to_bits(), field.height_m.to_bits());
        assert_eq!(decoded.samples[0].point.map(f64::to_bits), field.samples[0].point.map(f64::to_bits));
        let GroundRecords::Versioned(entries) = &actual else { unreachable!() };
        assert_eq!(ground_binary::encode(&actual_block, entries).unwrap(), current);
        let newest = encode_ground_block(&actual_block, &actual).unwrap();
        assert!(newest.starts_with(b"NUWAXBL6"));
        assert_eq!(decode_ground_records(&newest).unwrap(), records);
    }
}

#[test]
fn bl5_rejects_truncation_counts_tags_trailing_and_invalid_float_without_panicking() {
    let (block, records) = fixture();
    let GroundRecords::Versioned(entries) = &records else { unreachable!() };
    let bytes = ground_binary::encode(&block, entries).unwrap();
    let start = 16 + block.len() * 12;
    for end in 0..bytes.len() {
        let mut truncated = bytes[..end].to_vec();
        if end >= start { truncated[12..16].copy_from_slice(&((end - start) as u32).to_le_bytes()); }
        assert!(decode_ground_records(&truncated).is_err(), "截断位置{end}");
    }
    for (offset, value) in [(start, u32::MAX.to_le_bytes().to_vec()),
        (start + 6, vec![255]), (start + 7 + 12, f64::NAN.to_le_bytes().to_vec()),
        (start + 7 + 12 + 8 + 16, vec![33])] {
        let mut changed = bytes.clone(); changed[offset..offset + value.len()].copy_from_slice(&value);
        assert!(decode_ground_records(&changed).is_err());
    }
    let mut trailing = bytes.clone(); trailing.push(0);
    let size = (trailing.len() - start) as u32;
    trailing[12..16].copy_from_slice(&size.to_le_bytes());
    assert!(decode_ground_records(&trailing).is_err());
}

#[test]
fn failed_block_preserves_previous_root_and_retry_clears_typed_failure() {
    let repo = Repository::temporary().unwrap();
    let mut writer = Writer::new(repo.clone(), 0.05, MAX_BLOCK_BYTES).unwrap();
    let domain = Domain { session: "failure".into(), generation: 1, frame_generation: 0, producer_generation: 1 };
    writer.observe([[0,0,0]], 1).unwrap();
    let before = writer.snapshot(domain.clone(), 1, 1).unwrap();
    let old = repo.read_object(before.blocks[0].object, MAX_BLOCK_BYTES).unwrap();
    writer.observe([[1,0,0]], 2).unwrap();
    repo.fail_put_after(Some(0));
    assert!(writer.snapshot(domain.clone(), 2, 2).is_err());
    assert_eq!(writer.block_write_failure().unwrap().key, [0,0,0]);
    assert_eq!(repo.read_object(before.blocks[0].object, MAX_BLOCK_BYTES).unwrap(), old);
    repo.fail_put_after(None);
    let after = writer.snapshot(domain, 2, 2).unwrap();
    assert!(writer.block_write_failure().is_none());
    assert_eq!(after.blocks[0].points, 2);
}
