//! 使用全部原结构上限验证容量，不降低层数或原样本数。
use super::*;
const STAMP: u64 = 1_793_277_430_786_632_777;
const Q: f32 = 0.05;

fn dense(layers: u16) -> (Block, GroundRecords, Arc<LayerSource>) {
    let source = Arc::new(LayerSource::Lidar { backend:1, runtime_generation:1, sensor_frame_id:"g1_imu_base_link".into() });
    let mut block = Block::new(); let mut records = BTreeMap::new();
    for z in 0..layers {
        for xy in 0..1024 {
            let local = xy + 1024*z; let x = i32::from(xy%32); let y = i32::from(xy/32);
            block.insert(local, Evidence {observed_ns:STAMP,hits:32});
            records.insert(local, GroundRecord::HeightfieldV1(HeightfieldFragment {
                anchor:[x,y,i32::from(z)], height_m:(f64::from(z)+0.0625)*f64::from(Q),
                occupied_bounds:[f64::from(z)*f64::from(Q),(f64::from(z)+1.)*f64::from(Q)],
                members:vec![(local,[1.,0.,0.,0.,0.,0.,0.,0.])],
                samples:(0..32_u32).map(|i|LayerSample {source:source.clone(),
                    point:[(f64::from(x)+0.03141592653589793+f64::from(i)/33.)*f64::from(Q),
                        (f64::from(y)+0.04+f64::from(i)/34.)*f64::from(Q), (f64::from(z)+0.0625)*f64::from(Q)],
                    source_sequence:u64::from(i)+1,source_timestamp_ns:STAMP-31+u64::from(i),point_index:u32::from(local)}).collect(),
            }));
        }
    }
    (block,GroundRecords::Versioned(Arc::new(records)),source)
}
fn root(object:ObjectIdentity, points:usize, sources:Vec<Arc<LayerSource>>) -> Manifest {
    Manifest {version:5,domain:Domain{session:"worst-ground".into(),},
        revision:1,geometry_revision:1,source_timestamp_ns:STAMP,voxel_size_m:Q,
        blocks:vec![BlockRef {key:[0,0,0],object,points:points as u32,block_revision:1,geometry_revision:1}],
        source_watermarks:sources.into_iter().map(|source|SourceWatermark {source,source_sequence:32,source_timestamp_ns:STAMP}).collect()}
}
fn artifact(name:&str, root:&Manifest, repo:&Repository) {
    if let Some(directory)=std::env::var_os("NEH_HEIGHTFIELD_ARCHIVE_DIR") {
        use std::io::Read;
        let(mut file,_)=archive_file(root,repo).unwrap();let mut bytes=Vec::new();file.read_to_end(&mut bytes).unwrap();
        std::fs::create_dir_all(&directory).unwrap();std::fs::write(std::path::Path::new(&directory).join(name),bytes).unwrap();
    }
}

#[test]
fn bl6_three_four_and_maximum_layers_preserve_every_sample_and_original_shape() {
    assert_eq!(MAX_BLOCK_NUMERIC_BYTES,54_231_069);
    assert_eq!(MAX_BLOCK_BYTES,91_979_805);
    for layers in [3,4,32] {
        let(block,records,source)=dense(layers);
        let bytes=encode_ground_block(&block,&records).unwrap();
        let expected=29+1655*block.len()+ground_shared::source_bytes(&source);
        assert_eq!(bytes.len(),expected);
        assert!(bytes.len()>MAX_V1_TO_V5_BLOCK_BYTES && bytes.len()<=MAX_BLOCK_BYTES);
        let(decoded,actual)=decode_block_parts(&bytes).unwrap();
        assert_eq!(decoded,block);assert_eq!(actual,records);
        assert_eq!(block_summary(&bytes).unwrap(),(block.len() as u32,STAMP));
        let repo=Repository::temporary().unwrap();let id=ObjectId::random().unwrap().into();repo.put_object(id,&bytes).unwrap();
        let manifest=root(id,block.len(),vec![source]);
        let mut reader=Receiver::with_cache_budget(Repository::temporary().unwrap(),4*1024*1024);
        let received=reader.receive(&Transfer{root:manifest.encode().unwrap(),included:BTreeMap::new()},&manifest.domain,
            |_,r|repo.read_object(r.object,MAX_BLOCK_BYTES)).unwrap();
        assert_eq!(reader.ground_records(&received.blocks[0]).unwrap(),records);
        assert!(reader.reader.cached_bytes()<=4*1024*1024);
        if layers<32 {artifact(&format!("BL6-{layers}层.ar5"),&manifest,&repo);}
        eprintln!("BL6_CAPACITY layers={layers} records={} samples={} bytes={} theoretical_max={MAX_BLOCK_BYTES}",block.len(),block.len()*32,bytes.len());
    }
}

#[test]
fn bl6_near_full_source_and_numeric_bounds_fit_and_removing_root_sources_revalidates_cached_objects() {
    let(block,mut records,normal)=dense(32);
    let declared=Arc::new(LayerSource::Auxiliary {sensor_id:format!("z{}","a".repeat(MAX_MANIFEST_BYTES-4096-26)),
        clock_id:"c".into(),runtime_generation:1,session_generation:1});
    let old=Arc::new(LayerSource::Auxiliary {sensor_id:format!("y{}","a".repeat(MAX_V1_TO_V5_BLOCK_BYTES-1024-26)),
        clock_id:"c".into(),runtime_generation:1,session_generation:1});
    let field=match records.versioned_mut().get_mut(&0).unwrap() {GroundRecord::HeightfieldV1(f)=>f,_=>unreachable!()};
    field.samples[30].source=old.clone(); field.samples[31].source=declared.clone();
    // 未声明来源确实能完整来自旧BL5，而非凭空增加兼容额度。
    let mut legacy=field.clone();legacy.samples=vec![legacy.samples[30].clone()];
    assert!(ground_binary::encode(&[(0,block[&0])].into(),&[(0,GroundRecord::HeightfieldV1(legacy))].into()).unwrap().len()<=MAX_V1_TO_V5_BLOCK_BYTES);
    let bytes=encode_ground_block(&block,&records).unwrap();
    assert!(bytes.len()<=MAX_BLOCK_BYTES && MAX_BLOCK_BYTES-bytes.len()<8192);
    let repo=Repository::temporary().unwrap();let id=ObjectId::random().unwrap().into();repo.put_object(id,&bytes).unwrap();
    let manifest=root(id,block.len(),vec![normal,declared]);assert!(manifest.encode().unwrap().len()<=MAX_MANIFEST_BYTES);
    let mut receiver=Receiver::with_cache_budget(Repository::temporary().unwrap(),256*1024*1024);
    receiver.receive(&Transfer{root:manifest.encode().unwrap(),included:BTreeMap::new()},&manifest.domain,
        |_,r|repo.read_object(r.object,MAX_BLOCK_BYTES)).unwrap();
    let mut bad=manifest.clone();bad.revision=2;bad.source_watermarks.clear();
    let error=receiver.receive(&Transfer{root:bad.encode().unwrap(),included:BTreeMap::new()},&bad.domain,
        |_,_|panic!("已缓存对象不应补取")).unwrap_err();
    assert!(error.contains("unregistered source"),"{error}");
    assert_eq!(receiver.current().unwrap().revision,1);
    let mut small=match &records { GroundRecords::Versioned(values) => values.as_ref().clone() };small.retain(|&key,_|key==0);
    let small_block:Block=[(0,block[&0])].into();
    let small_bytes=encode_ground_block(&small_block,&GroundRecords::Versioned(Arc::new(small))).unwrap();
    let small_id=ObjectId::random().unwrap().into();repo.put_object(small_id,&small_bytes).unwrap();
    let mut small_root=manifest.clone();small_root.blocks[0].object=small_id;small_root.blocks[0].points=1;
    artifact("BL6-来源边界.ar5",&small_root,&repo);
    let mut writer=Writer::new(repo,Q,4*1024*1024).unwrap();assert!(writer.restore(&bad).is_err());
    eprintln!("BL6_EXTREME records=32768 samples=1048576 bytes={} bound={MAX_BLOCK_BYTES} slack={}",bytes.len(),MAX_BLOCK_BYTES-bytes.len());
}



#[test]
fn bl6_rejects_bad_lengths_indices_modes_and_all_truncations_of_a_small_record() {
    let(block,records,_)=dense(1);
    let mut one=match &records { GroundRecords::Versioned(values) => values.as_ref().clone() };one.retain(|&key,_|key==0);
    let small:Block=[(0,block[&0])].into();
    let bytes=encode_ground_block(&small,&GroundRecords::Versioned(Arc::new(one))).unwrap();let start=28;
    for end in 0..bytes.len() {
        let mut bad=bytes[..end].to_vec();
        if end>=start {bad[12..16].copy_from_slice(&((end-start) as u32).to_le_bytes());}
        assert!(decode_ground_records(&bad).is_err(),"截断{end}");
    }
    for (offset,value) in [(start,vec![3]),(start+1,u32::MAX.to_le_bytes().to_vec()),
        (start+5,u32::MAX.to_le_bytes().to_vec()),(start+9,vec![255])] {
        let mut bad=bytes.clone();bad[offset..offset+value.len()].copy_from_slice(&value);assert!(decode_ground_records(&bad).is_err());
    }
    let mut oversized=bytes.clone();oversized.resize(MAX_BLOCK_BYTES+1,0);
    oversized[12..16].copy_from_slice(&((MAX_BLOCK_BYTES+1-start) as u32).to_le_bytes());
    assert_eq!(decode_ground_records(&oversized).unwrap_err(),"精细块格式或长度无效");
    let mut bad=bytes;bad.push(0);let size=(bad.len()-start) as u32;bad[12..16].copy_from_slice(&size.to_le_bytes());
    assert!(decode_ground_records(&bad).is_err());
}

#[test]
fn failed_candidate_cannot_lend_its_source_declaration_to_a_later_root() {
    let repo=Repository::temporary().unwrap();
    let base:Block=[(0,Evidence{observed_ns:STAMP,hits:1})].into();
    let old_id=ObjectId::random().unwrap().into();repo.put_object(old_id,&encode_block(&base).unwrap()).unwrap();
    let old=root(old_id,1,vec![]);
    let mut receiver=Receiver::with_cache_budget(Repository::temporary().unwrap(),16*1024*1024);
    receiver.receive(&Transfer{root:old.encode().unwrap(),included:BTreeMap::new()},&old.domain,
        |_,r|repo.read_object(r.object,MAX_BLOCK_BYTES)).unwrap();
    let(_,all,_)=dense(1);let mut records=match &all { GroundRecords::Versioned(values) => values.as_ref().clone() };records.retain(|&key,_|key==0);
    let source=Arc::new(LayerSource::Auxiliary {sensor_id:"x".repeat(MAX_V1_TO_V5_BLOCK_BYTES+1),
        clock_id:"c".into(),runtime_generation:1,session_generation:1});
    let GroundRecord::HeightfieldV1(field)=records.get_mut(&0).unwrap() else{unreachable!()};
    field.samples.truncate(1);field.samples[0].source=source.clone();
    let bytes=encode_ground_block(&base,&GroundRecords::Versioned(Arc::new(records))).unwrap();
    let id=ObjectId::random().unwrap().into();repo.put_object(id,&bytes).unwrap();
    let mut candidate=root(id,1,vec![source]);candidate.revision=2;candidate.geometry_revision=2;
    candidate.blocks[0].block_revision=2;candidate.blocks[0].geometry_revision=2;
    candidate.blocks.push(BlockRef{key:[1,0,0],object:ObjectId::random().unwrap().into(),points:1,block_revision:2,geometry_revision:2});
    assert!(receiver.receive(&Transfer{root:candidate.encode().unwrap(),included:BTreeMap::new()},&candidate.domain,
        |_,r|repo.read_object(r.object,MAX_BLOCK_BYTES)).is_err());
    assert_eq!(receiver.current().unwrap().revision,1);
    candidate.blocks.pop();candidate.source_watermarks.clear();
    let error=receiver.receive(&Transfer{root:candidate.encode().unwrap(),included:BTreeMap::new()},&candidate.domain,
        |_,_|panic!("失败候选已留下对象，不应补取")).unwrap_err();
    assert!(error.contains("unregistered source"),"{error}");
    assert_eq!(receiver.current().unwrap().revision,1);
}
