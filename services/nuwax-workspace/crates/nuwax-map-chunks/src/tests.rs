use super::*;
use std::io::{Cursor, Read};
const BUDGET: usize = 4 * 1024 * 1024;
fn domain() -> Domain { Domain { session: "robot-owner".into(), generation: 1, frame_generation: 0, producer_generation: 2 } }
fn writer() -> (Repository, Writer) { let r = Repository::temporary().unwrap(); let w = Writer::new(r.clone(), 0.125, BUDGET).unwrap(); (r,w) }
fn one() -> (Repository, Writer, Arc<Manifest>) { let (r,mut w)=writer(); w.observe([[0,0,0]],1).unwrap(); let m=w.snapshot(domain(),1,1).unwrap(); (r,w,m) }
fn transfer(root: &Manifest, source: &Repository) -> Transfer {
    Transfer { root: root.encode().unwrap(), included: root.blocks.iter().map(|b| (b.object,source.read_object(b.object,MAX_BLOCK_BYTES).unwrap())).collect() }
}
fn receive(receiver: &mut Receiver, root: &Manifest, source: &Repository) -> Result<Arc<Manifest>,String> {
    receiver.receive(&transfer(root, source), &root.domain, |_,_| panic!("unexpected fetch"))
}
fn body(local: u16, stamp: u64, hits: u16) -> Vec<u8> { encode_block(&BTreeMap::from([(local,Evidence {observed_ns:stamp,hits})])).unwrap() }
fn archive_bytes(root: &Manifest, bodies: &[Vec<u8>]) -> Vec<u8> {
    let json=root.encode().unwrap(); let mut out=b"NUWAXAR2".to_vec(); out.extend((json.len() as u64).to_le_bytes()); out.extend(json);
    out.extend((bodies.len() as u64).to_le_bytes()); for b in bodies {out.extend((b.len() as u32).to_le_bytes());out.extend(b);} out
}
fn legacy_bytes(stamp: u64, bodies: &[Vec<u8>], shared: bool) -> Vec<u8> {
    let refs:Vec<_>=bodies.iter().enumerate().map(|(i,_)|LegacyBlockRef{key:[i as i32,0,0],sha256:LegacyAlias([if shared {5} else {5+i as u8};32]),geometry_sha256:LegacyAlias([9;32]),points:1}).collect();
    let root=LegacyManifest{version:1,domain:domain(),revision:4,source_timestamp_ns:stamp,voxel_size_m:0.125,blocks:refs};
    let json=serde_json::to_vec_pretty(&root).unwrap(); let mut out=b"NUWAXAR1".to_vec();out.extend((json.len() as u64).to_le_bytes());out.extend([7;32]);out.extend(json);
    out.extend((bodies.len() as u64).to_le_bytes());for b in bodies {out.extend((b.len() as u32).to_le_bytes());out.extend(b);}out
}
#[test]
fn block_roundtrip_negative_coordinates_and_evidence() {
    for key in [[-1,-1,-1],[-32,31,32],[i32::MIN,0,i32::MAX]] { assert_eq!(global_key(block_key(key),local_key(key)).unwrap(),key); }
    let bytes=body(32767,u64::MAX,u16::MAX);assert_eq!(encode_block(&decode_block(&bytes).unwrap()).unwrap(),bytes);
    assert!(encode_block(&BTreeMap::from([(0,Evidence{observed_ns:0,hits:1})])).is_err());
    let mut bad=bytes.clone();bad.push(0);assert!(decode_block(&bad).is_err());
    let mut bad=bytes;bad[12]=1;assert!(decode_block(&bad).is_err());
}
#[test]
fn identity_json_namespace_and_no_content_dedup() {
    let (r,mut w)=writer();w.observe([[0,0,0],[32,0,0]],1).unwrap();let m=w.snapshot(domain(),1,1).unwrap();
    assert_ne!(m.blocks[0].object,m.blocks[1].object);assert_eq!(r.read_object(m.blocks[0].object,MAX_BLOCK_BYTES).unwrap(),r.read_object(m.blocks[1].object,MAX_BLOCK_BYTES).unwrap());
    let json=serde_json::to_value(&m.blocks[0].object).unwrap();assert_eq!(json["kind"],"assigned");assert_eq!(json["value"].as_array().unwrap().len(),16);
    assert!(ObjectIdentity::Assigned(ObjectId([0;16])).validate().is_err());assert!(ObjectIdentity::Legacy(LegacyAlias([0;32])).validate().is_err());
    assert!(std::fs::read_dir(r.path()).unwrap().all(|e|e.unwrap().file_name().to_str().unwrap().starts_with("id-")));
}
#[test]
fn manifest_rejects_invalid_revisions_and_shared_intrinsic_conflicts() {
    let (_,_,m)=one();let mut bad=(*m).clone();bad.blocks[0].geometry_revision=2;assert!(bad.validate().is_err());
    let mut bad=(*m).clone();let mut b=bad.blocks[0].clone();b.key=[1,0,0];b.points=2;bad.blocks.push(b);assert!(bad.validate().is_err());
    let mut bad=(*m).clone();bad.blocks[0].key=[i32::MAX,0,0];assert!(bad.validate().is_err());
    let mut value=serde_json::to_value(&*m).unwrap();value["unexpected"]=1.into();assert!(Manifest::decode(&serde_json::to_vec(&value).unwrap()).is_err());
}
#[test]
fn noop_seal_and_flush_do_zero_object_work() {
    let (r,mut w,m)=one();let counts=r.counts();w.flush().unwrap();let next=w.snapshot(domain(),2,1).unwrap();
    assert_eq!(counts,r.counts());assert_eq!(m.blocks,next.blocks);assert_eq!(m.geometry_revision,next.geometry_revision);
    w.observe([[0,0,0]],1).unwrap();let before=r.counts();let again=w.snapshot(domain(),3,1).unwrap();assert_eq!(before,r.counts());assert_eq!(m.blocks,again.blocks);
}
#[test]
fn evidence_change_advances_only_changed_block_revision() {
    let (r,mut w)=writer();w.observe([[0,0,0],[3200,0,0]],1).unwrap();let old=w.snapshot(domain(),1,1).unwrap();
    w.observe([[0,0,0]],2).unwrap();let before=r.counts();let new=w.snapshot(domain(),2,2).unwrap();
    assert_eq!(r.counts().1-before.1,1);assert_eq!(new.geometry_revision,old.geometry_revision);assert_eq!(new.blocks[1],old.blocks[1]);
    assert_ne!(new.blocks[0].object,old.blocks[0].object);assert_eq!(new.blocks[0].block_revision,2);assert_eq!(new.blocks[0].geometry_revision,1);
}
#[test]
fn net_membership_survives_pressure_flush_without_baseline_payloads() {
    let (r,mut w,m)=one();w.remove([[0,0,0]]).unwrap();w.flush().unwrap();w.collect_sealed(&[]).unwrap();
    assert!(r.read_object(m.blocks[0].object,MAX_BLOCK_BYTES).is_ok());w.observe([[0,0,0]],2).unwrap();w.flush().unwrap();
    let same=w.snapshot(domain(),2,2).unwrap();assert_eq!(same.geometry_revision,1);assert_eq!(same.blocks[0].geometry_revision,1);
    w.remove([[0,0,0]]).unwrap();let empty=w.snapshot(domain(),3,2).unwrap();assert!(empty.blocks.is_empty());assert_eq!(empty.geometry_revision,3);
}
#[test]
fn partial_flush_keeps_ids_retry_and_actual_edit_allocates_new_id() {
    let r=Repository::temporary().unwrap();let mut w=Writer::new(r.clone(),0.125,BUDGET*2).unwrap();w.observe([[0,0,0],[32,0,0]],1).unwrap();r.fail_put_after(Some(1));assert!(w.flush().is_err());
    let ids=w.pending_ids();assert_eq!(ids.len(),2);w.collect_sealed(&[]).unwrap();assert!(r.read_object(ids[0],MAX_BLOCK_BYTES).is_ok());
    let encoded=w.encoded_blocks;assert!(w.flush().is_err());assert_eq!(w.pending_ids(),ids);assert_eq!(w.encoded_blocks,encoded);
    w.observe([[0,0,0]],2).unwrap();r.fail_put_after(None);let root=w.snapshot(domain(),1,2).unwrap();
    assert_ne!(root.blocks[0].object,ids[0]);assert_eq!(root.blocks[1].object,ids[1]);assert_eq!(decode_block(&r.read_object(ids[0],MAX_BLOCK_BYTES).unwrap()).unwrap()[&0].observed_ns,1);
}
#[test]
fn successful_retry_preserves_all_pending_ids() {
    let r=Repository::temporary().unwrap();let mut w=Writer::new(r.clone(),0.125,BUDGET*2).unwrap();w.observe([[0,0,0],[32,0,0]],1).unwrap();r.fail_put_after(Some(1));assert!(w.flush().is_err());let ids=w.pending_ids();
    r.fail_put_after(None);let m=w.snapshot(domain(),1,1).unwrap();assert_eq!(m.blocks.iter().map(|b|b.object).collect::<Vec<_>>(),ids);
}
#[test]
fn restore_same_domain_preserves_revisions_new_domain_restamps() {
    let (r,_,m)=one();let mut w=Writer::new(r.clone(),0.125,BUDGET).unwrap();w.restore(&m).unwrap();assert!(w.snapshot(domain(),1,1).is_err());
    let next=w.snapshot(domain(),2,1).unwrap();assert_eq!(m.blocks,next.blocks);assert_eq!(next.geometry_revision,1);
    let mut other=domain();other.frame_generation=1;let moved=w.snapshot(other,1,1).unwrap();assert_eq!(m.blocks[0].object,moved.blocks[0].object);assert_eq!(moved.geometry_revision,1);
}
#[test]
fn source_bound_can_decrease_but_cannot_precede_evidence() {
    let (r,mut w)=writer();w.observe([[0,0,0]],1).unwrap();let first=w.snapshot(domain(),1,10).unwrap();let next=w.snapshot(domain(),2,5).unwrap();
    let mut recv=Receiver::new(Repository::temporary().unwrap());receive(&mut recv,&first,&r).unwrap();receive(&mut recv,&next,&r).unwrap();
    w.observe([[0,0,0]],8).unwrap();assert!(w.snapshot(domain(),3,7).is_err());
}
#[test]
fn immutable_install_conflict_never_overwrites() {
    let r=Repository::temporary().unwrap();let id=ObjectIdentity::Assigned(ObjectId([3;16]));let old=body(0,1,1);let new=body(0,2,2);
    r.put_object(id,&old).unwrap();r.put_object(id,&old).unwrap();assert!(r.put_object(id,&new).is_err());assert_eq!(r.read_object(id,MAX_BLOCK_BYTES).unwrap(),old);
}
#[cfg(unix)]
#[test]
fn repository_rejects_symlink_and_directory_without_overwrite() {
    let r=Repository::temporary().unwrap();let alias=ObjectIdentity::Legacy(LegacyAlias([3;32]));let path=r.path().join("03".repeat(32));
    let target=r.path().join("outside");std::fs::write(&target,b"unchanged").unwrap();std::os::unix::fs::symlink(&target,&path).unwrap();
    assert!(r.read_object(alias,100).is_err());assert!(r.put_object(alias,b"different").is_err());assert_eq!(std::fs::read(&target).unwrap(),b"unchanged");
    std::fs::remove_file(&path).unwrap();std::fs::create_dir(&path).unwrap();assert!(r.read_object(alias,100).is_err());assert!(r.put_object(alias,b"different").is_err());
}
#[test]
fn legacy_alias_paths_and_shared_spatial_keys_are_retained() {
    let bytes=legacy_bytes(1,&[body(0,1,1),body(0,1,1)],true);let mut a=ArchiveReader::open(Cursor::new(bytes)).unwrap();let r=Repository::temporary().unwrap();let root=a.import(&r).unwrap();
    assert!(r.path().join("05".repeat(32)).is_file());let mut w=Writer::new(r.clone(),0.125,BUDGET).unwrap();w.restore_archived(&root).unwrap();assert!(w.snapshot(domain(),4,1).is_err());
    let current=w.snapshot(domain(),5,1).unwrap();assert_eq!(current.blocks[0].object,current.blocks[1].object);assert_eq!(current.blocks[0].geometry_revision,5);
    let mut cache=RegionReader::new(r.clone(),BUDGET);cache.block(&current.blocks[0]).unwrap();let count=r.counts();cache.block(&current.blocks[1]).unwrap();assert_eq!(count,r.counts());
}
#[test]
fn cached_block_hit_does_not_reopen_or_rescan_members() {
    let (r,_,m)=one();let mut cache=RegionReader::new(r.clone(),BUDGET);let first=cache.block(&m.blocks[0]).unwrap();let counts=r.counts();
    let mut rehome=m.blocks[0].clone();rehome.key=[50,0,0];rehome.geometry_revision=99;let again=cache.block(&rehome).unwrap();assert!(Arc::ptr_eq(&first,&again));assert_eq!(counts,r.counts());
}
#[test]
fn duplicate_root_zero_work_and_included_conflict_is_admitted() {
    let (r,_,m)=one();let dst=Repository::temporary().unwrap();let mut recv=Receiver::new(dst.clone());let first=receive(&mut recv,&m,&r).unwrap();let counts=dst.counts();
    let empty=Transfer{root:m.encode().unwrap(),included:BTreeMap::new()};let same=recv.receive(&empty,&domain(),|_,_|panic!("fetch")).unwrap();assert!(Arc::ptr_eq(&first,&same));assert_eq!(counts,dst.counts());
    let mut bad=transfer(&m,&r);bad.included.insert(m.blocks[0].object,body(1,1,1));assert!(recv.receive(&bad,&domain(),|_,_|panic!("fetch")).is_err());assert!(Arc::ptr_eq(&first,&recv.current().unwrap()));
}
#[test]
fn failed_fetch_preserves_current_root_and_retained_old_reader() {
    let (src,mut w,one)=one();let dst=Repository::temporary().unwrap();let mut recv=Receiver::new(dst.clone());let pin=receive(&mut recv,&one,&src).unwrap();
    w.observe([[32,0,0]],2).unwrap();let two=w.snapshot(domain(),2,2).unwrap();let incomplete=Transfer{root:two.encode().unwrap(),included:BTreeMap::new()};
    assert!(recv.receive(&incomplete,&domain(),|_,_|Err("offline".into())).is_err());assert_eq!(recv.current().unwrap().revision,1);
    receive(&mut recv,&two,&src).unwrap();w.remove([[0,0,0]]).unwrap();let three=w.snapshot(domain(),3,2).unwrap();receive(&mut recv,&three,&src).unwrap();
    assert!(dst.read_object(pin.blocks[0].object,MAX_BLOCK_BYTES).is_ok());drop(pin);drop(one);
    let four=w.snapshot(domain(),4,2).unwrap();receive(&mut recv,&four,&src).unwrap();assert!(dst.read_object(two.blocks[0].object,MAX_BLOCK_BYTES).is_err());
}
#[test]
fn receiver_fetches_once_and_keeps_intrinsic_metadata() {
    let (src,_,m)=one();let dst=Repository::temporary().unwrap();let mut recv=Receiver::new(dst.clone());let mut calls=0;
    recv.receive(&Transfer{root:m.encode().unwrap(),included:BTreeMap::new()},&domain(),|root,b|{assert_eq!(root.identity(),m.identity());calls+=1;src.read_object(b.object,MAX_BLOCK_BYTES)}).unwrap();assert_eq!(calls,1);
    let mut next=(*m).clone();next.revision=2;let count=dst.counts();recv.receive(&Transfer{root:next.encode().unwrap(),included:BTreeMap::new()},&domain(),|_,_|panic!("fetch")).unwrap();assert_eq!(count.0,dst.counts().0);
}
#[test]
fn receiver_rejects_false_geometry_and_block_revision_claims() {
    let (src,mut w,m)=one();let mut recv=Receiver::new(Repository::temporary().unwrap());receive(&mut recv,&m,&src).unwrap();w.observe([[1,0,0]],2).unwrap();let next=w.snapshot(domain(),2,2).unwrap();
    let mut bad=(*next).clone();bad.geometry_revision=1;bad.blocks[0].geometry_revision=1;assert!(receive(&mut recv,&bad,&src).is_err());
    let mut bad=(*next).clone();bad.blocks[0].block_revision=1;bad.blocks[0].geometry_revision=1;assert!(receive(&mut recv,&bad,&src).is_err());receive(&mut recv,&next,&src).unwrap();
}
#[test]
fn receiver_removal_needs_geometry_advance_and_resolution_change_is_allowed() {
    let (src,_,m)=one();let mut recv=Receiver::new(Repository::temporary().unwrap());receive(&mut recv,&m,&src).unwrap();
    let mut next=(*m).clone();next.revision=2;next.voxel_size_m=0.25;receive(&mut recv,&next,&src).unwrap();assert_eq!(next.changed_from(&m),vec![[0,0,0]]);
    let mut empty=next.clone();empty.revision=3;empty.blocks.clear();assert!(receive(&mut recv,&empty,&src).is_err());empty.geometry_revision=3;receive(&mut recv,&empty,&src).unwrap();
}
#[test]
fn receiver_rejects_future_evidence_domain_mismatch_and_extra_inclusions() {
    let (src,mut w,_)=one();w.observe([[0,0,0]],3).unwrap();let m=w.snapshot(domain(),2,3).unwrap();let mut bad=(*m).clone();bad.source_timestamp_ns=2;
    let mut recv=Receiver::new(Repository::temporary().unwrap());assert!(receive(&mut recv,&bad,&src).is_err());let mut other=domain();other.frame_generation=1;assert!(recv.receive(&transfer(&m,&src),&other,|_,_|panic!()).is_err());
    let mut t=transfer(&m,&src);t.included.insert(ObjectIdentity::Assigned(ObjectId([9;16])),body(0,1,1));assert!(recv.receive(&t,&domain(),|_,_|panic!()).is_err());
}
#[test]
fn archive_ar2_stream_roundtrip_and_exact_reference_binding() {
    let (r,_,m)=one();let (mut file,size)=archive_file(&m,&r).unwrap();let mut bytes=Vec::new();file.read_to_end(&mut bytes).unwrap();assert_eq!(bytes.len() as u64,size);assert_eq!(&bytes[..8],b"NUWAXAR2");
    let mut reader=ArchiveReader::open(Cursor::new(bytes)).unwrap();assert_eq!(reader.root(),&ArchivedRoot::Current((*m).clone()));let mut bad=m.blocks[0].clone();bad.block_revision+=1;assert!(reader.block(&bad).is_err());
    let target=Repository::temporary().unwrap();reader.import(&target).unwrap();assert_eq!(target.read_object(m.blocks[0].object,MAX_BLOCK_BYTES).unwrap(),r.read_object(m.blocks[0].object,MAX_BLOCK_BYTES).unwrap());
}
#[test]
fn archive_ar1_preserves_literal_json_and_opaque_aliases() {
    let bytes=legacy_bytes(1,&[body(0,1,1)],false);let size=u64::from_le_bytes(bytes[8..16].try_into().unwrap()) as usize;
    let reader=ArchiveReader::open(Cursor::new(bytes.clone())).unwrap();let ArchivedRoot::Legacy(root)=reader.root() else{panic!()};assert_eq!(root.raw_json,bytes[48..48+size]);assert_eq!(root.alias,LegacyAlias([7;32]));
}
#[test]
fn both_archive_versions_reject_truncation_trailing_counts_and_evidence() {
    let (_,_,m)=one();for bytes in [archive_bytes(&m,&[body(0,1,1)]),legacy_bytes(1,&[body(0,1,1)],false)] {
        let mut bad=bytes.clone();bad.pop();assert!(ArchiveReader::open(Cursor::new(bad)).is_err());let mut bad=bytes.clone();bad.push(0);assert!(ArchiveReader::open(Cursor::new(bad)).is_err());
        let offset=if &bytes[..8]==b"NUWAXAR1"{48}else{16};let json=u64::from_le_bytes(bytes[8..16].try_into().unwrap()) as usize;
        let mut bad=bytes.clone();bad[offset+json..offset+json+8].copy_from_slice(&2u64.to_le_bytes());assert!(ArchiveReader::open(Cursor::new(bad)).is_err());
        let mut bad=bytes;let stamp=offset+json+8+4+16+2;bad[stamp..stamp+8].copy_from_slice(&2u64.to_le_bytes());assert!(ArchiveReader::open(Cursor::new(bad)).unwrap().block_by_key([0,0,0]).is_err());
    }
}
#[test]
fn both_archive_versions_reject_shared_id_different_same_length_bytes() {
    let (_,_,m)=one();let mut root=(*m).clone();let mut b=root.blocks[0].clone();b.key=[1,0,0];root.blocks.push(b);
    for changed in [body(1,1,1),body(0,1,2)] {
        let mut archive=ArchiveReader::open(Cursor::new(archive_bytes(&root,&[body(0,1,1),changed.clone()]))).unwrap();
        let repository=Repository::temporary().unwrap();assert!(archive.import(&repository).is_err());assert_eq!(repository.read_object(root.blocks[0].object,MAX_BLOCK_BYTES).unwrap(),body(0,1,1));
        assert!(ArchiveReader::open(Cursor::new(legacy_bytes(1,&[body(0,1,1),changed],true))).unwrap().block_by_key([1,0,0]).is_err());
    }
    assert!(ArchiveReader::open(Cursor::new(archive_bytes(&root,&[body(0,1,1),body(0,1,1)]))).is_ok());
}
#[test]
fn archive_import_existing_conflict_preserves_first_bytes() {
    let (_,_,m)=one();let bytes=archive_bytes(&m,&[body(0,1,1)]);let mut reader=ArchiveReader::open(Cursor::new(bytes)).unwrap();let repo=Repository::temporary().unwrap();let old=body(1,1,1);repo.put_object(m.blocks[0].object,&old).unwrap();
    assert!(reader.import(&repo).is_err());assert_eq!(repo.read_object(m.blocks[0].object,MAX_BLOCK_BYTES).unwrap(),old);
}
#[test]
fn local_region_skips_far_objects_and_enforces_budget() {
    let (r,mut w)=writer();w.observe([[-1,0,0],[0,0,0],[32000,0,0]],1).unwrap();let root=w.snapshot(domain(),1,1).unwrap();let mut reader=RegionReader::new(r.clone(),BUDGET);let n=r.counts().0;
    assert_eq!(reader.region(&root,[-2,-2,-2],[2,2,2],2).unwrap().len(),2);assert_eq!(r.counts().0-n,2);assert!(reader.region(&root,[-2,-2,-2],[2,2,2],1).is_err());
}
#[test]
fn many_membership_edits_remain_bounded_and_gc_pins_baseline() {
    let (_,mut w)=writer();
    for x in 0..64 {w.seed_unknown((0..32768).map(|local|global_key([x,0,0],local).unwrap())).unwrap();w.flush().unwrap();}
    w.snapshot(domain(),1,1).unwrap();
    for x in 0..64 {w.remove([[x*32,0,0]]).unwrap();w.flush().unwrap();w.collect_sealed(&[]).unwrap();assert!(w.resident_bytes()<=BUDGET);}
    let root=w.snapshot(domain(),2,1).unwrap();assert_eq!(root.geometry_revision,2);assert_eq!(root.points(),64*32767);
}
#[test]
fn literal_cross_language_ar1_fixture() {
    let mut reader=ArchiveReader::open(Cursor::new(include_bytes!("fixtures/python-ar1-literal.bin").as_slice())).unwrap();
    assert_eq!(reader.root().points(),2);let points=reader.region([-32,0,32],[-1,31,63],2).unwrap();assert_eq!(points.len(),2);assert_eq!(points[1].1.observed_ns,u64::MAX);assert_eq!(points[1].1.hits,u16::MAX);
    assert_eq!(reader.root().revision(),9_007_199_254_740_993);
    reader.import(&Repository::temporary().unwrap()).unwrap();
}
#[test]
fn literal_cross_language_ar2_fixture() {
    let mut reader=ArchiveReader::open(Cursor::new(include_bytes!("fixtures/python-ar2-literal.bin").as_slice())).unwrap();
    assert_eq!(reader.root().points(),2);let points=reader.region([-32,0,32],[-1,31,63],2).unwrap();assert_eq!(points.len(),2);assert_eq!(points[1].1.observed_ns,u64::MAX);assert_eq!(points[1].1.hits,u16::MAX);
    assert_eq!(reader.root().revision(),9_007_199_254_740_993);
    reader.import(&Repository::temporary().unwrap()).unwrap();
}

#[test]
fn archive_open_is_framing_only_until_a_body_is_consumed() {
    struct Counted { cursor: Cursor<Vec<u8>>, reads: std::rc::Rc<std::cell::Cell<usize>> }
    impl std::io::Read for Counted {
        fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
            let size = self.cursor.read(out)?; self.reads.set(self.reads.get() + size); Ok(size)
        }
    }
    impl std::io::Seek for Counted {
        fn seek(&mut self, position: std::io::SeekFrom) -> std::io::Result<u64> { std::io::Seek::seek(&mut self.cursor,position) }
    }
    let (_,_,m)=one();let mut root=(*m).clone();let mut second=root.blocks[0].clone();second.key=[1,0,0];root.blocks.push(second);
    let bytes=archive_bytes(&root,&[body(0,1,1),body(0,1,1)]);let reads=std::rc::Rc::new(std::cell::Cell::new(0));
    let mut reader=ArchiveReader::open(Counted{cursor:Cursor::new(bytes),reads:reads.clone()}).unwrap();
    assert_eq!(reads.get(),24+root.encode().unwrap().len()+8);
    let before=reads.get();reader.block_by_key([1,0,0]).unwrap();assert_eq!(reads.get()-before,56);
    let before=reads.get();reader.block_by_key([1,0,0]).unwrap();assert_eq!(reads.get()-before,28);
    let mut bytes=archive_bytes(&m,&[body(0,1,1)]);let start=bytes.len()-28;bytes[start]=0;
    let mut reader=ArchiveReader::open(Cursor::new(bytes)).unwrap();assert!(reader.block_by_key([0,0,0]).is_err());
}

#[test]
fn receiver_gc_releases_cached_payloads_outside_live_roots() {
    let (source,_,root)=one();let target=Repository::temporary().unwrap();let mut receiver=Receiver::new(target.clone());receive(&mut receiver,&root,&source).unwrap();
    assert!(!receiver.reader.cache_is_empty());
    let mut empty=(*root).clone();empty.revision=2;empty.geometry_revision=2;empty.blocks.clear();receive(&mut receiver,&empty,&source).unwrap();
    assert!(receiver.reader.cache_is_empty());assert!(target.read_object(root.blocks[0].object,MAX_BLOCK_BYTES).is_err());
}

#[test]
fn injectable_id_source_is_preserved_and_zero_ids_are_rejected() {
    let repository=Repository::temporary().unwrap();let mut next=40u8;
    let mut writer=Writer::with_id_source(repository.clone(),0.125,BUDGET,move||{next+=1;Ok(ObjectId([next;16]))}).unwrap();
    writer.observe([[0,0,0]],1).unwrap();let root=writer.snapshot(domain(),1,1).unwrap();assert_eq!(root.blocks[0].object,ObjectId([41;16]).into());
    let mut writer=Writer::with_id_source(repository,0.125,BUDGET,||Ok(ObjectId([0;16]))).unwrap();writer.observe([[0,0,0]],1).unwrap();assert!(writer.flush().is_err());
}

#[test]
fn cleanup_failure_does_not_install_candidate_root() {
    let (source, mut writer, first) = one();
    let destination = Repository::temporary().unwrap();
    let mut receiver = Receiver::new(destination.clone());
    let pinned = receive(&mut receiver, &first, &source).unwrap();
    writer.observe([[32, 0, 0]], 2).unwrap();
    let next = writer.snapshot(domain(), 2, 2).unwrap();
    // A malformed unreferenced object entry deterministically fails collection.
    let obstruction = destination.path().join(format!("id-{}", "ff".repeat(16)));
    std::fs::create_dir(&obstruction).unwrap();
    assert!(receive(&mut receiver, &next, &source).is_err());
    assert!(Arc::ptr_eq(&pinned, &receiver.current().unwrap()));
    assert!(destination.read_object(pinned.blocks[0].object, MAX_BLOCK_BYTES).is_ok());
    std::fs::remove_dir(obstruction).unwrap();
    assert_eq!(receive(&mut receiver, &next, &source).unwrap().revision, 2);
}
