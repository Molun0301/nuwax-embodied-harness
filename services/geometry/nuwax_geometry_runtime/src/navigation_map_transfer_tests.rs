use super::*;
use nuwax_map_chunks::{Domain, Repository, Writer, Receiver};
fn domain() -> Domain { Domain { session: "同机发布".into() } }

#[test]
fn unchanged_roots_do_not_read_objects_and_new_domains_bootstrap_normally() {
    let repository = Repository::temporary().unwrap();
    let mut writer = Writer::new(repository.clone(), 0.05, 4*1024*1024).unwrap();
    writer.observe([[0,0,0]], 1).unwrap();
    let root = writer.seal(domain(),1,1).unwrap();
    let reference = &root.manifest().blocks[0];
    let file = repository.path().join(format!("id-{}", match reference.object { nuwax_map_chunks::ObjectIdentity::Assigned(id)=>id.0.iter().map(|v|format!("{v:02x}")).collect::<String>() }));
    std::fs::remove_file(file).unwrap();
    let transfer = prepare(&root).unwrap();
    assert_eq!(transfer.root, root.manifest().encode().unwrap());
    assert_eq!(transfer.repository_path, repository.path().to_string_lossy());
    let mut other = domain(); other.session="新域".into();
    let root = writer.seal(other,2,1).unwrap();
    assert_eq!(nuwax_map_chunks::Manifest::decode(&prepare(&root).unwrap().root).unwrap().domain.session,"新域");
}

#[test]
fn dropped_intermediate_publication_is_recovered_from_complete_root() {
    let repository = Repository::temporary().unwrap();
    let mut writer = Writer::new(repository.clone(),0.05,4*1024*1024).unwrap();
    writer.observe([[0,0,0]],1).unwrap();let first=writer.seal(domain(),1,1).unwrap();
    let mut receiver=Receiver::new(repository.read_only());
    receiver.receive(&prepare(&first).unwrap(),&domain()).unwrap();
    writer.observe([[32,0,0]],2).unwrap();writer.seal(domain(),2,2).unwrap();
    writer.observe([[64,0,0]],3).unwrap();let latest=writer.seal(domain(),3,3).unwrap();
    let root=receiver.receive(&prepare(&latest).unwrap(),&domain()).unwrap();
    assert_eq!(root.blocks,latest.manifest().blocks);
    assert_eq!(root.blocks.iter().map(|r|receiver.block(r).unwrap().len()).sum::<usize>(),3);
}
