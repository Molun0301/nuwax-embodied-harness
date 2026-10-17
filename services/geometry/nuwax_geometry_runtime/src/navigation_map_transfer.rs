//! Bounded inline delta for the existing immutable-root publication.
//!
//! Every message still carries the complete root. The retained publication is
//! only a transfer baseline; it cannot authorize an incomplete consumer map.
use nuwax_map_chunks::{Manifest, SealedMap, Transfer, MAX_BLOCK_BYTES};
use std::collections::BTreeMap;

const INLINE_BYTES: usize = 128 * 1024;

pub(super) fn prepare(fine: &SealedMap, published: Option<&Manifest>) -> Result<Transfer, String> {
    let root = fine.manifest();
    let mut transfer = Transfer {
        root: root.encode()?,
        included: BTreeMap::new(),
    };
    // An occupancy-only publication retains the exact immutable root and must
    // not read any of its objects, even when their files are temporarily busy.
    if published.is_some_and(|previous| previous.identity() == root.identity()) {
        return Ok(transfer);
    }
    let previous = published.filter(|previous| {
        previous.domain == root.domain
            && previous.voxel_size_m.to_bits() == root.voxel_size_m.to_bits()
    });
    // Both manifests are key-sorted. Merge their references without allocating
    // another index, scanning point payloads, or cloning the root's block list.
    let mut previous_blocks = previous
        .into_iter()
        .flat_map(|root| &root.blocks)
        .peekable();
    let mut included_bytes = 0;
    for block in &root.blocks {
        // Object identity includes evidence-only changes as well as geometry.
        // Unchanged spatial-prefix blocks must not crowd a changed tail block
        // out of the inline budget and force a needless reliable RPC roundtrip.
        while previous_blocks
            .peek()
            .is_some_and(|old| old.key < block.key)
        {
            previous_blocks.next();
        }
        if previous_blocks
            .peek()
            .is_some_and(|old| old.key == block.key && old.object == block.object)
        {
            continue;
        }
        if included_bytes + 16 + block.points as usize * 12 > INLINE_BYTES {
            continue;
        }
        let bytes = fine.repository.read_object(block.object, MAX_BLOCK_BYTES)?;
        if included_bytes + bytes.len() > INLINE_BYTES {
            continue;
        }
        included_bytes += bytes.len();
        transfer.included.insert(block.object, bytes);
    }
    Ok(transfer)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_map_chunks::{Domain, Receiver, Repository, Writer};

    fn domain() -> Domain {
        Domain {
            session: "incremental-transfer".into(),
            generation: 1,
            frame_generation: 1,
            producer_generation: 1,
        }
    }

    #[test]
    fn changed_tail_block_is_inline_and_cached_consumer_needs_no_rpc() {
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
        writer
            .observe(
                (0..12).flat_map(|block| {
                    (0..32).flat_map(move |x| (0..32).map(move |y| [block * 32 + x, y, 0]))
                }),
                1,
            )
            .unwrap();
        let first = writer.seal(domain(), 1, 1).unwrap();
        let initial = prepare(&first, None).unwrap();
        assert_eq!(first.manifest().blocks.len(), 12);
        assert_eq!(initial.included.len(), 10);
        let mut receiver = Receiver::new(Repository::temporary().unwrap());
        let mut initial_fetches = 0;
        receiver
            .receive(&initial, &domain(), |_, block| {
                initial_fetches += 1;
                repository.read_object(block.object, MAX_BLOCK_BYTES)
            })
            .unwrap();
        assert_eq!(initial_fetches, 2);

        // A real repeat observation changes only the final block's evidence.
        // Geometry is intentionally unchanged, so geometry_revision is not a
        // sufficient transfer key.
        writer.observe([[11 * 32, 0, 0]], 2).unwrap();
        let second = writer.seal(domain(), 2, 2).unwrap();
        let tail = second.manifest().blocks.last().unwrap();
        assert_eq!(
            first.manifest().geometry_revision,
            second.manifest().geometry_revision
        );
        assert_ne!(first.manifest().blocks.last().unwrap().object, tail.object);
        // Supplying no baseline is precisely the former prefix-only selection
        // for a different root. Exercise its unnecessary RPC as a comparison.
        let former = prepare(&second, None).unwrap();
        assert!(!former.included.contains_key(&tail.object));
        let mut former_receiver = Receiver::new(Repository::temporary().unwrap());
        former_receiver
            .receive(&initial, &domain(), |_, block| {
                repository.read_object(block.object, MAX_BLOCK_BYTES)
            })
            .unwrap();
        let mut former_fetches = 0;
        former_receiver
            .receive(&former, &domain(), |_, block| {
                former_fetches += 1;
                repository.read_object(block.object, MAX_BLOCK_BYTES)
            })
            .unwrap();
        assert_eq!(former.included.len(), 10);
        assert_eq!(former_fetches, 1);
        let incremental = prepare(&second, Some(first.manifest())).unwrap();
        assert_eq!(incremental.included.len(), 1);
        assert!(incremental.included.contains_key(&tail.object));
        assert!(incremental.included.values().map(Vec::len).sum::<usize>() <= INLINE_BYTES);
        let current = receiver
            .receive(&incremental, &domain(), |_, _| {
                panic!("retained consumer must already own every unchanged object")
            })
            .unwrap();
        assert_eq!(&*current, &**second.manifest());
        assert_eq!(
            receiver.block(tail).unwrap().get(&0).unwrap().observed_ns,
            2
        );

        // A late subscriber still receives a complete root and can fetch exact
        // missing members. Inline delta is never treated as a full snapshot.
        let mut late = Receiver::new(Repository::temporary().unwrap());
        let mut late_fetches = 0;
        let current = late
            .receive(&incremental, &domain(), |_, block| {
                late_fetches += 1;
                repository.read_object(block.object, MAX_BLOCK_BYTES)
            })
            .unwrap();
        assert_eq!(late_fetches, 11);
        assert_eq!(&*current, &**second.manifest());
        eprintln!("map_transfer blocks=12 initial_inline={} initial_rpc={} former_inline={} former_rpc={} delta_inline={} delta_bytes={} cached_rpc=0 late_rpc={}",
            initial.included.len(), initial_fetches, former.included.len(), former_fetches, incremental.included.len(),
            incremental.included.values().map(Vec::len).sum::<usize>(), late_fetches);
    }

    #[test]
    fn unchanged_roots_do_not_read_objects_and_new_domains_bootstrap_normally() {
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
        writer.observe([[0, 0, 0]], 1).unwrap();
        let first = writer.seal(domain(), 1, 1).unwrap();
        let moved = repository.path().with_extension("transfer-unavailable");
        std::fs::rename(repository.path(), &moved).unwrap();
        let repeated = prepare(&first, Some(first.manifest()));
        std::fs::rename(&moved, repository.path()).unwrap();
        assert!(repeated.unwrap().included.is_empty());
        let rehomed = first.rehome_frame(&first.manifest().identity(), 2).unwrap();
        let transfer = prepare(&rehomed, Some(first.manifest())).unwrap();
        assert_eq!(
            transfer.included.len(),
            1,
            "new frame has no transfer baseline"
        );
        assert_eq!(
            Manifest::decode(&transfer.root).unwrap(),
            **rehomed.manifest()
        );
    }

    #[test]
    fn dropped_intermediate_publication_is_recovered_from_complete_root() {
        let repository = Repository::temporary().unwrap();
        let mut writer = Writer::new(repository.clone(), 0.05, 4 * 1024 * 1024).unwrap();
        writer.observe([[0, 0, 0], [32, 0, 0]], 1).unwrap();
        let first = writer.seal(domain(), 1, 1).unwrap();
        let mut receiver = Receiver::new(Repository::temporary().unwrap());
        receiver
            .receive(&prepare(&first, None).unwrap(), &domain(), |_, _| {
                panic!("initial small map is fully inline")
            })
            .unwrap();
        writer.observe([[0, 0, 0]], 2).unwrap();
        let missed = writer.seal(domain(), 2, 2).unwrap();
        writer.observe([[32, 0, 0]], 3).unwrap();
        let latest = writer.seal(domain(), 3, 3).unwrap();
        let transfer = prepare(&latest, Some(missed.manifest())).unwrap();
        assert_eq!(transfer.included.len(), 1);
        let mut fetched = 0;
        let admitted = receiver
            .receive(&transfer, &domain(), |root, block| {
                assert_eq!(root.identity(), latest.manifest().identity());
                fetched += 1;
                repository.read_object(block.object, MAX_BLOCK_BYTES)
            })
            .unwrap();
        assert_eq!(fetched, 1);
        assert_eq!(&*admitted, &**latest.manifest());
        assert_eq!(
            receiver.block(&admitted.blocks[0]).unwrap()[&0].observed_ns,
            2
        );
        assert_eq!(
            receiver.block(&admitted.blocks[1]).unwrap()[&0].observed_ns,
            3
        );
    }
}
