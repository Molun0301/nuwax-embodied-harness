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
#[path = "navigation_map_transfer_tests.rs"]
mod tests;
