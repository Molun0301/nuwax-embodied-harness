//! 同机导航只携带完整根及所属对象目录，不复制块载荷。
use nuwax_map_chunks::{SealedMap, Transfer};

pub(super) fn prepare(fine: &SealedMap) -> Result<Transfer, String> {
    Ok(Transfer { root: fine.manifest().encode()?, repository_path: fine.repository.path().to_string_lossy().into_owned() })
}

#[cfg(test)]
#[path = "navigation_map_transfer_tests.rs"]
mod tests;
