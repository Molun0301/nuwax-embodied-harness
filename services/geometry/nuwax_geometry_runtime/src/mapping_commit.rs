use crate::mapping_backend::MappingBackendKind;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct MappingCommitId {
    pub(crate) boot_nonce: u128,
    pub(crate) sequence: u64,
}

#[allow(dead_code)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct OccupancyCommitReceipt {
    pub(crate) commit_id: MappingCommitId,
    pub(crate) session_id: String,
    pub(crate) session_generation: u64,
    pub(crate) occupancy_revision: u64,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) backend_kind: MappingBackendKind,
    pub(crate) backend_generation: u64,
}

#[allow(dead_code)]
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct CloudCommitReceipt {
    pub(crate) commit_id: MappingCommitId,
    pub(crate) session_generation: u64,
    pub(crate) occupancy_revision: u64,
    pub(crate) cloud_revision: u64,
    pub(crate) source_timestamp_ns: u64,
    pub(crate) point_count: u32,
}

#[derive(Debug)]
pub(crate) struct MappingCommitClock {
    boot_nonce: u128,
    next_sequence: u64,
}

impl MappingCommitClock {
    pub(crate) fn new() -> Self {
        Self {
            boot_nonce: uuid::Uuid::new_v4().as_u128(),
            next_sequence: 0,
        }
    }

    pub(crate) fn reserve(&mut self) -> Result<MappingCommitId, String> {
        let sequence = self
            .next_sequence
            .checked_add(1)
            .ok_or_else(|| "mapping commit sequence exhausted".to_owned())?;
        self.next_sequence = sequence;
        Ok(MappingCommitId {
            boot_nonce: self.boot_nonce,
            sequence,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn commit_clock_is_boot_scoped_and_positive() {
        let mut clock = MappingCommitClock::new();
        let first = clock.reserve().expect("first commit id");
        let second = clock.reserve().expect("second commit id");
        assert_ne!(first.boot_nonce, 0);
        assert_eq!(first.boot_nonce, second.boot_nonce);
        assert_eq!(first.sequence, 1);
        assert_eq!(second.sequence, 2);
    }
}
