#![allow(dead_code)]

//! OccupancyGrid process-shared mmap sidecar.
//!
//! Layout v2 uses two payload slots and a process-shared atomic publication header. A writer
//! marks the global write sequence odd before touching the inactive slot, verifies and flushes
//! that slot, atomically publishes `(generation, active_slot)`, then marks the sequence even.
//! Readers only accept a snapshot when the sequence is stable/even before and after copying and
//! the slot checksum matches. The active slot is never modified in place.

use std::io::{Error, ErrorKind};
use std::sync::atomic::{fence, AtomicU64, Ordering};

use memmap2::{Mmap, MmapMut};
use tracing::info;

use crate::lcm_runtime::MapState;
use crate::pointcloud::{
    capture_product_owned_mmap, unlink_product_owned_mmap, ProductOwnedMmapIdentity,
};

#[cfg(not(target_has_atomic = "64"))]
compile_error!("occupancy SHM v2 requires lock-free 64-bit atomics");

const OCC_SHM_MAGIC: u64 = 0x4E55_5741_584F_4343;
pub const OCCUPANCY_SHM_LAYOUT_VERSION: u32 = 2;
pub const OCCUPANCY_SHM_PAYLOAD_SCHEMA_VERSION: u32 = 1;
const OCC_INITIAL_CAPACITY: usize = 16 * 1024 * 1024;
const OCC_GROW_FACTOR: usize = 2;
const SLOT_COUNT: usize = 2;

// Global header. Atomic u64 fields are page-base + 8-byte aligned.
const GLOBAL_HEADER_BYTES: usize = 128;
const GLOBAL_MAGIC_OFFSET: usize = 0;
const GLOBAL_LAYOUT_VERSION_OFFSET: usize = 8;
const GLOBAL_HEADER_BYTES_OFFSET: usize = 12;
const GLOBAL_SLOT_CAPACITY_OFFSET: usize = 16;
const GLOBAL_WRITE_SEQUENCE_OFFSET: usize = 24;
const GLOBAL_PUBLISHED_STATE_OFFSET: usize = 32;

// Fixed per-slot header.
const SLOT_MAGIC: u64 = 0x4E55_5741_5853_4C54;
const SLOT_HEADER_BYTES: usize = 256;
const SLOT_FRAME_ID_MAX_BYTES: usize = 128;
const SLOT_MAGIC_OFFSET: usize = 0;
const SLOT_LAYOUT_VERSION_OFFSET: usize = 8;
const SLOT_SCHEMA_VERSION_OFFSET: usize = 12;
const SLOT_HEADER_BYTES_OFFSET: usize = 16;
const SLOT_FRAME_ID_LEN_OFFSET: usize = 20;
const SLOT_GENERATION_OFFSET: usize = 24;
const SLOT_REVISION_OFFSET: usize = 32;
const SLOT_TIMESTAMP_OFFSET: usize = 40;
const SLOT_PAYLOAD_LEN_OFFSET: usize = 48;
const SLOT_WIDTH_OFFSET: usize = 56;
const SLOT_HEIGHT_OFFSET: usize = 60;
const SLOT_RESOLUTION_OFFSET: usize = 64;
const SLOT_ORIGIN_X_OFFSET: usize = 72;
const SLOT_ORIGIN_Y_OFFSET: usize = 80;
const SLOT_CHECKSUM_OFFSET: usize = 88;
const SLOT_FRAME_ID_OFFSET: usize = 120;

#[derive(Clone, Debug, PartialEq, Eq)]
struct MapWriteIdentity {
    width: u32,
    height: u32,
    resolution_bits: u32,
    origin_x_bits: u64,
    origin_y_bits: u64,
    frame_id: String,
    revision: u64,
    timestamp_ns: u64,
    data_len: usize,
    data_checksum: [u8; 32],
}

/// Opaque proof that one complete occupancy payload was flushed into an
/// inactive slot without changing the reader-visible global header.
///
/// The token is deliberately non-`Clone`: dropping it aborts publication.
/// `commit_prepared` also validates the writer/session/base-state epoch, so a
/// restart or a newer prepare makes an older token unusable.
#[derive(Debug)]
pub(crate) struct PreparedOccupancyWrite {
    staging_epoch: u64,
    session_generation: u64,
    base_published_generation: u64,
    base_active_slot: Option<usize>,
    base_slot_capacity: usize,
    base_last_revision: u64,
    staged_generation: u64,
    staged_slot: Option<usize>,
    staged_slot_capacity: usize,
    revision: u64,
    identity: MapWriteIdentity,
}

/// Exact descriptor facts returned after the global header has been committed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OccupancyShmCommit {
    pub file_size: u64,
    pub revision: u64,
    pub generation: u64,
    pub active_slot: usize,
    pub layout_version: u32,
}

/// Single-use rollback authority for one occupancy session-fence advance.
///
/// Rollback restores the prior logical session identity only while no prepare
/// or publication has crossed the installed fence. The staging epoch remains
/// consumed so a token prepared before the advance cannot become valid again.
#[derive(Debug)]
pub(crate) struct OccupancySessionFenceTransaction {
    previous_session_generation: u64,
    previous_last_revision: u64,
    previous_last_identity: Option<MapWriteIdentity>,
    installed_session_generation: u64,
    installed_staging_epoch: u64,
    base_published_generation: u64,
    base_active_slot: Option<usize>,
    base_slot_capacity: usize,
    base_physical_write_count: u64,
    advanced: bool,
}

impl MapWriteIdentity {
    fn from_map(map: &MapState) -> Self {
        let bytes = map_data_bytes(map);
        Self {
            width: map.width,
            height: map.height,
            resolution_bits: map.resolution.to_bits(),
            origin_x_bits: map.origin_x.to_bits(),
            origin_y_bits: map.origin_y.to_bits(),
            frame_id: map.frame_id.clone(),
            revision: map.revision,
            timestamp_ns: map.timestamp_ns,
            data_len: map.data.len(),
            data_checksum: crate::map_checksum::sha256_bytes(bytes),
        }
    }
}

pub struct OccupancyGridShmWriter {
    file: std::fs::File,
    mmap: MmapMut,
    capacity: usize,
    slot_capacity: usize,
    active_slot: Option<usize>,
    published_generation: u64,
    session_generation: u64,
    last_revision: u64,
    last_identity: Option<MapWriteIdentity>,
    staging_epoch: u64,
    physical_write_count: u64,
    shm_path: std::path::PathBuf,
    shm_identity: ProductOwnedMmapIdentity,
}

impl OccupancyGridShmWriter {
    pub fn new(shm_path: &std::path::Path) -> std::io::Result<Self> {
        Self::new_with_capacity(shm_path, OCC_INITIAL_CAPACITY)
    }

    fn new_with_capacity(
        shm_path: &std::path::Path,
        initial_capacity: usize,
    ) -> std::io::Result<Self> {
        let usable = initial_capacity.saturating_sub(GLOBAL_HEADER_BYTES);
        let slot_capacity = (usable / SLOT_COUNT).max(SLOT_HEADER_BYTES + 1);
        let capacity = total_capacity(slot_capacity)?;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(shm_path)?;
        let shm_identity = capture_product_owned_mmap(shm_path, &file)?;
        file.set_len(usize_to_u64(capacity, "occupancy SHM capacity")?)?;
        let mut mmap = unsafe { MmapMut::map_mut(&file)? };
        mmap.fill(0);
        write_u64(&mut mmap, GLOBAL_MAGIC_OFFSET, OCC_SHM_MAGIC);
        write_u32(
            &mut mmap,
            GLOBAL_LAYOUT_VERSION_OFFSET,
            OCCUPANCY_SHM_LAYOUT_VERSION,
        );
        write_u32(
            &mut mmap,
            GLOBAL_HEADER_BYTES_OFFSET,
            GLOBAL_HEADER_BYTES as u32,
        );
        atomic_store_u64(
            &mmap,
            GLOBAL_SLOT_CAPACITY_OFFSET,
            usize_to_u64(slot_capacity, "occupancy slot capacity")?,
            Ordering::Release,
        )?;
        atomic_store_u64(&mmap, GLOBAL_WRITE_SEQUENCE_OFFSET, 0, Ordering::Release)?;
        atomic_store_u64(&mmap, GLOBAL_PUBLISHED_STATE_OFFSET, 0, Ordering::Release)?;
        mmap.flush_range(0, GLOBAL_HEADER_BYTES)?;
        info!(
            "occupancy grid SHM v2 created: {:?} (total={} slot={})",
            shm_path, capacity, slot_capacity
        );
        Ok(Self {
            file,
            mmap,
            capacity,
            slot_capacity,
            active_slot: None,
            published_generation: 0,
            session_generation: 0,
            last_revision: 0,
            last_identity: None,
            staging_epoch: 0,
            physical_write_count: 0,
            shm_path: shm_path.to_path_buf(),
            shm_identity,
        })
    }

    pub(crate) fn cleanup_product_owned_file(&mut self) -> std::io::Result<()> {
        unlink_product_owned_mmap(&self.shm_path, self.shm_identity)
    }

    /// Fence revision tracking to a mapping session while keeping the physical
    /// SHM publication generation monotonic across sessions.
    pub fn begin_session(&mut self, session_generation: u64) -> std::io::Result<()> {
        self.begin_session_transactional(session_generation)
            .map(drop)
    }

    pub(crate) fn begin_session_transactional(
        &mut self,
        session_generation: u64,
    ) -> std::io::Result<OccupancySessionFenceTransaction> {
        if session_generation == 0 {
            return Err(Error::new(
                ErrorKind::InvalidInput,
                "mapping session generation must be nonzero",
            ));
        }
        if session_generation < self.session_generation {
            return Err(Error::new(
                ErrorKind::InvalidInput,
                format!(
                    "mapping session generation regressed: current={} incoming={session_generation}",
                    self.session_generation
                ),
            ));
        }
        let previous_session_generation = self.session_generation;
        let previous_last_revision = self.last_revision;
        let previous_last_identity = self.last_identity.clone();
        let base_published_generation = self.published_generation;
        let base_active_slot = self.active_slot;
        let base_slot_capacity = self.slot_capacity;
        let base_physical_write_count = self.physical_write_count;
        let mut advanced = false;
        if session_generation > self.session_generation {
            self.staging_epoch = self.staging_epoch.checked_add(1).ok_or_else(|| {
                Error::new(
                    ErrorKind::InvalidData,
                    "occupancy SHM staging epoch exhausted",
                )
            })?;
            self.session_generation = session_generation;
            self.last_revision = 0;
            self.last_identity = None;
            advanced = true;
        }
        Ok(OccupancySessionFenceTransaction {
            previous_session_generation,
            previous_last_revision,
            previous_last_identity,
            installed_session_generation: self.session_generation,
            installed_staging_epoch: self.staging_epoch,
            base_published_generation,
            base_active_slot,
            base_slot_capacity,
            base_physical_write_count,
            advanced,
        })
    }

    pub(crate) fn rollback_session_fence(
        &mut self,
        transaction: OccupancySessionFenceTransaction,
    ) -> std::io::Result<()> {
        if !transaction.advanced {
            return Ok(());
        }
        if self.session_generation != transaction.installed_session_generation
            || self.staging_epoch != transaction.installed_staging_epoch
            || self.published_generation != transaction.base_published_generation
            || self.active_slot != transaction.base_active_slot
            || self.slot_capacity != transaction.base_slot_capacity
            || self.physical_write_count != transaction.base_physical_write_count
            || self.last_revision != 0
            || self.last_identity.is_some()
        {
            return Err(Error::new(
                ErrorKind::InvalidInput,
                "occupancy SHM session fence crossed prepare or publication before rollback",
            ));
        }
        self.session_generation = transaction.previous_session_generation;
        self.last_revision = transaction.previous_last_revision;
        self.last_identity = transaction.previous_last_identity;
        Ok(())
    }

    /// Serialize and atomically publish one map revision into the inactive slot.
    pub fn write_occupancy(&mut self, map: &MapState) -> std::io::Result<u64> {
        let prepared = self.prepare_occupancy(map)?;
        Ok(self.commit_prepared(prepared)?.revision)
    }

    /// Write, verify, and flush a map into an inactive slot while leaving the
    /// reader-visible global publication header unchanged.
    pub(crate) fn prepare_occupancy(
        &mut self,
        map: &MapState,
    ) -> std::io::Result<PreparedOccupancyWrite> {
        validate_map(map)?;
        let identity = MapWriteIdentity::from_map(map);
        if map.revision < self.last_revision {
            return Err(Error::new(
                ErrorKind::InvalidInput,
                format!(
                    "occupancy revision regressed: last={} incoming={}",
                    self.last_revision, map.revision
                ),
            ));
        }
        if map.revision == self.last_revision {
            if self.last_identity.as_ref() == Some(&identity) {
                self.staging_epoch = self.staging_epoch.checked_add(1).ok_or_else(|| {
                    Error::new(
                        ErrorKind::InvalidData,
                        "occupancy SHM staging epoch exhausted",
                    )
                })?;
                return Ok(PreparedOccupancyWrite {
                    staging_epoch: self.staging_epoch,
                    session_generation: self.session_generation,
                    base_published_generation: self.published_generation,
                    base_active_slot: self.active_slot,
                    base_slot_capacity: self.slot_capacity,
                    base_last_revision: self.last_revision,
                    staged_generation: self.published_generation,
                    staged_slot: None,
                    staged_slot_capacity: self.slot_capacity,
                    revision: map.revision,
                    identity,
                });
            }
            if self.last_identity.is_some() {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    format!(
                        "occupancy revision {} has conflicting content",
                        map.revision
                    ),
                ));
            }
        }

        self.staging_epoch = self.staging_epoch.checked_add(1).ok_or_else(|| {
            Error::new(
                ErrorKind::InvalidData,
                "occupancy SHM staging epoch exhausted",
            )
        })?;

        let required_slot = SLOT_HEADER_BYTES
            .checked_add(map.data.len())
            .ok_or_else(|| Error::new(ErrorKind::InvalidInput, "occupancy slot size overflow"))?;
        let staged_slot_capacity = if required_slot > self.slot_capacity {
            required_slot.max(
                self.slot_capacity
                    .checked_mul(OCC_GROW_FACTOR)
                    .unwrap_or(required_slot),
            )
        } else {
            self.slot_capacity
        };
        let layout_grows = staged_slot_capacity > self.slot_capacity;
        if layout_grows {
            self.grow(staged_slot_capacity)?;
        }

        let next_generation = self
            .published_generation
            .checked_add(1)
            .filter(|generation| *generation <= (u64::MAX >> 1))
            .ok_or_else(|| Error::new(ErrorKind::InvalidData, "SHM generation exhausted"))?;
        let next_slot = match (layout_grows, self.active_slot) {
            // A grown slot1 starts after both old physical slots, so a failed write can safely
            // restore the old descriptor regardless of which old slot was active.
            (true, _) => 1,
            (false, Some(slot)) => 1 - slot,
            (false, None) => 0,
        };
        self.write_and_verify_slot(
            next_slot,
            staged_slot_capacity,
            next_generation,
            map,
            &identity,
        )?;

        Ok(PreparedOccupancyWrite {
            staging_epoch: self.staging_epoch,
            session_generation: self.session_generation,
            base_published_generation: self.published_generation,
            base_active_slot: self.active_slot,
            base_slot_capacity: self.slot_capacity,
            base_last_revision: self.last_revision,
            staged_generation: next_generation,
            staged_slot: Some(next_slot),
            staged_slot_capacity,
            revision: map.revision,
            identity,
        })
    }

    /// Atomically expose a previously flushed inactive slot.
    ///
    /// All fallible validation happens before the first global-header store.
    /// The mmap is transient process-shared state (startup truncates it), so
    /// release-atomic visibility, rather than crash-durable header flushing,
    /// defines the commit acknowledgement.
    pub(crate) fn commit_prepared(
        &mut self,
        prepared: PreparedOccupancyWrite,
    ) -> std::io::Result<OccupancyShmCommit> {
        self.validate_prepared_base(&prepared)?;
        let file_size = usize_to_u64(self.capacity, "occupancy SHM file size")?;

        let Some(next_slot) = prepared.staged_slot else {
            let active_slot = self.active_slot.ok_or_else(|| {
                Error::new(
                    ErrorKind::InvalidData,
                    "idempotent occupancy commit has no active slot",
                )
            })?;
            return Ok(OccupancyShmCommit {
                file_size,
                revision: prepared.revision,
                generation: self.published_generation,
                active_slot,
                layout_version: OCCUPANCY_SHM_LAYOUT_VERSION,
            });
        };

        let previous_stable_sequence = self
            .published_generation
            .checked_mul(2)
            .ok_or_else(|| Error::new(ErrorKind::InvalidData, "SHM write sequence overflow"))?;
        let in_progress_sequence = previous_stable_sequence
            .checked_add(1)
            .ok_or_else(|| Error::new(ErrorKind::InvalidData, "SHM write sequence overflow"))?;
        let next_stable_sequence = prepared
            .staged_generation
            .checked_mul(2)
            .ok_or_else(|| Error::new(ErrorKind::InvalidData, "SHM write sequence overflow"))?;
        let next_published_state = encode_published_state(prepared.staged_generation, next_slot)?;
        let previous_published_state = match self.active_slot {
            Some(active_slot) => encode_published_state(self.published_generation, active_slot)?,
            None if self.published_generation == 0 => 0,
            None => {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "published occupancy generation has no active slot",
                ));
            }
        };
        let staged_slot_capacity = usize_to_u64(
            prepared.staged_slot_capacity,
            "staged occupancy slot capacity",
        )?;
        let current_slot_capacity = usize_to_u64(self.slot_capacity, "occupancy slot capacity")?;

        let sequence = atomic_u64_at(&self.mmap, GLOBAL_WRITE_SEQUENCE_OFFSET)?;
        let published_state = atomic_u64_at(&self.mmap, GLOBAL_PUBLISHED_STATE_OFFSET)?;
        let slot_capacity = atomic_u64_at(&self.mmap, GLOBAL_SLOT_CAPACITY_OFFSET)?;
        if sequence.load(Ordering::Acquire) != previous_stable_sequence
            || published_state.load(Ordering::Acquire) != previous_published_state
            || slot_capacity.load(Ordering::Acquire) != current_slot_capacity
        {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "occupancy SHM global header differs from prepared base state",
            ));
        }

        sequence.store(in_progress_sequence, Ordering::Release);
        slot_capacity.store(staged_slot_capacity, Ordering::Release);
        fence(Ordering::Release);
        published_state.store(next_published_state, Ordering::Release);
        sequence.store(next_stable_sequence, Ordering::Release);

        self.slot_capacity = prepared.staged_slot_capacity;
        self.active_slot = Some(next_slot);
        self.published_generation = prepared.staged_generation;
        self.last_revision = prepared.revision;
        self.last_identity = Some(prepared.identity);
        self.physical_write_count = self.physical_write_count.saturating_add(1);
        Ok(OccupancyShmCommit {
            file_size,
            revision: self.last_revision,
            generation: self.published_generation,
            active_slot: next_slot,
            layout_version: OCCUPANCY_SHM_LAYOUT_VERSION,
        })
    }

    /// Consume a prepared token without exposing its inactive slot.
    pub(crate) fn abort_prepared(&mut self, prepared: PreparedOccupancyWrite) -> bool {
        self.validate_prepared_base(&prepared).is_ok()
    }

    fn validate_prepared_base(&self, prepared: &PreparedOccupancyWrite) -> std::io::Result<()> {
        if prepared.staging_epoch != self.staging_epoch
            || prepared.session_generation != self.session_generation
            || prepared.base_published_generation != self.published_generation
            || prepared.base_active_slot != self.active_slot
            || prepared.base_slot_capacity != self.slot_capacity
            || prepared.base_last_revision != self.last_revision
        {
            return Err(Error::new(
                ErrorKind::InvalidInput,
                "occupancy SHM prepared token is stale",
            ));
        }
        Ok(())
    }

    fn write_and_verify_slot(
        &mut self,
        slot: usize,
        slot_capacity: usize,
        generation: u64,
        map: &MapState,
        identity: &MapWriteIdentity,
    ) -> std::io::Result<()> {
        let slot_offset = slot_offset(slot, slot_capacity)?;
        let payload_offset = slot_offset
            .checked_add(SLOT_HEADER_BYTES)
            .ok_or_else(|| Error::new(ErrorKind::InvalidInput, "payload offset overflow"))?;
        let payload_end = payload_offset
            .checked_add(map.data.len())
            .ok_or_else(|| Error::new(ErrorKind::InvalidInput, "payload end overflow"))?;
        if payload_end > self.mmap.len() {
            return Err(Error::new(
                ErrorKind::UnexpectedEof,
                "inactive occupancy slot exceeds mapped file",
            ));
        }

        self.mmap[slot_offset..slot_offset + SLOT_HEADER_BYTES].fill(0);
        write_u64(&mut self.mmap, slot_offset + SLOT_MAGIC_OFFSET, SLOT_MAGIC);
        write_u32(
            &mut self.mmap,
            slot_offset + SLOT_LAYOUT_VERSION_OFFSET,
            OCCUPANCY_SHM_LAYOUT_VERSION,
        );
        write_u32(
            &mut self.mmap,
            slot_offset + SLOT_SCHEMA_VERSION_OFFSET,
            OCCUPANCY_SHM_PAYLOAD_SCHEMA_VERSION,
        );
        write_u32(
            &mut self.mmap,
            slot_offset + SLOT_HEADER_BYTES_OFFSET,
            SLOT_HEADER_BYTES as u32,
        );
        write_u32(
            &mut self.mmap,
            slot_offset + SLOT_FRAME_ID_LEN_OFFSET,
            map.frame_id.len() as u32,
        );
        write_u64(
            &mut self.mmap,
            slot_offset + SLOT_GENERATION_OFFSET,
            generation,
        );
        write_u64(
            &mut self.mmap,
            slot_offset + SLOT_REVISION_OFFSET,
            map.revision,
        );
        write_u64(
            &mut self.mmap,
            slot_offset + SLOT_TIMESTAMP_OFFSET,
            map.timestamp_ns,
        );
        write_u64(
            &mut self.mmap,
            slot_offset + SLOT_PAYLOAD_LEN_OFFSET,
            usize_to_u64(map.data.len(), "occupancy payload length")?,
        );
        write_u32(&mut self.mmap, slot_offset + SLOT_WIDTH_OFFSET, map.width);
        write_u32(&mut self.mmap, slot_offset + SLOT_HEIGHT_OFFSET, map.height);
        write_f32(
            &mut self.mmap,
            slot_offset + SLOT_RESOLUTION_OFFSET,
            map.resolution,
        );
        write_f64(
            &mut self.mmap,
            slot_offset + SLOT_ORIGIN_X_OFFSET,
            map.origin_x,
        );
        write_f64(
            &mut self.mmap,
            slot_offset + SLOT_ORIGIN_Y_OFFSET,
            map.origin_y,
        );
        self.mmap[slot_offset + SLOT_CHECKSUM_OFFSET..slot_offset + SLOT_CHECKSUM_OFFSET + 32]
            .copy_from_slice(&identity.data_checksum);
        let frame_end = slot_offset + SLOT_FRAME_ID_OFFSET + map.frame_id.len();
        self.mmap[slot_offset + SLOT_FRAME_ID_OFFSET..frame_end]
            .copy_from_slice(map.frame_id.as_bytes());
        self.mmap[payload_offset..payload_end].copy_from_slice(map_data_bytes(map));

        let persisted_checksum =
            crate::map_checksum::sha256_bytes(&self.mmap[payload_offset..payload_end]);
        if persisted_checksum != identity.data_checksum {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "inactive occupancy slot checksum verification failed",
            ));
        }
        self.mmap
            .flush_range(slot_offset, SLOT_HEADER_BYTES + map.data.len())?;
        Ok(())
    }

    fn grow(&mut self, new_slot_capacity: usize) -> std::io::Result<()> {
        let new_slot_capacity = new_slot_capacity.max(SLOT_HEADER_BYTES + 1);
        let new_capacity = total_capacity(new_slot_capacity)?;
        if new_capacity <= self.capacity {
            return Ok(());
        }
        self.mmap.flush()?;
        self.file
            .set_len(usize_to_u64(new_capacity, "occupancy SHM capacity")?)?;
        let new_mmap = unsafe { MmapMut::map_mut(&self.file)? };
        self.mmap = new_mmap;
        self.capacity = new_capacity;
        info!(
            "occupancy grid SHM v2 allocation grew in-place: total={} staged_slot={}",
            new_capacity, new_slot_capacity
        );
        Ok(())
    }

    pub fn shm_path(&self) -> &std::path::Path {
        &self.shm_path
    }

    pub fn layout_version(&self) -> u32 {
        OCCUPANCY_SHM_LAYOUT_VERSION
    }

    pub fn published_generation(&self) -> u64 {
        self.published_generation
    }

    pub fn active_slot(&self) -> Option<usize> {
        self.active_slot
    }

    pub fn last_revision(&self) -> u64 {
        self.last_revision
    }

    pub fn capacity(&self) -> usize {
        self.capacity
    }

    pub fn physical_write_count(&self) -> u64 {
        self.physical_write_count
    }

    pub(crate) fn session_generation(&self) -> u64 {
        self.session_generation
    }

    #[cfg(test)]
    fn staging_epoch(&self) -> u64 {
        self.staging_epoch
    }
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) struct OccupancyGridShmSnapshot {
    pub layout_version: u32,
    pub schema_version: u32,
    pub generation: u64,
    pub active_slot: usize,
    pub revision: u64,
    pub timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin_x: f64,
    pub origin_y: f64,
    pub frame_id: String,
    pub payload_checksum: [u8; 32],
    pub data: Vec<i8>,
}

/// Read-only helper that owns a separate mmap, matching another process' access pattern.
pub(crate) struct OccupancyGridShmReader {
    file: std::fs::File,
    mmap: Mmap,
}

impl OccupancyGridShmReader {
    pub(crate) fn open(path: &std::path::Path) -> std::io::Result<Self> {
        let file = std::fs::OpenOptions::new().read(true).open(path)?;
        let mmap = unsafe { Mmap::map(&file)? };
        Ok(Self { file, mmap })
    }

    pub(crate) fn read_stable(&mut self) -> std::io::Result<Option<OccupancyGridShmSnapshot>> {
        self.remap_if_size_changed()?;
        validate_global_header(&self.mmap)?;
        let sequence_before =
            atomic_load_u64(&self.mmap, GLOBAL_WRITE_SEQUENCE_OFFSET, Ordering::Acquire)?;
        if sequence_before & 1 == 1 {
            return Ok(None);
        }
        let published_state =
            atomic_load_u64(&self.mmap, GLOBAL_PUBLISHED_STATE_OFFSET, Ordering::Acquire)?;
        let (generation, active_slot) = decode_published_state(published_state)?;
        if generation == 0 {
            return Ok(None);
        }
        if sequence_before / 2 != generation {
            return Ok(None);
        }
        let slot_capacity_u64 =
            atomic_load_u64(&self.mmap, GLOBAL_SLOT_CAPACITY_OFFSET, Ordering::Acquire)?;
        let slot_capacity = usize::try_from(slot_capacity_u64).map_err(|_| {
            Error::new(
                ErrorKind::InvalidData,
                "occupancy slot capacity exceeds usize",
            )
        })?;
        if slot_capacity < SLOT_HEADER_BYTES {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "occupancy slot capacity is smaller than the fixed header",
            ));
        }
        let slot_offset = slot_offset(active_slot, slot_capacity)?;
        let slot_end = slot_offset
            .checked_add(slot_capacity)
            .ok_or_else(|| Error::new(ErrorKind::InvalidData, "occupancy slot end overflow"))?;
        if slot_end > self.mmap.len() {
            self.remap_if_size_changed()?;
            return Ok(None);
        }

        let header = parse_slot_header(&self.mmap, slot_offset, slot_capacity)?;
        let payload_offset = slot_offset + SLOT_HEADER_BYTES;
        let payload_end = payload_offset
            .checked_add(header.payload_len)
            .ok_or_else(|| Error::new(ErrorKind::InvalidData, "occupancy payload end overflow"))?;
        if payload_end > slot_end {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "occupancy payload exceeds its published slot",
            ));
        }
        let payload = self.mmap[payload_offset..payload_end].to_vec();

        fence(Ordering::Acquire);
        let published_state_after =
            atomic_load_u64(&self.mmap, GLOBAL_PUBLISHED_STATE_OFFSET, Ordering::Acquire)?;
        let sequence_after =
            atomic_load_u64(&self.mmap, GLOBAL_WRITE_SEQUENCE_OFFSET, Ordering::Acquire)?;
        if sequence_before != sequence_after
            || sequence_after & 1 == 1
            || published_state != published_state_after
        {
            return Ok(None);
        }
        if header.generation != generation {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "occupancy slot generation differs from published generation",
            ));
        }
        let expected_cells = checked_cells(header.width, header.height)?;
        if expected_cells != payload.len() {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "occupancy slot dimensions differ from payload length",
            ));
        }
        let checksum = crate::map_checksum::sha256_bytes(&payload);
        if checksum != header.payload_checksum {
            return Err(Error::new(
                ErrorKind::InvalidData,
                "occupancy slot payload checksum mismatch",
            ));
        }

        Ok(Some(OccupancyGridShmSnapshot {
            layout_version: OCCUPANCY_SHM_LAYOUT_VERSION,
            schema_version: header.schema_version,
            generation,
            active_slot,
            revision: header.revision,
            timestamp_ns: header.timestamp_ns,
            width: header.width,
            height: header.height,
            resolution: header.resolution,
            origin_x: header.origin_x,
            origin_y: header.origin_y,
            frame_id: header.frame_id,
            payload_checksum: checksum,
            data: payload.into_iter().map(|byte| byte as i8).collect(),
        }))
    }

    fn remap_if_size_changed(&mut self) -> std::io::Result<()> {
        let file_len = usize::try_from(self.file.metadata()?.len()).map_err(|_| {
            Error::new(
                ErrorKind::InvalidData,
                "occupancy SHM file length exceeds usize",
            )
        })?;
        if file_len != self.mmap.len() {
            self.mmap = unsafe { Mmap::map(&self.file)? };
        }
        Ok(())
    }
}

struct SlotHeader {
    schema_version: u32,
    generation: u64,
    revision: u64,
    timestamp_ns: u64,
    payload_len: usize,
    width: u32,
    height: u32,
    resolution: f32,
    origin_x: f64,
    origin_y: f64,
    frame_id: String,
    payload_checksum: [u8; 32],
}

fn parse_slot_header(
    buf: &[u8],
    slot_offset: usize,
    slot_capacity: usize,
) -> std::io::Result<SlotHeader> {
    if read_u64(buf, slot_offset + SLOT_MAGIC_OFFSET)? != SLOT_MAGIC {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy slot magic mismatch",
        ));
    }
    if read_u32(buf, slot_offset + SLOT_LAYOUT_VERSION_OFFSET)? != OCCUPANCY_SHM_LAYOUT_VERSION {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy slot layout version mismatch",
        ));
    }
    if read_u32(buf, slot_offset + SLOT_HEADER_BYTES_OFFSET)? as usize != SLOT_HEADER_BYTES {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy slot header size mismatch",
        ));
    }
    let frame_id_len = read_u32(buf, slot_offset + SLOT_FRAME_ID_LEN_OFFSET)? as usize;
    if frame_id_len > SLOT_FRAME_ID_MAX_BYTES {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy slot frame id exceeds fixed header capacity",
        ));
    }
    let payload_len_u64 = read_u64(buf, slot_offset + SLOT_PAYLOAD_LEN_OFFSET)?;
    let payload_len = usize::try_from(payload_len_u64).map_err(|_| {
        Error::new(
            ErrorKind::InvalidData,
            "occupancy payload length exceeds usize",
        )
    })?;
    if payload_len > slot_capacity - SLOT_HEADER_BYTES {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy payload length exceeds slot capacity",
        ));
    }
    let frame_start = slot_offset + SLOT_FRAME_ID_OFFSET;
    let frame_end = frame_start + frame_id_len;
    let frame_id = std::str::from_utf8(
        buf.get(frame_start..frame_end)
            .ok_or_else(|| Error::new(ErrorKind::UnexpectedEof, "occupancy frame id truncated"))?,
    )
    .map_err(|_| Error::new(ErrorKind::InvalidData, "occupancy frame id is not UTF-8"))?
    .to_string();
    let mut payload_checksum = [0u8; 32];
    payload_checksum.copy_from_slice(
        buf.get(slot_offset + SLOT_CHECKSUM_OFFSET..slot_offset + SLOT_CHECKSUM_OFFSET + 32)
            .ok_or_else(|| Error::new(ErrorKind::UnexpectedEof, "occupancy checksum truncated"))?,
    );
    Ok(SlotHeader {
        schema_version: read_u32(buf, slot_offset + SLOT_SCHEMA_VERSION_OFFSET)?,
        generation: read_u64(buf, slot_offset + SLOT_GENERATION_OFFSET)?,
        revision: read_u64(buf, slot_offset + SLOT_REVISION_OFFSET)?,
        timestamp_ns: read_u64(buf, slot_offset + SLOT_TIMESTAMP_OFFSET)?,
        payload_len,
        width: read_u32(buf, slot_offset + SLOT_WIDTH_OFFSET)?,
        height: read_u32(buf, slot_offset + SLOT_HEIGHT_OFFSET)?,
        resolution: read_f32(buf, slot_offset + SLOT_RESOLUTION_OFFSET)?,
        origin_x: read_f64(buf, slot_offset + SLOT_ORIGIN_X_OFFSET)?,
        origin_y: read_f64(buf, slot_offset + SLOT_ORIGIN_Y_OFFSET)?,
        frame_id,
        payload_checksum,
    })
}

fn validate_global_header(buf: &[u8]) -> std::io::Result<()> {
    if buf.len() < GLOBAL_HEADER_BYTES {
        return Err(Error::new(
            ErrorKind::UnexpectedEof,
            "occupancy SHM global header is truncated",
        ));
    }
    if read_u64(buf, GLOBAL_MAGIC_OFFSET)? != OCC_SHM_MAGIC {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy SHM magic mismatch",
        ));
    }
    if read_u32(buf, GLOBAL_LAYOUT_VERSION_OFFSET)? != OCCUPANCY_SHM_LAYOUT_VERSION {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy SHM layout version mismatch",
        ));
    }
    if read_u32(buf, GLOBAL_HEADER_BYTES_OFFSET)? as usize != GLOBAL_HEADER_BYTES {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy SHM global header size mismatch",
        ));
    }
    Ok(())
}

fn validate_map(map: &MapState) -> std::io::Result<()> {
    let expected_cells = checked_cells(map.width, map.height)?;
    if expected_cells != map.data.len() {
        return Err(Error::new(
            ErrorKind::InvalidInput,
            format!(
                "occupancy data length mismatch: {}x{}={} data_len={}",
                map.width,
                map.height,
                expected_cells,
                map.data.len()
            ),
        ));
    }
    if map.frame_id.len() > SLOT_FRAME_ID_MAX_BYTES {
        return Err(Error::new(
            ErrorKind::InvalidInput,
            "occupancy frame id exceeds v2 fixed header capacity",
        ));
    }
    Ok(())
}

fn checked_cells(width: u32, height: u32) -> std::io::Result<usize> {
    usize::try_from(width)
        .ok()
        .and_then(|width| {
            usize::try_from(height)
                .ok()
                .and_then(|height| width.checked_mul(height))
        })
        .ok_or_else(|| {
            Error::new(
                ErrorKind::InvalidInput,
                "occupancy dimensions overflow usize",
            )
        })
}

fn total_capacity(slot_capacity: usize) -> std::io::Result<usize> {
    slot_capacity
        .checked_mul(SLOT_COUNT)
        .and_then(|slots| GLOBAL_HEADER_BYTES.checked_add(slots))
        .ok_or_else(|| Error::new(ErrorKind::InvalidInput, "occupancy SHM capacity overflow"))
}

fn slot_offset(slot: usize, slot_capacity: usize) -> std::io::Result<usize> {
    if slot >= SLOT_COUNT {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "occupancy active slot is invalid",
        ));
    }
    slot.checked_mul(slot_capacity)
        .and_then(|offset| GLOBAL_HEADER_BYTES.checked_add(offset))
        .ok_or_else(|| Error::new(ErrorKind::InvalidData, "occupancy slot offset overflow"))
}

fn encode_published_state(generation: u64, active_slot: usize) -> std::io::Result<u64> {
    if active_slot >= SLOT_COUNT || generation == 0 || generation > (u64::MAX >> 1) {
        return Err(Error::new(
            ErrorKind::InvalidInput,
            "invalid occupancy published generation/slot",
        ));
    }
    Ok((generation << 1) | active_slot as u64)
}

fn decode_published_state(state: u64) -> std::io::Result<(u64, usize)> {
    let generation = state >> 1;
    let slot = (state & 1) as usize;
    if slot >= SLOT_COUNT {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "invalid occupancy active slot",
        ));
    }
    Ok((generation, slot))
}

fn map_data_bytes(map: &MapState) -> &[u8] {
    // i8 and u8 have identical one-byte layout; SHM persists the original two's-complement bytes.
    unsafe { std::slice::from_raw_parts(map.data.as_ptr().cast::<u8>(), map.data.len()) }
}

fn usize_to_u64(value: usize, label: &str) -> std::io::Result<u64> {
    u64::try_from(value)
        .map_err(|_| Error::new(ErrorKind::InvalidInput, format!("{label} exceeds u64")))
}

fn atomic_u64_at(buf: &[u8], offset: usize) -> std::io::Result<&AtomicU64> {
    let end = offset
        .checked_add(std::mem::size_of::<AtomicU64>())
        .ok_or_else(|| Error::new(ErrorKind::InvalidData, "atomic offset overflow"))?;
    if end > buf.len() {
        return Err(Error::new(
            ErrorKind::UnexpectedEof,
            "atomic field is truncated",
        ));
    }
    let pointer = unsafe { buf.as_ptr().add(offset).cast_mut().cast::<u64>() };
    if (pointer as usize) % std::mem::align_of::<AtomicU64>() != 0 {
        return Err(Error::new(
            ErrorKind::InvalidData,
            "process-shared atomic field is not aligned",
        ));
    }
    Ok(unsafe { AtomicU64::from_ptr(pointer) })
}

fn atomic_store_u64(
    buf: &[u8],
    offset: usize,
    value: u64,
    ordering: Ordering,
) -> std::io::Result<()> {
    atomic_u64_at(buf, offset)?.store(value, ordering);
    Ok(())
}

fn atomic_load_u64(buf: &[u8], offset: usize, ordering: Ordering) -> std::io::Result<u64> {
    Ok(atomic_u64_at(buf, offset)?.load(ordering))
}

fn write_u32(buf: &mut [u8], offset: usize, value: u32) {
    buf[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn write_u64(buf: &mut [u8], offset: usize, value: u64) {
    buf[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

fn write_f32(buf: &mut [u8], offset: usize, value: f32) {
    write_u32(buf, offset, value.to_bits());
}

fn write_f64(buf: &mut [u8], offset: usize, value: f64) {
    write_u64(buf, offset, value.to_bits());
}

fn read_u32(buf: &[u8], offset: usize) -> std::io::Result<u32> {
    let bytes = buf
        .get(offset..offset + 4)
        .ok_or_else(|| Error::new(ErrorKind::UnexpectedEof, "u32 field is truncated"))?;
    Ok(u32::from_le_bytes(
        bytes.try_into().expect("length checked"),
    ))
}

fn read_u64(buf: &[u8], offset: usize) -> std::io::Result<u64> {
    let bytes = buf
        .get(offset..offset + 8)
        .ok_or_else(|| Error::new(ErrorKind::UnexpectedEof, "u64 field is truncated"))?;
    Ok(u64::from_le_bytes(
        bytes.try_into().expect("length checked"),
    ))
}

fn read_f32(buf: &[u8], offset: usize) -> std::io::Result<f32> {
    Ok(f32::from_bits(read_u32(buf, offset)?))
}

fn read_f64(buf: &[u8], offset: usize) -> std::io::Result<f64> {
    Ok(f64::from_bits(read_u64(buf, offset)?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, AtomicUsize};
    use std::sync::Arc;

    fn test_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("nuwax_occ_{name}_{}.mmap", std::process::id()))
    }

    fn sample_map(width: u32, height: u32, fill: i8) -> MapState {
        MapState {
            width,
            height,
            resolution: 0.05,
            origin_x: -1.0,
            origin_y: -2.0,
            frame_id: "map".to_string(),
            revision: 1,
            timestamp_ns: 12345,
            data: vec![fill; (width * height) as usize],
        }
    }

    fn read_until_snapshot(
        reader: &mut OccupancyGridShmReader,
        revision: u64,
    ) -> OccupancyGridShmSnapshot {
        for _ in 0..10_000 {
            if let Some(snapshot) = reader.read_stable().expect("stable read") {
                if snapshot.revision == revision {
                    return snapshot;
                }
            }
            std::thread::yield_now();
        }
        panic!("revision {revision} was not observed");
    }

    #[test]
    fn product_cleanup_unlinks_owned_occupancy_mmap() {
        let path = test_path("owned_cleanup");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");

        writer
            .cleanup_product_owned_file()
            .expect("owned occupancy mmap cleanup");

        assert!(!path.exists());
    }

    #[test]
    fn product_cleanup_rejects_replaced_occupancy_mmap() {
        let path = test_path("replaced_cleanup");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        std::fs::remove_file(&path).expect("remove owned name");
        std::fs::write(&path, b"foreign replacement").expect("install replacement");

        let error = writer
            .cleanup_product_owned_file()
            .expect_err("replacement must be preserved");

        assert!(error
            .to_string()
            .contains("refusing to unlink replaced mmap path"));
        assert_eq!(
            std::fs::read(&path).expect("replacement remains"),
            b"foreign replacement"
        );
        std::fs::remove_file(path).expect("remove replacement");
    }

    #[test]
    fn product_cleanup_is_idempotent_when_occupancy_mmap_is_missing() {
        let path = test_path("missing_cleanup");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        std::fs::remove_file(&path).expect("remove owned name");

        writer
            .cleanup_product_owned_file()
            .expect("first missing cleanup");
        writer
            .cleanup_product_owned_file()
            .expect("repeated missing cleanup");
    }

    #[test]
    fn create_writer_succeeds_with_v2_atomic_header() {
        let path = test_path("create");
        let _ = std::fs::remove_file(&path);
        let writer = OccupancyGridShmWriter::new(&path).expect("create");
        assert_eq!(writer.layout_version(), 2);
        assert_eq!(writer.published_generation(), 0);
        assert_eq!(writer.active_slot(), None);
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");
        assert!(reader.read_stable().expect("empty read").is_none());
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn write_then_separate_mmap_reader_roundtrips_fixed_header_and_checksum() {
        let path = test_path("roundtrip");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");
        let map = sample_map(4, 4, 100);
        writer.write_occupancy(&map).expect("write");

        let snapshot = read_until_snapshot(&mut reader, 1);
        assert_eq!(snapshot.layout_version, 2);
        assert_eq!(snapshot.schema_version, 1);
        assert_eq!(snapshot.generation, 1);
        assert_eq!(snapshot.active_slot, 0);
        assert_eq!((snapshot.width, snapshot.height), (4, 4));
        assert_eq!(snapshot.frame_id, "map");
        assert_eq!(snapshot.data, map.data);
        assert_eq!(
            snapshot.payload_checksum,
            crate::map_checksum::sha256_bytes(map_data_bytes(&map))
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn grow_keeps_inode_and_reader_remaps_both_slots_safely() {
        let path = test_path("grow");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new_with_capacity(&path, 4096).expect("create");
        let initial_capacity = writer.capacity();
        let initial_inode = file_identity(&path);
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");

        let first = sample_map(16, 16, 7);
        writer.write_occupancy(&first).expect("first slot");
        assert_eq!(read_until_snapshot(&mut reader, 1).data, first.data);

        let mut second = sample_map(16, 16, 8);
        second.revision = 2;
        second.timestamp_ns = 23456;
        writer.write_occupancy(&second).expect("second slot");
        assert_eq!(writer.active_slot(), Some(1));
        assert_eq!(read_until_snapshot(&mut reader, 2).data, second.data);

        let mut third = sample_map(64, 64, 42);
        third.revision = 3;
        third.timestamp_ns = 34567;
        writer
            .write_occupancy(&third)
            .expect("grow while old slot1 is active");
        assert!(writer.capacity() > initial_capacity);
        assert_eq!(file_identity(&path), initial_inode);
        let snapshot = read_until_snapshot(&mut reader, 3);
        assert_eq!(snapshot.active_slot, 1);
        assert_eq!(snapshot.data, third.data);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn duplicate_revision_is_idempotent_but_conflict_and_regression_are_rejected() {
        let path = test_path("identity");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        let mut map = sample_map(4, 4, 100);
        map.revision = 2;

        writer.write_occupancy(&map).expect("first write");
        writer.write_occupancy(&map).expect("same revision replay");
        assert_eq!(writer.physical_write_count(), 1);
        assert_eq!(writer.published_generation(), 1);

        let mut conflicting = map.clone();
        conflicting.data[0] = 0;
        assert_eq!(
            writer
                .write_occupancy(&conflicting)
                .expect_err("conflict")
                .kind(),
            ErrorKind::InvalidData
        );
        let mut regressed = map;
        regressed.revision = 1;
        assert_eq!(
            writer
                .write_occupancy(&regressed)
                .expect_err("regression")
                .kind(),
            ErrorKind::InvalidInput
        );
        assert_eq!(writer.physical_write_count(), 1);
        assert_eq!(writer.last_revision(), 2);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn prepare_and_abort_leave_public_header_and_reader_snapshot_unchanged() {
        let path = test_path("prepare_abort");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new_with_capacity(&path, 4096).expect("create");
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");
        let first = sample_map(16, 16, 7);
        writer.write_occupancy(&first).expect("baseline write");
        let baseline = read_until_snapshot(&mut reader, 1);
        let header_before = writer.mmap[..GLOBAL_HEADER_BYTES].to_vec();

        let mut second = sample_map(16, 16, 8);
        second.revision = 2;
        second.timestamp_ns = 23456;
        let prepared = writer.prepare_occupancy(&second).expect("prepare");

        assert_eq!(&writer.mmap[..GLOBAL_HEADER_BYTES], header_before);
        assert_eq!(read_until_snapshot(&mut reader, 1), baseline);
        assert!(writer.abort_prepared(prepared));
        assert_eq!(&writer.mmap[..GLOBAL_HEADER_BYTES], header_before);
        assert_eq!(read_until_snapshot(&mut reader, 1), baseline);
        assert_eq!(writer.published_generation(), 1);
        assert_eq!(writer.physical_write_count(), 1);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn commit_exposes_only_the_exact_flushed_prepare_token() {
        let path = test_path("prepare_commit");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");
        let first = sample_map(4, 4, 1);
        writer.write_occupancy(&first).expect("baseline write");

        let mut second = sample_map(4, 4, 2);
        second.revision = 2;
        second.timestamp_ns = 23456;
        let prepared = writer.prepare_occupancy(&second).expect("prepare");
        assert_eq!(read_until_snapshot(&mut reader, 1).data, first.data);

        let commit = writer.commit_prepared(prepared).expect("commit");
        assert_eq!(commit.revision, 2);
        assert_eq!(commit.generation, 2);
        assert_eq!(commit.active_slot, 1);
        assert_eq!(commit.file_size, writer.capacity() as u64);
        let snapshot = read_until_snapshot(&mut reader, 2);
        assert_eq!(snapshot.generation, commit.generation);
        assert_eq!(snapshot.active_slot, commit.active_slot);
        assert_eq!(snapshot.data, second.data);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn restart_and_newer_prepare_invalidate_old_tokens_without_publication() {
        let path = test_path("stale_tokens");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");
        writer.begin_session(1).expect("session one");
        let first = sample_map(4, 4, 1);
        writer.write_occupancy(&first).expect("baseline write");
        let header_before = writer.mmap[..GLOBAL_HEADER_BYTES].to_vec();

        let mut stale_after_restart = sample_map(4, 4, 2);
        stale_after_restart.revision = 2;
        let stale_after_restart = writer
            .prepare_occupancy(&stale_after_restart)
            .expect("prepare before restart");
        writer.begin_session(2).expect("session two");
        assert_eq!(
            writer
                .commit_prepared(stale_after_restart)
                .expect_err("restart must invalidate token")
                .kind(),
            ErrorKind::InvalidInput
        );
        assert_eq!(&writer.mmap[..GLOBAL_HEADER_BYTES], header_before);

        let mut superseded = sample_map(4, 4, 3);
        superseded.revision = 3;
        let superseded = writer
            .prepare_occupancy(&superseded)
            .expect("first session-two prepare");
        let mut current = sample_map(4, 4, 4);
        current.revision = 4;
        let current_token = writer
            .prepare_occupancy(&current)
            .expect("superseding prepare");
        assert_eq!(
            writer
                .commit_prepared(superseded)
                .expect_err("newer prepare must invalidate old token")
                .kind(),
            ErrorKind::InvalidInput
        );
        assert_eq!(&writer.mmap[..GLOBAL_HEADER_BYTES], header_before);
        writer
            .commit_prepared(current_token)
            .expect("current token commit");
        assert_eq!(read_until_snapshot(&mut reader, 4).data, current.data);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn occupancy_session_fence_rollback_restores_identity_without_resurrecting_tokens() {
        let path = test_path("session_fence_rollback");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        writer.begin_session(1).expect("session one");
        let baseline = sample_map(4, 4, 1);
        writer.write_occupancy(&baseline).expect("baseline write");

        let mut stale = sample_map(4, 4, 2);
        stale.revision = 2;
        let stale = writer
            .prepare_occupancy(&stale)
            .expect("prepare before session fence");
        let epoch_before = writer.staging_epoch();
        let transaction = writer
            .begin_session_transactional(2)
            .expect("transactional session fence");
        let installed_epoch = writer.staging_epoch();
        assert!(installed_epoch > epoch_before);

        writer
            .rollback_session_fence(transaction)
            .expect("rollback untouched session fence");

        assert_eq!(writer.session_generation(), 1);
        assert_eq!(writer.last_revision(), 1);
        assert_eq!(writer.staging_epoch(), installed_epoch);
        assert_eq!(
            writer
                .commit_prepared(stale)
                .expect_err("pre-fence prepare must remain stale after rollback")
                .kind(),
            ErrorKind::InvalidInput
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn occupancy_session_fence_rollback_rejects_after_prepare_or_commit() {
        let prepare_path = test_path("session_fence_prepare_crossed");
        let _ = std::fs::remove_file(&prepare_path);
        let mut prepare_writer =
            OccupancyGridShmWriter::new(&prepare_path).expect("prepare writer");
        prepare_writer.begin_session(1).expect("session one");
        prepare_writer
            .write_occupancy(&sample_map(4, 4, 1))
            .expect("baseline write");
        let prepare_transaction = prepare_writer
            .begin_session_transactional(2)
            .expect("prepare transaction");
        let staged = prepare_writer
            .prepare_occupancy(&sample_map(4, 4, 2))
            .expect("cross fence with prepare");
        assert_eq!(
            prepare_writer
                .rollback_session_fence(prepare_transaction)
                .expect_err("prepare must prevent rollback")
                .kind(),
            ErrorKind::InvalidInput
        );
        assert!(prepare_writer.abort_prepared(staged));
        let _ = std::fs::remove_file(&prepare_path);

        let commit_path = test_path("session_fence_commit_crossed");
        let _ = std::fs::remove_file(&commit_path);
        let mut commit_writer = OccupancyGridShmWriter::new(&commit_path).expect("commit writer");
        commit_writer.begin_session(1).expect("session one");
        commit_writer
            .write_occupancy(&sample_map(4, 4, 1))
            .expect("baseline write");
        let commit_transaction = commit_writer
            .begin_session_transactional(2)
            .expect("commit transaction");
        let staged = commit_writer
            .prepare_occupancy(&sample_map(4, 4, 2))
            .expect("prepare committed revision");
        commit_writer
            .commit_prepared(staged)
            .expect("cross fence with commit");
        assert_eq!(
            commit_writer
                .rollback_session_fence(commit_transaction)
                .expect_err("commit must prevent rollback")
                .kind(),
            ErrorKind::InvalidInput
        );
        assert_eq!(commit_writer.session_generation(), 2);
        let _ = std::fs::remove_file(&commit_path);
    }

    #[test]
    fn aborted_growth_preserves_old_layout_and_committed_growth_remaps_reader() {
        let path = test_path("staged_growth");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new_with_capacity(&path, 1024).expect("create");
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");
        let first = sample_map(8, 8, 1);
        writer.write_occupancy(&first).expect("first write");
        let mut second = sample_map(8, 8, 2);
        second.revision = 2;
        writer.write_occupancy(&second).expect("second write");
        let baseline = read_until_snapshot(&mut reader, 2);
        let published_capacity = writer.slot_capacity;
        let header_before = writer.mmap[..GLOBAL_HEADER_BYTES].to_vec();
        let inode_before = file_identity(&path);

        let mut grown = sample_map(128, 64, 3);
        grown.revision = 3;
        let prepared = writer.prepare_occupancy(&grown).expect("growth prepare");
        assert!(writer.capacity() > 1024);
        assert_eq!(writer.slot_capacity, published_capacity);
        assert_eq!(&writer.mmap[..GLOBAL_HEADER_BYTES], header_before);
        assert_eq!(read_until_snapshot(&mut reader, 2), baseline);
        assert!(writer.abort_prepared(prepared));
        assert_eq!(read_until_snapshot(&mut reader, 2), baseline);

        let prepared = writer
            .prepare_occupancy(&grown)
            .expect("repeat growth prepare");
        let commit = writer.commit_prepared(prepared).expect("growth commit");
        assert_eq!(commit.active_slot, 1);
        assert_eq!(file_identity(&path), inode_before);
        let snapshot = read_until_snapshot(&mut reader, 3);
        assert_eq!(snapshot.data, grown.data);
        assert_eq!(snapshot.active_slot, commit.active_slot);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn failed_prepare_leaves_previous_header_and_snapshot_readable() {
        let path = test_path("failed_prepare");
        let _ = std::fs::remove_file(&path);
        let mut writer = OccupancyGridShmWriter::new(&path).expect("create");
        let mut reader = OccupancyGridShmReader::open(&path).expect("reader mmap");
        let first = sample_map(4, 4, 1);
        writer.write_occupancy(&first).expect("baseline write");
        let baseline = read_until_snapshot(&mut reader, 1);
        let header_before = writer.mmap[..GLOBAL_HEADER_BYTES].to_vec();

        let mut malformed = sample_map(4, 4, 2);
        malformed.revision = 2;
        malformed.data.pop();
        assert_eq!(
            writer
                .prepare_occupancy(&malformed)
                .expect_err("malformed payload must fail prepare")
                .kind(),
            ErrorKind::InvalidInput
        );
        assert_eq!(&writer.mmap[..GLOBAL_HEADER_BYTES], header_before);
        assert_eq!(read_until_snapshot(&mut reader, 1), baseline);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn concurrent_separate_mmaps_never_accept_torn_snapshot() {
        const WRITES: u64 = 3_000;
        let path = test_path("stress");
        let _ = std::fs::remove_file(&path);
        let writer = OccupancyGridShmWriter::new_with_capacity(&path, 1024).expect("create");
        let done = Arc::new(AtomicBool::new(false));
        let accepted = Arc::new(AtomicUsize::new(0));
        let in_progress_reads = Arc::new(AtomicUsize::new(0));

        let reader_path = path.clone();
        let reader_done = done.clone();
        let writer_done = done.clone();
        let reader_accepted = accepted.clone();
        let reader_in_progress = in_progress_reads.clone();
        let reader_thread = std::thread::spawn(move || {
            let mut reader = OccupancyGridShmReader::open(&reader_path).expect("reader mmap");
            let mut final_reads = 0usize;
            while !reader_done.load(Ordering::Acquire) || final_reads < 1_000 {
                if reader_done.load(Ordering::Acquire) {
                    final_reads += 1;
                }
                match reader.read_stable().expect("stable reader protocol") {
                    Some(snapshot) => {
                        assert_eq!(
                            snapshot.data.len(),
                            (snapshot.width as usize) * (snapshot.height as usize)
                        );
                        assert_eq!(
                            snapshot.payload_checksum,
                            crate::map_checksum::sha256_bytes(
                                &snapshot
                                    .data
                                    .iter()
                                    .map(|value| *value as u8)
                                    .collect::<Vec<_>>()
                            )
                        );
                        let expected = (snapshot.revision % 101) as i8;
                        assert!(snapshot.data.iter().all(|value| *value == expected));
                        reader_accepted.fetch_add(1, Ordering::Relaxed);
                    }
                    None => {
                        reader_in_progress.fetch_add(1, Ordering::Relaxed);
                    }
                }
                std::thread::yield_now();
            }
        });

        let writer_thread = std::thread::spawn(move || {
            let mut writer = writer;
            for revision in 1..=WRITES {
                let width = if revision % 3 == 0 { 96 } else { 64 };
                let height = if revision % 5 == 0 { 48 } else { 32 };
                let mut map = sample_map(width, height, (revision % 101) as i8);
                map.revision = revision;
                map.timestamp_ns = revision * 1_000_000;
                writer.write_occupancy(&map).expect("stress write");
                if revision % 8 == 0 {
                    std::thread::yield_now();
                }
            }
            writer_done.store(true, Ordering::Release);
        });

        writer_thread.join().expect("writer thread");
        reader_thread.join().expect("reader thread");
        assert!(accepted.load(Ordering::Relaxed) > 100);
        assert!(in_progress_reads.load(Ordering::Relaxed) > 0);
        let _ = std::fs::remove_file(&path);
    }

    #[cfg(unix)]
    fn file_identity(path: &std::path::Path) -> u64 {
        use std::os::unix::fs::MetadataExt;
        std::fs::metadata(path).expect("metadata").ino()
    }

    #[cfg(not(unix))]
    fn file_identity(path: &std::path::Path) -> u64 {
        std::fs::metadata(path).expect("metadata").len()
    }
}
