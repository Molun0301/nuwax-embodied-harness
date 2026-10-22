//! Bounded diagnostic of the existing activation validation/decode pair only.
//! No production preparation/install path or allocator is changed here.
use super::*;
use crate::activation_identity::{CanonicalCloudTargetV2, CheckpointMemberIdentityV2};
use crate::proto;
use std::time::Instant;

struct NoHash(bool);
impl NoHash {
    fn enter() -> Self {
        Self(crate::map_checksum::FORBID_HASH.with(|v| v.replace(true)))
    }
}
impl Drop for NoHash {
    fn drop(&mut self) {
        crate::map_checksum::FORBID_HASH.with(|v| v.set(self.0));
    }
}

fn target(count: usize) -> CanonicalCloudTargetV2 {
    let commit = proto::MappingCommitIdentity { boot_nonce: (1..=16).collect(), sequence: 5 };
    let member = proto::CheckpointMemberIdentity {
        checkpoint_id: "00112233445566778899aabbccddeeff".into(),
        role: proto::CheckpointMemberRole::CanonicalCloud as i32,
        occupancy_receipt: Some(proto::CanonicalOccupancyCommitReceipt {
            commit_id: Some(commit.clone()), session_id: "decode-diagnostic".into(),
            session_generation: 7, occupancy_revision: 11, source_timestamp_ns: 100,
            backend_kind: "voxel".into(), backend_generation: 3,
        }),
        cloud_receipt: Some(proto::CanonicalCloudCommitReceipt {
            commit_id: Some(commit), session_generation: 7, occupancy_revision: 11,
            cloud_revision: 9, source_timestamp_ns: 100, point_count: count as u32,
        }),
        frame_id: "map".into(), frame_generation: 0, producer_generation: 0,
    };
    let parent = CheckpointMemberIdentityV2::try_from(member.clone()).unwrap();
    let mut bytes = Vec::with_capacity(count * 12);
    // Exactly representable 12.5 cm canonical voxel centers, maximum 125 m square.
    for index in 0..count {
        for value in [(index % 1000) as f32 * 0.125 + 0.0625,
                      (index / 1000) as f32 * 0.125 + 0.0625, 0.0625] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
    }
    CanonicalCloudTargetV2::admit(proto::CloudMapCheckpoint {
        frame_id: "map".into(), source_revision: 9, point_count: count as u32,
        voxel_size: 0.125, xyz_f32_le: bytes,
        payload_schema_version: 2, member_identity: Some(member),
    }, parent.checkpoint_id(), parent.occupancy_receipt(), parent.cloud_receipt().unwrap(),
        parent.frame_id()).unwrap()
}

// Optional read-only GNU allocator footprint, not cumulative allocation counts
// or exact live Rust bytes. There is no malloc_trim or allocator replacement.
#[cfg(all(target_os = "linux", target_env = "gnu"))]
#[allow(unsafe_code)]
mod gnu {
    use std::ffi::{c_char, c_void};
    #[repr(C)]
    struct Mallinfo2 {
        arena: usize, ordblks: usize, smblks: usize, hblks: usize, hblkhd: usize,
        usmblks: usize, fsmblks: usize, uordblks: usize, fordblks: usize, keepcost: usize,
    }
    #[link(name = "dl")]
    unsafe extern "C" { fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void; }
    type Info = unsafe extern "C" fn() -> Mallinfo2;
    pub(super) fn info() -> Option<[usize; 5]> {
        if std::env::var_os("LD_PRELOAD").is_some_and(|v| !v.is_empty()) { return None; }
        let symbol = unsafe { dlsym(std::ptr::null_mut(), c"mallinfo2".as_ptr()) };
        if symbol.is_null() { return None; }
        let call: Info = unsafe { std::mem::transmute(symbol) };
        let v = unsafe { call() };
        Some([v.arena, v.uordblks, v.fordblks, v.hblkhd, v.hblks])
    }
}

fn memory(arm: &str, count: usize, phase: &str, capacity: Option<usize>) {
    #[cfg(all(target_os = "linux", target_env = "gnu"))]
    let allocator = gnu::info();
    #[cfg(not(all(target_os = "linux", target_env = "gnu")))]
    let allocator: Option<[usize; 5]> = None;
    let status = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    let value = |key: &str| status.lines().find_map(|line| line.strip_prefix(key)
        .and_then(|v| v.split_whitespace().next()).and_then(|v| v.parse::<u64>().ok()));
    eprintln!("ACTIVATION_DECODE_MEMORY arm={arm} points={count} phase={phase} rss_kib={:?} hwm_kib={:?} allocator_arena_inuse_free_mmapbytes_mmapcount={allocator:?} set_capacity={capacity:?} allocator_not_exact_live_rust_bytes=true samples_not_atomic=true",
        value("VmRSS:"), value("VmHWM:"));
}

fn decode(owner: &CanonicalCloudTargetV2, writer: &CloudMapShmWriter) -> HashSet<VoxelKey> {
    decode_checkpoint_voxels(owner.voxel_size(), owner.point_count(), owner.xyz_f32_le(), writer.max_points).unwrap()
}

fn check(keys: &HashSet<VoxelKey>, count: usize) {
    assert_eq!(keys.len(), count);
    for &(x, y, z) in keys {
        assert!((0..1000).contains(&x) && y >= 0 && z == 0);
        assert!((y as usize) * 1000 + (x as usize) < count);
    }
}

#[test]
#[ignore = "bounded activation decode timing/retained-lifetime diagnostic, opt in explicitly"]
fn assigned_activation_decode_pair_diagnostic() {
    let _no_hash = NoHash::enter(); // Before fixture, writer, or payload construction.
    let started = Instant::now();
    let count: usize = std::env::var("NEH_ACTIVATION_DIAG_POINTS").unwrap_or_else(|_| "100000".into()).parse().unwrap();
    assert!([1000, 100_000, 1_000_000].contains(&count));
    let arm = std::env::var("NEH_ACTIVATION_DIAG_ARM").unwrap_or_else(|_| "baseline".into());
    assert!(arm == "baseline" || arm == "retained");
    let baseline = arm == "baseline";
    memory(&arm, count, "before_fixture", None);
    let owner = target(count);
    let alias = owner.clone();
    assert_eq!(owner.xyz_f32_le().as_ptr(), alias.xyz_f32_le().as_ptr());
    drop(alias);
    let mut writer = CloudMapShmWriter::new_private_for_test().unwrap();
    memory(&arm, count, "immutable_owner_writer", None);
    let retained = if baseline {
        writer.validate_checkpoint(owner.voxel_size(), owner.point_count(), owner.xyz_f32_le()).unwrap();
        None
    } else { Some(decode(&owner, &writer)) };
    memory(&arm, count, "after_preflight", retained.as_ref().map(|keys| keys.capacity()));
    // Arbitrary lifetime illustration only, NOT an actual archive/restore allocation
    // and NOT a prediction of activation peak RSS. Touch every byte explicitly.
    let mut scratch = vec![0_u8; 8 * 1024 * 1024];
    scratch.fill(0x5a);
    std::hint::black_box(&scratch);
    memory(&arm, count, "arbitrary_8mib_intervening_scratch", retained.as_ref().map(|keys| keys.capacity()));
    let keys = retained.unwrap_or_else(|| decode(&owner, &writer));
    check(&keys, count);
    memory(&arm, count, "decode_result_live_with_scratch", Some(keys.capacity()));
    drop(keys);
    drop(scratch);
    memory(&arm, count, "results_and_scratch_dropped", None);

    // Fresh processes isolate each arm. Each timed result is checked outside timing.
    // This compares the existing validate + decode pair with one decoder result;
    // it excludes RPC, archive work, backend restore, VoxelIndex, publication and locks.
    for sample in 0..7 {
        let before = Instant::now();
        if baseline {
            writer.validate_checkpoint(owner.voxel_size(), owner.point_count(), owner.xyz_f32_le()).unwrap();
        }
        let keys = decode(&owner, &writer);
        let elapsed = before.elapsed().as_secs_f64() * 1000.0;
        check(&keys, count);
        eprintln!("ACTIVATION_DECODE_TIMING arm={arm} points={count} sample={sample} ms={elapsed:.6} set_capacity={} set_constructions_by_source={} exact_allocation_call_count=unavailable",
            keys.capacity(), if baseline { 2 } else { 1 });
        drop(keys);
    }
    // Current writer limits must still be checked at installation, even for an
    // already admitted immutable target. No production bypass is implemented.
    writer.max_points = count - 1;
    let reject = writer.validate_checkpoint(owner.voxel_size(), owner.point_count(), owner.xyz_f32_le()).unwrap_err();
    assert!(reject.contains("exceeds SHM capacity"));
    assert_eq!(decode_checkpoint_voxels(owner.voxel_size(), owner.point_count(), owner.xyz_f32_le(), writer.max_points).unwrap_err(), reject);
    eprintln!("ACTIVATION_DECODE_CONTROL arm={arm} points={count} reduced_capacity_rejected=true no_fine_install_exercised=false no_production_install_changed=true test_wall_ms={:.6}", started.elapsed().as_secs_f64() * 1000.0);
    drop(writer);
    drop(owner);
    memory(&arm, count, "all_fixture_owners_dropped", None);
}
