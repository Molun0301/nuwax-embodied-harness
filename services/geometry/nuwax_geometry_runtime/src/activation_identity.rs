//! 激活 v2 的只读身份与目标准入基础；尚未接入默认激活路径。
//!
//! 身份由生产方分配，不能由内容推导。旧 v1 摘要及准入守卫保持独立。
#![cfg_attr(not(test), allow(dead_code))]

use std::sync::Arc;

use crate::mapping_backend::MappingBackendKind;
use crate::mapping_commit::{CloudCommitReceipt, MappingCommitId, OccupancyCommitReceipt};
use crate::pointcloud::{
    CM_POINT_SIZE_BYTES, IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES, MAX_VOXEL_SIZE,
};
use crate::proto::{self, CheckpointMemberRole};

/// 不同拥有方只共用编码，不能互换身份：32 个小写十六进制字符，且不能全零。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AssignedId([u8; 16]);

impl AssignedId {
    fn parse(value: &str) -> Result<Self, String> {
        if !crate::planar_artifacts::assigned_id(value) {
            return Err(
                "assigned identity must be 32 lowercase hexadecimal digits and not all zero".into(),
            );
        }
        let mut bytes = [0; 16];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&value[index * 2..index * 2 + 2], 16)
                .map_err(|_| "invalid assigned asset identity")?;
        }
        Ok(Self(bytes))
    }

    fn encode(self) -> String {
        uuid::Uuid::from_bytes(self.0).simple().to_string()
    }
}

/// 一个完整、不可变资产的分配身份；替换资产必须获得新身份，没有额外修订时钟。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct AssetIdentityV2(AssignedId);

impl AssetIdentityV2 {
    pub(crate) fn parse(value: &str) -> Result<Self, String> {
        AssignedId::parse(value).map(Self)
    }

    pub(crate) fn to_wire(self) -> proto::AssetIdentityV2 {
        proto::AssetIdentityV2 {
            asset_id: self.0.encode(),
        }
    }
}

/// 冻结检查点的父身份，不能用完整资产身份代替。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct CheckpointIdentityV2(AssignedId);

impl CheckpointIdentityV2 {
    pub(crate) fn parse(value: &str) -> Result<Self, String> {
        AssignedId::parse(value).map(Self)
    }

    pub(crate) fn to_wire_id(self) -> String {
        self.0.encode()
    }
}

impl TryFrom<&proto::AssetIdentityV2> for AssetIdentityV2 {
    type Error = String;

    fn try_from(value: &proto::AssetIdentityV2) -> Result<Self, Self::Error> {
        Self::parse(&value.asset_id)
    }
}

fn commit_id_from_wire(
    value: Option<&proto::MappingCommitIdentity>,
) -> Result<MappingCommitId, String> {
    let value = crate::server::validate_mapping_commit_identity(value, "assigned member")?;
    Ok(MappingCommitId {
        boot_nonce: u128::from_be_bytes(
            value
                .boot_nonce
                .as_slice()
                .try_into()
                .map_err(|_| "assigned member commit nonce must contain 16 bytes")?,
        ),
        sequence: value.sequence,
    })
}

/// 沿用已有提交收据，转换时不生成时钟、不读取时间、不处理资产字节。
pub(crate) fn occupancy_receipt_from_wire(
    value: &proto::CanonicalOccupancyCommitReceipt,
) -> Result<OccupancyCommitReceipt, String> {
    let backend_kind = match value.backend_kind.as_str() {
        "voxel" => MappingBackendKind::Voxel,
        "raytracing" => MappingBackendKind::Raytracing,
        "slam_toolbox" => MappingBackendKind::SlamToolbox,
        _ => return Err("assigned member backend kind is unknown".into()),
    };
    if value.session_id.is_empty()
        || value.session_id.trim() != value.session_id
        || value.session_generation == 0
        || value.occupancy_revision == 0
        || value.source_timestamp_ns == 0
        || value.backend_generation == 0
    {
        return Err("assigned member occupancy receipt is incomplete".into());
    }
    Ok(OccupancyCommitReceipt {
        commit_id: commit_id_from_wire(value.commit_id.as_ref())?,
        session_id: value.session_id.clone(),
        session_generation: value.session_generation,
        occupancy_revision: value.occupancy_revision,
        source_timestamp_ns: value.source_timestamp_ns,
        backend_kind,
        backend_generation: value.backend_generation,
    })
}

pub(crate) fn cloud_receipt_from_wire(
    value: &proto::CanonicalCloudCommitReceipt,
) -> Result<CloudCommitReceipt, String> {
    if value.session_generation == 0
        || value.occupancy_revision == 0
        || value.cloud_revision == 0
        || value.source_timestamp_ns == 0
        || value.point_count == 0
    {
        return Err("assigned member cloud receipt is incomplete".into());
    }
    Ok(CloudCommitReceipt {
        commit_id: commit_id_from_wire(value.commit_id.as_ref())?,
        session_generation: value.session_generation,
        occupancy_revision: value.occupancy_revision,
        cloud_revision: value.cloud_revision,
        source_timestamp_ns: value.source_timestamp_ns,
        point_count: value.point_count,
    })
}

/// 拥有式、已验证的检查点成员描述；调用方只能借用内部收据。
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct CheckpointMemberIdentityV2 {
    checkpoint_id: CheckpointIdentityV2,
    role: CheckpointMemberRole,
    occupancy_receipt: OccupancyCommitReceipt,
    cloud_receipt: Option<CloudCommitReceipt>,
    frame_id: String,
    frame_generation: u64,
    producer_generation: u64,
}

impl TryFrom<proto::CheckpointMemberIdentity> for CheckpointMemberIdentityV2 {
    type Error = String;

    fn try_from(value: proto::CheckpointMemberIdentity) -> Result<Self, Self::Error> {
        // 保留配对提交的顺序语义：云收据可以落后于最新占据提交。
        crate::planar_artifacts::validate_member_identity(&value)?;
        let occupancy_receipt = occupancy_receipt_from_wire(
            value
                .occupancy_receipt
                .as_ref()
                .ok_or("assigned member occupancy receipt is required")?,
        )?;
        let cloud_receipt = value
            .cloud_receipt
            .as_ref()
            .map(cloud_receipt_from_wire)
            .transpose()?;
        Ok(Self {
            checkpoint_id: CheckpointIdentityV2::parse(&value.checkpoint_id)?,
            role: CheckpointMemberRole::try_from(value.role)
                .map_err(|_| "assigned member role is unknown")?,
            occupancy_receipt,
            cloud_receipt,
            frame_id: value.frame_id,
            frame_generation: value.frame_generation,
            producer_generation: value.producer_generation,
        })
    }
}

impl CheckpointMemberIdentityV2 {
    pub(crate) fn checkpoint_id(&self) -> CheckpointIdentityV2 {
        self.checkpoint_id
    }
    pub(crate) fn role(&self) -> CheckpointMemberRole {
        self.role
    }
    pub(crate) fn occupancy_receipt(&self) -> &OccupancyCommitReceipt {
        &self.occupancy_receipt
    }
    pub(crate) fn cloud_receipt(&self) -> Option<&CloudCommitReceipt> {
        self.cloud_receipt.as_ref()
    }
    pub(crate) fn frame_id(&self) -> &str {
        &self.frame_id
    }
    pub(crate) fn frame_generation(&self) -> u64 {
        self.frame_generation
    }
    pub(crate) fn producer_generation(&self) -> u64 {
        self.producer_generation
    }

    pub(crate) fn validate_parent(
        &self,
        checkpoint_id: CheckpointIdentityV2,
        role: CheckpointMemberRole,
        occupancy_receipt: &OccupancyCommitReceipt,
        cloud_receipt: Option<&CloudCommitReceipt>,
        frame_id: &str,
    ) -> Result<(), String> {
        if self.checkpoint_id != checkpoint_id
            || self.role != role
            || self.occupancy_receipt != *occupancy_receipt
            || self.cloud_receipt.as_ref() != cloud_receipt
            || self.frame_id != frame_id
        {
            return Err("assigned member descriptor differs from its frozen parent".into());
        }
        Ok(())
    }
}

/// 有限点的实际坐标范围；不添加地图物理尺寸上限。
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct CanonicalCloudBoundsV2 {
    pub(crate) min: [f32; 3],
    pub(crate) max: [f32; 3],
}

#[derive(Debug)]
struct CanonicalCloudTargetDataV2 {
    member: CheckpointMemberIdentityV2,
    source_revision: u64,
    point_count: u32,
    voxel_size: f32,
    payload_schema_version: u32,
    bounds: CanonicalCloudBoundsV2,
    xyz_f32_le: Vec<u8>,
}

/// Vec 所有权直接移入 Arc 的私有对象；克隆目标只共享对象，不复制完整点云。
/// 不暴露可变引用或 Arc 内部，因此下游无法改写已准入的字节及收据。
#[derive(Clone, Debug)]
pub(crate) struct CanonicalCloudTargetV2(Arc<CanonicalCloudTargetDataV2>);

impl PartialEq for CanonicalCloudTargetV2 {
    fn eq(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
            || (self.0.member == other.0.member
                && self.0.source_revision == other.0.source_revision
                && self.0.point_count == other.0.point_count
                && self.0.voxel_size.to_bits() == other.0.voxel_size.to_bits()
                && self.0.payload_schema_version == other.0.payload_schema_version
                && self.0.xyz_f32_le == other.0.xyz_f32_le)
    }
}

// 准入排除了非有限描述值；上述比较仍保留描述浮点及载荷的精确位语义。
impl Eq for CanonicalCloudTargetV2 {}

fn checked_cloud_size(point_count: u32) -> Result<usize, String> {
    if point_count == 0 {
        return Err("immutable target must contain at least one point".into());
    }
    let size = usize::try_from(point_count)
        .map_err(|_| "immutable target point_count exceeds usize")?
        .checked_mul(CM_POINT_SIZE_BYTES)
        .ok_or("immutable target payload size overflow")?;
    if size > IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES {
        return Err(format!(
            "immutable target exceeds {IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES} bytes: {size}"
        ));
    }
    Ok(size)
}

impl CanonicalCloudTargetV2 {
    pub(crate) fn admit(
        checkpoint: proto::CloudMapCheckpoint,
        expected_parent_id: CheckpointIdentityV2,
        expected_occupancy_receipt: &OccupancyCommitReceipt,
        expected_cloud_receipt: &CloudCommitReceipt,
        expected_map_frame_id: &str,
    ) -> Result<Self, String> {
        if checkpoint.payload_schema_version != 2 || !checkpoint.xyz_sha256.is_empty() {
            return Err("assigned cloud target requires unmixed identity schema 2".into());
        }
        if checkpoint.frame_id.is_empty()
            || checkpoint.frame_id.len() > 256
            || checkpoint.frame_id.trim() != checkpoint.frame_id
            || checkpoint.frame_id.chars().any(char::is_control)
            || checkpoint.frame_id != expected_map_frame_id
        {
            return Err("immutable target frame must match the canonical map frame".into());
        }
        if checkpoint.source_revision == 0
            || checkpoint.source_revision != expected_cloud_receipt.cloud_revision
            || checkpoint.point_count != expected_cloud_receipt.point_count
        {
            return Err("immutable target geometry differs from its cloud receipt".into());
        }
        if !checkpoint.voxel_size.is_finite()
            || checkpoint.voxel_size <= 0.0
            || checkpoint.voxel_size > MAX_VOXEL_SIZE
        {
            return Err("immutable target voxel_size must be finite, positive and within the existing limit".into());
        }
        let expected_size = checked_cloud_size(checkpoint.point_count)?;
        if checkpoint.xyz_f32_le.len() != expected_size {
            return Err(format!(
                "immutable target payload size mismatch: expected={expected_size} actual={}",
                checkpoint.xyz_f32_le.len()
            ));
        }
        let member = CheckpointMemberIdentityV2::try_from(
            checkpoint
                .member_identity
                .ok_or("assigned cloud target member identity is required")?,
        )?;
        member.validate_parent(
            expected_parent_id,
            CheckpointMemberRole::CanonicalCloud,
            expected_occupancy_receipt,
            Some(expected_cloud_receipt),
            expected_map_frame_id,
        )?;

        let mut bounds = CanonicalCloudBoundsV2 {
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
        };
        for (index, point) in checkpoint
            .xyz_f32_le
            .chunks_exact(CM_POINT_SIZE_BYTES)
            .enumerate()
        {
            for (axis, coordinate) in point.chunks_exact(4).enumerate() {
                let value = f32::from_le_bytes(
                    coordinate
                        .try_into()
                        .map_err(|_| "immutable target coordinate size is invalid")?,
                );
                if !value.is_finite() {
                    return Err(format!("immutable target point {index} is non-finite"));
                }
                bounds.min[axis] = bounds.min[axis].min(value);
                bounds.max[axis] = bounds.max[axis].max(value);
            }
        }
        Ok(Self(Arc::new(CanonicalCloudTargetDataV2 {
            member,
            source_revision: checkpoint.source_revision,
            point_count: checkpoint.point_count,
            voxel_size: checkpoint.voxel_size,
            payload_schema_version: checkpoint.payload_schema_version,
            bounds,
            xyz_f32_le: checkpoint.xyz_f32_le,
        })))
    }

    pub(crate) fn member(&self) -> &CheckpointMemberIdentityV2 {
        &self.0.member
    }
    pub(crate) fn source_revision(&self) -> u64 {
        self.0.source_revision
    }
    pub(crate) fn point_count(&self) -> u32 {
        self.0.point_count
    }
    pub(crate) fn voxel_size(&self) -> f32 {
        self.0.voxel_size
    }
    pub(crate) fn payload_schema_version(&self) -> u32 {
        self.0.payload_schema_version
    }
    pub(crate) fn bounds(&self) -> CanonicalCloudBoundsV2 {
        self.0.bounds
    }
    pub(crate) fn xyz_f32_le(&self) -> &[u8] {
        &self.0.xyz_f32_le
    }

    /// 相同分配身份不能覆盖不同描述或字节；同一共享对象免除重复字节比较。
    pub(crate) fn validate_replay(&self, candidate: &Self) -> Result<(), String> {
        if self != candidate {
            return Err("immutable assigned cloud target replay conflict".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHECKPOINT_ID: &str = "00112233445566778899aabbccddeeff";

    fn wire_member() -> proto::CheckpointMemberIdentity {
        let commit_id = proto::MappingCommitIdentity {
            boot_nonce: (1..=16).collect(),
            sequence: 5,
        };
        proto::CheckpointMemberIdentity {
            checkpoint_id: CHECKPOINT_ID.into(),
            role: CheckpointMemberRole::CanonicalCloud as i32,
            occupancy_receipt: Some(proto::CanonicalOccupancyCommitReceipt {
                commit_id: Some(commit_id.clone()),
                session_id: "source-session".into(),
                session_generation: 7,
                occupancy_revision: 11,
                source_timestamp_ns: 100,
                backend_kind: "voxel".into(),
                backend_generation: 3,
            }),
            cloud_receipt: Some(proto::CanonicalCloudCommitReceipt {
                commit_id: Some(commit_id),
                session_generation: 7,
                occupancy_revision: 11,
                cloud_revision: 9,
                source_timestamp_ns: 100,
                point_count: 2,
            }),
            frame_id: "map".into(),
            frame_generation: 0,
            producer_generation: 0,
        }
    }

    fn wire_target() -> proto::CloudMapCheckpoint {
        proto::CloudMapCheckpoint {
            frame_id: "map".into(),
            source_revision: 9,
            point_count: 2,
            voxel_size: 0.1,
            xyz_f32_le: [-1.0_f32, 2.0, 3.5, 4.0, -5.0, 6.0]
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect(),
            xyz_sha256: String::new(),
            payload_schema_version: 2,
            member_identity: Some(wire_member()),
        }
    }

    fn parent() -> CheckpointMemberIdentityV2 {
        CheckpointMemberIdentityV2::try_from(wire_member()).unwrap()
    }

    fn admit(checkpoint: proto::CloudMapCheckpoint) -> Result<CanonicalCloudTargetV2, String> {
        let parent = parent();
        CanonicalCloudTargetV2::admit(
            checkpoint,
            parent.checkpoint_id(),
            parent.occupancy_receipt(),
            parent.cloud_receipt().unwrap(),
            parent.frame_id(),
        )
    }

    #[test]
    fn assigned_ids_reuse_strict_checkpoint_encoding_and_wire_roundtrip() {
        let id = AssetIdentityV2::parse(CHECKPOINT_ID).unwrap();
        assert_eq!(id.to_wire().asset_id, CHECKPOINT_ID);
        assert_eq!(AssetIdentityV2::try_from(&id.to_wire()).unwrap(), id);
        let checkpoint_id = CheckpointIdentityV2::parse(CHECKPOINT_ID).unwrap();
        assert_eq!(checkpoint_id.to_wire_id(), CHECKPOINT_ID);
        for invalid in [
            String::new(),
            "0".repeat(32),
            "1".repeat(31),
            "1".repeat(33),
            CHECKPOINT_ID.to_uppercase(),
            format!(" {CHECKPOINT_ID}"),
            format!("{CHECKPOINT_ID}\n"),
            "00112233-4455-6677-8899-aabbccddeeff00".into(),
            "g".repeat(32),
            "é".repeat(16),
        ] {
            assert!(
                AssetIdentityV2::parse(&invalid).is_err(),
                "accepted {invalid:?}"
            );
            assert!(
                CheckpointIdentityV2::parse(&invalid).is_err(),
                "accepted {invalid:?}"
            );
        }
    }

    #[test]
    fn typed_members_preserve_exact_receipts_and_lagging_cloud_order() {
        let member = parent();
        assert_eq!(
            member.occupancy_receipt().commit_id.boot_nonce,
            u128::from_be_bytes((1..=16).collect::<Vec<_>>().try_into().unwrap())
        );
        assert_eq!(member.role(), CheckpointMemberRole::CanonicalCloud);
        assert_eq!(member.frame_generation(), 0);
        assert_eq!(member.producer_generation(), 0);
        let mut wire = wire_member();
        let occupancy = wire.occupancy_receipt.as_mut().unwrap();
        occupancy.commit_id.as_mut().unwrap().sequence += 1;
        occupancy.occupancy_revision += 1;
        occupancy.source_timestamp_ns += 1;
        assert!(CheckpointMemberIdentityV2::try_from(wire.clone()).is_ok());
        wire.cloud_receipt.as_mut().unwrap().occupancy_revision += 1;
        assert!(CheckpointMemberIdentityV2::try_from(wire).is_err());
    }

    #[test]
    fn member_roles_frames_and_paired_receipts_are_validated() {
        let mut cases = Vec::new();
        let mut value = wire_member();
        value.role = 0;
        cases.push(value);
        let mut value = wire_member();
        value.role = 99;
        cases.push(value);
        let mut value = wire_member();
        value.frame_id = " map".into();
        cases.push(value);
        let mut value = wire_member();
        value.frame_id = "ma\np".into();
        cases.push(value);
        let mut value = wire_member();
        value.occupancy_receipt = None;
        cases.push(value);
        let mut value = wire_member();
        value.cloud_receipt = None;
        cases.push(value);
        let mut value = wire_member();
        value.frame_generation = 1;
        cases.push(value);
        let mut value = wire_member();
        value.producer_generation = 1;
        cases.push(value);
        let mut value = wire_member();
        value.cloud_receipt.as_mut().unwrap().source_timestamp_ns += 1;
        cases.push(value);
        let mut value = wire_member();
        value
            .cloud_receipt
            .as_mut()
            .unwrap()
            .commit_id
            .as_mut()
            .unwrap()
            .boot_nonce[0] += 1;
        cases.push(value);
        let mut value = wire_member();
        value
            .cloud_receipt
            .as_mut()
            .unwrap()
            .commit_id
            .as_mut()
            .unwrap()
            .sequence = 0;
        cases.push(value);
        let mut value = wire_member();
        value
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .source_timestamp_ns = 0;
        cases.push(value);
        for value in cases {
            assert!(CheckpointMemberIdentityV2::try_from(value).is_err());
        }
    }

    #[test]
    fn typed_members_preserve_existing_fine_and_planar_ownership_rules() {
        for role in [
            CheckpointMemberRole::BackendState,
            CheckpointMemberRole::CanonicalOccupancy,
            CheckpointMemberRole::CanonicalCloud,
            CheckpointMemberRole::DenseCloud,
        ] {
            let mut value = wire_member();
            value.role = role as i32;
            assert_eq!(
                CheckpointMemberIdentityV2::try_from(value).unwrap().role(),
                role
            );
        }
        let mut fine = wire_member();
        fine.role = CheckpointMemberRole::FineMap as i32;
        assert!(CheckpointMemberIdentityV2::try_from(fine.clone()).is_err());
        fine.frame_generation = 13;
        fine.producer_generation = 2;
        let fine = CheckpointMemberIdentityV2::try_from(fine).unwrap();
        assert_eq!(fine.frame_generation(), 13);
        assert_eq!(fine.producer_generation(), 2);

        let mut planar = wire_member();
        planar.role = CheckpointMemberRole::PlanarGraph as i32;
        planar.occupancy_receipt.as_mut().unwrap().backend_kind = "slam_toolbox".into();
        assert!(CheckpointMemberIdentityV2::try_from(planar.clone()).is_err());
        planar.cloud_receipt = None;
        assert!(CheckpointMemberIdentityV2::try_from(planar)
            .unwrap()
            .cloud_receipt()
            .is_none());
    }

    #[test]
    fn cloud_admission_matches_role_frame_parent_and_both_receipts_exactly() {
        let mut cases = Vec::new();
        let mut value = wire_target();
        value.member_identity = None;
        cases.push(value);
        let mut value = wire_target();
        value.member_identity.as_mut().unwrap().checkpoint_id = "1".repeat(32);
        cases.push(value);
        let mut value = wire_target();
        value.member_identity.as_mut().unwrap().role = CheckpointMemberRole::DenseCloud as i32;
        cases.push(value);
        let mut value = wire_target();
        value.member_identity.as_mut().unwrap().frame_id = "odom".into();
        cases.push(value);
        let mut value = wire_target();
        value.frame_id = "odom".into();
        cases.push(value);
        let mut value = wire_target();
        value
            .member_identity
            .as_mut()
            .unwrap()
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .backend_generation += 1;
        cases.push(value);
        let mut value = wire_target();
        value
            .member_identity
            .as_mut()
            .unwrap()
            .cloud_receipt
            .as_mut()
            .unwrap()
            .cloud_revision += 1;
        cases.push(value);
        let mut value = wire_target();
        value.source_revision += 1;
        cases.push(value);
        let mut value = wire_target();
        value.point_count += 1;
        cases.push(value);
        for value in cases {
            assert!(admit(value).is_err());
        }
    }

    #[test]
    fn immutable_admission_moves_bytes_and_shares_owned_receipts() {
        let checkpoint = wire_target();
        let allocation = checkpoint.xyz_f32_le.as_ptr();
        let target = admit(checkpoint).unwrap();
        assert_eq!(target.xyz_f32_le().as_ptr(), allocation);
        let clone = target.clone();
        assert!(Arc::ptr_eq(&target.0, &clone.0));
        assert_eq!(clone.xyz_f32_le().as_ptr(), allocation);
        assert!(std::ptr::eq(
            target.member().occupancy_receipt(),
            clone.member().occupancy_receipt()
        ));
        assert_eq!(target.source_revision(), 9);
        assert_eq!(target.point_count(), 2);
        assert_eq!(target.voxel_size(), 0.1);
        assert_eq!(target.payload_schema_version(), 2);
        assert_eq!(
            target.bounds(),
            CanonicalCloudBoundsV2 {
                min: [-1.0, -5.0, 3.5],
                max: [4.0, 2.0, 6.0]
            }
        );
        drop(target);
        assert_eq!(clone.xyz_f32_le().as_ptr(), allocation);
        assert_eq!(clone.member().cloud_receipt().unwrap().cloud_revision, 9);
    }

    #[test]
    fn same_identity_cannot_replay_altered_bytes_or_descriptors() {
        let target = admit(wire_target()).unwrap();
        target.validate_replay(&target.clone()).unwrap();
        target
            .validate_replay(&admit(wire_target()).unwrap())
            .unwrap();
        let mut changed = wire_target();
        changed.xyz_f32_le[..4].copy_from_slice(&1.0_f32.to_le_bytes());
        assert!(target.validate_replay(&admit(changed).unwrap()).is_err());
        let mut changed = wire_target();
        changed.voxel_size = 0.2;
        assert!(target.validate_replay(&admit(changed).unwrap()).is_err());

        // 数值相等也不能隐藏字节改变。
        let mut positive_zero = wire_target();
        positive_zero.xyz_f32_le[..4].copy_from_slice(&0.0_f32.to_le_bytes());
        let mut negative_zero = wire_target();
        negative_zero.xyz_f32_le[..4].copy_from_slice(&(-0.0_f32).to_le_bytes());
        assert_ne!(admit(positive_zero).unwrap(), admit(negative_zero).unwrap());

        // 即使使用另一组内部一致的父收据准入，同一 ID 也不能重放不同收据。
        let mut changed = wire_target();
        let member = changed.member_identity.as_mut().unwrap();
        member
            .occupancy_receipt
            .as_mut()
            .unwrap()
            .backend_generation += 1;
        let new_parent = CheckpointMemberIdentityV2::try_from(member.clone()).unwrap();
        let changed = CanonicalCloudTargetV2::admit(
            changed,
            new_parent.checkpoint_id(),
            new_parent.occupancy_receipt(),
            new_parent.cloud_receipt().unwrap(),
            new_parent.frame_id(),
        )
        .unwrap();
        assert!(target.validate_replay(&changed).is_err());
    }

    #[test]
    fn admission_preserves_existing_byte_frame_voxel_and_finite_geometry_limits() {
        assert_eq!(
            checked_cloud_size((IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES / CM_POINT_SIZE_BYTES) as u32)
                .unwrap(),
            IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES
        );
        assert!(checked_cloud_size(0).is_err());
        assert!(checked_cloud_size(
            (IMMUTABLE_ACTIVATED_TARGET_MAX_BYTES / CM_POINT_SIZE_BYTES + 1) as u32
        )
        .is_err());
        assert!(checked_cloud_size(u32::MAX).is_err());
        for voxel in [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            0.0,
            -0.0,
            -1.0,
            MAX_VOXEL_SIZE * 2.0,
        ] {
            let mut value = wire_target();
            value.voxel_size = voxel;
            assert!(admit(value).is_err());
        }
        let mut value = wire_target();
        value.voxel_size = MAX_VOXEL_SIZE;
        assert!(admit(value).is_ok());
        for coordinate in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            for axis in 0..6 {
                let mut value = wire_target();
                value.xyz_f32_le[axis * 4..axis * 4 + 4].copy_from_slice(&coordinate.to_le_bytes());
                assert!(admit(value).is_err());
            }
        }
        let mut value = wire_target();
        value.xyz_f32_le.push(0);
        assert!(admit(value).is_err());
        let mut value = wire_target();
        value.xyz_f32_le.pop();
        assert!(admit(value).is_err());
        for length in [0, 256, 257] {
            let mut value = wire_target();
            value.frame_id = "m".repeat(length);
            value.member_identity.as_mut().unwrap().frame_id = value.frame_id.clone();
            let parent = parent();
            let frame = value.frame_id.clone();
            let result = CanonicalCloudTargetV2::admit(
                value,
                parent.checkpoint_id(),
                parent.occupancy_receipt(),
                parent.cloud_receipt().unwrap(),
                &frame,
            );
            assert_eq!(result.is_ok(), length == 256);
        }
        let mut value = wire_target();
        value.xyz_f32_le[..4].copy_from_slice(&f32::MAX.to_le_bytes());
        assert_eq!(admit(value).unwrap().bounds().max[0], f32::MAX);
    }

    #[test]
    fn assigned_target_never_accepts_legacy_or_mixed_identity_envelopes() {
        for version in [0, 1, 3] {
            let mut value = wire_target();
            value.payload_schema_version = version;
            assert!(admit(value).is_err());
        }
        let mut value = wire_target();
        value.xyz_sha256 = "a".repeat(64);
        assert!(admit(value).is_err());
    }
}
