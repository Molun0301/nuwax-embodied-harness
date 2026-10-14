//! `nuwax-motion-safety-guard` private actuator authority.
//!
//! This reference crate contains the generated private-UDS contract, durable
//! boot epochs, lease arbitration, and a fail-closed actuator boundary. The
//! production socket/SDK owner is the Python guard process because the official
//! Go2 SDK in this repository is `unitree_sdk2py`; this crate does not bind the
//! production socket or provide a competing binary.
//!
//! **与 Python v2 guard 对齐**：
//! - v2 private-UDS contract
//! - 5 ack 完成点(`ACK_COMPLETION_POINTS`)
//! - 5 source(`SOURCE_ALLOWLIST`) + 7 action(`ACTION_ALLOWLIST`)
//! - lease-only arbitration(`state_machine.MotionResourceArbitrator`)
//! - dedup cache + priority reserve(`dedup_cache.DedupCache`)
//! - operation namespace(4 类:`operation_namespace.OperationNamespace`)
//! - epoch.json schema(`w6_skeleton.GuardEpochRecord`)
//! - 3 UID + 1 group(`uids_provisioning.EXPECTED_USERS` / `EXPECTED_GROUPS`)
//! - 时序常量:`LEASE_RENEWAL_WINDOW_MS=600_000` / `RENEW_INTERVAL_MS=250` /
//!   `OWNER_TIMEOUT_MS=750` / `COMMAND_DEADLINE_MS=300`
//!
//! **模块布局**：
//! - [`wire`]：由 `services/navigation/proto/motion_guard.proto` 生成的唯一 wire 类型面
//! - [`state`]：LeaseState + MotionResourceArbitrator
//!   (对应 `state_machine.py`)
//! - [`namespace`]：operation namespace(4 类 key + canonical 表达 + payload hash)
//!   (对应 `operation_namespace.py`)
//! - [`cache`]：dedup cache + priority reserve(对应 `dedup_cache.py`)
//! - [`epoch`]：epoch.json schema + provisioned.json schema + 3 UID + 1 group
//!   (对应 `w6_skeleton.GuardEpochRecord` + `uids_provisioning.py`)
//! - [`constants`]：FROZEN 时序常量 + UDS 路径 + 默认容量
//!   (对应 `w6_skeleton.MotionGuard.*` + `w29_uds_constants.py`)
//!
//! **测试**覆盖:
//! - typed request/response protobuf roundtrip
//! - canonical checksum 与 corrupt/missing epoch fail-closed
//! - lease exclusivity + ordinary global stop
//! - priority reserve 物理隔离
//! - 时序常量与 W32 Python 端 1:1 对齐
//! - 3 UID / 1 group 与 sysusers.d 1:1 对齐

#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod actuator;
pub mod cache;
pub mod client;
pub mod constants;
mod crypto;
pub mod epoch;
pub mod namespace;
pub mod persistence;
pub mod state;
pub mod wire;

// 顶层 re-export：让 `nuwax_motion_safety_guard::*` 一行能用上核心枚举与 trait
pub use actuator::{ActuatorBackend, ActuatorError, UnavailableActuatorBackend};
pub use cache::{CacheHealth, DedupCache, DedupResult};
pub use client::{
    ClientConfig, CommandConfirmation, GlobalStopConfirmation, HeldRecoveryFenceProof,
    LeaseMetadata, MotionAdmissionProof, MotionGuardClient, MotionGuardClientError,
    PriorityFenceRequest, ReadinessStatus, RenewalMetadata, ZeroConfirmation,
};
pub use constants::{
    COMMAND_DEADLINE_MS, DEFAULT_EPOCH_PATH, DEFAULT_REQUEST_TIMEOUT_MS, DEFAULT_UDS_PATH,
    LEASE_RENEWAL_WINDOW_MS, MAX_FRAME_BYTES, OWNER_TIMEOUT_MS, RENEW_INTERVAL_MS,
    RENEW_TO_OWNER_TIMEOUT_RATIO,
};
pub use epoch::{
    GuardEpochRecord, ProvisionedRecord, EXPECTED_GROUPS, EXPECTED_USERS, PROVISIONED_PATH,
    SCHEMA_VERSION_EPOCH, SCHEMA_VERSION_PROVISIONED,
};
pub use namespace::{
    derive_payload_hash, key_to_canonical, AcquireKeyBuilder, GlobalStopKeyBuilder,
    LeaseBoundKeyBuilder, OperationKey, OperationNamespace, QueryKeyBuilder,
};
pub use state::{LeaseState, MotionResourceArbitrator};
