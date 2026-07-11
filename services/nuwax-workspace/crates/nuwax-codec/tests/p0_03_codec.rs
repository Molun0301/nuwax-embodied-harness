//! P0-03 回归测试:wire-format 一致(LCM 1.x 字节序 + `expected_lcm_bytes` 物理删除)。
//!
//! ## 背景 (2026-07-11 rework22)
//!
//! 用户决策(rework22 P0-03):**不做双向 golden**(不与 Python/C++ LCM
//! 字节级对齐)。本测试只保证:
//!
//! 1. `expected_lcm_bytes` 函数已被**物理删除**(恒等 None 占位函数移除);
//! 2. Pose / Odometry / PointCloud2 wire-format encoder 输出 LCM 1.x 标准
//!    大端字节序 + 字段顺序与 `lcm-gen` 生成的 `*.c` 文件一致;
//! 3. encoder 输出**不**含 envelope(只 payload 部分),由 liblcm 在 wire
//!    上自行加 8 字节 preamble。
//!
//! ## 字节级断言来源
//!
//! - 1.0f64 IEEE-754 BE = 0x3FF0000000000000
//!   = [0x3F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
//! - 1.5f64 IEEE-754 BE = 0x3FF8000000000000
//!   = [0x3F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
//! - 上游 `lcm-gen` 生成的 `geometry_msgs__Pose.c` 中 `__encode_array` 字段顺序:
//!   `position_x, position_y, position_z, orientation_w, orientation_x,
//!    orientation_y, orientation_z` —— `w` 在前与 ROS geometry_msgs 习惯
//!   `(x, y, z, w)` **不同**,必须保留。
//!
//! ## 不在范围内
//!
//! - 与 Python `lcmtypes/generated/python/geometry_msgs__Pose.py` 的字节
//!   级双向 golden(用户决策不做)。

use nuwax_codec::lcm::{
    encode_odom_minimal, encode_pointcloud2_stub, encode_pose, golden_lcm_message,
};

/// Pose wire-format:7 个 f64 BE = 56 字节,顺序 (x,y,z,qw,qx,qy,qz)。
#[test]
fn p0_03_pose_wire_format_is_56_bytes_big_endian() {
    let position = [1.0f64, 2.0f64, 0.5f64];
    let orientation_wxyz = [0.7f64, 0.1f64, 0.2f64, 0.3f64]; // (qw, qx, qy, qz)

    let bytes = encode_pose(position, orientation_wxyz);

    assert_eq!(
        bytes.len(),
        56,
        "Pose wire-format 必须为 56 字节(7 × f64 BE),got {}",
        bytes.len()
    );

    assert_eq!(
        &bytes[0..8],
        &[0x3F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        "position_x = 1.0f64 BE 字节序"
    );
    assert_eq!(
        &bytes[8..16],
        &[0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        "position_y = 2.0f64 BE 字节序"
    );
    assert_eq!(
        &bytes[16..24],
        &[0x3F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        "position_z = 0.5f64 BE 字节序"
    );
    // qw 在前:qw = 0.7f64 BE = 0x3FE6666666666666
    assert_eq!(
        &bytes[24..32],
        &[0x3F, 0xE6, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66],
        "orientation_w = 0.7f64 BE 字节序 (w 在前)"
    );
    assert_ne!(&bytes[32..40], &[0u8; 8], "orientation_x 不应全 0");
    assert_ne!(&bytes[40..48], &[0u8; 8], "orientation_y 不应全 0");
    assert_ne!(&bytes[48..56], &[0u8; 8], "orientation_z 不应全 0");
}

/// Odometry minimal wire-format:13 × f64 BE = 104 字节。
#[test]
fn p0_03_odom_wire_format_is_104_bytes_big_endian() {
    let pose_pos = [0.1f64, 0.2f64, 0.3f64];
    let pose_quat = [1.0f64, 0.0f64, 0.0f64, 0.0f64]; // 单位四元数 (qw=1)
    let twist_lin = [1.5f64, 0.0f64, 0.0f64];
    let twist_ang = [0.0f64, 0.0f64, 0.5f64];

    let bytes = encode_odom_minimal(pose_pos, pose_quat, twist_lin, twist_ang);

    assert_eq!(
        bytes.len(),
        104,
        "Odometry minimal wire-format 必须为 104 字节(13 × f64 BE),got {}",
        bytes.len()
    );
    // 字段顺序:pose_pos(3) + pose_quat(4) + twist_lin(3) + twist_ang(3) = 13 × f64。
    // pose_quat 区段起始偏移 = 3 × 8 = 24;qw 是该区段第一个 f64。
    assert_eq!(
        &bytes[24..32],
        &[0x3F, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        "pose.orientation_w = 1.0f64 BE (qw 在前,位于 pose_quat 区段开头)"
    );
    // twist_lin 区段起始偏移 = (3+4) × 8 = 56;linear.x 是该区段第一个 f64。
    assert_eq!(
        &bytes[56..64],
        &[0x3F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        "twist.linear.x = 1.5f64 BE"
    );
}

/// PointCloud2 stub wire-format:u32 BE len + UTF-8 bytes + 3 × u32 BE。
#[test]
fn p0_03_pointcloud2_stub_wire_format_len_prefixed_string_then_u32_fields() {
    let frame_id = "map";
    let width = 640u32;
    let height = 480u32;
    let point_step = 32u32;

    let bytes = encode_pointcloud2_stub(frame_id, width, height, point_step);

    // 长度 = 4 (u32 len) + 3 ("map") + 12 (3 × u32) = 19 字节
    assert_eq!(
        bytes.len(),
        4 + frame_id.len() + 12,
        "PointCloud2 stub = u32 len + frame_id bytes + 3 × u32 BE"
    );

    // u32 BE 长度字段 = 3
    assert_eq!(
        &bytes[0..4],
        &[0x00, 0x00, 0x00, 0x03],
        "frame_id 长度字段必须为 u32 BE"
    );
    // UTF-8 bytes
    assert_eq!(&bytes[4..7], b"map", "frame_id UTF-8 bytes");
    // width = 640u32 BE
    assert_eq!(
        &bytes[7..11],
        &[0x00, 0x00, 0x02, 0x80],
        "width = 640u32 BE"
    );
    // height = 480u32 BE = 0x000001E0
    assert_eq!(
        &bytes[11..15],
        &[0x00, 0x00, 0x01, 0xE0],
        "height = 480u32 BE"
    );
    // point_step = 32u32 BE = 0x00000020
    assert_eq!(
        &bytes[15..19],
        &[0x00, 0x00, 0x00, 0x20],
        "point_step = 32u32 BE"
    );
}

/// Pose payload 可被 `golden_lcm_message` 包装成完整 LCM wire format
/// (8 字节 preamble + 56 字节 payload = 64 字节)。
#[test]
fn p0_03_pose_payload_combines_with_lcm_preamble() {
    let bytes = encode_pose([0.0f64; 3], [1.0f64, 0.0f64, 0.0f64, 0.0f64]);
    let wire = golden_lcm_message(
        "/pose",
        "geometry_msgs/PoseStamped", // STRUCT_SEEDS 已注册
        &bytes,
        1,
    )
    .expect("golden_lcm_message ok");

    assert_eq!(wire.len(), 8 + 56, "wire = 8 preamble + 56 payload");
    assert_eq!(&wire[0..4], b"LC02");
    assert_eq!(&wire[4..8], &[0x00, 0x00, 0x00, 0x01]);
    // payload 前 8 字节 = position_x = 0.0 BE
    assert_eq!(&wire[8..16], &[0u8; 8]);
}

/// `expected_lcm_bytes` 函数已**物理删除**(用户决策 rework22 P0-03):
/// 该测试通过 `include_str!` 把 `../src/lcm.rs` 源码嵌入,断言
/// 不含 `pub fn expected_lcm_bytes` 定义。若未来有人恢复该函数,
/// 此断言会失败;CI 阶段由 `grep -R "pub fn expected_lcm_bytes" crates/`
/// 直接 fail —— 双保险。
#[test]
fn p0_03_expected_lcm_bytes_physically_removed() {
    let src = include_str!("../src/lcm.rs");
    assert!(
        !src.contains("pub fn expected_lcm_bytes"),
        "P0-03 硬约束:expected_lcm_bytes 已物理删除,不得再恢复"
    );
    // 同时,模块不再 `pub use expected_lcm_bytes`(防御性)
    // —— 通过 `pub fn` 与 `pub use` 两种形态同时排除。
    assert!(
        !src.contains("pub use ") || !src.contains("expected_lcm_bytes"),
        "expected_lcm_bytes 也不应被 pub use 重新导出"
    );
}
