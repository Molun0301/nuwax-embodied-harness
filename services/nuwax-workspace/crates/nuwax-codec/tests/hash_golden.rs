//! MurmurHash3 x64 (seed=0) 固化 fixture 测试
//!
//! 5 个 fixture 由 upstream SMhasher canonical MurmurHash3_x64_128
//! reference C 实现,在 2026-07-07 由 `scripts/ci/regen_lcm_goldens.sh`
//! 跑出固化。每个 fixture 都对应一个**已知** SMhasher 测试向量或一个
//! 真实使用场景(geometry_msgs/Twist = 即将被 `lcm_hash_64` 包裹的类型名)。
//!
//! **不要**手工修改这里的期望值;若 fixture 失败,先校验
//! `nuwax-codec::hash::murmur_hash64a` 实现是否与 SMhasher 对齐。

use nuwax_codec::hash::murmur_hash64a;

/// 5 个固化 fixture:
/// - "hello" / empty / 16-byte zeros / 32-byte zeros / 32-byte 0xff
/// - "geometry_msgs/Twist" 作为面向 LCM 的特殊语义 fixture
const FIXTURES: &[(&str, &[u8], u64)] = &[
    ("hello", b"hello", 0xcbd8a7b341bd9b02),
    ("empty", b"", 0x0000000000000000),
    ("zero_16", &[0u8; 16], 0x4bbd1bf27da918d6),
    ("zero_32", &[0u8; 32], 0x31723c284ade5cd0),
    ("ff_32", &[0xffu8; 32], 0x7592e77dc55113f1),
];

#[test]
fn murmur_hash64a_fixtures_match_smhasher_reference() {
    for (name, input, expected) in FIXTURES {
        let got = murmur_hash64a(input);
        assert_eq!(
            got, *expected,
            "fixture {name} failed: got 0x{got:016x}, expected 0x{expected:016x}"
        );
    }
}

#[test]
fn murmur_hash64a_geometry_msgs_twist() {
    // SMhasher reference output for 20-byte "geometry_msgs/Twist"
    let got = murmur_hash64a(b"geometry_msgs/Twist");
    assert_eq!(got, 0x93a62df428b5c932);
}

#[test]
fn murmur_hash64a_determinism() {
    // 同一输入两次必须产生相同输出(对 fake-backend 内部稳定性至关重要)
    let a = murmur_hash64a(b"nuwax_msgs/NavigationState");
    let b = murmur_hash64a(b"nuwax_msgs/NavigationState");
    assert_eq!(a, b);
}

#[test]
fn murmur_hash64a_avalanche_distinct_inputs_differ() {
    // 几个相邻 FQN 的 MurmurHash3 输出应该全不同
    let inputs: &[&[u8]] = &[
        b"geometry_msgs/PoseStamped",
        b"geometry_msgs/Twist",
        b"geometry_msgs/Transform",
        b"nav_msgs/Odometry",
    ];
    for i in 0..inputs.len() {
        for j in (i + 1)..inputs.len() {
            assert_ne!(
                murmur_hash64a(inputs[i]),
                murmur_hash64a(inputs[j]),
                "collision on inputs[{i}] vs inputs[{j}]"
            );
        }
    }
}