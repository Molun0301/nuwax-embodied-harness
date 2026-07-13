// M #3 (RALPLAN §Phase 2): MapRevision checksum helper。
//
// 真实 SHA-256 字节级与 Python `hashlib.sha256(d).hexdigest()` 一致,
// 是 W29 Python 端 `MapStreamEventBroker.StreamSnapshot.parsed["checksum"]`
// 期望的真正哈希(不是伪造 sentinel,FakeBackend 已物理删除)。
//
// W41 fix: 原 W37 草稿 `use sha2::{Digest, Sha256};` 走 `sha2` crate
// 但 `nuwax-geometry-runtime/Cargo.toml` 未引入该 crate(workspace 也
// 没有),cargo build 会 fail-fast "unresolved import `sha2`"。本文件
// 直接内联一份 FIPS-180-4 SHA-256 标准实现,无外部依赖,字节级输出
// 与 `sha2` crate / `hashlib.sha256()` / `openssl sha256` 完全一致。
//
// 参考: FIPS 180-4 §6.2 SHA-256;测试向量: NIST CAVP。

/// SHA-256 hex 字符串(64 个小写 hex 字符)。空输入也合法,直接给
/// `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`。
#[must_use]
pub fn sha256_hex(data: &[u8]) -> String {
    let out = sha256_bytes(data);
    let mut hex_buf = String::with_capacity(out.len() * 2);
    for byte in out {
        use std::fmt::Write as _;
        let _ = write!(&mut hex_buf, "{byte:02x}");
    }
    hex_buf
}

/// SHA-256 原始 32 字节摘要。
#[must_use]
pub fn sha256_bytes(data: &[u8]) -> [u8; 32] {
    // FIPS 180-4 §5.3.3 SHA-256 初始哈希值 H(0)
    const H_INIT: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    // FIPS 180-4 §4.2.2 SHA-256 轮常量 K[0..63]
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];

    // FIPS 180-4 §5.1.1 填充: 原消息 + 0x80 + 0x00... + 64-bit 大端长度
    let bit_len = (data.len() as u64).wrapping_mul(8);
    let mut msg = data.to_vec();
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0x00);
    }
    msg.extend_from_slice(&bit_len.to_be_bytes());

    // FIPS 180-4 §6.2.2 主循环
    let mut h = H_INIT;
    for chunk in msg.chunks_exact(64) {
        let mut w = [0u32; 64];
        for i in 0..16 {
            w[i] = u32::from_be_bytes([
                chunk[i * 4],
                chunk[i * 4 + 1],
                chunk[i * 4 + 2],
                chunk[i * 4 + 3],
            ]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh] = h;
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ ((!e) & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let mj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(mj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
        h[5] = h[5].wrapping_add(f);
        h[6] = h[6].wrapping_add(g);
        h[7] = h[7].wrapping_add(hh);
    }

    let mut out = [0u8; 32];
    for (i, &v) in h.iter().enumerate() {
        out[i * 4..i * 4 + 4].copy_from_slice(&v.to_be_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sha256_hex_empty_matches_rfc6234() {
        // RFC 6234 / FIPS 180-4: SHA-256("") =
        // e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855
        assert_eq!(
            sha256_hex(b""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
            "SHA-256 空输入与 FIPS 180-4 一致",
        );
    }

    #[test]
    fn sha256_hex_abc_matches_python_hashlib() {
        // Python: hashlib.sha256(b"abc").hexdigest()
        //        = 'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad'
        assert_eq!(
            sha256_hex(b"abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            "SHA-256(\"abc\") 与 Python hashlib 一致",
        );
    }

    #[test]
    fn sha256_hex_stable_same_input_same_output() {
        let a = sha256_hex(b"hello world");
        let b = sha256_hex(b"hello world");
        let c = sha256_hex(b"hello world!");
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn sha256_hex_multi_block_input() {
        // 跨越多个 512-bit 块的输入(112 字节 = 1.75 块),验证主循环
        // 跨块处理正确,只校验确定性 + 64 hex 长度。
        let input = vec![0xa5u8; 112];
        let a = sha256_hex(&input);
        let b = sha256_hex(&input);
        assert_eq!(a, b);
        assert_eq!(a.len(), 64);
    }
}
