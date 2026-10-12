// 规范地图摘要沿用 SHA-256（256位安全散列）。使用已有 sha2 库，避免
// 手写实现每次复制完整输入并在位姿共享锁内执行标量压缩循环。
use sha2::{Digest, Sha256};

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

#[cfg(test)]
std::thread_local! { pub(crate) static FORBID_HASH: std::cell::Cell<bool> = const { std::cell::Cell::new(false) }; }

/// SHA-256 原始 32 字节摘要。
#[must_use]
pub fn sha256_bytes(data: &[u8]) -> [u8; 32] {
    #[cfg(test)] FORBID_HASH.with(|v| assert!(!v.get(), "legacy map hash reached assigned planar path"));
    Sha256::digest(data).into()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "同输入摘要耗时对照"]
    fn sha256_pose_latency_budget() {
        let data = vec![0x5a; 16 * 1024 * 1024];
        let started = std::time::Instant::now();
        for _ in 0..7 {
            std::hint::black_box(sha256_bytes(std::hint::black_box(&data)));
        }
        println!("seven_16mib_hash_ms={:.3}", started.elapsed().as_secs_f64() * 1000.0);
    }

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
    fn sha256_million_a_matches_standard_vector() {
        assert_eq!(sha256_hex(&vec![b'a'; 1_000_000]),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
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
