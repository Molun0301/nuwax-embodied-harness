//! 地图块分段的可协商无损压缩；不改变不可变对象、分段预算或校验身份。

/// 仅在压缩确实减小时使用；输出仍受原始256KiB分段预算限制。
pub fn compress(bytes: &[u8]) -> Option<Vec<u8>> {
    if bytes.is_empty() || bytes.len() > nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES {
        return None;
    }
    let encoded = lz4_flex::block::compress(bytes);
    (encoded.len() < bytes.len()).then_some(encoded)
}

/// 解压前固定原分段大小，禁止压缩内容声明更大输出并导致无界分配。
pub fn decompress(bytes: &[u8], expected: usize) -> Result<Vec<u8>, String> {
    if expected == 0 || expected > nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES
        || bytes.is_empty() || bytes.len() >= expected {
        return Err("精细块压缩段长度无效".into());
    }
    let mut decoded = vec![0; expected];
    let written = lz4_flex::block::decompress_into(bytes, &mut decoded)
        .map_err(|_| "精细块压缩段损坏".to_string())?;
    if written != expected { return Err("精细块解压长度不匹配".into()); }
    Ok(decoded)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lossless_parts_preserve_all_bytes_and_reject_wrong_extent() {
        let original: Vec<_> = (0..nuwax_map_chunks::MAX_BLOCK_TRANSFER_PART_BYTES)
            .map(|i| (i % 251) as u8).collect();
        let encoded = compress(&original).unwrap();
        assert_eq!(decompress(&encoded, original.len()).unwrap(), original);
        assert!(decompress(&encoded, original.len() - 1).is_err());
        assert!(decompress(&encoded, original.len() + 1).is_err());
        assert!(decompress(&[255], original.len()).is_err());
        assert!(decompress(&encoded, 0).is_err());
        assert!(compress(&[]).is_none());
    }
}
