//! 整组原生输入：惯性窗口和扫描共享同一个LCM分片重组结果。

use super::schema;
use crate::{CodecError, CodecResult};

/// 仓库LioInputFrame消息的生成指纹。
pub const FINGERPRINT: [u8; 8] = 0xc323_3f5a_145a_8a6a_u64.to_be_bytes();

/// 已经完整解码的一组输入。
#[derive(Debug)]
pub struct LioInputFrame {
    /// 完整惯性窗口，保留源顺序。
    pub imu_samples: Vec<schema::Imu>,
    /// 带处理序列的原始扫描。
    pub cloud: schema::PointCloud2,
}

/// 拼装已经分别通过既有编码器校验的消息体。
pub fn encode_body(cloud: &[u8], imu: &[Vec<u8>]) -> CodecResult<Vec<u8>> {
    if imu.len() > 4096 {
        return Err(CodecError::LcmEncode("整组惯性样本超过4096".into()));
    }
    let size = imu
        .iter()
        .try_fold(cloud.len() + 4, |total, sample| {
            total.checked_add(sample.len())
        })
        .ok_or_else(|| CodecError::LcmEncode("整组输入长度溢出".into()))?;
    if size > 8 * 1024 * 1024 {
        return Err(CodecError::LcmEncode("整组输入超过8MiB".into()));
    }
    let mut out = Vec::with_capacity(size);
    out.extend_from_slice(&(imu.len() as i32).to_be_bytes());
    for sample in imu {
        out.extend_from_slice(sample);
    }
    out.extend_from_slice(cloud);
    Ok(out)
}

/// 严格解码整组输入，不在部分惯性可用时交付点云。
pub fn decode_body(bytes: &[u8]) -> CodecResult<LioInputFrame> {
    if bytes.len() < 4 || bytes.len() > 8 * 1024 * 1024 {
        return Err(CodecError::LcmDecode("整组输入长度不合法".into()));
    }
    let count = i32::from_be_bytes(bytes[..4].try_into().unwrap());
    if !(0..=4096).contains(&count) {
        return Err(CodecError::LcmDecode("惯性窗口数量不合法".into()));
    }
    let mut rest = &bytes[4..];
    let mut imu_samples = Vec::with_capacity(count as usize);
    for _ in 0..count {
        let (sample, consumed) = schema::decode_imu_prefix(rest)?;
        imu_samples.push(sample);
        rest = &rest[consumed..];
    }
    let mut cloud_wire = schema::POINT_CLOUD2_FINGERPRINT.to_vec();
    cloud_wire.extend_from_slice(rest);
    Ok(LioInputFrame {
        imu_samples,
        cloud: schema::decode_point_cloud2(&cloud_wire)?,
    })
}
