//! 与PlanarScan.lcm及生成的C/Python绑定一致的规范二维扫描。
use crate::{CodecError, CodecResult};
/// 由lcm-gen生成的消息指纹。
pub const FINGERPRINT: [u8; 8] = 0x2983_f7eb_2e8b_8396u64.to_be_bytes();
/// 单帧允许的最大编码字节数。
pub const MAX_BYTES: usize = 1024 * 1024;
#[derive(Debug, Clone, PartialEq)]
/// 保留物理时间、来源与光束索引的二维消息。
pub struct PlanarScan {
    /// 接收拥有方绑定的运行代际。
    pub runtime_generation: i64,
    /// 逻辑传感器标识。
    pub sensor_id: String,
    /// 来源单调序号。
    pub source_sequence: i64,
    /// 扫描开始采样时间，纳秒。
    pub timestamp_ns: i64,
    /// 时间所属时钟域。
    pub clock_domain: String,
    /// 设备或主机时钟标识。
    pub clock_id: String,
    /// 原始雷达坐标系。
    pub frame_id: String,
    /// 真实外参引用。
    pub calibration_ref: String,
    /// 光束数量。
    pub beam_count: i32,
    /// 1等角，2显式角度。
    pub angular_layout: u8,
    /// 等角布局的起始角。
    pub angle_min_rad: f64,
    /// 有方向的角度步进。
    pub angle_increment_rad: f64,
    /// 最小量程，米。
    pub range_min_m: f64,
    /// 最大量程，米。
    pub range_max_m: f64,
    /// 完整采集窗口长度。
    pub scan_duration_ns: i64,
    /// 1硬件、2估算、3接收。
    pub timestamp_quality: u8,
    /// 0无、1硬件、2估算。
    pub time_offsets_quality: u8,
    /// 时间不确定度；负一表示未知。
    pub timing_uncertainty_ns: i64,
    /// 是否在上游完成运动补偿。
    pub motion_compensated: bool,
    /// 小端单精度距离数组。
    pub ranges_f32_le: Vec<u8>,
    /// 显式角度数组；等角布局为空。
    pub angles_f32_le: Vec<u8>,
    /// 0坏测量、1有效回波、2明确无返回。
    pub beam_states_u8: Vec<u8>,
    /// 可选物理强度数组。
    pub intensities_f32_le: Vec<u8>,
    /// 可选逐束纳秒偏移数组。
    pub time_offsets_ns_le: Vec<u8>,
}
fn bad(message: &str) -> CodecError {
    CodecError::LcmDecode(message.into())
}
impl PlanarScan {
    /// 校验字段尺寸、光束状态及物理时间，不修改输入。
    pub fn validate(&self) -> CodecResult<()> {
        if self.runtime_generation <= 0
            || self.source_sequence < 0
            || self.timestamp_ns < 0
            || !(1..=32768).contains(&self.beam_count)
        {
            return Err(bad("二维来源、时间或束数非法"));
        }
        if self.sensor_id.trim().is_empty()
            || self.frame_id.trim().is_empty()
            || !matches!(
                self.clock_domain.as_str(),
                "unix_ns" | "monotonic_ns" | "device_ns"
            )
            || self.clock_domain == "device_ns" && self.clock_id.is_empty()
        {
            return Err(bad("二维来源坐标或时钟域非法"));
        }
        for text in [
            &self.sensor_id,
            &self.clock_domain,
            &self.clock_id,
            &self.frame_id,
            &self.calibration_ref,
        ] {
            if text.len() > 255 || text.contains('\0') {
                return Err(bad("二维字符串超限或含零字节"));
            }
        }
        let n = self.beam_count as usize;
        if self.ranges_f32_le.len() != 4 * n
            || self.beam_states_u8.len() != n
            || !self.intensities_f32_le.is_empty() && self.intensities_f32_le.len() != 4 * n
        {
            return Err(bad("二维距离、状态或强度长度非法"));
        }
        if !self.range_min_m.is_finite()
            || !self.range_max_m.is_finite()
            || self.range_min_m < 0.0
            || self.range_max_m <= self.range_min_m
        {
            return Err(bad("二维量程非法"));
        }
        if self.angular_layout == 1 {
            if !self.angles_f32_le.is_empty()
                || !self.angle_min_rad.is_finite()
                || !self.angle_increment_rad.is_finite()
                || self.angle_increment_rad == 0.0
                || self.angle_increment_rad.abs() * (n - 1) as f64 > std::f64::consts::TAU + 1e-5
            {
                return Err(bad("等角扫描参数非法"));
            }
        } else if self.angular_layout == 2 {
            if self.angles_f32_le.len() != 4 * n
                || self.angle_min_rad != 0.0
                || self.angle_increment_rad != 0.0
            {
                return Err(bad("显式角度长度或冗余字段非法"));
            }
        } else {
            return Err(bad("未知二维角度布局"));
        }
        if !(0..=5_000_000_000).contains(&self.scan_duration_ns)
            || self
                .timestamp_ns
                .checked_add(self.scan_duration_ns)
                .is_none()
            || !(1..=3).contains(&self.timestamp_quality)
            || self.time_offsets_quality > 2
            || self.timing_uncertainty_ns < -1
        {
            return Err(bad("二维时间质量或范围非法"));
        }
        if self.time_offsets_ns_le.is_empty() {
            if self.time_offsets_quality != 0 {
                return Err(bad("缺少逐束时间载荷"));
            }
        } else if self.time_offsets_ns_le.len() != 8 * n || self.time_offsets_quality == 0 {
            return Err(bad("逐束时间长度或质量非法"));
        }
        for (state, bytes) in self
            .beam_states_u8
            .iter()
            .zip(self.ranges_f32_le.chunks_exact(4))
        {
            let range = f32::from_le_bytes(bytes.try_into().unwrap()) as f64;
            if *state > 2
                || *state == 1
                    && (!range.is_finite() || range < self.range_min_m || range > self.range_max_m)
            {
                return Err(bad("有效回波超出量程"));
            }
        }
        for values in [&self.angles_f32_le, &self.intensities_f32_le] {
            if values
                .chunks_exact(4)
                .any(|b| !f32::from_le_bytes(b.try_into().unwrap()).is_finite())
            {
                return Err(bad("角度或强度非有限"));
            }
        }
        if self
            .time_offsets_ns_le
            .chunks_exact(8)
            .any(|b| u64::from_le_bytes(b.try_into().unwrap()) > self.scan_duration_ns as u64)
        {
            return Err(bad("逐束时间超出采集窗口"));
        }
        Ok(())
    }
}
fn string(out: &mut Vec<u8>, value: &str) {
    out.extend_from_slice(&((value.len() + 1) as i32).to_be_bytes());
    out.extend_from_slice(value.as_bytes());
    out.push(0);
}
fn bytes(out: &mut Vec<u8>, value: &[u8]) {
    out.extend_from_slice(&(value.len() as i32).to_be_bytes());
    out.extend_from_slice(value);
}
/// 编码带生成指纹的完整二维消息。
pub fn encode(scan: &PlanarScan) -> CodecResult<Vec<u8>> {
    scan.validate()?;
    let mut out = Vec::new();
    out.extend_from_slice(&FINGERPRINT);
    out.extend_from_slice(&1i32.to_be_bytes());
    out.extend_from_slice(&scan.runtime_generation.to_be_bytes());
    string(&mut out, &scan.sensor_id);
    out.extend_from_slice(&scan.source_sequence.to_be_bytes());
    out.extend_from_slice(&scan.timestamp_ns.to_be_bytes());
    for value in [
        &scan.clock_domain,
        &scan.clock_id,
        &scan.frame_id,
        &scan.calibration_ref,
    ] {
        string(&mut out, value);
    }
    out.extend_from_slice(&scan.beam_count.to_be_bytes());
    out.push(scan.angular_layout);
    for value in [
        scan.angle_min_rad,
        scan.angle_increment_rad,
        scan.range_min_m,
        scan.range_max_m,
    ] {
        out.extend_from_slice(&value.to_be_bytes());
    }
    out.extend_from_slice(&scan.scan_duration_ns.to_be_bytes());
    out.push(scan.timestamp_quality);
    out.push(scan.time_offsets_quality);
    out.extend_from_slice(&scan.timing_uncertainty_ns.to_be_bytes());
    out.push(u8::from(scan.motion_compensated));
    for value in [
        &scan.ranges_f32_le,
        &scan.angles_f32_le,
        &scan.beam_states_u8,
        &scan.intensities_f32_le,
        &scan.time_offsets_ns_le,
    ] {
        bytes(&mut out, value);
    }
    if out.len() > MAX_BYTES {
        return Err(bad("二维消息超限"));
    }
    Ok(out)
}
struct Reader<'a> {
    bytes: &'a [u8],
    pos: usize,
}
impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> CodecResult<&'a [u8]> {
        let end = self
            .pos
            .checked_add(n)
            .filter(|n| *n <= self.bytes.len())
            .ok_or_else(|| bad("二维消息不完整"))?;
        let result = &self.bytes[self.pos..end];
        self.pos = end;
        Ok(result)
    }
    fn u8(&mut self) -> CodecResult<u8> {
        Ok(self.take(1)?[0])
    }
    fn i32(&mut self) -> CodecResult<i32> {
        Ok(i32::from_be_bytes(self.take(4)?.try_into().unwrap()))
    }
    fn i64(&mut self) -> CodecResult<i64> {
        Ok(i64::from_be_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn f64(&mut self) -> CodecResult<f64> {
        Ok(f64::from_be_bytes(self.take(8)?.try_into().unwrap()))
    }
    fn size(&mut self, max: usize) -> CodecResult<usize> {
        let n = self.i32()?;
        if n < 0 || n as usize > max {
            return Err(bad("二维字段长度非法"));
        }
        Ok(n as usize)
    }
    fn string(&mut self) -> CodecResult<String> {
        let n = self.size(256)?;
        let data = self.take(n)?;
        if data.last() != Some(&0) {
            return Err(bad("二维字符串缺少终止符"));
        }
        String::from_utf8(data[..n - 1].to_vec()).map_err(|_| bad("二维字符串编码非法"))
    }
    fn blob(&mut self) -> CodecResult<Vec<u8>> {
        let n = self.size(32768 * 8)?;
        Ok(self.take(n)?.to_vec())
    }
}
/// 有界解码并校验完整消息，拒绝尾随字节。
pub fn decode(input: &[u8]) -> CodecResult<PlanarScan> {
    if input.len() > MAX_BYTES {
        return Err(bad("二维消息超限"));
    }
    let mut r = Reader {
        bytes: input,
        pos: 0,
    };
    if r.take(8)? != FINGERPRINT || r.i32()? != 1 {
        return Err(bad("二维消息指纹或版本不匹配"));
    }
    let scan = PlanarScan {
        runtime_generation: r.i64()?,
        sensor_id: r.string()?,
        source_sequence: r.i64()?,
        timestamp_ns: r.i64()?,
        clock_domain: r.string()?,
        clock_id: r.string()?,
        frame_id: r.string()?,
        calibration_ref: r.string()?,
        beam_count: r.i32()?,
        angular_layout: r.u8()?,
        angle_min_rad: r.f64()?,
        angle_increment_rad: r.f64()?,
        range_min_m: r.f64()?,
        range_max_m: r.f64()?,
        scan_duration_ns: r.i64()?,
        timestamp_quality: r.u8()?,
        time_offsets_quality: r.u8()?,
        timing_uncertainty_ns: r.i64()?,
        motion_compensated: match r.u8()? {
            0 => false,
            1 => true,
            _ => return Err(bad("二维补偿标志非法")),
        },
        ranges_f32_le: r.blob()?,
        angles_f32_le: r.blob()?,
        beam_states_u8: r.blob()?,
        intensities_f32_le: r.blob()?,
        time_offsets_ns_le: r.blob()?,
    };
    if r.pos != input.len() {
        return Err(bad("二维消息含尾随字节"));
    }
    scan.validate()?;
    Ok(scan)
}
