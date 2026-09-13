//! 二维导航观测：同时保留规范图身份和真实扫描来源，撤销不是占位地图。
use super::schema::{self, OccupancyGrid};
use crate::{CodecError, CodecResult};

/// 由lcm-gen（类型生成器）生成并经真实通信核对的指纹。
pub const FINGERPRINT: [u8; 8] = 0xc0fe_7172_5d24_0712_u64.to_be_bytes();
/// 二维导航观测的完整协议类型。
pub const TYPE_NAME: &str = "nuwax_msgs.PlanarNavigationMap";
/// 与规范栅格通道分离的导航派生通道。
pub const DEFAULT_TOPIC: &str = "/nav/planar_occupancy";
/// 含内层栅格的最大消息预算。
pub const MAX_BYTES: usize = 64 * 1024 * 1024;
/// 接纳时允许的最大真实扫描年龄。
pub const MAX_SOURCE_AGE_NS: i64 = 500_000_000;

#[derive(Clone, Debug, PartialEq, Default)]
/// 配套规范图身份与新鲜扫描证据；清空显式使用撤销标志。
pub struct PlanarNavigationMap {
    /// 规范地图拥有方运行代际。
    pub runtime_generation: i64,
    /// 已提交建图会话标识。
    pub session_id: String,
    /// 建图会话代次。
    pub session_generation: i64,
    /// 已提交规范地图修订。
    pub map_revision: i64,
    /// 已提交规范地图的有效栅格摘要。
    pub map_payload_sha256: [u8; 32],
    /// 当前运行中单调递增的观测修订。
    pub observation_revision: i64,
    /// 本次真实扫描来源序号。
    pub source_sequence: i64,
    /// 本次扫描映射到Unix（纪元时间）的采样时间。
    pub source_timestamp_ns: i64,
    /// 该观测是否明确撤销导航来源。
    pub cleared: bool,
    /// 新鲜扫描与规范物理地图和禁区合成的导航栅格。
    pub grid: OccupancyGrid,
}
fn bad(text: impl Into<String>) -> CodecError {
    CodecError::LcmDecode(text.into())
}
impl PlanarNavigationMap {
    /// 校验身份、栅格结构、值域和内外时间一致性。
    pub fn validate(&self) -> CodecResult<()> {
        if self.runtime_generation <= 0
            || self.session_generation <= 0
            || self.observation_revision <= 0
            || self.session_id.is_empty()
            || self.session_id.len() > 255
            || self.session_id.contains('\0')
        {
            return Err(bad("二维导航拥有方或观测身份非法"));
        }
        if self.cleared {
            if !self.grid.data.is_empty() || self.grid.info.width != 0 || self.grid.info.height != 0
            {
                return Err(bad("二维导航撤销必须不含栅格"));
            }
            return Ok(());
        }
        let grid = &self.grid;
        if self.map_revision <= 0
            || self.map_payload_sha256 == [0; 32]
            || self.source_sequence < 0
            || self.source_timestamp_ns <= 0
            || grid.header.frame_id.is_empty()
            || grid.info.width <= 0
            || grid.info.height <= 0
            || (grid.info.width as usize).checked_mul(grid.info.height as usize)
                != Some(grid.data.len())
            || grid.data.len() > MAX_BYTES
            || !grid.info.resolution.is_finite()
            || grid.info.resolution <= 0.0
            || grid.info.origin.position.iter().any(|v| !v.is_finite())
            || grid.info.origin.orientation_xyzw != [0.0, 0.0, 0.0, 1.0]
            || grid.data.iter().any(|v| !matches!(*v, -1 | 0 | 100))
            || grid.header.stamp_sec as i64 * 1_000_000_000 + grid.header.stamp_nsec as i64
                != self.source_timestamp_ns
        {
            return Err(bad("二维导航栅格、规范图身份或真实源时间非法"));
        }
        Ok(())
    }
}
/// 编码LCM（轻量通信与编组）消息体。
pub fn encode_body(value: &PlanarNavigationMap) -> CodecResult<Vec<u8>> {
    value.validate()?;
    let mut out = Vec::new();
    out.extend(1i32.to_be_bytes());
    out.extend(value.runtime_generation.to_be_bytes());
    out.extend(((value.session_id.len() + 1) as i32).to_be_bytes());
    out.extend(value.session_id.as_bytes());
    out.push(0);
    out.extend(value.session_generation.to_be_bytes());
    out.extend(value.map_revision.to_be_bytes());
    out.extend(value.map_payload_sha256);
    for n in [
        value.observation_revision,
        value.source_sequence,
        value.source_timestamp_ns,
    ] {
        out.extend(n.to_be_bytes());
    }
    out.push(u8::from(value.cleared));
    out.extend(schema::encode_occupancy_grid_body(&value.grid)?);
    if out.len() > MAX_BYTES {
        return Err(bad("二维导航观测超限"));
    }
    Ok(out)
}
/// 有界解码消息体并完整核对内层栅格。
pub fn decode_body(input: &[u8]) -> CodecResult<PlanarNavigationMap> {
    if input.len() > MAX_BYTES {
        return Err(bad("二维导航观测超限"));
    }
    let mut reader = Reader { input, pos: 0 };
    if i32::from_be_bytes(reader.take(4)?.try_into().unwrap()) != 1 {
        return Err(bad("二维导航版本不支持"));
    }
    let runtime_generation = reader.i64()?;
    let size = i32::from_be_bytes(reader.take(4)?.try_into().unwrap());
    if !(2..=256).contains(&size) {
        return Err(bad("二维会话字符串超限"));
    }
    let raw = reader.take(size as usize)?;
    if raw.last() != Some(&0) {
        return Err(bad("二维会话字符串未终止"));
    }
    let session_id = std::str::from_utf8(&raw[..raw.len() - 1])
        .map_err(|_| bad("二维会话编码非法"))?
        .to_owned();
    let session_generation = reader.i64()?;
    let map_revision = reader.i64()?;
    let map_payload_sha256 = reader.take(32)?.try_into().unwrap();
    let observation_revision = reader.i64()?;
    let source_sequence = reader.i64()?;
    let source_timestamp_ns = reader.i64()?;
    let cleared = match reader.take(1)?[0] {
        0 => false,
        1 => true,
        _ => return Err(bad("二维撤销标志非法")),
    };
    let grid = schema::decode_occupancy_grid_body(&input[reader.pos..])?;
    let value = PlanarNavigationMap {
        runtime_generation,
        session_id,
        session_generation,
        map_revision,
        map_payload_sha256,
        observation_revision,
        source_sequence,
        source_timestamp_ns,
        cleared,
        grid,
    };
    value.validate()?;
    Ok(value)
}
struct Reader<'a> {
    input: &'a [u8],
    pos: usize,
}
impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> CodecResult<&'a [u8]> {
        let end = self
            .pos
            .checked_add(n)
            .filter(|n| *n <= self.input.len())
            .ok_or_else(|| bad("二维导航观测不完整"))?;
        let result = &self.input[self.pos..end];
        self.pos = end;
        Ok(result)
    }
    fn i64(&mut self) -> CodecResult<i64> {
        Ok(i64::from_be_bytes(self.take(8)?.try_into().unwrap()))
    }
}

/// 每个实际消费者保存自己的已接纳顺序；来源倒退不刷新任何接收期限。
#[derive(Default)]
pub struct Admission {
    latest: Option<(i64, i64, String, i64, i64)>,
}
impl Admission {
    /// 接纳严格前进的真实观测；返回是否切换来源域。
    pub fn admit(&mut self, value: &PlanarNavigationMap, now_ns: i64) -> Result<bool, String> {
        value.validate().map_err(|e| e.to_string())?;
        if !value.cleared
            && (value.source_timestamp_ns > now_ns + 50_000_000
                || now_ns.saturating_sub(value.source_timestamp_ns) > MAX_SOURCE_AGE_NS)
        {
            return Err("二维导航观测过期或位于未来".into());
        }
        let domain = (value.runtime_generation, value.session_generation);
        let changed = self
            .latest
            .as_ref()
            .is_none_or(|old| (old.0, old.1) != domain);
        if let Some(old) = &self.latest {
            if domain < (old.0, old.1)
                || (domain == (old.0, old.1)
                    && (old.2 != value.session_id
                        || value.observation_revision <= old.3
                        || !value.cleared && value.source_timestamp_ns <= old.4))
            {
                return Err("二维导航观测来源已过期或冲突".into());
            }
        }
        self.latest = Some((
            value.runtime_generation,
            value.session_generation,
            value.session_id.clone(),
            value.observation_revision,
            value.source_timestamp_ns,
        ));
        Ok(changed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn value() -> PlanarNavigationMap {
        PlanarNavigationMap {
            runtime_generation: 1,
            session_id: "map-a".into(),
            session_generation: 1,
            map_revision: 4,
            map_payload_sha256: [3; 32],
            observation_revision: 1,
            source_sequence: 0,
            source_timestamp_ns: 1_000_000_000,
            cleared: false,
            grid: OccupancyGrid {
                header: schema::Header {
                    frame_id: "map".into(),
                    stamp_sec: 1,
                    ..Default::default()
                },
                info: schema::MapMetaData {
                    width: 2,
                    height: 1,
                    resolution: 1.0,
                    origin: schema::Pose {
                        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                        ..Default::default()
                    },
                    ..Default::default()
                },
                data: vec![0, 100],
            },
        }
    }
    #[test]
    fn source_age_clear_and_generation_are_independent_from_map_revision() {
        let mut admission = Admission::default();
        let mut map = value();
        assert!(admission.admit(&map, 1_000_000_000).unwrap());
        assert!(admission.admit(&map, 1_000_000_001).is_err());
        map.observation_revision = 2;
        map.source_sequence = 1;
        map.source_timestamp_ns += 1;
        map.grid.header.stamp_nsec = 1;
        assert!(!admission.admit(&map, 1_000_000_001).unwrap());
        let cleared = PlanarNavigationMap {
            cleared: true,
            observation_revision: 3,
            grid: Default::default(),
            ..map.clone()
        };
        assert!(!admission.admit(&cleared, 1_000_000_001).unwrap());
        assert!(admission.admit(&map, 1_000_000_001).is_err());
        map.session_generation = 2;
        map.session_id = "map-b".into();
        map.observation_revision = 4;
        assert!(admission.admit(&map, 2_000_000_000).is_err());
        assert!(admission.admit(&map, 1_000_000_001).unwrap());
        assert!(admission.admit(&cleared, 1_000_000_001).is_err());
    }
    #[test]
    fn codec_rejects_truncated_nested_grid_and_conflicting_source_stamp() {
        let map = value();
        let body = encode_body(&map).unwrap();
        assert_eq!(decode_body(&body).unwrap(), map);
        assert!(decode_body(&body[..body.len() - 1]).is_err());
        let mut mismatched = map;
        mismatched.grid.header.stamp_nsec = 1;
        assert!(encode_body(&mismatched).is_err());
        assert_eq!(crate::hash::fingerprint(TYPE_NAME, 0).unwrap(), FINGERPRINT);
    }
}
