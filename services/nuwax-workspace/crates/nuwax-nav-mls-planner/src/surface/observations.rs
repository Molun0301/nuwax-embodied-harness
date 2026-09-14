//! 真实精细观察的共享索引；时间更新不失效几何与机体计算缓存。
use super::*;
use std::collections::BTreeMap;

/// 同一个精细体素的实际扫描证据，命中数不等同概率或新鲜扫描数。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SurfaceObservation {
    /// 原始采样的Unix纳秒时间，零表示未知。
    pub observed_ns: u64,
    /// 不同采样时间的累计命中数。
    pub hits: u16,
}

/// 按固定空间块共享的精细观察，业务来源身份由外层提交票据约束。
#[derive(Clone, Debug)]
pub struct SurfaceObservations {
    pub(super) size_m: f32,
    block_edge: i32,
    blocks: im::OrdMap<[i32; 3], Arc<BTreeMap<[i32; 3], SurfaceObservation>>>,
}
impl SurfaceObservations {
    /// 建立空观察索引，空索引不会证明任何支撑已观察。
    pub fn new(size_m: f32, block_edge: i32) -> Result<Self, SurfaceError> {
        if !size_m.is_finite() || size_m <= 0.0 || !(1..=128).contains(&block_edge) {
            return Err(SurfaceError::InvalidConfig("精细观察网格参数无效".into()));
        }
        Ok(Self {
            size_m,
            block_edge,
            blocks: Default::default(),
        })
    }

    /// 原构建拥有方替换一个已验证的完整块；空块撤销旧证据。
    pub fn replace_block(
        &mut self,
        block: [i32; 3],
        values: BTreeMap<[i32; 3], SurfaceObservation>,
    ) -> Result<(), SurfaceError> {
        if values.len() > (self.block_edge as usize).pow(3)
            || values
                .keys()
                .any(|key| key.map(|v| v.div_euclid(self.block_edge)) != block)
        {
            return Err(SurfaceError::InvalidConfig("观察块坐标或容量无效".into()));
        }
        if values.is_empty() {
            self.blocks.remove(&block);
        } else {
            self.blocks.insert(block, Arc::new(values));
        }
        Ok(())
    }

    /// 查询实际位置的证据，不修改时间或命中次数。
    pub fn get(&self, key: [i32; 3]) -> Option<SurfaceObservation> {
        self.blocks
            .get(&key.map(|v| v.div_euclid(self.block_edge)))?
            .get(&key)
            .copied()
    }
}
impl SurfaceSnapshot {
    /// 几何相同即可继续复用静态路线；观察层由局部执行独立检查。
    pub fn same_geometry(&self, previous: &Self) -> bool {
        Arc::ptr_eq(&self.data, &previous.data)
    }

    /// 将同一次完整提交的观察附到已有几何上。
    pub fn with_observations(&self, observations: Option<Arc<SurfaceObservations>>) -> Self {
        Self {
            data: Arc::clone(&self.data),
            observations,
        }
    }

    /// 已提交的真实观察；没有精细证据时为未知。
    pub fn observations(&self) -> Option<&Arc<SurfaceObservations>> {
        self.observations.as_ref()
    }
}
