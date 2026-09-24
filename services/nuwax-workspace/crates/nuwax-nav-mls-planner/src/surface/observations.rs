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
    source_timestamp_ns: u64,
    block_edge: i32,
    blocks: im::HashMap<[i32; 3], Arc<AHashMap<[i32; 3], SurfaceObservation>>, ahash::RandomState>,
    obstacle_cache: Arc<std::sync::Mutex<AHashMap<[i32; 3], bool>>>,
}
impl SurfaceObservations {
    /// 建立空观察索引，空索引不会证明任何支撑已观察。
    pub fn new(size_m: f32, block_edge: i32) -> Result<Self, SurfaceError> {
        if !size_m.is_finite() || size_m <= 0.0 || !(1..=128).contains(&block_edge) {
            return Err(SurfaceError::InvalidConfig("精细观察网格参数无效".into()));
        }
        Ok(Self {
            size_m,
            source_timestamp_ns: 0,
            block_edge,
            blocks: Default::default(),
            obstacle_cache: Default::default(),
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
            self.blocks.insert(block, Arc::new(values.into_iter().collect()));
        }
        self.obstacle_cache = Default::default();
        Ok(())
    }

    /// 查询实际位置的证据，不修改时间或命中次数。
    pub fn get(&self, key: [i32; 3]) -> Option<SurfaceObservation> {
        self.blocks
            .get(&key.map(|v| v.div_euclid(self.block_edge)))?
            .get(&key)
            .copied()
    }

    /// 当前完整提交的源采样；零表示旧输入没有可用的帧身份。
    pub fn set_source_timestamp_ns(&mut self, timestamp_ns: u64) {
        if self.source_timestamp_ns != timestamp_ns {
            self.obstacle_cache = Default::default();
        }
        self.source_timestamp_ns = timestamp_ns;
    }

    /// 本帧障碍立即保留；历史碰撞须有重复命中或连片空间证据。
    /// 只影响导航派生判断，不删除原始地图或把未知证据当作自由。
    pub(super) fn supports_obstacle(&self, key: [i32; 3]) -> bool {
        if let Some(value) = self.obstacle_cache.lock().unwrap_or_else(|e| e.into_inner()).get(&key).copied() {
            return value;
        }
        let supported = self.check_obstacle_support(key);
        let mut cache = self.obstacle_cache.lock().unwrap_or_else(|e| e.into_inner());
        if cache.len() >= 8192 {cache.clear();}
        cache.insert(key,supported);
        supported
    }

    fn check_obstacle_support(&self, key: [i32; 3]) -> bool {
        let Some(value) = self.get(key) else { return false; };
        if self.source_timestamp_ns == 0 || value.observed_ns == 0 || value.hits == 0
            || value.observed_ns >= self.source_timestamp_ns || value.hits >= 3 {
            return true;
        }
        let mut neighbours = 0;
        for x in -1..=1 { for y in -1..=1 { for z in -1..=1 {
            if x == 0 && y == 0 && z == 0 { continue; }
            let (Some(x),Some(y),Some(z)) = (key[0].checked_add(x),key[1].checked_add(y),key[2].checked_add(z)) else {continue;};
            if let Some(other) = self.get([x,y,z]) {
                if other.hits >= 3 || other.observed_ns == 0 { return true; }
                neighbours += 1;
                if neighbours >= 2 { return true; }
            }
        }}}
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn isolated_history_does_not_replace_current_or_confirmed_obstacles() {
        let mut grid = SurfaceObservations::new(0.05, 32).unwrap();
        let values = [
            ([1,1,1], SurfaceObservation { observed_ns: 100, hits: 1 }),
            ([5,5,5], SurfaceObservation { observed_ns: 1, hits: 3 }),
            ([10,10,10], SurfaceObservation { observed_ns: 1, hits: 1 }),
            ([15,15,15], SurfaceObservation { observed_ns: 1, hits: 1 }),
            ([16,15,15], SurfaceObservation { observed_ns: 1, hits: 1 }),
            ([15,16,15], SurfaceObservation { observed_ns: 1, hits: 1 }),
            ([20,20,20], SurfaceObservation { observed_ns: 0, hits: 0 }),
        ].into_iter().collect();
        grid.replace_block([0,0,0], values).unwrap();
        assert!(grid.supports_obstacle([10,10,10]), "旧格式没有帧证据时保留占据");
        grid.set_source_timestamp_ns(100);
        for key in [[1,1,1],[5,5,5],[15,15,15],[20,20,20]] {
            assert!(grid.supports_obstacle(key), "本帧、稳定及连片障碍不能被过滤");
        }
        assert!(!grid.supports_obstacle([10,10,10]), "历史孤立单次回波不作实体墙");
        assert!(grid.get([10,10,10]).is_some(), "原始观察保留");
        let previous = grid.clone();
        grid.set_source_timestamp_ns(101);
        assert!(!grid.supports_obstacle([1,1,1]), "新采样不能沿用旧帧的即时命中许可");
        assert!(previous.supports_obstacle([1,1,1]), "旧不可变观察仍保留自己的缓存");
        grid.replace_block([0,0,0],BTreeMap::new()).unwrap();
        assert!(!grid.supports_obstacle([5,5,5]));
        assert!(previous.supports_obstacle([5,5,5]));
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
            current_body_support: self.current_body_support.clone(),
            body_query_cache: Default::default(),
        }
    }

    /// 已提交的真实观察；没有精细证据时为未知。
    pub fn observations(&self) -> Option<&Arc<SurfaceObservations>> {
        self.observations.as_ref()
    }
}
