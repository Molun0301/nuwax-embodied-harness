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
    complete_geometry_source: bool,
    block_edge: i32,
    blocks: im::HashMap<[i32; 3], Arc<AHashMap<[i32; 3], SurfaceObservation>>, ahash::RandomState>,
    // 按XY维护真实占据层号，局部地形查询不扫描整图或所有竖直块。
    columns: im::HashMap<[i32; 2], Arc<Vec<i32>>, ahash::RandomState>,
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
            complete_geometry_source: false,
            block_edge,
            blocks: Default::default(),
            columns: Default::default(),
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
        // 块替换之后须由完整提交的拥有方重新确认，不把旧根的完整性带入部分更新。
        self.complete_geometry_source=false;
        let mut replacement=AHashMap::<[i32;2],Vec<i32>>::new();
        for key in values.keys() {replacement.entry([key[0],key[1]]).or_default().push(key[2]);}
        let mut affected:AHashSet<[i32;2]>=replacement.keys().copied().collect();
        if let Some(previous)=self.blocks.get(&block) {
            affected.extend(previous.keys().map(|key|[key[0],key[1]]));
        }
        for column in affected {
            let mut levels=self.columns.get(&column).map_or_else(Vec::new,|levels|levels.as_ref().clone());
            levels.retain(|z|z.div_euclid(self.block_edge)!=block[2]);
            if let Some(replacement)=replacement.remove(&column) {levels.extend(replacement);}
            levels.sort_unstable();levels.dedup();
            if levels.is_empty() {self.columns.remove(&column);}
            else if self.columns.get(&column).is_none_or(|previous|previous.as_slice()!=levels.as_slice()) {
                self.columns.insert(column,Arc::new(levels));
            }
        }
        if values.is_empty() {
            self.blocks.remove(&block);
        } else {
            self.blocks.insert(block, Arc::new(values.into_iter().collect()));
        }
        self.obstacle_cache = Default::default();
        Ok(())
    }

    /// 确认本索引覆盖同域、同精度、同次几何提交的全部已知占据。
    /// 只能在拥有方验证并装配完整源之后调用；不表示自由空间、观察新鲜度或命中质量。
    /// 后续任意成功块替换都会撤销该事实。
    pub fn mark_complete_geometry_source(&mut self) {
        self.complete_geometry_source=true;
    }

    pub(super) fn is_complete_geometry_source(&self)->bool {
        self.complete_geometry_source
    }

    /// 仅比较占据几何；新扫描时间和命中次数不使层析代价重建。
    pub fn same_geometry(&self,other:&Self)->bool {
        self.size_m.to_bits()==other.size_m.to_bits()
            && self.complete_geometry_source==other.complete_geometry_source
            && self.columns.ptr_eq(&other.columns)
    }
    pub(super) fn navigation_columns(&self)->impl Iterator<Item=([i32;2],&[i32])> {
        self.columns.iter().map(|(&key,levels)|(key,levels.as_slice()))
    }

    /// 查询实际位置的证据，不修改时间或命中次数。
    pub fn get(&self, key: [i32; 3]) -> Option<SurfaceObservation> {
        self.blocks
            .get(&key.map(|v| v.div_euclid(self.block_edge)))?
            .get(&key)
            .copied()
    }

    /// 同一原始细列的全部实测层号；稳定性由原占据证据判断继续核对。
    pub(super) fn column_levels(&self,x:i32,y:i32)->Option<&[i32]> {
        self.columns.get(&[x,y]).map(|levels|levels.as_slice())
    }

    /// 完整块存在只说明该范围已提交；具体细格是否有证据仍须查询get。
    pub(super) fn has_complete_block(&self,key:[i32;3])->bool {
        self.blocks.contains_key(&key.map(|v|v.div_euclid(self.block_edge)))
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
    fn complete_geometry_fact_is_scoped_to_the_immutable_committed_source() {
        let mut grid=SurfaceObservations::new(0.05,32).unwrap();
        assert!(!grid.is_complete_geometry_source());
        let row=SurfaceObservation{observed_ns:0,hits:0};
        grid.replace_block([0,0,0],BTreeMap::from([([0,0,0],row)])).unwrap();
        grid.mark_complete_geometry_source();let previous=grid.clone();
        grid.set_source_timestamp_ns(100);
        assert!(grid.is_complete_geometry_source(),"只更新根时间不改变占据全集");
        assert!(grid.replace_block([0,0,0],BTreeMap::from([([32,0,0],row)])).is_err());
        assert!(grid.is_complete_geometry_source(),"失败输入不能改动已提交事实");
        grid.replace_block([0,0,0],BTreeMap::new()).unwrap();
        assert!(!grid.is_complete_geometry_source(),"删除或替换后必须重新验证完整根");
        assert!(previous.is_complete_geometry_source());
        assert!(previous.get([0,0,0]).is_some());
    }

    #[test]
    fn local_columns_follow_block_replacement_and_preserve_immutable_snapshots() {
        let mut observations=SurfaceObservations::new(0.05,2).unwrap();
        let row=SurfaceObservation {observed_ns:0,hits:0};
        observations.replace_block([0,0,-1],BTreeMap::from([([0,0,-1],row)])).unwrap();
        observations.replace_block([0,0,0],BTreeMap::from([([0,0,0],row),([0,0,1],row)])).unwrap();
        observations.replace_block([0,0,1],BTreeMap::from([([0,0,2],row)])).unwrap();
        let previous=observations.clone();
        assert_eq!(observations.column_levels(0,0),Some([-1,0,1,2].as_slice()));
        observations.replace_block([0,0,0],BTreeMap::from([([0,0,1],row),([1,0,0],row)])).unwrap();
        assert_eq!(observations.column_levels(0,0),Some([-1,1,2].as_slice()));
        assert_eq!(observations.column_levels(1,0),Some([0].as_slice()));
        observations.replace_block([0,0,0],BTreeMap::new()).unwrap();
        assert_eq!(observations.column_levels(0,0),Some([-1,2].as_slice()));
        assert!(observations.column_levels(1,0).is_none());
        assert_eq!(previous.column_levels(0,0),Some([-1,0,1,2].as_slice()));
        assert!(observations.replace_block([0,0,0],BTreeMap::from([([0,0,2],row)])).is_err());
        assert_eq!(observations.column_levels(0,0),Some([-1,2].as_slice()));
    }

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
        Arc::ptr_eq(&self.data, &previous.data) && self.ground_evidence==previous.ground_evidence
    }

    /// 将同一次完整提交的观察附到已有几何上。
    pub fn with_observations(&self, observations: Option<Arc<SurfaceObservations>>) -> Self {
        // 同一不可变观察的再次挂接不是输入变化；保留已经完成的精确几何查询。
        // 新对象（即使数值相等）仍走原失效路径，不为旧观察延长有效期。
        if match (&self.observations, &observations) {
            (None, None) => true,
            (Some(previous), Some(next)) => Arc::ptr_eq(previous, next),
            _ => false,
        } {
            return self.clone();
        }
        Self {
            ground_evidence:self.ground_evidence,
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

#[cfg(test)]
mod snapshot_cache_tests {
    use super::*;

    #[test]
    fn identical_observation_owner_preserves_queries_but_replacement_retires_them() {
        let points: Vec<_> = (-10..=10).flat_map(|x|
            (-10..=10).map(move |y| (x as f32 * 0.1, y as f32 * 0.1, 0.0))).collect();
        let config = crate::mls_planner::Config { world_frame: "map".into(), voxel_size: 0.1,
            robot_height: 0.55, surface_closing_radius: 0.0, node_spacing_m: 0.3,
            wall_clearance_m: 0.0, wall_buffer_m: 0.1, wall_buffer_weight: 2.0,
            max_slope_deg: 18.0, step_threshold_m: 0.15, min_step_tread_depth_m: 0.2,
            step_penalty_weight: 1.0, goal_tolerance: 0.1, viz_publish_hz: 0.0 };
        let map = SurfaceSnapshot::from_points(&points, config, 0.05).unwrap();
        let unchanged = map.with_observations(None);
        assert!(Arc::ptr_eq(&map.body_query_cache, &unchanged.body_query_cache));
        let observations = Arc::new(SurfaceObservations::new(0.05, 32).unwrap());
        let observed = map.with_observations(Some(observations.clone()));
        assert!(!Arc::ptr_eq(&map.body_query_cache, &observed.body_query_cache));
        let repeated = observed.with_observations(Some(observations.clone()));
        assert!(Arc::ptr_eq(&observed.body_query_cache, &repeated.body_query_cache));
        let replacement = observed.with_observations(Some(Arc::new((*observations).clone())));
        assert!(!Arc::ptr_eq(&observed.body_query_cache, &replacement.body_query_cache),
            "新提交即使数据相等也不能沿用旧观察的许可缓存");
        assert!(!Arc::ptr_eq(&observed.body_query_cache, &observed.with_observations(None).body_query_cache));
    }
}
