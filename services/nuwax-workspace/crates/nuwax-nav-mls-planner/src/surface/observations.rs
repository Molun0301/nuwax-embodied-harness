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

/// A measured layer with exact source members and precomputed bounded statistics.
/// All coordinates and normals are in the snapshot map frame.
#[derive(Clone, Debug)]
pub struct ObservedGroundLayer {
    /// Stable only inside the immutable source: global anchor voxel.
    pub anchor: [i32; 3],
    /// Exact inlier occupied levels at the anchor's fine XY column.
    pub members: Vec<i32>,
    /// Observed bounded-return statistics, not a calibrated confidence estimate.
    pub statistics: nuwax_map_chunks::surface_statistics::ObservationStatistics,
    /// Unit upward map-frame normal.
    pub normal: [f64; 3],
    /// Producer-associated local plane origin, distinct from the raw sample mean.
    pub plane_origin: [f64; 3],
}
impl ObservedGroundLayer {
    /// Query the associated plane without rescanning or refitting its returns.
    pub fn height_at(&self, xy: [f64; 2]) -> Option<f64> {
        let mean=self.plane_origin;
        let z=mean[2]-(self.normal[0]*(xy[0]-mean[0])+self.normal[1]*(xy[1]-mean[1]))/self.normal[2];
        z.is_finite().then_some(z)
    }
    /// The same GroundLayer geometry used by global and body consumers.
    pub fn ground_at(&self, xy:[f64;2], quantum:f64)->Option<GroundLayer> {
        let first=*self.members.first()?;let last=*self.members.last()?;
        let (minimum,maximum)=self.statistics.observed_extent();
        Some(GroundLayer {
            occupied_interval:[f64::from(first)*quantum,(f64::from(last)+1.0)*quantum],
            contact_interval:[minimum[2],maximum[2]],
            representative_z:self.height_at(xy)?,
            evidence:GroundEvidence::MeasuredSurface,
            approximation:GroundApproximation::ObservedLayer,
        })
    }
    // Every stored derived field is retained bit-for-bit. Original sample
    // identities and voxel evidence remain in the newly admitted raw source.
    fn query_eq(&self,other:&Self)->bool {
        let left=self.statistics.moments();let right=other.statistics.moments();
        let (left_min,left_max)=self.statistics.observed_extent();
        let (right_min,right_max)=other.statistics.observed_extent();
        self.anchor==other.anchor && self.members==other.members
            && self.normal.map(f64::to_bits)==other.normal.map(f64::to_bits)
            && self.plane_origin.map(f64::to_bits)==other.plane_origin.map(f64::to_bits)
            && left.count()==right.count()
            && left.mean().map(f64::to_bits)==right.mean().map(f64::to_bits)
            && left.m2().map(f64::to_bits)==right.m2().map(f64::to_bits)
            && left_min.map(f64::to_bits)==right_min.map(f64::to_bits)
            && left_max.map(f64::to_bits)==right_max.map(f64::to_bits)
    }
    fn geometry_eq(&self,other:&Self)->bool {
        self.anchor==other.anchor && self.members==other.members
            && self.normal==other.normal && self.plane_origin==other.plane_origin
    }
}

/// 按固定空间块共享的精细观察，业务来源身份由外层提交票据约束。
#[derive(Clone, Debug)]
pub struct SurfaceObservations {
    pub(super) size_m: f32,
    #[cfg(test)]
    source_timestamp_ns: u64,
    complete_geometry_source: bool,
    block_edge: i32,
    blocks: im::HashMap<[i32; 3], Arc<AHashMap<[i32; 3], SurfaceObservation>>, ahash::RandomState>,
    // 按XY维护真实占据层号，局部地形查询不扫描整图或所有竖直块。
    columns: im::HashMap<[i32; 2], Arc<Vec<i32>>, ahash::RandomState>,
    #[cfg(test)]
    obstacle_cache: Arc<std::sync::Mutex<AHashMap<[i32; 3], bool>>>,
    observed_layers: im::HashMap<[i32;3],Arc<BTreeMap<u16,ObservedGroundLayer>>,ahash::RandomState>,
    native_ground: super::native_ground::NativeGroundIndex,
    // Identity only changes when a measured plane/member geometry changes.
    layer_geometry: Arc<()>,
    // 纯占据几何的固定参考场；统计刷新与私有查询视图共享，真实成员变化才撤销。
    pub(super) legacy_reference_cache: Arc<std::sync::Mutex<super::body::reference_field::LegacyReferenceCache>>,
}
impl SurfaceObservations {
    pub(super) fn fork_for_planning(&self) -> Self {
        Self {
            size_m: self.size_m,
            #[cfg(test)]
            source_timestamp_ns: self.source_timestamp_ns,
            complete_geometry_source: self.complete_geometry_source,
            block_edge: self.block_edge,
            blocks: self.blocks.clone(),
            columns: self.columns.clone(),
            observed_layers: self.observed_layers.clone(),
            native_ground: self.native_ground.clone(),
            layer_geometry: self.layer_geometry.clone(),
            legacy_reference_cache: self.legacy_reference_cache.clone(),
            #[cfg(test)]
            obstacle_cache: Default::default(),
        }
    }

    /// 建立空观察索引，空索引不会证明任何支撑已观察。
    pub fn new(size_m: f32, block_edge: i32) -> Result<Self, SurfaceError> {
        if !size_m.is_finite() || size_m <= 0.0 || !(1..=128).contains(&block_edge) {
            return Err(SurfaceError::InvalidConfig("精细观察网格参数无效".into()));
        }
        Ok(Self {
            size_m,
            #[cfg(test)]
            source_timestamp_ns: 0,
            complete_geometry_source: false,
            block_edge,
            blocks: Default::default(),
            columns: Default::default(),
            observed_layers: Default::default(),
            native_ground: Default::default(),
            layer_geometry: Arc::new(()),
            legacy_reference_cache: Default::default(),
            #[cfg(test)]
            obstacle_cache: Default::default(),
        })
    }

    /// 原构建拥有方替换一个已验证的完整块；空块撤销旧证据。
    pub fn replace_block(
        &mut self,
        block: [i32; 3],
        values: BTreeMap<[i32; 3], SurfaceObservation>,
    ) -> Result<(), SurfaceError> {
        self.replace_block_with_geometry_changes(block, values).map(|_| ())
    }

    /// Replace a complete block and return only XY columns whose final occupied
    /// levels changed, including removals. Evidence-only updates return empty.
    pub fn replace_block_with_geometry_changes(
        &mut self,
        block: [i32; 3],
        values: BTreeMap<[i32; 3], SurfaceObservation>,
    ) -> Result<Vec<[i32; 2]>, SurfaceError> {
        let mut changed = Vec::new();
        if values.len() > (self.block_edge as usize).pow(3) {
            return Err(SurfaceError::InvalidConfig("观察块坐标或容量无效".into()));
        }
        let previous = self.blocks.get(&block);
        let mut same_membership = previous.map_or(values.is_empty(), |previous| previous.len() == values.len());
        // Check exact membership during the mandatory coordinate-validation walk.
        // Stop probing after the first mismatch; no content/identity digest is needed.
        for key in values.keys() {
            if key.map(|v| v.div_euclid(self.block_edge)) != block {
                return Err(SurfaceError::InvalidConfig("观察块坐标或容量无效".into()));
            }
            same_membership = same_membership && previous.is_some_and(|previous| previous.contains_key(key));
        }
        // 块替换之后须由完整提交的拥有方重新确认，不把旧根的完整性带入部分更新。
        self.complete_geometry_source=false;
        // Evidence-only refreshes leave occupied cells unchanged, so avoid allocating
        // per-column vectors or cloning, sorting and deduplicating column levels.
        if !same_membership {
            self.legacy_reference_cache = Default::default();
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
                let differs = self.columns.get(&column)
                    .map_or(!levels.is_empty(), |previous| previous.as_slice() != levels.as_slice());
                if differs {
                    changed.push(column);
                    if levels.is_empty() { self.columns.remove(&column); }
                    else { self.columns.insert(column, Arc::new(levels)); }
                }
            }
        }
        if values.is_empty() {
            self.blocks.remove(&block);
        } else {
            self.blocks.insert(block, Arc::new(values.into_iter().collect()));
        }
        // Removing source members invalidates their layer immediately, even if
        // this caller is a legacy block-only producer. Retained snapshots keep it.
        if let Some(old)=self.observed_layers.get(&block).filter(|old|!same_membership
            && old.values().any(|layer|layer.members.iter().any(|&z|
                self.get([layer.anchor[0],layer.anchor[1],z]).is_none()))) {
            let retained:BTreeMap<_,_>=old.iter().filter_map(|(&anchor,layer)| {
                if layer.members.iter().all(|&z|self.get([layer.anchor[0],layer.anchor[1],z]).is_some()) {
                    Some((anchor,layer.clone()))
                } else {changed.push([layer.anchor[0],layer.anchor[1]]);None}
            }).collect();
            if retained.len()!=old.len() {
                if retained.is_empty() {self.observed_layers.remove(&block);}
                else {self.observed_layers.insert(block,Arc::new(retained));}
                self.layer_geometry=Arc::new(());
            }
        }
        changed.sort_unstable();changed.dedup();
        #[cfg(test)] {self.obstacle_cache = Default::default();}
        Ok(changed)
    }

    /// Replace measured layer records after replacing the corresponding occupancy
    /// block. Empty records explicitly restore legacy compatibility for that block.
    /// Returns fine XY columns affected by plane/member changes, including deletion.
    pub fn replace_ground_layers(&mut self,block:[i32;3],layers:Arc<nuwax_map_chunks::LayerObservations>)
        ->Result<Vec<[i32;2]>,SurfaceError> {
        let invalid=||SurfaceError::InvalidConfig("invalid measured ground layer block".into());
        if !layers.is_empty() && (self.block_edge!=nuwax_map_chunks::BLOCK_SIDE
            || layers.len()>nuwax_map_chunks::MAX_LAYERS_PER_BLOCK) {return Err(invalid());}
        let mut converted=BTreeMap::new();let mut claimed=AHashSet::new();
        for (&anchor,layer) in layers.iter() {
            if layer.members.is_empty() || layer.members[0]!=anchor
                || layer.members.len()>nuwax_map_chunks::MAX_LAYER_MEMBERS
                || layer.samples.is_empty() || layer.samples.len()>nuwax_map_chunks::MAX_LAYER_SAMPLES
                || layer.members.windows(2).any(|w|w[0]>=w[1]) {return Err(invalid());}
            let key=nuwax_map_chunks::global_key(block,anchor).map_err(|_|invalid())?;
            let mut members=Vec::with_capacity(layer.members.len());
            for &local in &layer.members {
                let member=nuwax_map_chunks::global_key(block,local).map_err(|_|invalid())?;
                if member[..2]!=key[..2] || self.get(member).is_none() || !claimed.insert(local) {return Err(invalid());}
                members.push(member[2]);
            }
            // Layer classification, identities and plane validity belong to the
            // admitted source/codec. Here only cross-block membership is checked.
            let statistics=layer.statistics().map_err(|_|invalid())?;
            converted.insert(anchor,ObservedGroundLayer{anchor:key,members,statistics,normal:layer.normal,plane_origin:layer.plane_origin});
        }
        let old=self.observed_layers.get(&block);
        let mut same_query=old.map_or(converted.is_empty(),|old|old.len()==converted.len());
        let mut changed=AHashSet::new();
        if let Some(old)=old {for (&anchor,layer) in old.iter() {
            if converted.get(&anchor).is_none_or(|new|!layer.geometry_eq(new)) {
                changed.insert([layer.anchor[0],layer.anchor[1]]);
            }
        }}
        for (&anchor,layer) in &converted {
            let previous=old.and_then(|old|old.get(&anchor));
            same_query=same_query && previous.is_some_and(|old|old.query_eq(layer));
            if previous.is_none_or(|old|!old.geometry_eq(layer)) {
                changed.insert([layer.anchor[0],layer.anchor[1]]);
            }
        }
        // Preserve the existing query payload only after full conversion and
        // validation, using the mandatory changed-block walk above.
        if !same_query {
            if converted.is_empty() {self.observed_layers.remove(&block);}
            else {self.observed_layers.insert(block,Arc::new(converted));}
        }
        if !changed.is_empty() {self.layer_geometry=Arc::new(());self.complete_geometry_source=false;}
        let mut changed:Vec<_>=changed.into_iter().collect();changed.sort_unstable();Ok(changed)
    }

    /// Measured layers at exactly one fine XY; uses the existing occupied-column
    /// index and block-local anchors, never scans the whole map or refits samples.
    pub fn ground_layers(&self,column:[i32;2])->impl Iterator<Item=&ObservedGroundLayer> {
        self.columns.get(&column).filter(|_|!self.observed_layers.is_empty())
            .into_iter().flat_map(|levels|levels.iter()).filter_map(move|&z| {
            let key=[column[0],column[1],z];
            self.observed_layers.get(&nuwax_map_chunks::block_key(key))?
                .get(&nuwax_map_chunks::local_key(key))
        })
    }

    /// All source representations use this borrowing query. Legacy plane
    /// compatibility remains available through ground_layers for old fixtures.
    pub fn ground_records(&self,column:[i32;2])->impl Iterator<Item=SourceGroundRecord<'_>> {
        self.ground_layers(column).map(SourceGroundRecord::LegacyPlane)
            .chain(self.native_ground.records(column))
    }

    pub fn ground_record(&self,key:GroundKey)->Option<SourceGroundRecord<'_>> {
        let anchor=key.anchor();
        self.ground_records([anchor[0],anchor[1]]).find(|record|record.key()==key)
    }

    /// Install lightweight native projections after all occupancy and record
    /// fragments for this block-XY have passed the complete-root admission.
    pub fn replace_heightfield_columns(&mut self,block_xy:[i32;2],
        blocks:&[(nuwax_map_chunks::Key,nuwax_map_chunks::GroundRecords)]) -> Vec<[i32;2]> {
        let changed=self.native_ground.replace_block_xy(block_xy,blocks);
        if !changed.is_empty() {self.layer_geometry=Arc::new(());self.complete_geometry_source=false;}
        changed
    }

    pub(super) fn ground_member(&self,key:[i32;3])->bool {
        self.ground_records([key[0],key[1]]).any(|record|record.contains_member(key))
    }

    /// 确认本索引覆盖同域、同精度、同次几何提交的全部已知占据。
    /// 只能在拥有方验证并装配完整源之后调用；不表示自由空间、观察新鲜度或命中质量。
    /// 后续任意成功块替换都会撤销该事实。
    pub fn mark_complete_geometry_source(&mut self) {
        self.complete_geometry_source=true;
    }

    /// Whether the admitted owner assembled every column from one complete source.
    pub fn is_complete_geometry_source(&self)->bool {
        self.complete_geometry_source
    }

    /// 完整源是否包含独立于占据带顶的实测地面层；仅检查索引状态。
    pub(super) fn has_measured_ground_layers(&self)->bool {
        self.complete_geometry_source && (!self.observed_layers.is_empty() || !self.native_ground.is_empty())
    }

    /// 仅比较占据几何；新扫描时间和命中次数不使层析代价重建。
    pub fn same_geometry(&self,other:&Self)->bool {
        self.size_m.to_bits()==other.size_m.to_bits()
            && self.complete_geometry_source==other.complete_geometry_source
            && self.columns.ptr_eq(&other.columns)
            && Arc::ptr_eq(&self.layer_geometry,&other.layer_geometry)
    }
    // Sufficient equality for Explore's derived queries, not raw evidence.
    // Exact columns and block_edge also preserve block presence for partial
    // sources: replace_block never retains an empty raw block.
    pub(super) fn same_planning_query_view(&self,other:&Self)->bool {
        self.size_m.to_bits()==other.size_m.to_bits()
            && self.block_edge==other.block_edge
            && self.complete_geometry_source==other.complete_geometry_source
            && self.columns.ptr_eq(&other.columns)
            && self.observed_layers.ptr_eq(&other.observed_layers)
            && Arc::ptr_eq(&self.layer_geometry,&other.layer_geometry)
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
    /// Borrow exact occupied levels from one indexed column, without a map scan.
    pub fn column_levels(&self,x:i32,y:i32)->Option<&[i32]> {
        self.columns.get(&[x,y]).map(|levels|levels.as_slice())
    }

    /// 完整块存在只说明该范围已提交；具体细格是否有证据仍须查询get。
    pub(super) fn has_complete_block(&self,key:[i32;3])->bool {
        self.blocks.contains_key(&key.map(|v|v.div_euclid(self.block_edge)))
    }

    /// 当前完整提交的源采样；零表示旧输入没有可用的帧身份。
    #[cfg(test)]
    pub fn set_source_timestamp_ns(&mut self, timestamp_ns: u64) {
        if self.source_timestamp_ns != timestamp_ns {
            #[cfg(test)] {self.obstacle_cache = Default::default();}
        }
        self.source_timestamp_ns = timestamp_ns;
    }

    /// 本帧障碍立即保留；历史碰撞须有重复命中或连片空间证据。
    /// 只影响导航派生判断，不删除原始地图或把未知证据当作自由。
    #[cfg(test)]
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

    #[cfg(test)]
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

// One borrowed block per lookup scope. Exact evidence stays owned by this immutable
// observation root; the entry cannot be reused with a different owner and stores no permission.
#[cfg(test)]
pub(super) struct ObservationBlockLookup<'a> {
    owner:&'a SurfaceObservations,
    last:Option<([i32;3],Option<&'a AHashMap<[i32;3],SurfaceObservation>>)>,
    #[cfg(any(test,feature="body-query-profile"))]
    roots:u64,
}
#[cfg(test)]
impl<'a> ObservationBlockLookup<'a> {
    pub(super) fn new(owner:&'a SurfaceObservations)->Self{Self{owner,last:None,#[cfg(any(test,feature="body-query-profile"))] roots:0}}
    pub(super) fn get(&mut self,key:[i32;3])->Option<SurfaceObservation> {
        let block=key.map(|v|v.div_euclid(self.owner.block_edge));
        if self.last.as_ref().is_none_or(|(old,_)|*old!=block) {
            #[cfg(any(test,feature="body-query-profile"))] {self.roots+=1;}
            self.last=Some((block,self.owner.blocks.get(&block).map(|values|values.as_ref())));
        }
        self.last.as_ref().and_then(|(_,values)|*values).and_then(|values|values.get(&key)).copied()
    }
}


#[cfg(all(test,feature="body-query-profile"))]
impl Drop for ObservationBlockLookup<'_> {
    fn drop(&mut self) {
        crate::body_query_profile::add(crate::body_query_profile::Counter::ObservationBlockRootResolutions,self.roots);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn borrowed_block_probe_matches_owner_updates_and_missing_keys() {
        let mut original=SurfaceObservations::new(0.01,32).unwrap();
        for key in [[-33,-1,-1],[-32,0,0],[-1,31,31],[0,0,0],[0,0,1],[32,32,32]] {
            let block=key.map(|v:i32|v.div_euclid(32));
            let mut values=original.blocks.get(&block).map_or_else(BTreeMap::new,|old|old.iter().map(|(&k,&v)|(k,v)).collect());
            values.insert(key,SurfaceObservation{observed_ns:(key[2]+40)as u64,hits:2});original.replace_block(block,values).unwrap();
        }
        let retained=original.clone();
        let mut metadata_only=retained.clone();
        let refreshed:BTreeMap<_,_>=retained.blocks.get(&[0,0,0]).unwrap().iter().map(|(&key,&evidence)|
            (key,SurfaceObservation{observed_ns:evidence.observed_ns+1000,hits:evidence.hits+1})).collect();
        metadata_only.replace_block([0,0,0],refreshed).unwrap();
        original.replace_block([0,0,0],BTreeMap::from([([0,0,0],SurfaceObservation{observed_ns:999,hits:7})])).unwrap();
        original.replace_block([-2,-1,-1],BTreeMap::new()).unwrap();
        for owner in [&retained,&metadata_only,&original] {
            let mut probe=ObservationBlockLookup::new(owner);
            for _ in 0..3 {for key in [[-33,-1,-1],[-32,0,0],[-1,31,31],[0,0,0],[0,0,1],[0,0,2],[32,32,32],[64,0,0],[64,1,0],[0,0,0]] {
                assert_eq!(probe.get(key),owner.get(key));
            }}
        }
        assert_ne!(retained.get([0,0,0]),metadata_only.get([0,0,0]));
        assert_eq!(retained.blocks.get(&[0,0,0]).unwrap().len(),metadata_only.blocks.get(&[0,0,0]).unwrap().len());
        assert_ne!(retained.get([0,0,0]),original.get([0,0,0]));
        assert!(retained.get([-33,-1,-1]).is_some());assert!(original.get([-33,-1,-1]).is_none());
    }

    #[test]
    fn borrowed_block_probe_uses_all_axes_and_non_power_of_two_edges() {
        for edge in [1,3,31,32,128] {
            let mut owner=SurfaceObservations::new(0.01,edge).unwrap();
            let mut groups=BTreeMap::<_,BTreeMap<_,_>>::new();
            let keys:Vec<[i32;3]>=[i32::MIN,-129,-33,-1,0,1,31,32,128,i32::MAX].into_iter().flat_map(|x|
                [[x,0,0],[0,x,0],[0,0,x]]).collect();
            for &key in &keys {groups.entry(key.map(|v|v.div_euclid(edge))).or_default().insert(key,
                SurfaceObservation{observed_ns:123,hits:2});}
            for (block,values) in groups {owner.replace_block(block,values).unwrap();}
            let mut probe=ObservationBlockLookup::new(&owner);
            for key in keys.into_iter().chain([[7,7,7],[-7,-7,-7],[0,0,0]]) {
                assert_eq!(probe.get(key),owner.get(key),"edge={edge} key={key:?}");
            }
        }
    }

    #[test]
    #[ignore="static lookup/set-construction attribution; direct get remains available"]
    fn benchmark_observation_lookup_and_ordered_set_construction() {
        use std::{hint::black_box,time::Instant};
        let mut groups=BTreeMap::<_,BTreeMap<_,_>>::new();
        for x in 0_i32..600 {for y in 0_i32..300 {let key=[x,y,9];groups.entry(key.map(|v:i32|v.div_euclid(32))).or_default()
            .insert(key,SurfaceObservation{observed_ns:10_000_000_000,hits:2});}}
        let mut owner=SurfaceObservations::new(0.01,32).unwrap();for (block,values) in groups{owner.replace_block(block,values).unwrap();}
        let rectangle:Vec<_>=(50..110).flat_map(|x|(100..132).map(move|y|[x,y,9])).collect();
        let absent_keys:Vec<_>=rectangle.iter().map(|k|[k[0],k[1],10]).collect();
        let absent_blocks:Vec<_>=rectangle.iter().map(|k|[k[0],k[1],41]).collect();
        let alternating:Vec<_>=(0..rectangle.len()).map(|i|if i%2==0{[50,100,9]}else{[550,250,9]}).collect();
        for (label,keys) in [("ordered_rectangle",&rectangle),("missing_keys_present_blocks",&absent_keys),
            ("missing_blocks",&absent_blocks),("alternating_far_blocks",&alternating)] {
            let expected:Vec<_>=keys.iter().map(|&k|owner.get(k)).collect();
            let mut probe=ObservationBlockLookup::new(&owner);let actual:Vec<_>=keys.iter().map(|&k|probe.get(k)).collect();
            assert_eq!(actual,expected);let roots=probe.roots;
            let mut direct_ms=Vec::new();let mut borrowed_ms=Vec::new();
            for round in 0..6 {for borrowed in if round%2==0{[false,true]}else{[true,false]} {
                let started=Instant::now();
                for _ in 0..100 {
                    let mut probe=ObservationBlockLookup::new(black_box(&owner));
                    for (key,expected) in keys.iter().zip(&expected) {
                        let actual=if borrowed{probe.get(black_box(*key))}else{black_box(&owner).get(black_box(*key))};
                        assert_eq!(black_box(actual),*expected);
                    }
                }
                let ms=started.elapsed().as_secs_f64()*1e3;if borrowed{borrowed_ms.push(ms)}else{direct_ms.push(ms)}
            }}
            eprintln!("OBSERVATION_LOOKUP label={label} cells={} owner_cells=180000 owner_blocks={} preflight_root_resolutions_per_sequence={roots} loops=100 timed_counter_retention_unqualified=true captures_immutable_owner=true includes=value_equality warmed=true direct_ms={direct_ms:?} borrowed_ms={borrowed_ms:?}",keys.len(),owner.blocks.len());
            let preborrowed:Vec<_>=keys.iter().map(|key|owner.blocks.get(&key.map(|v|v.div_euclid(owner.block_edge)))).collect();
            let expected_presence:Vec<_>=preborrowed.iter().map(|v|v.is_some()).collect();
            let mut outer_ms=Vec::new();let mut inner_ms=Vec::new();
            for round in 0..6 {for outer in if round%2==0{[false,true]}else{[true,false]} {
                let started=Instant::now();
                for _ in 0..100 {for (i,key) in keys.iter().enumerate() {
                    if outer {
                        let block=black_box(*key).map(|v|v.div_euclid(owner.block_edge));
                        assert_eq!(black_box(owner.blocks.get(&block)).is_some(),expected_presence[i]);
                    }else {
                        let actual=black_box(preborrowed[i]).and_then(|block|block.get(black_box(key))).copied();
                        assert_eq!(black_box(actual),expected[i]);
                    }
                }}
                let ms=started.elapsed().as_secs_f64()*1e3;if outer{outer_ms.push(ms)}else{inner_ms.push(ms)}
            }}
            eprintln!("OBSERVATION_LOOKUP_COMPONENT label={label} loops=100 outer_includes=block_key_arithmetic_and_presence_equality inner_preborrows_blocks=true absent_block_inner_is_option_branch_only=true excludes=preborrow_setup components_are_not_additive=true outer_ms={outer_ms:?} inner_ms={inner_ms:?}");
        }
        for (label,keys) in [("rectangle",rectangle),("rounded_ordered",(0..100).flat_map(|x|(0..100).map(move|y|[x,y,9])).filter(|k|{let x=k[0]-50;let y=k[1]-50;x*x+y*y<=2500}).collect())] {
            let expected:std::collections::BTreeSet<_>=keys.iter().copied().collect();
            let mut insertion_ms=Vec::new();let mut bulk_ms=Vec::new();
            for round in 0..6 {for bulk in if round%2==0{[false,true]}else{[true,false]} {
                let started=Instant::now();for _ in 0..100 {
                    let actual:std::collections::BTreeSet<_>=if bulk{black_box(&keys).iter().copied().collect()}
                        else{let mut set=std::collections::BTreeSet::new();for &key in black_box(&keys){set.insert(key);}set};
                    assert_eq!(black_box(&actual),&expected);
                }
                let ms=started.elapsed().as_secs_f64()*1e3;if bulk{bulk_ms.push(ms)}else{insertion_ms.push(ms)}
            }}
            eprintln!("OBSERVATION_SET_BUILD label={label} cells={} loops=100 includes=construction_equality_retirement excludes=source_selection bulk_may_allocate_temporary_vector=true insertion_ms={insertion_ms:?} bulk_ms={bulk_ms:?}",keys.len());
        }
    }

    // Original rebuilding path retained only as a differential oracle and matched benchmark.
    // Returns the number of column vectors rebuilt (including unchanged columns).
    fn replace_block_reference(
        grid: &mut SurfaceObservations,
        block: [i32; 3],
        values: BTreeMap<[i32; 3], SurfaceObservation>,
    ) -> Result<usize, SurfaceError> {
        if values.len() > (grid.block_edge as usize).pow(3)
            || values.keys().any(|key| key.map(|v| v.div_euclid(grid.block_edge)) != block)
        {
            return Err(SurfaceError::InvalidConfig("观察块坐标或容量无效".into()));
        }
        grid.complete_geometry_source = false;
        let mut replacement = AHashMap::<[i32; 2], Vec<i32>>::new();
        for key in values.keys() {
            replacement.entry([key[0], key[1]]).or_default().push(key[2]);
        }
        let mut affected: AHashSet<[i32; 2]> = replacement.keys().copied().collect();
        if let Some(previous) = grid.blocks.get(&block) {
            affected.extend(previous.keys().map(|key| [key[0], key[1]]));
        }
        let rebuilt = affected.len();
        for column in affected {
            let mut levels = grid.columns.get(&column).map_or_else(Vec::new, |levels| levels.as_ref().clone());
            levels.retain(|z| z.div_euclid(grid.block_edge) != block[2]);
            if let Some(replacement) = replacement.remove(&column) { levels.extend(replacement); }
            levels.sort_unstable();
            levels.dedup();
            if levels.is_empty() { grid.columns.remove(&column); }
            else if grid.columns.get(&column).is_none_or(|previous| previous.as_slice() != levels.as_slice()) {
                grid.columns.insert(column, Arc::new(levels));
            }
        }
        if values.is_empty() { grid.blocks.remove(&block); }
        else { grid.blocks.insert(block, Arc::new(values.into_iter().collect())); }
        grid.obstacle_cache = Default::default();
        Ok(rebuilt)
    }

    #[test]
    fn block_updates_match_rebuilding_reference_for_exact_membership() {
        let mut actual = SurfaceObservations::new(0.05, 2).unwrap();
        let mut reference = actual.clone();
        // Include adjacent vertical blocks, negative coordinates, evidence refreshes,
        // equal-size key swaps, deletions, missing empty blocks and rejected coordinates.
        let updates = [
            ([0,0,-1], vec![[0,0,-1]]),
            ([0,0,1], vec![[0,0,2]]),
            ([-1,0,0], vec![[-1,0,0],[-2,1,1]]),
            ([0,0,0], vec![[0,0,0],[0,0,1]]),
            ([0,0,0], vec![[0,0,1],[0,0,0],[0,0,0]]),
            ([0,0,0], vec![[0,0,1],[1,0,0]]),
            ([0,0,0], vec![[0,0,1]]),
            ([0,0,0], vec![]),
            ([0,0,0], vec![]),
            ([0,0,0], vec![[0,0,2]]),
            ([-1,0,0], vec![]),
        ];
        for (index, (block, keys)) in updates.into_iter().enumerate() {
            actual.mark_complete_geometry_source();
            reference.mark_complete_geometry_source();
            actual.set_source_timestamp_ns(100 + index as u64);
            reference.set_source_timestamp_ns(100 + index as u64);
            let previous = actual.clone();
            let old_blocks = previous.blocks.clone();
            let old_columns = previous.columns.clone();
            // Duplicate inputs retain BTreeMap's last-value-wins semantics.
            let values: BTreeMap<_, _> = keys.into_iter().enumerate().map(|(n, key)|
                (key, SurfaceObservation { observed_ns: index as u64 + n as u64, hits: n as u16 })).collect();
            let result = actual.replace_block(block, values.clone());
            let expected = replace_block_reference(&mut reference, block, values);
            assert_eq!(result.is_ok(), expected.is_ok());
            assert_eq!(actual.blocks, reference.blocks);
            assert_eq!(actual.columns, reference.columns);
            assert_eq!(actual.complete_geometry_source, reference.complete_geometry_source);
            assert_eq!(previous.blocks, old_blocks);
            assert_eq!(previous.columns, old_columns);
            if result.is_err() {
                assert!(actual.blocks.ptr_eq(&previous.blocks));
                assert!(actual.columns.ptr_eq(&previous.columns));
                assert!(Arc::ptr_eq(&actual.obstacle_cache, &previous.obstacle_cache));
            } else {
                assert!(!Arc::ptr_eq(&actual.obstacle_cache, &previous.obstacle_cache));
            }
            for x in -2..=2 { for y in 0..=2 { for z in -2..=3 {
                let key = [x,y,z];
                assert_eq!(actual.get(key), reference.get(key));
                assert_eq!(actual.supports_obstacle(key), reference.supports_obstacle(key));
                assert_eq!(actual.has_complete_block(key), reference.has_complete_block(key));
            }}}
        }
    }

    #[test]
    fn evidence_refresh_reuses_geometry_but_retires_obstacle_permissions() {
        let key = [1,1,1];
        let mut grid = SurfaceObservations::new(0.05, 32).unwrap();
        grid.set_source_timestamp_ns(100);
        grid.replace_block([0,0,0], BTreeMap::from([
            (key, SurfaceObservation { observed_ns: 100, hits: 1 }),
        ])).unwrap();
        grid.mark_complete_geometry_source();
        assert!(grid.supports_obstacle(key));
        let original = grid.clone();
        // An older isolated observation must not inherit a current-frame permission.
        let older = SurfaceObservation { observed_ns: 1, hits: 1 };
        grid.replace_block([0,0,0], BTreeMap::from([(key, older)])).unwrap();
        assert!(grid.columns.ptr_eq(&original.columns));
        assert!(!grid.blocks.ptr_eq(&original.blocks));
        assert!(!grid.is_complete_geometry_source());
        assert!(!grid.same_geometry(&original));
        assert_eq!(grid.get(key), Some(older));
        assert!(!grid.supports_obstacle(key));
        assert!(original.supports_obstacle(key));
        assert_eq!(original.get(key).unwrap().observed_ns, 100);
        grid.mark_complete_geometry_source();
        assert!(grid.same_geometry(&original));
        let confirmed = SurfaceObservation { observed_ns: 1, hits: 3 };
        grid.replace_block([0,0,0], BTreeMap::from([(key, confirmed)])).unwrap();
        assert_eq!(grid.get(key), Some(confirmed));
        assert!(grid.supports_obstacle(key));
        grid.set_source_timestamp_ns(101);
        assert!(grid.supports_obstacle(key));
        assert!(original.supports_obstacle(key));
    }

    #[test]
    fn neighbor_evidence_refresh_retires_cached_cross_block_decisions() {
        let center = [1,0,0];
        let neighbor = [2,0,0];
        let weak = SurfaceObservation { observed_ns: 1, hits: 1 };
        let mut grid = SurfaceObservations::new(0.05, 2).unwrap();
        grid.set_source_timestamp_ns(100);
        grid.replace_block([0,0,0], BTreeMap::from([(center, weak)])).unwrap();
        grid.replace_block([1,0,0], BTreeMap::from([(neighbor, weak)])).unwrap();
        assert!(!grid.supports_obstacle(center));
        let previous = grid.clone();
        let strong = SurfaceObservation { observed_ns: 1, hits: 3 };
        grid.replace_block([1,0,0], BTreeMap::from([(neighbor, strong)])).unwrap();
        assert!(grid.columns.ptr_eq(&previous.columns));
        assert!(grid.supports_obstacle(center));
        assert!(!previous.supports_obstacle(center));
        assert_eq!(previous.get(neighbor), Some(weak));
    }

    #[test]
    #[ignore = "matched local observation-block microbenchmark; run explicitly with --nocapture"]
    fn benchmark_evidence_only_block_refresh() {
        // One fully occupied horizontal 32x32 block, not a full-map scan fixture.
        let values: BTreeMap<_, _> = (0..32).flat_map(|x| (0..32).map(move |y|
            ([x,y,0], SurfaceObservation { observed_ns: 1, hits: 1 }))).collect();
        let mut seed = SurfaceObservations::new(0.05, 32).unwrap();
        seed.replace_block([0,0,0], values.clone()).unwrap();
        let updates: Vec<_> = (2..202).map(|observed_ns| values.keys().map(|&key|
            (key, SurfaceObservation { observed_ns, hits: 2 })).collect::<BTreeMap<_, _>>()).collect();
        benchmark_updates("evidence-only", &seed, &updates, true);
        // Same cardinality, but every occupied layer changes; membership rejects on
        // its first key before taking the unchanged baseline reconstruction path.
        let changes: Vec<_> = (2..202).map(|observed_ns| values.keys().map(|&[x,y,_]|
            ([x,y,1 + (observed_ns % 2) as i32], SurfaceObservation { observed_ns, hits: 2 }))
            .collect::<BTreeMap<_, _>>()).collect();
        benchmark_updates("changed-membership", &seed, &changes, false);
        // A last-key-only layer toggle forces nearly the full membership scan before
        // falling back, bounding the extra lookup work on equal-cardinality changes.
        let late_changes: Vec<_> = (2..202).map(|observed_ns| values.keys().map(|&[x,y,z]| {
            let z = if x == 31 && y == 31 { 1 + (observed_ns % 2) as i32 } else { z };
            ([x,y,z], SurfaceObservation { observed_ns, hits: 2 })
        }).collect::<BTreeMap<_, _>>()).collect();
        benchmark_updates("late-key-changed-membership", &seed, &late_changes, false);
    }

    fn benchmark_updates(
        label: &str,
        seed: &SurfaceObservations,
        updates: &[BTreeMap<[i32; 3], SurfaceObservation>],
        unchanged_geometry: bool,
    ) {
        let run = |use_reference| {
            let mut grid = seed.clone();
            let start = std::time::Instant::now();
            let mut rebuilt_columns = 0;
            for update in updates {
                if use_reference {
                    rebuilt_columns += replace_block_reference(&mut grid, [0,0,0], update.clone()).unwrap();
                } else {
                    grid.replace_block([0,0,0], update.clone()).unwrap();
                }
            }
            (grid, start.elapsed(), rebuilt_columns)
        };
        // Warm both paths and alternate measured order to reduce order bias.
        run(true);
        run(false);
        let mut old_times = Vec::new();
        let mut new_times = Vec::new();
        for round in 0..5 {
            let (reference, optimized) = if round % 2 == 0 {
                (run(true), run(false))
            } else {
                let optimized = run(false);
                (run(true), optimized)
            };
            assert_eq!(optimized.0.blocks, reference.0.blocks);
            assert_eq!(optimized.0.columns, reference.0.columns);
            if unchanged_geometry { assert!(optimized.0.columns.ptr_eq(&seed.columns)); }
            assert_eq!(reference.2, updates.len() * 1024);
            old_times.push(reference.1);
            new_times.push(optimized.1);
        }
        old_times.sort();
        new_times.sort();
        eprintln!("{label}: {} matched block updates, five alternating rounds; median reference={:?}, optimized={:?}; reference rebuilt {} column vectors; optimized rebuilt {}",
            updates.len(), old_times[2], new_times[2], updates.len() * 1024,
            if unchanged_geometry { 0 } else { updates.len() * 1024 });
    }

    #[test]
    fn planning_fork_shares_observation_roots_without_sharing_obstacle_cache() {
        let mut source = SurfaceObservations::new(0.1, 32).unwrap();
        source.replace_block([0,0,0], BTreeMap::from([
            ([1,1,1], SurfaceObservation {observed_ns: 100, hits: 1}),
            ([10,10,10], SurfaceObservation {observed_ns: 1, hits: 1}),
        ])).unwrap();
        source.set_source_timestamp_ns(100);
        source.mark_complete_geometry_source();
        let expected: Vec<_> = [[1,1,1],[10,10,10],[20,20,20]].map(|key| source.supports_obstacle(key)).into();
        let guard = source.obstacle_cache.lock().unwrap();
        let fork = source.fork_for_planning();
        assert!(source.blocks.ptr_eq(&fork.blocks));
        assert!(source.columns.ptr_eq(&fork.columns));
        assert!(!Arc::ptr_eq(&source.obstacle_cache, &fork.obstacle_cache));
        assert_eq!(source.source_timestamp_ns, fork.source_timestamp_ns);
        assert_eq!(source.complete_geometry_source, fork.complete_geometry_source);
        assert_eq!(source.block_edge, fork.block_edge);
        assert!(fork.obstacle_cache.try_lock().unwrap().is_empty());
        for (key, expected) in [[1,1,1],[10,10,10],[20,20,20]].into_iter().zip(expected) {
            assert_eq!(source.get(key), fork.get(key));
            assert_eq!(fork.supports_obstacle(key), expected);
        }
        fork.obstacle_cache.lock().unwrap().clear();
        assert_eq!(guard.len(), 3);
    }

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
            ground_evidence:self.ground_evidence,recent_obstacles:self.recent_obstacles.clone(),
            data: Arc::clone(&self.data),
            observations,
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

#[cfg(test)]
mod measured_layer_tests {
    use super::*;
    use nuwax_map_chunks::{LayerObservation,LayerObservations,LayerSample};

    fn record(plane:f64,sample:f64,time:u64)->Arc<LayerObservations> {
        Arc::new(BTreeMap::from([(0,LayerObservation {members:vec![0],normal:[0.0,0.0,1.0],
            plane_origin:[0.025,0.025,plane],samples:vec![LayerSample {source: Default::default(),
                point:[0.025,0.025,sample],source_sequence:1,source_timestamp_ns:time,point_index:0,
            }],})]))
    }
    fn source()->SurfaceObservations {
        let mut source=SurfaceObservations::new(0.05,32).unwrap();
        source.replace_block([0,0,0],BTreeMap::from([([0,0,0],SurfaceObservation{observed_ns:100,hits:1})])).unwrap();
        source.mark_complete_geometry_source();source
    }

    #[test]
    fn measured_layer_diffs_ignore_diagnostic_samples_but_preserve_plane_and_deletion() {
        let mut source=source();
        assert_eq!(source.replace_ground_layers([0,0,0],record(0.025,0.02,10)).unwrap(),vec![[0,0]]);
        source.mark_complete_geometry_source();let first=source.clone();
        assert_eq!(source.ground_layers([0,0]).next().unwrap().height_at([0.025,0.025]),Some(0.025));
        assert!(source.replace_ground_layers([0,0,0],record(0.025,0.03,20)).unwrap().is_empty());
        source.mark_complete_geometry_source();assert!(source.same_geometry(&first));
        assert_ne!(source.ground_layers([0,0]).next().unwrap().statistics,
            first.ground_layers([0,0]).next().unwrap().statistics,"updated bounded diagnostics are retained");
        assert_eq!(source.replace_ground_layers([0,0,0],record(0.03,0.03,20)).unwrap(),vec![[0,0]]);
        source.mark_complete_geometry_source();assert!(!source.same_geometry(&first));
        let changed=source.replace_block_with_geometry_changes([0,0,0],BTreeMap::new()).unwrap();
        assert_eq!(changed,vec![[0,0]]);assert_eq!(source.ground_layers([0,0]).count(),0);
        assert_eq!(first.ground_layers([0,0]).count(),1,"old snapshot remains immutable");
    }

    #[test]
    fn measured_layer_replacement_is_transactional_and_rejects_fabricated_members() {
        let mut source=source();source.replace_ground_layers([0,0,0],record(0.025,0.02,10)).unwrap();
        source.mark_complete_geometry_source();let old=source.clone();
        for member in [1_u16,32_u16] {
            let mut invalid=record(0.025,0.02,10).as_ref().clone();
            let mut layer=invalid.remove(&0).unwrap();layer.members=vec![member];invalid.insert(member,layer);
            assert!(source.replace_ground_layers([0,0,0],Arc::new(invalid)).is_err());
            assert!(source.same_geometry(&old));
            assert_eq!(source.ground_layers([0,0]).next().unwrap().height_at([0.025,0.025]),Some(0.025));
        }
        assert_eq!(source.replace_ground_layers([0,0,0],Arc::new(BTreeMap::new())).unwrap(),vec![[0,0]]);
        assert_eq!(source.ground_layers([0,0]).count(),0);
        assert!(source.get([0,0,0]).is_some(),"removing layer metadata does not clear occupancy");
    }
}

#[cfg(test)]
mod planning_query_equivalence_tests {
    use super::*;
    use nuwax_map_chunks::surface_statistics::ObservationStatistics;
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample, LayerSource};

    fn statistics(points: &[[f64; 3]]) -> ObservationStatistics {
        assert!(!points.is_empty());
        if points.len() == 1 {
            return ObservationStatistics::from_point(points[0]).unwrap();
        }
        let (left, right) = points.split_at(points.len() / 2);
        statistics(left).merged(&statistics(right)).unwrap()
    }

    // The field layout is intentionally explicit: count, mean XYZ, all six M2
    // terms, minimum XYZ, maximum XYZ. Tests never forge private math fields.
    fn statistics_bits(value: ObservationStatistics) -> [u64; 16] {
        let moments = value.moments();
        let (minimum, maximum) = value.observed_extent();
        let mut bits = [0; 16];
        bits[0] = moments.count();
        bits[1..4].copy_from_slice(&moments.mean().map(f64::to_bits));
        bits[4..10].copy_from_slice(&moments.m2().map(f64::to_bits));
        bits[10..13].copy_from_slice(&minimum.map(f64::to_bits));
        bits[13..16].copy_from_slice(&maximum.map(f64::to_bits));
        bits
    }

    fn layer(statistics: ObservationStatistics) -> ObservedGroundLayer {
        ObservedGroundLayer {
            anchor: [0, 0, 0],
            members: vec![0, 1],
            statistics,
            normal: [0.0, 0.0, 1.0],
            plane_origin: [0.025, 0.025, 0.025],
        }
    }

    fn assert_only_statistics_field_changes(
        left: ObservationStatistics,
        right: ObservationStatistics,
        expected: usize,
    ) {
        let left_bits = statistics_bits(left);
        let right_bits = statistics_bits(right);
        let changed: Vec<_> = (0..16).filter(|&i| left_bits[i] != right_bits[i]).collect();
        assert_eq!(changed, vec![expected], "the fixture must isolate its stated field");
        let left = layer(left);
        let right = layer(right);
        assert!(left.geometry_eq(&right), "same_geometry keeps its existing contract");
        assert!(left.query_eq(&left.clone()));
        assert!(!left.query_eq(&right), "omitted stored field {expected}");
        assert!(!right.query_eq(&left), "comparison must be symmetric");
    }

    #[test]
    fn planning_layer_query_equality_checks_each_stored_statistics_field() {
        assert_only_statistics_field_changes(
            statistics(&[[0.0; 3]]), statistics(&[[0.0; 3], [0.0; 3]]), 0,
        );
        for axis in 0..3 {
            let point = |value| {
                let mut result = [0.0; 3];
                result[axis] = value;
                result
            };
            // Balanced binary merges make every intermediate dyadic/exact.
            assert_only_statistics_field_changes(
                statistics(&[point(0.0), point(0.0), point(0.0), point(4.0)]),
                statistics(&[point(0.0), point(4.0), point(4.0), point(4.0)]),
                1 + axis,
            );
            assert_only_statistics_field_changes(
                statistics(&[point(-2.0), point(2.0), point(0.0), point(0.0)]),
                statistics(&[point(-2.0), point(2.0), point(-1.0), point(1.0)]),
                4 + [0, 3, 5][axis],
            );
            assert_only_statistics_field_changes(
                statistics(&[point(0.0), point(4.0), point(3.0), point(3.0)]),
                statistics(&[point(1.0), point(1.0), point(4.0), point(4.0)]),
                10 + axis,
            );
            assert_only_statistics_field_changes(
                statistics(&[point(0.0), point(-4.0), point(-3.0), point(-3.0)]),
                statistics(&[point(-1.0), point(-1.0), point(-4.0), point(-4.0)]),
                13 + axis,
            );
        }
        for (x, y, m2_index) in [(0, 1, 1), (0, 2, 2), (1, 2, 4)] {
            let point = |a, b| {
                let mut result = [0.0; 3];
                result[x] = a;
                result[y] = b;
                result
            };
            assert_only_statistics_field_changes(
                statistics(&[point(1.0, 1.0), point(-1.0, -1.0),
                    point(1.0, -1.0), point(-1.0, 1.0)]),
                statistics(&[point(1.0, 1.0), point(-1.0, -1.0),
                    point(1.0, 1.0), point(-1.0, -1.0)]),
                4 + m2_index,
            );
        }
    }

    #[test]
    fn planning_layer_query_equality_checks_anchor_members_normal_origin_and_signed_zero() {
        let original = layer(statistics(&[[0.0; 3]]));
        for axis in 0..3 {
            let mut changed = original.clone();
            changed.anchor[axis] += 1;
            assert!(!original.query_eq(&changed), "anchor axis {axis}");
            let mut changed = original.clone();
            changed.normal[axis] = f64::from_bits(changed.normal[axis].to_bits() + 1);
            assert!(!original.query_eq(&changed), "normal axis {axis}");
            let mut changed = original.clone();
            changed.plane_origin[axis] = f64::from_bits(changed.plane_origin[axis].to_bits() + 1);
            assert!(!original.query_eq(&changed), "plane origin axis {axis}");

            let mut positive = original.clone();
            let mut negative = original.clone();
            positive.plane_origin[axis] = 0.0;
            negative.plane_origin[axis] = -0.0;
            assert!(positive.geometry_eq(&negative));
            assert!(!positive.query_eq(&negative), "origin zero sign axis {axis}");

            // A singleton preserves the supplied signs in mean and both extrema.
            let mut point = [0.0; 3];
            point[axis] = -0.0;
            let negative = layer(statistics(&[point]));
            let left = statistics_bits(original.statistics);
            let right = statistics_bits(negative.statistics);
            let changed: Vec<_> = (0..16).filter(|&i| left[i] != right[i]).collect();
            assert_eq!(changed, vec![1 + axis, 10 + axis, 13 + axis]);
            assert_eq!(original.statistics, negative.statistics, "numeric == merges zero signs");
            assert!(!original.query_eq(&negative), "stored statistics zero sign axis {axis}");
        }
        for axis in 0..2 {
            let mut negative = original.clone();
            negative.normal[axis] = -0.0;
            assert!(original.geometry_eq(&negative));
            assert!(!original.query_eq(&negative), "normal zero sign axis {axis}");
        }
        for members in [vec![0], vec![0, 2], vec![0, 1, 2], vec![1, 0]] {
            let mut changed = original.clone();
            changed.members = members;
            assert!(!original.query_eq(&changed));
        }
    }

    fn record(time: u64, sample_z: f64) -> Arc<LayerObservations> {
        Arc::new(BTreeMap::from([(0, LayerObservation {
            members: vec![nuwax_map_chunks::local_key([0, 0, 0]),
                nuwax_map_chunks::local_key([0, 0, 1])],
            normal: [0.0, 0.0, 1.0],
            plane_origin: [0.025, 0.025, 0.025],
            samples: vec![LayerSample {
                source: Default::default(), point: [0.025, 0.025, sample_z],
                source_sequence: 1, source_timestamp_ns: time, point_index: 0,
            }],
        })]))
    }

    fn occupancy(time: u64, hits: u16) -> BTreeMap<[i32; 3], SurfaceObservation> {
        [[0, 0, 0], [0, 0, 1], [1, 0, 0]].into_iter()
            .map(|key| (key, SurfaceObservation { observed_ns: time, hits })).collect()
    }

    fn source() -> SurfaceObservations {
        let mut source = SurfaceObservations::new(0.05, 32).unwrap();
        source.replace_block([0, 0, 0], occupancy(100, 1)).unwrap();
        source.replace_ground_layers([0, 0, 0], record(10, 0.02)).unwrap();
        source.mark_complete_geometry_source();
        source
    }

    #[test]
    fn planning_equivalent_layer_replacement_keeps_payload_root_and_new_raw_evidence() {
        let original = source();
        let mut refreshed = original.clone();
        let old_payload = original.observed_layers.get(&[0, 0, 0]).unwrap().clone();
        assert!(refreshed.replace_block_with_geometry_changes([0, 0, 0], occupancy(200, 2))
            .unwrap().is_empty());
        assert!(!refreshed.same_planning_query_view(&original), "partial is not complete");
        let mut next = record(20, 0.02).as_ref().clone();
        let sample = &mut next.get_mut(&0).unwrap().samples[0];
        sample.source = Arc::new(LayerSource::Lidar {
            backend: 1, runtime_generation: 1, sensor_frame_id: "lidar".into(),
        });
        sample.source_sequence = 2;
        sample.point_index = 17;
        assert!(refreshed.replace_ground_layers([0, 0, 0], Arc::new(next)).unwrap().is_empty());
        refreshed.mark_complete_geometry_source();
        assert!(original.columns.ptr_eq(&refreshed.columns));
        assert!(original.observed_layers.ptr_eq(&refreshed.observed_layers));
        assert!(Arc::ptr_eq(&old_payload, refreshed.observed_layers.get(&[0, 0, 0]).unwrap()));
        assert!(!original.blocks.ptr_eq(&refreshed.blocks), "fresh raw evidence remains installed");
        assert!(original.same_geometry(&refreshed));
        assert!(original.same_planning_query_view(&refreshed));
        assert!(refreshed.same_planning_query_view(&original));
        assert_eq!(original.get([0, 0, 0]), Some(SurfaceObservation { observed_ns: 100, hits: 1 }));
        assert_eq!(refreshed.get([0, 0, 0]), Some(SurfaceObservation { observed_ns: 200, hits: 2 }));
        assert_eq!(statistics_bits(original.ground_layers([0, 0]).next().unwrap().statistics),
            statistics_bits(refreshed.ground_layers([0, 0]).next().unwrap().statistics));
    }

    #[test]
    fn planning_layer_statistics_change_retires_query_but_not_geometry_identity() {
        let original = source();
        let mut changed = original.clone();
        assert!(changed.replace_ground_layers([0, 0, 0], record(20, 0.03)).unwrap().is_empty());
        changed.mark_complete_geometry_source();
        assert!(original.columns.ptr_eq(&changed.columns));
        assert!(!original.observed_layers.ptr_eq(&changed.observed_layers));
        assert!(original.same_geometry(&changed));
        assert!(!original.same_planning_query_view(&changed));
        assert_eq!(original.ground_layers([0, 0]).next().unwrap().statistics.moments().mean()[2], 0.02);
        assert_eq!(changed.ground_layers([0, 0]).next().unwrap().statistics.moments().mean()[2], 0.03);
    }

    #[test]
    fn planning_layer_plane_member_anchor_and_removal_changes_are_not_equivalent() {
        let original = source();
        for variant in 0..5 {
            let mut changed = original.clone();
            let mut next = record(10, 0.02).as_ref().clone();
            match variant {
                0 => next.get_mut(&0).unwrap().plane_origin[2] = 0.035,
                1 => next.get_mut(&0).unwrap().normal = [0.6, 0.0, 0.8],
                2 => next.get_mut(&0).unwrap().members.truncate(1),
                3 => {
                    let mut moved = next.remove(&0).unwrap();
                    let anchor = nuwax_map_chunks::local_key([0, 0, 1]);
                    moved.members = vec![anchor];
                    next.insert(anchor, moved);
                }
                _ => next.clear(),
            }
            assert!(!changed.replace_ground_layers([0, 0, 0], Arc::new(next)).unwrap().is_empty());
            changed.mark_complete_geometry_source();
            assert!(original.columns.ptr_eq(&changed.columns));
            assert!(!original.same_geometry(&changed));
            assert!(!original.same_planning_query_view(&changed), "layer variant {variant}");
        }
    }

    #[test]
    fn planning_layer_signed_zero_changes_payload_without_widening_geometry_equality() {
        let original = source();
        let mut changed = original.clone();
        let mut next = record(10, 0.02).as_ref().clone();
        next.get_mut(&0).unwrap().normal[0] = -0.0;
        assert!(changed.replace_ground_layers([0, 0, 0], Arc::new(next)).unwrap().is_empty());
        assert!(original.same_geometry(&changed));
        assert!(!original.observed_layers.ptr_eq(&changed.observed_layers));
        assert!(!original.same_planning_query_view(&changed));
    }

    #[test]
    fn planning_query_equality_keeps_size_block_partition_and_completeness() {
        let mut original = SurfaceObservations::new(0.05, 32).unwrap();
        original.replace_block([0, 0, 0], occupancy(100, 1)).unwrap();
        let mut complete = original.clone();
        complete.mark_complete_geometry_source();
        assert!(!original.same_planning_query_view(&complete));
        assert!(!complete.same_planning_query_view(&original));
        let mut resized = original.clone();
        resized.size_m = f32::from_bits(original.size_m.to_bits() + 1);
        assert!(!original.same_planning_query_view(&resized));
        // All fixture keys fit both partitions; only the existing block's covered
        // empty positions differ. Preserve roots to isolate the scalar guard.
        let mut repartitioned = original.clone();
        repartitioned.block_edge = 16;
        assert!(original.columns.ptr_eq(&repartitioned.columns));
        assert!(original.observed_layers.ptr_eq(&repartitioned.observed_layers));
        assert!(original.same_geometry(&repartitioned), "do not silently change same_geometry");
        assert!(original.has_complete_block([31, 0, 0]));
        assert!(!repartitioned.has_complete_block([31, 0, 0]));
        assert!(!original.same_planning_query_view(&repartitioned));
    }

    #[test]
    fn planning_metadata_refresh_preserves_exact_presence_across_signed_block_boundaries() {
        let keys = [[-33, -1, -1], [-32, 0, 0], [-1, 31, 31], [0, 0, 0],
            [31, 31, 31], [32, 32, 32], [63, 0, 0], [64, 0, 0]];
        let mut blocks: BTreeMap<[i32; 3], BTreeMap<[i32; 3], SurfaceObservation>> = BTreeMap::new();
        for key in keys {
            blocks.entry(nuwax_map_chunks::block_key(key)).or_default()
                .insert(key, SurfaceObservation { observed_ns: 100, hits: 1 });
        }
        let mut original = SurfaceObservations::new(0.05, 32).unwrap();
        for (&block, values) in &blocks {
            original.replace_block(block, values.clone()).unwrap();
        }
        original.mark_complete_geometry_source();
        let mut refreshed = original.clone();
        for (&block, values) in &blocks {
            let updated = values.keys().map(|&key|
                (key, SurfaceObservation { observed_ns: 200, hits: 2 })).collect();
            assert!(refreshed.replace_block_with_geometry_changes(block, updated).unwrap().is_empty());
            assert!(refreshed.replace_ground_layers(block, Arc::new(BTreeMap::new())).unwrap().is_empty());
        }
        refreshed.mark_complete_geometry_source();
        assert!(original.same_planning_query_view(&refreshed));
        let probes = [-65, -64, -33, -32, -31, -1, 0, 1, 30, 31, 32, 33, 63, 64, 65];
        for x in probes { for y in probes { for z in probes {
            let key = [x, y, z];
            let present = keys.contains(&key);
            let block_present = blocks.contains_key(&nuwax_map_chunks::block_key(key));
            assert_eq!(original.get(key).is_some(), present);
            assert_eq!(refreshed.get(key).is_some(), present);
            assert_eq!(original.has_complete_block(key), block_present);
            assert_eq!(refreshed.has_complete_block(key), block_present);
        }}}
        let mut removed = refreshed.clone();
        removed.replace_block([1, 1, 1], BTreeMap::new()).unwrap();
        removed.mark_complete_geometry_source();
        assert!(!refreshed.same_planning_query_view(&removed));
        assert!(refreshed.has_complete_block([33, 33, 33]));
        assert!(!removed.has_complete_block([33, 33, 33]));
        assert!(refreshed.get([32, 32, 32]).is_some());
        assert!(removed.get([32, 32, 32]).is_none());
    }

    #[test]
    fn planning_occupancy_add_remove_and_move_retires_view_even_with_same_coarse_owner() {
        let original = source();
        for variant in 0..3 {
            let mut changed = original.clone();
            let mut values = occupancy(100, 1);
            match variant {
                0 => { values.insert([2, 0, 0], SurfaceObservation { observed_ns: 100, hits: 1 }); }
                1 => { values.remove(&[1, 0, 0]); }
                _ => { values.remove(&[0, 0, 1]);
                    values.insert([0, 0, 2], SurfaceObservation { observed_ns: 100, hits: 1 }); }
            }
            assert!(!changed.replace_block_with_geometry_changes([0, 0, 0], values).unwrap().is_empty());
            changed.mark_complete_geometry_source();
            assert!(!original.same_planning_query_view(&changed), "occupancy variant {variant}");
        }
        assert_eq!(original.ground_layers([0, 0]).count(), 1, "retained source is unchanged");
    }

    #[test]
    fn planning_observation_forks_keep_shared_payload_but_divergent_cow_branches_are_distinct() {
        let original = source();
        let fork = original.fork_for_planning();
        assert!(original.same_planning_query_view(&fork));
        assert!(original.blocks.ptr_eq(&fork.blocks));
        assert!(original.columns.ptr_eq(&fork.columns));
        assert!(original.observed_layers.ptr_eq(&fork.observed_layers));
        let mut left = fork.clone();
        let mut right = fork.clone();
        left.replace_ground_layers([0, 0, 0], record(20, 0.03)).unwrap();
        right.replace_ground_layers([0, 0, 0], record(20, 0.04)).unwrap();
        assert!(original.same_geometry(&left));
        assert!(original.same_geometry(&right));
        assert!(!original.same_planning_query_view(&left));
        assert!(!original.same_planning_query_view(&right));
        assert!(!left.same_planning_query_view(&right), "equal update counts are not identity");
        assert!(original.same_planning_query_view(&fork), "COW never alters retained roots");
        let rebuilt = source();
        assert_eq!(statistics_bits(original.ground_layers([0, 0]).next().unwrap().statistics),
            statistics_bits(rebuilt.ground_layers([0, 0]).next().unwrap().statistics));
        assert!(!original.same_planning_query_view(&rebuilt), "no full-content fallback for independent roots");
    }

    #[test]
    fn planning_observation_geometry_round_trip_does_not_recover_old_persistent_identity() {
        let original = source();
        let mut changed = original.clone();
        changed.replace_block([0, 0, 0], BTreeMap::from([
            ([0, 0, 0], SurfaceObservation { observed_ns: 100, hits: 1 }),
        ])).unwrap();
        changed.replace_block([0, 0, 0], occupancy(100, 1)).unwrap();
        changed.replace_ground_layers([0, 0, 0], record(10, 0.02)).unwrap();
        changed.mark_complete_geometry_source();
        assert_eq!(original.column_levels(0, 0), changed.column_levels(0, 0));
        assert_eq!(original.column_levels(1, 0), changed.column_levels(1, 0));
        assert!(original.ground_layers([0, 0]).next().unwrap()
            .query_eq(changed.ground_layers([0, 0]).next().unwrap()));
        assert!(!original.columns.ptr_eq(&changed.columns));
        assert!(!original.same_planning_query_view(&changed));
    }

    #[test]
    fn planning_query_payload_reuse_never_bypasses_layer_validation() {
        let original = source();
        for variant in 0..4 {
            let mut next = record(10, 0.02).as_ref().clone();
            match variant {
                0 => next.get_mut(&0).unwrap().members.push(nuwax_map_chunks::local_key([0, 0, 2])),
                1 => next.get_mut(&0).unwrap().members.reverse(),
                2 => next.get_mut(&0).unwrap().samples.clear(),
                _ => next.get_mut(&0).unwrap().samples[0].point[2] = f64::NAN,
            }
            let mut changed = original.clone();
            assert!(changed.replace_ground_layers([0, 0, 0], Arc::new(next)).is_err());
            assert!(original.same_planning_query_view(&changed), "rejected conversion is transactional");
            assert!(original.observed_layers.ptr_eq(&changed.observed_layers));
            assert_eq!(changed.get([0, 0, 0]), original.get([0, 0, 0]));
        }
    }
}
