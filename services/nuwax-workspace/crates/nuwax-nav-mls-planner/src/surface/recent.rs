//! Independent source-time-window obstacle returns. No terrain or ground lease.
use super::*;
use std::collections::BTreeMap;

pub(super) struct RecentFrame {
    pub sequence:u64,
    pub timestamp:u64,
    pub columns:BTreeMap<[i32;2],Vec<i32>>,
    pub surface_points:AHashMap<[i32;3],Vec<[f64;3]>>,
}
pub(super) struct RecentObstacles {
    pub size:f64,
    pub frames:Vec<Arc<RecentFrame>>,
}
impl SurfaceSnapshot {
    /// Replace the live obstacle view from one admitted, fully domain-checked cloud.
    /// Indices are original PointCloud2 indices. Zero retention keeps only this frame.
    /// At most eight immutable frames are retained, independently of terrain identity.
    pub fn with_recent_obstacle_points(&self,source_sequence:u64,source_timestamp_ns:u64,
        retention_ns:u64,ground_point_indices:&[u32],
        points:impl IntoIterator<Item=(u32,[f64;3])>)->Result<Self,SurfaceError> {
        if source_sequence==0 || source_timestamp_ns==0 {
            return Err(SurfaceError::InvalidConfig("recent cloud identity must be nonzero".into()));
        }
        let size=f64::from(self.source_voxel_size_m());
        if let Some(previous)=self.recent_obstacles.as_ref().and_then(|r|r.frames.last()) {
            if source_sequence<=previous.sequence || source_timestamp_ns<previous.timestamp {
                return Err(SurfaceError::InvalidConfig("recent cloud is not source-monotonic".into()));
            }
        }
        let ground:AHashSet<u32>=ground_point_indices.iter().copied().collect();
        let mut columns:BTreeMap<[i32;2],Vec<i32>>=BTreeMap::new();
        let mut surface_points:AHashMap<[i32;3],Vec<[f64;3]>>=AHashMap::new();
        let mut obstacles=AHashSet::new();
        for (index,point) in points {
            if !point.into_iter().all(f64::is_finite) {continue;}
            let scaled=point.map(|x|(x/size).floor());
            if scaled.into_iter().any(|x|x<f64::from(i32::MIN) || x>=f64::from(i32::MAX)) {
                return Err(SurfaceError::InvalidConfig("recent point exceeds voxel range".into()));
            }
            let [x,y,z]=scaled.map(|x|x as i32);
            columns.entry([x,y]).or_default().push(z);
            if ground.contains(&index) {surface_points.entry([x,y,z]).or_default().push(point);}
            else {obstacles.insert([x,y,z]);}
        }
        surface_points.retain(|key,_|!obstacles.contains(key));
        for zs in columns.values_mut() {zs.sort_unstable();zs.dedup();}
        let mut frames=Vec::new();
        if retention_ns>0 {
            if let Some(previous)=&self.recent_obstacles {
                if previous.size==size {
                    frames.extend(previous.frames.iter().filter(|frame|
                        source_timestamp_ns.saturating_sub(frame.timestamp)<=retention_ns).cloned());
                }
            }
        }
        if frames.len()>=8 {frames.drain(..frames.len()-7);}
        frames.push(Arc::new(RecentFrame{sequence:source_sequence,timestamp:source_timestamp_ns,columns,surface_points}));
        let mut result=self.clone();
        result.recent_obstacles=Some(Arc::new(RecentObstacles{size,frames}));
        result.body_query_cache=Default::default();
        Ok(result)
    }
    /// Reset independent live observations after authority or source-generation change.
    pub fn clear_recent_obstacles(&self)->Self {
        let mut result=self.clone();result.recent_obstacles=None;
        result.body_query_cache=Default::default();result
    }
    /// Preserve the admitted live source across a same-domain terrain refresh.
    pub fn with_recent_obstacles_from(&self,previous:&Self)->Self {
        let mut result=self.clone();result.recent_obstacles=previous.recent_obstacles.clone();
        result.body_query_cache=Default::default();result
    }
}
