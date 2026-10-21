//! Independent source-time-window obstacle returns. No terrain or ground lease.
use super::*;
use std::collections::BTreeMap;

#[cfg(test)]
#[path = "recent_columns_diagnostics.rs"]
mod columns_diagnostics;

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
    /// 导出原生规划器使用的近期观测；实测地面保留原高度，其余占据按源体素输出。
    pub fn navigation_recent_samples(&self) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
        let mut ground = Vec::new();
        let mut obstacles = Vec::new();
        if let Some(recent) = &self.recent_obstacles {
            for frame in &recent.frames {
                for (&[x, y], heights) in &frame.columns {
                    for &z in heights {
                        if let Some(measured) = frame.surface_points.get(&[x, y, z]) {
                            ground.extend_from_slice(measured);
                        } else {
                            obstacles.push(
                                [x, y, z].map(|value| (f64::from(value) + 0.5) * recent.size),
                            );
                        }
                    }
                }
            }
        }
        (ground, obstacles)
    }

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
        let mut voxel_keys=Vec::new();
        let mut surface_points:AHashMap<[i32;3],Vec<[f64;3]>>=AHashMap::new();
        let mut obstacles=AHashSet::new();
        for (index,point) in points {
            if !point.into_iter().all(f64::is_finite) {continue;}
            let scaled=point.map(|x|(x/size).floor());
            if scaled.into_iter().any(|x|x<f64::from(i32::MIN) || x>=f64::from(i32::MAX)) {
                return Err(SurfaceError::InvalidConfig("recent point exceeds voxel range".into()));
            }
            let [x,y,z]=scaled.map(|x|x as i32);
            voxel_keys.push([x,y,z]);
            if ground.contains(&index) {surface_points.entry([x,y,z]).or_default().push(point);}
            else {obstacles.insert([x,y,z]);}
        }
        surface_points.retain(|key,_|!obstacles.contains(key));
        // Sort only integer occupancy keys. Ground-return point order stays unchanged.
        voxel_keys.sort_unstable();
        voxel_keys.dedup();
        let mut groups=Vec::new();
        for column in voxel_keys.chunk_by(|a,b|a[..2]==b[..2]) {
            let [x,y,_]=column[0];
            let heights:Vec<i32>=column.iter().map(|key|key[2]).collect();
            groups.push(([x,y],heights));
        }
        // Retire flat scratch before the final tree's bulk construction.
        drop(voxel_keys);
        let columns:BTreeMap<[i32;2],Vec<i32>>=groups.into_iter().collect();
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

// Exact swept disk against a point/voxel after clipping the segment to the
// obstacle's vertical overlap interval. Ground contact itself is not volume.
fn planar_recent_overlap(from:[f64;3],to:[f64;3],radius:f64,height:f64,
    low:[f64;3],high:[f64;3])->bool {
    let dz=to[2]-from[2];
    let lower=low[2]-height+1e-6;let upper=high[2]-1e-6;
    let (mut first,mut last)=(0.0_f64,1.0_f64);
    if dz==0.0 {
        if from[2]<lower || from[2]>upper {return false;}
    } else {
        let a=(lower-from[2])/dz;let b=(upper-from[2])/dz;
        first=first.max(a.min(b));last=last.min(a.max(b));
        if first>last {return false;}
    }
    let at=|t:f64|[from[0]+(to[0]-from[0])*t,from[1]+(to[1]-from[1])*t];
    let a=at(first);let b=at(last);let d=[b[0]-a[0],b[1]-a[1]];
    let (mut enter,mut leave)=(0.0_f64,1.0_f64);
    let crosses=(0..2).all(|axis| {
        if d[axis]==0.0 {return a[axis]>=low[axis] && a[axis]<=high[axis];}
        let t0=(low[axis]-a[axis])/d[axis];let t1=(high[axis]-a[axis])/d[axis];
        enter=enter.max(t0.min(t1));leave=leave.min(t0.max(t1));enter<=leave
    });
    if crosses {return true;}
    let rectangle_distance=|p:[f64;2]| {
        let q:[f64;2]=std::array::from_fn(|axis|p[axis].clamp(low[axis],high[axis]));
        (p[0]-q[0]).hypot(p[1]-q[1])
    };
    let point_distance=|p:[f64;2]| {
        let norm=d[0]*d[0]+d[1]*d[1];
        let t=if norm==0.0 {0.0}else{((p[0]-a[0])*d[0]+(p[1]-a[1])*d[1])/norm}.clamp(0.0,1.0);
        (p[0]-a[0]-t*d[0]).hypot(p[1]-a[1]-t*d[1])
    };
    let mut distance=rectangle_distance(a).min(rectangle_distance(b));
    for x in [low[0],high[0]] {for y in [low[1],high[1]] {distance=distance.min(point_distance([x,y]));}}
    distance<=radius+1e-9
}

impl SurfaceSnapshot {
    /// Check the existing legacy planar radius/height against the current recent
    /// source. This is only the recent-obstacle component: callers must also
    /// check persistent geometry, known terrain and forbidden boundaries.
    /// No body envelope, ground observation, or map mutation is created.
    pub fn validate_planar_recent_motion(&self,from:[f64;3],to:[f64;3])->Result<(),SurfaceError> {
        if !from.into_iter().chain(to).all(f64::is_finite) {return Err(SurfaceError::InvalidPose);}
        let Some(recent)=&self.recent_obstacles else {return Ok(());};
        let radius=f64::from(self.config().wall_clearance_m);
        let height=f64::from(self.config().robot_height);
        let size=recent.size;
        let low:[i32;2]=std::array::from_fn(|axis|((from[axis].min(to[axis])-radius)/size).floor() as i32);
        let high:[i32;2]=std::array::from_fn(|axis|((from[axis].max(to[axis])+radius)/size).floor() as i32);
        for frame in &recent.frames {for (&[x,y],levels) in frame.columns.range(low..=high) {
            if y<low[1] || y>high[1] {continue;}
            for &z in levels {
                let key=[x,y,z];
                if let Some(points)=frame.surface_points.get(&key) {
                    if points.iter().any(|&point|planar_recent_overlap(from,to,radius,height,point,point)) {
                        return Err(SurfaceError::InsufficientClearance);
                    }
                } else if planar_recent_overlap(from,to,radius,height,
                    key.map(|v|f64::from(v)*size),key.map(|v|(f64::from(v)+1.0)*size)) {
                    return Err(SurfaceError::InsufficientClearance);
                }
            }
        }}
        Ok(())
    }
}

#[cfg(test)]
mod planar_tests {
    use super::*;
    fn source()->SurfaceSnapshot {
        let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.6,
            surface_closing_radius:0.0,node_spacing_m:0.4,wall_clearance_m:0.2,
            wall_buffer_m:0.2,wall_buffer_weight:0.0,step_threshold_m:0.2,max_slope_deg:35.0,
            min_step_tread_depth_m:0.3,step_penalty_weight:0.0,goal_tolerance:0.1,viz_publish_hz:0.0};
        SurfaceSnapshot::from_points(&[(10.05,10.05,0.05)],config,0.1).unwrap()
    }
    #[test]
    fn planar_recent_sweep_keeps_low_obstacles_and_uses_current_source() {
        let source=source();let from=[0.0,0.0,0.1];let to=[1.0,0.0,0.1];
        assert!(source.body_envelope().is_none());
        for point in [[0.5,0.0,0.15],[0.5,0.0,0.65],[0.5,0.249,0.35]] {
            let blocked=source.with_recent_obstacle_points(1,1,0,&[],[(0,point)]).unwrap();
            assert!(blocked.validate_planar_recent_motion(from,from).is_ok());
            assert!(blocked.validate_planar_recent_motion(to,to).is_ok());
            assert_eq!(blocked.validate_planar_recent_motion(from,to),Err(SurfaceError::InsufficientClearance));
            let clear=blocked.with_recent_obstacle_points(2,2,0,&[],[]).unwrap();
            clear.validate_planar_recent_motion(from,to).unwrap();
            assert!(blocked.validate_planar_recent_motion(from,to).is_err());
        }
        for point in [[0.5,0.0,-0.05],[0.5,0.0,0.95],[0.5,0.35,0.35]] {
            source.with_recent_obstacle_points(1,1,0,&[],[(0,point)]).unwrap()
                .validate_planar_recent_motion(from,to).unwrap();
        }
    }
    #[test]
    fn planar_recent_surface_points_keep_coordinates_and_height_dependent_sweep() {
        let source=source();let from=[0.0,0.0,0.1];let to=[1.0,0.0,0.6];
        for point in [[0.5,0.0,0.05],[0.5,0.0,0.2]] {
            source.with_recent_obstacle_points(1,1,0,&[0],[(0,point)]).unwrap()
                .validate_planar_recent_motion(from,to).unwrap();
        }
        let blocked=source.with_recent_obstacle_points(1,1,0,&[0],[(0,[0.5,0.0,0.9])]).unwrap();
        assert_eq!(blocked.validate_planar_recent_motion(from,to),Err(SurfaceError::InsufficientClearance));
    }
}
