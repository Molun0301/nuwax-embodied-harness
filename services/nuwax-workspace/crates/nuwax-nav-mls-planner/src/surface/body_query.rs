//! 未来足迹只需要运动许可；共享未变列，并在查询时核对原真实邻接规则。
use super::*;

/// 仅供实际机体地形推进及完整运动核验的私有足迹视图。
/// 不提供路线、前沿或快照提取接口，不能作为全局规划地图使用。
#[derive(Clone,Debug)]
pub struct SurfaceBodyQueryView {
    snapshot: SurfaceSnapshot,
}
impl SurfaceBodyQueryView {
    /// 与完整快照的几何、观察和当前足迹证明全部同一时，可复用同段许可。
    pub fn same_body_query_view(&self,other:&SurfaceSnapshot)->bool {
        self.snapshot.same_body_query_view(other)
    }
    /// 沿同一真实地形响应推进，仍保留实际起点的高度与姿态偏差。
    pub fn body_pose_after_motion(&self,from:([f64;3],[f64;4]),xy:[f64;2],yaw:f64)
        ->Result<([f64;3],[f64;4]),SurfaceError> {
        self.snapshot.body_pose_after_motion(from,xy,yaw)
    }
    /// 完整检查机体、支撑、实际占据接触事件及外部边界。
    pub fn validate_body_motion(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),boundary:&[[f64;2]])
        ->Result<(),SurfaceError> {
        self.snapshot.validate_body_motion(from,to,boundary)
    }
}
impl SurfaceSnapshot {
    /// 派生未来真实站姿的机体查询视图；来源与当前足迹的权限仍逐姿态独立。
    /// 硬墙距参与许可时沿原完整构建，其余情况只延后不影响许可的路由计算。
    pub fn body_query_with_current_support(&self,body:[f64;3],quaternion:[f64;4])
        ->Result<SurfaceBodyQueryView,SurfaceError> {
        self.current_body_support_view(body,quaternion,true).map(|snapshot|SurfaceBodyQueryView{snapshot})
    }

    /// 只有几何、实际观察和当前足迹证明三者同一，才能复用机体查询许可。
    pub fn same_body_query_view(&self,other:&Self)->bool {
        fn same<T>(a:&Option<Arc<T>>,b:&Option<Arc<T>>)->bool {
            match(a,b){(None,None)=>true,(Some(a),Some(b))=>Arc::ptr_eq(a,b),_=>false}
        }
        self.ground_evidence==other.ground_evidence && Arc::ptr_eq(&self.data,&other.data)&&same(&self.observations,&other.observations)
            &&same(&self.current_body_support,&other.current_body_support)
    }

    pub(super) fn can_defer_body_query_edges(&self)->bool {
        let config=self.config();
        // 软项只影响排序，但溢出成无穷也会拒绝边；仅在其保守上界有限时延后。
        let rise=(self.data.candidate_step_cells as f32+2.0)*config.voxel_size;
        let upper_cost=config.voxel_size.hypot(rise)*(2.0+2.0*config.wall_buffer_weight)
            +config.step_penalty_weight*rise;
        config.wall_clearance_m==0.0 && upper_cost.is_finite()
    }

    pub(super) fn with_body_query_columns(&self,updates:&[SurfaceColumnUpdate])->Result<Self,SurfaceError> {
        debug_assert!(self.can_defer_body_query_edges());
        let mut columns=(*self.data.occupied_columns).clone();
        let mut heights=(*self.data.height_bounds).clone();
        // 机体查询只读取单元身份及真实支撑层；不复制全图路由和墙距数组。
        let mut graph=PlannerGraph {cells:self.data.physical_graph.cells.clone(),
            surface_lookup:self.data.physical_graph.surface_lookup.clone(),..Default::default()};
        let mut count=self.data.occupied_voxels;let mut seen=AHashSet::new();
        for update in updates {
            let (x,y)=(update.column[0],update.column[1]);
            // 与完整列更新保持同一输入约束，极端坐标/高度也不能借查询视图绕过拒绝。
            if !seen.insert((x,y)) || x.abs_diff(0)>i32::MAX as u32/2 || y.abs_diff(0)>i32::MAX as u32/2
                || update.cells.iter().any(|&(z,b)|z.abs_diff(0)>i32::MAX as u32/2
                    ||!b.iter().all(|v|v.is_finite())||b[0]>=b[1])
                || update.cells.windows(2).any(|p|p[0].0>=p[1].0) {
                return Err(SurfaceError::InvalidPointCloud);
            }
            if let Some(old)=columns.remove(&(x,y)) {
                count-=old.len();for z in old {heights.remove(&(x,y,z));}
            }
            let zs:Vec<_>=update.cells.iter().map(|&(z,_)|z).collect();count+=zs.len();
            if !zs.is_empty() {columns.insert((x,y),zs.clone());}
            if self.config().voxel_size.to_bits()!=self.source_voxel_size_m().to_bits() {
                for &(z,bounds) in &update.cells {heights.insert((x,y,z),bounds);}
            }
            let mut surface=Vec::new();update::column_supports(x,y,&zs,self.config(),&heights,&mut surface);
            let next:Vec<_>=surface.iter().map(|p|p.2).collect();
            if let Some(old)=graph.surface_lookup.remove(&(x,y)) {
                for z in old {if next.binary_search(&z).is_err(){graph.cells.remove((x,y,z));}}
            }
            for &cell in &surface {graph.cells.insert(cell);}
            if !next.is_empty(){graph.surface_lookup.insert((x,y),next);}
        }
        if columns.is_empty()||count>MAX_SOURCE_POINTS {return Err(SurfaceError::InvalidPointCloud);}
        let graph=Arc::new(graph);
        Ok(Self {ground_evidence:self.ground_evidence,observations:self.observations.clone(),current_body_support:None,body_query_cache:Default::default(),
            data:Arc::new(SurfaceData {body_query_only:true,body_envelope:self.data.body_envelope.clone(),
                height_bounds:Arc::new(heights),frontier_cache:Default::default(),body_cache:Default::default(),
                routing_cache:Default::default(),physical_graph:graph.clone(),graph,
                regions:self.data.regions.clone(),occupied_columns:Arc::new(columns),occupied_voxels:count,
                virtual_obstacles:self.data.virtual_obstacles.clone(),virtual_blocked_cells:Arc::new(AHashSet::new()),
                config:self.config().clone(),source_voxel_size_m:self.source_voxel_size_m(),
                profile_cells:self.data.profile_cells,candidate_step_cells:self.data.candidate_step_cells})})
    }

    pub(super) fn body_query_cell_is_clear(&self,cell:VoxelKey)->bool {
        self.data.graph.cells.id(cell).is_some()&&self.data.virtual_obstacles.as_ref().is_none_or(|mask|
            mask.permits_cell(cell,f64::from(self.config().voxel_size),0.0))
    }
    pub(super) fn body_query_transition(&self,from:VoxelKey,to:VoxelKey)->Option<SurfaceTransition> {
        if !self.body_query_cell_is_clear(from)||!self.body_query_cell_is_clear(to)
            ||(from.2-to.2).abs()>self.data.candidate_step_cells {return None;}
        classify_transition(&self.data.graph.surface_lookup,from,to,self.config(),&self.data.height_bounds,
            self.source_voxel_size_m(),self.data.profile_cells,self.data.candidate_step_cells)
    }
    pub(super) fn body_query_edge_is_allowed(&self,from:VoxelKey,to:VoxelKey)->bool {
        let transition=self.body_query_transition(from,to);
        let Some(model)=self.body_envelope() else {return transition.is_some();};
        let rise=self.cell_position(to)[2]-self.cell_position(from)[2];
        match transition {
            Some(SurfaceTransition::StepUp)=>rise+2.0*model.support_uncertainty_m<=model.max_step_up_m+1e-6,
            Some(SurfaceTransition::StepDown)=>-rise+2.0*model.support_uncertainty_m<=model.max_step_down_m+1e-6,
            Some(_)=>true,None=>false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn scene(clearance:f32,hole:bool)->SurfaceSnapshot {
        let mut points:Vec<_>=(-12..12).flat_map(|x|(-8..8).filter_map(move |y| {
            (!(hole&&x==0&&y==0)).then_some(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,
                if x>=5 {0.225}else{0.025}))
        })).collect();
        points.push((0.45,0.05,0.525));
        let config=Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.2,surface_closing_radius:0.0,
            node_spacing_m:0.3,wall_clearance_m:clearance,wall_buffer_m:0.2,wall_buffer_weight:2.0,
            max_slope_deg:18.0,step_threshold_m:0.21,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,
            goal_tolerance:0.1,viz_publish_hz:0.0};
        let model=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.2,-0.12],[0.3,-0.12],[0.3,0.12],[-0.2,0.12]],
            z_bounds_m:[-0.1,0.15],body_to_support_z_m:0.4,max_step_up_m:0.21,max_step_down_m:0.1,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        SurfaceSnapshot::from_points(&points,config,0.1).unwrap().with_body_envelope(model).unwrap()
    }

    #[test]
    fn deferred_body_view_matches_full_support_edges_motion_and_virtual_boundary() {
        let from=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        for clearance in [0.0,0.01] {for masked in [false,true] {
            let mut source=scene(clearance,true);
            if masked {
                let mut blocked=vec![false;40*40];
                for y in 0..40 {blocked[y*40+25]=true;}
                source=source.with_virtual_obstacles([-2.0,-2.0],0.1,40,40,&blocked).unwrap();
            }
            assert!(source.validate_body_pose(from.0,from.1).is_err());
            let full=source.with_current_body_support(from.0,from.1).unwrap();
            let quick=source.body_query_with_current_support(from.0,from.1).unwrap();
            assert_eq!(quick.snapshot.data.body_query_only,clearance==0.0);
            assert_eq!(full.data.occupied_columns,quick.snapshot.data.occupied_columns);
            assert_eq!(full.data.height_bounds,quick.snapshot.data.height_bounds);
            assert_eq!(full.data.physical_graph.surface_lookup,quick.snapshot.data.physical_graph.surface_lookup);
            for cell in full.cell_keys() {
                assert_eq!(full.cell_is_clear(cell),quick.snapshot.cell_is_clear(cell));
                for &(dx,dy) in &DIRECTIONS {
                    for &z in full.data.graph.surface_lookup.get(&(cell.0+dx,cell.1+dy)).into_iter().flatten() {
                        let next=(cell.0+dx,cell.1+dy,z);
                        assert_eq!(full.has_edge(cell,next),quick.snapshot.has_edge(cell,next),"{cell:?}→{next:?}");
                        assert_eq!(full.has_geometric_edge(cell,next),quick.snapshot.has_geometric_edge(cell,next));
                    }
                }
            }
            for x in [-0.4,-0.05,0.05,0.1,0.3,0.6] {for yaw in [-0.2_f64,0.0,0.2] {
                let expected=full.body_pose_after_motion(from,[x,0.05],yaw);
                assert_eq!(expected,quick.body_pose_after_motion(from,[x,0.05],yaw));
                let to=([x,0.05,0.5],[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()]);
                assert_eq!(full.validate_body_motion(from,to,&[]),quick.validate_body_motion(from,to,&[]));
                let boundary=[[-0.2,-0.2],[0.2,-0.2],[0.2,0.2],[-0.2,0.2]];
                assert_eq!(full.validate_body_motion(from,to,&boundary),quick.validate_body_motion(from,to,&boundary));
            }}
            assert!(source.validate_body_pose(from.0,from.1).is_err(),"未来视图不能写回源图盲区");
        }}
    }

    #[test]
    fn body_query_identity_keeps_observation_and_actual_footprint_proofs_separate() {
        let from=([0.05,0.05,0.5],[0.0,0.0,0.0,1.0]);
        let clear=scene(0.0,false);let copy=clear.clone();
        assert!(clear.same_body_query_view(&copy));
        assert!(clear.body_query_with_current_support(from.0,from.1).unwrap().same_body_query_view(&clear));
        let observations=Arc::new(SurfaceObservations::new(0.1,32).unwrap());
        let observed=clear.with_observations(Some(observations.clone()));
        assert!(!observed.same_body_query_view(&clear));
        assert!(observed.same_body_query_view(&clear.with_observations(Some(observations.clone()))));
        let mut refreshed=observations.as_ref().clone();refreshed.set_source_timestamp_ns(10);
        assert!(!observed.same_body_query_view(&clear.with_observations(Some(Arc::new(refreshed)))));
        let blind=scene(0.0,true);
        let a=blind.body_query_with_current_support(from.0,from.1).unwrap();
        let b=blind.body_query_with_current_support(from.0,from.1).unwrap();
        assert!(!a.same_body_query_view(&b.snapshot),"不同派生足迹不能因几何相似复用最终许可");
        assert!(!a.same_body_query_view(&blind));
        assert!(a.same_body_query_view(&a.snapshot.clone()));
        let moved=([from.0[0]-0.1,from.0[1],from.0[2]],from.1);
        let next=a.snapshot.body_query_with_current_support(moved.0,moved.1).unwrap();
        let direct=blind.body_query_with_current_support(moved.0,moved.1).unwrap();
        assert_eq!(next.validate_body_motion(moved,from,&[]),direct.validate_body_motion(moved,from,&[]),
            "后续足迹派生始终回到同一原始图，不累积旧盲区授权");
    }
}
