//! 连通表面上的全域有限软势场。距离只用于居中排序，不是机体净空或通行许可。
use super::*;

fn neighbours(graph:&PlannerGraph,blocked:&AHashSet<CellId>,cell:VoxelKey,height:i64,out:&mut Vec<CellId>)->[bool;4] {
    let mut present=[false;4];
    for (index,(dx,dy)) in DIRECTIONS.into_iter().enumerate() {
        let (Some(x),Some(y))=(cell.0.checked_add(dx),cell.1.checked_add(dy)) else {continue;};
        for &z in graph.surface_lookup.get(&(x,y)).into_iter().flatten() {
            if (i64::from(z)-i64::from(cell.2)).abs()>height {continue;}
            let Some(id)=graph.cells.id((x,y,z)) else {continue;};
            if blocked.contains(&id) {continue;}
            present[index]=true;out.push(id);
        }
    }
    present
}

fn build(map:&SurfaceSnapshot)->Vec<f64> {
    let graph=&map.data.graph;let config=map.config();let size=config.voxel_size;
    // 与现墙距相同的地形邻列语义；运动剖面或单向台阶限制不被误当成实体墙。
    let ramp=config.max_slope_deg.to_radians().tan()*size+size;
    let height=(config.step_threshold_m.max(ramp)/size).ceil() as i64;
    let blocked=map.data.virtual_blocked_cells.as_ref();
    let mut adjacent=Vec::with_capacity(8);let mut boundary=Vec::new();
    for id in graph.cells.ids().filter(|id|!blocked.contains(id)) {
        adjacent.clear();
        if neighbours(graph,blocked,graph.cells.coord(id),height,&mut adjacent).contains(&false) {boundary.push(id);}
    }
    let root=graph.cells.slot_capacity();
    let mut distances=nuwax_explore_routing::shortest_costs(root+1,root,None,|id,out| {
        if id==root {
            out.extend(boundary.iter().map(|&id|(id as usize,f64::from(size)*0.5)));return;
        }
        adjacent.clear();
        neighbours(graph,blocked,graph.cells.coord(id as CellId),height,&mut adjacent);
        out.extend(adjacent.iter().map(|&id|(id as usize,f64::from(size))));
    });
    distances.truncate(root);distances
}

pub(super) fn distances(map:&SurfaceSnapshot)->Arc<Vec<f64>> {
    let mut cache=map.data.routing_cache.lock().unwrap_or_else(|e|e.into_inner());
    if let Some(distances)=&cache.centering {return Arc::clone(distances);}
    let distances=Arc::new(build(map));cache.centering=Some(Arc::clone(&distances));
    #[cfg(test)] {cache.centering_builds+=1;}
    distances
}

pub(super) fn tail_multiplier(map:&SurfaceSnapshot,cell:VoxelKey)->f64 {
    let source=map;let config=source.config();
    if config.wall_buffer_m<=0.0 || config.wall_buffer_weight<=0.0 {return 1.0;}
    // 私有新增支撑没有基础场单元时取有限中性值，仍受当前视图的近墙项与完整许可约束。
    let Some(id)=source.data.graph.cells.id(cell) else {return 1.0;};
    let distance=distances(source).get(id as usize).copied().unwrap_or(f64::INFINITY);
    if !distance.is_finite() {return 1.0;}
    let clearance=(distance-f64::from(config.wall_clearance_m)).max(0.0);
    let scale=f64::from(config.wall_buffer_m);
    1.0+f64::from(config.wall_buffer_weight)*(scale/(scale+clearance)).powi(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn config()->Config {
        Config {world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,surface_closing_radius:0.0,
            node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.4,wall_buffer_weight:3.0,max_slope_deg:18.0,
            step_threshold_m:0.15,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,goal_tolerance:0.1,viz_publish_hz:0.0}
    }
    fn corridor(half:i32,upper:bool)->SurfaceSnapshot {
        let mut points:Vec<_>=(-55..55).flat_map(|x|(-half..half).map(move|y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
        if upper {points.extend((-55..55).flat_map(|x|(-2..2).map(move|y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,1.05))));}
        SurfaceSnapshot::from_points(&points,config(),0.1).unwrap()
    }
    fn key(map:&SurfaceSnapshot,p:[f64;3])->VoxelKey {map.locate_support(p).unwrap().cell}

    #[test]
    fn broad_soft_field_removes_flat_cost_band_and_guides_toward_the_corridor_middle() {
        let map=corridor(10,false);
        let locations=[-0.45,-0.25,-0.05].map(|y|key(&map,[0.05,y,0.1]));
        for cell in locations {
            let id=map.data.graph.cells.id(cell).unwrap();
            assert_eq!(map.data.graph.wall_state.dist[id as usize],map.config().wall_buffer_m,
                "保留原有界硬墙距，不能用全域软场覆盖它");
        }
        let costs=locations.map(|cell|map.soft_traversal_multiplier(cell));
        assert!(costs[0]>costs[1] && costs[1]>costs[2] && costs[2]>1.0);
        let before:Vec<_>=map.data.graph.cells.ids().map(|id|(id,map.data.graph.cells.neighbors(id).iter()
            .map(|e|(e.dest,e.cost.to_bits())).collect::<Vec<_>>())).collect();
        let start=[-4.0,-0.5,0.1];let goal=[4.0,-0.5,0.1];
        let path=map.plan(start,goal).unwrap();
        let middle=path.windows(2).find_map(|pair| {
            (pair[0][0]<=0.0 && pair[1][0]>=0.0 && pair[0][0]!=pair[1][0]).then(|| {
                let t=-pair[0][0]/(pair[1][0]-pair[0][0]);pair[0][1]*(1.0-t)+pair[1][1]*t
            })
        }).unwrap();
        assert!(middle.abs()<0.2,"远离旧缓冲带后仍应比较居中收益: {path:?}");
        let score=path.windows(2).map(|p|map.soft_segment_cost(p[0],p[1]).unwrap()).sum::<f64>();
        assert!(score<map.soft_segment_cost(start,goal).unwrap());
        map.validate_path(&path).unwrap();
        let after:Vec<_>=map.data.graph.cells.ids().map(|id|(id,map.data.graph.cells.neighbors(id).iter()
            .map(|e|(e.dest,e.cost.to_bits())).collect::<Vec<_>>())).collect();
        assert_eq!(before,after,"偏好查询不做全图边重加权");
    }

    #[test]
    fn soft_boundary_field_keeps_vertical_layers_and_narrow_reachability_separate() {
        let plain=corridor(10,false);let stacked=corridor(10,true);
        let lower=key(&stacked,[0.05,0.05,0.1]);let upper=key(&stacked,[0.05,0.05,1.1]);
        assert_eq!(stacked.soft_traversal_multiplier(lower),plain.soft_traversal_multiplier(key(&plain,[0.05,0.05,0.1])));
        assert!(stacked.soft_traversal_multiplier(upper)>stacked.soft_traversal_multiplier(lower),
            "上层窄边界只能影响其自身连通表面，不能投影成下层墙");
        let narrow=corridor(2,false);let mut cfg=config();cfg.wall_buffer_weight=0.0;
        let points:Vec<_>=(-55..55).flat_map(|x|(-2..2).map(move|y|
            ((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,0.05))).collect();
        let disabled=SurfaceSnapshot::from_points(&points,cfg,0.1).unwrap();
        for cell in narrow.cell_keys() {
            let value=narrow.soft_traversal_multiplier(cell);
            assert!(value.is_finite() && value>=1.0 && value<=1.0+f64::from(config().wall_buffer_weight));
            assert_eq!(narrow.neighbors(cell).into_iter().collect::<AHashSet<_>>(),disabled.neighbors(cell).into_iter().collect::<AHashSet<_>>());
        }
        narrow.plan([-4.0,0.0,0.1],[4.0,0.0,0.1]).unwrap();
        assert!(disabled.data.routing_cache.lock().unwrap().centering.is_none());
    }

    #[test]
    fn ordinary_observation_refresh_and_private_footprints_reuse_one_base_field() {
        let points:Vec<_>=(-10..10).flat_map(|x|(-10..10).filter_map(move|y|
            (!(x==0 && y==0)).then_some(((x as f32+0.5)*0.1,(y as f32+0.5)*0.1,-0.025)))).collect();
        let model=SurfaceBodyEnvelope {footprint_xy_m:vec![[-0.15,-0.15],[0.15,-0.15],[0.15,0.15],[-0.15,0.15]],
            z_bounds_m:[-0.1,0.1],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let map=SurfaceSnapshot::from_points(&points,config(),0.05).unwrap().with_body_envelope(model).unwrap();
        let cell=key(&map,[-0.45,-0.45,0.0]);let first=map.soft_traversal_multiplier(cell);let field=distances(&map);
        for time in 0..10 {
            let mut observations=SurfaceObservations::new(0.05,32).unwrap();observations.set_source_timestamp_ns(time);
            let refreshed=map.with_observations(Some(Arc::new(observations)));
            assert_eq!(refreshed.soft_traversal_multiplier(cell),first);
            assert!(Arc::ptr_eq(&field,&distances(&refreshed)));
            let private=map.with_current_body_support([0.05+time as f64*0.001,0.05,0.4],[0.0,0.0,0.0,1.0]).unwrap();
            assert_eq!(private.soft_traversal_multiplier(cell),first);
            assert_eq!(private.locate_support([0.05,0.05,0.0]),Err(SurfaceError::MissingSupport),
                "query view cannot turn a missing column into observed ground");
            assert!(Arc::ptr_eq(&field,&distances(&private)));
            assert_eq!(private.data.routing_cache.lock().unwrap().centering_builds,1);
        }
        assert_eq!(map.data.routing_cache.lock().unwrap().centering_builds,1,
            "重复查询、普通观察时刻和私有足迹都不得每拍全图构建");
        let changed=map.with_virtual_obstacles([-2.0,-2.0],0.1,40,40,&vec![false;1600]).unwrap();
        let different=distances(&changed);
        assert!(!Arc::ptr_eq(&field,&different));
        assert_eq!(changed.data.routing_cache.lock().unwrap().centering_builds,1);
        assert_eq!(map.data.routing_cache.lock().unwrap().centering_builds,1);
    }
}
