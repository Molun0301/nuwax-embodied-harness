//! 实际标定接触平面在完整原始细观察中的支撑证明；不扩大定位量化或跨层。
use super::*;

impl SurfaceSnapshot {
    /// 实际接触平面须被完整、连通的真实细带证据包围，不能只比较高度极值。
    pub(super) fn observed_contact_plane_covers_reference(&self,body:[f64;3],q:[f64;4],reference:[f64;3])->bool {
        let Some(observations)=self.observations.as_ref() else{return false;};
        if !observations.is_complete_geometry_source() || observations.size_m.to_bits()!=self.source_voxel_size_m().to_bits(){return false;}
        let Some(model)=self.body_envelope() else{return false;};let Ok(r)=rotation(q) else{return false;};if r[2][2]<=0.0{return false;}
        let fine=f64::from(observations.size_m);let coarse=f64::from(self.config().voxel_size);
        let footprint:Vec<_>=model.footprint_xy_m.iter().map(|p| {let p=world(r,body,[p[0],p[1],-model.body_to_support_z_m]);[p[0],p[1]]}).collect();
        let low:[f64;2]=std::array::from_fn(|i|(footprint.iter().map(|p|p[i]).fold(f64::INFINITY,f64::min)/fine).floor());
        let high:[f64;2]=std::array::from_fn(|i|(footprint.iter().map(|p|p[i]).fold(f64::NEG_INFINITY,f64::max)/fine).floor());
        if low.iter().chain(high.iter()).any(|v|!v.is_finite()||*v<f64::from(i32::MIN)||*v>=f64::from(i32::MAX)){return false;}
        let low=low.map(|v|v as i32);let high=high.map(|v|v as i32);
        let width=i64::from(high[0])-i64::from(low[0])+1;let height_cells=i64::from(high[1])-i64::from(low[1])+1;
        if width.checked_mul(height_cells).is_none_or(|count|count>16_384){return false;}
        let height=|p:[f64;2]|body[2]-(r[2][0]*(p[0]-body[0])+r[2][1]*(p[1]-body[1])+model.body_to_support_z_m)/r[2][2];
        let mut selected=AHashMap::new();let mut evidence=vec![];
        for x in low[0]..=high[0] {for y in low[1]..=high[1] {
            let mut overlap=footprint.clone();
            for (axis,bound,sign) in [(0,x as f64*fine,1.),(0,(x+1)as f64*fine,-1.),(1,y as f64*fine,1.),(1,(y+1)as f64*fine,-1.)]{overlap=clip_contact_evidence(&overlap,|p|(p[axis]-bound)*sign);}
            if overlap.is_empty(){continue;}
            let key=(((x as f64+0.5)*fine/coarse).floor() as i32,((y as f64+0.5)*fine/coarse).floor() as i32);
            if !self.complete_observations_cover_column(key.0,key.1){return false;}
            let bounds=[overlap.iter().map(|&p|height(p)).fold(f64::INFINITY,f64::min),overlap.iter().map(|&p|height(p)).fold(f64::NEG_INFINITY,f64::max)];
            let middle=(bounds[0]+bounds[1])*0.5;let distance=|b:[f64;2]|(b[0]-middle).max(middle-b[1]).max(0.0);
            let bands=self.fine_contact_bands(x,y);let Some(b)=bands.iter().map(|entry|&entry.fit).min_by(|a,b|distance(**a).total_cmp(&distance(**b)).then_with(||(a[1]-middle).abs().total_cmp(&(b[1]-middle).abs())).then_with(||a[1].total_cmp(&b[1]))).copied() else{return false;};
            if b[1]-bounds[0]>model.max_step_up_m+fine+model.support_uncertainty_m+1e-6
                || bounds[1]-b[1]>model.max_step_down_m+fine+model.support_uncertainty_m+1e-6{return false;}
            if self.current_body_support.as_ref().is_some_and(|current| {
                let bottom=(b[0]/fine-1.0).round() as i32;let top=(b[1]/fine-1.0).round() as i32;
                (bottom..=top).any(|z|current.cleared_voxels.contains(&[x,y,z]))
            }){return false;}
            selected.insert((x,y),b);
            let near=clip_contact_evidence(&overlap,|p|height(p)-(b[0]-fine));evidence.extend(clip_contact_evidence(&near,|p|b[1]+fine-height(p)));
        }}
        let hull=hull(evidence);if hull.len()<3 || !inside_boundary([reference[0],reference[1]],&hull){return false;}
        let seed=((reference[0]/fine).floor() as i32,(reference[1]/fine).floor() as i32);if !selected.contains_key(&seed){return false;}
        let mut seen=AHashSet::from_iter([seed]);let mut queue=VecDeque::from([seed]);
        while let Some(key)=queue.pop_front(){for(dx,dy)in DIRECTIONS {
            let next=(key.0+dx,key.1+dy);let Some(b)=selected.get(&next) else{continue;};let rise=b[1]-selected[&key][1];
            if rise<=model.max_step_up_m+fine+1e-6 && -rise<=model.max_step_down_m+fine+1e-6 && seen.insert(next){queue.push_back(next);}
        }}
        seen.len()==selected.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn scene(kind:u8,complete:bool)->SurfaceSnapshot {
        let mut points=vec![];
        for x in -20_i32..20 {for y in -14_i32..14 {
            if kind==2&&x==0&&y==0 || kind==3&&x==0 {continue;}
            let top=match kind {
                0=>if (-1..=1).contains(&x)&&(0..=1).contains(&y){0.05}else{0.1},
                1=>0.05,
                2|3=>if (-1..=1).contains(&x)&&(0..=1).contains(&y){0.05}else{0.1},
                4=>if x<0{0.05}else{0.65},
                5=>0.05,
                _=>if (-1..=1).contains(&x)&&(0..=1).contains(&y){0.05}else{0.4},
            };
            points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,top-0.025));
            if kind==5{points.push(((x as f32+0.5)*0.05,(y as f32+0.5)*0.05,0.625));}
        }}
        let config=Config{world_frame:"map".into(),voxel_size:0.1,robot_height:0.1,surface_closing_radius:0.0,
            node_spacing_m:0.3,wall_clearance_m:0.0,wall_buffer_m:0.4,wall_buffer_weight:2.0,
            step_threshold_m:0.15,max_slope_deg:18.0,min_step_tread_depth_m:0.2,step_penalty_weight:2.0,
            goal_tolerance:0.2,viz_publish_hz:0.0};
        let model=SurfaceBodyEnvelope{footprint_xy_m:vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
            z_bounds_m:[-0.448,0.057],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
        for &(x,y,z) in &points{let key=[x,y,z].map(|v|(f64::from(v)/f64::from(0.05_f32)).floor() as i32);blocks.entry(key.map(|v|v.div_euclid(32))).or_default().insert(key,SurfaceObservation{observed_ns:0,hits:0});}
        let mut observations=SurfaceObservations::new(0.05,32).unwrap();for(k,v)in blocks{observations.replace_block(k,v).unwrap();}if complete{observations.mark_complete_geometry_source();}
        SurfaceSnapshot::from_points(&points,config,0.05).unwrap().with_body_envelope(model).unwrap().with_observations(Some(Arc::new(observations)))
    }
    #[test]
    fn observed_contact_plane_preserves_real_support_without_granting_floating_holes_or_layers() {
        let map=scene(0,true);let q=[0.0,0.0,0.0,1.0];let from=([-0.00001,0.025,0.51],q);let to=([0.00001,0.025,0.51],q);
        let reference=map.support_position_from_body(to.0,q,0.4).unwrap();
        assert!(map.locate_support(reference).is_err(),"中心粗列较低，不代表整个实际接触平面缺失");
        assert!(map.observed_contact_plane_covers_reference(to.0,q,reference));
        map.validate_body_pose(from.0,q).unwrap();map.validate_body_pose(to.0,q).unwrap();map.validate_body_motion(from,to,&[]).unwrap();
        let next=map.body_pose_after_motion(from,[to.0[0],to.0[1]],0.0).unwrap();map.validate_body_motion(from,next,&[]).unwrap();
        for(kind,height,label)in [(1,0.51,"平地悬空"),(2,0.51,"实际细列缺测"),(3,0.51,"实际断层"),(4,0.75,"跨层断开"),(5,0.75,"不同楼层"),(6,0.51,"超出原机体高度能力")] {
            let map=scene(kind,true);let p=[to.0[0],to.0[1],height];let reference=map.support_position_from_body(p,q,0.4).unwrap();
            assert!(!map.observed_contact_plane_covers_reference(p,q,reference),"{label}不能靠其它位置高度的min/max通过");
            assert!(map.validate_body_pose(p,q).is_err(),"{label}最终机体许可也必须拒绝");
        }
        let partial=scene(0,false);
        assert!(!partial.observed_contact_plane_covers_reference(to.0,q,reference),"未声明完整来源不能补出连续接触证明");
        assert!(partial.validate_body_pose(to.0,q).is_err());
        let mut wrong=map.observations.as_ref().unwrap().as_ref().clone();wrong.size_m=0.1;
        assert!(!map.with_observations(Some(Arc::new(wrong))).observed_contact_plane_covers_reference(to.0,q,reference),"源精度不一致拒绝");
        let mut mask=vec![false;40*40];mask[20*40+20]=true;
        assert!(map.with_virtual_obstacles([-1.,-1.],0.05,40,40,&mask).unwrap().validate_body_pose(to.0,q).is_err());
    }
}

fn clip_contact_evidence(polygon:&[[f64;2]],distance:impl Fn([f64;2])->f64)->Vec<[f64;2]> {
    let mut out=vec![];
    for (&a,&b) in polygon.iter().zip(polygon.iter().cycle().skip(1)) {
        let da=distance(a);let db=distance(b);if da>=0.0{out.push(a);}
        if (da>=0.0)!=(db>=0.0){let t=da/(da-db);out.push([a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])]);}
    }
    out
}
