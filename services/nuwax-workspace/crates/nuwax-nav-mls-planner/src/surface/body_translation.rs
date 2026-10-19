//! 实际机体平移及姿态插值的连续接触事件；只在真实占据相交区间补做原机体核验。
use super::*;

impl PrismQuery {
    /// 各分离轴投影的重叠时间取交集，得到整段平移中的实际接触区间。
    fn expanded_translated_interval(&self,displacement:[f64;3],min:[f64;3],max:[f64;3],padding:f64)->Option<[f64;2]> {
        let centre = std::array::from_fn(|i| (min[i] + max[i]) * 0.5 - self.origin[i]);
        let half = std::array::from_fn(|i| (max[i] - min[i]) * 0.5);
        let mut interval = [0.0_f64, 1.0_f64];
        for &(axis, low, high) in &self.axes {
            let position = dot(axis, centre);
            let radius = dot(axis.map(f64::abs), half);
            let velocity = -dot(axis, displacement);
            let allowance=padding*dot(axis,axis).sqrt();
            // 候选时间区间必须覆盖后续逐姿态相交判断。近乎平行时，点积的
            // 末位误差会被除以极小速度放大；向外包住运算误差，不改最终碰撞许可。
            let velocity_error=32.0*f64::EPSILON*(0..3).map(|i|(axis[i]*displacement[i]).abs()).sum::<f64>();
            let rounding=32.0*f64::EPSILON*((0..3).map(|i|axis[i].abs()
                *(min[i].abs()+max[i].abs()+2.0*self.origin[i].abs()+displacement[i].abs())).sum::<f64>()
                +low.abs()+high.abs()+radius+allowance);
            let lower = low - radius - allowance - 1e-10 - rounding;
            let upper = high + radius + allowance + 1e-10 + rounding;
            if velocity.abs() <= velocity_error {
                if position < lower || position > upper { return None; }
            } else {
                let first = (lower - position) / velocity;
                let last = (upper - position) / velocity;
                interval[0] = interval[0].max(first.min(last));
                interval[1] = interval[1].min(first.max(last));
                if interval[0] > interval[1] { return None; }
            }
        }
        Some(interval)
    }
}

// 归一化四元数线性插值下，旋转矩阵的分子是二次多项式。
// 顶点/面和边/边的接触方程乘以正的四元数模长平方后至多三次，
// 可直接枚举接触时刻，不按固定距离加密整段。
struct MotionFrame {
    position:[f64;3],
    rotation:[[f64;3];3],
    prism:Option<PrismQuery>,
}
struct RotationEvents {
    samples: [([[f64;3];3],f64,[f64;3]);4],
    vertices: Vec<([f64;3],[[f64;3];4])>,
    faces: Vec<([f64;3],f64,[[f64;3];4])>,
    edges: Vec<([f64;3],[f64;3],[[f64;3];4],[[f64;3];4])>,
    minimum_upward_z:f64,
    ground_normal:[f64;3],
    normal_delta_bound:f64,
    // 缓存只属于这一段实际运动，不跨位姿、机体模型或地图快照复用。
    root_equations:std::cell::RefCell<AHashMap<[u64;4],Vec<f64>>>,
    frames:std::cell::RefCell<AHashMap<u64,MotionFrame>>,
    vertex_bounds:Vec<([f64;3],[f64;3])>,
    face_bounds:Vec<([f64;3],[f64;3])>,
    edge_bounds:Vec<([f64;3],[f64;3])>,
    from: ([f64;3],[f64;4]),
    to: ([f64;3],[f64;4]),
}
// This query-local interpolation is also sufficient for support-grid events.
// Body feature bounds belong only to the full occupied-volume contact pass.
struct MotionInterpolation {
    samples: [([[f64;3];3],f64,[f64;3]);4],
    from: ([f64;3],[f64;4]),
    to: ([f64;3],[f64;4]),
    middle:[[f64;3];3],
}
impl MotionInterpolation {
    fn new(from:([f64;3],[f64;4]),to:([f64;3],[f64;4]))->Self {
        let norm=|q:[f64;4]| {let n=q.iter().map(|v|v*v).sum::<f64>().sqrt();q.map(|v|v/n)};
        let qa=norm(from.1);let mut qb=norm(to.1);
        if qa.iter().zip(qb).map(|(a,b)|a*b).sum::<f64>()<0.0 {qb=qb.map(|v|-v);}
        let samples:[([[f64;3];3],f64,[f64;3]);4]=std::array::from_fn(|i| {
            let t=i as f64/3.0;
            let q=std::array::from_fn(|j|qa[j]*(1.0-t)+qb[j]*t);
            let n=q.iter().map(|v|v*v).sum::<f64>();
            let r=rotation(q).expect("已验证四元数插值").map(|a|a.map(|v|v*n));
            (r,n,std::array::from_fn(|j|from.0[j]*(1.0-t)+to.0[j]*t))
        });
        let middle=rotation(std::array::from_fn(|i|qa[i]+qb[i])).expect("已验证最短插值");
        Self {samples,from:(from.0,qa),to:(to.0,qb),middle}
    }
}
impl RotationEvents {
    fn new(model:&SurfaceBodyEnvelope,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]))->Self {
        let interpolation=MotionInterpolation::new(from,to);
        let qa=interpolation.from.1;let qb=interpolation.to.1;
        let samples=interpolation.samples;
        // 以中间姿态覆盖整段旋转。单位四元数弦长等于2*sin(转角/4)，
        // 乘特征半径即可界定其相对中间姿态的最大位移，微小真实转角也保留。
        let middle=interpolation.middle;
        let angular_bound=qa.iter().zip(qb).map(|(a,b)|(a-b).powi(2)).sum::<f64>().sqrt();
        let displacement:[f64;3]=std::array::from_fn(|i|to.0[i]-from.0[i]);
        #[cfg(test)]
        interpolation_reuse_tests::record_event_geometry_build();
        let sweep_bounds=|vertices:&[[f64;3]]| {
            let padding=vertices.iter().map(|v|dot(*v,*v).sqrt()).fold(0.0,f64::max)*angular_bound;
            let mut min=[f64::INFINITY;3];let mut max=[f64::NEG_INFINITY;3];
            for &v in vertices {let point=world(middle,from.0,v);for i in 0..3 {
                min[i]=min[i].min(point[i]+displacement[i].min(0.0)-padding);
                max[i]=max[i].max(point[i]+displacement[i].max(0.0)+padding);
            }}
            (min,max)
        };
        let local_vertices:Vec<_>=model.footprint_xy_m.iter().flat_map(|p|model.z_bounds_m.map(|z|[p[0],p[1],z])).collect();
        let vertex_bounds=local_vertices.iter().map(|&v|sweep_bounds(&[v])).collect();
        let vertices=local_vertices.into_iter().map(|v|(v,samples.map(|(r,_,_)|world(r,[0.0;3],v)))).collect();
        let mut face_bounds:Vec<_>=model.z_bounds_m.into_iter().map(|z|sweep_bounds(
            &model.footprint_xy_m.iter().map(|p|[p[0],p[1],z]).collect::<Vec<_>>())).collect();
        let mut faces=vec![([0.0,0.0,1.0],model.z_bounds_m[0]),([0.0,0.0,1.0],model.z_bounds_m[1])];
        let mut edges=Vec::new();
        for (a,b) in model.footprint_xy_m.iter().zip(model.footprint_xy_m.iter().cycle().skip(1)) {
            let e=[b[0]-a[0],b[1]-a[1],0.0];let n=[e[1],-e[0],0.0];
            faces.push((n,n[0]*a[0]+n[1]*a[1]));
            face_bounds.push(sweep_bounds(&[a,b].into_iter().flat_map(|p|
                model.z_bounds_m.map(|z|[p[0],p[1],z])).collect::<Vec<_>>()));
            for z in model.z_bounds_m {edges.push(([a[0],a[1],z],e));}
            edges.push(([a[0],a[1],model.z_bounds_m[0]],[0.0,0.0,model.z_bounds_m[1]-model.z_bounds_m[0]]));
        }
        // 四帧特征变换只属于本次机体运动，与查询哪个占据体素无关。
        let faces=faces.into_iter().map(|(n,h)|(n,h,samples.map(|(r,_,_)|world(r,[0.0;3],n)))).collect();
        let edge_bounds=edges.iter().map(|&(v,e)|sweep_bounds(&[v,std::array::from_fn(|i|v[i]+e[i])])).collect();
        let edges=edges.into_iter().map(|(v,e)|(v,e,samples.map(|(r,_,_)|world(r,[0.0;3],e)),
            samples.map(|(r,_,_)|world(r,[0.0;3],cross(v,e))))).collect();
        Self {samples,vertices,faces,edges,minimum_upward_z:minimum_upward_z(qa,qb),
            ground_normal:rotation(qa).expect("已验证实际姿态")[2],normal_delta_bound:2.0*angular_bound,
            root_equations:Default::default(),frames:Default::default(),
            vertex_bounds,face_bounds,edge_bounds,from:(from.0,qa),to:(to.0,qb)}
    }
    /// 每个占据列的接触平面下界，覆盖全部实际平移和最短四元数插值。
    /// 单位四元数弦长的两倍界定法向变化；只跳过整列已证安全的真实地面。
    fn ground_allowance(&self,model:&SurfaceBodyEnvelope,x:i32,y:i32,size:f64,
        contact:f64,uncertainty:f64)->f64 {
        let upward=self.minimum_upward_z;
        if upward<=0.0 {return f64::NEG_INFINITY;}
        let normal=self.ground_normal;
        if normal[2]<=0.0 {return f64::NEG_INFINITY;}
        let lower=|height:f64| {
            let mut minimum=f64::INFINITY;let mut error=0.0_f64;
            for p in [self.from.0,self.to.0] {for ix in [x,x+1] {for iy in [y,y+1] {
                let dx=f64::from(ix)*size-p[0];let dy=f64::from(iy)*size-p[1];
                let numerator=height-normal[0]*dx-normal[1]*dy;
                minimum=minimum.min(p[2]+numerator/normal[2]);
                error=error.max(dx.hypot(dy)+numerator.abs()/normal[2]);
            }}}
            minimum-self.normal_delta_bound*error/upward-1e-10
        };
        (lower(model.z_bounds_m[0].max(-model.body_to_support_z_m))+contact+1e-6)
            .min(lower(0.0)+uncertainty)
    }
    fn frame(&self,t:f64)->MotionFrame {
        let q:[f64;4]=std::array::from_fn(|i|self.from.1[i]*(1.0-t)+self.to.1[i]*t);
        MotionFrame {position:std::array::from_fn(|i|if self.from.0[i]==self.to.0[i] {self.from.0[i]} else {
            self.from.0[i]*(1.0-t)+self.to.0[i]*t}),rotation:rotation(q).expect("已验证四元数插值"),prism:None}
    }
    fn transform(&self,t:f64)->([f64;3],[[f64;3];3]) {
        let mut frames=self.frames.borrow_mut();
        let frame=frames.entry(t.to_bits()).or_insert_with(||self.frame(t));
        (frame.position,frame.rotation)
    }
    fn intersects_at(&self,model:&SurfaceBodyEnvelope,t:f64,min:[f64;3],max:[f64;3])->bool {
        let mut frames=self.frames.borrow_mut();
        let frame=frames.entry(t.to_bits()).or_insert_with(||self.frame(t));
        if frame.prism.is_none() {
            frame.prism=Some(PrismQuery::new(model,frame.rotation,frame.position));
        }
        frame.prism.as_ref().expect("已构造实际机体凸柱").intersects(min,max)
    }
    fn contacts(&self,model:&SurfaceBodyEnvelope,min:[f64;3],max:[f64;3],interval:[f64;2],changes:&[f64])->Vec<[f64;2]> {
        let mut times=vec![interval[0],interval[1]];
        times.extend(changes.iter().copied().filter(|&t|t>interval[0]&&t<interval[1]));
        // 无限平面或直线的根只是候选；只保留落在真实有限面/边上的接触。
        let mut record=|values:[f64;4],accept:&dyn Fn(f64)->bool| {
            let mut equations=self.root_equations.borrow_mut();
            let roots=match equations.entry(values.map(f64::to_bits)) {
                std::collections::hash_map::Entry::Occupied(entry)=> {
                    entry.into_mut()
                },
                std::collections::hash_map::Entry::Vacant(entry)=> {
                    let mut roots=Vec::new();
                    cubic_event_roots(values,[0.0,1.0],&mut roots);
                    entry.insert(roots)
                },
            };
            times.extend(roots.iter().copied().filter(|&t|t>=interval[0] && t<=interval[1] && accept(t)));
        };
        let overlaps=|(lower,upper):([f64;3],[f64;3])|(0..3).all(|i|lower[i]<=max[i]+1e-9 && upper[i]>=min[i]-1e-9);
        for (index,&(vertex,frames)) in self.vertices.iter().enumerate() {
            if !overlaps(self.vertex_bounds[index]) {continue;}
            for axis in 0..3 {for bound in [min[axis],max[axis]] {
            if bound<self.vertex_bounds[index].0[axis]-1e-9 || bound>self.vertex_bounds[index].1[axis]+1e-9 {continue;}
            record(std::array::from_fn(|i|frames[i][axis]+(self.samples[i].2[axis]-bound)*self.samples[i].1),&|t| {
                let (p,r)=self.transform(t);let point=world(r,p,vertex);
                (0..3).all(|i|point[i]>=min[i]-1e-9 && point[i]<=max[i]+1e-9)
            });
        }}}
        let corners:Vec<[f64;3]>=(0..8).map(|bits|std::array::from_fn(|i|if bits&(1<<i)==0 {min[i]}else{max[i]})).collect();
        for (index,&(_,height,frames)) in self.faces.iter().enumerate() {
            if !overlaps(self.face_bounds[index]) {continue;}
            for &corner in &corners {
            if !(0..3).all(|i|corner[i]>=self.face_bounds[index].0[i]-1e-9 && corner[i]<=self.face_bounds[index].1[i]+1e-9) {continue;}
            record(std::array::from_fn(|j|dot(frames[j],std::array::from_fn(|i|corner[i]-self.samples[j].2[i]))-height*self.samples[j].1),&|t| {
                let (p,r)=self.transform(t);
                let relative=std::array::from_fn(|i|corner[i]-p[i]);let local=r.map(|axis|dot(axis,relative));
                local[2]>=model.z_bounds_m[0]-1e-9 && local[2]<=model.z_bounds_m[1]+1e-9
                    && self.faces.iter().all(|&(normal,height,_)|dot(normal,local)<=height+1e-9 || normal==[0.0,0.0,1.0])
            });
        }}
        for (index,&(vertex,edge,directions,crosses)) in self.edges.iter().enumerate() {
            if !overlaps(self.edge_bounds[index]) {continue;}
            for axis in 0..3 {
            let axis_vector=std::array::from_fn(|i|if i==axis {1.0}else{0.0});
            // 每个世界轴只有四条不同的体素边，沿该轴的角点坐标不改变方程。
            for &corner in corners.iter().filter(|c|c[axis]==min[axis]) {
                if !(0..3).all(|i| i==axis || (corner[i]>=self.edge_bounds[index].0[i]-1e-9 && corner[i]<=self.edge_bounds[index].1[i]+1e-9)) {continue;}
                record(std::array::from_fn(|j| {
                    dot(std::array::from_fn(|i|corner[i]-self.samples[j].2[i]),cross(directions[j],axis_vector))
                        -crosses[j][axis]
                }),&|t| {
                    let (p,r)=self.transform(t);
                    let start=world(r,p,vertex);let direction=world(r,[0.0;3],edge);
                    let other=axis_vector.map(|v|v*(max[axis]-min[axis]));
                    let normal=cross(direction,other);let denominator=dot(normal,normal);
                    if denominator<1e-24 {return true;}
                    let delta=std::array::from_fn(|i|corner[i]-start[i]);
                    let a=dot(cross(delta,other),normal)/denominator;
                    let b=dot(cross(delta,direction),normal)/denominator;
                    (-1e-9..=1.0+1e-9).contains(&a) && (-1e-9..=1.0+1e-9).contains(&b)
                });
            }
        }}
        times.sort_by(f64::total_cmp);times.dedup_by(|a,b|(*a-*b).abs()<1e-12);
        let intersects=|t|self.intersects_at(model,t,min,max);
        let mut result:Vec<[f64;2]>=Vec::new();
        for pair in times.windows(2) {
            if !intersects((pair[0]+pair[1])*0.5) {continue;}
            if let Some(last)=result.last_mut().filter(|last|last[1]==pair[0]) {last[1]=pair[1];}
            else {result.push([pair[0],pair[1]]);}
        }
        // 相切接触没有非零长度区间，也必须保留。
        for t in times {if !result.iter().any(|r|t>=r[0]&&t<=r[1]) && intersects(t) {result.push([t,t]);}}
        result
    }
}

// 归一化插值的竖直分量为1-2*(x²+y²)/|q|²，分式导数的分子至多二次。
// 在端点和全部驻点取最小值，平地偏航因此保持原倾斜界。
fn minimum_upward_z(a:[f64;4],b:[f64;4])->f64 {
    let d:[f64;4]=std::array::from_fn(|i|b[i]-a[i]);
    let numerator=[a[0]*a[0]+a[1]*a[1],2.0*(a[0]*d[0]+a[1]*d[1]),d[0]*d[0]+d[1]*d[1]];
    let denominator=[a.iter().map(|v|v*v).sum::<f64>(),2.0*a.iter().zip(d).map(|(a,d)|a*d).sum::<f64>(),
        d.iter().map(|v|v*v).sum::<f64>()];
    let quadratic=numerator[2]*denominator[1]-numerator[1]*denominator[2];
    let linear=2.0*(numerator[2]*denominator[0]-numerator[0]*denominator[2]);
    let constant=numerator[1]*denominator[0]-numerator[0]*denominator[1];
    let value=|t:f64| {
        let q:[f64;4]=std::array::from_fn(|i|a[i]*(1.0-t)+b[i]*t);
        1.0-2.0*(q[0]*q[0]+q[1]*q[1])/q.iter().map(|v|v*v).sum::<f64>()
    };
    let mut result=value(0.0).min(value(0.5)).min(value(1.0));
    let mut include=|t:f64| {if (0.0..=1.0).contains(&t) {result=result.min(value(t));}};
    if quadratic==0.0 {
        if linear!=0.0 {include(-constant/linear);}
    } else {
        let discriminant=linear*linear-4.0*quadratic*constant;
        if discriminant>=0.0 {
            let q=-0.5*(linear+discriminant.sqrt().copysign(linear));
            if q==0.0 {include(-linear/(2.0*quadratic));}
            else {include(q/quadratic);include(constant/q);}
        }
    }
    // 单位输入下分母至少为1/2；覆盖有限次乘加及归一化舍入，保持包围界保守。
    (result-64.0*f64::EPSILON).max(-1.0)
}

fn cubic_event_roots(values:[f64;4],interval:[f64;2],roots:&mut Vec<f64>) {
    let [f0,f1,f2,f3]=values;
    let a=4.5*(f3-3.0*f2+3.0*f1-f0);
    let b=4.5*(f2-2.0*f1+f0)-a;
    let c=3.0*(f1-f0)-a/9.0-b/3.0;
    let evaluate=|t:f64|((a*t+b)*t+c)*t+f0;
    let scale=values.into_iter().map(f64::abs).fold(1.0,f64::max);
    let epsilon=scale*1e-12;
    if a.abs()+b.abs()+c.abs()<epsilon {return;}
    let mut limits=vec![interval[0],interval[1]];
    if a.abs()<epsilon {
        if b.abs()>=epsilon {let t=-c/(2.0*b);if t>interval[0]&&t<interval[1] {limits.push(t);}}
    } else {
        let d=b*b-3.0*a*c;
        if d>=0.0 {for t in [(-b-d.sqrt())/(3.0*a),(-b+d.sqrt())/(3.0*a)] {
            if t>interval[0]&&t<interval[1] {limits.push(t);}
        }}
    }
    limits.sort_by(f64::total_cmp);
    for &t in &limits {if evaluate(t).abs()<=epsilon {roots.push(t);}}
    for pair in limits.windows(2) {
        let (mut left,mut right)=(pair[0],pair[1]);let sign=evaluate(left).is_sign_positive();
        if sign==evaluate(right).is_sign_positive() {continue;}
        while right-left>1e-12 {
            let mid=(left+right)*0.5;
            if evaluate(mid).is_sign_positive()==sign {left=mid;}else{right=mid;}
        }
        roots.push((left+right)*0.5);
    }
}

fn grid_interval_samples(mut times:Vec<f64>)->Vec<f64> {
    times.sort_by(f64::total_cmp);times.dedup_by(|a,b|(*a-*b).abs()<1e-12);
    let mut samples=Vec::with_capacity(times.len()*2);
    for pair in times.windows(2) {samples.push(pair[0]);samples.push((pair[0]+pair[1])*0.5);}
    samples.push(1.0);samples
}

// All crossed source cells, including boundary and interior samples. The same
// reference geometry is used with and without a calibrated body envelope.
pub(super) fn reference_grid_fractions(a:[f64;3],b:[f64;3],size:f64)->Result<Vec<f64>,SurfaceError> {
    if !a.into_iter().chain(b).all(|v|v.is_finite() && (v/size).abs()<f64::from(i32::MAX)*0.5) {
        return Err(SurfaceError::InvalidPose);
    }
    let mut times=vec![0.0,1.0];
    for axis in 0..3 {
        let delta=b[axis]-a[axis];if delta==0.0 {continue;}
        let first=(a[axis].min(b[axis])/size).floor() as i32+1;
        let last=(a[axis].max(b[axis])/size).floor() as i32;
        for index in first..=last {
            let t=(f64::from(index)*size-a[axis])/delta;
            if t>0.0 && t<1.0 {times.push(t);}
        }
    }
    Ok(grid_interval_samples(times))
}

impl SurfaceSnapshot {
    /// 按实际四元数插值求足下参考穿过精细网格的全部时刻。
    pub(super) fn body_support_grid_fractions(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),size:f64)
        -> Result<Vec<f64>,SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        if rotation(from.1)?==rotation(to.1)? {
            let a=self.support_position_from_body(from.0,from.1,model.body_to_support_z_m)?;
            let b=self.support_position_from_body(to.0,to.1,model.body_to_support_z_m)?;
            return reference_grid_fractions(a,b,size);
        }
        let events=MotionInterpolation::new(from,to);
        let point=[0.0,0.0,-model.body_to_support_z_m];
        let middle=rotation(std::array::from_fn(|i|events.from.1[i]+events.to.1[i]))?;
        let centre=world(middle,from.0,point);
        let padding=model.body_to_support_z_m*events.from.1.iter().zip(events.to.1)
            .map(|(a,b)|(a-b).powi(2)).sum::<f64>().sqrt();
        let mut times=vec![0.0,1.0];
        for axis in 0..3 {
            let displacement=to.0[axis]-from.0[axis];
            let first=((centre[axis]+displacement.min(0.0)-padding)/size).floor() as i32+1;
            let last=((centre[axis]+displacement.max(0.0)+padding)/size).floor() as i32;
            for index in first..=last {
                let boundary=f64::from(index)*size;
                cubic_event_roots(events.samples.map(|(r,n,p)|world(r,[0.0;3],point)[axis]+(p[axis]-boundary)*n),[0.0,1.0],&mut times);
            }
        }
        Ok(grid_interval_samples(times))
    }

    /// 保留原地面、精细观察和当前占据语义；不把接触候选直接当作碰撞。
    pub(super) fn validate_body_contact_events(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        mut check:impl FnMut(f64)->Result<(),SurfaceError>,
    ) -> Result<(), SurfaceError> {
        self.validate_body_contact_events_in_context(from,to,QuerySegment::LEGACY,|t,collision| {
            if let Some(collision)=collision {Err(collision.error())} else {check(t)}
        })
    }

    pub(super) fn validate_body_contact_events_in_context(
        &self,
        from: ([f64; 3], [f64; 4]),
        to: ([f64; 3], [f64; 4]),
        context:QuerySegment<'_>,
        check:impl FnMut(f64,Option<BodyRejection>)->Result<(),SurfaceError>,
    ) -> Result<(), SurfaceError> {
        let r = rotation(from.1)?;
        let rotating=r != rotation(to.1)?;
        if !rotating && from.0 == to.0 && context.constant() { return Ok(()); }
        let model = self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let mut rigid_motion=self.fixed_body_primitives()?.map(|primitives|
            contact::RigidMotionQuery::new(primitives,from,to)).transpose()?;
        let check=std::cell::RefCell::new(check);
        let events=rotating.then(||RotationEvents::new(model,from,to));
        let angle=events.as_ref().map_or(0.0,|events|2.0*events.from.1.iter().zip(events.to.1)
            .map(|(a,b)|a*b).sum::<f64>().abs().min(1.0).acos());
        let radius=model.radius().hypot(model.z_bounds_m.into_iter().map(f64::abs).fold(0.0,f64::max)).max(model.body_to_support_z_m);
        let padding=2.0*radius*(angle*0.5).sin();
        let displacement = std::array::from_fn(|i| to.0[i] - from.0[i]);
        let prism = PrismQuery::new(model, r, from.0);
        let vertices: Vec<_> = model.footprint_xy_m.iter().flat_map(|p| {
            model.z_bounds_m.map(|z| world(r, from.0, [p[0], p[1], z]))
        }).collect();
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for point in &vertices {
            for i in 0..3 {
                min[i] = min[i].min(point[i].min(point[i] + displacement[i]));
                max[i] = max[i].max(point[i].max(point[i] + displacement[i]));
            }
        }
        for i in 0..3 {min[i]-=padding;max[i]+=padding;}
        if let Some(motion)=&rigid_motion {
            let (low,high)=motion.bounds()?;
            for axis in 0..3 {min[axis]=min[axis].min(low[axis]);max[axis]=max[axis].max(high[axis]);}
        }
        let size = f64::from(self.config().voxel_size);
        let from_support = self.support_position_from_body(from.0, from.1, model.body_to_support_z_m)?;
        let to_support = self.support_position_from_body(to.0, to.1, model.body_to_support_z_m)?;
        // 接触期间支撑格变化可能改变地面/障碍归属，不能只查整段接触区间的中点。
        let mut changes = Vec::new();
        for axis in 0..3 {
            let delta = to_support[axis] - from_support[axis];
            if let Some(events)=&events {
                let first=((from_support[axis].min(to_support[axis])-padding)/size).floor() as i32+1;
                let last=((from_support[axis].max(to_support[axis])+padding)/size).floor() as i32;
                for index in first..=last {
                    let bound=f64::from(index)*size;
                    cubic_event_roots(events.samples.map(|(r,n,p)|
                        world(r,[0.0;3],[0.0,0.0,-model.body_to_support_z_m])[axis]+(p[axis]-bound)*n),[0.0,1.0],&mut changes);
                }
                continue;
            }
            if delta == 0.0 { continue; }
            let first = (from_support[axis].min(to_support[axis]) / size).floor() as i32 + 1;
            let last = (from_support[axis].max(to_support[axis]) / size).floor() as i32;
            for index in first..=last {
                let t = (f64::from(index) * size - from_support[axis]) / delta;
                if t > 0.0 && t < 1.0 { changes.push(t); }
            }
        }
        context.append_changes(&mut changes);
        changes.sort_by(f64::total_cmp);
        changes.dedup();
        let mut checked=AHashSet::new();
        let mut record = |interval: [f64; 2]| -> Result<(),SurfaceError> {
            let mut sample=|t:f64| {
                if t>0.0 && t<1.0 && checked.insert(t.to_bits()) {(check.borrow_mut())(t,None)} else {Ok(())}
            };
            let mut previous = interval[0];
            sample(previous)?;
            for &next in changes.iter().filter(|&&t| t > interval[0] && t < interval[1]) {
                sample((previous + next) * 0.5)?;
                sample(next)?;
                previous = next;
            }
            sample((previous + interval[1]) * 0.5)?;
            sample(interval[1])
        };
        // Recent returns have no terrain exemption. Enumerate the same exact
        // translating/rotating prism contact events as persistent occupancy.
        if let Some(recent)=&self.recent_obstacles {
            let fine=recent.size;
            for frame in &recent.frames {
                let low=[(min[0]/fine).floor() as i32,(min[1]/fine).floor() as i32];
                let high=[(max[0]/fine).floor() as i32,(max[1]/fine).floor() as i32];
                for (&[x,y],zs) in frame.columns.range(low..=high) {
                    if y<low[1] || y>high[1] {continue;}
                    {
                            for &z in zs {
                                if let Some(points)=frame.surface_points.get(&[x,y,z]) {
                                    for &point in points {
                                        if let Some(motion)=&mut rigid_motion {
                                                if let Some(t)=motion.intersection(point,point,[0.0,1.0])? {
                                                    (check.borrow_mut())(t,Some(BodyRejection::Occupancy{cell:[x,y,z],source:OccupancySource::Recent,
                                                        z_bounds:[point[2],point[2]],ground_height:None}))?;
                                                }
                                        }
                                        if let Some(interval)=prism.expanded_translated_interval(displacement,point,point,padding) {
                                            if let Some(events)=&events {
                                                for interval in events.contacts(model,point,point,interval,&changes) {record(interval)?;}
                                            } else {record(interval)?;}
                                        }
                                    }
                                    continue;
                                }
                                let low=[x,y,z].map(|v|f64::from(v)*fine);
                                let high=[x,y,z].map(|v|f64::from(v+1)*fine);
                                if high[2]<min[2] || low[2]>max[2] {continue;}
                                if let Some(motion)=&mut rigid_motion {
                                        if let Some(t)=motion.intersection(low,high,[0.0,1.0])? {
                                            (check.borrow_mut())(t,Some(BodyRejection::Occupancy{cell:[x,y,z],source:OccupancySource::Recent,
                                                z_bounds:[low[2],high[2]],ground_height:None}))?;
                                        }
                                }
                                if let Some(interval)=prism.expanded_translated_interval(displacement,low,high,padding) {
                                    if let Some(events)=&events {
                                        for interval in events.contacts(model,low,high,interval,&changes) {record(interval)?;}
                                    } else {record(interval)?;}
                                }
                            }
                    }
                }
            }
        }
        // 精细占据可独立于粗地图存在/更高；只遍历本段局部 XY 的真实 column_levels。
        // 每个 fine key 在此处理一次，粗路径只判断是否有 fine 接管，不重复窄相位。
        if rigid_motion.is_some() { if let Some(observations) = &self.observations {
            let fine = f64::from(observations.size_m);
            for x in (min[0]/fine).floor() as i32..=(max[0]/fine).floor() as i32 {
                for y in (min[1]/fine).floor() as i32..=(max[1]/fine).floor() as i32 {
                    let Some(levels) = observations.column_levels(x,y) else { continue; };
                    for &z in levels {
                        let key = [x,y,z];
                        let low = key.map(|v|f64::from(v)*fine);
                        let high = key.map(|v|(f64::from(v)+1.0)*fine);
                        if high[2]<min[2] || low[2]>max[2] { continue; }
                        if let Some(motion)=&mut rigid_motion {
                            if let Some(t)=motion.intersection(low,high,[0.0,1.0])? {
                                (check.borrow_mut())(t,Some(BodyRejection::Occupancy {
                                    cell:key,source:OccupancySource::Fine,
                                    z_bounds:[low[2],high[2]],ground_height:None,
                                }))?;
                            }
                        }
                        if let Some(interval)=prism.expanded_translated_interval(displacement,low,high,padding) {
                            if let Some(events)=&events {
                                for interval in events.contacts(model,low,high,interval,&changes) {record(interval)?;}
                            } else {record(interval)?;}
                        }
                    }
                }
            }
        }}
        let uncertainty = f64::from(self.source_voxel_size_m()) + model.support_uncertainty_m;
        let contact = model.max_step_up_m.max(model.max_step_down_m) + uncertainty;
        let allowance = ((model.z_bounds_m[0].max(-model.body_to_support_z_m) / r[2][2])
            + contact + 1e-6).min(uncertainty);
        for x in (min[0] / size).floor() as i32..=(max[0] / size).floor() as i32 {
            for y in (min[1] / size).floor() as i32..=(max[1] / size).floor() as i32 {
                let Some(zs) = self.data.occupied_columns.get(&(x, y)) else { continue; };
                let column_interval = prism.expanded_translated_interval(displacement,
                    [f64::from(x)*size, f64::from(y)*size, min[2]-size],
                    [f64::from(x+1)*size, f64::from(y+1)*size, max[2]+size],padding);
                let Some(column_interval)=column_interval.or_else(||rigid_motion.as_ref().map(|_|[0.0,1.0])) else {continue;};
                let ground = self.data.physical_graph.surface_lookup.get(&(x, y))
                    .into_iter().flatten().map(|&z| self.cell_position((x,y,z))[2])
                    .fold(f64::INFINITY, f64::min);
                let plane = from.0[2] - (((f64::from(x)+0.5)*size-from.0[0])*r[2][0]
                    + ((f64::from(y)+0.5)*size-from.0[1])*r[2][1]) / r[2][2];
                let plane_change = displacement[2]
                    + (displacement[0]*r[2][0]+displacement[1]*r[2][1])/r[2][2];
                let allowed = (plane + plane_change*column_interval[0])
                    .min(plane + plane_change*column_interval[1]) + allowance;
                let allowed=if rotating && ground.is_finite() {
                    events.as_ref().expect("已构造实际旋转轨迹")
                        .ground_allowance(model,x,y,size,contact,uncertainty)
                } else if rotating {
                    // 没有真实支撑面的列保留原保守界，不能将其低障碍当作地面跳过。
                    let upward=events.as_ref().expect("已构造实际旋转轨迹").minimum_upward_z;
                    if upward>0.0 {
                        let reach=(max[0]-min[0]+size).hypot(max[1]-min[1]+size);
                        let slope=(1.0-upward*upward).max(0.0).sqrt()/upward;
                        from.0[2].min(to.0[2])-reach*slope
                            +(model.z_bounds_m[0].max(-model.body_to_support_z_m)/upward+contact+1e-6).min(uncertainty)
                    } else {f64::NEG_INFINITY}
                } else {allowed};
                let cutoff = if rigid_motion.is_none() && context.is_legacy() {(ground + 1e-6).min(allowed - 1e-10)} else {f64::NEG_INFINITY};
                for &z in zs {
                    let bounds = self.data.height_bounds.get(&(x,y,z)).copied().unwrap_or([
                        z as f32*self.config().voxel_size, (z as f32+1.0)*self.config().voxel_size]);
                    if f64::from(bounds[1]) <= cutoff || f64::from(bounds[1]) < min[2]
                        || f64::from(bounds[0]) > max[2] { continue; }
                    let mut found_fine = false;
                    if let Some(observations) = &self.observations {
                        let fine = f64::from(observations.size_m);
                        for ix in (f64::from(x)*size/fine).round() as i32..(f64::from(x+1)*size/fine).round() as i32 {
                            for iy in (f64::from(y)*size/fine).round() as i32..(f64::from(y+1)*size/fine).round() as i32 {
                                for iz in (f64::from(bounds[0])/fine+1e-5).floor() as i32..(f64::from(bounds[1])/fine-1e-5).ceil() as i32 {
                                    let key = [ix,iy,iz];
                                    if observations.get(key).is_none() { continue; }
                                    found_fine = true;
                                    if rigid_motion.is_none() {
                                        if f64::from(iz+1)*fine <= cutoff { continue; }
                                        let low=key.map(|v|f64::from(v)*fine);let high=key.map(|v|f64::from(v+1)*fine);
                                        if let Some(interval)=prism.expanded_translated_interval(displacement,low,high,padding) {
                                            if let Some(events)=&events {for interval in events.contacts(model,low,high,interval,&changes) {record(interval)?;}}
                                            else {record(interval)?;}
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if !found_fine {
                        let low=[f64::from(x)*size,f64::from(y)*size,f64::from(bounds[0])];
                        let high=[f64::from(x+1)*size,f64::from(y+1)*size,f64::from(bounds[1])];
                        if let Some(motion)=&mut rigid_motion {
                                if let Some(t)=motion.intersection(low,high,[0.0,1.0])? {
                                    (check.borrow_mut())(t,Some(BodyRejection::Occupancy{cell:[x,y,z],source:OccupancySource::CoarseFallback,
                                        z_bounds:[low[2],high[2]],ground_height:None}))?;
                                }
                        }
                        if let Some(interval) = prism.expanded_translated_interval(displacement,low,high,padding) {
                            if let Some(events)=&events {for interval in events.contacts(model,low,high,interval,&changes) {record(interval)?;}}
                            else {record(interval)?;}
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn translated_interval_contains_all_grazing_pose_contacts() {
        let model=SurfaceBodyEnvelope {fixed_collision:None,footprint_xy_m:vec![[-0.3,-0.16],[0.3,-0.16],[0.3,0.16],[-0.3,0.16]],
            z_bounds_m:[-0.4,0.06],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let low=[0.025,0.025,-0.2];let high=[0.075,0.075,-0.15];
        for n in 1..80 {
            let angle=f64::from(n)*0.073;let r=rotation([0.0,0.0,(angle*0.5).sin(),(angle*0.5).cos()]).unwrap();
            let gap=0.16+(r[1][0].abs()+r[1][1].abs())*0.025+1e-10/0.6;
            for jitter in -20..=20 {
                let from=[0.05-r[1][0]*gap+f64::from(jitter)*1e-17,0.05-r[1][1]*gap,0.0];
                let delta=r[0].map(|v|v*0.0043);
                let interval=PrismQuery::new(&model,r,from).expanded_translated_interval(delta,low,high,0.0);
                for t in [0.0,0.25,0.5,0.75,1.0] {
                    let at=std::array::from_fn(|i|from[i]+delta[i]*t);
                    if PrismQuery::new(&model,r,at).intersects(low,high) {
                        assert!(interval.is_some_and(|v|v[0]<=t+1e-8 && v[1]>=t-1e-8),
                            "候选区间不能遗漏实际相交：angle={angle} jitter={jitter} t={t} interval={interval:?}");
                    }
                }
            }
        }
    }

    #[test]
    fn ground_allowance_covers_all_actual_column_corners_and_attitudes() {
        let model=SurfaceBodyEnvelope {fixed_collision:None,footprint_xy_m:vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]],
            z_bounds_m:[-0.448,0.1],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        let uncertainty=0.05;let contact=0.20;let size=0.1;
        for a in [[0.0,0.0,0.0,1.0],[0.10,-0.09,-0.7,0.7],[0.65,0.0,0.0,0.76]] {
            for b in [a,[a[0]+1e-18,a[1],a[2],a[3]],[a[0]+0.001,a[1]-0.002,a[2]+0.003,a[3]],
                [0.3,0.2,0.4,0.8],[-0.5,0.4,-0.4,0.7]] {
                let events=RotationEvents::new(&model,([0.02,-0.01,0.4],a),([0.12,-0.07,0.43],b));
                for x in -3..=3 {for y in -3..=3 {
                    let bound=events.ground_allowance(&model,x,y,size,contact,uncertainty);
                    for step in 0..=100 {
                        let (p,r)=events.transform(step as f64/100.0);
                        if r[2][2]<=0.0 {assert!(!bound.is_finite());continue;}
                        for ix in [x,x+1] {for iy in [y,y+1] {
                            let plane=p[2]-((f64::from(ix)*size-p[0])*r[2][0]
                                +(f64::from(iy)*size-p[1])*r[2][1])/r[2][2];
                            let actual=(plane+model.z_bounds_m[0].max(-model.body_to_support_z_m)/r[2][2]+contact+1e-6)
                                .min(plane+uncertainty);
                            assert!(bound<=actual,"逐列下界不可跳过真实超限接触: {bound} > {actual}");
                        }}
                    }
                }}
            }
        }
        let from=([0.02,-0.01,0.4],[0.08,-0.08,-0.73,0.67]);
        let to=([0.04,-0.01,0.4],[0.0801,-0.0801,-0.73,0.67]);
        let events=RotationEvents::new(&model,from,to);
        assert!(events.ground_allowance(&model,0,0,size,contact,uncertainty)>0.16,
            "实际足下列无需继承整个扫掠外包框的远端坡面下降量");
    }

    #[test]
    fn finite_feature_bounds_cover_real_quaternion_motion() {
        let model=SurfaceBodyEnvelope {fixed_collision:None,footprint_xy_m:vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]],
            z_bounds_m:[-0.4,0.1],body_to_support_z_m:0.4,max_step_up_m:0.15,max_step_down_m:0.15,
            max_cross_slope_deg:18.0,support_uncertainty_m:0.0};
        for q in [[0.001,-0.002,0.0004,1.0],[0.2,-0.1,0.7,0.6],[1e-18,-2e-18,0.0,1.0]] {
            let events=RotationEvents::new(&model,([2.0,-1.0,0.5],[0.0,0.0,0.0,1.0]),([2.4,-0.7,0.6],q));
            let contains=|(low,high):([f64;3],[f64;3]),point:[f64;3]|
                (0..3).all(|i|point[i]>=low[i]-1e-12 && point[i]<=high[i]+1e-12);
            for index in 0..=100 {
                let (p,r)=events.transform(index as f64/100.0);
                assert!(events.minimum_upward_z<=r[2][2],"解析倾斜界须覆盖每个真实姿态");
                for (index,&(v,_)) in events.vertices.iter().enumerate() {
                    assert!(contains(events.vertex_bounds[index],world(r,p,v)));
                }
                for (index,&(normal,height,_)) in events.faces.iter().enumerate() {
                    for &(v,_) in &events.vertices {if (dot(normal,v)-height).abs()<1e-12 {
                        assert!(contains(events.face_bounds[index],world(r,p,v)));
                    }}
                }
                for (index,&(v,e,_,_)) in events.edges.iter().enumerate() {
                    for fraction in [0.0,0.5,1.0] {
                        assert!(contains(events.edge_bounds[index],world(r,p,std::array::from_fn(|i|v[i]+e[i]*fraction))));
                    }
                }
            }
        }
    }

    #[test]
    fn yaw_motion_preserves_tilt_bound_without_discarding_real_tilt() {
        let raw=[0.03_f64,-0.02,0.1,1.0];let norm=raw.iter().map(|v|v*v).sum::<f64>().sqrt();
        let from=raw.map(|v|v/norm);let expected=rotation(from).unwrap()[2][2];
        for angle in [0.001_f64,0.08,0.5,2.8] {
            let (s,c)=(angle*0.5).sin_cos();
            let to=[c*from[0]-s*from[1],s*from[0]+c*from[1],c*from[2]+s*from[3],c*from[3]-s*from[2]];
            let bound=minimum_upward_z(from,to);
            assert!(bound<=expected && (bound-expected).abs()<1e-12,
                "真实偏航保留原倾斜，不能把偏航角累加成倾斜: {angle} {bound} {expected}");
        }
    }

    #[test]
    fn cubic_contact_events_retain_crossings_and_tangent_contact() {
        for expected in [[0.12,0.51,0.91],[0.28,0.28,0.76]] {
            let values=std::array::from_fn(|i| {let t=i as f64/3.0;expected.into_iter().map(|root|t-root).product()});
            let mut roots=Vec::new();cubic_event_roots(values,[0.0,1.0],&mut roots);
            for t in expected {assert!(roots.iter().any(|&r|(r-t).abs()<1e-8),"接触时刻{t}缺失: {roots:?}");}
        }
    }
}

#[cfg(test)]
#[path = "body_interpolation_reuse_tests.rs"]
mod interpolation_reuse_tests;
