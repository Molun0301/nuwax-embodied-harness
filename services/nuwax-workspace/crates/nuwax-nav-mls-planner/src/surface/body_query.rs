//! Immutable actual-body geometry queries; never synthesize or clear source cells.
use super::*;

/// 共享原始不可变几何的机体查询视图。
/// 不提供路线、前沿或快照提取接口，不能作为全局规划地图使用。
#[derive(Clone,Debug)]
pub struct SurfaceBodyQueryView {
    snapshot: SurfaceSnapshot,
}
impl SurfaceBodyQueryView {
    /// Associate the initial actual support point with this immutable owner.
    /// Later transitions consume the returned state, never the changing body Z.
    pub fn start_ground_state(&self,from:([f64;3],[f64;4]))->Result<BodyGroundState,SurfaceError> {
        self.snapshot.start_owned_ground_state(from)
    }
    /// Reassociate an already observed actual pose without granting a motion
    /// permission or changing its geometry. Unknown samples carry prior h.
    /// The previous state must belong to this same ground source.
    pub fn observe_actual_ground(&self,from:([f64;3],[f64;4]),previous:&BodyGroundState,
        actual:([f64;3],[f64;4]))->Result<BodyGroundState,SurfaceError> {
        // An actual observation may already be on another observed floor. Its
        // support Z selects a source layer; the returned height still comes only
        // from that layer. This is reassociation, not permission to move there.
        let observed=self.snapshot.start_owned_ground_state(actual)?;
        if matches!(observed.query_state(),GroundQueryState::Observed|GroundQueryState::LegacyGeometry) {
            return Ok(observed);
        }
        // A fresh unknown start carries actual support Z as its initial seed.
        // Do not adopt that seed here: retain the previous actual trajectory's
        // last known h (including any known columns crossed before this gap).
        let reference=self.snapshot.owned_ground_motion_reference(from,actual,previous)?;
        Ok(BodyGroundState {sample:reference.at(1.0).clone()})
    }
    /// Ground association can survive a new recent cloud, while collision
    /// proofs must still retire. No value comparison or content hash is used.
    pub fn same_ground_source(&self,other:&SurfaceSnapshot)->bool {
        let observations=match (&self.snapshot.observations,&other.observations) {
            (None,None)=>true,(Some(a),Some(b))=>Arc::ptr_eq(a,b),_=>false,
        };
        self.snapshot.ground_evidence==other.ground_evidence
            && Arc::ptr_eq(&self.snapshot.data,&other.data) && observations
    }
    /// Transport the actual pose using source height at the actual support XY.
    /// The result retains the same segment interpretation for its swept check.
    pub fn project_motion(&self,from:([f64;3],[f64;4]),ground:&BodyGroundState,xy:[f64;2],yaw:f64,
        witness:&mut Option<BodyFailureWitness>)->Result<BodyMotionProjection,SurfaceError> {
        self.snapshot.project_owned_ground_motion(from,ground,xy,yaw,witness)
    }
    /// Sweep a projection with the exact ground interpretation that produced it.
    pub fn validate_projected_motion(&self,from:([f64;3],[f64;4]),projection:&BodyMotionProjection,
        boundary:&[[f64;2]],witness:&mut Option<BodyFailureWitness>)->Result<(),SurfaceError> {
        if from!=projection.from {return Err(SurfaceError::InvalidPose);}
        let reference=BodyQueryReference::Ground(Arc::clone(&projection.reference));
        self.snapshot.validate_body_motion_in_context(from,projection.pose,boundary,
            QuerySegment {reference:Some(&reference),samples:[0.0,1.0]},witness)
    }
    /// Check an externally supplied actual trajectory without projecting its Z or
    /// attitude. Return only the next small association after a successful sweep.
    pub fn validate_motion_from_ground(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),
        ground:&BodyGroundState,boundary:&[[f64;2]],witness:&mut Option<BodyFailureWitness>)
        ->Result<BodyGroundState,SurfaceError> {
        self.snapshot.validate_owned_actual_support(from,&ground.sample)?;
        let reference=self.snapshot.owned_ground_motion_reference(from,to,ground)?;
        let next=BodyGroundState {sample:reference.at(1.0).clone()};
        self.snapshot.validate_owned_actual_support(to,&next.sample)?;
        let reference=BodyQueryReference::Ground(Arc::new(reference));
        self.snapshot.validate_body_motion_in_context(from,to,boundary,
            QuerySegment {reference:Some(&reference),samples:[0.0,1.0]},witness)?;
        Ok(next)
    }
    /// 几何、原始观察和近期障碍同源时，可复用相同位姿的运动检查。
    pub fn same_body_query_view(&self,other:&SurfaceSnapshot)->bool {
        self.snapshot.same_body_query_view(other)
    }
    /// 沿同一真实地形响应推进，仍保留实际起点的高度与姿态偏差。
    pub fn body_pose_after_motion(&self,from:([f64;3],[f64;4]),xy:[f64;2],yaw:f64)
        ->Result<([f64;3],[f64;4]),SurfaceError> {
        self.snapshot.body_pose_after_motion(from,xy,yaw)
    }
    /// Same projection, with one actual body rejection sample when available.
    pub fn body_pose_after_motion_with_witness(&self,from:([f64;3],[f64;4]),xy:[f64;2],yaw:f64,
        slot:&mut Option<BodyFailureWitness>)->Result<([f64;3],[f64;4]),SurfaceError> {
        self.snapshot.body_pose_after_motion_with_witness(from,xy,yaw,slot)
    }
    /// Same sweep, preserving the rejected sampled pose through cache hits.
    pub fn validate_body_motion_with_witness(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),boundary:&[[f64;2]],
        slot:&mut Option<BodyFailureWitness>)->Result<(),SurfaceError> {
        self.snapshot.validate_body_motion_with_witness(from,to,boundary,slot)
    }
    /// 完整检查机体、支撑、实际占据接触事件及外部边界。
    pub fn validate_body_motion(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),boundary:&[[f64;2]])
        ->Result<(),SurfaceError> {
        self.snapshot.validate_body_motion(from,to,boundary)
    }
}

/// Small source-local ground association carried by a search node or controller.
/// Equality includes only data used by future transitions; it copies no members.
#[derive(Clone,Debug)]
pub struct BodyGroundState {
    pub(in crate::surface) sample:GroundSample,
}
impl BodyGroundState {
    fn identity(&self)->(u8,[u64;3],Option<[u64;3]>,&[NativeGroundSource]) {
        let sample=&self.sample;
        let mut origin=sample.reference_origin.map(f64::to_bits);
        if sample.reference_normal.is_none() || sample.query_state==GroundQueryState::Unknown {
            origin[0]=0;origin[1]=0;
        }
        (sample.query_state as u8,origin,sample.reference_normal.map(|n|n.map(f64::to_bits)),
            &sample.reference_sources)
    }
    /// Whether the current support column has direct, legacy or unknown height.
    pub fn query_state(&self)->GroundQueryState {self.sample.query_state}
    /// Last usable source height, carried unchanged through an unknown column.
    pub fn height(&self)->f64 {self.sample.reference_origin[2]}
    /// Exact native source key, when the last association has a measured record.
    pub fn ground_key(&self)->Option<GroundKey> {
        self.sample.reference_sources.first().and_then(|source|match source {
            NativeGroundSource::ObservedFine{anchor}=>Some(GroundKey::LegacyPlane(*anchor)),
            NativeGroundSource::HeightfieldFine{key}=>Some(*key),_=>None,
        })
    }
}
impl PartialEq for BodyGroundState {fn eq(&self,other:&Self)->bool {self.identity()==other.identity()}}
impl Eq for BodyGroundState {}
impl std::hash::Hash for BodyGroundState {
    fn hash<H:std::hash::Hasher>(&self,state:&mut H) {std::hash::Hash::hash(&self.identity(),state);}
}

/// One projected endpoint and its short-lived, owner-local swept interpretation.
#[derive(Clone,Debug)]
pub struct BodyMotionProjection {
    pub(in crate::surface) from:([f64;3],[f64;4]),
    pub(in crate::surface) pose:([f64;3],[f64;4]),
    pub(in crate::surface) ground:BodyGroundState,
    pub(in crate::surface) reference:Arc<LocalGroundReference>,
}
impl BodyMotionProjection {
    /// Projected actual body position and complete quaternion.
    pub fn pose(&self)->([f64;3],[f64;4]) {self.pose}
    /// Small endpoint state to carry after this motion is accepted.
    pub fn ground(&self)->&BodyGroundState {&self.ground}
}
impl SurfaceSnapshot {
    /// A zero-copy geometry query over the unchanged source. Ground evidence is
    /// descriptive; actual pose and motion validation remain explicit operations.
    pub fn body_query(&self) -> SurfaceBodyQueryView {
        SurfaceBodyQueryView { snapshot: self.clone() }
    }

    /// Compatibility alias: validate the actual pose and share the unchanged source.
    pub fn body_query_with_current_support(&self,body:[f64;3],quaternion:[f64;4])
        ->Result<SurfaceBodyQueryView,SurfaceError> {
        self.validated_body_snapshot(body,quaternion,&mut None).map(|snapshot|SurfaceBodyQueryView{snapshot})
    }

    /// Same geometry-only alias, preserving an actual collision witness.
    pub fn body_query_with_current_support_and_witness(&self,body:[f64;3],quaternion:[f64;4],
        slot:&mut Option<BodyFailureWitness>)->Result<SurfaceBodyQueryView,SurfaceError> {
        self.validated_body_snapshot(body,quaternion,slot).map(|snapshot|SurfaceBodyQueryView{snapshot})
    }

    /// Geometry, observation and recent-obstacle identity for exact-pose query reuse.
    pub fn same_body_query_view(&self,other:&Self)->bool {
        fn same<T>(a:&Option<Arc<T>>,b:&Option<Arc<T>>)->bool {
            match(a,b){(None,None)=>true,(Some(a),Some(b))=>Arc::ptr_eq(a,b),_=>false}
        }
        self.ground_evidence==other.ground_evidence && Arc::ptr_eq(&self.data,&other.data)&&same(&self.observations,&other.observations)
            &&same(&self.recent_obstacles,&other.recent_obstacles)
    }

    /// Conservative equality for Explore's derived planning queries only.
    /// Admitted raw timestamps/hits can differ; this is neither source identity
    /// nor permission to reuse a live body-execution cache. All derived layer
    /// fields, occupancy, completeness, body geometry and recent input remain equal.
    pub fn same_planning_query_view(&self,other:&Self)->bool {
        let observations=match (&self.observations,&other.observations) {
            (None,None)=>true,
            (Some(left),Some(right))=>left.same_planning_query_view(right),
            _=>false,
        };
        let recent=match (&self.recent_obstacles,&other.recent_obstacles) {
            (None,None)=>true,
            (Some(left),Some(right))=>Arc::ptr_eq(left,right),
            _=>false,
        };
        self.ground_evidence==other.ground_evidence && Arc::ptr_eq(&self.data,&other.data)
            && observations && recent
    }

    pub(super) fn can_defer_body_query_edges(&self)->bool {
        let config=self.config();
        // 软项只影响排序，但溢出成无穷也会拒绝边；仅在其保守上界有限时延后。
        let rise=(self.data.candidate_step_cells as f32+2.0)*config.voxel_size;
        let upper_cost=config.voxel_size.hypot(rise)*(2.0+2.0*config.wall_buffer_weight)
            +config.step_penalty_weight*rise;
        config.wall_clearance_m==0.0 && upper_cost.is_finite()
    }

    /// Conservative XY read radius for a reference-only geometry refresh advisory.
    /// `pure_yaw` requires every reference quaternion to have exactly zero X/Y;
    /// `xy_scale` bounds its coordinates. This never grants a motion permission.
    pub fn reference_geometry_scope_radius(&self,pure_yaw:bool,xy_scale:f64)->Option<f64> {
        if !self.can_defer_body_query_edges() || !xy_scale.is_finite() || xy_scale<0.0
            || self.observations.as_ref().is_some_and(|observations|
                observations.size_m.to_bits()!=self.source_voxel_size_m().to_bits()) {return None;}
        let model=self.body_envelope()?;
        let s=f64::from(self.config().voxel_size);let f=f64::from(self.source_voxel_size_m());
        let mut r=model.radius();
        // Same rotation scale as validate_body_motion_checked_with_witness.
        let mut rho=r.hypot(model.z_bounds_m.into_iter().map(f64::abs)
            .fold(model.body_to_support_z_m,f64::max));
        if let Some(geometry)=&model.fixed_collision {
            for primitive in &geometry.primitives {
                let shape_radius=match primitive.shape {
                    nuwax_types::FixedCollisionShape::Box{size_m}=>
                        (size_m[0]*0.5).hypot(size_m[1]*0.5).hypot(size_m[2]*0.5),
                    nuwax_types::FixedCollisionShape::Sphere{radius_m}=>radius_m,
                    nuwax_types::FixedCollisionShape::Cylinder{radius_m,length_m}=>radius_m.hypot(length_m*0.5),
                };
                let [x,y,z]=primitive.translation_m;
                // Actual queries append the rigid world-AABB corners to their
                // footprint. Cover corners of the union of all primitive AABBs,
                // including corners combining extents from different primitives.
                let fixed_xy=std::f64::consts::SQRT_2*(x.hypot(y)+shape_radius);
                let fixed_xyz=3.0_f64.sqrt()*(x.hypot(y).hypot(z)+shape_radius);
                if !fixed_xy.is_finite() || !fixed_xyz.is_finite() {return None;}
                r=r.max(fixed_xy);rho=rho.max(fixed_xyz);
            }
        }
        let (projection,h)=if pure_yaw {(r,0.0)}else{(rho,rho)};
        // MLS cell centres use f32. Cover their rounding before ground_patch's
        // ceil, and round the final world-space bound outward as well.
        let rounding=16.0*f64::from(f32::EPSILON)*xy_scale.max(rho).max(s).max(1.0)+1e-9;
        let diagonal=std::f64::consts::SQRT_2;let d=h+s/diagonal+rounding;
        let profile=(f64::from(self.data.profile_cells)+1.0)*s;
        // Keep the actual fine fallback's f32 division/ceil, not a f64 variant.
        let depth=(self.config().min_step_tread_depth_m/self.source_voxel_size_m()).ceil();
        if !depth.is_finite() || depth<1.0 || depth>=i32::MAX as f32 {return None;}
        let radii=[
            // Raw ground_patch and its final cardinal contact_neighbors probe;
            // this read chain does not traverse classified edges or wall costs.
            d+((projection+d+2.0*s+rounding)/s).ceil()*s+s,
            // validate_segment's support/diagonal side columns and side_profile.
            h+diagonal*s+profile,
            // locate_step_contact's legal_step profile within the footprint.
            projection+diagonal*s+profile,
            // Fine fringe plus supports_obstacle's neighbouring evidence.
            rho+diagonal*f,
            // Independent fine_observed_tread_support_samples axial tread,
            // diagonal side column, and neighbouring evidence. Never omit it.
            h+f64::from(depth)*f+2.0*diagonal*f,
        ];
        let radius=radii.into_iter().fold(0.0,f64::max)+rounding;
        (radii.into_iter().all(|v|v.is_finite()&&v>=0.0)&&radius.is_finite()).then_some(radius)
    }


}

// Request-local reference data. None of these private modes is selected by the
// public body-query constructors. Members always remain owned by observations.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub(in crate::surface) enum NativeGroundSource {
    ObservedFine { anchor: [i32; 3] },
    HeightfieldFine { key: GroundKey },
    LegacyFineRun { xy: [i32; 2], first: i32, last: i32 },
    CoarseCell(VoxelKey),
}
#[derive(Clone, Debug, PartialEq)]
pub(in crate::surface) struct GroundSample {
    pub reference_origin: [f64; 3],
    pub reference_normal: Option<[f64; 3]>,
    pub query_state: GroundQueryState,
    pub reference_sources: Vec<NativeGroundSource>,
}
impl GroundSample {
    pub(in crate::surface) fn height_at(&self, xy: [f64; 2]) -> f64 {
        self.reference_normal.map_or(self.reference_origin[2], |n|
            self.reference_origin[2] - (n[0]*(xy[0]-self.reference_origin[0])
                + n[1]*(xy[1]-self.reference_origin[1]))/n[2])
    }
}
#[derive(Clone, Debug)]
pub(in crate::surface) struct LocalGroundReference {
    pub first: GroundSample,
    pub changes: Vec<(f64, GroundSample)>,
}
impl LocalGroundReference {
    pub(in crate::surface) fn at(&self, t: f64) -> &GroundSample {
        // The owner supplies this one segment's ordered changes. Right-continuous
        // lookup has an explicit first sample, never an inferred layer/sample 0.
        self.changes.iter().rev().find(|(at,_)| *at<=t)
            .map_or(&self.first, |(_,sample)| sample)
    }
}
#[derive(Clone, Debug)]
pub(in crate::surface) enum BodyQueryReference {
    Ground(Arc<LocalGroundReference>),
    Unanchored,
}
#[derive(Clone, Debug)]
pub(in crate::surface) struct BoundBodyQueryView {
    snapshot: SurfaceSnapshot,
    reference: BodyQueryReference,
}
#[derive(Clone, Copy)]
pub(in crate::surface) enum QuerySample<'a> {
    LegacyNominal,
    Ground(&'a GroundSample),
    Unanchored,
}
impl<'a> QuerySample<'a> {
    pub(in crate::surface) fn is_legacy(self) -> bool { matches!(self,Self::LegacyNominal) }
    pub(in crate::surface) fn ground(self) -> Option<&'a GroundSample> {
        match self { Self::Ground(sample)=>Some(sample), _=>None }
    }
    fn same(self, other: Self) -> bool {
        match (self,other) {
            (Self::Ground(a),Self::Ground(b))=>std::ptr::eq(a,b),
            (Self::LegacyNominal,Self::LegacyNominal)|(Self::Unanchored,Self::Unanchored)=>true,
            _=>false,
        }
    }
}
#[derive(Clone, Copy)]
pub(in crate::surface) struct QuerySegment<'a> {
    reference: Option<&'a BodyQueryReference>,
    samples: [f64; 2],
}
impl<'a> QuerySegment<'a> {
    pub(in crate::surface) const LEGACY: Self = Self { reference:None, samples:[0.0,1.0] };
    pub(in crate::surface) fn for_reference(reference:&'a BodyQueryReference)->Self {
        Self {reference:Some(reference),samples:[0.0,1.0]}
    }
    pub(in crate::surface) fn at(self, s: f64) -> QuerySample<'a> {
        match self.reference {
            None=>QuerySample::LegacyNominal,
            Some(BodyQueryReference::Unanchored)=>QuerySample::Unanchored,
            Some(BodyQueryReference::Ground(reference))=>QuerySample::Ground(reference.at(
                self.samples[0]+s*(self.samples[1]-self.samples[0]))),
        }
    }
    pub(in crate::surface) fn is_legacy(self) -> bool { self.reference.is_none() }
    pub(in crate::surface) fn append_changes(self, changes: &mut Vec<f64>) {
        let Some(BodyQueryReference::Ground(reference))=self.reference else {return;};
        let delta=self.samples[1]-self.samples[0];
        if delta==0.0 {return;}
        changes.extend(reference.changes.iter().filter_map(|(at,_)| {
            let s=(*at-self.samples[0])/delta;(s>0.0&&s<1.0).then_some(s)
        }));
    }
    pub(in crate::surface) fn constant(self) -> bool {
        if !self.at(0.0).same(self.at(1.0)) {return false;}
        let Some(BodyQueryReference::Ground(reference))=self.reference else {return true;};
        let [a,b]=self.samples;
        !reference.changes.iter().any(|(at,_)| *at>a.min(b)&&*at<a.max(b))
    }
}
impl BoundBodyQueryView {
    fn segment(&self,samples:[f64;2])->QuerySegment<'_> {
        QuerySegment {reference:Some(&self.reference),samples}
    }
    #[cfg(test)]
    pub(in crate::surface) fn segment_for_diagnostic(&self,samples:[f64;2])->QuerySegment<'_> {
        self.segment(samples)
    }
    pub(in crate::surface) fn pose_at(&self,actual:([f64;3],[f64;4]),t:f64,
        witness:&mut Option<BodyFailureWitness>)->Result<(),SurfaceError> {
        self.snapshot.validate_body_pose_in_context(actual.0,actual.1,self.segment([t,t]).at(0.0),witness)
    }
    pub(in crate::surface) fn project_from_to_samples(&self,from:([f64;3],[f64;4]),xy:[f64;2],yaw:f64,
        samples:[f64;2],witness:&mut Option<BodyFailureWitness>)->Result<([f64;3],[f64;4]),SurfaceError> {
        self.snapshot.body_pose_after_motion_in_context(from,xy,yaw,self.segment(samples),witness)
    }
    pub(in crate::surface) fn motion_on_segment(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),
        samples:[f64;2],boundary:&[[f64;2]],witness:&mut Option<BodyFailureWitness>)->Result<(),SurfaceError> {
        self.snapshot.validate_body_motion_in_context(from,to,boundary,self.segment(samples),witness)
    }
    pub(in crate::surface) fn geometry_checker<'a>(&'a self,boundary:&'a [[f64;2]])
        ->impl FnMut(([f64;3],[f64;4]),([f64;3],[f64;4]),[f64;2],&mut Option<BodyFailureWitness>)
            ->Result<(),SurfaceError>+'a {
        move |from,to,samples,witness|self.motion_on_segment(from,to,samples,boundary,witness)
    }
}
impl SurfaceSnapshot {
    pub(in crate::surface) fn bind_body_query(&self,reference:BodyQueryReference)->BoundBodyQueryView {
        // C0-a bypasses association-sensitive results; source-only caches stay
        // source-only. No shared cache is cleared or planning snapshot rebuilt.
        BoundBodyQueryView {snapshot:self.clone(),reference}
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
        let model=SurfaceBodyEnvelope {fixed_collision:None,footprint_xy_m:vec![[-0.2,-0.12],[0.3,-0.12],[0.3,0.12],[-0.2,0.12]],
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
            source.validate_body_pose(from.0,from.1).unwrap();
            let full=source.with_current_body_support(from.0,from.1).unwrap();
            let quick=source.body_query_with_current_support(from.0,from.1).unwrap();
            assert!(quick.same_body_query_view(&source));
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
            source.validate_body_pose(from.0,from.1).unwrap();
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
        assert!(a.same_body_query_view(&b.snapshot),"immutable queries share unchanged source identity");
        assert!(a.same_body_query_view(&blind));
        assert!(a.same_body_query_view(&a.snapshot.clone()));
        let moved=([from.0[0]-0.1,from.0[1],from.0[2]],from.1);
        let next=a.snapshot.body_query_with_current_support(moved.0,moved.1).unwrap();
        let direct=blind.body_query_with_current_support(moved.0,moved.1).unwrap();
        assert_eq!(next.validate_body_motion(moved,from,&[]),direct.validate_body_motion(moved,from,&[]),
            "后续足迹派生始终回到同一原始图，不累积旧盲区授权");
    }
}

#[cfg(test)]
mod planning_snapshot_identity_tests {
    use super::*;
    use std::collections::BTreeMap;
    use nuwax_map_chunks::{LayerObservation, LayerObservations, LayerSample};

    fn scene() -> SurfaceSnapshot {
        let points: Vec<_> = (-12..12).flat_map(|x| (-8..8).map(move |y|
            ((x as f32 + 0.5) * 0.1, (y as f32 + 0.5) * 0.1, 0.025))).collect();
        let config = Config {
            world_frame: "map".into(), voxel_size: 0.1, robot_height: 0.2,
            surface_closing_radius: 0.0, node_spacing_m: 0.3,
            wall_clearance_m: 0.0, wall_buffer_m: 0.2, wall_buffer_weight: 2.0,
            max_slope_deg: 18.0, step_threshold_m: 0.21,
            min_step_tread_depth_m: 0.2, step_penalty_weight: 2.0,
            goal_tolerance: 0.1, viz_publish_hz: 0.0,
        };
        let model = SurfaceBodyEnvelope {
            fixed_collision: None,
            footprint_xy_m: vec![[-0.2, -0.12], [0.3, -0.12], [0.3, 0.12], [-0.2, 0.12]],
            z_bounds_m: [-0.1, 0.15], body_to_support_z_m: 0.4,
            max_step_up_m: 0.21, max_step_down_m: 0.1,
            max_cross_slope_deg: 18.0, support_uncertainty_m: 0.0,
        };
        SurfaceSnapshot::from_points(&points, config, 0.05).unwrap()
            .with_body_envelope(model).unwrap()
    }

    fn record(time: u64, z: f64) -> Arc<LayerObservations> {
        Arc::new(BTreeMap::from([(0, LayerObservation {
            members: vec![0], normal: [0.0, 0.0, 1.0],
            plane_origin: [0.025, 0.025, 0.025],
            samples: vec![LayerSample {
                source: Default::default(), point: [0.025, 0.025, z],
                source_sequence: time, source_timestamp_ns: time, point_index: 0,
            }],
        })]))
    }

    fn observations() -> SurfaceObservations {
        let mut observations = SurfaceObservations::new(0.05, 32).unwrap();
        refresh(&mut observations, 100, 1, 0.02);
        observations
    }

    fn refresh(observations: &mut SurfaceObservations, time: u64, hits: u16, z: f64) {
        observations.replace_block([0, 0, 0], BTreeMap::from([
            ([0, 0, 0], SurfaceObservation { observed_ns: time, hits }),
        ])).unwrap();
        observations.replace_ground_layers([0, 0, 0], record(time, z)).unwrap();
        observations.mark_complete_geometry_source();
    }

    #[test]
    fn planning_snapshot_metadata_equivalence_keeps_body_execution_identity_strict() {
        let observations = Arc::new(observations());
        let original = scene().with_observations(Some(observations.clone()));
        let mut next = observations.as_ref().clone();
        refresh(&mut next, 200, 2, 0.02);
        let refreshed = original.with_observations(Some(Arc::new(next)));
        assert!(original.same_geometry(&refreshed));
        assert!(original.same_planning_query_view(&refreshed));
        assert!(refreshed.same_planning_query_view(&original));
        assert!(!original.same_body_query_view(&refreshed));
        assert!(!original.body_query().same_body_query_view(&refreshed));
        assert!(!Arc::ptr_eq(&original.body_query_cache, &refreshed.body_query_cache));
        assert!(!Arc::ptr_eq(original.observations().unwrap(), refreshed.observations().unwrap()));
        assert_eq!(original.observations().unwrap().get([0, 0, 0]),
            Some(SurfaceObservation { observed_ns: 100, hits: 1 }));
        assert_eq!(refreshed.observations().unwrap().get([0, 0, 0]),
            Some(SurfaceObservation { observed_ns: 200, hits: 2 }));

        let copied_owner = original.with_observations(Some(Arc::new(observations.as_ref().clone())));
        assert!(original.same_planning_query_view(&copied_owner));
        assert!(!original.same_body_query_view(&copied_owner));
        assert!(!Arc::ptr_eq(&original.body_query_cache, &copied_owner.body_query_cache));
        let reattached = original.with_observations(Some(observations));
        assert!(original.same_body_query_view(&reattached));
        assert!(original.same_planning_query_view(&reattached));
        assert!(Arc::ptr_eq(&original.body_query_cache, &reattached.body_query_cache));
    }

    #[test]
    fn planning_snapshot_keeps_observation_option_size_partition_and_completeness() {
        let base = scene();
        assert!(base.same_planning_query_view(&base.clone()));
        let empty = base.with_observations(Some(Arc::new(SurfaceObservations::new(0.05, 32).unwrap())));
        assert!(base.same_geometry(&empty));
        assert!(!base.same_planning_query_view(&empty));
        assert!(!empty.same_planning_query_view(&base));
        for (size, edge) in [(0.1, 32), (0.05, 16)] {
            let changed = base.with_observations(Some(Arc::new(SurfaceObservations::new(size, edge).unwrap())));
            assert!(empty.same_geometry(&changed));
            assert!(!empty.same_planning_query_view(&changed));
        }
        let original = base.with_observations(Some(Arc::new(observations())));
        let mut partial = original.observations().unwrap().as_ref().clone();
        partial.replace_block([0, 0, 0], BTreeMap::from([
            ([0, 0, 0], SurfaceObservation { observed_ns: 200, hits: 2 }),
        ])).unwrap();
        let partial = original.with_observations(Some(Arc::new(partial)));
        assert!(original.same_geometry(&partial));
        assert!(!original.same_planning_query_view(&partial));
        assert!(!original.same_planning_query_view(&original.with_observations(None)));
    }

    #[test]
    fn planning_snapshot_statistics_and_fine_keys_still_invalidate_under_same_data() {
        let original = scene().with_observations(Some(Arc::new(observations())));
        let mut changed = original.observations().unwrap().as_ref().clone();
        refresh(&mut changed, 200, 2, 0.03);
        assert!(original.observations().unwrap().same_geometry(&changed));
        let changed = original.with_observations(Some(Arc::new(changed)));
        assert!(original.same_geometry(&changed));
        assert!(!original.same_planning_query_view(&changed));
        for key in [[0, 0, 1], [1, 0, 0]] {
            let mut changed = original.observations().unwrap().as_ref().clone();
            changed.replace_block([0, 0, 0], BTreeMap::from([
                (key, SurfaceObservation { observed_ns: 200, hits: 2 }),
            ])).unwrap();
            changed.mark_complete_geometry_source();
            let changed = original.with_observations(Some(Arc::new(changed)));
            assert!(original.same_geometry(&changed), "coarse data owner is deliberately unchanged");
            assert!(!original.same_planning_query_view(&changed));
        }
    }

    #[test]
    fn planning_snapshot_rejects_different_data_body_mask_and_ground_interpretation() {
        let original = scene().with_observations(Some(Arc::new(observations())));
        let rebuilt = scene().with_observations(original.observations().cloned());
        assert!(!original.same_planning_query_view(&rebuilt), "independently rebuilt data stays conservative");
        let volume = original.clone().with_ground_evidence(GroundEvidence::OccupiedVolume);
        assert!(Arc::ptr_eq(&original.data, &volume.data));
        assert!(!original.same_planning_query_view(&volume));
        assert!(!original.same_body_query_view(&volume));
        assert!(!original.same_geometry(&volume));
        let mut envelope = original.body_envelope().unwrap().clone();
        envelope.z_bounds_m[1] += 0.01;
        let body = original.with_body_envelope(envelope).unwrap();
        assert!(!original.same_planning_query_view(&body));
        let mut blocked = vec![false; 40 * 40];
        blocked[20 * 40 + 20] = true;
        let masked = original.with_virtual_obstacles([-2.0, -2.0], 0.1, 40, 40, &blocked).unwrap();
        assert!(!original.same_planning_query_view(&masked));
        assert!(!original.same_geometry(&masked));
    }

    #[test]
    fn planning_snapshot_keeps_recent_owner_presence_refresh_clear_and_retention_guards() {
        let original = scene().with_observations(Some(Arc::new(observations())));
        let recent = original.with_recent_obstacle_points(1, 100, 0, &[],
            [(0, [0.025, 0.025, 0.4])]).unwrap();
        assert!(original.same_geometry(&recent));
        assert!(!original.same_planning_query_view(&recent));
        assert!(!recent.same_planning_query_view(&original));
        assert!(recent.same_planning_query_view(&recent.clone()));
        let changed = recent.with_recent_obstacle_points(2, 200, 0, &[],
            [(0, [0.025, 0.025, 0.4])]).unwrap();
        assert!(!recent.same_planning_query_view(&changed), "same points do not alias a new recent owner");
        let retained = recent.with_recent_obstacle_points(2, 200, 200, &[],
            [(0, [0.025, 0.025, 0.4])]).unwrap();
        assert!(!recent.same_planning_query_view(&retained));
        assert!(!changed.same_planning_query_view(&retained));
        assert!(!recent.same_planning_query_view(&recent.clear_recent_obstacles()));
        let empty_recent = original.with_recent_obstacle_points(1, 100, 0, &[], []).unwrap();
        assert!(!original.same_planning_query_view(&empty_recent), "None differs even from an empty recent frame");
        let mut next = original.observations().unwrap().as_ref().clone();
        refresh(&mut next, 200, 2, 0.02);
        let carried = original.with_observations(Some(Arc::new(next)))
            .with_recent_obstacles_from(&recent);
        assert!(recent.same_planning_query_view(&carried));
        assert!(!recent.same_body_query_view(&carried));
    }

    #[test]
    fn planning_snapshot_private_forks_never_masquerade_as_original_source_identity() {
        let original = scene().with_observations(Some(Arc::new(observations())));
        let left = original.fork_for_planning();
        let right = original.fork_for_planning();
        assert!(original.observations().unwrap()
            .same_planning_query_view(left.observations().unwrap()));
        assert!(!Arc::ptr_eq(&original.data, &left.data));
        assert!(!Arc::ptr_eq(&left.data, &right.data));
        assert!(!original.same_planning_query_view(&left));
        assert!(!original.same_body_query_view(&left));
        assert!(!original.same_geometry(&left));
        assert!(!left.same_planning_query_view(&right));
        assert!(left.same_planning_query_view(&left.clone()));
        assert!(left.same_body_query_view(&left.clone()));
    }
}
