//! Owner-bound scalar ground transport. Heights belong to actual support XY;
//! the calibrated offset participates only in body/support geometry conversion.
use super::*;
use crate::surface::body_query::{BodyGroundState,BodyMotionProjection,BodyQueryReference,
    BodyNavigationSegment,NavigationGroundSample,GroundSample,GroundSampleMode,LocalGroundReference};

fn transport_attitude(actual:[f64;4],before:Option<[f64;3]>,after:Option<[f64;3]>,yaw:f64)
    ->Result<[f64;4],SurfaceError> {
    let actual_rotation=rotation(actual)?;
    if actual_rotation[2][2]<=0.0 || !yaw.is_finite() {return Err(SurfaceError::InvalidPose);}
    let current_yaw=actual_rotation[0][1].atan2(actual_rotation[0][0]);
    // Missing direction is not a horizontal terrain observation. Use one common
    // frame so that it contributes no terrain-attitude change.
    let before=before.or(after).unwrap_or([0.0,0.0,1.0]);
    let after=after.unwrap_or(before);
    if before==after && (yaw-current_yaw).abs()<1e-12 {return Ok(actual);}
    let frame=|n:[f64;3],a:f64| {
        let forward=normalize([a.cos(),a.sin(),-(n[0]*a.cos()+n[1]*a.sin())/n[2]]);
        [forward,cross(n,forward),n]
    };
    let base=frame(before,current_yaw);let next=frame(after,yaw);
    let mut columns=actual_rotation.map(|axis|world(next,[0.0;3],base.map(|basis|dot(basis,axis))));
    let normal=columns[2];let (sine,cosine)=yaw.sin_cos();
    columns[0]=normalize([cosine*normal[2],sine*normal[2],-cosine*normal[0]-sine*normal[1]]);
    columns[1]=cross(normal,columns[0]);
    Ok(quaternion_from_columns(columns))
}

impl SurfaceSnapshot {
    pub(in crate::surface) fn validate_owned_actual_support(&self,pose:([f64;3],[f64;4]),
        ground:&GroundSample)->Result<(),SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let support=self.support_position_from_body(pose.0,pose.1,model.body_to_support_z_m)?;
        let xy=[support[0],support[1]];
        if self.known_terrain_height(xy,ground.height_at(xy)).is_some_and(|height| {
            let rounding=f64::from(self.source_voxel_size_m())+model.support_uncertainty_m+1e-6;
            height-support[2]>model.max_step_up_m+rounding
                || support[2]-height>model.max_step_down_m+rounding
        }) {return Err(SurfaceError::UntraversableSegment);}
        Ok(())
    }

    pub(in crate::surface) fn start_owned_ground_state(&self,from:([f64;3],[f64;4]))
        ->Result<BodyGroundState,SurfaceError> {
        self.validate_complete_source_resolution()?;
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        if rotation(from.1)?[2][2]<=0.0 {return Err(SurfaceError::InvalidPose);}
        let support=self.support_position_from_body(from.0,from.1,model.body_to_support_z_m)?;
        if !support.into_iter().all(f64::is_finite) {return Err(SurfaceError::InvalidPose);}
        let initial=GroundSample {mode:crate::surface::body_query::GroundSampleMode::Compatibility,reference_origin:support,reference_normal:None,
            query_state:GroundQueryState::Unknown,reference_sources:vec![]};
        Ok(BodyGroundState {sample:self.advance_owned_ground([support[0],support[1]],&initial)})
    }

    fn advance_owned_ground(&self,xy:[f64;2],previous:&GroundSample)->GroundSample {
        let reference=previous.height_at(xy);
        let mut native=None;
        let query=self.direct_ground_query_recorded(xy,reference,Some(&mut native));
        let Some(height)=query.height else {
            let mut carried=previous.clone();
            // Do not extrapolate a missing column with a retained optional plane.
            carried.query_state=GroundQueryState::Unknown;
            carried.reference_normal=None;
            return carried;
        };
        let direction=if query.state==GroundQueryState::LegacyGeometry {
            // The direct legacy scalar query's [0,0,1] is not a measured
            // direction. Reuse the original compatibility contact field for
            // attitude only; missing/rank-deficient fits never erase valid h.
            self.legacy_owned_ground_direction(xy,height)
        } else {query.normal};
        GroundSample {mode:crate::surface::body_query::GroundSampleMode::Compatibility,reference_origin:[xy[0],xy[1],height],reference_normal:direction,
            query_state:query.state,reference_sources:native.into_iter().collect()}
    }

    fn legacy_owned_ground_direction(&self,xy:[f64;2],height:f64)->Option<[f64;3]> {
        let model=self.body_envelope()?;
        let tolerance=model.max_step_up_m.max(model.max_step_down_m)
            +f64::from(self.source_voxel_size_m())+model.support_uncertainty_m;
        let centre=self.locate_near_support([xy[0],xy[1],height],tolerance).ok()?;
        let (_,attitude)=self.body_motion_contact_frame(centre,height,0.0).ok()?;
        Some(rotation(attitude).ok()?[2])
    }

    fn advance_navigation_ground(&self,xy:[f64;2],previous:&GroundSample,
        navigation:NavigationGroundSample<'_>)->Result<GroundSample,SurfaceError> {
        let (height,key)=match navigation {
            NavigationGroundSample::Unanchored=>return Ok(self.advance_owned_ground(xy,previous)),
            NavigationGroundSample::KnownNative{record,reference_xy}=>
                (record.height_at(reference_xy).ok_or(SurfaceError::InvalidPose)?,Some(record.key())),
            NavigationGroundSample::KnownLegacy{height,..}|NavigationGroundSample::Carried{height}=>
                (height,None),
        };
        let fine=f64::from(self.source_voxel_size_m());
        let column=xy.map(|v|(v/fine).floor() as i32);
        let selected=key.filter(|key|key.anchor()[..2]==column);
        let mut native=None;
        let query=self.direct_ground_query_selected(xy,height,selected,Some(&mut native));
        let Some(height)=query.height else {
            let mut carried=previous.clone();
            carried.mode=GroundSampleMode::NavigationScalar;
            carried.query_state=GroundQueryState::Unknown;
            carried.reference_normal=None;
            return Ok(carried);
        };
        let mode=if matches!(native,Some(NativeGroundSource::ObservedFine{..})) {
            GroundSampleMode::NavigationPlane
        } else {GroundSampleMode::NavigationScalar};
        let direction=if query.state==GroundQueryState::LegacyGeometry {
            self.legacy_owned_ground_direction(xy,height)
        } else {query.normal};
        Ok(GroundSample {mode,reference_origin:[xy[0],xy[1],height],reference_normal:direction,
            query_state:query.state,reference_sources:native.into_iter().collect()})
    }

    pub(in crate::surface) fn navigation_initial_support(&self,from:([f64;3],[f64;4]),
        ground:&BodyGroundState,navigation:&BodyNavigationSegment<'_,'_>)->Result<BodyGroundState,SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let support=self.support_position_from_body(from.0,from.1,model.body_to_support_z_m)?;
        Ok(BodyGroundState {sample:self.advance_navigation_ground([support[0],support[1]],
            &ground.sample,navigation.at(0.0))?})
    }

    pub(in crate::surface) fn owned_ground_motion_reference(&self,from:([f64;3],[f64;4]),
        to:([f64;3],[f64;4]),ground:&BodyGroundState)->Result<LocalGroundReference,SurfaceError> {
        self.ground_motion_reference_impl(from,to,ground,None)
    }

    pub(in crate::surface) fn navigation_ground_motion_reference(&self,from:([f64;3],[f64;4]),
        to:([f64;3],[f64;4]),ground:&BodyGroundState,navigation:&BodyNavigationSegment<'_,'_>)
        ->Result<LocalGroundReference,SurfaceError> {
        self.ground_motion_reference_impl(from,to,ground,Some(navigation))
    }

    fn ground_motion_reference_impl(&self,from:([f64;3],[f64;4]),to:([f64;3],[f64;4]),
        ground:&BodyGroundState,navigation:Option<&BodyNavigationSegment<'_,'_>>)
        ->Result<LocalGroundReference,SurfaceError> {
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let size=f64::from(self.observations.as_ref().map_or(self.config().voxel_size,|o|o.size_m));
        let mut fractions=self.body_support_grid_fractions(from,to,size)?;
        if let Some(navigation)=navigation {
            fractions.extend_from_slice(navigation.changes());
            fractions.sort_by(f64::total_cmp);fractions.dedup();
        }
        let mut reference=LocalGroundReference {first:ground.sample.clone(),changes:vec![]};
        let mut previous=ground.sample.clone();
        // Actual support XY follows the complete interpolated body quaternion,
        // including an offset shift during a turn. Never use body-centre XY.
        for t in fractions.into_iter().filter(|t|*t>0.0) {
            let pose=interpolated_body_pose(from,to,t);
            let support=self.support_position_from_body(pose.0,pose.1,model.body_to_support_z_m)?;
            let next=match navigation {
                Some(navigation)=>self.advance_navigation_ground([support[0],support[1]],&previous,navigation.at(t))?,
                None=>self.advance_owned_ground([support[0],support[1]],&previous),
            };
            // A semantic boundary must reach contact events even when its
            // resolved physical sample equals the prior one.
            if next!=previous || navigation.is_some_and(|n|n.changes().binary_search_by(|v|v.total_cmp(&t)).is_ok()) {
                reference.changes.push((t,next.clone()));
            }
            previous=next;
        }
        Ok(reference)
    }

    pub(in crate::surface) fn project_owned_ground_motion(&self,from:([f64;3],[f64;4]),
        ground:&BodyGroundState,xy:[f64;2],yaw:f64,witness:&mut Option<BodyFailureWitness>)
        ->Result<BodyMotionProjection,SurfaceError> {
        self.project_ground_motion_impl(from,ground,xy,yaw,None,witness)
    }

    pub(in crate::surface) fn project_navigation_ground_motion(&self,from:([f64;3],[f64;4]),
        ground:&BodyGroundState,xy:[f64;2],yaw:f64,navigation:&BodyNavigationSegment<'_,'_>,
        witness:&mut Option<BodyFailureWitness>)->Result<BodyMotionProjection,SurfaceError> {
        self.project_ground_motion_impl(from,ground,xy,yaw,Some(navigation),witness)
    }

    fn project_ground_motion_impl(&self,from:([f64;3],[f64;4]),ground:&BodyGroundState,
        xy:[f64;2],yaw:f64,navigation:Option<&BodyNavigationSegment<'_,'_>>,
        witness:&mut Option<BodyFailureWitness>)->Result<BodyMotionProjection,SurfaceError> {
        *witness=None;
        if !from.0.into_iter().chain(xy).chain([yaw]).all(f64::is_finite) {
            return Err(SurfaceError::InvalidPose);
        }
        self.validate_complete_source_resolution()?;
        let initial;
        let ground=if let Some(navigation)=navigation {
            initial=self.navigation_initial_support(from,ground,navigation)?;&initial
        } else {ground};
        self.validate_owned_actual_support(from,&ground.sample)?;
        let model=self.body_envelope().ok_or(SurfaceError::InvalidPose)?;
        let old_offset=rotated_support_offset(from.1,model.body_to_support_z_m)?;
        let stationary=(xy[0]-from.0[0]).hypot(xy[1]-from.0[1])<1e-9;
        // An in-place turn has no terrain displacement. Keep the actual world
        // body normal, including its measured bias relative to a legacy fit.
        // Rotating that bias around a fitted normal invents a support-XY shift.
        let mut q=if stationary {
            let normal=rotation(from.1)?[2];let (sine,cosine)=yaw.sin_cos();
            let forward=normalize([cosine*normal[2],sine*normal[2],-cosine*normal[0]-sine*normal[1]]);
            quaternion_from_columns([forward,cross(normal,forward),normal])
        } else {transport_attitude(from.1,ground.sample.reference_normal,None,yaw)?};
        let mut reference=self.ground_motion_reference_impl(from,([xy[0],xy[1],from.0[2]],q),ground,navigation)?;
        let next=reference.at(1.0);
        let transported=if stationary {q} else {
            transport_attitude(from.1,ground.sample.reference_normal,next.reference_normal,yaw)?};
        if transported!=q {
            q=transported;
            // The attitude response changed the physical support XY; select its
            // column again before using h. No plane-at-offset height correction.
            reference=self.ground_motion_reference_impl(from,([xy[0],xy[1],from.0[2]],q),ground,navigation)?;
        }
        let next=reference.at(1.0).clone();
        let offset=rotated_support_offset(q,model.body_to_support_z_m)?;
        let z=if stationary {from.0[2]} else {
            let known_height_delta=next.reference_origin[2]-ground.sample.reference_origin[2];
            // A segment may cross known terrain before reaching an unknown
            // column. Carry that last observed height, without adding a new
            // terrain or offset response inside the unknown portion.
            let offset_delta=if next.query_state==GroundQueryState::Unknown {0.0}
                else {old_offset[2]-offset[2]};
            from.0[2]+known_height_delta+offset_delta
        };
        let pose=([xy[0],xy[1],z],q);
        let reference=Arc::new(reference);
        let context=BodyQueryReference::Ground(Arc::clone(&reference));
        self.validate_known_terrain_motion_in_context(from,pose,QuerySegment::for_reference(&context))?;
        self.validate_body_pose_in_context(pose.0,pose.1,QuerySegment::for_reference(&context).at(1.0),witness)?;
        Ok(BodyMotionProjection {from,pose,ground:BodyGroundState {sample:next},reference})
    }
}
