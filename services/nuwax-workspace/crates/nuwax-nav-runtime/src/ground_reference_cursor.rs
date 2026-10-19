//! Internal geometric consumption seam. Planned samples never establish an
//! actual ground-layer match or final navigation success.
use super::*;
use nuwax_nav_scan::GroundCandidateBinding;
use crate::replan_feedback::GroundActualBasis;
use std::sync::atomic::{AtomicBool,Ordering};

#[derive(Clone,Copy,Debug,PartialEq)]
pub(crate) struct GroundCursor {pub(crate) edge:usize,pub(crate) fraction:f64}
#[derive(Clone)]
pub(crate) struct GroundResume {binding:Arc<GroundCandidateBinding>,cursor:GroundCursor,actual:PoseStamped}
pub(super) struct GroundExecution {
    binding:Arc<GroundCandidateBinding>,cursor:GroundCursor,last_actual:PoseStamped,xy_cumulative:Vec<f64>,
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) enum GroundConsumption {Unchanged,Advanced,LocalReferenceEnd {covers_original_goal:bool}}
/// A predicted command is not arrival. The internal caller must handle the
/// neutral local end separately until typed actual-layer/stop evidence exists.
pub(crate) enum GroundControlStep {
    Control(PlannerStep),
    LocalReferenceEnd { covers_original_goal: bool },
}
struct GroundActualProposal {
    basis: GroundActualBasis,
    cursor: GroundCursor,
    path_index: usize,
    progress_m: f64,
    actual: PoseStamped,
    outcome: GroundConsumption,
}
fn checkpoint(cancelled:Option<&AtomicBool>)->Result<(),String> {
    if cancelled.is_some_and(|flag|flag.load(Ordering::Relaxed)) {Err("ground consumption cancelled".into())}else{Ok(())}
}
fn pose_at(path:&Path3D,cursor:GroundCursor)->PoseStamped {
    if path.poses.len()==1 {return path.poses[0].clone();}
    let a=&path.poses[cursor.edge];let b=&path.poses[cursor.edge+1];
    // Reuse the original path interpolation, without reselection of a segment.
    super::interpolate_pose(a,b,cursor.fraction)
}
impl GroundExecution {
    pub(super) fn matches_source(&self,source:&SurfaceSnapshot)->bool {self.binding.candidate().matches_source(source)}
    fn basis_at(&self,cursor:GroundCursor,actual:&PoseStamped)->GroundActualBasis {
        GroundActualBasis::new(Arc::clone(&self.binding),cursor.edge,cursor.fraction,
            [actual.pose.position.x,actual.pose.position.y])
    }
    pub(super) fn basis(&self)->GroundActualBasis {self.basis_at(self.cursor,&self.last_actual)}
}
impl SurfaceFollower {
    /// An explicit new candidate starts at its own first actual pose. A retained
    /// resume token is owner-coupled; there is no nearest-route reattachment.
    pub(crate) fn admit_ground_candidate(&mut self,path:Path3D,goal:PoseStamped,
        binding:Arc<GroundCandidateBinding>,resume:Option<GroundResume>,cancelled:Option<&AtomicBool>)
        ->Result<(),String> {
        checkpoint(cancelled)?;
        self.refresh_current_body_view();
        if path.poses.len()!=binding.edges().len()+1 {
            return Err("candidate geometry and reference edges differ".into());
        }
        if binding.edges().is_empty() && binding.stationary_pose()!=Some(body_control::body(&path.poses[0])) {
            return Err("stationary candidate binding belongs to a different actual pose".into());
        }
        crate::validate_map_pose(&goal,"ground candidate goal").map_err(|e|e.to_string())?;
        for pose in &path.poses {crate::validate_map_pose(pose,"ground candidate path").map_err(|e|e.to_string())?;}
        for (edge,pair) in path.poses.windows(2).enumerate() {
            if [body_control::body(&pair[0]),body_control::body(&pair[1])]!=binding.edges()[edge].motion() {
                return Err("candidate binding belongs to different motion geometry".into());
            }
        }
        let cursor=match resume.as_ref() {
            Some(resume) if Arc::ptr_eq(&binding,&resume.binding)
                && self.ground_execution.as_ref().is_some_and(|current|
                    Arc::ptr_eq(&current.binding,&resume.binding) && current.cursor==resume.cursor
                        && current.last_actual.pose==resume.actual.pose)=>resume.cursor,
            Some(_)=>return Err("resume is not the current consumed candidate address".into()),
            None=>GroundCursor {edge:0,fraction:0.0},
        };
        let map=self.source_map.as_ref().ok_or("ground candidate has no current source")?;
        if !binding.candidate().matches_source(map) {return Err("ground candidate source retired".into());}
        let actual=self.odometry.as_ref().ok_or("ground candidate has no actual pose")?;
        crate::validate_map_pose(actual,"ground candidate actual").map_err(|e|e.to_string())?;
        // A new candidate owns its exact first full pose. Even ratio-zero
        // interpolation normalizes the quaternion and may change its bits.
        let selected=resume.as_ref().map_or_else(||path.poses[0].clone(),|resume|resume.actual.clone());
        // A nonzero connector must be a real candidate edge. Admission cannot
        // consume an unexecuted bridge between actual and an old route address.
        if actual.pose!=selected.pose {return Err("resume requires an explicit unconsumed connector".into());}
        let (_,ground)=self.current_ground_query_owned()?;
        let navigation=binding.candidate().walk_state_at(cursor.edge,cursor.fraction,
            [actual.pose.position.x,actual.pose.position.y]).ok_or("invalid candidate address")?;
        let query=navigation.bind_body_query(map).map_err(|e|e.to_string())?;
        body_control::validate_control_actual_pose(&query,body_control::body(actual),&ground,
            Some(navigation),&self.boundary)?;
        let cumulative=cumulative_lengths(&path);
        let mut xy_cumulative=Vec::with_capacity(path.poses.len());xy_cumulative.push(0.0);
        for pair in path.poses.windows(2) {
            xy_cumulative.push(xy_cumulative.last().unwrap()+(pair[1].pose.position.x-pair[0].pose.position.x)
                .hypot(pair[1].pose.position.y-pair[0].pose.position.y));
        }
        checkpoint(cancelled)?;
        // A retained resume already owns the actual target-switching state.
        // Its waypoint target may be ahead of the still-partial geometric edge.
        if resume.is_none() {
            self.progress_m=0.0;
            self.path_index=1;
        }
        self.alignment_target_index=None;
        self.selected_body_command=None;
        self.last_tracked_pose=Some(actual.clone());
        self.ground_execution=Some(GroundExecution {binding,cursor,last_actual:actual.clone(),xy_cumulative});
        self.path=Some(path);self.goal=Some(goal);self.cumulative=cumulative;
        self.invalid_input=None;self.controller.reset_errors();
        Ok(())
    }
    pub(crate) fn ground_resume(&self)->Option<GroundResume> {
        self.ground_execution.as_ref().map(|execution|GroundResume {
            binding:Arc::clone(&execution.binding),cursor:execution.cursor,actual:execution.last_actual.clone()})
    }
    pub(crate) fn ground_cursor(&self)->Option<GroundCursor> {self.ground_execution.as_ref().map(|e|e.cursor)}
    pub(crate) fn committed_ground_basis(&self)->Option<GroundActualBasis> {
        self.ground_execution.as_ref().map(GroundExecution::basis)
    }
    pub(super) fn owns_ground_basis(&self,basis:&GroundActualBasis)->bool {
        self.ground_execution.as_ref().zip(self.source_map.as_ref()).is_some_and(|(execution,source)|
            basis.matches_binding(&execution.binding) && execution.matches_source(source))
    }
    /// Read the current actual observation through the same bounded adjacent
    /// proposal as consumption. No prediction or request capture commits it.
    pub(super) fn proposed_ground_basis(&self)->Result<Option<GroundActualBasis>,String> {
        if self.ground_execution.is_none() {return Ok(None);}
        self.propose_ground_actual(None).map(|proposal|Some(proposal.basis))
    }
    fn propose_ground_actual(&self,cancelled:Option<&AtomicBool>)->Result<GroundActualProposal,String> {
        checkpoint(cancelled)?;
        if let Some(error)=&self.invalid_input {return Err(error.clone());}
        let execution=self.ground_execution.as_ref().ok_or("no ground candidate")?;
        let map=self.source_map.as_ref().ok_or("no current source")?;
        if !execution.matches_source(map) {return Err("ground candidate source retired".into());}
        let actual=self.odometry.as_ref().ok_or("no actual pose")?;
        crate::validate_map_pose(actual,"ground actual observation").map_err(|e|e.to_string())?;
        let path=self.path.as_ref().ok_or("no candidate path")?;
        let goal=self.goal.as_ref().ok_or("no candidate goal")?;
        let (_,ground)=self.current_ground_query_owned()?;
        if actual.pose==execution.last_actual.pose {
            let basis=execution.basis_at(execution.cursor,actual);
            let navigation=basis.walk_state()?;
            let query=navigation.bind_body_query(map).map_err(|e|e.to_string())?;
            body_control::validate_control_actual_pose(&query,body_control::body(actual),&ground,
                Some(navigation),&self.boundary)?;
            return Ok(GroundActualProposal {basis,cursor:execution.cursor,path_index:self.path_index,
                progress_m:self.progress_m,actual:actual.clone(),outcome:GroundConsumption::Unchanged});
        }
        if execution.binding.edges().is_empty() {
            return Err("motion beyond a stationary candidate requires a real connector".into());
        }
        let displacement=(execution.last_actual.pose.position.x-actual.pose.position.x)
            .hypot(execution.last_actual.pose.position.y-actual.pose.position.y);
        let old=execution.cursor;
        let xy_cumulative=&execution.xy_cumulative;
        let old_progress=xy_cumulative[old.edge]+old.fraction*(xy_cumulative[old.edge+1]-xy_cumulative[old.edge]);
        let limit=old_progress+displacement+if displacement>0.0 {self.config.controller.goal_tolerance}else{0.0};
        let mut cursor=old;
        // Directed adjacency only. A later nearby return leg is never examined
        // until the incoming edge's actual projection reaches its endpoint.
        loop {
            checkpoint(cancelled)?;
            let a=&path.poses[cursor.edge];let b=&path.poses[cursor.edge+1];
            let dx=b.pose.position.x-a.pose.position.x;let dy=b.pose.position.y-a.pose.position.y;
            let length2=dx*dx+dy*dy;
            let t=if length2>0.0 {
                (((actual.pose.position.x-a.pose.position.x)*dx+(actual.pose.position.y-a.pose.position.y)*dy)/length2)
                    .clamp(cursor.fraction,1.0)
            } else {
                let angle=angle_diff(b.pose.orientation.yaw(),a.pose.orientation.yaw());
                if angle!=0.0 {(angle_diff(actual.pose.orientation.yaw(),a.pose.orientation.yaw())/angle).clamp(cursor.fraction,1.0)}
                else if execution.binding.candidate().constant_association(cursor.edge) {1.0}else{cursor.fraction}
            };
            let length=xy_cumulative[cursor.edge+1]-xy_cumulative[cursor.edge];
            cursor.fraction=if length>0.0 {t.min(((limit-xy_cumulative[cursor.edge])/length).clamp(cursor.fraction,1.0))}else{t};
            let projected=pose_at(path,cursor);
            if (projected.pose.position.x-actual.pose.position.x).hypot(projected.pose.position.y-actual.pose.position.y)
                >self.config.max_path_deviation_m {
                return Err("actual pose left its adjacent candidate segment".into());
            }
            if cursor.fraction<1.0 || cursor.edge+1==execution.binding.edges().len() {break;}
            cursor=GroundCursor {edge:cursor.edge+1,fraction:0.0};
        }
        let mut adjacent=body_control::AdjacentWaypointProgress {path_index:self.path_index,progress_m:self.progress_m};
        // Ground consumption is XY-geometric advice, not a claimed actual layer.
        // The original helper still performs the same real connector/turn checks.
        let provisional=execution.basis_at(cursor,actual);
        let navigation=provisional.walk_state()?;
        let query=navigation.bind_body_query(map).map_err(|e|e.to_string())?;
        body_control::advance_adjacent_waypoints_with_navigation::<true,true>(&self.config,map,&query,&ground,
            Some(navigation),path,goal,actual,
            &self.cumulative,&self.options,true,&mut adjacent,|_|true,|index,event| {
                checkpoint(cancelled)?;
                let edge=index-1;
                Ok(edge<=cursor.edge && match event {
                    body_control::AdjacentWaypointAdvance::Duplicate=>execution.binding.candidate().constant_association(edge),
                    body_control::AdjacentWaypointAdvance::Reached {..}=>true,
                })
            })?;
        // Control target switching at a curve midpoint is not full edge travel.
        // Conversely an unresolved adjacent turn cannot be skipped by projection.
        if cursor.edge>=adjacent.path_index {
            cursor=GroundCursor {edge:adjacent.path_index-1,fraction:1.0};
        }
        let basis=execution.basis_at(cursor,actual);
        body_control::validate_control_actual_pose(&query,body_control::body(actual),&ground,
            Some(basis.walk_state()?),&self.boundary)?;
        let changed=cursor!=old;
        let at_end=cursor.edge+1==execution.binding.edges().len() && cursor.fraction==1.0;
        let covers=execution.binding.covers_original_goal();
        let outcome=if at_end {GroundConsumption::LocalReferenceEnd {covers_original_goal:covers}}
            else if changed {GroundConsumption::Advanced}else{GroundConsumption::Unchanged};
        Ok(GroundActualProposal {basis,cursor,path_index:adjacent.path_index,
            progress_m:self.cumulative[cursor.edge]+cursor.fraction*(self.cumulative[cursor.edge+1]-self.cumulative[cursor.edge]),
            actual:actual.clone(),outcome})
    }
    /// Only actual observations commit. The final checkpoint owns the sole write
    /// of cursor/last_actual/progress; a failed proposal leaves all four intact.
    pub(crate) fn consume_ground_actual(&mut self,cancelled:Option<&AtomicBool>)->Result<GroundConsumption,String> {
        checkpoint(cancelled)?;
        self.refresh_current_body_view();
        let proposal=self.propose_ground_actual(cancelled)?;
        checkpoint(cancelled)?;
        self.path_index=proposal.path_index;self.progress_m=proposal.progress_m;
        self.last_tracked_pose=Some(proposal.actual.clone());
        let execution=self.ground_execution.as_mut().expect("proposal retains its current owner");
        execution.cursor=proposal.cursor;execution.last_actual=proposal.actual;
        Ok(proposal.outcome)
    }
    /// Explicit finite-window control seam. Normal RouteFollower admission and
    /// public step do not invoke this until rolling connector/suffix is wired.
    pub(crate) fn step_ground_internal(&mut self,robot_width_m:f64,cancelled:Option<&AtomicBool>)
        ->Result<GroundControlStep,String> {
        self.last_command_dispatch_bound=None;
        self.consume_ground_actual(cancelled)?;
        let execution=self.ground_execution.as_ref().ok_or("no ground candidate")?;
        if execution.binding.edges().is_empty() ||
            execution.cursor.edge+1==execution.binding.edges().len() && execution.cursor.fraction==1.0 {
            return Ok(GroundControlStep::LocalReferenceEnd {covers_original_goal:execution.binding.covers_original_goal()});
        }
        let basis=execution.basis();
        Ok(GroundControlStep::Control(self.follow_ground_control_step(robot_width_m,basis)))
    }

}

#[cfg(test)]
mod tests {
    use super::*;
    use nuwax_nav_types_messages::{NavFrameId,Pose3,Timestamp};
    fn stamped(p:([f64;3],[f64;4]),tick:i64)->PoseStamped {
        PoseStamped {frame_id:NavFrameId::Map,stamp:Timestamp::from_unix(1,(tick*50_000_000) as u32),
            pose:Pose3 {position:Point3::new(p.0[0],p.0[1],p.0[2]),
                orientation:Quaternion {x:p.1[0],y:p.1[1],z:p.1[2],w:p.1[3]}}}
    }
    fn fixture(final_yaw:Option<f64>)->(SurfaceFollower,Arc<GroundCandidateBinding>,Path3D,PoseStamped) {
        use nuwax_nav_pct::ground_route::{GroundRouteStart,GroundRouteGoal};
        let source=Arc::new(super::super::test_snapshot(0.0,false).with_body_envelope(
            nuwax_nav_mls_planner::SurfaceBodyEnvelope {fixed_collision:None,
            footprint_xy_m:vec![[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],z_bounds_m:[-0.05,0.2],
            body_to_support_z_m:0.3,max_step_up_m:0.2,max_step_down_m:0.2,max_cross_slope_deg:20.,support_uncertainty_m:0.}).unwrap());
        let start=([1.05,1.05,0.4],[0.,0.,0.,1.]);let to=[1.55,1.05];
        let route=nuwax_nav_pct::PctPlanner::default().ground_route_with_limits_and_penalties(&source,start,
            GroundRouteStart {xy:[start.0[0],start.0[1]],ground_z:None},GroundRouteGoal {xy:to,requested_ground_z:None},
            0.0,&[],None,None,nuwax_nav_pct::SearchLimits::default(),&[]).unwrap();
        let tomo=nuwax_nav_pct::Tomogram::from_surface(&source).unwrap();
        let caps=MotionKinematics {model:MotionModel::Differential,max_forward_mps:1.,max_reverse_mps:0.,
            max_lateral_mps:0.,max_angular_rps:0.8,minimum_turning_radius_m:0.,can_rotate_in_place:true};
        let actual_ground=source.body_query().start_ground_state(start).unwrap();
        let mapped=nuwax_nav_scan::plan_mapped_ground(&source,&tomo,start,&actual_ground,
            &[[1.05,1.05,0.1],[1.55,1.05,0.1]],&caps,&nuwax_nav_scan::ScanConfig::default(),final_yaw,
            None,&[],false,false,None,route.reference.full_window().unwrap()).unwrap();
        let binding=Arc::new(mapped.binding);
        let path=Path3D {poses:mapped.trajectory.poses.into_iter().map(|p|stamped(p,0)).collect()};
        let goal=path.poses.last().unwrap().clone();
        let mut config=SurfaceFollowerConfig::new(0.3);config.controller.simulation=true;config.basic_kinematics=Some(caps);
        let mut owner=SurfaceFollower::new(config).unwrap();
        owner.update_execution_map(ExecutionMap::Surface(source)).unwrap();owner.update_odometry(stamped(start,0));
        owner.admit_ground_candidate(path.clone(),goal.clone(),binding.clone(),None,None).unwrap();
        (owner,binding,path,goal)
    }
    #[test]
    fn c1b_05_actual_adjacent_cursor_never_consumes_planned_coverage() {
        let (mut owner,binding,_,_)=fixture(None);
        assert!(binding.planned_join());assert_eq!(owner.ground_cursor(),Some(GroundCursor {edge:0,fraction:0.}));
        assert_eq!(owner.consume_ground_actual(None).unwrap(),GroundConsumption::Unchanged);
        owner.update_odometry(stamped(([1.14,1.05,0.4],[0.,0.,0.,1.]),1));
        assert_eq!(owner.consume_ground_actual(None).unwrap(),GroundConsumption::Advanced);
        assert!((owner.progress_m()-0.09).abs()<1e-9);
        let cursor=owner.ground_cursor();assert!(cursor.unwrap().fraction>0. && cursor.unwrap().fraction<1.);
        for _ in 0..3 {assert_eq!(owner.consume_ground_actual(None).unwrap(),GroundConsumption::Unchanged);assert_eq!(owner.ground_cursor(),cursor);}
        assert!(owner.path_index>cursor.unwrap().edge,"control target switch is separate from actual edge fraction");
        let mut yaw_only=owner.odometry.as_ref().unwrap().clone();yaw_only.pose.orientation=Quaternion::from_yaw(0.01);
        owner.update_odometry(yaw_only);owner.consume_ground_actual(None).unwrap();assert_eq!(owner.ground_cursor(),cursor);
        let mut height_only=owner.odometry.as_ref().unwrap().clone();height_only.pose.position.z+=0.001;
        owner.update_odometry(height_only);owner.consume_ground_actual(None).unwrap();assert_eq!(owner.ground_cursor(),cursor);
    }
    #[test]
    fn c1b_06_explicit_midsegment_resume_and_unconsumed_connector() {
        let (mut owner,binding,path,goal)=fixture(None);
        assert!(path.poses[0].pose.position.distance(path.poses[1].pose.position)>0.);
        assert_eq!(owner.progress_m(),0.0,"nonzero first connector is not admitted as already walked");
        let half=pose_at(&path,GroundCursor {edge:0,fraction:0.5});owner.update_odometry(half);
        owner.consume_ground_actual(None).unwrap();let token=owner.ground_resume().unwrap();let before=owner.ground_cursor();
        owner.admit_ground_candidate(path.clone(),goal.clone(),binding.clone(),Some(token.clone()),None).unwrap();
        assert_eq!(owner.ground_cursor(),before);
        let different_owner=Arc::new(binding.as_ref().clone());
        assert!(owner.admit_ground_candidate(path,goal,different_owner,Some(token),None).is_err());
        assert_eq!(owner.ground_cursor(),before);
    }
    #[test]
    fn c1b_07_failed_cancelled_and_retired_inputs_do_not_commit() {
        let (mut owner,binding,path,goal)=fixture(None);let before=owner.ground_cursor();
        owner.update_odometry(stamped(([1.10,1.05,0.4],[0.,0.,0.,1.]),1));
        let cancel=AtomicBool::new(true);assert!(owner.consume_ground_actual(Some(&cancel)).is_err());assert_eq!(owner.ground_cursor(),before);
        let clear=owner.source_map.as_ref().unwrap().clone();
        let blocked=Arc::new(clear.with_recent_obstacle_points(1,2_000_000_000,0,&[],[(0,[1.10,1.05,0.4])]).unwrap());
        owner.update_execution_map(ExecutionMap::Surface(blocked)).unwrap();
        assert!(owner.consume_ground_actual(None).is_err());assert_eq!(owner.ground_cursor(),before);
        let changed=Arc::new(clear.with_body_envelope(nuwax_nav_mls_planner::SurfaceBodyEnvelope {
            fixed_collision:None,footprint_xy_m:vec![[-0.11,-0.1],[0.11,-0.1],[0.11,0.1],[-0.11,0.1]],
            z_bounds_m:[-0.05,0.2],body_to_support_z_m:0.3,max_step_up_m:0.2,max_step_down_m:0.2,
            max_cross_slope_deg:20.,support_uncertainty_m:0.}).unwrap());
        owner.update_execution_map(ExecutionMap::Surface(changed)).unwrap();assert!(owner.ground_cursor().is_none());
        assert!(owner.admit_ground_candidate(path,goal,binding,None,None).is_err());
    }
    #[test]
    fn c1b_08_terminal_geometric_cursor_is_not_arrival_or_layer_match() {
        let (mut owner,binding,path,_)=fixture(Some(0.2));
        let same_xy=path.poses.windows(2).filter(|p|p[0].pose.position==p[1].pose.position).count();
        assert!(same_xy>0,"fixed final yaw creates real same-XY samples");
        let mut last=GroundConsumption::Unchanged;
        for (i,p) in path.poses.iter().enumerate().skip(1) {
            let mut actual=p.clone();actual.stamp=Timestamp::from_unix(1,(i*50_000_000) as u32);
            owner.update_odometry(actual);last=owner.consume_ground_actual(None).unwrap();
        }
        assert_eq!(last,GroundConsumption::LocalReferenceEnd {covers_original_goal:binding.covers_original_goal()});
        let c=owner.ground_cursor().unwrap();assert_eq!(c.edge+1,binding.edges().len());assert_eq!(c.fraction,1.);
        assert_eq!(owner.consume_ground_actual(None).unwrap(),GroundConsumption::Unchanged);
        // This enum contains no Matched/Succeeded variant. The actual source
        // association and post-stop arrival contract are deliberately separate.
        owner.stop();assert!(owner.ground_cursor().is_none());
    }
}
