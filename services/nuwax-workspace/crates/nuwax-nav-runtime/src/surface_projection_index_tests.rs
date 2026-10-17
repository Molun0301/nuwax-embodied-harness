// Index provenance regressions only. Existing geometry and control stay real.
mod projection_index_tests {
    use super::*;

    fn turn_then_translate_then_turn() -> Path3D {
        let half=std::f64::consts::FRAC_PI_2;
        Path3D { poses: vec![
            body_pose([1.05,1.55,0.4],Quaternion::IDENTITY,0),
            body_pose([1.05,1.55,0.4],Quaternion::from_yaw(half),0),
            body_pose([1.05,1.85,0.4],Quaternion::from_yaw(half),0),
            body_pose([1.05,1.85,0.4],Quaternion::from_yaw(std::f64::consts::PI),0),
            body_pose([0.75,1.85,0.4],Quaternion::from_yaw(std::f64::consts::PI),0),
        ] }
    }

    #[test]
    fn admission_tracks_projected_segment_without_skipping_later_yaw() {
        for at_endpoint in [false,true] {
            let path=turn_then_translate_then_turn();
            let (mut follower,_map)=omni_route_follower(path.poses.clone(),true);
            let mut current=path.poses[2].clone();
            if !at_endpoint {current.pose.position.y=1.75;}
            follower.update_odometry(current.clone());
            follower.observe_applied_command(Twist2::ZERO);
            follower.replace_execution_path(ExecutionPath::Surface(path.clone()),None,path.poses[4].clone()).unwrap();
            assert_eq!(follower.path_index,2,
                "PROJECTED_SEGMENT_REPLAYED: the selected incoming segment ends at B90, including t=1");
            assert_eq!(follower.path.as_ref().unwrap().poses,path.poses,
                "index repair must not rewrite the candidate or its remaining yaw sequence");
            assert_eq!(follower.goal.as_ref().unwrap(),&path.poses[4]);
            assert_eq!(follower.last_command,Twist2::ZERO);
            if at_endpoint {
                let step=follower.step(0.3);
                let PlannerStep::Command(command)=step else {panic!("remaining B180 turn must execute: {step:?}")};
                assert_eq!(command.linear,Point2::new(0.0,0.0));
                assert_ne!(command.angular_z,0.0,"B180 was skipped with equal cumulative length");
                assert_eq!(follower.path_index,3,"the later same-position turn remains unconsumed");
            }
        }
    }

    #[test]
    fn initial_projection_sample_keeps_its_positive_segment_owner() {
        let path=turn_then_translate_then_turn();
        let (mut follower,_map)=omni_route_follower(path.poses.clone(),true);
        // Nearest position is the window's initial sample, so no later strict
        // distance improvement replaces pose_at_distance's initial choice.
        let current=body_pose([1.03,1.55,0.4],Quaternion::IDENTITY,0);
        follower.update_odometry(current);
        follower.replace_execution_path(ExecutionPath::Surface(path.clone()),None,path.poses[4].clone()).unwrap();
        assert_eq!(follower.progress_m,0.0);
        assert_eq!(follower.path_index,2,
            "INITIAL_SAMPLE_LOST_SEGMENT_OWNER: initial interpolated sample belongs to A90->B90");
    }

    #[test]
    fn admission_keeps_first_point_and_degenerate_index_semantics() {
        let path=turn_then_translate_then_turn();
        let (mut follower,_map)=omni_route_follower(path.poses.clone(),true);
        assert_eq!(follower.path_index,1);
        for poses in [vec![path.poses[0].clone()],path.poses[..2].to_vec()] {
            let mut current=poses[0].clone();current.pose.position.x+=0.02;
            follower.update_odometry(current);
            let goal=poses.last().unwrap().clone();
            follower.replace_execution_path(ExecutionPath::Surface(Path3D {poses:poses.clone()}),None,goal).unwrap();
            assert_eq!(follower.progress_m,0.0);
            assert_eq!(follower.path_index,1,"no positive segment: retain original single/rotation behavior");
            assert_eq!(follower.path.as_ref().unwrap().poses,poses);
        }
    }

    #[test]
    fn moving_lateral_replacement_preserves_applied_and_forward_feedback() {
        // Reuse the existing calibrated forward-feedback unit fixture and its
        // unchanged 20 Hz model. This is not the default-10-Hz owner benchmark.
        let poses:Vec<_>=(0..=30).map(|i|
            body_pose([1.05+0.05*i as f64,1.55,0.4],Quaternion::IDENTITY,0)).collect();
        let path=Path3D {poses:poses.clone()};
        let (mut follower,map)=omni_route_follower(poses,true);
        let mut current=path.poses[4].clone();current.pose.position.y+=0.01;
        let mut applied=Twist2 {linear:Point2::new(0.3,0.0),angular_z:0.0};
        follower.update_odometry(current.clone());follower.observe_applied_command(applied);
        follower.replace_execution_path(ExecutionPath::Surface(path.clone()),None,path.poses.last().unwrap().clone()).unwrap();
        assert_eq!(follower.last_command,applied,"replacement must not counterfeit zero applied velocity");
        assert!(follower.braking_deadline.is_none() && follower.simulation_braking_deadline_ns.is_none());
        assert_eq!(follower.path.as_ref().unwrap().poses,path.poses,
            "no exact-vertex connector may manufacture an extra stop-turn sequence");
        let initial=current.pose.position;
        for tick in 0..4 {
            current.stamp.nanos=1_000_000_000+tick*50_000_000;
            follower.update_odometry(current.clone());follower.observe_applied_command(applied);
            let mut step=follower.step(0.3);
            if let PlannerStep::Command(command)=step {
                if let Some(rechecked)=follower.revalidate_selected_command_at_current_state(command) {step=rechecked;}
            }
            let PlannerStep::Command(command)=step else {panic!("compatible moving replacement must keep tracking: {step:?}")};
            assert!(command.linear.x>0.0,"artificial stop at ordinary lateral tracking error: {command:?}");
            assert_eq!(command.linear.y,0.0);
            apply_forward_feedback(&map,&mut current,&mut applied,command);
        }
        assert!(current.pose.position.x>initial.x,"calibrated feedback must actually advance");
    }
}
