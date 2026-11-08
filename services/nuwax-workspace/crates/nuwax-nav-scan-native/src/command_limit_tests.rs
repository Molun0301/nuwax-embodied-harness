// 命令盒与物理响应是两种约束；不能把命令的硬投影冒充实测瞬间降速。
fn device_limit_config(req: &mut PlanRequest<'_>) {
    req.config.sample_dt = 0.05;
    req.config.max_yaw_rate = 0.2;
    req.config.max_yaw_acceleration = 0.5;
    req.config.max_velocity = 0.5;
    req.config.max_reverse_velocity = 0.5;
    req.config.max_lateral_velocity = 0.3;
    req.config.max_acceleration = 0.5;
}

fn assert_device_box(result: &PlanResult, config: &Config) {
    assert!(result.command_forward >= -config.max_reverse_velocity
        && result.command_forward <= config.max_velocity);
    assert!(result.command_lateral.abs() <= config.max_lateral_velocity);
    assert!(result.command_yaw_rate.abs() <= config.max_yaw_rate,
        "设备命令盒不允许借规划加减速越界：{result:?}");
}

fn drive_with_initial_overspeed(planner: &mut Planner, req: &mut PlanRequest<'_>, escape: bool) {
    let start = req.start_position;
    let mut moving_with_curve = false;
    let mut overspeed_observed = false;
    for tick in 0..60 {
        req.odometry_timestamp_ns += 50_000_000;
        let old_w = req.start_yaw_rate.unwrap();
        let old_world = req.start_velocity.unwrap();
        let yaw = req.start_yaw;
        let old = [yaw.cos()*old_world[0]+yaw.sin()*old_world[1],
            -yaw.sin()*old_world[0]+yaw.cos()*old_world[1]];
        let result = planner.step(req).unwrap();
        assert_device_box(&result, &req.config);
        assert_eq!(result.motion_boundary_sources[2], MotionBoundarySource::Measured);
        if tick == 0 {
            assert_eq!(result.command_yaw_rate,old_w.signum()*req.config.max_yaw_rate,
                "首拍应选择与软限候选最近的合法命令盒端点");
            assert!((result.command_yaw_rate-old_w).abs()
                > req.config.max_yaw_acceleration*req.config.sample_dt,
                "实测超限时必须优先恢复合法命令域");
            if escape {
                assert_eq!(planner.plan_count(), 0, "脱困夹具首拍不应已有正常轨迹");
                assert!(result.samples.len() >= 2 && result.command_forward > 0.);
                assert_eq!(result.samples[0].yaw_rate, old_w, "退出证明保留真实超限角速");
            }
        }
        moving_with_curve |= !result.samples.is_empty()
            && result.command_forward.hypot(result.command_lateral) > 1e-6;
        let dt = result.command_duration;
        let delta = [result.command_forward-old[0], result.command_lateral-old[1]];
        let scale = (req.config.max_acceleration*dt/delta[0].hypot(delta[1])
            .max(f64::MIN_POSITIVE)).min(1.);
        let next = [old[0]+scale*delta[0], old[1]+scale*delta[1]];
        let next_w = old_w+(result.command_yaw_rate-old_w).clamp(
            -req.config.max_yaw_acceleration*dt, req.config.max_yaw_acceleration*dt);
        // 独立执行器继续按真实加减速响应；首拍实际仍超限，绝非直接采用限幅值。
        if tick == 0 { assert!(next_w.abs() > req.config.max_yaw_rate); }
        overspeed_observed |= next_w.abs() > req.config.max_yaw_rate;
        let mean = [(old[0]+next[0])*0.5, (old[1]+next[1])*0.5];
        let w = (old_w+next_w)*0.5;
        let (a,b) = if w.abs()>1e-9 {((w*dt).sin()/w,(1.-(w*dt).cos())/w)} else {(dt,0.)};
        let dx = a*mean[0]-b*mean[1];let dy=b*mean[0]+a*mean[1];
        req.start_position[0] += yaw.cos()*dx-yaw.sin()*dy;
        req.start_position[1] += yaw.sin()*dx+yaw.cos()*dy;
        req.start_yaw += w*dt;
        req.start_yaw_rate = Some(next_w);
        req.start_velocity = Some([next[0]*req.start_yaw.cos()-next[1]*req.start_yaw.sin(),
            next[0]*req.start_yaw.sin()+next[1]*req.start_yaw.cos(),0.]);
        req.start_acceleration = None;
    }
    assert!(overspeed_observed && moving_with_curve);
    assert!((req.start_position[0]-start[0]).hypot(req.start_position[1]-start[1])>0.02,
        "合法命令仍须产生实际平移进展，不能以持续零速通过");
    if escape { assert!(planner.plan_count()>0, "退出后应继续原正常规划"); }
}

#[test]
fn device_limits_keep_real_positive_and_negative_yaw_overspeed_and_move() {
    let route = [[0.,0.,0.35],[2.,0.,0.35]];
    for (rate,acceleration) in [(0.35,0.5),(0.41123045284585513,4.)] {
      for sign in [-1.,1.] {
        let mut req=request(&route,&[]);device_limit_config(&mut req);
        req.config.max_yaw_acceleration=acceleration;
        req.start_velocity=Some([0.1,0.,0.]);req.start_yaw_rate=Some(sign*rate);
        drive_with_initial_overspeed(&mut fixture_planner(),&mut req,false);
      }
    }
}

#[test]
fn device_limits_cover_both_linear_axes_when_measured_state_exceeds_them() {
    let route = [[0.,0.,0.35],[2.,0.,0.35]];
    for sign in [-1.,1.] {
        let mut req=request(&[],&[]);device_limit_config(&mut req);
        req.config.max_velocity=1.;req.config.max_reverse_velocity=0.5;
        req.start_velocity=Some([sign*1.2,sign*0.5,0.]);req.start_yaw_rate=Some(sign*0.35);
        let result=fixture_planner().step(&req).unwrap();
        assert_device_box(&result,&req.config);
        assert!(result.command_forward.abs()>0. && result.command_lateral.abs()>0.);
        // 首拍计划尚未初始化，计划来源标志仍可为初始值；此处验证真实命令确实在
        // 无曲线时根据原超限测速恢复到最近的轴界，而非把测速丢弃后给零命令。
        assert!(result.samples.is_empty());
        assert_eq!(result.command_forward,if sign>0. {req.config.max_velocity} else {-req.config.max_reverse_velocity});
        assert_eq!(result.command_lateral,sign*req.config.max_lateral_velocity);
    }
}

#[test]
fn device_limits_do_not_clip_actual_yaw_before_the_collision_proof() {
    let obstacles=[[0.275,0.225,0.35]];
    let mut req=request(&[],&obstacles);device_limit_config(&mut req);
    req.config.resolution=0.05;req.config.body_length=0.6;req.config.body_width=0.36;
    req.goal_yaw=Some(0.2);req.start_yaw_rate=Some(0.2);
    assert!(fixture_planner().step(&req).is_ok(),"较低实际角速的停车扫掠应有净空");
    req.start_yaw_rate=Some(0.35);
    let error=fixture_planner().step(&req).unwrap_err();
    assert!(error.contains("braking sweep"),"真实超限角速的碰撞不能被裁成低速后放行：{error}");
}

#[test]
fn device_limits_escape_with_real_overspeed_then_continue_normal_planning() {
    let route=[[0.045,0.025,0.35],[1.,0.025,0.35]];
    let obstacles=[[-0.025,0.025,0.35]];
    let mut req=request(&route,&obstacles);device_limit_config(&mut req);
    req.config.resolution=0.05;req.config.body_length=0.1;req.config.body_width=0.1;
    req.config.max_velocity=0.2;req.config.max_acceleration=0.1;
    req.start_position=route[0];req.start_velocity=Some([0.075,0.,0.]);req.start_yaw_rate=Some(0.35);
    drive_with_initial_overspeed(&mut fixture_planner(),&mut req,true);
}
