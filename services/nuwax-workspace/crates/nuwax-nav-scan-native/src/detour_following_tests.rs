// 全局图已知墙体、局部概率图暂未观测墙体时，完整参考仍必须决定绕行方向。
#[test]
fn global_detour_is_not_replaced_by_a_straight_local_curve() {
    check_detour_curve(5.);
}

#[test]
fn local_detour_window_keeps_the_global_corner() {
    check_detour_curve(3.);
}

fn check_detour_curve(horizon: f64) {
    let route = [[0., 0., 0.35], [2.4, 0., 0.35], [2.4, -1.7, 0.35]];
    let mut req = request(&route, &[]);
    req.config.planning_horizon = horizon;
    let mut planner = fixture_planner();
    let mut curve = None;
    for tick in 0..20 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let result = planner.step(&req).unwrap();
        if !result.samples.is_empty() {
            curve = Some(result);
            break;
        }
    }
    let curve = curve.expect("原局部求解器必须生成绕行轨迹，不能以永久等待代替跟随");
    assert!(curve.samples.len() > 2);
    let mut yaw = 0.;
    for sample in &curve.samples {
        if sample.velocity[0].hypot(sample.velocity[1]) > 1e-8 {
            yaw = sample.velocity[1].atan2(sample.velocity[0]);
        }
        // 独立矩形分离轴判断：全局墙体为[0.8,1.8]×[-1.1,-0.3]，
        // 原参考的两个线段与完整机身均有净空，首尾连线则穿墙。
        let (cs, sn) = (yaw.cos(), yaw.sin());
        let dx = 1.3 - sample.position[0];
        let dy = -0.7 - sample.position[1];
        let overlaps = dx.abs() < 0.35 * cs.abs() + 0.15 * sn.abs() + 0.5
            && dy.abs() < 0.35 * sn.abs() + 0.15 * cs.abs() + 0.4
            && (dx * cs + dy * sn).abs() < 0.35 + 0.5 * cs.abs() + 0.4 * sn.abs()
            && (-dx * sn + dy * cs).abs() < 0.15 + 0.5 * sn.abs() + 0.4 * cs.abs();
        assert!(!overlaps, "局部曲线切过全局参考绕开的已知墙体：{sample:?}");
    }
    let end = curve.samples.last().unwrap().position;
    assert!(end.into_iter().zip(curve.local_target).all(|(a,b)|(a-b).abs()<1e-6));
    if horizon == 5. {
        assert!((end[0] - 2.4).hypot(end[1] + 1.7) < 1e-6);
    }
}

#[test]
fn replanning_starts_at_the_measured_body_when_execution_lags() {
    let route = [[0., 0., 0.35], [4., 0., 0.35]];
    let updated_route = [[0., 0., 0.35], [4.1, 0., 0.35]];
    let mut req = request(&route, &[]);
    let mut planner = fixture_planner();
    let mut plans = 0;
    // 真实反馈保持静止，不能将已发布命令冒充已完成位移；连续推进原调度时刻。
    for tick in 0..80 {
        // 新全局参考强制执行中重规划；静止反馈不能由时间流逝伪装成已行走一米。
        if tick == 20 { req.reference = &updated_route; }
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let result = planner.step(&req).unwrap();
        if result.replanned && !result.samples.is_empty() {
            plans += 1;
            let p = result.samples[0].position;
            assert!(p[0].hypot(p[1]) <= req.config.max_velocity * req.config.sample_dt + 1e-6,
                "新轨迹的首位置必须绑定实际机体，不能续用旧计划位置：{p:?}");
        }
    }
    assert!(plans >= 2, "必须覆盖至少一次执行中重规划，不能只检查初次轨迹");
}

#[test]
fn global_detour_commands_reach_the_goal_without_cutting_the_wall() {
    let route = [[0., 0., 0.35], [2.4, 0., 0.35], [2.4, -1.7, 0.35]];
    let mut req = request(&route, &[]);
    req.config.planning_horizon = 5.;
    let mut planner = fixture_planner();
    // 沿用原闭环夹具600拍及实际命令积分，不把导出样条当成实际运动。
    for tick in 0..600 {
        req.odometry_timestamp_ns = 1_000_000_000 + tick * 100_000_000;
        let result = planner.step(&req).unwrap();
        let yaw = req.start_yaw;
        let world = req.start_velocity.unwrap();
        let previous = [yaw.cos()*world[0]+yaw.sin()*world[1], -yaw.sin()*world[0]+yaw.cos()*world[1]];
        let next = [result.command_forward,result.command_lateral];
        let dt = result.command_duration;
        assert!((next[0]-previous[0]).hypot(next[1]-previous[1]) <= req.config.max_acceleration*dt+1e-8);
        let v = [(previous[0]+next[0])/2., (previous[1]+next[1])/2.];
        let w = (req.start_yaw_rate.unwrap()+result.command_yaw_rate)/2.;
        let (a,b) = if w.abs()>1e-9 {((w*dt).sin()/w,(1.-(w*dt).cos())/w)} else {(dt,0.)};
        let dx=a*v[0]-b*v[1];let dy=b*v[0]+a*v[1];
        req.start_position[0]+=yaw.cos()*dx-yaw.sin()*dy;
        req.start_position[1]+=yaw.sin()*dx+yaw.cos()*dy;
        req.start_yaw+=w*dt;
        req.start_yaw_rate=Some(result.command_yaw_rate);
        req.start_velocity=Some([next[0]*req.start_yaw.cos()-next[1]*req.start_yaw.sin(),
            next[0]*req.start_yaw.sin()+next[1]*req.start_yaw.cos(),0.]);
        req.start_acceleration=None;
        let (cs,sn)=(req.start_yaw.cos(),req.start_yaw.sin());
        let dx=1.3-req.start_position[0];let dy=-0.7-req.start_position[1];
        let overlaps=dx.abs()<0.35*cs.abs()+0.15*sn.abs()+0.5
            && dy.abs()<0.35*sn.abs()+0.15*cs.abs()+0.4
            && (dx*cs+dy*sn).abs()<0.35+0.5*cs.abs()+0.4*sn.abs()
            && (-dx*sn+dy*cs).abs()<0.15+0.5*sn.abs()+0.4*cs.abs();
        assert!(!overlaps,"第{tick}拍实际机身切过全局墙体：{:?}",req.start_position);
        if (req.start_position[0]-2.4).hypot(req.start_position[1]+1.7)<0.15 {return;}
    }
    panic!("原600拍内没有沿绕行到达：{:?}",req.start_position);
}
