// 10赫兹位姿/20赫兹控制，命令经独立刚体积分回到下一次位姿采样。
struct PoseLoopResult {
    longitudinal_rms: f64,
    lateral_rms: f64,
    commands: Vec<[f64;3]>,
}

fn drive_pose_cadence(pose_every: usize, extrapolate: bool) -> PoseLoopResult {
    let mut cfg=config();cfg.sample_dt=0.05;cfg.max_velocity=0.5;cfg.max_acceleration=0.5;
    cfg.planning_horizon=5.;cfg.max_lateral_velocity=0.;
    let route=[[0.,0.,0.4],[10.,0.,0.4]];
    let mut owner=MapOwner::new(&cfg).unwrap();owner.seed_occupied(&[],route[0]);
    owner.update_surface(&[],&[],&[],&[]).unwrap();let map=owner.snapshot();
    let mut actual=MotionState {position:[0.,0.03,0.4],velocity:[0.5,0.,0.],planning_velocity:[0.5,0.,0.],
        timestamp_ns:1_000_000_000,measured_velocity:1,measured_yaw_rate:1,..Default::default()};
    let curve=ScanPlanner::default().plan(&cfg,&map,&route,&actual,None,0.,None).unwrap();
    let mut tracker=ScanTracker::default();tracker.install(&curve);
    let mut sample=actual;let mut previous=[0.5,0.,0.];let mut commands=Vec::new();
    let (mut along,mut across)=(0.,0.);
    for tick in 0..100 {
        actual.timestamp_ns=1_000_000_000+tick as i64*50_000_000;
        if tick%pose_every==0 {sample=actual;}
        tracker.set_pose_age(if extrapolate {std::time::Duration::from_nanos((actual.timestamp_ns-sample.timestamp_ns) as u64)} else {std::time::Duration::ZERO});
        let result=tracker.track_and_check(&cfg,&map,&sample,previous,None).unwrap();
        assert!(!result.no_candidate() && !result.deviated(),"空场跟踪不可拒绝：{result:?}");
        let command=result.command;let w=command[2];let dt=cfg.sample_dt;
        let (a,b)=if w.abs()>1e-12 {((w*dt).sin()/w,(1.-(w*dt).cos())/w)} else {(dt,0.)};
        let (sn,cs)=actual.yaw.sin_cos();let dx=a*command[0]-b*command[1];let dy=b*command[0]+a*command[1];
        actual.position[0]+=cs*dx-sn*dy;actual.position[1]+=sn*dx+cs*dy;actual.yaw+=w*dt;
        actual.velocity=[actual.yaw.cos()*command[0]-actual.yaw.sin()*command[1],actual.yaw.sin()*command[0]+actual.yaw.cos()*command[1],0.];
        actual.yaw_rate=w;actual.planning_velocity=actual.velocity;
        along+=(actual.position[0]-0.5*(tick+1) as f64*dt).powi(2);across+=actual.position[1].powi(2);
        assert!(command[0].hypot(command[1])<=0.5+1e-10);
        commands.push(command);previous=command;
    }
    PoseLoopResult {longitudinal_rms:(along/100.).sqrt(),lateral_rms:(across/100.).sqrt(),commands}
}

#[test]
fn n3_ten_hz_pose_twenty_hz_control_closed_loop() {
    let before=drive_pose_cadence(2,false);let after=drive_pose_cadence(2,true);
    eprintln!("N3位姿10Hz/控制20Hz 匀速0.5米/秒 横向RMS前={:.9}米 后={:.9}米 纵向RMS前={:.9}米 后={:.9}米",before.lateral_rms,after.lateral_rms,before.longitudinal_rms,after.longitudinal_rms);
    assert!(after.longitudinal_rms<before.longitudinal_rms,"来源重复拍的纵向滞后应减少");
}

#[test]
fn n3_same_frequency_commands_are_bit_identical() {
    let before=drive_pose_cadence(1,false);let after=drive_pose_cadence(1,true);
    assert_eq!(before.commands.iter().map(|v|v.map(f64::to_bits)).collect::<Vec<_>>(),
        after.commands.iter().map(|v|v.map(f64::to_bits)).collect::<Vec<_>>(),"同频零年龄命令逐拍逐位相同");
}

#[test]
fn n3_missing_measurements_do_not_extrapolate_their_pose_components() {
    let mut state=MotionState {position:[1.,2.,3.],velocity:[0.5,0.,0.],yaw:0.2,yaw_rate:0.4,timestamp_ns:1_000_000_000,..Default::default()};
    let age=std::time::Duration::from_millis(50);
    let missing=state.extrapolated(age);assert_eq!((missing.position,missing.yaw,missing.timestamp_ns),(state.position,state.yaw,state.timestamp_ns));
    state.measured_velocity=1;let linear=state.extrapolated(age);
    assert_eq!((linear.position,linear.yaw,linear.timestamp_ns),([1.025,2.,3.],0.2,1_050_000_000));
    state.measured_velocity=0;state.measured_yaw_rate=1;let angular=state.extrapolated(age);
    assert_eq!(angular.position,state.position);assert!((angular.yaw-0.22).abs()<1e-12);
}

#[test]
fn n3_near_obstacle_closed_loop() {
    let mut cfg=config();cfg.resolution=0.05;cfg.sample_dt=0.05;cfg.max_velocity=0.5;
    cfg.max_reverse_velocity=0.5;cfg.max_lateral_velocity=0.;cfg.max_acceleration=0.5;cfg.planning_horizon=5.;
    // 0.5米宽、1米长的通道；0.3米宽机身在中心的真实净空为0.1米。
    let obstacles:Vec<_>=(16..=36).flat_map(|x| [-0.25,0.25].into_iter().flat_map(move |y|
        (4..=12).map(move |z| [x as f64*0.05,y,z as f64*0.05]))).collect();
    let route=[[0.,0.,0.4],[10.,0.,0.4]];
    let mut owner=MapOwner::new(&cfg).unwrap();owner.seed_occupied(&obstacles,route[0]);
    owner.update_surface(&obstacles,&[],&[],&[]).unwrap();let map=owner.snapshot();
    let mut actual=MotionState {position:route[0],velocity:[0.5,0.,0.],planning_velocity:[0.5,0.,0.],
        timestamp_ns:1_000_000_000,measured_velocity:1,measured_yaw_rate:1,..Default::default()};
    let curve=ScanPlanner::default().plan(&cfg,&map,&route,&actual,None,0.,None).unwrap();
    let mut tracker=ScanTracker::default();tracker.install(&curve);
    let mut sample=actual;let mut previous=[0.5,0.,0.];let mut body_velocity=[0.5_f64,0.];
    let (mut rejected,mut minimum,mut field_minimum,mut passed)=(0,f64::INFINITY,f64::INFINITY,None);
    for tick in 0..400 {
        actual.timestamp_ns=1_000_000_000+tick as i64*50_000_000;
        if tick%2==0 {sample=actual;}
        tracker.set_pose_age(std::time::Duration::from_nanos((actual.timestamp_ns-sample.timestamp_ns) as u64));
        let result=tracker.track_and_check(&cfg,&map,&sample,previous,None).unwrap();
        if result.no_candidate() {rejected+=1;}
        let command=result.command;
        // 独立执行器按原加减速限制响应命令；归零指令不会令真实速度瞬时归零。
        let dx=command[0]-body_velocity[0];let dy=command[1]-body_velocity[1];let norm=dx.hypot(dy);
        let gain=if norm>cfg.max_acceleration*cfg.sample_dt {cfg.max_acceleration*cfg.sample_dt/norm} else {1.};
        body_velocity[0]+=dx*gain;body_velocity[1]+=dy*gain;
        let w=command[2].clamp(actual.yaw_rate-cfg.max_yaw_acceleration*cfg.sample_dt,actual.yaw_rate+cfg.max_yaw_acceleration*cfg.sample_dt);
        let (a,b)=if w.abs()>1e-12 {((w*cfg.sample_dt).sin()/w,(1.-(w*cfg.sample_dt).cos())/w)} else {(cfg.sample_dt,0.)};
        let (sn,cs)=actual.yaw.sin_cos();let x=a*body_velocity[0]-b*body_velocity[1];let y=b*body_velocity[0]+a*body_velocity[1];
        actual.position[0]+=cs*x-sn*y;actual.position[1]+=sn*x+cs*y;actual.yaw+=w*cfg.sample_dt;
        actual.velocity=[actual.yaw.cos()*body_velocity[0]-actual.yaw.sin()*body_velocity[1],actual.yaw.sin()*body_velocity[0]+actual.yaw.cos()*body_velocity[1],0.];
        actual.yaw_rate=w;actual.planning_velocity=actual.velocity;previous=command;
        // 旋转矩形的轴向外包给出到两段真实墙面的独立保守距离。
        let half_x=cfg.body_length/2.*actual.yaw.cos().abs()+cfg.body_width/2.*actual.yaw.sin().abs();
        let half_y=cfg.body_length/2.*actual.yaw.sin().abs()+cfg.body_width/2.*actual.yaw.cos().abs();
        let xgap=(0.8-actual.position[0]-half_x).max(actual.position[0]-half_x-1.8).max(0.);
        let ygap=0.25-actual.position[1].abs()-half_y;
        minimum=minimum.min(if ygap<0. && xgap==0. {ygap} else {xgap.hypot(ygap.max(0.))});
        field_minimum=field_minimum.min(map.pose_clearance(actual.position,actual.yaw).body_clearance);
        if actual.position[0]>=2.5 {passed=Some((tick+1) as f64*cfg.sample_dt);break;}
    }
    eprintln!("N3贴近障碍闭环 拒绝命令次数={rejected} 最小真实净空={minimum:.9}米 净空场最小={field_minimum:.9}米 通过时间={passed:?}秒 最后位置={:?}",actual.position);
    assert!(minimum>0.,"独立机身矩形不能穿墙");
    assert!(passed.is_some(),"原400拍预算内应通过通道");
}
