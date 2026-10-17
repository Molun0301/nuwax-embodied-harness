// 厚层几何只能提供固定近似参考，实际高度/相对倾斜必须沿同一参考场输运。
fn thick_ramp_nominal_height(x: f64, y: f64) -> f64 {
    0.15 * x.clamp(0.0, 7.0) + 0.035 * (1.2 * x).sin()
        + 0.025 * (1.7 * y + 0.4 * x).sin() + 0.010 * (3.1 * x + 2.3 * y).cos()
}

fn thick_ramp_motion_fixture() -> SurfaceSnapshot {
    let q = f64::from(0.05_f32);
    let mut points = Vec::new();
    let mut blocks = std::collections::BTreeMap::<[i32; 3], std::collections::BTreeMap<[i32; 3], SurfaceObservation>>::new();
    for ix in -20_i32..180 {
        for iy in -50_i32..50 {
            let code = (ix * 73 + iy * 151).rem_euclid(997);
            if code % 11 == 0 { continue; }
            let jitter = ((code % 7) as f64 - 3.0) * 0.003;
            let x = (f64::from(ix) + 0.5) * q + jitter;
            let y = (f64::from(iy) + 0.5) * q - jitter;
            let top = (thick_ramp_nominal_height(x, y) / q).round() as i32 - 1;
            for layer in 0..=2 + code % 3 {
                let point = (x as f32, y as f32, ((f64::from(top - layer) + 0.5) * q) as f32);
                let key = [point.0, point.1, point.2].map(|v| (f64::from(v) / q).floor() as i32);
                blocks.entry(key.map(|v| v.div_euclid(32))).or_default()
                    .insert(key, SurfaceObservation { observed_ns: 0, hits: 0 });
                points.push(point);
            }
        }
    }
    let mut observations = SurfaceObservations::new(0.05, 32).unwrap();
    for (key, values) in blocks { observations.replace_block(key, values).unwrap(); }
    observations.mark_complete_geometry_source();
    let mut config = scene(&[], false).config().clone();
    config.wall_clearance_m = 0.0;
    config.robot_height = 0.1;
    config.step_threshold_m = 0.15;
    let mut envelope = model();
    envelope.footprint_xy_m = vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
    envelope.z_bounds_m = [-0.448,0.057];
    envelope.max_step_up_m = 0.15;
    envelope.max_step_down_m = 0.15;
    envelope.max_cross_slope_deg = 18.0;
    SurfaceSnapshot::from_points(&points, config, 0.05).unwrap()
        .with_body_envelope(envelope).unwrap().with_observations(Some(Arc::new(observations)))
}

fn thick_ramp_follow(map: &SurfaceSnapshot, mut pose: ([f64; 3], [f64; 4]), end_x: f64, step: f64)
    -> ([f64; 3], [f64; 4]) {
    let start_x = pose.0[0];
    let count = ((end_x - start_x).abs() / step).round() as usize;
    for index in 1..=count {
        let x = start_x + (end_x - start_x) * index as f64 / count as f64;
        let next = map.body_pose_after_motion(pose, [x, pose.0[1]], 0.0)
            .unwrap_or_else(|error| panic!("厚层坡道完整运动拒绝，步长={step}，x={x}，from={pose:?}：{error}"));
        map.validate_body_motion(pose, next, &[])
            .unwrap_or_else(|error| panic!("厚层坡道严格扫掠拒绝，步长={step}，x={x}：{error}"));
        pose = next;
    }
    pose
}

#[test]
fn sustained_thick_ramp_motion_preserves_height_instead_of_sinking() {
    let map = thick_ramp_motion_fixture();
    let start = ([0.0, 1.1, thick_ramp_nominal_height(0.0, 1.1) + 0.4], [0.0,0.0,0.0,1.0]);
    map.validate_body_pose(start.0, start.1).unwrap();
    let upper = thick_ramp_follow(&map, start, 3.0, 0.02);
    let expected = thick_ramp_nominal_height(3.0, 1.1) + 0.4;
    assert!((upper.0[2] - expected).abs() < 0.03,
        "同层固定几何近似不应在连续坡道累计落后十几厘米：{upper:?}，名义参考={expected}");
    let returned = thick_ramp_follow(&map, upper, 0.0, 0.02);
    let orientation_error = rotation(returned.1).unwrap().iter().flatten()
        .zip(rotation(start.1).unwrap().iter().flatten()).map(|(a,b)|(a-b).abs()).fold(0.0,f64::max);
    eprintln!("厚坡上行Z={}，往返高度误差={}，姿态误差={orientation_error}", upper.0[2], returned.0[2]-start.0[2]);
    assert!((returned.0[2] - start.0[2]).abs() < 1e-8 && orientation_error < 1e-8);
}

#[test]
fn thick_ramp_motion_is_independent_of_step_partition() {
    let map = thick_ramp_motion_fixture();
    let start = ([0.0, 1.1, thick_ramp_nominal_height(0.0, 1.1) + 0.4], [0.0,0.0,0.0,1.0]);
    let reference = thick_ramp_follow(&map, start, 3.0, 0.1);
    for step in [0.02, 0.005] {
        let actual = thick_ramp_follow(&map, start, 3.0, step);
        let attitude = rotation(actual.1).unwrap().iter().flatten()
            .zip(rotation(reference.1).unwrap().iter().flatten()).map(|(a,b)|(a-b).abs()).fold(0.0,f64::max);
        eprintln!("厚坡步长={step}，高度差={}，姿态差={attitude}", actual.0[2]-reference.0[2]);
        assert!((actual.0[2]-reference.0[2]).abs()<1e-8 && attitude<1e-8);
    }
}

#[test]
fn thick_ramp_motion_preserves_actual_tilt_relative_to_its_reference() {
    let map = thick_ramp_motion_fixture();
    let (sr,cr)=(0.06_f64*0.5).sin_cos();
    let (sp,cp)=(-0.04_f64*0.5).sin_cos();
    let q = [sr*cp,cr*sp,-sr*sp,cr*cp];
    let start = ([0.0, 1.1, thick_ramp_nominal_height(0.0, 1.1)+0.42], q);
    map.validate_body_pose(start.0,start.1).unwrap();
    let relative = |pose: ([f64;3],[f64;4])| {
        let actual = rotation(pose.1).unwrap();
        let height = pose.0[2]-0.4/actual[2][2];
        let center = map.locate_near_support([pose.0[0],pose.0[1],height],0.3).unwrap();
        let (_,ground) = map.body_motion_contact_frame(center,height,0.0).unwrap();
        rotation(ground).unwrap().map(|basis|dot(basis,actual[2]))
    };
    let before = relative(start);
    let end = thick_ramp_follow(&map,start,3.0,0.02);
    let after = relative(end);
    assert!(before.iter().zip(after).all(|(a,b)|(a-b).abs()<1e-9), "动态参考不能重置真实倾斜偏差");
    let returned = thick_ramp_follow(&map,end,0.0,0.02);
    assert!((returned.0[2]-start.0[2]).abs()<1e-8);
}

#[test]
fn static_interval_and_fixed_motion_reference_never_share_cached_planes() {
    for motion_first in [false,true] {
        let map = ground_band_sweep_scene(2.025);
        let reference = 0.01;
        let center = map.locate_near_support([-0.65,0.05,reference],0.3).unwrap();
        let query = |motion| if motion {
            map.body_motion_contact_frame(center,reference,0.0).unwrap()
        } else {
            map.body_contact_frame(center,reference,0.0).unwrap()
        };
        let first = query(motion_first);
        let second = query(!motion_first);
        let static_frame = if motion_first {second} else {first};
        let motion_frame = if motion_first {first} else {second};
        assert!((static_frame.0-reference).abs()<1e-12, "静态共同可行区间保留原实际参考");
        assert!((motion_frame.0-static_frame.0).abs()>0.005, "夹具应具有不同的静态/动态参考");
        assert_eq!(query(motion_first),first);
        assert_eq!(query(!motion_first),second);
        assert_eq!(map.body_query_cache.lock().unwrap().planes.len(),2, "模式必须进入缓存身份");
    }
}

#[test]
fn common_legacy_contact_band_turn_keeps_the_feasible_horizontal_frame() {
    let q=f64::from(0.05_f32);
    let mut points=Vec::new();
    let mut blocks=std::collections::BTreeMap::<[i32;3],std::collections::BTreeMap<[i32;3],SurfaceObservation>>::new();
    for x in -24..24 {for y in -24..24 {
        let px=(f64::from(x)+0.5)*q;let py=(f64::from(y)+0.5)*q;
        let top=((0.15*px+0.1)/q).round() as i32-1;
        for z in top-4..=top {
            let key=[x,y,z];points.push((px as f32,py as f32,((f64::from(z)+0.5)*q) as f32));
            blocks.entry(key.map(|v|v.div_euclid(32))).or_default()
                .insert(key,SurfaceObservation{observed_ns:0,hits:0});
        }
    }}
    let mut observations=SurfaceObservations::new(0.05,32).unwrap();
    for (key,values) in blocks {observations.replace_block(key,values).unwrap();}
    observations.mark_complete_geometry_source();
    let mut config=scene(&[],false).config().clone();config.wall_clearance_m=0.0;config.robot_height=0.1;
    let mut envelope=model();envelope.footprint_xy_m=vec![[-0.3,-0.15],[0.3,-0.15],[0.3,0.15],[-0.3,0.15]];
    envelope.z_bounds_m=[-0.448,0.057];envelope.max_step_up_m=0.15;envelope.max_step_down_m=0.15;
    let map=SurfaceSnapshot::from_points(&points,config,0.05).unwrap().with_body_envelope(envelope).unwrap()
        .with_observations(Some(Arc::new(observations)));
    let (sr,cr)=(0.02_f64*0.5).sin_cos();let (sp,cp)=(-0.04_f64*0.5).sin_cos();
    let from=([0.0,0.0,0.4],[sr*cp,cr*sp,-sr*sp,cr*cp]);
    let actual=rotation(from.1).unwrap();let yaw=actual[0][1].atan2(actual[0][0]);
    map.validate_body_pose(from.0,from.1).unwrap();
    for turn in [-3.0_f64,-2.5,-1.0,-0.5,0.5,1.0,2.5,3.0] {
        let next=map.body_pose_after_motion(from,[0.0,0.0],yaw+turn).unwrap();
        map.validate_body_motion(from,next,&[]).unwrap();
        assert_eq!(next.0,from.0);
        let expected=actual.map(|axis|[turn.cos()*axis[0]-turn.sin()*axis[1],
            turn.sin()*axis[0]+turn.cos()*axis[1],axis[2]]);
        assert!(rotation(next.1).unwrap().iter().flatten().zip(expected.iter().flatten())
            .all(|(a,b)|(a-b).abs()<1e-10),
            "共同水平接触可行时，区间中点差异不能强加额外倾斜并阻塞原地转向");
    }
}
