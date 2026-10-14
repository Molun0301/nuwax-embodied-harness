//! Exact frozen ray oracle and bounded stack-axis allocation experiment.
use super::*;

fn frozen_voxel_view_is_clear(
    from: [f64; 3],
    to: [f64; 3],
    size: f64,
    max_steps: usize,
    mut occupied: impl FnMut([i32; 3]) -> bool,
) -> bool {
    if !size.is_finite()
        || size <= 0.0
        || from
            .iter()
            .chain(to.iter())
            .any(|v| !v.is_finite() || (v / size).abs() > i32::MAX as f64 * 0.5)
    {
        return false;
    }
    let mut cell = from.map(|v| (v / size).floor() as i32);
    let target = to.map(|v| (v / size).floor() as i32);
    let delta: [f64; 3] = std::array::from_fn(|i| to[i] - from[i]);
    let signs = delta.map(|v| if v >= 0.0 { 1 } else { -1 });
    let steps = delta.map(|v| {
        if v.abs() > 1e-12 {
            size / v.abs()
        } else {
            f64::INFINITY
        }
    });
    let mut next: [f64; 3] = std::array::from_fn(|i| {
        if delta[i].abs() > 1e-12 {
            ((cell[i] + if signs[i] > 0 { 1 } else { 0 }) as f64 * size - from[i]) / delta[i]
        } else {
            f64::INFINITY
        }
    });
    for _ in 0..max_steps {
        if occupied(cell) {
            return false;
        }
        if cell == target {
            return true;
        }
        let time = next.into_iter().fold(f64::INFINITY, f64::min);
        let axes: Vec<_> = (0..3).filter(|&i| (next[i] - time).abs() < 1e-10).collect();
        if axes.is_empty() {
            return false;
        }
        for mask in 1..(1 << axes.len()) {
            let mut touched = cell;
            for (bit, &axis) in axes.iter().enumerate() {
                if mask & (1 << bit) != 0 {
                    touched[axis] += signs[axis];
                }
            }
            if occupied(touched) {
                return false;
            }
        }
        for axis in axes {
            cell[axis] += signs[axis];
            next[axis] += steps[axis];
        }
    }
    false
}

fn candidate_voxel_view_is_clear(
    from: [f64; 3],
    to: [f64; 3],
    size: f64,
    max_steps: usize,
    mut occupied: impl FnMut([i32; 3]) -> bool,
) -> bool {
    if !size.is_finite()
        || size <= 0.0
        || from
            .iter()
            .chain(to.iter())
            .any(|v| !v.is_finite() || (v / size).abs() > i32::MAX as f64 * 0.5)
    {
        return false;
    }
    let mut cell = from.map(|v| (v / size).floor() as i32);
    let target = to.map(|v| (v / size).floor() as i32);
    let delta: [f64; 3] = std::array::from_fn(|i| to[i] - from[i]);
    let signs = delta.map(|v| if v >= 0.0 { 1 } else { -1 });
    let steps = delta.map(|v| {
        if v.abs() > 1e-12 {
            size / v.abs()
        } else {
            f64::INFINITY
        }
    });
    let mut next: [f64; 3] = std::array::from_fn(|i| {
        if delta[i].abs() > 1e-12 {
            ((cell[i] + if signs[i] > 0 { 1 } else { 0 }) as f64 * size - from[i]) / delta[i]
        } else {
            f64::INFINITY
        }
    });
    for _ in 0..max_steps {
        if occupied(cell) {
            return false;
        }
        if cell == target {
            return true;
        }
        let time = next.into_iter().fold(f64::INFINITY, f64::min);
        let mut axis_storage = [0; 3];
        let mut axis_count = 0;
        for i in 0..3 {
            if (next[i] - time).abs() < 1e-10 {
                axis_storage[axis_count] = i;
                axis_count += 1;
            }
        }
        let axes = &axis_storage[..axis_count];
        if axes.is_empty() {
            return false;
        }
        for mask in 1..(1 << axes.len()) {
            let mut touched = cell;
            for (bit, &axis) in axes.iter().enumerate() {
                if mask & (1 << bit) != 0 {
                    touched[axis] += signs[axis];
                }
            }
            if occupied(touched) {
                return false;
            }
        }
        for &axis in axes {
            cell[axis] += signs[axis];
            next[axis] += steps[axis];
        }
    }
    false
}


fn recorded_ray(which:usize,from:[f64;3],to:[f64;3],size:f64,limit:usize,blocker:usize)->(bool,Vec<[i32;3]>) {
    let mut calls=Vec::new();
    let occupied=|cell:[i32;3]| {
        calls.push(cell);
        match blocker {0=>false,1=>true,2=>calls.len()==3,3=>cell==[2,2,2],_=>calls.len()%7==0}
    };
    let result=match which {
        0=>frozen_voxel_view_is_clear(from,to,size,limit,occupied),
        1=>candidate_voxel_view_is_clear(from,to,size,limit,occupied),
        _=>voxel_view_is_clear(from,to,size,limit,occupied),
    };
    (result,calls)
}

#[test]
fn stack_axes_preserve_every_occupancy_callback_and_ray_result() {
    let coordinates=[-1.0,-0.0,0.0,f64::MIN_POSITIVE,1e-12,f64::from_bits(1e-12_f64.to_bits()+1),0.1,1.0,1.0+1e-10,1.0-1e-10];
    let mut cases=Vec::new();
    for &x in &coordinates {for &y in &coordinates {
        cases.push(([0.0,-0.0,0.0],[x,y,x],0.1));
        cases.push(([x,y,0.1],[-x,-y,-0.1],0.2));
    }}
    for invalid in [f64::NAN,f64::INFINITY,f64::NEG_INFINITY,f64::MAX] {
        cases.push(([invalid,0.0,0.0],[0.1;3],0.1));
        cases.push(([0.1;3],[0.0,invalid,0.0],0.1));
        cases.push(([0.1;3],[0.2;3],invalid));
    }
    for size in [-0.0,0.0,-0.1,f64::MIN_POSITIVE] {cases.push(([0.1;3],[0.2;3],size));}
    for seed in 0..128 {
        let value=|factor:i32,offset:i32|((seed*factor+offset)%101-50) as f64*0.07;
        cases.push(([value(13,7),value(3,9),value(37,6)], [value(11,8),value(17,4),value(23,8)],0.2));
    }
    for (from,to,size) in cases {for limit in [0,1,2,16,128] {for blocker in 0..5 {
        let expected=recorded_ray(0,from,to,size,limit,blocker);
        assert_eq!(recorded_ray(1,from,to,size,limit,blocker),expected);
        assert_eq!(recorded_ray(2,from,to,size,limit,blocker),expected);
    }}}
}

fn ray(which:usize,from:[f64;3],to:[f64;3],size:f64,limit:usize,block:bool)->bool {
    let occupied=|cell:[i32;3]|block && cell[0]==1;
    match which {
        0=>voxel_view_is_clear(from,to,size,limit,occupied),
        _=>candidate_voxel_view_is_clear(from,to,size,limit,occupied),
    }
}

#[test]
#[ignore="fixed serial paired direct-ray controls"]
fn stack_axes_ray_profile() {
    for (kind,from,to,limit,block) in [
        (0,[0.1;3],[0.1;3],65536,false),
        (1,[0.1;3],[0.3,0.1,0.1],65536,false),
        (2,[0.1;3],[15.1,4.3,3.7],65536,false),
        (3,[0.1;3],[15.1;3],65536,false),
        (4,[0.1;3],[15.1;3],65536,true),
        (5,[0.1;3],[15.1;3],8,false),
    ] {
        let expected=ray(0,from,to,0.2,limit,block);assert_eq!(ray(1,from,to,0.2,limit,block),expected);
        for pair in 0..3 {for which in if pair%2==0 {[0,1]}else{[1,0]} {
            let started=std::time::Instant::now();let mut accepted=0;
            for _ in 0..10_000 {accepted+=usize::from(std::hint::black_box(ray(which,std::hint::black_box(from),std::hint::black_box(to),std::hint::black_box(0.2),std::hint::black_box(limit),std::hint::black_box(block))));}
            let us=started.elapsed().as_secs_f64()*1e6;
            assert_eq!(accepted,usize::from(expected)*10_000);
            println!("{{\"suite\":\"ray\",\"kind\":{kind},\"pair\":{pair},\"candidate\":{},\"us\":{us},\"iterations\":10000,\"accepted\":{accepted}}}",which==1);
        }}
    }
}

fn view_fixture(kind:usize)->(ObservationModel,Vec<ObservationTarget>,Vec<([f64;3],[f64;4])>) {
    let model=ObservationModel {
        kind:if kind==3 {ObservationKind::Planar}else{ObservationKind::Volume},
        min_range_m:0.1,max_range_m:if kind==0 {5.0}else{10.0},
        horizontal_fov_rad:if kind==2 {std::f64::consts::FRAC_PI_2}else{std::f64::consts::TAU},
        vertical_min_rad:if kind==3 {0.0}else{-std::f64::consts::FRAC_PI_2},
        vertical_max_rad:0.0,sensor_translation_body_m:[0.0;3],sensor_rotation_body_xyzw:[0.0,0.0,0.0,1.0],
        max_rays:if kind==0 {128}else{512},
    };
    let width=if kind==0 {20}else{80};
    // Cell centers at the actual ray grid resolution; no boundary-only target
    // construction. These are synthetic map-like controls, not captured maps.
    let targets=(0..width).flat_map(|x|(0..width).map(move|y|ObservationTarget{
        cell:[x,y,0],position:[(x as f64-width as f64*0.5+0.5)*0.2,(y as f64-width as f64*0.5+0.5)*0.2,0.1]})).collect();
    let poses=(0..16).map(|i|{
        let yaw=i as f64*0.17;
        ([0.05+i as f64*0.02,0.05,if kind==3 {0.1}else{0.5}],[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()])
    }).collect();(model,targets,poses)
}

fn view_batch(which:usize,model:&ObservationModel,targets:&[ObservationTarget],poses:&[([f64;3],[f64;4])])->Vec<(BTreeSet<[i32;3]>,usize,bool)> {
    poses.iter().map(|&(p,q)|{
        let v=estimate_view(model,p,q,targets,|from,to|{
            let occupied=|cell:[i32;3]|cell[0]==5 && (-3..=3).contains(&cell[1]) && (0..=5).contains(&cell[2]);
            match which {0=>voxel_view_is_clear(from,to,0.2,65536,occupied),_=>candidate_voxel_view_is_clear(from,to,0.2,65536,occupied)}
        }).unwrap();(v.cells,v.rays,v.budget_limited)
    }).collect()
}

#[test]
#[ignore="fixed serial paired full-view stack-axis diagnostic"]
fn stack_axes_view_profile() {
    for kind in 0..4 {
        let (model,targets,poses)=view_fixture(kind);
        let expected=view_batch(0,&model,&targets,&poses);
        assert_eq!(view_batch(1,&model,&targets,&poses),expected);
        for pair in 0..3 {for which in if pair%2==0 {[0,1]}else{[1,0]} {
            let started=std::time::Instant::now();
            let actual=std::hint::black_box(view_batch(which,std::hint::black_box(&model),std::hint::black_box(&targets),std::hint::black_box(&poses)));
            let us=started.elapsed().as_secs_f64()*1e6;
            assert_eq!(actual,expected);
            let rays:usize=actual.iter().map(|v|v.1).sum();let cells:usize=actual.iter().map(|v|v.0.len()).sum();
            println!("{{\"suite\":\"view\",\"kind\":{kind},\"pair\":{pair},\"candidate\":{},\"us\":{us},\"targets\":{},\"views\":{},\"rays\":{rays},\"cells\":{cells}}}",which==1,targets.len(),poses.len());
        }}
    }
}
