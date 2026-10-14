//! Frozen reference and bounded filter-order diagnostic; no production alternative.
use super::*;

fn frozen_estimate_view(
    model: &ObservationModel,
    body_position: [f64; 3],
    body_rotation: [f64; 4],
    targets: &[ObservationTarget],
    mut visible: impl FnMut([f64; 3], [f64; 3]) -> bool,
) -> Result<ViewEstimate, String> {
    model.validate()?;
    if body_position
        .iter()
        .chain(body_rotation.iter())
        .any(|v| !v.is_finite())
        || (body_rotation.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-6
    {
        return Err("观察收益缺少有效机体姿态".into());
    }
    let offset = rotate(body_rotation, model.sensor_translation_body_m);
    let origin = std::array::from_fn(|i| body_position[i] + offset[i]);
    let mut eligible = Vec::new();
    let mut seen = BTreeSet::new();
    for target in targets {
        if target.position.iter().any(|v| !v.is_finite()) || !seen.insert(target.cell) {
            continue;
        }
        let delta = std::array::from_fn(|i| target.position[i] - origin[i]);
        let sensor = rotate(
            inverse(model.sensor_rotation_body_xyzw),
            rotate(inverse(body_rotation), delta),
        );
        let horizontal = sensor[0].hypot(sensor[1]);
        let range = horizontal.hypot(sensor[2]);
        let azimuth = sensor[1].atan2(sensor[0]);
        let elevation = sensor[2].atan2(horizontal);
        let vertical = if model.kind == ObservationKind::Planar {
            sensor[2].abs() <= 1e-5
        } else {
            elevation >= model.vertical_min_rad - 1e-9 && elevation <= model.vertical_max_rad + 1e-9
        };
        if range >= model.min_range_m
            && range <= model.max_range_m
            && azimuth.abs() <= model.horizontal_fov_rad * 0.5 + 1e-9
            && vertical
        {
            eligible.push(target);
        }
    }
    eligible.sort_by_key(|target| target.cell);
    let count = eligible.len().min(model.max_rays);
    let mut estimate = ViewEstimate {
        budget_limited: eligible.len() > count,
        ..Default::default()
    };
    for i in 0..count {
        let target = eligible[i * eligible.len() / count];
        estimate.rays += 1;
        if visible(origin, target.position) {
            estimate.cells.insert(target.cell);
        }
    }
    Ok(estimate)
}


fn candidate_estimate_view(
    model: &ObservationModel,
    body_position: [f64; 3],
    body_rotation: [f64; 4],
    targets: &[ObservationTarget],
    mut visible: impl FnMut([f64; 3], [f64; 3]) -> bool,
) -> Result<ViewEstimate, String> {
    model.validate()?;
    if body_position
        .iter()
        .chain(body_rotation.iter())
        .any(|v| !v.is_finite())
        || (body_rotation.iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-6
    {
        return Err("观察收益缺少有效机体姿态".into());
    }
    let offset = rotate(body_rotation, model.sensor_translation_body_m);
    let origin = std::array::from_fn(|i| body_position[i] + offset[i]);
    let mut eligible = Vec::new();
    let mut seen = BTreeSet::new();
    for target in targets {
        if target.position.iter().any(|v| !v.is_finite()) || !seen.insert(target.cell) {
            continue;
        }
        let delta = std::array::from_fn(|i| target.position[i] - origin[i]);
        let sensor = rotate(
            inverse(model.sensor_rotation_body_xyzw),
            rotate(inverse(body_rotation), delta),
        );
        let horizontal = sensor[0].hypot(sensor[1]);
        let range = horizontal.hypot(sensor[2]);
        if !(range >= model.min_range_m && range <= model.max_range_m) {
            continue;
        }
        let vertical = if model.kind == ObservationKind::Planar {
            sensor[2].abs() <= 1e-5
        } else {
            let elevation = sensor[2].atan2(horizontal);
            elevation >= model.vertical_min_rad - 1e-9 && elevation <= model.vertical_max_rad + 1e-9
        };
        if !vertical { continue; }
        let azimuth = sensor[1].atan2(sensor[0]);
        if azimuth.abs() <= model.horizontal_fov_rad * 0.5 + 1e-9 {
            eligible.push(target);
        }
    }
    eligible.sort_by_key(|target| target.cell);
    let count = eligible.len().min(model.max_rays);
    let mut estimate = ViewEstimate {
        budget_limited: eligible.len() > count,
        ..Default::default()
    };
    for i in 0..count {
        let target = eligible[i * eligible.len() / count];
        estimate.rays += 1;
        if visible(origin, target.position) {
            estimate.cells.insert(target.cell);
        }
    }
    Ok(estimate)
}


fn calibration(planar: bool) -> ObservationModel {
    ObservationModel {
        kind: if planar { ObservationKind::Planar } else { ObservationKind::Volume },
        min_range_m: 0.1, max_range_m: 5.0,
        horizontal_fov_rad: std::f64::consts::TAU,
        vertical_min_rad: if planar { 0.0 } else { -0.8 },
        vertical_max_rad: if planar { 0.0 } else { 0.8 },
        sensor_translation_body_m: [0.0; 3],
        sensor_rotation_body_xyzw: [0.0, 0.0, 0.0, 1.0],
        max_rays: 128,
    }
}

type Recorded = (Result<(BTreeSet<[i32; 3]>, usize, bool), String>, Vec<([u64; 3], [u64; 3])>);
fn recorded(which: usize, model: &ObservationModel, body: [f64; 3], rotation: [f64; 4], targets: &[ObservationTarget]) -> Recorded {
    let mut calls = Vec::new();
    let visible = |a: [f64; 3], b: [f64; 3]| {
        calls.push((a.map(f64::to_bits), b.map(f64::to_bits)));
        // Mixed actual callback answers make ray order part of the oracle.
        calls.len() % 3 != 0
    };
    let result = match which {
        0 => frozen_estimate_view(model, body, rotation, targets, visible),
        1 => candidate_estimate_view(model, body, rotation, targets, visible),
        _ => estimate_view(model, body, rotation, targets, visible),
    };
    (result.map(|v| (v.cells, v.rays, v.budget_limited)), calls)
}

fn assert_same(model: &ObservationModel, body: [f64; 3], q: [f64; 4], targets: &[ObservationTarget]) {
    let expected = recorded(0, model, body, q, targets);
    assert_eq!(recorded(1, model, body, q, targets), expected);
    assert_eq!(recorded(2, model, body, q, targets), expected);
}

#[test]
fn filter_order_preserves_errors_dedup_budget_and_float_boundaries() {
    let q = [0.0, 0.0, 0.0, 1.0];
    for planar in [false, true] {
        let mut model = calibration(planar);
        let mut targets = Vec::new();
        for (i, x) in [0.0, -0.0, 0.1, f64::from_bits(0.1_f64.to_bits()-1),
            f64::from_bits(0.1_f64.to_bits()+1), 5.0,
            f64::from_bits(5.0_f64.to_bits()-1),f64::from_bits(5.0_f64.to_bits()+1),
            f64::MIN_POSITIVE,f64::MAX,-f64::MAX,f64::INFINITY,f64::NEG_INFINITY,f64::NAN].into_iter().enumerate() {
            for (j,z) in [-0.0,0.0,1e-5,-1e-5,f64::from_bits(1e-5_f64.to_bits()+1),1.0,-1.0,f64::MAX].into_iter().enumerate() {
                targets.push(ObservationTarget { cell: [i as i32,j as i32,0],position:[x, -0.0,z] });
            }
        }
        // Invalid first position must not consume a duplicate's identity; finite
        // out-of-range first position must consume it, even if the later one fits.
        targets.extend([
            ObservationTarget{cell:[300,0,0],position:[f64::NAN,0.0,0.0]},
            ObservationTarget{cell:[300,0,0],position:[1.0,0.0,0.0]},
            ObservationTarget{cell:[301,0,0],position:[100.0,0.0,0.0]},
            ObservationTarget{cell:[301,0,0],position:[1.0,0.0,0.0]},
            ObservationTarget{cell:[302,0,0],position:[-1.0,0.0,0.0]},
            ObservationTarget{cell:[302,0,0],position:[1.0,-0.0,0.0]},
        ]);
        for rays in [16,128,8192] {
            model.max_rays=rays;
            for fov in [std::f64::consts::TAU,std::f64::consts::FRAC_PI_2,1e-9] {
                model.horizontal_fov_rad=fov;
                assert_same(&model,[0.0;3],q,&targets);
                assert_same(&model,[f64::MAX,-f64::MAX,0.0],q,&targets);
            }
        }
        for axis in 0..4 { for value in [f64::NAN,f64::INFINITY,2.0] {
            let mut bad=q;bad[axis]=value;assert_same(&model,[0.0;3],bad,&targets);
        }}
        for value in [f64::NAN,f64::INFINITY] { assert_same(&model,[value,0.0,0.0],q,&targets); }
        model.max_rays=15;assert_same(&model,[0.0;3],q,&targets);
        model.max_rays=128;model.max_range_m=f64::NAN;assert_same(&model,[0.0;3],q,&targets);
    }
    // Deterministic full-pose/extrinsics, shuffled and duplicate targets, and
    // vertical/horizontal FOV boundary samples without numerical approximations.
    for seed in 0..48 {
        let mut model=calibration(false);let angle=seed as f64*0.071;
        model.horizontal_fov_rad=0.2+(seed%20) as f64*0.2;
        model.sensor_translation_body_m=[0.25,-0.1,0.3];
        model.sensor_rotation_body_xyzw=[(angle*0.5).sin(),0.0,0.0,(angle*0.5).cos()];
        let q=[0.0,(angle*0.25).sin(),0.0,(angle*0.25).cos()];
        let mut targets=Vec::new();
        for i in 0..257 {
            let x=((i*73+seed*31)%257) as f64*0.07-9.0;
            let y=((i*31+seed*7)%127) as f64*0.08-5.0;
            let z=((i*17+seed*11)%67) as f64*0.06-2.0;
            targets.push(ObservationTarget{cell:[i%101,i%13,i%3],position:[x,y,z]});
        }
        assert_same(&model,[0.1,-0.3,0.5],q,&targets);
        targets.reverse();assert_same(&model,[0.1,-0.3,0.5],q,&targets);
    }
}

fn fixture(kind: usize) -> (ObservationModel,Vec<ObservationTarget>,Vec<([f64;3],[f64;4])>) {
    let mut model=calibration(kind==3);
    if kind==4 {model.vertical_min_rad=-0.1;model.vertical_max_rad=0.1;}
    let targets=match kind {
        // All-in-range control: dense targets within the calibrated field.
        0 => (0..1024).map(|i| ObservationTarget {cell:[i,0,0],position:[0.25+(i%32) as f64*0.1,-1.5+(i/32) as f64*0.1,0.0]}).collect(),
        // Synthetic spatial-map distributions: increasing extents at fixed
        // resolution retain both visible near targets and out-of-range targets.
        1|2|3 => {
            let side=if kind==2 {128} else {64};
            (0..side).flat_map(|x|(0..side).map(move|y|ObservationTarget{
                cell:[x,y,0],position:[(x as f64-side as f64*0.5)*0.25,(y as f64-side as f64*0.5)*0.25,0.0]})).collect()
        },
        // Vertical reject control, still in radial range.
        4 => (0..4096).map(|i|ObservationTarget{cell:[i,0,0],position:[0.25+(i%32) as f64*0.01,-0.16+(i%33) as f64*0.01,3.0]}).collect(),
        // Capacity-only all-out-of-range negative/control, not typical mapping.
        _ => (0..16384).map(|i|ObservationTarget{cell:[i,0,0],position:[30.0+(i%128) as f64*0.1,(i/128) as f64*0.1,0.0]}).collect(),
    };
    let poses=(0..16).map(|i| {
        let yaw=i as f64*0.17;
        ([i as f64*0.03,-0.2,if kind==3 {0.0}else{0.5}], [0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()])
    }).collect();
    (model,targets,poses)
}

fn estimate_batch(which: usize,model:&ObservationModel,targets:&[ObservationTarget],poses:&[([f64;3],[f64;4])]) -> Vec<(BTreeSet<[i32;3]>,usize,bool)> {
    poses.iter().map(|&(body,q)| {
        let visible=|from,to|voxel_view_is_clear(from,to,0.2,65_536,|cell|
            cell[0]==5 && (-3..=3).contains(&cell[1]) && (0..=5).contains(&cell[2]));
        let result=match which {
            0=>estimate_view(model,body,q,targets,visible),
            _=>candidate_estimate_view(model,body,q,targets,visible),
        }.unwrap();
        (result.cells,result.rays,result.budget_limited)
    }).collect()
}

#[test]
#[ignore="fixed serial paired full-view diagnostic; no adaptive samples"]
fn observation_filter_order_profile() {
    for kind in 0..6 {
        let (model,targets,poses)=fixture(kind);
        for &(body,q) in &poses {assert_same(&model,body,q,&targets);}
        let expected=estimate_batch(0,&model,&targets,&poses);
        assert_eq!(estimate_batch(1,&model,&targets,&poses),expected);
        // Three fixed pairs, alternate execution order. Timer includes complete
        // estimate_view and actual voxel rays, excludes fixture/oracle/printing.
        for pair in 0..3 {
            let order=if pair%2==0 {[0,1]} else {[1,0]};
            for which in order {
                let start=std::time::Instant::now();
                let actual=std::hint::black_box(estimate_batch(which,std::hint::black_box(&model),std::hint::black_box(&targets),std::hint::black_box(&poses)));
                let us=start.elapsed().as_secs_f64()*1e6;
                assert_eq!(actual,expected);
                let rays:usize=actual.iter().map(|v|v.1).sum();
                let cells:usize=actual.iter().map(|v|v.0.len()).sum();
                println!("{{\"kind\":{kind},\"targets\":{},\"views\":{},\"pair\":{pair},\"candidate\":{},\"us\":{us},\"rays\":{rays},\"cells\":{cells}}}",targets.len(),poses.len(),which==1);
            }
        }
    }
}
