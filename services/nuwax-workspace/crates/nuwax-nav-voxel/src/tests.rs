use super::*;

fn model() -> BodyModel {
    BodyModel {
        footprint: vec![[-0.3, -0.16], [0.3, -0.16], [0.3, 0.16], [-0.3, 0.16]],
        z_bounds: [-0.448, 0.057],
        support_offset: 0.4,
        step_up: 0.15,
        step_down: 0.15,
        max_cross_slope_degrees: 18.0,
    }
}
fn cloud(heights: impl Fn(f64, f64) -> Vec<f64>, solid: bool) -> Vec<(f32, f32, f32)> {
    let q = f64::from(0.05_f32);
    let mut points = Vec::new();
    for x in -30..91 {
        for y in -20..21 {
            let px = (f64::from(x) + 0.5) * q;
            let py = (f64::from(y) + 0.5) * q;
            for height in heights(px, py) {
                let top = (height / q).round() as i32 - 1;
                for z in (if solid { (-1).min(top) } else { top })..=top {
                    points.push((px as f32, py as f32, ((f64::from(z) + 0.5) * q) as f32));
                }
            }
        }
    }
    points
}
fn pose(x: f64, y: f64, z: f64) -> BodyPose {
    ([x, y, z], [0.0, 0.0, 0.0, 1.0])
}

#[test]
fn overlapping_levels_remain_separate_and_vertical_motion_is_rejected() {
    let map = VoxelMap::from_points(&cloud(|_, _| vec![0.0, 2.0], false), 0.05, model()).unwrap();
    let lower = pose(0.0, 0.0, 0.4);
    let upper = pose(0.0, 0.0, 2.4);
    map.validate_pose(lower).unwrap();
    map.validate_pose(upper).unwrap();
    assert!(map.validate_motion(lower, upper).is_err());
    assert!(map
        .plan_reference(lower, &[[0.0, 0.0, 0.0], [0.0, 0.0, 2.0]], 0.81)
        .is_err());
    assert!((map.primitive(upper, 0.8, 0.0).unwrap().last().unwrap().0[2] - 2.4).abs() < 1e-8);
}

#[test]
fn occupied_stair_interiors_do_not_pin_contact_height_to_the_ground_floor() {
    let points = cloud(
        |x, _| {
            vec![if x < 0.0 {
                0.0
            } else {
                ((x / 0.5).floor() + 1.0).min(6.0) * 0.1
            }]
        },
        true,
    );
    let map = VoxelMap::from_points(&points, 0.05, model()).unwrap();
    let up = map.primitive(pose(-0.75, 0.0, 0.4), 4.25, 0.0).unwrap();
    let end = *up.last().unwrap();
    assert!((end.0[2] - 1.0).abs() < 0.03, "实心阶顶应抬升机体：{end:?}");
    let turned = *map
        .primitive(end, 0.0, std::f64::consts::PI)
        .unwrap()
        .last()
        .unwrap();
    let down = map.primitive(turned, 4.25, 0.0).unwrap();
    assert!((down.last().unwrap().0[2] - 0.4).abs() < 0.03);
}

#[test]
fn obstacle_between_clear_endpoints_is_rejected_by_swept_volume() {
    let mut points = cloud(|_, _| vec![0.0], false);
    points.push((0.025, 0.025, 0.275));
    let map = VoxelMap::from_points(&points, 0.05, model()).unwrap();
    let a = pose(-0.8, 0.0, 0.4);
    let b = pose(0.8, 0.0, 0.4);
    map.validate_pose(a).unwrap();
    map.validate_pose(b).unwrap();
    assert!(map.validate_motion(a, b).is_err());
    assert!(map.primitive(a, 1.6, 0.0).is_err());
}

#[test]
fn missing_columns_high_steps_and_low_ceilings_are_not_free_space() {
    for points in [
        cloud(
            |x, _| {
                if (0.0..0.1).contains(&x) {
                    vec![]
                } else {
                    vec![0.0]
                }
            },
            false,
        ),
        cloud(|x, _| vec![if x >= 0.0 { 0.4 } else { 0.0 }], true),
        cloud(
            |x, _| if x >= 0.0 { vec![0.0, 0.3] } else { vec![0.0] },
            false,
        ),
    ] {
        let map = VoxelMap::from_points(&points, 0.05, model()).unwrap();
        assert!(map.primitive(pose(-0.8, 0.0, 0.4), 1.6, 0.0).is_err());
    }
}

#[test]
fn in_place_rotation_preserves_height_and_world_body_normal() {
    let map = VoxelMap::from_points(&cloud(|x, _| vec![0.1 * x], false), 0.05, model()).unwrap();
    let q = UnitQuaternion::from_euler_angles(0.03, -0.05, 0.2).into_inner();
    let from = ([0.0, 0.0, 0.4], [q.i, q.j, q.k, q.w]);
    let end = *map
        .primitive(from, 0.0, std::f64::consts::FRAC_PI_2)
        .unwrap()
        .last()
        .unwrap();
    assert_eq!(from.0, end.0);
    assert!(
        (rotation(from.1).unwrap().column(2) - rotation(end.1).unwrap().column(2)).norm() < 1e-9
    );
}

#[test]
fn genuine_thirty_three_centimeter_door_is_not_closed_by_sweep_padding() {
    let q = f64::from(0.01_f32);
    let mut points = Vec::new();
    for x in -130..131 {
        for y in -70..100 {
            points.push((
                ((f64::from(x) + 0.5) * q) as f32,
                ((f64::from(y) + 0.5) * q) as f32,
                (-q * 0.5) as f32,
            ));
            if (0..4).contains(&x) && !(0..33).contains(&y) {
                for z in 0..65 {
                    points.push((
                        ((f64::from(x) + 0.5) * q) as f32,
                        ((f64::from(y) + 0.5) * q) as f32,
                        ((f64::from(z) + 0.5) * q) as f32,
                    ));
                }
            }
        }
    }
    let map = VoxelMap::from_points(&points, 0.01, model()).unwrap();
    map.primitive(pose(-0.8, 0.165, 0.4), 1.6, 0.0).unwrap();
}

#[test]
fn surface_measurement_bands_and_solid_occupancy_have_explicit_different_meanings() {
    let q = f64::from(0.05_f32);
    let mut points = Vec::new();
    for x in -30..31 {
        for y in -20..21 {
            for z in -2..=if x % 3 == 0 { 0 } else { -1 } {
                points.push((
                    ((f64::from(x) + 0.5) * q) as f32,
                    ((f64::from(y) + 0.5) * q) as f32,
                    ((f64::from(z) + 0.5) * q) as f32,
                ));
            }
        }
    }
    let occupied = VoxelMap::from_points(&points, 0.05, model()).unwrap();
    assert!(occupied.ground_frame([0.0, 0.0], 0.0).unwrap().0 > 0.0);
    let measured = occupied.with_surface_measurements();
    let from = pose(-0.5, 0.0, 0.4);
    let path = measured.primitive(from, 1.0, 0.0).unwrap();
    assert!(path
        .iter()
        .all(|p| (p.0[2] - 0.4).abs() < 1e-9 && rotation(p.1).unwrap().column(2).x.abs() < 1e-9));
}
