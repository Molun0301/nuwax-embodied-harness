use super::*;

const IDENTITY: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
const Q_IDENTITY: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

fn primitive(shape: RigidShape) -> RigidPrimitive {
    RigidPrimitive::new(shape, [0.0; 3], IDENTITY).unwrap()
}

fn query(shape: RigidShape) -> RigidBodyQuery {
    RigidBodyQuery::new(&[primitive(shape)], ([0.0; 3], Q_IDENTITY)).unwrap()
}

#[test]
fn contact_continuous_sphere_graze_lies_between_all_old_prism_samples() {
    let primitives: std::sync::Arc<[RigidPrimitive]> = vec![
        RigidPrimitive::new(
            RigidShape::Sphere { radius_m: 0.047 },
            [0.293, 0.0, -0.06],
            IDENTITY,
        )
        .unwrap(),
    ]
    .into();
    let from = ([-1.0, 0.0, 0.4], Q_IDENTITY);
    let to = ([1.0, 0.0, 0.4], Q_IDENTITY);
    let point = [0.123, 0.047, 0.34];
    let mut motion = RigidMotionQuery::new(primitives.clone(), from, to).unwrap();
    // 旧整机矩形 x=[-.24,.34] 接触时段 [.3915,.6815]；
    // q=.05 的固定步采样，加支撑 .1 网格事件/区间中点，均未取到 t=.415。
    let mut old_samples: Vec<f64> = (0..=80).map(|i| f64::from(i) / 80.0).collect();
    let events = [0.3915, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.6815];
    old_samples.extend(events);
    old_samples.extend(events.windows(2).map(|pair| (pair[0] + pair[1]) * 0.5));
    for t in old_samples {
        assert!(
            !RigidBodyQuery::new(&primitives, motion.pose(t))
                .unwrap()
                .intersects_point(point)
                .unwrap(),
            "old t={t}"
        );
    }
    assert!(
        RigidBodyQuery::new(&primitives, motion.pose(0.415))
            .unwrap()
            .intersects_point(point)
            .unwrap()
    );
    let t = motion
        .intersection(point, point, [0.0, 1.0])
        .unwrap()
        .expect("闭球相切必须保留");
    assert!(
        RigidBodyQuery::new(&primitives, motion.pose(t))
            .unwrap()
            .intersects_point(point)
            .unwrap()
    );
    eprintln!(
        "RIGID_CONTINUOUS_GRAZE t={t:.16} intervals={} prepared={}",
        motion.work(),
        motion.prepared_poses()
    );
}

#[test]
fn contact_continuous_rotation_hits_each_real_shape_between_clear_endpoints() {
    let yaw = |angle: f64| [0.0, 0.0, (angle * 0.5).sin(), (angle * 0.5).cos()];
    let from = ([0.0; 3], yaw(-0.4));
    let to = ([0.0; 3], yaw(0.7));
    let cases = [
        RigidPrimitive::new(
            RigidShape::Box {
                size_m: [0.4, 0.01, 0.02],
            },
            [0.0; 3],
            IDENTITY,
        )
        .unwrap(),
        RigidPrimitive::new(
            RigidShape::Cylinder {
                radius_m: 0.005,
                length_m: 0.4,
            },
            [0.0; 3],
            [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0],
        )
        .unwrap(),
        RigidPrimitive::new(
            RigidShape::Sphere { radius_m: 0.005 },
            [0.19, 0.0, 0.0],
            IDENTITY,
        )
        .unwrap(),
    ];
    let point = [0.19, 0.0, 0.0];
    for primitive in cases {
        let primitives: std::sync::Arc<[RigidPrimitive]> = vec![primitive].into();
        for pose in [from, to] {
            assert!(
                !RigidBodyQuery::new(&primitives, pose)
                    .unwrap()
                    .intersects_point(point)
                    .unwrap()
            );
        }
        let mut motion = RigidMotionQuery::new(primitives.clone(), from, to).unwrap();
        let t = motion
            .intersection(point, point, [0.0, 1.0])
            .unwrap()
            .expect("真实实体中途接触");
        assert!(
            RigidBodyQuery::new(&primitives, motion.pose(t))
                .unwrap()
                .intersects_point(point)
                .unwrap()
        );
        eprintln!(
            "RIGID_CONTINUOUS_ROTATION shape={:?} t={t} intervals={} prepared={}",
            primitives[0].shape,
            motion.work(),
            motion.prepared_poses()
        );
    }
}

#[test]
fn contact_continuous_close_clear_sphere_rotation_does_not_inflate_its_own_radius() {
    let primitives: std::sync::Arc<[RigidPrimitive]> =
        vec![primitive(RigidShape::Sphere { radius_m: 0.047 })].into();
    let q = [0.0, 0.0, 0.6_f64.sin(), 0.6_f64.cos()];
    for (from, to, point) in [
        (
            ([-1.0, 0.0, 0.0], Q_IDENTITY),
            ([1.0, 0.0, 0.0], Q_IDENTITY),
            [0.123, 0.0470001, 0.0],
        ),
        (([0.0; 3], Q_IDENTITY), ([0.0; 3], q), [0.0, 0.0470001, 0.0]),
    ] {
        let mut motion = RigidMotionQuery::new(primitives.clone(), from, to).unwrap();
        assert_eq!(motion.intersection(point, point, [0.0, 1.0]).unwrap(), None);
        assert!(
            motion.work() <= 3,
            "近贴但分离时应以区间证书直接排除：{}",
            motion.work()
        );
    }
}

#[test]
fn contact_continuous_many_rough_floor_voxels_share_three_prepared_poses() {
    let primitives: std::sync::Arc<[RigidPrimitive]> = vec![primitive(RigidShape::Box {
        size_m: [0.3762, 0.0935, 0.114],
    })]
    .into();
    let mut motion = RigidMotionQuery::new(
        primitives,
        ([-1.0, 0.0, 0.4], Q_IDENTITY),
        ([1.0, 0.0, 0.4], Q_IDENTITY),
    )
    .unwrap();
    for x in -50..50 {
        for y in -50..50 {
            let low = [f64::from(x) * 0.01, f64::from(y) * 0.01, 0.15];
            let high = [low[0] + 0.01, low[1] + 0.01, 0.16];
            assert_eq!(motion.intersection(low, high, [0.0, 1.0]).unwrap(), None);
        }
    }
    assert_eq!(motion.prepared_poses(), 3);
    assert_eq!(motion.work(), 10_000);
}

#[test]
fn contact_fixed_bounds_include_entity_outside_the_supplied_outer_envelope() {
    let primitive = RigidPrimitive::new(
        RigidShape::Sphere { radius_m: 0.1 },
        [2.0, 0.0, 0.0],
        IDENTITY,
    )
    .unwrap();
    let query = RigidBodyQuery::new(&[primitive.clone()], ([0.0, 0.0, 0.4], Q_IDENTITY)).unwrap();
    let (low, high) = query.bounds().unwrap().unwrap();
    assert!(low[0] <= 1.9 && high[0] >= 2.1 && low[2] <= 0.3 && high[2] >= 0.5);
    let mut motion = RigidMotionQuery::new(
        vec![primitive].into(),
        ([0.0, 0.0, 0.4], Q_IDENTITY),
        ([0.5, 0.0, 0.4], Q_IDENTITY),
    )
    .unwrap();
    let point = [2.25, 0.0, 0.4];
    assert!(
        motion
            .intersection(point, point, [0.0, 1.0])
            .unwrap()
            .is_some()
    );
}

#[test]
fn contact_box_closed_faces_and_submillimeter_separation() {
    let query = query(RigidShape::Box {
        size_m: [2.0, 4.0, 6.0],
    });
    assert!(query.intersects_point([1.0, 2.0, 3.0]).unwrap());
    assert!(!query.intersects_point([1.0 + 1e-7, 0.0, 0.0]).unwrap());
    assert!(
        query
            .intersects_voxel([1.0, -0.1, -0.1], [1.5, 0.1, 0.1])
            .unwrap()
    );
    assert!(
        !query
            .intersects_voxel([1.0 + 1e-7, -0.1, -0.1], [1.5, 0.1, 0.1])
            .unwrap()
    );
}

#[test]
fn contact_local_rotation_translation_and_body_attitude_compose_in_order() {
    // 局部 X 长轴先转为机身 Y，再经机身绕 X 旋转转为世界 Z。
    let primitive = RigidPrimitive::new(
        RigidShape::Box {
            size_m: [2.0, 0.2, 0.4],
        },
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    )
    .unwrap();
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let query = RigidBodyQuery::new(&[primitive], ([3.0, 4.0, 5.0], [h, 0.0, 0.0, h])).unwrap();
    assert!(query.intersects_point([4.0, 4.0, 5.9]).unwrap());
    assert!(!query.intersects_point([4.0, 4.3, 5.0]).unwrap());
    assert!(!query.intersects_point([4.11, 4.0, 5.0]).unwrap());
    assert!(
        query
            .intersects_voxel([3.99, 3.99, 5.95], [4.01, 4.01, 6.05])
            .unwrap()
    );
    assert!(
        !query
            .intersects_voxel([3.99, 4.21, 4.99], [4.01, 4.3, 5.01])
            .unwrap()
    );
}

#[test]
fn contact_rotated_box_rejects_world_aabb_corner() {
    let c = std::f64::consts::FRAC_1_SQRT_2;
    let primitive = RigidPrimitive::new(
        RigidShape::Box {
            size_m: [2.0, 0.2, 0.2],
        },
        [0.0; 3],
        [c, -c, 0.0, c, c, 0.0, 0.0, 0.0, 1.0],
    )
    .unwrap();
    let query = RigidBodyQuery::new(&[primitive], ([0.0; 3], Q_IDENTITY)).unwrap();
    // 仍在旋转实体世界 AABB 内，但与真实细长盒分离。
    assert!(
        !query
            .intersects_voxel([0.6, -0.7, -0.05], [0.7, -0.6, 0.05])
            .unwrap()
    );
    assert!(
        query
            .intersects_voxel([0.6, 0.6, -0.05], [0.7, 0.7, 0.05])
            .unwrap()
    );
}

#[test]
fn contact_sphere_voxel_uses_nearest_distance_and_keeps_tangency() {
    let query = query(RigidShape::Sphere { radius_m: 1.0 });
    assert!(
        !query
            .intersects_voxel([0.8, 0.8, -0.1], [0.9, 0.9, 0.1])
            .unwrap()
    );
    assert!(
        query
            .intersects_voxel([1.0, -0.1, -0.1], [1.1, 0.1, 0.1])
            .unwrap()
    );
    assert!(
        query
            .intersects_voxel([0.6, 0.8, 0.0], [0.7, 0.9, 0.1])
            .unwrap()
    );
    assert!(
        !query
            .intersects_voxel([0.60001, 0.8, 0.0], [0.7, 0.9, 0.1])
            .unwrap()
    );
}

#[test]
fn contact_cylinder_is_finite_and_not_its_bounding_box_or_sphere() {
    let query = query(RigidShape::Cylinder {
        radius_m: 1.0,
        length_m: 0.4,
    });
    assert!(query.intersects_point([1.0, 0.0, 0.2]).unwrap());
    assert!(
        !query
            .intersects_voxel([0.8, 0.8, -0.1], [0.9, 0.9, 0.1])
            .unwrap()
    );
    assert!(
        !query
            .intersects_voxel([-0.1, -0.1, 0.20001], [0.1, 0.1, 0.3])
            .unwrap()
    );
    assert!(
        query
            .intersects_voxel([0.99, -0.01, 0.2], [1.01, 0.01, 0.3])
            .unwrap()
    );
    // 长条体素穿过圆盘，但每个体素角点都在圆柱外。
    assert!(
        query
            .intersects_voxel([-2.0, -0.01, -0.1], [2.0, 0.01, 0.1])
            .unwrap()
    );
    assert!(query.intersects_voxel([-2.0; 3], [2.0; 3]).unwrap());
}

#[test]
fn contact_tilted_cylinder_requires_cap_clipping_before_radial_projection() {
    let c = std::f64::consts::FRAC_1_SQRT_2;
    let primitive = RigidPrimitive::new(
        RigidShape::Cylinder {
            radius_m: 0.1,
            length_m: 0.2,
        },
        [0.0; 3],
        [c, 0.0, c, 0.0, 1.0, 0.0, -c, 0.0, c],
    )
    .unwrap();
    let query = RigidBodyQuery::new(&[primitive], ([0.0; 3], Q_IDENTITY)).unwrap();
    // 径向投影在 z≈0.2 处过原点，轴向投影在 z≈-0.2 处过端面；
    // 两个独立范围都重叠，但不存在同时满足两者的点。
    assert!(
        !query
            .intersects_voxel([0.19, -0.01, -0.25], [0.21, 0.01, 0.25])
            .unwrap()
    );
    // 缩近 X 后相交发生在端面裁出的边中部，8 原角点仍全部在外。
    assert!(
        query
            .intersects_voxel([0.09, -0.01, -0.25], [0.11, 0.01, 0.25])
            .unwrap()
    );
}

#[test]
fn contact_cylinder_cap_only_edge_and_point_intersections_are_closed() {
    let query = query(RigidShape::Cylinder {
        radius_m: 1.0,
        length_m: 2.0,
    });
    assert!(
        query
            .intersects_voxel([-2.0, 0.0, 1.0], [2.0, 0.0, 1.0])
            .unwrap()
    );
    assert!(
        query
            .intersects_voxel([1.0, 0.0, 1.0], [2.0, 0.0, 2.0])
            .unwrap()
    );
    assert!(
        !query
            .intersects_voxel([1.00001, 0.0, 1.0], [2.0, 0.0, 2.0])
            .unwrap()
    );
}

#[test]
fn contact_zero_width_voxels_agree_with_points_for_all_shapes() {
    let shapes = [
        RigidShape::Box {
            size_m: [2.0, 1.0, 0.4],
        },
        RigidShape::Sphere { radius_m: 0.7 },
        RigidShape::Cylinder {
            radius_m: 0.7,
            length_m: 0.4,
        },
    ];
    for shape in shapes {
        let query = query(shape);
        for point in [
            [0.0; 3],
            [0.7, 0.0, 0.0],
            [0.0, 0.0, 0.2],
            [0.8, 0.8, 0.0],
            [2.0; 3],
        ] {
            assert_eq!(
                query.intersects_point(point).unwrap(),
                query.intersects_voxel(point, point).unwrap(),
                "{shape:?} {point:?}"
            );
        }
    }
}

#[test]
fn contact_axis_permutation_preserves_voxel_result_for_all_shapes() {
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let shapes = [
        RigidShape::Box {
            size_m: [2.0, 0.4, 0.6],
        },
        RigidShape::Sphere { radius_m: 0.5 },
        RigidShape::Cylinder {
            radius_m: 0.5,
            length_m: 0.2,
        },
    ];
    for shape in shapes {
        let primitives = [primitive(shape)];
        let original = RigidBodyQuery::new(&primitives, ([0.0; 3], Q_IDENTITY)).unwrap();
        let rotated =
            RigidBodyQuery::new(&primitives, ([7.0, 8.0, 9.0], [h, 0.0, 0.0, h])).unwrap();
        for (min, max) in [
            ([-0.1; 3], [0.1; 3]),
            ([0.45, 0.0, -0.1], [0.55, 0.1, 0.1]),
            ([0.4, 0.4, -0.05], [0.5, 0.5, 0.05]),
            ([0.0, 0.0, 0.2], [0.1, 0.1, 0.4]),
        ] {
            let rotated_min = [7.0 + min[0], 8.0 - max[2], 9.0 + min[1]];
            let rotated_max = [7.0 + max[0], 8.0 - min[2], 9.0 + max[1]];
            assert_eq!(
                original.intersects_voxel(min, max).unwrap(),
                rotated.intersects_voxel(rotated_min, rotated_max).unwrap(),
                "{shape:?} {min:?} {max:?}"
            );
        }
    }
}

#[test]
fn contact_actual_go2_fixed_primitives_do_not_treat_leg_envelope_as_rigid_trunk() {
    // 来自 reports/geometry-go2-fixed.json，父任务使用 Python 3.8.10 实际解析。
    let primitives = [
        RigidPrimitive::new(
            RigidShape::Box {
                size_m: [0.3762, 0.0935, 0.114],
            },
            [0.0; 3],
            IDENTITY,
        )
        .unwrap(),
        RigidPrimitive::new(
            RigidShape::Cylinder {
                radius_m: 0.05,
                length_m: 0.09,
            },
            [0.285, 0.0, 0.01],
            IDENTITY,
        )
        .unwrap(),
        RigidPrimitive::new(
            RigidShape::Sphere { radius_m: 0.047 },
            [0.293, 0.0, -0.060000000000000005],
            IDENTITY,
        )
        .unwrap(),
    ];
    let query = RigidBodyQuery::new(&primitives, ([0.0, 0.0, 0.4], Q_IDENTITY)).unwrap();
    // 全腿 z_min=-.448；5/10/15 cm 起伏都在该包络中，但没有撞固定刚体。
    for height in [0.05, 0.10, 0.15] {
        assert!(height >= 0.4 - 0.448);
        assert!(!query.intersects_point([0.0, 0.0, height]).unwrap());
        assert!(
            !query
                .intersects_voxel(
                    [-0.025, -0.025, height - 0.025],
                    [0.025, 0.025, height + 0.025]
                )
                .unwrap()
        );
    }
    assert!(query.intersects_point([0.0, 0.0, 0.4]).unwrap());
    assert!(query.intersects_point([0.285, 0.0, 0.41]).unwrap());
    assert!(query.intersects_point([0.293, 0.0, 0.34]).unwrap());
    // 相同高度在头下与后方空间不同，不用统一“躯干底高”代替实体。
    assert!(query.intersects_point([0.293, 0.0, 0.30]).unwrap());
    assert!(!query.intersects_point([-0.1, 0.0, 0.30]).unwrap());
    assert!(!query.intersects_point([0.285 + 0.04, 0.04, 0.41]).unwrap());
    assert!(
        query
            .intersects_voxel([-0.01, -0.01, 0.35], [0.01, 0.01, 0.36])
            .unwrap()
    );
}

#[test]
fn contact_extreme_finite_radius_does_not_overflow_into_voxel_false_negative() {
    for shape in [
        RigidShape::Sphere { radius_m: f64::MAX },
        RigidShape::Cylinder {
            radius_m: f64::MAX,
            length_m: 1.0,
        },
        RigidShape::Cylinder {
            radius_m: f64::MAX,
            length_m: f64::MAX,
        },
    ] {
        let query = query(shape);
        for point in [[0.0; 3], [f64::MAX, 0.0, 0.0]] {
            assert!(
                query.intersects_point(point).unwrap(),
                "{shape:?} {point:?}"
            );
            assert!(
                query.intersects_voxel(point, point).unwrap(),
                "{shape:?} {point:?}"
            );
        }
        // x/y 分别有限，但 hypot 与 radius+tol 都会溢出；不能比较 inf<=inf。
        let outside = [f64::MAX, f64::MAX, 0.0];
        assert!(!query.intersects_point(outside).unwrap(), "{shape:?}");
        assert!(
            !query.intersects_voxel(outside, outside).unwrap(),
            "{shape:?}"
        );
    }
}

#[test]
fn contact_invalid_geometry_is_not_silently_made_rigid() {
    for shape in [
        RigidShape::Box {
            size_m: [1.0, 0.0, 1.0],
        },
        RigidShape::Sphere { radius_m: f64::NAN },
        RigidShape::Cylinder {
            radius_m: 1.0,
            length_m: -1.0,
        },
    ] {
        assert!(RigidPrimitive::new(shape, [0.0; 3], IDENTITY).is_err());
    }
    let shape = RigidShape::Sphere { radius_m: 1.0 };
    for matrix in [
        [2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        [1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    ] {
        assert!(RigidPrimitive::new(shape, [0.0; 3], matrix).is_err());
    }
    assert!(RigidPrimitive::new(shape, [f64::INFINITY, 0.0, 0.0], IDENTITY).is_err());
}

#[test]
fn contact_invalid_pose_and_voxel_return_errors_even_for_empty_geometry() {
    let empty = RigidBodyQuery::new(&[], ([0.0; 3], Q_IDENTITY)).unwrap();
    assert!(!empty.intersects_point([0.0; 3]).unwrap());
    assert!(!empty.intersects_voxel([-1.0; 3], [1.0; 3]).unwrap());
    assert!(empty.intersects_point([f64::NAN, 0.0, 0.0]).is_err());
    assert!(empty.intersects_voxel([1.0; 3], [0.0; 3]).is_err());
    assert!(
        empty
            .intersects_voxel([0.0; 3], [f64::INFINITY; 3])
            .is_err()
    );
    assert!(RigidBodyQuery::new(&[], ([0.0; 3], [0.0; 4])).is_err());
    assert!(RigidBodyQuery::new(&[], ([f64::NAN; 3], Q_IDENTITY)).is_err());
}


#[test]
fn contact_continuous_axial_cylinder_yaw_clearance_reuses_one_interval_for_2692_voxels() {
    let primitives: std::sync::Arc<[RigidPrimitive]> = vec![primitive(RigidShape::Cylinder {
        radius_m: 0.3, length_m: 0.1,
    })].into();
    let mut motion = RigidMotionQuery::new(primitives,
        ([0.0,0.0,0.051],Q_IDENTITY),
        ([0.0,0.0,0.051],[0.0,0.0,1.0,0.0])).unwrap();
    let mut count = 0;
    for x in -30..30 { for y in -30..30 {
        let low = [f64::from(x)*0.01,f64::from(y)*0.01,-0.01];
        let high = [f64::from(x+1)*0.01,f64::from(y+1)*0.01,0.0];
        if low[0].abs().max(high[0].abs()).hypot(low[1].abs().max(high[1].abs())) >= 0.3 {continue;}
        count += 1;
        assert_eq!(motion.intersection(low,high,[0.0,1.0]).unwrap(),None,"{low:?}");
    }}
    assert_eq!(count,2692);
    assert_eq!(motion.work(),2692);
    assert_eq!(motion.prepared_poses(),3);
    assert_eq!(motion.prepared_intervals(),1);
    eprintln!("RIGID_AXIAL_CLEAR voxels={count} intervals={} prepared={} certificates={}",
        motion.work(),motion.prepared_poses(),motion.prepared_intervals());
}

#[test]
fn contact_static_set_does_not_gain_a_second_absolute_tolerance_in_motion() {
    for shape in [RigidShape::Sphere{radius_m:0.3},
        RigidShape::Cylinder{radius_m:0.3,length_m:0.1}] {
        let primitives: std::sync::Arc<[RigidPrimitive]> = vec![primitive(shape)].into();
        let point = [0.3 + 1.5e-10,0.0,0.0];
        for q in [Q_IDENTITY,[0.0,0.0,1.0,0.0]] {
            let mut motion = RigidMotionQuery::new(primitives.clone(),([0.0;3],Q_IDENTITY),([0.0;3],q)).unwrap();
            assert_eq!(motion.intersection(point,point,[0.0,1.0]).unwrap(),None,"{shape:?}");
            assert_eq!(motion.work(),1);
        }
    }
}

#[test]
fn contact_candidate_bounds_outward_round_at_f32_voxel_boundary() {
    let q = f64::from(0.1_f32);
    let primitive = RigidPrimitive::new(RigidShape::Sphere{radius_m:0.346},[0.086,0.0,0.0],IDENTITY).unwrap();
    let primitives:std::sync::Arc<[RigidPrimitive]>=vec![primitive].into();
    let origin=[-0.23199999701976773,0.0,0.425];
    let yaw=|angle:f64|[0.0,0.0,(angle*0.5).sin(),(angle*0.5).cos()];
    let point=[2.0*q,0.0,origin[2]];
    let query=RigidBodyQuery::new(&primitives,(origin,Q_IDENTITY)).unwrap();
    assert!(query.intersects_point(point).unwrap());
    let (_,high)=query.bounds().unwrap().unwrap();
    assert!(high[0]>=point[0] && (high[0]/q).floor()>=2.0);
    let mut motion=RigidMotionQuery::new(primitives,(origin,yaw(-0.2)),(origin,yaw(0.2))).unwrap();
    let (_,high)=motion.bounds().unwrap();
    assert!(high[0]>=point[0] && (high[0]/q).floor()>=2.0);
    assert!(motion.intersection(point,point,[0.0,1.0]).unwrap().is_some());
}


#[test]
fn contact_constant_tilted_pose_keeps_one_tolerance_and_one_interval() {
    let q=[0.21,-0.37,0.18,0.83];
    let body=[0.4,-0.2,0.7];
    let axes=rotation(q).unwrap();
    for shape in [RigidShape::Sphere{radius_m:0.3},
        RigidShape::Cylinder{radius_m:0.3,length_m:0.1},
        RigidShape::Box{size_m:[0.6,0.2,0.1]}] {
        let primitives:std::sync::Arc<[RigidPrimitive]>=vec![primitive(shape)].into();
        let point=world(axes,body,[0.3+1.5e-10,0.0,0.0]);
        assert!(!RigidBodyQuery::new(&primitives,(body,q)).unwrap().intersects_point(point).unwrap());
        let mut motion=RigidMotionQuery::new(primitives,(body,q),(body,q)).unwrap();
        assert_eq!(motion.intersection(point,point,[0.0,1.0]).unwrap(),None,"{shape:?}");
        assert_eq!(motion.work(),1);
        assert_eq!(motion.prepared_intervals(),1);
    }
}

#[test]
fn contact_far_coordinate_bounds_cover_the_narrow_phase_point_scale() {
    let scale=1e100;
    let primitives:std::sync::Arc<[RigidPrimitive]>=vec![RigidPrimitive::new(
        RigidShape::Sphere{radius_m:scale},[scale,0.0,0.0],IDENTITY).unwrap()].into();
    let point=[2.0*scale+1.5*ROUNDING_FACTOR*scale,0.0,0.0];
    let pose=([0.0;3],Q_IDENTITY);
    let query=RigidBodyQuery::new(&primitives,pose).unwrap();
    assert!(query.intersects_point(point).unwrap());
    let (_,high)=query.bounds().unwrap().unwrap();
    assert!(high[0]>=point[0],"candidate={high:?}, point={point:?}");
    let motion=RigidMotionQuery::new(primitives,pose,pose).unwrap();
    let (_,high)=motion.bounds().unwrap();
    assert!(high[0]>=point[0]);
}
