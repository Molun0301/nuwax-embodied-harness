// B2 的 nav_bridge.rs:encode_goal（87c8369c）固定载荷，覆盖可选约束与完整无符号代次。
#[test]
fn geometry_b2_goal_payload_matches_navigation_codec() {
    let bytes = "00000004672d3100fedcba987654321000000000000000010dfb38d2000000046d6170003ff000000000000040000000000000003fd999999999999a0000000000000000000000000000000000000000000000003ff0000000000000010000013fc333333333333300000002bff8000000000000400000000000000040080000000000004010000000000000";
    let payload = bytes
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| u8::from_str_radix(std::str::from_utf8(pair).unwrap(), 16).unwrap())
        .collect::<Vec<_>>();
    let goal = decode_navigation_goal_body(&payload).unwrap();
    assert_eq!(goal.session_id, "g-1");
    assert_eq!(goal.motion_generation, 0xfedcba9876543210);
    assert_eq!(goal.goal.header.stamp_sec, 1);
    assert_eq!(goal.goal.header.stamp_nsec, 234567890);
    assert_eq!(goal.goal.header.frame_id, "map");
    assert_eq!(goal.goal.pose.position, [1., 2., 0.4]);
    assert_eq!(goal.goal.pose.orientation_xyzw, [0., 0., 0., 1.]);
    assert_eq!(goal.allow_reverse, Some(false));
    assert_eq!(goal.position_tolerance_m, None);
    assert_eq!(goal.yaw_tolerance_rad, Some(0.15));
    assert_eq!(goal.boundary_points, vec![[-1.5, 2., 0.], [3., 4., 0.]]);
    assert_eq!(encode_navigation_goal_body(&goal).unwrap(), payload);
}
