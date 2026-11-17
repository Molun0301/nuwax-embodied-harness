//! 编解码：LCM 消息与导航类型之间的转换和发布辅助。

use super::*;

#[cfg(any(feature="guard-runtime",feature="simple-2d",feature="replanning-astar",feature="cmu-native",feature="explore",feature="mls"))]
pub(super) fn decode_navigation_goal_input(payload: &[u8]) -> Result<schema::NavigationGoal, String> {
    let goal = schema::decode_navigation_goal_body(payload).map_err(|e| e.to_string())?;
    if goal.goal.header.frame_id != "map" {
        return Err(format!(
            "目标坐标系必须为map，收到{}",
            goal.goal.header.frame_id
        ));
    }
    Ok(goal)
}

#[cfg(feature = "patrolling")]
pub(super) fn decode_pose_body(body: &[u8]) -> NavStackResult<PoseStamped> {
    let pose = schema::decode_pose_stamped(&prefixed(schema::POSE_STAMPED_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("PoseStamped decode: {error}")))?;
    pose_from_schema(&pose)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime"
))]
pub(super) fn pose_from_schema(pose: &schema::PoseStamped) -> NavStackResult<PoseStamped> {
    Ok(PoseStamped {
        frame_id: NavFrameId::from_static(&pose.header.frame_id),
        stamp: timestamp(&pose.header)?,
        pose: Pose3 {
            position: Point3::new(
                pose.pose.position[0],
                pose.pose.position[1],
                pose.pose.position[2],
            ),
            orientation: Quaternion {
                x: pose.pose.orientation_xyzw[0],
                y: pose.pose.orientation_xyzw[1],
                z: pose.pose.orientation_xyzw[2],
                w: pose.pose.orientation_xyzw[3],
            },
        },
    })
}

#[cfg(any(feature = "basic-path-follower", feature = "explore"))]
pub(super) fn decode_odometry_pose_body(body: &[u8]) -> NavStackResult<PoseStamped> {
    let odometry = schema::decode_odometry(&prefixed(schema::ODOMETRY_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Odometry decode: {error}")))?;
    Ok(PoseStamped {
        frame_id: NavFrameId::from_static(&odometry.header.frame_id),
        stamp: timestamp(&odometry.header)?,
        pose: Pose3 {
            position: Point3::new(
                odometry.pose.position[0],
                odometry.pose.position[1],
                odometry.pose.position[2],
            ),
            orientation: Quaternion {
                x: odometry.pose.orientation_xyzw[0],
                y: odometry.pose.orientation_xyzw[1],
                z: odometry.pose.orientation_xyzw[2],
                w: odometry.pose.orientation_xyzw[3],
            },
        },
    })
}

#[cfg(any(
    feature = "basic-path-follower",
    all(test, feature = "replanning-astar")
))]
pub(super) fn decode_path_body(body: &[u8]) -> NavStackResult<Path2D> {
    let path = schema::decode_path(&prefixed(schema::PATH_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Path decode: {error}")))?;
    path_from_schema(&path)
}

#[cfg(feature = "guard-runtime")]
pub(super) fn execution_path_from_schema(
    path: &schema::Path,
    surface: bool,
) -> NavStackResult<nuwax_nav_runtime::ExecutionPath> {
    if path.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "scheduler path frame must be map, got {}",
            path.header.frame_id
        )));
    }
    if surface {
        if path
            .poses
            .iter()
            .any(|pose| pose.header.frame_id != path.header.frame_id)
        {
            return Err(NavStackError::InvalidConfig(
                "Path pose frame does not match path frame".into(),
            ));
        }
        Ok(nuwax_nav_runtime::ExecutionPath::Surface(
            nuwax_nav_types_messages::Path3D {
                poses: path
                    .poses
                    .iter()
                    .map(pose_from_schema)
                    .collect::<NavStackResult<Vec<_>>>()?,
            },
        ))
    } else {
        path_from_schema(path).map(nuwax_nav_runtime::ExecutionPath::Planar)
    }
}

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "cmu-native",
    feature = "explore"
))]
pub(super) fn path_from_schema(path: &schema::Path) -> NavStackResult<Path2D> {
    if path.poses.iter().any(|pose| {
        !pose.header.frame_id.is_empty() && pose.header.frame_id != path.header.frame_id
    }) {
        return Err(NavStackError::InvalidConfig(
            "Path pose frame does not match the path header frame".into(),
        ));
    }
    Ok(Path2D::stamped(
        NavFrameId::from_static(&path.header.frame_id),
        timestamp(&path.header)?,
        path.poses
            .iter()
            .map(|pose| Point2::new(pose.pose.position[0], pose.pose.position[1]))
            .collect(),
    ))
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
pub(super) fn decode_cloud_body(body: &[u8]) -> NavStackResult<PointCloud> {
    let cloud = schema::decode_point_cloud2(&prefixed(schema::POINT_CLOUD2_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("PointCloud2 decode: {error}")))?;
    point_cloud_from_schema(cloud)
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
pub(super) fn point_cloud_from_schema(cloud: schema::PointCloud2) -> NavStackResult<PointCloud> {
    let width = usize::try_from(cloud.width)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 width is negative".into()))?;
    let height = usize::try_from(cloud.height)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 height is negative".into()))?;
    let point_step = usize::try_from(cloud.point_step)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 point_step is negative".into()))?;
    let row_step = usize::try_from(cloud.row_step)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 row_step is negative".into()))?;
    if width > 0 && height > 0 && (point_step == 0 || row_step < width * point_step) {
        return Err(NavStackError::InvalidConfig(
            "PointCloud2 stride metadata is inconsistent".into(),
        ));
    }
    let needed = height
        .checked_sub(1)
        .and_then(|rows| rows.checked_mul(row_step))
        .and_then(|bytes| bytes.checked_add(width.checked_mul(point_step)?))
        .unwrap_or(0);
    if cloud.data.len() < needed {
        return Err(NavStackError::InvalidConfig(format!(
            "PointCloud2 data has {} bytes, needs {needed}",
            cloud.data.len()
        )));
    }
    let x = cloud_field(&cloud.fields, "x")?;
    let y = cloud_field(&cloud.fields, "y")?;
    let z = cloud_field(&cloud.fields, "z")?;
    let mut points = Vec::with_capacity(width.saturating_mul(height));
    for row in 0..height {
        for column in 0..width {
            let base = row * row_step + column * point_step;
            points.push(Point3::new(
                read_cloud_scalar(&cloud.data, base, x, cloud.is_bigendian)?,
                read_cloud_scalar(&cloud.data, base, y, cloud.is_bigendian)?,
                read_cloud_scalar(&cloud.data, base, z, cloud.is_bigendian)?,
            ));
        }
    }
    Ok(PointCloud {
        stamp: timestamp(&cloud.header)?,
        frame_id: NavFrameId::from_static(&cloud.header.frame_id),
        points,
        intensities: Vec::new(),
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
pub(super) fn cloud_field<'a>(
    fields: &'a [schema::PointField],
    name: &str,
) -> NavStackResult<&'a schema::PointField> {
    fields
        .iter()
        .find(|field| field.name == name && field.count >= 1)
        .ok_or_else(|| NavStackError::InvalidConfig(format!("PointCloud2 missing {name} field")))
}

#[cfg(any(feature = "cmu-native", feature = "explore", feature = "mls"))]
pub(super) fn read_cloud_scalar(
    data: &[u8],
    base: usize,
    field: &schema::PointField,
    big_endian: bool,
) -> NavStackResult<f64> {
    let offset = usize::try_from(field.offset)
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 field offset is negative".into()))?;
    let start = base
        .checked_add(offset)
        .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 field offset overflow".into()))?;
    match field.datatype {
        7 => {
            let bytes: [u8; 4] = data
                .get(start..start + 4)
                .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 f32 is truncated".into()))?
                .try_into()
                .expect("slice length checked");
            Ok(if big_endian {
                f32::from_be_bytes(bytes)
            } else {
                f32::from_le_bytes(bytes)
            } as f64)
        }
        8 => {
            let bytes: [u8; 8] = data
                .get(start..start + 8)
                .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 f64 is truncated".into()))?
                .try_into()
                .expect("slice length checked");
            Ok(if big_endian {
                f64::from_be_bytes(bytes)
            } else {
                f64::from_le_bytes(bytes)
            })
        }
        datatype => Err(NavStackError::InvalidConfig(format!(
            "PointCloud2 field {} uses unsupported datatype {datatype}",
            field.name
        ))),
    }
}

#[cfg(feature = "simple-2d")]
pub(super) fn decode_simple_occupancy(
    body: &[u8],
) -> NavStackResult<nuwax_nav_types_messages::OccupancyGrid2D> {
    decode_simple_occupancy_with_boundary(body, None)
}

#[cfg(feature = "simple-2d")]
pub(super) fn decode_simple_occupancy_with_boundary(
    body: &[u8],
    boundary: Option<&NavigationBoundary>,
) -> NavStackResult<nuwax_nav_types_messages::OccupancyGrid2D> {
    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "simple planner OccupancyGrid frame must be map, got {}",
            grid.header.frame_id,
        )));
    }
    let width = usize::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = usize::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let resolution = f64::from(grid.info.resolution);
    if width == 0 || height == 0 || !resolution.is_finite() || resolution <= 0.0 {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid dimensions and resolution must be positive".into(),
        ));
    }
    if grid.data.len() != width.saturating_mul(height) {
        return Err(NavStackError::InvalidConfig(format!(
            "OccupancyGrid has {} cells, expected {}",
            grid.data.len(),
            width * height,
        )));
    }
    if grid
        .info
        .origin
        .position
        .iter()
        .chain(grid.info.origin.orientation_xyzw.iter())
        .any(|value| !value.is_finite())
    {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid origin contains a non-finite value".into(),
        ));
    }
    let rotation = grid.info.origin.orientation_xyzw;
    let identity = rotation[0].abs() <= 1e-9
        && rotation[1].abs() <= 1e-9
        && rotation[2].abs() <= 1e-9
        && (rotation[3].abs() - 1.0).abs() <= 1e-9;
    if !identity || grid.info.origin.position[2].abs() > 1e-9 {
        return Err(NavStackError::InvalidConfig(
            "simple planner cannot consume a rotated/non-planar occupancy origin".into(),
        ));
    }
    use nuwax_nav_types_messages::{OccupancyCell, OccupancyGrid2D};

    let mut cells = Vec::with_capacity(grid.data.len());
    for y in 0..height {
        for x in 0..width {
            let value = grid.data[y * width + x];
            let center = Point2::new(
                grid.info.origin.position[0] + (x as f64 + 0.5) * resolution,
                grid.info.origin.position[1] + (y as f64 + 0.5) * resolution,
            );
            cells.push(
                if boundary.is_some_and(|boundary| !boundary.contains(center)) || value >= 50 {
                    OccupancyCell::Occupied
                } else if value < 0 {
                    OccupancyCell::Unknown
                } else {
                    OccupancyCell::Free
                },
            );
        }
    }
    timestamp(&grid.header)?;
    Ok(OccupancyGrid2D::new(
        Point2::new(grid.info.origin.position[0], grid.info.origin.position[1]),
        resolution,
        width as u32,
        height as u32,
        cells,
    ))
}

#[cfg(any(feature = "replanning-astar", feature = "guard-runtime"))]
pub(super) fn decode_astar_costmap(body: &[u8]) -> NavStackResult<nuwax_nav_astar::navigation_map::CostMap2D> {
    use nuwax_nav_astar::navigation_map::{CostMap2D, OCCUPIED, UNKNOWN};

    let grid = schema::decode_occupancy_grid_body(body)
        .map_err(|error| NavStackError::Internal(format!("OccupancyGrid decode: {error}")))?;
    timestamp(&grid.header)?;
    if grid.header.frame_id != "map" {
        return Err(NavStackError::InvalidConfig(format!(
            "A* OccupancyGrid frame must be map, got {}",
            grid.header.frame_id
        )));
    }
    if grid
        .info
        .origin
        .position
        .iter()
        .chain(grid.info.origin.orientation_xyzw.iter())
        .any(|value| !value.is_finite())
    {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid origin contains a non-finite value".into(),
        ));
    }
    let orientation = grid.info.origin.orientation_xyzw;
    let identity_rotation = orientation[0].abs() <= 1e-9
        && orientation[1].abs() <= 1e-9
        && orientation[2].abs() <= 1e-9
        && (orientation[3].abs() - 1.0).abs() <= 1e-9;
    if !identity_rotation || grid.info.origin.position[2].abs() > 1e-9 {
        return Err(NavStackError::InvalidConfig(
            "A* CostMap2D cannot represent a rotated or non-planar OccupancyGrid origin".into(),
        ));
    }
    let width = usize::try_from(grid.info.width)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid width is negative".into()))?;
    let height = usize::try_from(grid.info.height)
        .map_err(|_| NavStackError::InvalidConfig("OccupancyGrid height is negative".into()))?;
    let resolution = f64::from(grid.info.resolution);
    if width == 0 || height == 0 || !resolution.is_finite() || resolution <= 0.0 {
        return Err(NavStackError::InvalidConfig(
            "OccupancyGrid dimensions and resolution must be positive".into(),
        ));
    }
    if grid.data.len() != width.saturating_mul(height) {
        return Err(NavStackError::InvalidConfig(format!(
            "OccupancyGrid has {} cells, expected {}",
            grid.data.len(),
            width * height
        )));
    }
    let mut map = CostMap2D::new("map", width, height, resolution);
    map.set_origin(Point2::new(
        grid.info.origin.position[0],
        grid.info.origin.position[1],
    ));
    for y in 0..height {
        for x in 0..width {
            let value = grid.data[y * width + x];
            map.set(
                x,
                y,
                if value < 0 {
                    UNKNOWN
                } else if value >= 50 {
                    OCCUPIED
                } else {
                    value as u8
                },
            );
        }
    }
    Ok(map)
}

#[cfg(feature = "replanning-astar")]
pub(super) fn decode_astar_costmap_with_boundary(
    body: &[u8],
    boundary: Option<&NavigationBoundary>,
) -> NavStackResult<nuwax_nav_astar::navigation_map::CostMap2D> {
    use nuwax_nav_astar::navigation_map::OCCUPIED;

    let mut map = decode_astar_costmap(body)?;
    let Some(boundary) = boundary else {
        return Ok(map);
    };
    for y in 0..map.height() {
        for x in 0..map.width() {
            if !boundary.contains(map.cell_center_world(x, y)) {
                map.set(x, y, OCCUPIED);
            }
        }
    }
    Ok(map)
}

#[cfg(any(feature = "replanning-astar", feature = "guard-runtime"))]
pub(super) fn require_map_pose(pose: &PoseStamped, component: &str) -> NavStackResult<()> {
    if pose.frame_id == NavFrameId::Map {
        Ok(())
    } else {
        Err(NavStackError::InvalidConfig(format!(
            "{component} frame must be map, got {}",
            pose.frame_id.as_str()
        )))
    }
}

#[cfg(any(feature = "click-goal", feature = "patrolling", feature = "explore"))]
pub(super) async fn publish_pose(
    transport: &LcmTransport,
    channel: &str,
    pose: &PoseStamped,
) -> NavStackResult<()> {
    let message = schema_pose(pose)?;
    let body = schema::encode_pose_stamped_body(&message)
        .map_err(|error| NavStackError::Internal(format!("PoseStamped encode: {error}")))?;
    publish_body(transport, channel, "geometry_msgs.PoseStamped", body).await
}

#[cfg(feature = "simple-2d")]
pub(super) async fn publish_point(
    transport: &LcmTransport,
    channel: &str,
    point: &PointStamped,
) -> NavStackResult<()> {
    let body = schema::encode_point_stamped_body(&schema::PointStamped {
        header: schema_header(point.stamp, point.frame_id.as_str())?,
        point: [point.point.x, point.point.y, point.point.z],
    })
    .map_err(|error| NavStackError::Internal(format!("PointStamped encode: {error}")))?;
    publish_body(transport, channel, "geometry_msgs.PointStamped", body).await
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) struct VersionedPathPublication<'a> {
    pub(super) path_yaws: Option<&'a [f64]>,
    pub(super) goal: &'a schema::NavigationGoal,
    pub(super) planner_runtime_generation: i64,
    pub(super) path_revision: i64,
    pub(super) path: &'a Path2D,
    pub(super) boundary_safe: bool,
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn build_versioned_path_envelope(
    publication: VersionedPathPublication<'_>,
    source_timestamp_ns: i64,
) -> NavStackResult<Option<schema::PathEnvelope>> {
    if !publication.boundary_safe || publication.path.points.is_empty() {
        return Ok(None);
    }
    if publication.path_yaws.is_some_and(|yaws| yaws.len()!=publication.path.points.len() || yaws.iter().any(|yaw|!yaw.is_finite())) {
        return Err(NavStackError::InvalidConfig("oriented path dimensions or yaw are invalid".into()));
    }
    let stamp = Timestamp::from_unix(
        source_timestamp_ns / 1_000_000_000,
        (source_timestamp_ns % 1_000_000_000) as u32,
    );
    let header = schema_header(stamp, publication.goal.goal.header.frame_id.as_str())?;
    let path = schema::Path {
        header: header.clone(),
        poses: publication
            .path
            .points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                // 中途沿下一段方向前进；最后一点恢复用户要求的目标朝向。
                let mut orientation_xyzw = publication.path.points.get(index + 1).map_or(
                    publication.goal.goal.pose.orientation_xyzw,
                    |next| {
                        let half_yaw = (next.y - point.y).atan2(next.x - point.x) * 0.5;
                        [0.0, 0.0, half_yaw.sin(), half_yaw.cos()]
                    },
                );
                if let Some(yaw)=publication.path_yaws.and_then(|yaws|yaws.get(index)) {
                    orientation_xyzw=[0.0,0.0,(yaw*0.5).sin(),(yaw*0.5).cos()];
                }
                schema::PoseStamped {
                    header: header.clone(),
                    pose: schema::Pose {
                        position: [point.x, point.y, publication.goal.goal.pose.position[2]],
                        orientation_xyzw,
                    },
                }
            })
            .collect(),
    };
    build_versioned_path_from_schema(
        publication.goal,
        publication.planner_runtime_generation,
        publication.path_revision,
        source_timestamp_ns,
        path,
    )
    .map(Some)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn build_versioned_path_from_schema(
    goal: &schema::NavigationGoal,
    planner_runtime_generation: i64,
    path_revision: i64,
    source_timestamp_ns: i64,
    path: schema::Path,
) -> NavStackResult<schema::PathEnvelope> {
    let envelope = schema::PathEnvelope {
route_kind: 0,
schema_version: schema::PATH_ENVELOPE_SCHEMA_VERSION,
session_id: goal.session_id.clone(),
motion_generation: goal.motion_generation,
planner_runtime_generation: planner_runtime_generation,
path_revision: path_revision,
path: path
};
    Ok(envelope)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) async fn publish_versioned_path(
    transport: &LcmTransport,
    channel: &str,
    publication: VersionedPathPublication<'_>,
) -> NavStackResult<bool> {
    let source_timestamp_ns = current_unix_timestamp_ns()?;
    let Some(envelope) = build_versioned_path_envelope(publication, source_timestamp_ns)? else {
        return Ok(false);
    };
    let body = schema::encode_path_envelope_body(&envelope)
        .map_err(|error| NavStackError::Internal(format!("PathEnvelope encode: {error}")))?;
    publish_body(transport, channel, "nuwax_msgs.PathEnvelope", body).await?;
    Ok(true)
}

#[cfg(any(
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "cmu-native",
    feature = "explore",
    feature = "mls"
))]
pub(super) fn current_unix_timestamp_ns() -> NavStackResult<i64> {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|error| {
            NavStackError::Internal(format!("system clock before Unix epoch: {error}"))
        })?
        .as_nanos();
    i64::try_from(now)
        .map_err(|_| NavStackError::Internal("Unix timestamp exceeds int64 nanoseconds".into()))
}

#[cfg(any(
    feature = "guard-runtime",
    feature = "replanning-astar",
    feature = "patrolling",
    feature = "basic-path-follower"
))]
pub(super) async fn publish_bool(transport: &LcmTransport, channel: &str, value: bool) -> NavStackResult<()> {
    publish_body(
        transport,
        channel,
        "std_msgs.Bool",
        schema::encode_bool_body(&schema::Bool { data: value }),
    )
    .await
}

#[cfg(feature = "guard-runtime")]
pub(super) fn build_twist2_frame(channel: &str, twist: Twist2) -> NavStackResult<RawLcmFrame> {
    build_body_frame(
        channel,
        "geometry_msgs.Twist",
        schema::encode_twist_body(&schema::Twist {
            linear: [twist.linear.x, twist.linear.y, 0.0],
            angular: [0.0, 0.0, twist.angular_z],
        }),
    )
}

#[cfg(any(
    feature = "basic-path-follower",
    feature = "simple-2d",
    feature = "replanning-astar",
    feature = "guard-runtime",
    feature = "mls",
    feature = "patrolling"
))]
pub(super) async fn publish_twist2(
    transport: &LcmTransport,
    channel: &str,
    twist: Twist2,
) -> NavStackResult<()> {
    publish_body(
        transport,
        channel,
        "geometry_msgs.Twist",
        schema::encode_twist_body(&schema::Twist {
            linear: [twist.linear.x, twist.linear.y, 0.0],
            angular: [0.0, 0.0, twist.angular_z],
        }),
    )
    .await
}

#[cfg(any(feature = "click-goal", feature = "explore"))]
pub(super) fn schema_pose(pose: &PoseStamped) -> NavStackResult<schema::PoseStamped> {
    Ok(schema::PoseStamped {
        header: schema_header(pose.stamp, pose.frame_id.as_str())?,
        pose: schema::Pose {
            position: [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ],
            orientation_xyzw: [
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ],
        },
    })
}

#[cfg(any(
    feature = "cmu-native",
    feature = "explore",
    feature = "click-goal",
    feature = "patrolling",
    feature = "simple-2d",
    feature = "movement-manager",
    feature = "replanning-astar"
))]
pub(super) fn schema_header(stamp: Timestamp, frame_id: &str) -> NavStackResult<schema::Header> {
    Ok(schema::Header {
        seq: 0,
        stamp_sec: i32::try_from(stamp.unix_seconds()).map_err(|_| {
            NavStackError::InvalidConfig("timestamp seconds exceed LCM int32 range".into())
        })?,
        stamp_nsec: i32::try_from(stamp.unix_subsec_nanos()).expect("nanoseconds fit int32"),
        frame_id: frame_id.to_string(),
    })
}

pub(super) fn timestamp(header: &schema::Header) -> NavStackResult<Timestamp> {
    if !(0..1_000_000_000).contains(&header.stamp_nsec) {
        return Err(NavStackError::InvalidConfig(format!(
            "invalid nanosecond field {}",
            header.stamp_nsec
        )));
    }
    Ok(Timestamp::from_unix(
        i64::from(header.stamp_sec),
        header.stamp_nsec as u32,
    ))
}

pub(super) fn prefixed(fingerprint: [u8; 8], body: &[u8]) -> Vec<u8> {
    let mut encoded = Vec::with_capacity(8 + body.len());
    encoded.extend_from_slice(&fingerprint);
    encoded.extend_from_slice(body);
    encoded
}
