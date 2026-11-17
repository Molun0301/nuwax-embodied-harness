//! 地形图扩展与点云发布。

use super::*;

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) async fn run_terrain_map_ext(
    node: TaskChannels,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    use nuwax_nav_terrain_map_ext::module::{
        Module, ModuleOutput, TerrainMapExtModule, TICK_PERIOD_MS,
    };

    let transport = lcm_transport()?;
    let mut scans = typed_stream(&transport, &node.inputs[0], "sensor_msgs.PointCloud2").await?;
    let mut odometry = typed_stream(&transport, &node.inputs[1], "nav_msgs.Odometry").await?;
    let mut module = TerrainMapExtModule::new();
    Module::start(&mut module).map_err(|error| module_error("terrain_map_ext.start", error))?;
    let mut has_odometry = false;
    let mut scan_before_odometry = None;
    let mut ticker = tokio::time::interval(Duration::from_millis(TICK_PERIOD_MS));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            message = odometry.next() => {
                let frame = required_frame(message, "terrain_map_ext.odometry")?;
                Module::on_odometry(&mut module, decode_terrain_odometry_body(&frame.payload)?)
                    .map_err(|error| module_error("terrain_map_ext.odometry", error))?;
                has_odometry = true;
                if let Some(cloud) = scan_before_odometry.take() {
                    Module::on_registered_scan(&mut module, cloud)
                        .map_err(|error| module_error("terrain_map_ext.registered_scan", error))?;
                }
            }
            message = scans.next() => {
                let frame = required_frame(message, "terrain_map_ext.registered_scan")?;
                let cloud = decode_cloud_body(&frame.payload)?;
                if cloud.frame_id != NavFrameId::Map {
                    return Err(NavStackError::InvalidConfig(format!(
                        "terrain_map_ext registered scan frame must be map, got {}",
                        cloud.frame_id.as_str()
                    )));
                }
                if has_odometry {
                    Module::on_registered_scan(&mut module, cloud)
                        .map_err(|error| module_error("terrain_map_ext.registered_scan", error))?;
                } else {
                    scan_before_odometry = Some(cloud);
                }
            }
            _ = ticker.tick() => {
                match Module::tick(&mut module)
                    .map_err(|error| module_error("terrain_map_ext.tick", error))?
                {
                    ModuleOutput::TerrainMap(cloud) => {
                        if !cloud.is_empty() {
                            publish_point_cloud(&transport, &node.outputs[0], &cloud).await?;
                        }
                    }
                    ModuleOutput::Idle => {}
                }
            }
        }
    }

    Module::stop(&mut module).map_err(|error| module_error("terrain_map_ext.stop", error))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn decode_terrain_odometry_body(body: &[u8]) -> NavStackResult<nuwax_nav_types_messages::Odometry> {
    let odometry = schema::decode_odometry(&prefixed(schema::ODOMETRY_FINGERPRINT, body))
        .map_err(|error| NavStackError::Internal(format!("Odometry decode: {error}")))?;
    Ok(nuwax_nav_types_messages::Odometry {
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
        child_frame_id: NavFrameId::from_static(&odometry.child_frame_id),
        linear: Point3::new(
            odometry.twist_linear[0],
            odometry.twist_linear[1],
            odometry.twist_linear[2],
        ),
        angular: Point3::new(
            odometry.twist_angular[0],
            odometry.twist_angular[1],
            odometry.twist_angular[2],
        ),
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) async fn publish_point_cloud(
    transport: &LcmTransport,
    channel: &str,
    cloud: &PointCloud,
) -> NavStackResult<()> {
    let body = encode_point_cloud_body(cloud)?;
    publish_body(transport, channel, "sensor_msgs.PointCloud2", body).await
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn encode_point_cloud_body(cloud: &PointCloud) -> NavStackResult<Vec<u8>> {
    cloud
        .validate()
        .map_err(|reason| NavStackError::InvalidConfig(reason.into()))?;
    let width = i32::try_from(cloud.points.len())
        .map_err(|_| NavStackError::InvalidConfig("PointCloud2 width exceeds int32".into()))?;
    let row_step = width
        .checked_mul(28)
        .ok_or_else(|| NavStackError::InvalidConfig("PointCloud2 row_step overflow".into()))?;
    let mut data = Vec::with_capacity(row_step as usize);
    for (index, point) in cloud.points.iter().enumerate() {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(NavStackError::InvalidConfig(
                "PointCloud2 output contains a non-finite coordinate".into(),
            ));
        }
        data.extend_from_slice(&point.x.to_le_bytes());
        data.extend_from_slice(&point.y.to_le_bytes());
        data.extend_from_slice(&point.z.to_le_bytes());
        let intensity = cloud.intensities.get(index).copied().unwrap_or(0.0);
        if !intensity.is_finite() {
            return Err(NavStackError::InvalidConfig(
                "PointCloud2 output contains a non-finite intensity".into(),
            ));
        }
        data.extend_from_slice(&intensity.to_le_bytes());
    }
    schema::encode_point_cloud2_body(&schema::PointCloud2 {
        header: schema_header(cloud.stamp, cloud.frame_id.as_str())?,
        height: 1,
        width,
        fields: vec![
            schema::PointField {
                name: "x".into(),
                offset: 0,
                datatype: 8,
                count: 1,
            },
            schema::PointField {
                name: "y".into(),
                offset: 8,
                datatype: 8,
                count: 1,
            },
            schema::PointField {
                name: "z".into(),
                offset: 16,
                datatype: 8,
                count: 1,
            },
            schema::PointField {
                name: "intensity".into(),
                offset: 24,
                datatype: 7,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step: 28,
        row_step,
        data,
        is_dense: true,
    })
    .map_err(|error| NavStackError::Internal(format!("PointCloud2 encode: {error}")))
}
