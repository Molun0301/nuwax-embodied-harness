//! 对外旁路输出：坐标树、导航地图与按配置发布的人物跟随点云。
use crate::{proto, session::SessionHandle};
use nuwax_codec::lcm::schema;
use nuwax_protocol::{LcmMessageType, LcmTopic, RawLcmFrameBuilder};
use nuwax_transport::LcmTransport;
use std::sync::Arc;
use tokio::sync::oneshot;
#[path = "navigation_map_transfer.rs"]
mod navigation_map_transfer;

pub struct Outputs {
    stop: oneshot::Sender<()>,
    task: tokio::task::JoinHandle<()>,
}
#[derive(Default)]
struct MapOutput {
    session_id: u64,
    frame_id: String,
    revision: i64,
    prior: Option<Arc<nuwax_map_chunks::Manifest>>,
}

impl MapOutput {
    async fn clear(&mut self, transport: &LcmTransport) -> Result<(), String> {
        if self.prior.take().is_none() {
            return Ok(());
        }
        self.revision += 1;
        send_navigation_map(transport, &schema::NavigationMap3D {
            session_id: self.session_id.to_string(),
            revision: self.revision,
            frame_id: self.frame_id.clone(),
            cleared: true,
            cloud: schema::PointCloud2 {
                header: header(0, &self.frame_id, 0),
                ..Default::default()
            },
            ..Default::default()
        }).await
    }
}

impl Outputs {
    pub fn start(session: SessionHandle) -> Result<Self, String> {
        let url = nuwax_config::transport()?.lcm_url.clone();
        let transport = LcmTransport::new_real(&url).map_err(|e| e.to_string())?;
        let (stop, mut stopped) = oneshot::channel();
        let recent_transport = transport.clone();
        let recent_session = session.clone();
        let recent_task = tokio::spawn(async move {
            let mut routes = recent_session.route_rx();
            loop {
                let route = routes.borrow_and_update().clone();
                if let Some(route) = route {
                    let mut recent = route.mapper.recent_cloud_rx();
                    loop {
                        tokio::select! {
                            changed=routes.changed()=>{if changed.is_err(){return;}break;},
                            changed=recent.changed()=>{
                                if changed.is_err(){break;}
                                let bytes=recent.borrow_and_update().clone();
                                if routes.borrow().as_ref().is_some_and(|current|current.id==route.id) {
                                    if let Some(bytes)=bytes {if let Err(reason)=publish(&recent_transport, nuwax_codec::lcm::recent_cloud::DEFAULT_TOPIC,
                                        nuwax_codec::lcm::recent_cloud::TYPE_NAME, bytes).await {recent_session.report_output_error(route.id, reason);}}
                                }
                            },
                        }
                    }
                } else if routes.changed().await.is_err() {
                    return;
                }
            }
        });
        let follow = nuwax_config::person_follow()?.autostart;
        let task = tokio::spawn(async move {
            let routes = session.route_rx();
            let mut poses = session.pose_rx();
            let mut maps = session.map_rx();
            let mut output = MapOutput::default();
            let mut follow_sent: Option<(u64, u64)> = None;
            loop {
                let (id, result) = tokio::select! {
                    _=&mut stopped=>break,
                    changed=poses.changed()=>{
                        if changed.is_err(){break;}
                        let id = routes.borrow().as_ref().map_or(0, |route| route.id);
                        let tree=session.frame_tree();
                        let message=schema::TfMessage{transforms:tree.transforms.iter().map(|edge|{
                            let p=edge.translation.clone().unwrap_or_default();let q=edge.rotation.clone().unwrap_or_default();
                            schema::TransformStamped{header:header(edge.timestamp_ns,&edge.parent_frame,0),child_frame_id:edge.child_frame.clone(),transform:schema::Transform{translation:[p.x,p.y,p.z],rotation_xyzw:[q.x,q.y,q.z,q.w]}}
                        }).collect()};
                        let result = match schema::encode_tf_message_body(&message){Ok(bytes)=>publish(&transport,"/tf","tf2_msgs.TFMessage",bytes).await,Err(e)=>Err(e.to_string())};
                        (id, result)
                    },
                    changed=maps.changed()=>{
                        if changed.is_err(){break;}
                        let id = routes.borrow().as_ref().map_or(0, |route| route.id);
                        let map=maps.borrow_and_update().clone();
                        let result = if let Some(map)=map {
                            match publish_map(&transport, &session, &map, &mut output).await {
                                Ok(Some((sealed, changed))) if follow && changed => {
                                    publish_follow(&transport, &session, &sealed, &mut follow_sent).await
                                }
                                Ok(_) => Ok(()),
                                Err(reason) => Err(reason),
                            }
                        } else {
                            follow_sent = None;
                            output.clear(&transport).await
                        };
                        (id, result)
                    },
                };
                if let Err(reason) = result {
                    session.report_output_error(id, reason);
                }
            }
            recent_task.abort();
            let _ = recent_task.await;
        });
        Ok(Self { stop, task })
    }
    pub async fn shutdown(self) {
        let _ = self.stop.send(());
        let _ = self.task.await;
    }
}
async fn publish_map(
    transport: &LcmTransport,
    session: &SessionHandle,
    _map: &proto::MapUpdate,
    output: &mut MapOutput,
) -> Result<Option<(Arc<nuwax_map_chunks::SealedMap>, bool)>, String> {
    let mapper = match session.mapper() {
        Ok(mapper) => mapper,
        Err(_) => return Ok(None),
    };
    let id = mapper.session_id();
    let publication = mapper.publication().await;
    if session
        .mapper()
        .ok()
        .is_none_or(|current| current.session_id() != id)
    {
        return Ok(None);
    }
    let publication = publication?;
    let Some(map) = publication.map else {
        return Ok(None);
    };
    let effective = grid(&map, true)?;
    publish(
        transport,
        "/map/occupancy",
        "nav_msgs.OccupancyGrid",
        schema::encode_occupancy_grid_body(&effective).map_err(|e| e.to_string())?,
    )
    .await?;
    let Some(sealed) = publication.fine else {
        return Ok(None);
    };
    let root = sealed.manifest();
    let stamp = root.source_timestamp_ns;
    let revision = if output.session_id == id {
        output.revision + 1
    } else {
        1
    };
    let mut virtual_obstacles = publication
        .forbidden
        .map(|mask| {
            grid(
                &proto::MapUpdate {
                    map_name: mask.map_name,
                    meta: mask.meta,
                    data: mask.data.clone(),
                    raw_data: mask.data,
                },
                true,
            )
        })
        .transpose()?;
    if let Some(mask) = &mut virtual_obstacles {
        mask.header = header(stamp, &effective.header.frame_id, root.revision);
    }
    let message = schema::NavigationMap3D {
        session_id: root.domain.session.clone(),
        revision,
        frame_id: effective.header.frame_id.clone(),
        source_timestamp_ns: stamp as i64,
        source_voxel_size: root.voxel_size_m,
        virtual_obstacles,
        fine: Some(navigation_map_transfer::prepare(
            &sealed,
            output.prior.as_deref(),
        )?),
        cloud: cloud(Vec::new(), &effective.header.frame_id, stamp, 0),
        ..Default::default()
    };
    send_navigation_map(transport, &message).await?;
    let changed = output
        .prior
        .as_ref()
        .is_none_or(|old| old.identity() != root.identity());
    output.session_id = id;
    output.frame_id = message.frame_id;
    output.revision = revision;
    output.prior = Some(root.clone());
    Ok(Some((sealed, changed)))
}

async fn send_navigation_map(
    transport: &LcmTransport,
    message: &schema::NavigationMap3D,
) -> Result<(), String> {
    let bytes = schema::encode_navigation_map_3d_body(message).map_err(|e| e.to_string())?;
    publish(transport, "/nav/map3d", "nuwax_msgs.NavigationMap3D", bytes).await
}
fn header(stamp: u64, frame: &str, revision: u64) -> schema::Header {
    schema::Header {
        seq: revision as i32,
        stamp_sec: (stamp / 1_000_000_000) as i32,
        stamp_nsec: (stamp % 1_000_000_000) as i32,
        frame_id: frame.into(),
    }
}
fn grid(map: &proto::MapUpdate, effective: bool) -> Result<schema::OccupancyGrid, String> {
    let meta = map.meta.as_ref().ok_or("栅格缺少元数据")?;
    Ok(schema::OccupancyGrid {
        header: header(meta.timestamp_ns, &meta.frame_id, 0),
        info: schema::MapMetaData {
            resolution: meta.resolution,
            width: meta.width as i32,
            height: meta.height as i32,
            origin: schema::Pose {
                position: [meta.origin_x, meta.origin_y, 0.],
                orientation_xyzw: [0., 0., 0., 1.],
            },
            ..Default::default()
        },
        data: if effective { &map.data } else { &map.raw_data }
            .iter()
            .map(|v| *v as i8)
            .collect(),
    })
}
fn cloud(bytes: Vec<u8>, frame: &str, stamp: u64, revision: u64) -> schema::PointCloud2 {
    schema::PointCloud2 {
        header: header(stamp, frame, revision),
        height: 1,
        width: (bytes.len() / 12) as i32,
        point_step: 12,
        row_step: bytes.len() as i32,
        fields: ["x", "y", "z"]
            .into_iter()
            .enumerate()
            .map(|(index, name)| schema::PointField {
                name: name.into(),
                offset: index as i32 * 4,
                datatype: 7,
                count: 1,
            })
            .collect(),
        data: bytes,
        is_dense: true,
        ..Default::default()
    }
}
async fn publish_follow(
    transport: &LcmTransport,
    session: &SessionHandle,
    sealed: &nuwax_map_chunks::SealedMap,
    sent: &mut Option<(u64, u64)>,
) -> Result<(), String> {
    let mapper = match session.mapper() {
        Ok(mapper) => mapper,
        Err(_) => return Ok(()),
    };
    let id = mapper.session_id();
    if sealed.manifest().domain.session != id.to_string() {
        return Ok(());
    }
    let snapshot = mapper.display_cloud().await;
    if session
        .mapper()
        .ok()
        .is_none_or(|current| current.session_id() != id)
    {
        return Ok(());
    }
    let snapshot = snapshot?;
    let frame = session.state().info.unwrap().map_frame_id;
    publish_follow_cloud(transport, &frame, id, snapshot, sent).await
}

async fn publish_follow_cloud(
    transport: &LcmTransport,
    frame: &str,
    session_id: u64,
    snapshot: crate::pointcloud::CloudSnapshot,
    sent: &mut Option<(u64, u64)>,
) -> Result<(), String> {
    let revision = (session_id, snapshot.revision);
    if *sent == Some(revision) {
        return Ok(());
    }
    let message = cloud(
        snapshot.display.to_vec(),
        frame,
        snapshot.source_timestamp_ns,
        snapshot.revision,
    );
    publish(
        transport,
        "/global_map",
        "sensor_msgs.PointCloud2",
        schema::encode_point_cloud2_body(&message).map_err(|error| error.to_string())?,
    )
    .await?;
    *sent = Some(revision);
    Ok(())
}

async fn publish(
    transport: &LcmTransport,
    topic: &str,
    kind: &str,
    bytes: Vec<u8>,
) -> Result<(), String> {
    let topic = topic.split('#').next().unwrap();
    let frame = RawLcmFrameBuilder::new(
        LcmTopic::new(topic).map_err(|e| e.to_string())?,
        LcmMessageType::new(kind).map_err(|e| e.to_string())?,
    )
    .payload(bytes)
    .build();
    transport.publish(frame).await.map_err(|e| e.to_string())
}

#[cfg(test)]
#[path = "outputs_tests.rs"]
mod tests;
