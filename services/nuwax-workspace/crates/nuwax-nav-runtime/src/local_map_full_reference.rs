//! 82d76046原地图线程，只用于逐拍等价与耗时对照。
use super::*;

pub(super) fn baseline_worker(initializer: Option<MapInitializer>) -> Result<MapWorker, String> {
    let mut owner: Option<MapOwner> = None;
    let mut previous: Option<MapInput> = None;
    let mut prepared: Option<PreparedMap> = None;
    let mut static_view: Option<([f64; 2], crate::native_map::NativeMap)> = None;
    Ok(MapWorker(Worker::new(
        "nav-map-maintenance",
        move |input: MapInput, reset| {
            let same_static_geometry = previous.as_ref().is_some_and(|old| {
                crate::native_map::same_source_geometry(&old.source, &input.source)
                    && old.config.planning_horizon == input.config.planning_horizon
                    && map_geometry(&old.config) == map_geometry(&input.config)
            });
            let same_geometry = same_static_geometry
                && previous.as_ref().is_some_and(|old| {
                    old.source.same_body_query_view(&input.source)
                        && crate::native_map::collision_ground_padding(&old.config)
                            == crate::native_map::collision_ground_padding(&input.config)
                        && old.boundary == input.boundary
                });
            let same_frame = previous.as_ref().is_some_and(|old| {
                old.frame.as_ref().map(|f| &f.identity) == input.frame.as_ref().map(|f| &f.identity)
            });
            let same_pose = previous
                .as_ref()
                .is_some_and(|old| old.pose == input.pose && old.yaw == input.yaw);
            let same_domain = previous.as_ref().is_none_or(|old| {
                old.frame.as_ref().map(|f| &f.identity.domain)
                    == input.frame.as_ref().map(|f| &f.identity.domain)
            });
            if reset
                || !same_domain
                || previous
                    .as_ref()
                    .is_some_and(|old| map_geometry(&old.config) != map_geometry(&input.config))
            {
                owner = None;
                prepared = None;
                static_view = None;
            }
            let rebuilt = owner.is_none();
            if rebuilt {
                owner = Some(MapOwner::new(&input.config)?);
            }
            let changed = !same_geometry || !same_frame || !same_pose || prepared.is_none();
            if changed {
                let c = input.config;
                let native = static_surface_view(&mut static_view, &input, same_static_geometry)?;
                let (recent_ground, recent_obstacles) = input.source.navigation_recent_samples();
                let obstacles: Vec<_> = native
                    .obstacles
                    .iter()
                    .copied()
                    .chain(recent_obstacles)
                    .collect();
                let ground = crate::native_map::collision_ground(
                    native.static_collision_ground.clone(),
                    recent_ground,
                    &c,
                    input.pose,
                    input.yaw,
                );
                let map = owner.as_mut().unwrap();
                if let Some(initialize) = initializer {
                    initialize(
                        map,
                        &obstacles,
                        &ground,
                        &input.boundary,
                        &native.forbidden_columns,
                        input.pose,
                    )?;
                } else {
                    map.update_surface(
                        &obstacles,
                        &ground,
                        &input.boundary,
                        &native.forbidden_columns,
                    )?;
                }
                if rebuilt || !same_frame {
                    if let Some(frame) = &input.frame {
                        map.ingest(
                            &frame.points_world,
                            &frame.ground_point_indices,
                            frame.sensor_origin_world,
                            frame.body_position_world,
                        )?;
                    }
                }
                if input.frame.is_none() && initializer.is_none() {
                    previous = Some(input);
                    return Err("等待同帧注册点云".into());
                }
                let snapshot = map.snapshot();
                prepared = Some(PreparedMap {
                    epoch: input.epoch,
                    source: input.source.clone(),
                    snapshot,
                    config: input.config,
                    changed: false,
                    checked_curve: None,
                });
            }
            let mut output = prepared.as_ref().unwrap().clone();
            output.epoch = input.epoch;
            output.source = input.source.clone();
            output.config = input.config;
            output.changed = input
                .active_snapshot
                .as_ref()
                .zip(input.active.as_ref())
                .is_some_and(|(old, curve)| {
                    !output.snapshot.same(old) && output.snapshot.affects(old, curve, input.phase)
                });
            output.checked_curve = input.active.clone();
            previous = Some(input);
            Ok(output)
        },
    )?))
}
