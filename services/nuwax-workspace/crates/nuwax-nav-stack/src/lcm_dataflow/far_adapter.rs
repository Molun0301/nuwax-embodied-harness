//! FAR 原生规划器路径适配。

use super::*;

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) static FAR_READY_GENERATION: AtomicI64 = AtomicI64::new(0);

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(crate) fn invalidate_native_planner_generation(node_id: &str) {
    if node_id == "far_planner" { FAR_READY_GENERATION.store(0, Ordering::Release); }
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn native_ready_generation() -> i64 { FAR_READY_GENERATION.load(Ordering::Acquire) }

#[cfg(any(feature = "cmu-native", feature = "explore"))]
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub(super) struct FarJoinKey {
    pub(super) backend: String,
    pub(super) runtime_generation: i64,
    pub(super) session_id: String,
    pub(super) motion_generation: u64,
    pub(super) path_revision: i64,
    pub(super) planner_runtime_generation: i64,
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn far_join_key(state: &schema::NavigationStatus) -> Option<FarJoinKey> {
    if state.state != nuwax_nav_far_planner_bridge::worker::FAR_EVENT_SUCCEEDED
        || state.runtime_generation <= 0
        || state.path_revision <= 0
        || state.planner_runtime_generation != state.runtime_generation
    {
        return None;
    }
    Some(FarJoinKey {
        backend: state.robot_frame.clone(),
        runtime_generation: state.runtime_generation,
        session_id: state.session_id.clone(),
        motion_generation: state.motion_generation as u64,
        path_revision: state.path_revision,
        planner_runtime_generation: state.planner_runtime_generation,
    })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn far_key_matches_goal(
    key: &FarJoinKey,
    goal: &schema::NavigationGoal,
    accepted_generation: i64,
) -> bool {
    accepted_generation > 0
        && key.backend == "far-visibility-graph"
        && key.runtime_generation == accepted_generation
        && key.session_id == goal.session_id
        && key.motion_generation == goal.motion_generation
        && key.planner_runtime_generation == accepted_generation
}

#[cfg(all(any(feature = "cmu-native", feature = "explore"), test))]
pub(super) fn far_terminal_matches_goal(
    state: &schema::NavigationStatus,
    goal: &schema::NavigationGoal,
    accepted_generation: i64,
) -> bool {
    far_join_key(state)
        .as_ref()
        .is_some_and(|key| far_key_matches_goal(key, goal, accepted_generation))
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn take_far_join(
    key: &FarJoinKey,
    pending_results: &mut HashMap<FarJoinKey, Path2D>,
    pending_terminals: &mut HashSet<FarJoinKey>,
    consumed: &mut HashSet<FarJoinKey>,
) -> Option<Path2D> {
    if consumed.contains(key)
        || !pending_results.contains_key(key)
        || !pending_terminals.contains(key)
    {
        return None;
    }
    let path = pending_results.remove(key)?;
    pending_terminals.remove(key);
    consumed.insert(key.clone());
    Some(path)
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn native_path_respects_goal(path: &Path2D, goal: &schema::NavigationGoal) -> bool {
    path.frame_id.as_str() == goal.goal.header.frame_id
        && navigation_boundary_from_goal(goal)
            .ok()
            .is_some_and(|boundary| {
                path_respects_navigation_boundary(path, boundary.as_ref(), 0.025)
            })
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) fn admit_native_adapter_goal(
    latest_goal: &mut Option<schema::NavigationGoal>,
    candidate: schema::NavigationGoal,
) -> bool {
    let Some(current) = latest_goal.as_ref() else {
        *latest_goal = Some(candidate);
        return true;
    };
    if candidate == *current { return false; }
    *latest_goal = Some(candidate);
    true
}



#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) async fn run_native_planner_path_adapter(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let transport = lcm_transport()?;
    run_native_planner_path_adapter_with_transport(node, body_reference_frame_id, transport, shutdown, ready).await
}

#[cfg(any(feature = "cmu-native", feature = "explore"))]
pub(super) async fn run_native_planner_path_adapter_with_transport(
    node: TaskChannels,
    body_reference_frame_id: Arc<str>,
    transport: LcmTransport,
    mut shutdown: oneshot::Receiver<()>,
    ready: ReadySender,
) -> NavStackResult<()> {
    let mut goals = typed_stream(&transport, &node.inputs[0], "nuwax_msgs.NavigationGoal").await?;
    let mut events =
        typed_stream(&transport, &node.inputs[1], "nuwax_msgs.NavigationStatus").await?;
    let mut far_results = if node.inputs.len() == 3 {
        Some(typed_stream(&transport, &node.inputs[2], "nuwax_msgs.NavigationStatus").await?)
    } else {
        None
    };
    let mut latest_goal: Option<schema::NavigationGoal> = None;
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut pending_results: HashMap<FarJoinKey, Path2D> = HashMap::new();
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut pending_terminals: HashSet<FarJoinKey> = HashSet::new();
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut consumed_far_results: HashSet<FarJoinKey> = HashSet::new();
    #[cfg(any(feature = "cmu-native", feature = "explore"))]
    let mut observed_far_generation = 0_i64;
    signal_ready(ready)?;

    loop {
        tokio::select! {
            biased;
            _ = &mut shutdown => break,
            frame = goals.next() => {
                let frame = required_frame(frame, "native_planner_path_adapter.goal")?;
                let goal = match decode_navigation_goal_input(&frame.payload) {
                    Ok(goal) => goal,
                    Err(_) => continue,
                };
                if !admit_native_adapter_goal(&mut latest_goal, goal) {
                    continue;
                }
                #[cfg(any(feature = "cmu-native", feature = "explore"))]
                {
                    pending_results.clear();
                    pending_terminals.clear();
                    consumed_far_results.clear();
                }
            }
            _frame = async {
                match far_results.as_mut() {
                    Some(results) => results.next().await,
                    None => futures::future::pending().await,
                }
            }, if far_results.is_some() => {
                #[cfg(any(feature = "cmu-native", feature = "explore"))]
                {
                    let frame = required_frame(_frame, "native_planner_path_adapter.far_result")?;
                    let accepted_generation = native_ready_generation();
                    if accepted_generation != observed_far_generation {
                        pending_results.clear();
                        pending_terminals.clear();
                        consumed_far_results.clear();
                        observed_far_generation = accepted_generation;
                    }
                    let state = match schema::decode_navigation_status_body(&frame.payload) {
                        Ok(state) => state,
                        Err(_) => continue,
                    };
                    let Some(goal) = latest_goal.as_ref() else { continue; };
                    let Some(key) = far_join_key(&state) else { continue; };
                    if !far_key_matches_goal(&key, goal, accepted_generation)
                        || state.planner_map_frame != goal.goal.header.frame_id
                        || state.planner_pose_frame != goal.goal.header.frame_id
                        || consumed_far_results.contains(&key)
                    {
                        continue;
                    }
                    let path = match nuwax_nav_far_planner_bridge::worker::decode_private_result_v3(&state) {
                        Ok(path) => path,
                        Err(_) => continue,
                    };
                    if path.header != goal.goal.header {
                        continue;
                    }
                    let path = match path_from_schema(&path) {
                        Ok(path) => path,
                        Err(_) => continue,
                    };
                    if pending_results.len() >= 64 && !pending_results.contains_key(&key) {
                        continue;
                    }
                    pending_results.insert(key.clone(), path);
                    let Some(path) = take_far_join(
                        &key,
                        &mut pending_results,
                        &mut pending_terminals,
                        &mut consumed_far_results,
                    ) else { continue; };
                    let _ = publish_versioned_path(
                        &transport,
                        &node.outputs[0],
                        VersionedPathPublication {
                    path_yaws: None,                            goal,
                            planner_runtime_generation: accepted_generation,
                            path_revision: key.path_revision,
                            path: &path,
                            boundary_safe: native_path_respects_goal(&path, goal),
                        },
                    ).await?;
                }
            }
            frame = events.next() => {
                let frame = required_frame(frame, "native_planner_path_adapter.event")?;
                if far_results.is_some() {
                    #[cfg(any(feature = "cmu-native", feature = "explore"))]
                    {
                        let state = match schema::decode_navigation_status_body(&frame.payload) {
                            Ok(state) => state,
                            Err(_) => continue,
                        };
                        let accepted_generation = native_ready_generation();
                        if accepted_generation != observed_far_generation {
                            pending_results.clear();
                            pending_terminals.clear();
                            consumed_far_results.clear();
                            observed_far_generation = accepted_generation;
                        }
                        if state.state == nuwax_nav_far_planner_bridge::worker::FAR_EVENT_READY {
                            continue;
                        }
                        let Some(goal) = latest_goal.as_ref() else { continue; };
                        let Some(key) = far_join_key(&state) else { continue; };
                        if !far_key_matches_goal(&key, goal, accepted_generation)
                            || state.planner_map_frame != goal.goal.header.frame_id
                            || state.planner_pose_frame != goal.goal.header.frame_id
                            || consumed_far_results.contains(&key)
                        {
                            continue;
                        }
                        if pending_terminals.len() >= 64 && !pending_terminals.contains(&key) {
                            continue;
                        }
                        pending_terminals.insert(key.clone());
                        let Some(path) = take_far_join(
                            &key,
                            &mut pending_results,
                            &mut pending_terminals,
                            &mut consumed_far_results,
                        ) else { continue; };
                        let _ = publish_versioned_path(
                            &transport,
                            &node.outputs[0],
                            VersionedPathPublication {
                    path_yaws: None,                                goal,
                                planner_runtime_generation: accepted_generation,
                                path_revision: key.path_revision,
                                path: &path,
                                boundary_safe: native_path_respects_goal(&path, goal),
                            },
                        ).await?;
                    }
                }
            }
        }
    }
    Ok(())
}
