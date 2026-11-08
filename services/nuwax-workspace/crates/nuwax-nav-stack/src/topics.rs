//! 两种固定组合的话题接线，不再描述图、节点依赖或端口契约。

use crate::ProductionPipeline;

#[derive(Clone, Debug)]
pub(crate) struct TaskChannels {
    pub id: String,
    pub inputs: Vec<String>,
    pub outputs: Vec<String>,
}

#[derive(Clone)]
pub(crate) struct NavigationTopics {
    pub odometry: String,
    pub transforms: String,
    pub map_pose: String,
    pub map: String,
    pub goal: String,
    pub cancel: String,
    pub path: String,
    pub waypoint: String,
    pub command: String,
    pub state: String,
    pub reached: String,
    pub stop: String,
}

impl NavigationTopics {
    pub(crate) fn for_pipeline(pipeline: ProductionPipeline) -> Self {
        Self {
            odometry: "/odom".into(), transforms: "/tf".into(), map_pose: "/nav/map_pose".into(),
            map: match pipeline { ProductionPipeline::Lio3D => "/nav/map3d", ProductionPipeline::Slam2D => "/nav/planar_occupancy" }.into(),
            goal: "/nav/goal".into(), cancel: "/nav/cancel".into(), path: "/nav/path".into(),
            waypoint: "way_point".into(), command: "/cmd_vel".into(), state: "/nav/state".into(),
            reached: "/nav/goal_reached".into(), stop: "/nav/stop_movement".into(),
        }
    }

    pub(crate) fn pose_adapter(&self) -> TaskChannels {
        TaskChannels { id: "map_pose_adapter".into(), inputs: vec![self.odometry.clone(),self.transforms.clone()], outputs: vec![self.map_pose.clone()] }
    }

    pub(crate) fn planner(&self, pipeline: ProductionPipeline) -> TaskChannels {
        match pipeline {
            ProductionPipeline::Lio3D => TaskChannels { id: "mls_planner".into(),
                inputs: vec![self.map.clone(),self.map_pose.clone(),self.goal.clone(),self.cancel.clone(),self.state.clone()], outputs: vec![self.path.clone()] },
            ProductionPipeline::Slam2D => TaskChannels { id: "simple_planner".into(),
                inputs: vec![self.map.clone(),self.map_pose.clone(),self.goal.clone(),self.stop.clone()], outputs: vec![self.waypoint.clone(),self.path.clone()] },
        }
    }

    pub(crate) fn scheduler(&self, pipeline: ProductionPipeline) -> TaskChannels {
        TaskChannels { id: match pipeline { ProductionPipeline::Lio3D => "surface_nav_scheduler", ProductionPipeline::Slam2D => "guarded_nav_scheduler" }.into(),
            inputs: vec![self.goal.clone(),self.map_pose.clone(),self.path.clone(),self.map.clone(),self.cancel.clone()],
            outputs: vec![self.command.clone(),self.state.clone(),self.reached.clone(),self.stop.clone()] }
    }
}
