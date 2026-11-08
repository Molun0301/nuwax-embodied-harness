//! 固定三维与二维导航任务的直接构造。

use crate::{ProductionPipeline, NavStackResult, NavStackError};
use crate::task::TaskThread;
use crate::topics::NavigationTopics;
use std::sync::Arc;

pub(crate) fn start_tasks(pipeline: ProductionPipeline) -> NavStackResult<Vec<TaskThread>> {
    let topics = NavigationTopics::for_pipeline(pipeline);
    let body: Arc<str> = Arc::from(nuwax_config::robot().map_err(NavStackError::InvalidConfig)?.base_frame.clone());
    let mut tasks = vec![crate::lcm_dataflow::map_pose_owner(&topics.pose_adapter(), body.clone())?];
    match pipeline {
        ProductionPipeline::Lio3D => {
            tasks.extend(crate::lcm_dataflow::start_surface_tasks(&topics, body)?);
        }
        ProductionPipeline::Slam2D => {
            tasks.push(crate::lcm_dataflow::simple_planner_owner(&topics.planner(pipeline), body.clone())?);
            tasks.push(crate::lcm_dataflow::guarded_nav_scheduler_owner(&topics.scheduler(pipeline), body)?);
        }
    }
    Ok(tasks)
}
