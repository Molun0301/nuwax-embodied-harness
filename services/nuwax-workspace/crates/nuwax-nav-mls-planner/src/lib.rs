// Copyright 2026 Nuwax.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! 共享多层表面地图：保存体素、实测高度、机体几何与前沿查询。
//!
//! SurfaceSnapshot（不可变表面快照）与SurfaceMapBuilder（表面构图器）由导航和探索共用。
//! 全局路径由nuwax-nav-pct-native中的原生PCT源码产生，局部轨迹由nuwax-nav-scan-native产生。
//! 本组件保留几何与图数据，不提供旧MLS独立规划器或消息工作进程。

#![deny(unsafe_code)]
#![warn(missing_docs)]

// 内部模块:仅在本 crate 内可见,集成测试通过 `crate::xxx` 引用
pub mod adjacency;
#[cfg(feature = "body-query-profile")]
pub mod body_query_profile;
pub mod dijkstra;
pub mod edges;
pub mod mls_planner;
pub mod nodes;
pub mod surface;
pub mod surfaces;
pub mod voxel;

pub use surface::{
    SurfaceObservation, SurfaceObservations, SurfaceBodyEnvelope, SurfaceBodyQueryView, SurfaceBuildKind, SurfaceBuildStats, SurfaceColumnUpdate, SurfaceError, SurfaceFrontier, SurfaceFrontierCacheStats,
    SurfaceLocation, SurfaceMapBuild, SurfaceMapBuilder, SurfaceSnapshot, SurfaceTransition,
};
