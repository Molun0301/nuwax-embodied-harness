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

//! # nuwax-nav-mls-planner
//!
//! 纯 Rust 多层表面 (Multi-Level Surface, MLS) 3D 路径规划器。
//!
//! 本 crate 是 nuwax 旧 `dimos-mls-planner` 的 Nuwax 移植版本。
//! 基础表面图算法沿用原实现，并剥离了以下旧依赖:
//! - `dimos-module`(提供 `native_config` 派生宏与 LCM transport)
//! - `lcm-msgs`(LCM 消息类型)
//! - `pyo3` / `numpy`(Python 绑定)
//!
//! 基础算法入口:
//! - `voxel::voxelize` / `surface_point_xyz`:体素坐标与表面采样
//! - `adjacency`:基于 morphological closing 的可通行表面提取
//! - `nodes` + `edges`:节点放置与图构建
//! - `dijkstra`:最短路径搜索
//! - `planner`:组合上述步骤产出路径
//!
//! 端点安全规则已收紧：输入为支撑面坐标，必须在自身水平单元存在满足
//! 净空的表面，仅允许一个体素的高度量化差；不会借邻近表面跳过断带或
//! 把地图外目标拼入路径。缓存续行同样检查当前支撑面，并按三维位置恢复。
//!
//! 公开 API:`Config` / `Planner` / `RegionBounds` / `edges_to_segments` /
//! `voxel::VoxelKey` / `voxel::voxelize` / `voxel::surface_point_xyz`。

#![deny(unsafe_code)]
#![warn(missing_docs)]

// 内部模块:仅在本 crate 内可见,集成测试通过 `crate::xxx` 引用
pub mod adjacency;
pub mod debug_events;
pub mod dijkstra;
pub mod edges;
pub mod evaluator;
pub mod goal_relay;
pub mod lcm_worker;
pub mod mesh_fixture;
pub mod mls_planner;
pub mod nodes;
pub mod planner;
pub mod scenarios;
pub mod stamp_pair;
pub mod surface;
pub mod surfaces;
pub mod transformer;
pub mod voxel;

pub use surface::{
    SurfaceObservation, SurfaceObservations, SurfaceBodyEnvelope, SurfaceBodyQueryView, SurfaceBuildKind, SurfaceBuildStats, SurfaceColumnUpdate, SurfaceError, SurfaceFrontier, SurfaceFrontierCacheStats,
    SurfaceLocation, SurfaceMapBuild, SurfaceMapBuilder, SurfaceSnapshot, SurfaceTransition,
};
