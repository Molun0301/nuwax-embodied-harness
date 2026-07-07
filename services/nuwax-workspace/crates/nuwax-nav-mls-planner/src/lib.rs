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
//! 原算法逻辑保持不变,只剥离了以下旧依赖:
//! - `dimos-module`(提供 `native_config` 派生宏与 LCM transport)
//! - `lcm-msgs`(LCM 消息类型)
//! - `pyo3` / `numpy`(Python 绑定)
//!
//! 算法层面与原版等价:
//! - `voxel::voxelize` / `surface_point_xyz`:体素坐标与表面采样
//! - `adjacency`:基于 morphological closing 的可通行表面提取
//! - `nodes` + `edges`:节点放置与图构建
//! - `dijkstra`:最短路径搜索
//! - `planner`:组合上述步骤产出路径
//!
//! 公开 API:`Config` / `Planner` / `RegionBounds` / `edges_to_segments` /
//! `voxel::VoxelKey` / `voxel::voxelize` / `voxel::surface_point_xyz`。

#![deny(unsafe_code)]
#![warn(missing_docs)]

// 内部模块:仅在本 crate 内可见,集成测试通过 `crate::xxx` 引用
pub mod adjacency;
pub mod dijkstra;
pub mod edges;
pub mod mls_planner;
pub mod nodes;
pub mod planner;
pub mod surfaces;
pub mod voxel;
