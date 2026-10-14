# nuwax-nav-simple-planner

纯 Rust 轻量栅格 A* 全局路径规划器（Phase 3 迁移自 Python `simple_planner.py`）。

## 模块

| 模块 | 职责 |
|---|---|
| `costmap` | 栅格 costmap：cell ↔ world、history-max、脏标记 |
| `inflation` | 障碍膨胀纯函数 |
| `astar` | 8 邻接 octile heuristic A* |
| `stuck` | Stuck 升级与 progress tick |
| `waypoint` | 沿 cached path 的 lookahead |
| `pointcloud` | z-相对地面分类写入 costmap |
| `planner` | `plan_on_costmap` + `SimplePlanner` 状态机 |
| `config` | 默认参数 |
| `module` | `feature = "module"` 启用的 stream/RPC stub |

## 公开 API（核心）

```rust
pub struct Point2 { pub x: f64, pub y: f64 }
pub struct Point3 { pub x: f64, pub y: f64, pub z: f64 }
pub struct Costmap { /* ... */ }

pub fn plan_on_costmap(
    costmap: &Costmap,
    start: Point2,
    goal: Point2,
    max_expansions: Option<usize>,
    inflation_override: Option<f64>,
) -> Result<Option<Vec<Point2>>, NavError>;

pub struct SimplePlanner { /* ... */ }
impl SimplePlanner {
    pub fn new(config: SimplePlannerConfig) -> Result<Self, NavError>;
    pub fn replace_costmap(&mut self, costmap: Costmap);
    pub fn ingest_world_terrain(&mut self, points: &[Point3]) -> bool;
    pub fn ingest_local_terrain(&mut self, points: &[Point3]) -> bool;
    pub fn update_odometry(&mut self, pose: Pose3);
    pub fn set_goal(&mut self, goal: Point3, now: f64);
    pub fn cancel_navigation(&mut self);
    pub fn replan_once(&mut self, mono_now: f64) -> (Option<Option<Vec<Point2>>>, bool, bool);
    pub fn update_waypoint(&mut self) -> Option<PointStamped>;
}
```

## 测试

```bash
cargo test -p nuwax-nav-simple-planner
```

测试覆盖：
- `tests/costmap_tests.rs`：TestCostmap（cell roundtrip、history-max、inflation、clear、构造校验）
- `tests/astar_tests.rs`：TestAstar（同 cell、直线、对角、wall detour、不可达、max expansion、octile 偏好）
- `tests/plan_tests.rs`：TestSimplePlannerPlan（开放路径、绕障、不可达、inflation_override 开 doorway、lookahead 三种情况）
- `tests/blocked_inflation_tests.rs`：TestBlockedAtInflation（半径包含邻居、阈值过滤、不修改 costmap、负值拒绝）
- `tests/stuck_escalation_tests.rs`：TestStuckEscalation（progress refresh、tiny 不算进展、shrink、floor、progress 后保持 shrunk）

## Foundation 依赖

- `nuwax-nav-types-messages`：替换本 crate 自带的 `Point2/Point3/Pose3/Timestamp/Path/PointCloud/Header`。
- `nuwax-nav-base`：将提供 `NavError` 分层与 `GlobalPlanner` trait；本 crate 当前自带 `NavError`。

## 平台约束

- 无 Open3D / NumPy / SciPy 依赖。
- `edition = "2021"`，MSRV 1.88（与 workspace 根 `Cargo.toml` 一致）。
- 默认 features 为空；启用 `module` 才编译 `module.rs`。