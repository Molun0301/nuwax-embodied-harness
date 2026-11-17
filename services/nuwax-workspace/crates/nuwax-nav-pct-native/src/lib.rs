//! 固定上游PCT源码的无ROS封装，仅从有效实测地面构建全局路线。
use std::cell::RefCell;
use std::ffi::{c_char, c_void, CStr};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

thread_local! {
    static CANCELLATIONS: RefCell<Vec<Arc<AtomicBool>>> = const { RefCell::new(Vec::new()) };
}
/// 在计算线程绑定现有拥有方的取消信号；嵌套调用和异常展开后恢复原作用域。
pub fn with_planning_cancellation<T>(flag: Arc<AtomicBool>, operation: impl FnOnce() -> T) -> T {
    struct Restore(usize);
    impl Drop for Restore {
        fn drop(&mut self) {
            CANCELLATIONS.with(|scopes| scopes.borrow_mut().truncate(self.0));
        }
    }
    let previous = CANCELLATIONS.with(|scopes| {
        let mut scopes = scopes.borrow_mut();
        let previous = scopes.len();
        scopes.push(flag);
        previous
    });
    let _restore = Restore(previous);
    operation()
}
/// 当前同步计算是否已被所属任务取消，不创建或改变地图状态。
pub fn planning_cancelled() -> bool {
    CANCELLATIONS.with(|scopes| {
        scopes
            .borrow()
            .iter()
            .any(|flag| flag.load(Ordering::Acquire))
    })
}

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    pub resolution: f64,
    pub point_resolution: f64,
    pub robot_height: f64,
    pub body_to_ground: f64,
    pub max_step: f64,
    pub max_step_down: f64,
    /// 最大连续坡度，单位为弧度；独立台阶由max_step约束。
    pub max_slope: f64,
    /// 无多边形外形时使用；已提供足迹时不使用外包圆作硬膨胀。
    pub footprint_radius: f64,
    pub body_z_min: f64,
    pub body_z_max: f64,
    pub search_margin: f64,
    pub footprint: Vec<[f64; 2]>,
    pub boundary: Vec<[f64; 2]>,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: 0.15,
            point_resolution: 0.05,
            robot_height: 0.5,
            body_to_ground: 0.4,
            max_step: 0.2,
            max_step_down: 0.2,
            max_slope: 30.0_f64.to_radians(),
            footprint_radius: 0.3,
            body_z_min: -0.2,
            body_z_max: 0.2,
            search_margin: 3.0,
            footprint: Vec::new(),
            boundary: Vec::new(),
        }
    }
}
/// XY格索引按Config.resolution，ground保存原实测高度；无顶面时ceiling为正无穷。
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cell {
    pub x: i32,
    pub y: i32,
    pub ground: f64,
    pub ceiling: f64,
}
/// 同一XY列允许多个占据区间，端点为米制真实区间。
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Band {
    pub x: i32,
    pub y: i32,
    pub low: f64,
    pub high: f64,
}
/// 完整路线与仅到已观测可达区域边缘的逼近路线。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RouteKind { Complete, Approach }
#[derive(Clone, Debug)]
pub struct Route {
    pub kind: RouteKind,
    /// 与起终点相同的机体中心坐标。
    pub path: Vec<[f64; 3]>,
    pub elapsed_ms: f64,
    pub expanded: usize,
}
/// 完整查询的真实路线长度、末点和末段朝向，不向探索复制整条路线。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RouteCost {
    pub cost: f64,
    pub endpoint: [f64; 3],
    pub yaw: f64,
    pub elapsed_ms: f64,
    pub expanded: usize,
}
/// 本轮真实转移检查及复用次数，以及单源搜索与完整查询的工作量。
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct MultiGoalStats {
    pub checks: usize,
    pub hits: usize,
    pub source_searches: usize,
    pub source_expanded: usize,
    pub exact_queries: usize,
    pub exact_searches: usize,
    pub exact_expanded: usize,
    pub source_elapsed_ms: f64,
    pub exact_elapsed_ms: f64,
}
#[repr(C)]
struct ConfigRaw {
    resolution: f64,
    point_resolution: f64,
    robot_height: f64,
    body_to_ground: f64,
    max_step: f64,
    max_step_down: f64,
    max_slope: f64,
    footprint_radius: f64,
    body_z_min: f64,
    body_z_max: f64,
    search_margin: f64,
    footprint: *const f64,
    footprint_len: usize,
    boundary: *const f64,
    boundary_len: usize,
}
#[repr(C)]
#[derive(Default)]
struct ResultRaw {
    points: *mut f64,
    length: usize,
    elapsed_ms: f64,
    expanded: usize,
    kind: i32,
}
unsafe extern "C" {
    fn pct_create(
        config: *const ConfigRaw,
        cells: *const Cell,
        ncells: usize,
        bands: *const Band,
        nbands: usize,
        points: *const f64,
        npoints: usize,
        err: *mut c_char,
        cap: usize,
    ) -> *mut c_void;
    fn pct_replace_columns(planner: *mut c_void, columns: *const [i32; 2], count: usize,
        cells: *const Cell, ncells: usize, bands: *const Band, nbands: usize);
    fn pct_replace_overlay(planner: *mut c_void, cells: *const Cell, ncells: usize,
        bands: *const Band, nbands: usize);
    fn pct_destroy(planner: *mut c_void);
    fn pct_geometry_sizes(planner: *const c_void, ncells: *mut usize, nbands: *mut usize);
    fn pct_geometry_copy(planner: *const c_void, cells: *mut Cell, bands: *mut Band);
    fn pct_result_free(result: *mut ResultRaw);
    fn pct_multi_goal_create(planner: *mut c_void, err: *mut c_char, cap: usize) -> *mut c_void;
    fn pct_multi_goal_destroy(search: *mut c_void);
    fn pct_multi_goal_stats(search: *const c_void) -> MultiGoalStats;
    fn pct_multi_goal_cost(
        search: *mut c_void, start: *const f64, goal: *const f64, cost: *mut RouteCost,
        cancel: extern "C" fn(*const c_void) -> bool, user: *const c_void,
        err: *mut c_char, cap: usize,
    ) -> i32;
    fn pct_source_begin(
        search: *mut c_void, start: *const f64, goals: *const f64, count: usize, reachable: *mut u8,
        err: *mut c_char, cap: usize,
    ) -> i32;
    fn pct_multi_goal_resolution(search: *const c_void) -> f64;
    fn pct_source_next(
        search: *mut c_void, index: *mut usize, weighted: *mut f64, out: *mut RouteCost, limit: f64,
        cancel: extern "C" fn(*const c_void) -> bool, user: *const c_void,
        err: *mut c_char, cap: usize,
    ) -> i32;
    fn pct_source_tree_distance(search: *const c_void, i: usize, j: usize, out: *mut f64,
        err: *mut c_char, cap: usize) -> i32;
    fn pct_multi_goal_grounds(search: *const c_void, x: f64, y: f64, out: *mut f64, cap: usize) -> usize;
    fn pct_plan(
        planner: *mut c_void,
        start: *const f64,
        goal: *const f64,
        result: *mut ResultRaw,
        cancel: extern "C" fn(*const c_void) -> bool,
        user: *const c_void,
        progressive: bool,
        err: *mut c_char,
        cap: usize,
    ) -> i32;
    #[cfg(test)]
    fn pct_retry_regression() -> bool;
}
/// 内核由实例唯一持有，所有搜索均通过可变借用串行执行。
pub struct Planner {
    native: NonNull<c_void>,
}
// 原生实例无共享全局可变状态，移动所有权不会产生并发访问。
unsafe impl Send for Planner {}
/// 一轮多目标评估共享惰性转移图；可变借用保证地图与机体配置不会在轮内变化。
/// 完整查询保留原目标窗口与启发式；排序使用按需展开的单源父链。
pub struct MultiGoalSearch<'a> {
    native: NonNull<c_void>,
    _planner: std::marker::PhantomData<&'a mut Planner>,
}
impl Drop for MultiGoalSearch<'_> {
    fn drop(&mut self) { unsafe { pct_multi_goal_destroy(self.native.as_ptr()) } }
}
/// 单源搜索中定值的一个已登记目标：绑定节点的加权路径代价，以及到端点的路线长度、端点与末段朝向。
/// 端点在绑定节点的实测地面上，绑定节点在目标所在格时平面位置取目标，否则取绑定格中心。
#[derive(Clone, Copy, Debug)]
pub struct SettledGoal {
    pub index: usize,
    pub weighted: f64,
    pub route: RouteCost,
}
impl MultiGoalSearch<'_> {
    /// 从起点开始一次按需展开的单源搜索并登记目标，替换上一次单源搜索。起点与目标都按完整查询的
    /// 端点绑定规则绑定到节点：所在格±10格内按三维距离（地面高度为`z`减机体到地面高度）最近的有效节点；
    /// 返回各目标是否有绑定节点，没有即不可达。绑定只读取附近列，不展开搜索。
    pub fn begin_source(&mut self, start: [f64; 3], goals: &[[f64; 3]]) -> Result<Vec<bool>, String> {
        let mut reachable = vec![0_u8; goals.len()];
        let mut error = [0; 512];
        let status = unsafe { pct_source_begin(self.native.as_ptr(), start.as_ptr(), goals.as_ptr().cast(),
            goals.len(), reachable.as_mut_ptr(), error.as_mut_ptr(), error.len()) };
        if status != 0 {
            return Err(native_error(&error));
        }
        Ok(reachable.into_iter().map(|value| value != 0).collect())
    }
    /// 继续当前单源搜索到下一个已登记目标的绑定节点定值；节点按加权路径代价从小到大定值，
    /// 绑定到同一节点的目标按登记顺序返回。可达范围已展开完，或下一个出队的加权代价超过`limit`
    /// （不出队，之后可用更大的上限继续）时返回`None`。
    pub fn next_settled(&mut self, limit: f64) -> Result<Option<SettledGoal>, String> {
        let mut index = 0;
        let mut weighted = 0.0;
        let mut route = RouteCost::default();
        let mut error = [0; 512];
        let status = unsafe { pct_source_next(self.native.as_ptr(), &mut index, &mut weighted, &mut route, limit,
            cancelled, std::ptr::null(), error.as_mut_ptr(), error.len()) };
        match status {
            0 => Ok(Some(SettledGoal { index, weighted, route })),
            1 => Ok(None),
            _ => Err(native_error(&error)),
        }
    }
    /// 当前单源搜索里两个已定值目标的绑定节点在搜索树上的距离（米）：两条父链长度之和减去两倍
    /// 最近公共祖先节点的父链长度。是两节点间真实最短路的上界；任一目标尚未定值时为`None`。
    pub fn tree_distance(&self, a: usize, b: usize) -> Result<Option<f64>, String> {
        let mut distance = 0.0;
        let mut error = [0; 512];
        let status = unsafe { pct_source_tree_distance(self.native.as_ptr(), a, b, &mut distance,
            error.as_mut_ptr(), error.len()) };
        match status {
            0 => Ok(Some(distance)),
            1 => Ok(None),
            _ => Err(native_error(&error)),
        }
    }
    /// 加权路径代价的基础每米代价（不含软代价）：相邻格的距离项按格数计，一格边长为原生分辨率米，
    /// 所以每米代价为 1/分辨率。
    pub fn cost_per_meter(&self) -> f64 {
        1.0 / unsafe { pct_multi_goal_resolution(self.native.as_ptr()) }
    }
    /// 平面点所在原生格的实测地面高度（静态列与本次临时叠加）；缺测列为空，不补造地面。
    pub fn measured_heights(&self, point: [f64; 2]) -> Vec<f64> {
        let count = unsafe { pct_multi_goal_grounds(self.native.as_ptr(), point[0], point[1],
            std::ptr::null_mut(), 0) };
        let mut heights = vec![0.0; count];
        unsafe { pct_multi_goal_grounds(self.native.as_ptr(), point[0], point[1], heights.as_mut_ptr(), count) };
        heights
    }
    pub fn cost(&mut self, start: [f64; 3], goal: [f64; 3]) -> Result<RouteCost, String> {
        self.cost_inner(start, goal, std::ptr::null())
    }
    pub fn cost_cancellable(&mut self, start: [f64; 3], goal: [f64; 3], cancel: &AtomicBool) -> Result<RouteCost, String> {
        self.cost_inner(start, goal, (cancel as *const AtomicBool).cast())
    }
    fn cost_inner(&mut self, start: [f64; 3], goal: [f64; 3], cancel: *const c_void) -> Result<RouteCost, String> {
        let mut result = RouteCost::default();
        let mut error = [0; 512];
        let status = unsafe { pct_multi_goal_cost(self.native.as_ptr(), start.as_ptr(), goal.as_ptr(),
            &mut result, cancelled, cancel, error.as_mut_ptr(), error.len()) };
        if status == 0 { Ok(result) } else { Err(native_error(&error)) }
    }
    pub fn stats(&self) -> MultiGoalStats {
        unsafe { pct_multi_goal_stats(self.native.as_ptr()) }
    }
}
impl Drop for Planner {
    fn drop(&mut self) {
        unsafe { pct_destroy(self.native.as_ptr()) }
    }
}
fn native_error(error: &[c_char]) -> String {
    unsafe { CStr::from_ptr(error.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}
extern "C" fn cancelled(user: *const c_void) -> bool {
    if user.is_null() {
        planning_cancelled()
    } else {
        unsafe { &*user.cast::<AtomicBool>() }.load(Ordering::Acquire)
    }
}
impl Planner {
    /// 创建只属于本轮的多目标评估；释放后全部共享转移随之回收。
    pub fn multi_goal_search(&mut self) -> Result<MultiGoalSearch<'_>, String> {
        let mut error = [0; 512];
        let native = unsafe { pct_multi_goal_create(self.native.as_ptr(), error.as_mut_ptr(), error.len()) };
        NonNull::new(native).map(|native| MultiGoalSearch { native, _planner: std::marker::PhantomData })
            .ok_or_else(|| native_error(&error))
    }
    pub fn new(config: Config, points: &[[f64; 3]]) -> Result<Self, String> {
        Self::create(config, &[], &[], points)
    }
    pub fn from_cells(config: Config, cells: &[Cell], bands: &[Band]) -> Result<Self, String> {
        Self::create(config, cells, bands, &[])
    }
    fn create(
        config: Config,
        cells: &[Cell],
        bands: &[Band],
        points: &[[f64; 3]],
    ) -> Result<Self, String> {
        let raw = ConfigRaw {
            resolution: config.resolution,
            point_resolution: config.point_resolution,
            robot_height: config.robot_height,
            body_to_ground: config.body_to_ground,
            max_step: config.max_step,
            max_step_down: config.max_step_down,
            max_slope: config.max_slope,
            footprint_radius: config.footprint_radius,
            body_z_min: config.body_z_min,
            body_z_max: config.body_z_max,
            search_margin: config.search_margin,
            footprint: config.footprint.as_ptr().cast(),
            footprint_len: config.footprint.len(),
            boundary: config.boundary.as_ptr().cast(),
            boundary_len: config.boundary.len(),
        };
        let mut error = [0; 512];
        // 创建期间C++复制全部切片；返回后不保留任何Rust借用。
        let native = unsafe {
            pct_create(
                &raw,
                cells.as_ptr(),
                cells.len(),
                bands.as_ptr(),
                bands.len(),
                points.as_ptr().cast(),
                points.len(),
                error.as_mut_ptr(),
                error.len(),
            )
        };
        NonNull::new(native)
            .map(|native| Self { native })
            .ok_or_else(|| native_error(&error))
    }
    /// 替换已变化的静态列；空列同时撤回旧地面与占据。
    pub fn replace_columns(&mut self, columns: &[[i32; 2]], cells: &[Cell], bands: &[Band]) {
        unsafe { pct_replace_columns(self.native.as_ptr(), columns.as_ptr(), columns.len(),
            cells.as_ptr(), cells.len(), bands.as_ptr(), bands.len()) }
    }
    /// 近期地面与禁行、近期占据按规划输入整体替换，静态列继续保留。
    pub fn replace_overlay(&mut self, cells: &[Cell], bands: &[Band]) {
        unsafe { pct_replace_overlay(self.native.as_ptr(), cells.as_ptr(), cells.len(),
            bands.as_ptr(), bands.len()) }
    }
    /// 导出相同原生实测分类结果供局部规划复用。
    pub fn geometry(&self) -> (Vec<Cell>, Vec<Band>) {
        let (mut ncells, mut nbands) = (0, 0);
        unsafe { pct_geometry_sizes(self.native.as_ptr(), &mut ncells, &mut nbands) };
        let mut cells = Vec::with_capacity(ncells);
        let mut bands = Vec::with_capacity(nbands);
        unsafe {
            pct_geometry_copy(self.native.as_ptr(), cells.as_mut_ptr(), bands.as_mut_ptr());
            cells.set_len(ncells);
            bands.set_len(nbands);
        }
        (cells, bands)
    }
    pub fn plan(&mut self, start: [f64; 3], goal: [f64; 3]) -> Result<Route, String> {
        self.plan_inner(start, goal, std::ptr::null(), false)
    }
    pub fn plan_cancellable(
        &mut self,
        start: [f64; 3],
        goal: [f64; 3],
        cancel: &AtomicBool,
    ) -> Result<Route, String> {
        self.plan_inner(start, goal, (cancel as *const AtomicBool).cast(), false)
    }
    /// 完整路线优先；失败时返回本次搜索已展开区域内三维距离最近的可达格。
    pub fn plan_progressive(&mut self, start: [f64; 3], goal: [f64; 3], cancel: Option<&AtomicBool>) -> Result<Route, String> {
        self.plan_inner(start, goal, cancel.map_or(std::ptr::null(), |flag| (flag as *const AtomicBool).cast()), true)
    }
    fn plan_inner(
        &mut self,
        start: [f64; 3],
        goal: [f64; 3],
        cancel: *const c_void,
        progressive: bool,
    ) -> Result<Route, String> {
        let mut result = ResultRaw::default();
        let mut error = [0; 512];
        // 调用同步完成，取消指针在整个原生搜索期间有效。
        let status = unsafe {
            pct_plan(
                self.native.as_ptr(),
                start.as_ptr(),
                goal.as_ptr(),
                &mut result,
                cancelled,
                cancel,
                progressive,
                error.as_mut_ptr(),
                error.len(),
            )
        };
        let route = if status == 0 && result.length > 0 {
            let values = unsafe { std::slice::from_raw_parts(result.points, result.length * 3) };
            Ok(Route {
                kind: if result.kind == 0 { RouteKind::Complete } else { RouteKind::Approach },
                path: values.chunks_exact(3).map(|v| [v[0], v[1], v[2]]).collect(),
                elapsed_ms: result.elapsed_ms,
                expanded: result.expanded,
            })
        } else {
            Err(native_error(&error))
        };
        unsafe { pct_result_free(&mut result) };
        route
    }
}

/// 原始点分类与Planner::new共用C++实现，保留实际测量高度与占据区间。
pub fn classify_points(
    config: &Config,
    points: &[[f64; 3]],
) -> Result<(Vec<Cell>, Vec<Band>), String> {
    Ok(Planner::new(config.clone(), points)?.geometry())
}

#[cfg(test)]
mod tests {
    include!("clearance_tests.rs");
    use super::*;
    fn config() -> Config {
        Config {
            resolution: 0.1,
            point_resolution: 0.03,
            footprint_radius: 0.12,
            max_step: 0.16,
            robot_height: 0.45,
            body_to_ground: 0.4,
            body_z_min: -0.2,
            body_z_max: 0.15,
            search_margin: 1.0,
            ..Default::default()
        }
    }
    fn flat_cells(x: std::ops::Range<i32>, y: std::ops::Range<i32>) -> Vec<Cell> {
        x.flat_map(|x| y.clone().map(move |y| Cell {
            x, y, ground: 0.0, ceiling: f64::INFINITY,
        })).collect()
    }
    #[test]
    fn progressive_gap_returns_nearest_reachable_cell_without_unknown_ground() {
        let cells: Vec<_> = flat_cells(0..40,-4..5).into_iter()
            .filter(|c|c.x<15 || c.x>=25).collect();
        let mut p=Planner::from_cells(config(),&cells,&[]).unwrap();
        let start=[0.35,0.05,0.4];let goal=[3.65,0.05,0.4];
        assert!(p.plan(start,goal).is_err(),"完整接口不能交付逼近结果");
        let route=p.plan_progressive(start,goal,None).unwrap();
        assert_eq!(route.kind,RouteKind::Approach);
        let end=route.path.last().unwrap();
        assert!((end[0]-1.45).abs()<1e-9 && (end[1]-0.05).abs()<1e-9 && (end[2]-0.4).abs()<1e-9);
        assert!(route.path.iter().all(|point|cells.iter().any(|cell|
            cell.x==(point[0]/0.1).floor() as i32 && cell.y==(point[1]/0.1).floor() as i32)));
        assert!(route.expanded>0);
    }

    #[test]
    fn progressive_missing_goal_still_searches_measured_reachable_region() {
        let cells=flat_cells(0..20,-4..5);
        let mut p=Planner::from_cells(config(),&cells,&[]).unwrap();
        assert!(p.plan([0.35,0.05,0.4],[8.05,0.05,0.4]).is_err());
        let route=p.plan_progressive([0.35,0.05,0.4],[8.05,0.05,0.4],None).unwrap();
        assert_eq!(route.kind,RouteKind::Approach);
        assert!((route.path.last().unwrap()[0]-1.95).abs()<1e-9);
        assert_eq!(p.geometry().0.len(),cells.len(),"缺测目标不能创建地面节点");
    }

    #[test]
    fn progressive_complete_path_preserves_original_search_and_cancellation() {
        let cells=flat_cells(0..40,-4..5);
        let mut p=Planner::from_cells(config(),&cells,&[]).unwrap();
        let start=[0.35,0.05,0.4];let goal=[3.65,0.05,0.4];
        let original=p.plan(start,goal).unwrap();
        let route=p.plan_progressive(start,goal,None).unwrap();
        assert_eq!(route.kind,RouteKind::Complete);
        assert_eq!(route.path,original.path);
        assert_eq!(route.expanded,original.expanded,"不得额外执行全图搜索");
        let cancelled=AtomicBool::new(true);
        assert!(p.plan_progressive(start,goal,Some(&cancelled)).unwrap_err().contains("取消"));
        assert_eq!(p.plan_progressive(start,goal,None).unwrap().path,original.path);
    }

    #[test]
    fn progressive_candidate_keeps_full_body_clear_of_a_wall() {
        let cells=flat_cells(0..40,-4..5);
        let wall:Vec<_>=(-6..7).map(|y|Band{x:15,y,low:0.0,high:1.5}).collect();
        let mut p=Planner::from_cells(config(),&cells,&wall).unwrap();
        let route=p.plan_progressive([0.35,0.05,0.4],[3.65,0.05,0.4],None).unwrap();
        assert_eq!(route.kind,RouteKind::Approach);
        assert!(route.path.iter().all(|p|p[0]+config().footprint_radius<1.5));
    }

    #[test]
    fn progressive_closest_distance_includes_height_instead_of_only_xy() {
        let mut cfg=config();cfg.max_slope=80_f64.to_radians();
        let cells:Vec<_>=flat_cells(0..35,0..1).into_iter().map(|mut cell| {
            cell.ground=f64::from((cell.x-9).max(0))*0.1;cell
        }).collect();
        let goal=[4.0,0.05,0.4];
        let distance=|cell:&Cell|((f64::from(cell.x)+0.5)*0.1-goal[0]).powi(2)+(cell.ground+cfg.body_to_ground-goal[2]).powi(2);
        let closest=cells.iter().min_by(|a,b|distance(a).total_cmp(&distance(b))).unwrap();
        let mut planner=Planner::from_cells(cfg,&cells,&[]).unwrap();
        let route=planner.plan_progressive([0.35,0.05,0.4],goal,None).unwrap();
        assert_eq!(route.kind,RouteKind::Approach);
        let endpoint=route.path.last().unwrap();
        assert!((endpoint[0]-(f64::from(closest.x)+0.5)*0.1).abs()<1e-9);
        assert!(endpoint[0]<3.0,"只按水平距离会错误选择最高的远端格");
    }

    #[test]
    fn entirely_unknown_space_has_no_native_surface_route() {
        let mut p = Planner::new(config(), &[]).unwrap();
        assert!(p.plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4]).is_err());
    }
    #[test]
    fn missing_ground_strip_does_not_connect_known_platforms() {
        let cells: Vec<_> = flat_cells(0..40,-4..5).into_iter()
            .filter(|c| c.x<15 || c.x>=25).collect();
        let mut p = Planner::from_cells(config(), &cells, &[]).unwrap();
        assert!(p.plan([0.35, 0.05, 0.4], [3.65, 0.05, 0.4]).is_err());
        assert!(p.plan([0.35, 0.05, 0.4], [1.05, 0.05, 0.4]).is_ok());
    }
    #[test]
    fn missing_endpoint_binds_to_an_existing_surface_without_filling_the_map() {
        let cells: Vec<_> = flat_cells(0..40,-4..5).into_iter()
            .filter(|c| [c.x,c.y]!=[0,0]).collect();
        let mut p = Planner::from_cells(config(), &cells, &[]).unwrap();
        let route = p.plan([0.05,0.05,0.4],[3.65,0.05,0.4]).unwrap();
        assert!(route.path.iter().all(|point| {
            let x=(point[0]/0.1).floor() as i32;
            let y=(point[1]/0.1).floor() as i32;
            cells.iter().any(|cell|cell.x==x&&cell.y==y)
        }));
        assert_eq!(p.geometry().0.len(),cells.len());
        assert!(p.plan([0.05,0.05,0.4],[8.0,0.05,0.4]).is_err());
    }
    #[test]
    fn upstream_clearance_cost_prefers_the_interior_without_closing_a_corridor() {
        let cells=flat_cells(0..60,-10..10);
        let mut p=Planner::from_cells(config(),&cells,&[]).unwrap();
        let route=p.plan([0.35,-0.85,0.4],[5.65,-0.85,0.4]).unwrap();
        assert!(route.path.iter().any(|p|p[0]>1.5&&p[0]<4.5&&p[1]>-0.75));
        let mut narrow=Planner::from_cells(config(),&flat_cells(0..60,-2..2),&[]).unwrap();
        assert!(narrow.plan([0.35,0.05,0.4],[5.65,0.05,0.4]).is_ok());
    }
    #[test]
    fn observed_wall_is_avoided_on_measured_ground() {
        let bands: Vec<_> = (-6..=6)
            .map(|y| Band {
                x: 15,
                y,
                low: 0.0,
                high: 1.5,
            })
            .collect();
        let cells = flat_cells(-5..36,-15..16);
        let mut p = Planner::from_cells(config(), &cells, &bands).unwrap();
        assert!(p.plan([1.55, 0.05, 0.4], [1.56, 0.06, 0.4]).is_err());
        let r = p.plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4]).unwrap();
        assert!(r.path.iter().any(|p| p[1].abs() > 0.7));
        assert!(r
            .path
            .iter()
            .all(|p| !(p[0] > 1.35 && p[0] < 1.75 && p[1].abs() < 0.7)));
        let mut cfg = config();
        cfg.footprint_radius = 0.0;
        let route = Planner::from_cells(cfg, &cells, &bands)
            .unwrap()
            .plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4])
            .unwrap();
        assert!(route.path.iter().any(|p| p[1] < -0.6 || p[1] > 0.7));
    }
    #[test]
    fn noisy_measurements_do_not_become_voxel_tops() {
        let mut points = Vec::new();
        for x in 0..30 {
            for y in -2..=2 {
                for z in [-0.009, 0.002, 0.007] {
                    points.push([(f64::from(x) + 0.5) * 0.1, (f64::from(y) + 0.5) * 0.1, z]);
                }
            }
        }
        let mut p = Planner::new(config(), &points).unwrap();
        let r = p.plan([0.05, 0.05, 0.4], [2.85, 0.05, 0.4]).unwrap();
        assert!(r.path.iter().all(|p| (p[2] - 0.402).abs() < 1e-6));
    }
    #[test]
    fn failed_search_reuses_the_same_upstream_object() {
        assert!(unsafe { pct_retry_regression() });
    }
    #[test]
    fn stairs_connect_measured_layers() {
        let cells: Vec<_> = (0..40)
            .flat_map(|x| {
                (-4..=4).map(move |y| Cell {
                    x,
                    y,
                    ground: f64::from(x / 5) * 0.12,
                    ceiling: f64::INFINITY,
                })
            })
            .collect();
        let mut cfg = config();
        cfg.boundary = vec![[0.0, -0.4], [4.0, -0.4], [4.0, 0.5], [0.0, 0.5]];
        let bands: Vec<_> = cells
            .iter()
            .map(|s| Band {
                x: s.x,
                y: s.y,
                low: -0.05,
                high: s.ground,
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
        let r = p.plan([0.25, 0.05, 0.4], [3.75, 0.05, 1.24]).unwrap();
        assert!(r.path.iter().any(|p| p[2] > 1.0));
        assert!(r
            .path
            .windows(2)
            .all(|p| (p[1][2] - p[0][2]).abs() <= 0.160001));
    }
    #[test]
    fn true_footprint_passes_narrow_corridor() {
        let mut cfg = config();
        cfg.footprint = vec![[-0.4, -0.12], [0.4, -0.12], [0.4, 0.12], [-0.4, 0.12]];
        cfg.footprint_radius = 0.42;
        let bands: Vec<_> = (0..40)
            .flat_map(|x| {
                [-3, 2].map(move |y| Band {
                    x,
                    y,
                    low: 0.0,
                    high: 2.0,
                })
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &flat_cells(0..40,-2..2), &bands).unwrap();
        let r = p.plan([0.55, 0.05, 0.4], [3.45, 0.05, 0.4]).unwrap();
        assert!(r.path.iter().all(|p| p[1].abs() < 0.1));
    }
    #[test]
    fn cancellation_reaches_native_search() {
        let mut p = Planner::from_cells(config(), &flat_cells(-5..56,-5..6), &[]).unwrap();
        let cancel = AtomicBool::new(true);
        assert!(p
            .plan_cancellable([0.0, 0.0, 0.4], [5.0, 0.0, 0.4], &cancel)
            .unwrap_err()
            .contains("取消"));
        cancel.store(false, Ordering::Relaxed);
        assert!(p
            .plan_cancellable([0.0, 0.0, 0.4], [5.0, 0.0, 0.4], &cancel)
            .is_ok());
    }
    #[test]
    fn descending_steps_use_the_separate_down_limit() {
        let mut cfg = config();
        cfg.max_step = 0.05;
        cfg.max_step_down = 0.25;
        cfg.boundary = vec![[0.0, -0.4], [3.0, -0.4], [3.0, 0.5], [0.0, 0.5]];
        let cells: Vec<_> = (0..30)
            .flat_map(|x| {
                (-4..=4).map(move |y| Cell {
                    x,
                    y,
                    ground: if x < 15 { 0.2 } else { 0.0 },
                    ceiling: f64::INFINITY,
                })
            })
            .collect();
        let bands: Vec<_> = cells
            .iter()
            .map(|c| Band {
                x: c.x,
                y: c.y,
                low: -0.05,
                high: c.ground,
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
        assert!(p.plan([0.35, 0.05, 0.6], [2.65, 0.05, 0.4]).is_ok());
        assert!(p.plan([2.65, 0.05, 0.4], [0.35, 0.05, 0.6]).is_err());
    }
    #[test]
    fn distinct_floors_do_not_teleport_between_layers() {
        let mut cfg = config();
        cfg.boundary = vec![[0.0, -0.4], [3.0, -0.4], [3.0, 0.5], [0.0, 0.5]];
        let cells: Vec<_> = (0..30)
            .flat_map(|x| {
                (-4..=4).flat_map(move |y| {
                    [
                        Cell {
                            x,
                            y,
                            ground: 0.0,
                            ceiling: 0.95,
                        },
                        Cell {
                            x,
                            y,
                            ground: 1.0,
                            ceiling: f64::INFINITY,
                        },
                    ]
                })
            })
            .collect();
        let bands: Vec<_> = (0..30)
            .flat_map(|x| {
                (-4..=4).map(move |y| Band {
                    x,
                    y,
                    low: 0.95,
                    high: 1.0,
                })
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
        assert!(p.plan([0.35, 0.05, 0.4], [2.65, 0.05, 1.4]).is_err());
        assert!(p.plan([0.35, 0.05, 1.4], [2.65, 0.05, 1.4]).is_ok());
    }
    #[test]
    fn shared_raw_classification_has_no_wall_interior_floors() {
        let points: Vec<_> = (0..=40)
            .map(|z| [1.05, 0.05, f64::from(z) * 0.025])
            .collect();
        let (cells, bands) = classify_points(&config(), &points).unwrap();
        assert_eq!(cells.len(), 1);
        assert_eq!(bands.len(), 1);
        assert!(cells[0].ground > 0.98);
        assert_eq!(bands[0].low, 0.0);
        assert_eq!(bands[0].high, 1.0);
    }
    #[test]
    fn measured_ground_noise_does_not_block_the_calibrated_foot() {
        let mut cfg = config();
        cfg.max_step = 0.0;
        cfg.body_z_min = -cfg.body_to_ground;
        let points: Vec<_> = (0..30)
            .flat_map(|x| {
                (-3..=3).flat_map(move |y| {
                    [-0.008, 0.002, 0.007]
                        .map(move |z| [(f64::from(x) + 0.5) * 0.1, (f64::from(y) + 0.5) * 0.1, z])
                })
            })
            .collect();
        let mut p = Planner::new(cfg, &points).unwrap();
        assert!(p.plan([0.35, 0.05, 0.402], [2.65, 0.05, 0.402]).is_ok());
    }
    #[test]
    fn owner_cancellation_scope_reaches_plan_and_restores() {
        let flag = Arc::new(AtomicBool::new(true));
        let mut p = Planner::from_cells(config(), &flat_cells(-5..16,-5..6), &[]).unwrap();
        with_planning_cancellation(flag, || {
            assert!(planning_cancelled());
            assert!(p
                .plan([0.0, 0.0, 0.4], [1.0, 0.0, 0.4])
                .unwrap_err()
                .contains("取消"));
            let explicit = AtomicBool::new(false);
            assert!(p
                .plan_cancellable([0.0, 0.0, 0.4], [1.0, 0.0, 0.4], &explicit)
                .is_ok());
        });
        assert!(!planning_cancelled());
        assert!(p.plan([0.0, 0.0, 0.4], [1.0, 0.0, 0.4]).is_ok());
    }
    #[test]
    fn bounded_map_search_expands_for_a_long_observed_wall() {
        let mut cfg = config();
        cfg.search_margin = 0.5;
        cfg.boundary = vec![[-0.5, -3.0], [3.5, -3.0], [3.5, 3.0], [-0.5, 3.0]];
        let bands: Vec<_> = (-20..=20)
            .map(|y| Band {
                x: 15,
                y,
                low: 0.0,
                high: 2.0,
            })
            .collect();
        let mut p = Planner::from_cells(cfg, &flat_cells(-5..36,-30..31), &bands).unwrap();
        let route = p.plan([0.05, 0.05, 0.4], [3.05, 0.05, 0.4]).unwrap();
        assert!(route.path.iter().any(|point| point[1].abs() > 2.1));
    }
    #[test]
    fn continuous_measured_slope_is_not_misclassified_as_small_steps() {
        for (gradient, allowed) in [(0.2, true), (1.0, false)] {
            let mut cfg = config();
            cfg.max_slope = 25.0_f64.to_radians();
            cfg.boundary = vec![[0.0, -0.2], [2.1, -0.2], [2.1, 0.3], [0.0, 0.3]];
            let cells: Vec<_> = (0..21)
                .flat_map(|x| {
                    (-2..=2).map(move |y| Cell {
                        x,
                        y,
                        ground: (f64::from(x) + 0.5) * 0.1 * gradient,
                        ceiling: f64::INFINITY,
                    })
                })
                .collect();
            let bands: Vec<_> = cells
                .iter()
                .map(|c| Band {
                    x: c.x,
                    y: c.y,
                    low: -0.1,
                    high: c.ground,
                })
                .collect();
            let mut p = Planner::from_cells(cfg, &cells, &bands).unwrap();
            assert_eq!(
                p.plan(
                    [0.25, 0.05, 0.4 + 0.25 * gradient],
                    [1.75, 0.05, 0.4 + 1.75 * gradient]
                )
                .is_ok(),
                allowed
            );
        }
    }
}

#[cfg(test)]
mod multi_goal_tests;
