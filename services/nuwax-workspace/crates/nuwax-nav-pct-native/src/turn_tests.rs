//! 拐点处原地转向的机身扫掠：节点处从上一条边方向转到下一条边方向时逐个朝向做机身碰撞判定。
use super::*;

const Q: f64 = 0.05;

fn config(length: f64, width: f64) -> Config {
    Config {
        resolution: Q,
        point_resolution: 0.03,
        robot_height: 0.45,
        body_to_ground: 0.4,
        max_step: 0.16,
        max_step_down: 0.16,
        body_z_min: -0.2,
        body_z_max: 0.15,
        search_margin: 1.0,
        footprint: vec![[-length / 2., -width / 2.], [length / 2., -width / 2.],
            [length / 2., width / 2.], [-length / 2., width / 2.]],
        ..Default::default()
    }
}

/// 全图实测地面；`free` 之外的格是从地面到 1.5 米的实体。
fn scene(free: impl Fn(f64, f64) -> bool) -> (Vec<Cell>, Vec<Band>) {
    let range = -80..80;
    let cells: Vec<_> = range.clone().flat_map(|x| range.clone().map(move |y| Cell {
        x, y, ground: 0., ceiling: f64::INFINITY,
    })).collect();
    let bands = cells.iter().filter(|c| !free((f64::from(c.x) + 0.5) * Q, (f64::from(c.y) + 0.5) * Q))
        .map(|c| Band { x: c.x, y: c.y, low: 0., high: 1.5 }).collect();
    (cells, bands)
}

/// 0.4 米宽的十字通道：水平臂 |y|≤0.2，竖直臂 |x|≤0.2。
fn cross(x: f64, y: f64) -> bool {
    y.abs() <= 0.2 || x.abs() <= 0.2
}

const LEFT: [f64; 3] = [-2.5, 0.025, 0.4];
const UP: [f64; 3] = [0.025, 2.5, 0.4];

#[test]
fn narrow_cross_turn_blocks_long_body_while_short_body_turns() {
    let (cells, bands) = scene(cross);
    // 1.0×0.3 米机身两端朝向都放得下十字中心，但 45 度朝向的长边扫过墙角。
    let mut long = Planner::from_cells(config(1.0, 0.3), &cells, &bands).unwrap();
    assert!(long.plan(LEFT, None, UP).unwrap_err().contains("PCT未找到"));
    // 0.25×0.25 米机身的对角线约0.354米，小于0.4米通道宽，可在中心转向。
    let mut short = Planner::from_cells(config(0.25, 0.25), &cells, &bands).unwrap();
    assert!(short.plan(LEFT, None, UP).is_ok());
    // 不转弯的直行路线不受影响。
    assert!(long.plan(LEFT, None, [2.5, 0.025, 0.4]).is_ok());
    // 单源搜索按同一转移判定：长机身到竖直臂不可达。
    let costs = long.multi_goal_search().unwrap().rank_costs(LEFT, &[UP, [2.5, 0.025, 0.4]]);
    assert!(costs[0].is_err(), "{:?}", costs[0]);
    assert!(costs[1].is_ok(), "{:?}", costs[1]);
}

#[test]
fn blocked_corner_turn_takes_a_detour_through_open_room() {
    // 十字左臂通到左侧房间，房间再经 1.2 米宽的通道接到上臂；房间与通道都容得下长机身转向。
    let free = |x: f64, y: f64| cross(x, y)
        || ((-4.0..=-2.5).contains(&x) && (-1.0..=2.8).contains(&y))
        || ((-4.0..=1.0).contains(&x) && (1.6..=2.8).contains(&y));
    let (cells, bands) = scene(free);
    let mut long = Planner::from_cells(config(1.0, 0.3), &cells, &bands).unwrap();
    let start = [-1.5, 0.025, 0.4];
    let goal = [0.025, 1.0, 0.4];
    let route = long.plan(start, None, goal).unwrap();
    assert!(route.path.iter().all(|p| p[0].abs() > 0.2 || p[1].abs() > 0.2), "路线不能经十字中心转向：{:?}", route.path);
    assert!(route.path.iter().any(|p| p[1] > 1.5), "路线须经房间绕行");
}

#[test]
fn start_heading_turn_is_checked_only_when_heading_is_given() {
    let (cells, bands) = scene(cross);
    let mut long = Planner::from_cells(config(1.0, 0.3), &cells, &bands).unwrap();
    let center = [0.025, 0.025, 0.4];
    // 朝上停在十字中心：转向左臂要扫过墙角。
    assert!(long.plan(center, Some(std::f64::consts::FRAC_PI_2), LEFT).is_err());
    // 拿不到当前朝向时不查起点；已朝左时不需转向。
    assert!(long.plan(center, None, LEFT).is_ok());
    assert!(long.plan(center, Some(std::f64::consts::PI), LEFT).is_ok());
}
