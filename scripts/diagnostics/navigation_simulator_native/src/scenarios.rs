//! 可复现的跨层点云场景；所有表面显式采样到源体素中心。
use serde_json::{json, Value};
use std::collections::BTreeSet;

pub struct Scenario {
    pub id: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub points: Vec<(f32, f32, f32)>,
    pub cases: Vec<Value>,
    pub occupied_volume: bool,
}

#[derive(Default)]
struct Cloud(BTreeSet<[i32; 3]>);
impl Cloud {
    fn sheet(&mut self, bounds: [f64; 4], height: impl Fn(f64, f64) -> f64, solid: bool) {
        let q = f64::from(0.05_f32);
        let [x0, x1, y0, y1] = bounds;
        for x in (x0 / q).round() as i32..(x1 / q).round() as i32 {
            for y in (y0 / q).round() as i32..(y1 / q).round() as i32 {
                let top = (height((f64::from(x) + 0.5) * q, (f64::from(y) + 0.5) * q) / q).round()
                    as i32
                    - 1;
                let bottom = if solid { (-1).min(top) } else { top };
                for z in bottom..=top {
                    self.0.insert([x, y, z]);
                }
            }
        }
    }
    fn points(self) -> Vec<(f32, f32, f32)> {
        let q = f64::from(0.05_f32);
        self.0
            .into_iter()
            .map(|p| {
                let p = p.map(|v| ((f64::from(v) + 0.5) * q) as f32);
                (p[0], p[1], p[2])
            })
            .collect()
    }
}

fn case(id: &str, name: &str, start: [f64; 4], goal: [f64; 4], reachable: bool) -> Value {
    json!({"id":id,"name":name,"start":start,"goal":goal,"expected_reachable":reachable})
}

pub fn multilevel() -> Vec<Scenario> {
    let mut result = Vec::new();
    let mut ramp = Cloud::default();
    for z in [0.0, 2.0] {
        ramp.sheet([-2.0, 2.0, -2.0, 2.0], |_, _| z, false);
    }
    ramp.sheet([2.0, 10.0, -0.9, 0.9], |x, _| (x - 2.0) * 0.25, false);
    ramp.sheet([10.0, 12.0, -0.9, 2.3], |_, _| 2.0, false);
    ramp.sheet([2.0, 10.0, 1.0, 2.3], |_, _| 2.0, false);
    result.push(Scenario {
        occupied_volume: false,
        id: "stacked-ramp",
        name: "双层重叠 · 外侧坡道",
        description:
            "上下层相距2米、起终点XY重叠；须沿14度外侧坡道和上层回廊换层，不能直接跳楼层。",
        points: ramp.points(),
        cases: vec![
            case(
                "ramp-up",
                "下层→上层 · 同XY绕坡道",
                [-1.0, 0.0, 0.4, 0.0],
                [-1.0, 0.0, 2.4, 0.0],
                true,
            ),
            case(
                "ramp-down",
                "上层→下层 · 同XY绕坡道",
                [-1.0, 0.0, 2.4, 0.0],
                [-1.0, 0.0, 0.4, 0.0],
                true,
            ),
            case(
                "ramp-lower",
                "同层直行 · 不误上楼",
                [-1.0, -1.0, 0.4, 0.0],
                [1.0, -1.0, 0.4, 0.0],
                true,
            ),
        ],
    });

    let mut surfaces = Cloud::default();
    surfaces.sheet([-2.0, 0.0, -1.2, 1.2], |_, _| 0.0, false);
    surfaces.sheet(
        [0.0, 10.0, -1.2, 1.2],
        |x, _| ((x / 0.5).floor() + 1.0).min(20.0) * 0.1,
        false,
    );
    surfaces.sheet([10.0, 13.0, -1.2, 1.2], |_, _| 2.0, false);
    for step in 0..20 {
        let x = step * 10;
        for y in -24..24 {
            for z in step * 2..step * 2 + 2 {
                surfaces.0.insert([x, y, z]);
            }
        }
    }
    result.push(Scenario{occupied_volume:false,id:"stairs",name:"双层楼梯 · 表面点云",
        description:"20级楼梯连接2米高差，阶高0.10米、踏面0.50米；仅采样踏面与立面，按点云地形更新机体姿态。",
        points:surfaces.points(),cases:vec![
            case("stairs-up","表面楼梯上行",[-1.0,0.0,0.4,0.0],[12.0,0.0,2.4,0.0],true),
            case("stairs-down","表面楼梯下行",[12.0,0.0,2.4,std::f64::consts::PI],[-1.0,0.0,0.4,0.0],true),
        ]});
    let mut stairs = Cloud::default();
    stairs.sheet([-2.0, 0.0, -1.2, 1.2], |_, _| 0.0, false);
    stairs.sheet(
        [0.0, 10.0, -1.2, 1.2],
        |x, _| ((x / 0.5).floor() + 1.0).min(20.0) * 0.1,
        true,
    );
    stairs.sheet([10.0, 13.0, -1.2, 1.2], |_, _| 2.0, false);
    result.push(Scenario{occupied_volume:true,id:"stairs-solid",name:"双层楼梯 · 实心占据",
        description:"与表面楼梯相同尺寸，但体素填充实心阶体；检验占据带厚度是否被错误解释为地面测量不确定性。",
        points:stairs.points(),cases:vec![
            case("solid-stairs-up","实心楼梯上行",[-1.0,0.0,0.4,0.0],[12.0,0.0,2.4,0.0],true),
            case("solid-stairs-down","实心楼梯下行",[12.0,0.0,2.4,std::f64::consts::PI],[-1.0,0.0,0.4,0.0],true),
        ]});

    let mut disconnected = Cloud::default();
    for z in [0.0, 2.0] {
        disconnected.sheet([-2.0, 2.0, -2.0, 2.0], |_, _| z, false);
    }
    result.push(Scenario {
        occupied_volume: false,
        id: "disconnected-floors",
        name: "双层断开 · 禁止跳层",
        description: "上下层XY完全重叠但没有楼梯或坡道；跨层目标应报告无可连接路线，同层仍可通行。",
        points: disconnected.points(),
        cases: vec![
            case(
                "disconnected-up",
                "不可达 · 无连接上楼",
                [-1.0, 0.0, 0.4, 0.0],
                [1.0, 0.0, 2.4, 0.0],
                false,
            ),
            case(
                "disconnected-down",
                "不可达 · 无连接下楼",
                [1.0, 0.0, 2.4, std::f64::consts::PI],
                [-1.0, 0.0, 0.4, 0.0],
                false,
            ),
            case(
                "upper-straight",
                "上层内部直行",
                [-1.0, 0.0, 2.4, 0.0],
                [1.0, 0.0, 2.4, 0.0],
                true,
            ),
        ],
    });

    let mut high_step = Cloud::default();
    high_step.sheet([-2.0, 0.0, -1.2, 1.2], |_, _| 0.0, false);
    high_step.sheet([0.0, 2.0, -1.2, 1.2], |_, _| 0.4, true);
    result.push(Scenario {
        occupied_volume: true,
        id: "excessive-step",
        name: "超能力台阶 · 40厘米",
        description: "0.40米立面高于当前0.15米上下阶能力，没有其他连接，双向均应拒绝。",
        points: high_step.points(),
        cases: vec![
            case(
                "step-too-high",
                "不可达 · 超限上阶",
                [-1.0, 0.0, 0.4, 0.0],
                [1.0, 0.0, 0.8, 0.0],
                false,
            ),
            case(
                "drop-too-high",
                "不可达 · 超限下阶",
                [1.0, 0.0, 0.8, std::f64::consts::PI],
                [-1.0, 0.0, 0.4, 0.0],
                false,
            ),
        ],
    });

    let mut ceiling = Cloud::default();
    ceiling.sheet([-2.0, 2.0, -1.2, 1.2], |_, _| 0.0, false);
    ceiling.sheet([0.0, 2.0, -1.2, 1.2], |_, _| 0.30, false);
    result.push(Scenario {
        occupied_volume: false,
        id: "low-ceiling",
        name: "低净空 · 不可穿楼板",
        description: "目标位于30厘米高的楼板下方，低于原机体所需净空；不能从下层穿入或误选板顶。",
        points: ceiling.points(),
        cases: vec![case(
            "ceiling-blocked",
            "不可达 · 低楼板下方",
            [-1.0, 0.0, 0.4, 0.0],
            [1.0, 0.0, 0.4, 0.0],
            false,
        )],
    });
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn multilevel_fixtures_have_distinct_support_heights_and_expectations() {
        let scenes = multilevel();
        assert_eq!(scenes.iter().map(|s| s.cases.len()).sum::<usize>(), 13);
        let ramp = &scenes[0];
        let near_origin: Vec<_> = ramp
            .points
            .iter()
            .filter(|&&(x, y, _)| x.abs() < 0.1 && y.abs() < 0.1)
            .collect();
        assert!(near_origin.iter().any(|p| p.2 < 0.0));
        assert!(near_origin.iter().any(|p| p.2 > 1.9));
        assert!(ramp
            .points
            .iter()
            .any(|&(x, _, z)| x > 5.9 && x < 6.1 && z > 0.9 && z < 1.1));
        assert!(scenes
            .iter()
            .flat_map(|s| &s.cases)
            .any(|c| c["expected_reachable"] == false));
    }
}
